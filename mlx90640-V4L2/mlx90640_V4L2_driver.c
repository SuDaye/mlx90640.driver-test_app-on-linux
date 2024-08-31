#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/fcntl.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-vmalloc.h>
#include "include/MLX90640_API.h"
#include "include/mlx90640_ioctl.h"

#define NORMAL_WIDTH 32
#define NORMAL_HEIGHT 24
#define NORMAL_BUFFER_SIZE (NORMAL_WIDTH * NORMAL_HEIGHT)

static int debug = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "enable debug information\n");

#define dprintk(args...)        \
    if (debug)                  \
    {                           \
        printk(KERN_INFO args); \
    }

#define DATA_NO_READY -8

/* intermediate buffers with raw data from the device */
struct mlx90640_frame_buf
{
    struct vb2_buffer vb; /* common v4l buffer stuff -- must be first */
    struct list_head list;
};

struct mlx90640_device
{
    struct cdev cdev;
    struct video_device vdev;
    dev_t devid;         /* 设备号  */
    struct class *class; /* 类      */
    struct device *dev;  /* 设备    */
    int major;           /* 主设备号 */

    struct v4l2_device v4l2_dev;

    struct i2c_client *client;

    struct list_head queued_bufs;

    u32 buffersize;
    u32 pixelformat;
    u32 width;
    u32 height;

    uint16_t *eeMLX90640;
    uint16_t *mlx90640Frame;
    struct mlx90640_frame_buf *buf_list;

    int refreshRate; /* refresh rate  if rateMessage is 0x04, refreshRate needs to be 8 */
    /* set by function MLX90640_SetRefreshRate */
    int refreshtime_ms; /* refresh time */
    u8 rateMessage;     /* send this menber through i2c to set the refresh rate */

    struct mutex vb_queue_lock;  /* Protects vb_queue and capt_file */
    spinlock_t queued_bufs_lock; /* Protects queued_bufs */

    struct task_struct *thread;
    bool streaming;
    struct vb2_queue mlx90640_vb_queue;
};

/* stream formats */
struct virtual_format
{
    char *name;
    u32 pixelformat;
    u32 buffersize;
    u32 width;
    u32 height;
};

/* format descriptions for capture and preview */
static struct virtual_format formats[] = {
    {
        .name = "MLX90640 GRAY",
        .pixelformat = V4L2_PIX_FMT_GREY,
        .buffersize = NORMAL_BUFFER_SIZE,
        .width = NORMAL_WIDTH,
        .height = NORMAL_HEIGHT,
    },
};

static const unsigned int NUM_FORMATS = ARRAY_SIZE(formats);

static int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead,
                            uint16_t *data, struct i2c_client *client)
{
    int ret;
    int i;
    uint8_t buf[2];
    struct i2c_msg msgs[2] = {
        {.addr = slaveAddr,
         .flags = 0,
         .len = 2,
         .buf = buf},
        {.addr = slaveAddr,
         .flags = I2C_M_RD,
         .len = nMemAddressRead * 2,
         .buf = (uint8_t *)data}}; // 直接将 data 用作缓冲区

    buf[0] = startAddress >> 8;
    buf[1] = startAddress & 0xFF;

    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret < 0)
    {
        pr_err("I2C read failed\n");
        return ret;
    }

    // 如果需要转换字节序，可以在原地进行
    for (i = 0; i < nMemAddressRead; i++)
    {
        data[i] = be16_to_cpu(data[i]); // 如果目标系统是小端序，这个转换是必要的
    }

    return 0;
}

static int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress,
                             uint16_t data, struct i2c_client *client)
{
    uint8_t buf[4];
    struct i2c_msg msg = {
        .addr = slaveAddr,
        .flags = 0,
        .len = 4,
        .buf = buf};

    buf[0] = writeAddress >> 8;
    buf[1] = writeAddress & 0xFF;
    buf[2] = data >> 8;
    buf[3] = data & 0xFF;

    if (i2c_transfer(client->adapter, &msg, 1) < 0)
    {
        pr_err("I2C write failed\n");
        return -EIO;
    }
    return 0;
}

static int ValidateAuxData(uint16_t *auxData)
{
    int i;
    if (auxData[0] == 0x7FFF)
        return -MLX90640_FRAME_DATA_ERROR;
    for (i = 8; i < 19; i++)
    {
        if (auxData[i] == 0x7FFF)
            return -MLX90640_FRAME_DATA_ERROR;
    }
    for (i = 20; i < 23; i++)
    {
        if (auxData[i] == 0x7FFF)
            return -MLX90640_FRAME_DATA_ERROR;
    }
    for (i = 24; i < 33; i++)
    {
        if (auxData[i] == 0x7FFF)
            return -MLX90640_FRAME_DATA_ERROR;
    }
    for (i = 40; i < 51; i++)
    {
        if (auxData[i] == 0x7FFF)
            return -MLX90640_FRAME_DATA_ERROR;
    }
    for (i = 52; i < 55; i++)
    {
        if (auxData[i] == 0x7FFF)
            return -MLX90640_FRAME_DATA_ERROR;
    }
    for (i = 56; i < 64; i++)
    {
        if (auxData[i] == 0x7FFF)
            return -MLX90640_FRAME_DATA_ERROR;
    }
    return MLX90640_NO_ERROR;
}

static int ValidateFrameData(uint16_t *frameData)
{
    uint8_t line = 0;
    int i;
    for (i = 0; i < MLX90640_PIXEL_NUM; i += MLX90640_LINE_SIZE)
    {
        if ((frameData[i] == 0x7FFF) && (line % 2 == frameData[833]))
            return -MLX90640_FRAME_DATA_ERROR;
        line = line + 1;
    }
    return MLX90640_NO_ERROR;
}

static int MLX90640_DumpEE(struct i2c_client *client, uint16_t *eeData)
{
    return MLX90640_I2CRead(client->addr, MLX90640_EEPROM_START_ADDRESS, MLX90640_EEPROM_DUMP_NUM, eeData, client);
}

static int MLX90640_GetFrameData(struct i2c_client *client, uint16_t *frameData)
{
    // 定义变量：dataReady用于检查数据是否准备好，controlRegister1存储控制寄存器的值，
    // statusRegister存储状态寄存器的值，error用于错误处理，data用于临时存储辅助数据，cnt是计数器
    uint16_t controlRegister1;
    uint16_t statusRegister;
    int error = 1;
    uint16_t envdata[64];
    uint8_t cnt = 0;
    uint8_t slaveAddr = client->addr;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_STATUS_REG, 1, &statusRegister, client);
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    if (MLX90640_GET_DATA_READY(statusRegister) == 0)
        return DATA_NO_READY;
    // 一旦数据准备好，重置状态寄存器
    error = MLX90640_I2CWrite(slaveAddr, MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE, client);
    if (error == -MLX90640_I2C_NACK_ERROR)
    {
        return error;
    }

    // 读取传感器像素数据并存储到 frameData 数组
    error = MLX90640_I2CRead(slaveAddr, MLX90640_PIXEL_DATA_START_ADDRESS, MLX90640_PIXEL_NUM, frameData, client);
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    // 读取辅助数据（如环境温度等）
    error = MLX90640_I2CRead(slaveAddr, MLX90640_AUX_DATA_START_ADDRESS, MLX90640_AUX_NUM, envdata, client);
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    // 读取控制寄存器的值，并将其存储到 frameData 数组的最后一个位置（索引 832）
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1, client);
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    frameData[832] = controlRegister1;
    // 使用宏 MLX90640_GET_FRAME 从状态寄存器中提取帧信息，并存储到 frameData 数组的索引 833
    frameData[833] = MLX90640_GET_FRAME(statusRegister);

    // 验证辅助数据的有效性
    error = ValidateAuxData(envdata);
    if (error == MLX90640_NO_ERROR)
    {
        // 如果验证成功，将辅助数据拷贝到 frameData 数组的指定位置（768-831）
        for (cnt = 0; cnt < MLX90640_AUX_NUM; cnt++)
        {
            frameData[cnt + MLX90640_PIXEL_NUM] = envdata[cnt];
        }
    }

    // 验证帧数据的有效性
    error = ValidateFrameData(frameData);
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    // 返回帧编号，作为该帧数据的标识符，即子页1还是子页2
    return frameData[833];
}

static int MLX90640_I2CGeneralReset(uint16_t *eeMLX90640, struct i2c_client *client)
{
    int ret;
    // 发送通用复位命令
    uint8_t reset_cmd = 0x06;
    struct i2c_msg msg = {
        .addr = 0x00, // 设备地址
        .flags = 0,
        .len = 1,
        .buf = &reset_cmd};
    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0)
    {
        dprintk("send reset message error, i2c_transfer returned: %d", ret);
        return ret;
    }
    if (eeMLX90640)
    {
        ret = MLX90640_DumpEE(client, eeMLX90640);
    }
    return ret;
}

/* 设置刷新率 */
static int MLX90640_SetRefreshRate(struct mlx90640_device *mlxdev, int *refreshRate)
{
    uint16_t controlRegister[1];
    uint16_t value;
    int error;
    uint16_t rateMessage;

    if (*refreshRate < 1)
    {
        rateMessage = MLX90640_REFRESHRATE_0HZ;
        *refreshRate = 0;
    }
    else if (*refreshRate == 1)
    {
        rateMessage = MLX90640_REFRESHRATE_1HZ;
        *refreshRate = 1;
    }
    else if (1 < *refreshRate && *refreshRate < 3)
    {
        rateMessage = MLX90640_REFRESHRATE_2HZ;
        *refreshRate = 2;
    }
    else if (3 <= *refreshRate && *refreshRate < 6)
    {
        rateMessage = MLX90640_REFRESHRATE_4HZ;
        *refreshRate = 4;
    }
    else if (6 <= *refreshRate && *refreshRate < 12)
    {
        rateMessage = MLX90640_REFRESHRATE_8HZ;
        *refreshRate = 8;
    }
    else if (12 <= *refreshRate && *refreshRate < 24)
    {
        rateMessage = MLX90640_REFRESHRATE_16HZ;
        *refreshRate = 16;
    }
    else if (24 <= *refreshRate && *refreshRate < 48)
    {
        rateMessage = MLX90640_REFRESHRATE_32HZ;
        *refreshRate = 32;
    }
    else
    {
        rateMessage = MLX90640_REFRESHRATE_64HZ;
        *refreshRate = 64;
    }

    value = ((uint16_t)rateMessage << MLX90640_CTRL_REFRESH_SHIFT);
    value &= ~MLX90640_CTRL_REFRESH_MASK;

    error = MLX90640_I2CRead(mlxdev->client->addr, MLX90640_CTRL_REG, 1, controlRegister, mlxdev->client);
    if (error == MLX90640_NO_ERROR)
    {
        value = (controlRegister[0] & MLX90640_CTRL_REFRESH_MASK) | value;
        error = MLX90640_I2CWrite(mlxdev->client->addr, MLX90640_CTRL_REG, value, mlxdev->client);
    }
    mlxdev->rateMessage = rateMessage;
    mlxdev->refreshRate = *refreshRate;
    dprintk("set rate message:%x, refresh rate:%d\n", rateMessage, *refreshRate);
    if (*refreshRate == 0)
        mlxdev->refreshtime_ms = 2000;
    else
        mlxdev->refreshtime_ms = 1000 / *refreshRate;
    return error;
}

/* Private functions */
static struct mlx90640_frame_buf *mlx90640_get_next_buf(struct mlx90640_device *mlxdev)
{
    unsigned long flags;
    struct mlx90640_frame_buf *buf = NULL;

    spin_lock_irqsave(&mlxdev->queued_bufs_lock, flags);
    if (list_empty(&mlxdev->queued_bufs))
        goto leave;

    buf = list_entry(mlxdev->queued_bufs.next, struct mlx90640_frame_buf, list);
    list_del(&buf->list);
leave:
    spin_unlock_irqrestore(&mlxdev->queued_bufs_lock, flags);
    return buf;
}

static int mlx90640_complete(struct mlx90640_device *mlxdev)
{
    struct mlx90640_frame_buf *buf;
    int framsize = sizeof(uint16_t) * MLX90640FRAME_SIZE;

    void *ptr;
    /* get free framebuffer */
    buf = mlx90640_get_next_buf(mlxdev);
    if (unlikely(buf == NULL))
    {
        goto skip;
    }

    /* fill framebuffer */
    ptr = vb2_plane_vaddr(&buf->vb, 0);
    memcpy(ptr, (uint8_t *)mlxdev->mlx90640Frame, framsize);

    vb2_set_plane_payload(&buf->vb, 0, framsize);
    vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);
    return 1;
skip:
    return -1;
}

static int streaming_thread(void *data)
{
    // struct mlx90640_frame_buf *buf;
    // void *ptr;
    int ret;
    // int cnt = 0;
    struct mlx90640_device *mlxdev = data;
    // int framsize = sizeof(uint16_t) * MLX90640FRAME_SIZE;

    while (!kthread_should_stop())
    {
        if (!mlxdev->streaming)
        {
            msleep(20); // 停止时减小 CPU 占用
            continue;
        }
        // 通过 I2C 读取数据
        ret = MLX90640_GetFrameData(mlxdev->client, mlxdev->mlx90640Frame);
        /* just for debug */
        /*for (ret = 0; ret < MLX90640FRAME_SIZE; ret++)
        {
            printk("%04d ", mlxdev->mlx90640Frame[ret]);
            if ((ret + 1) % 16 == 0)
                printk("\n"); // 每16个数据换行
        }*/

        /* don't need it? */
        /*if (ret == DATA_NO_READY)
        {
            if (cnt < 5)
            {
                cnt++;
                continue;
            }
            else
            {
                cnt = 0;
                dprintk("mlx90640 streaming_thread: data not ready, retry\n");
                msleep(20);
                continue;
            }
        }
        else if (ret < 0)
        {
            return ret;
        }*/

        // put the message to the buffer
        if (!mlx90640_complete(mlxdev))
        {
            continue;
        }
        dprintk("mlxdev->refreshtime_ms = %d\n", mlxdev->refreshtime_ms);
        msleep(mlxdev->refreshtime_ms);
    }
    return 0;
}

/* Videobuf2 operations */
static int mlx90640_queue_setup(struct vb2_queue *vq,
                                const struct v4l2_format *fmt, unsigned int *nbuffers,
                                unsigned int *nplanes, unsigned int sizes[], void *alloc_ctxs[])
{
    struct mlx90640_device *mlxdev = vb2_get_drv_priv(vq);

    dprintk("nbuffers=%d\n", *nbuffers);

    /* Need at least 8 buffers */
    if (vq->num_buffers + *nbuffers < 8)
        *nbuffers = 8 - vq->num_buffers;
    *nplanes = 1;
    sizes[0] = PAGE_ALIGN(mlxdev->buffersize);

    dprintk("nbuffers=%d sizes[0]=%d\n", *nbuffers, sizes[0]);
    return 0;
}

static void mlx90640_buf_queue(struct vb2_buffer *vb)
{
    struct mlx90640_device *mlxdev = vb2_get_drv_priv(vb->vb2_queue);
    struct mlx90640_frame_buf *buf = container_of(vb, struct mlx90640_frame_buf, vb);
    unsigned long flags;
    if (!mlxdev)
    {
        pr_err("Failed to get device private data from vb2_queue\n");
        return;
    }

    spin_lock_irqsave(&mlxdev->queued_bufs_lock, flags);

    list_add_tail(&buf->list, &mlxdev->queued_bufs);
    spin_unlock_irqrestore(&mlxdev->queued_bufs_lock, flags);
}

static int mlx90640_start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct mlx90640_device *mlxdev = vb2_get_drv_priv(vq);
    // struct mlx90640_frame_buf *buf = container_of(vb, struct mlx90640_frame_buf, vb);
    /* 启动硬件传输 */
    if (!mlxdev)
    {
        pr_err("start_streaming: Failed to get device private data from vb2_queue\n");
        return -EINVAL;
    }

    mlxdev->streaming = true;
    mlxdev->thread = kthread_run(streaming_thread, mlxdev, "mlx90640_thread");
    if (IS_ERR(mlxdev->thread))
    {
        mlxdev->streaming = false;
        return PTR_ERR(mlxdev->thread);
    }
    return 0;
}

static void mlx90640_stop_streaming(struct vb2_queue *vq)
{
    struct mlx90640_device *mlxdev = vb2_get_drv_priv(vq);
    struct mlx90640_frame_buf *buf;
    mlxdev->streaming = false;
    if (mlxdev->thread)
    {
        kthread_stop(mlxdev->thread);
        mlxdev->thread = NULL;
    }

    msleep(20);

    while (!list_empty(&mlxdev->queued_bufs))
    {
        buf = list_entry(mlxdev->queued_bufs.next,
                         struct mlx90640_frame_buf, list);
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
    }
}

static int mlx90640_querycap(struct file *file, void *fh,
                             struct v4l2_capability *cap)
{
    struct mlx90640_device *mlxdev = video_drvdata(file);

    strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
    strlcpy(cap->card, mlxdev->vdev.name, sizeof(cap->card));
    // usb_make_path(mlxdev->udev, cap->bus_info, sizeof(cap->bus_info));
    cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
    return 0;
}

static int mlx90640_enum_fmt_cap(struct file *file, void *priv,
                                 struct v4l2_fmtdesc *f)
{
    if (f->index >= NUM_FORMATS)
        return -EINVAL;

    strlcpy(f->description, formats[f->index].name, sizeof(f->description));
    f->pixelformat = formats[f->index].pixelformat;

    return 0;
}

static int mlx90640_g_fmt_cap(struct file *file, void *priv,
                              struct v4l2_format *f)
{
    struct mlx90640_device *mlxdev = video_drvdata(file);

    f->fmt.pix.width = mlxdev->width;
    f->fmt.pix.height = mlxdev->height;
    f->fmt.pix.pixelformat = mlxdev->pixelformat;
    return 0;
}

static int mlx90640_s_fmt_cap(struct file *file, void *priv,
                              struct v4l2_format *f)
{
    struct mlx90640_device *mlxdev = video_drvdata(file);
    struct vb2_queue *q = &mlxdev->mlx90640_vb_queue;
    int i;

    if (vb2_is_busy(q))
        return -EBUSY;

    for (i = 0; i < NUM_FORMATS; i++)
    {
        if (formats[i].pixelformat == f->fmt.pix.pixelformat)
        {
            mlxdev->pixelformat = formats[i].pixelformat;
            mlxdev->buffersize = formats[i].buffersize;
            mlxdev->width = formats[i].width;
            mlxdev->height = formats[i].height;
            f->fmt.pix.width = formats[i].width;
            f->fmt.pix.height = formats[i].height;
            return 0;
        }
    }

    mlxdev->pixelformat = formats[0].pixelformat;
    mlxdev->buffersize = formats[0].buffersize;
    mlxdev->width = formats[0].width;
    mlxdev->height = formats[0].height;
    f->fmt.pix.pixelformat = formats[0].pixelformat;
    f->fmt.pix.width = formats[0].width;
    f->fmt.pix.height = formats[0].height;
    return 0;
}

static int mlx90640_try_fmt_cap(struct file *file, void *priv,
                                struct v4l2_format *f)
{
    int i;

    for (i = 0; i < NUM_FORMATS; i++)
    {
        if (formats[i].pixelformat == f->fmt.pix.pixelformat)
        {
            f->fmt.pix.width = formats[i].width;
            f->fmt.pix.height = formats[i].height;
            return 0;
        }
    }

    f->fmt.pix.pixelformat = formats[0].pixelformat;
    f->fmt.pix.width = formats[0].width;
    f->fmt.pix.height = formats[0].height;

    return 0;
}

static long mlx90640_custom_cmd(struct file *file, void *fh, bool valid_prio, unsigned int cmd, void *arg)
{
    struct mlx90640_device *mlxdev = video_drvdata(file);
    int ret = 0;
    struct eeMLX90640_ioctl_data *data;

    if (!valid_prio)
    {
        dprintk("%s device busy\n", __func__);
        return -EBUSY;
    }

    /* If streaming is started, return error */
    if (vb2_is_busy(&mlxdev->mlx90640_vb_queue))
    {
        dprintk("%s device busy\n", __func__);
        return -EBUSY;
    }

    switch (cmd)
    {
    case VIDIOC_MLX90640_IOCP_GET_EEPROM:
        dprintk("ioctl get EEPROM through pointer\n");

        data = kzalloc(sizeof(struct eeMLX90640_ioctl_data), GFP_KERNEL);
        memcpy(&data->eeMLX90640, mlxdev->eeMLX90640, sizeof(uint16_t) * EE_MLX90640_SIZE);
        // 将数组从内核空间复制到用户空间
        memcpy(arg, data, sizeof(uint16_t) * EE_MLX90640_SIZE);

        /*if (ret != 0)
        {
            dprintk("Copy Failed--amount=%d\n", ret);
            kfree(data);
            return -EFAULT; // 复制失败
        }*/
        kfree(data);
        break;

    default:
        ret = -ENOTTY;
        break;
    }
    dprintk("ret = %d\n", ret);
    return ret;
}

static struct vb2_ops mlx90640_vb2_ops = {
    .queue_setup = mlx90640_queue_setup,
    .buf_queue = mlx90640_buf_queue,
    .start_streaming = mlx90640_start_streaming,
    .stop_streaming = mlx90640_stop_streaming,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
};

static const struct v4l2_file_operations virtual_fops = {
    .owner = THIS_MODULE,
    .open = v4l2_fh_open,
    .release = vb2_fop_release,
    .read = vb2_fop_read,
    .poll = vb2_fop_poll,
    .mmap = vb2_fop_mmap,
    .unlocked_ioctl = video_ioctl2,
};

static const struct v4l2_ioctl_ops virtual_ioctl_ops = {
    .vidioc_querycap = mlx90640_querycap,

    .vidioc_enum_fmt_vid_cap = mlx90640_enum_fmt_cap,
    .vidioc_g_fmt_vid_cap = mlx90640_g_fmt_cap,
    .vidioc_s_fmt_vid_cap = mlx90640_s_fmt_cap,
    .vidioc_try_fmt_vid_cap = mlx90640_try_fmt_cap,

    .vidioc_reqbufs = vb2_ioctl_reqbufs,
    .vidioc_create_bufs = vb2_ioctl_create_bufs,
    .vidioc_prepare_buf = vb2_ioctl_prepare_buf,
    .vidioc_querybuf = vb2_ioctl_querybuf,
    .vidioc_qbuf = vb2_ioctl_qbuf,
    .vidioc_dqbuf = vb2_ioctl_dqbuf,

    .vidioc_streamon = vb2_ioctl_streamon,
    .vidioc_streamoff = vb2_ioctl_streamoff,

    .vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
    .vidioc_unsubscribe_event = v4l2_event_unsubscribe,
    .vidioc_log_status = v4l2_ctrl_log_status,

    .vidioc_default = mlx90640_custom_cmd,
};

static struct video_device mlx90640_vdev = {
    .name = "mlx90640_device",
    .release = video_device_release_empty,
    .fops = &virtual_fops,
    .ioctl_ops = &virtual_ioctl_ops,
};

/* 通过I2C对MLX90640进行初始化 */
static int mlx90640_i2c_init(struct mlx90640_device *mlxdev, int *refresh_rate)
{
    int ret;
    dprintk("refresh_rate = %d\n", *refresh_rate);
    if (!*refresh_rate)
    // if (!refresh_rate) /* a mistake, but set refresh_rate as 0.5fps */
    {
        *refresh_rate = 8;
    }
    dprintk("now refresh_rate = %d\n", *refresh_rate);
    // 发送复位命令
    ret = MLX90640_I2CGeneralReset(mlxdev->eeMLX90640, mlxdev->client);
    if (ret < 0)
    {
        dprintk("MLX90640_I2CGeneralReset failed.\n");
        return ret;
    }

    // 设置刷新率
    ret = MLX90640_SetRefreshRate(mlxdev, refresh_rate);
    if (ret < 0)
    {
        dprintk("MLX90640_SetRefreshRate failed.\n");
        return ret;
    }

    return 0;
}

static void mlx90640_v4l2_release(struct v4l2_device *v)
{
    struct mlx90640_device *mlxdev = container_of(v, struct mlx90640_device, v4l2_dev);

    // v4l2_ctrl_handler_free(&mlxdev->hdl);
    v4l2_device_unregister(&mlxdev->v4l2_dev);
    // kfree(mlxdev);
}

static int mlx90640_device_init(struct mlx90640_device *mlxdev)
{
    int ret;

    mlxdev->pixelformat = formats[0].pixelformat;
    mlxdev->buffersize = formats[0].buffersize;
    mlxdev->width = formats[0].width;
    mlxdev->height = formats[0].height;

    spin_lock_init(&mlxdev->queued_bufs_lock);
    mutex_init(&mlxdev->vb_queue_lock);
    INIT_LIST_HEAD(&mlxdev->queued_bufs);
    /* 初始化vb_queue */
    mlxdev->mlx90640_vb_queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    mlxdev->mlx90640_vb_queue.io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;
    mlxdev->mlx90640_vb_queue.drv_priv = mlxdev;
    mlxdev->mlx90640_vb_queue.buf_struct_size = sizeof(struct mlx90640_frame_buf);
    mlxdev->mlx90640_vb_queue.ops = &mlx90640_vb2_ops,
    mlxdev->mlx90640_vb_queue.mem_ops = &vb2_vmalloc_memops;
    mlxdev->mlx90640_vb_queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC | V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
    ret = vb2_queue_init(&mlxdev->mlx90640_vb_queue);
    if (ret)
    {
        dprintk("Could not initialize vb2 queue\n");
        goto err_free_mem;
    }

    /* Init video_device structure */
    mlxdev->vdev = mlx90640_vdev;
    mlxdev->vdev.queue = &mlxdev->mlx90640_vb_queue;
    mlxdev->vdev.queue->lock = &mlxdev->vb_queue_lock;
    video_set_drvdata(&mlxdev->vdev, mlxdev);

    /* Register the v4l2_device structure */
    mlxdev->v4l2_dev.release = mlx90640_v4l2_release;
    dprintk(KERN_INFO "Exiting %s\n", KBUILD_MODNAME);
    strlcpy(mlxdev->v4l2_dev.name, KBUILD_MODNAME, sizeof(mlxdev->v4l2_dev.name)); // v4l2_device_register第一个参数为空则需要指定name
    ret = v4l2_device_register(NULL, &mlxdev->v4l2_dev);
    if (ret)
    {
        dprintk("Failed to register v4l2-device (%d)\n", ret);
        goto err_free_mem;
    }

    mlxdev->vdev.v4l2_dev = &mlxdev->v4l2_dev;

    mlxdev->dev = &mlxdev->vdev.dev;
    ret = video_register_device(&mlxdev->vdev, VFL_TYPE_GRABBER, -1);
    if (ret)
    {
        dev_err(mlxdev->dev, "Failed to register as video device (%d)\n", ret);
        goto err_unregister_v4l2_dev;
    }
    dev_info(mlxdev->dev, "Registered as %s\n", video_device_node_name(&mlxdev->vdev));
    return 0;

err_unregister_v4l2_dev:
    v4l2_device_unregister(&mlxdev->v4l2_dev);
err_free_mem:
    return ret;
}

static int mlx90640_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    struct mlx90640_device *mlxdev;

    mlxdev = kzalloc(sizeof(struct mlx90640_device), GFP_KERNEL);
    if (mlxdev == NULL)
    {
        pr_err("Could not allocate memory for state\n");
        return -ENOMEM;
    }
    dprintk("This is the i2c probe function\n");
    dprintk("debug = %d\n", debug);

    if (!client)
    {
        pr_err("Invalid I2C client\n");
        return -ENODEV;
    }

    mlxdev->client = client;
    dev_set_drvdata(&client->dev, mlxdev);

    ret = mlx90640_device_init(mlxdev);
    if (ret != 0)
    {
        pr_err("Could not init mlxdev:ret = %d\n", ret);
        goto err_free_mlxdev;
    }

    /* 这两个初始化放在别的地方 */
    mlxdev->eeMLX90640 = kzalloc(sizeof(uint16_t) * EE_MLX90640_SIZE, GFP_KERNEL);
    if (!mlxdev->eeMLX90640)
    {
        pr_err("Failed to allocate memory for eeMLX90640\n");
        return -ENOMEM;
    }
    mlxdev->mlx90640Frame = kzalloc(sizeof(uint16_t) * MLX90640FRAME_SIZE, GFP_KERNEL);
    if (!mlxdev->mlx90640Frame)
    {
        pr_err("Failed to allocate memory for mlx90640Frame\n");
        kfree(mlxdev->eeMLX90640);
        return -ENOMEM;
    }

    ret = mlx90640_i2c_init(mlxdev, &mlxdev->refreshRate);
    if (ret < 0)
    {
        goto i2c_init_error;
    }

    printk("MLX90640 I2C driver initialized successfully\n");
    return 0;

i2c_init_error:
    kfree(mlxdev->eeMLX90640);
    kfree(mlxdev->mlx90640Frame);
err_free_mlxdev:
    kfree(mlxdev);
    return ret;
}

static int mlx90640_i2c_remove(struct i2c_client *client)
{
    struct mlx90640_device *mlxdev;

    mlxdev = dev_get_drvdata(&client->dev);
    printk("This is the i2c remove function\n");
    mutex_lock(&mlxdev->vb_queue_lock);
    v4l2_device_unregister(&mlxdev->v4l2_dev);
    video_unregister_device(&mlxdev->vdev);
    mutex_unlock(&mlxdev->vb_queue_lock);
    v4l2_device_put(&mlxdev->v4l2_dev);
    kfree(mlxdev->eeMLX90640);
    kfree(mlxdev->mlx90640Frame);
    kfree(mlxdev);
    return 0;
}

static const struct i2c_device_id mlx90640_id[] = {
    {"drivertest,mlx90640", 0},
    {}};

static const struct of_device_id mlx90640_match_table[] = {
    {.compatible = "drivertest,mlx90640"},
    {/* Sentinel */}};

static struct i2c_driver mlx90640_i2c_driver = {
    .probe = mlx90640_i2c_probe,
    .remove = mlx90640_i2c_remove,
    .driver = {
        .name = "mlx90640",
        .owner = THIS_MODULE,
        .of_match_table = mlx90640_match_table,
    },
    .id_table = mlx90640_id,
};

static int __init mlx90640_driver_init(void)
{
    return i2c_add_driver(&mlx90640_i2c_driver);
}

static void __exit mlx90640_driver_exit(void)
{
    i2c_del_driver(&mlx90640_i2c_driver);
}

module_init(mlx90640_driver_init);
module_exit(mlx90640_driver_exit);

MODULE_AUTHOR("yuqi <19956325811@163.com>");
MODULE_DESCRIPTION("MLX90640 DRIVER BY V4L2");
MODULE_LICENSE("GPL");
