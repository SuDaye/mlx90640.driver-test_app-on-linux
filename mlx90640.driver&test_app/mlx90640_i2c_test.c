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
#include "MLX90640_API.h"
#include "mlx90640_ioctl.h"

#define MLX90640_ADDR 0x33

#define EE_MLX90640_SIZE 832
#define MLX90640FRAME_SIZE 834

#define MLX90640_CNT 1
#define MLX90640_NAME "mlx90640"

#define MLX90640_NO_ANSWER -1
#define MLX90640_READ_ERROR -8
#define MLX90640_READ_PAGE0 0
#define MLX90640_READ_PAGE1 1

/*
 * 总共有两个函数会用到I2C，分别是MLX90640_DumpEE和MLX90640_GetFrameData
 * 其中MLX90640_DumpEE只调用一次用于获取保存于EEPROM内的全部832个字的温度计算校准数据
 * MLX90640_GetFrameData是获取当前温度计算数据的
 * 可以将MLX90640_DumpEE获取到的数据使用ioctl传给用户态
 * MLX90640_GetFrameData获取的数据高频率使用，可以使用mmap映射
 *
static void MLX90640_GetTemperatureMap(void)
{
    float emissivity = 0.95;
    float Ta, tr;
    unsigned char slaveAddress;
    static uint16_t eeMLX90640[832];
    static uint16_t mlx90640Frame[834];
    paramsMLX90640 mlx90640;
    static float mlx90640To[768];
    int status;

    //MLX90640_DumpEE调用了MLX90640_I2CRead从MLX90640_EEPROM_START_ADDRESS读取了832个数据
    status = MLX90640_DumpEE(slaveAddress, eeMLX90640);

    //解析校正参数（上一函数）为计算参数，大量计算，不用i2c
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

    //调用MLX90640_I2CRead读取一帧计算用实时数据，834个字
    status = MLX90640_GetFrameData(0x33, mlx90640Frame);

    //此函数计算得到Ta（MLX90640外壳温度），返回值是浮点数
    Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
    tr = Ta - 8.0;
    //计算最终的温度图
    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
}*/

struct i2c_driver_test
{
    struct i2c_client *client;
    void *private_data;
};

struct mlx90640_dev
{
    dev_t devid;           /* 设备号	*/
    struct class *class;   /* 类 		*/
    struct device *device; /* 设备		*/
    int major;             /* 主设备号	*/
    u8 RefreshRate;        /* 测量刷新率 */
    struct device *dev;
    struct cdev cdev;
    struct i2c_driver_test i2c_driver_test;
    uint16_t *eeMLX90640;
    uint16_t *mlx90640Frame;
};

static struct mlx90640_dev mlx90640;

static void printk_mlx90640_retmsg(int ret)
{
    if (ret > 0)
    {
        return;
    }
    switch (ret)
    {
    //-1表示设备未应答
    case -1:
        printk("MLX90640 device didn't answer.\n");
        break;
    //-2表示重新读取后发现不是预期的值。
    case -2:
        printk("The value was not the expected value after the re-read.\n");
        break;
    // -8读取异常（最可能的情况是读取速率太低了）
    case -8:
        printk("Read exception (most likely the read rate is too low).\n");
        break;
    default:
        printk("An unknown error occurred: %d\n", ret);
        break;
    }
}

static int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
static int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data);

static int MLX90640_DumpEE(uint8_t slaveAddr, uint16_t *eeData)
{
    return MLX90640_I2CRead(slaveAddr, MLX90640_EEPROM_START_ADDRESS, MLX90640_EEPROM_DUMP_NUM, eeData);
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

static int MLX90640_GetFrameData(uint8_t slaveAddr, uint16_t *frameData)
{
    uint16_t dataReady = 0;
    uint16_t controlRegister1;
    uint16_t statusRegister;
    int error = 1;
    uint16_t data[64];
    uint8_t cnt = 0;

    while (dataReady == 0)
    {
        error = MLX90640_I2CRead(slaveAddr, MLX90640_STATUS_REG, 1, &statusRegister);
        if (error != MLX90640_NO_ERROR)
        {
            return error;
        }
        // dataReady = statusRegister & 0x0008;
        dataReady = MLX90640_GET_DATA_READY(statusRegister);
    }

    error = MLX90640_I2CWrite(slaveAddr, MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE);
    if (error == -MLX90640_I2C_NACK_ERROR)
    {
        return error;
    }

    error = MLX90640_I2CRead(slaveAddr, MLX90640_PIXEL_DATA_START_ADDRESS, MLX90640_PIXEL_NUM, frameData);
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    error = MLX90640_I2CRead(slaveAddr, MLX90640_AUX_DATA_START_ADDRESS, MLX90640_AUX_NUM, data);
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    frameData[832] = controlRegister1;
    // frameData[833] = statusRegister & 0x0001;
    frameData[833] = MLX90640_GET_FRAME(statusRegister);

    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    error = ValidateAuxData(data);
    if (error == MLX90640_NO_ERROR)
    {
        for (cnt = 0; cnt < MLX90640_AUX_NUM; cnt++)
        {
            frameData[cnt + MLX90640_PIXEL_NUM] = data[cnt];
        }
    }

    error = ValidateFrameData(frameData);
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }

    return frameData[833];
}

/* I2C基本操作的实现，是由开发者自己编写的。官方提供的API函数中，某些函数会调用这些操作，这些函数也移到这个文件中来 */
/* 复位，返回值同i2c_transfer。此外，Reset将更新eeMLX90640，因此指针uint16_t *eeMLX90640被传入
 * MLX90640_DumpEE之前要确认eeMLX90640非空 */
static int MLX90640_I2CGeneralReset(uint16_t *eeMLX90640)
{
    int ret;
    struct i2c_client *client = mlx90640.i2c_driver_test.client;
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
        printk("send reset message error, i2c_transfer returned: %d", ret);
        return ret;
    }
    if (eeMLX90640)
    {
        ret = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
    }
    return ret;
}

static int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    struct i2c_client *client = mlx90640.i2c_driver_test.client;
    int ret;
    int i;
    uint8_t buf[2];
    uint8_t *tmp;
    struct i2c_msg msgs[2] = {
        {.addr = slaveAddr,
         .flags = 0,
         .len = 2,
         .buf = buf},
        {.addr = slaveAddr,
         .flags = I2C_M_RD,
         .len = nMemAddressRead * 2,
         .buf = NULL}};
    tmp = kzalloc(sizeof(uint8_t) * nMemAddressRead * 2, GFP_KERNEL);

    if (!tmp)
    {
        pr_err("Failed to allocate memory for tmp in MLX90640_I2CRead\n");
        return -ENOMEM;
    }
    msgs[1].buf = tmp;
    buf[0] = startAddress >> 8;
    buf[1] = startAddress & 0xFF;
    // printk("data1: %x\n", *data);
    ret = i2c_transfer(client->adapter, msgs, 2);
    // printk("ret: %d\n", ret);
    if (ret < 0)
    {
        kfree(tmp);
        pr_err("I2C read failed\n");
        return ret;
    }
    // 将读取的字节转换为 uint16_t
    for (i = 0; i < nMemAddressRead; i++)
    {
        data[i] = (tmp[2 * i] << 8) | tmp[2 * i + 1];
    }
    // printk("data1: %x\n", *data);
    kfree(tmp);
    return 0;
}

static int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
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

    if (i2c_transfer(mlx90640.i2c_driver_test.client->adapter, &msg, 1) < 0)
    {
        pr_err("I2C write failed\n");
        return -EIO;
    }
    return 0;
}

/* 用到的官方提供的API函数,会进行一些更改 */
/* 设置测量分辨率函数 */
static int MLX90640_SetResolution(uint8_t slaveAddr, uint8_t resolution)
{
    uint16_t controlRegister1;
    uint16_t value;
    int error;

    // value = (resolution & 0x03) << 10;
    value = ((uint16_t)resolution << MLX90640_CTRL_RESOLUTION_SHIFT);
    value &= ~MLX90640_CTRL_RESOLUTION_MASK;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);

    if (error == MLX90640_NO_ERROR)
    {
        value = (controlRegister1 & MLX90640_CTRL_RESOLUTION_MASK) | value;
        error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    }

    return error;
}

static int MLX90640_GetCurResolution(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int resolutionRAM;
    int error;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }
    resolutionRAM = (controlRegister1 & ~MLX90640_CTRL_RESOLUTION_MASK) >> MLX90640_CTRL_RESOLUTION_SHIFT;

    return resolutionRAM;
}

/* 设置刷新率 */
static int MLX90640_SetRefreshRate(uint8_t slaveAddr, uint8_t refreshRate)
{
    uint16_t controlRegister[1];
    uint16_t value;
    int error;

    // value = (refreshRate & 0x07)<<7;
    value = ((uint16_t)refreshRate << MLX90640_CTRL_REFRESH_SHIFT);
    value &= ~MLX90640_CTRL_REFRESH_MASK;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, controlRegister);
    if (error == MLX90640_NO_ERROR)
    {
        value = (controlRegister[0] & MLX90640_CTRL_REFRESH_MASK) | value;
        error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    }
    mlx90640.RefreshRate = refreshRate;
    return error;
}

static int MLX90640_GetRefreshRate(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int refreshRate;
    int error;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    if (error != MLX90640_NO_ERROR)
    {
        return error;
    }
    refreshRate = (controlRegister1 & ~MLX90640_CTRL_REFRESH_MASK) >> MLX90640_CTRL_REFRESH_SHIFT;

    return refreshRate;
}

static int MLX90640_SetInterleavedMode(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    uint16_t value;
    int error;
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    if (error == 0)
    {
        value = (controlRegister1 & ~MLX90640_CTRL_MEAS_MODE_MASK);
        error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    }
    return error;
}

static int MLX90640_SetChessMode(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    uint16_t value;
    int error;
    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    if (error == 0)
    {
        value = (controlRegister1 | MLX90640_CTRL_MEAS_MODE_MASK);
        error = MLX90640_I2CWrite(slaveAddr, MLX90640_CTRL_REG, value);
    }
    return error;
}

static int MLX90640_SetSubpageMode(uint8_t slaveAddr, int modenumber)
{
    int ret;
    switch (modenumber)
    {
    case 0:
        printk("Set subpage TV mode\n");
        ret = MLX90640_SetInterleavedMode(slaveAddr);
        break;
    case 1:
        printk("Set subpage chess mode\n");
        ret = MLX90640_SetChessMode(slaveAddr);
        break;
    default:
        printk("Invalid mode number\n");
        return -1;
    }
    printk_mlx90640_retmsg(ret);
    return ret;
}

static int MLX90640_GetSubpageMode(uint8_t slaveAddr)
{
    uint16_t controlRegister1;
    int modeRAM;
    int error;

    error = MLX90640_I2CRead(slaveAddr, MLX90640_CTRL_REG, 1, &controlRegister1);
    if (error != 0)
    {
        return error;
    }
    modeRAM = (controlRegister1 & MLX90640_CTRL_MEAS_MODE_MASK) >> MLX90640_CTRL_MEAS_MODE_SHIFT;

    return modeRAM;
}

/* File_Operations MLX90640操作函数 */
static int mlx90640_i2c_open(struct inode *inode, struct file *file)
{
    struct mlx90640_dev *mlx90640_pdev;
    mlx90640_pdev = container_of(inode->i_cdev, struct mlx90640_dev, cdev);
    printk(KERN_DEBUG "mlx90640_i2c_open\n");
    file->private_data = mlx90640_pdev;
    return 0;
}

static ssize_t mlx90640_i2c_read(struct file *file, char __user *buf, size_t size, loff_t *offset)
{
    printk("mlx90640_i2c_read\n");
    return 0;
}

static ssize_t mlx90640_i2c_write(struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
    printk("mlx90640_i2c_write\n");
    return 0;
}

static int mlx90640_i2c_release(struct inode *inode, struct file *file)
{
    printk("mlx90640_i2c_release\n");
    return 0;
}

static long mlx90640_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

    struct mlx90640_dev *mlx90640_pdev = filp->private_data;
    int ret;
    bool is_admin;

    if (_IOC_TYPE(cmd) != MLX90640_IOC_MAGIC)
        return -ENOTTY; // 检查幻数(返回值POSIX标准规定，也用-EINVAL)
    if (_IOC_NR(cmd) > MLX90640_IOC_MAXNR)
        return -ENOTTY; // 检查命令编号
    is_admin = capable(CAP_SYS_ADMIN);
    printk(KERN_INFO "capable(CAP_SYS_ADMIN):%d\n", is_admin);
    if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        if (capable(CAP_SYS_ADMIN))
        {
            printk(KERN_WARNING "Non-admin user attempted write operation.\n");
            return -EPERM; // 返回权限不足错误
        }
    }

    switch (cmd)
    {
    case MLX90640_IOC_RESET:
        printk(KERN_INFO "ioctl:mlx90640 reset\n");
        ret = MLX90640_I2CGeneralReset(mlx90640_pdev->eeMLX90640);
        break;
    case MLX90640_IOCP_GET_EEPROM:
        printk(KERN_INFO "ioctl get EEPROM through pointer\n");
        // 将数组从内核空间复制到用户空间
        ret = copy_to_user((void __user *)arg, mlx90640_pdev->eeMLX90640, sizeof(uint16_t) * EE_MLX90640_SIZE);
        if (ret != 0)
        {
            printk(KERN_INFO "Copy Failed--amount=%d\n", ret);
            return -EFAULT; // 复制失败
        }
        break;
    // 设置与获取刷新率
    case MLX90640_IOCV_SET_REFRESHRATE:
        printk(KERN_INFO "ioctl MLX90640_SetRefreshRate\n");
        ret = MLX90640_SetRefreshRate(MLX90640_ADDR, arg);
        if (ret < MLX90640_NO_ERROR)
        {
            printk_mlx90640_retmsg(ret);
            return -EFAULT;
        }
        break;
    case MLX90640_IOCV_GET_REFRESHRATE:
        printk(KERN_INFO "ioctl MLX90640_GetRefreshRate\n");
        ret = MLX90640_GetRefreshRate(MLX90640_ADDR);
        if (ret != -1)
        {
            return ret;
        }
        printk_mlx90640_retmsg(ret);
        break;
    // 设置和读取分辨率
    case MLX90640_IOCV_SET_RESOLUTION:
        printk(KERN_INFO "ioctl MLX90640_SetResolution\n");
        ret = MLX90640_SetResolution(MLX90640_ADDR, arg);
        if (ret < MLX90640_NO_ERROR)
        {
            printk_mlx90640_retmsg(ret);
            return -EFAULT;
        }
        break;
    case MLX90640_IOCV_GET_RESOLUTION:
        printk(KERN_INFO "ioctl MLX90640_GetCurResolution\n");
        ret = MLX90640_GetCurResolution(MLX90640_ADDR);
        if (ret != -1)
        {
            return ret;
        }
        printk_mlx90640_retmsg(ret);
        break;
    // 设置与读取子页模式（一般使用棋盘模式，默认也是棋盘模式）
    case MLX90640_IOCV_SET_SUBPAGE_MODE:
        printk(KERN_INFO "ioctl MLX90640_SetSubpageMode\n");
        ret = MLX90640_SetSubpageMode(MLX90640_ADDR, arg);
        if (ret < MLX90640_NO_ERROR)
        {
            printk_mlx90640_retmsg(ret);
            return -EFAULT;
        }
        break;
    case MLX90640_IOCV_GET_SUBPAGE_MODE:
        printk(KERN_INFO "ioctl MLX90640_SetSubpageMode\n");
        ret = MLX90640_GetSubpageMode(MLX90640_ADDR);
        if (ret >= MLX90640_NO_ERROR)
        {
            return ret;
        }
        break;
    // 参数无效时，虽然前面做了cmd的检查，应该不会发生
    default:
        return -ENOTTY;
        break;
    }
    return ret;
}

static int mlx90640_frame_mmap(struct file *file, struct vm_area_struct *vma)
{
    int ret;
    unsigned long phy;
    struct mlx90640_dev *mlx90640_pdev = file->private_data;

    ret = MLX90640_GetFrameData(MLX90640_ADDR, mlx90640_pdev->mlx90640Frame);
    switch (ret)
    {
    case MLX90640_NO_ANSWER:
        printk(KERN_INFO "MLX90640_NO_ANSWER\n");
        break;
    case MLX90640_READ_ERROR:
        printk(KERN_INFO "MLX90640_READ_ERROR\n");
        break;
    case MLX90640_READ_PAGE0:
        printk(KERN_INFO "MLX90640_READ_PAGE0\n");
        break;
    case MLX90640_READ_PAGE1:
        printk(KERN_INFO "MLX90640_READ_PAGE1\n");
        break;
    default:
        return -EIO;
        break;
    }

    /* 获得物理地址 */
    phy = virt_to_phys(mlx90640_pdev->mlx90640Frame);

    /* 设置属性: cache, buffer */
    // 默认是读写
    // vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

    /* map */
    ret = remap_pfn_range(vma, vma->vm_start, phy >> PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot);
    if (ret)
    {
        printk("mmap remap_pfn_range failed\n");
        return -ENOBUFS;
    }

    return 0;
}

static const struct file_operations mlx90640_ops = {
    .owner = THIS_MODULE,
    .open = mlx90640_i2c_open,
    .read = mlx90640_i2c_read,
    .write = mlx90640_i2c_write,
    .release = mlx90640_i2c_release,
    .unlocked_ioctl = mlx90640_ioctl,
    .mmap = mlx90640_frame_mmap,
};

/* 通过I2C对MLX90640进行初始化 */
static int mlx90640_i2c_init(struct mlx90640_dev *pdev, uint16_t refresh_rate)
{
    int ret;
    // printk("Here is the MLX90640_I2CGeneralReset function.\n");
    // 发送复位命令
    ret = MLX90640_I2CGeneralReset(pdev->eeMLX90640);
    if (ret < 0)
    {
        printk("MLX90640_I2CGeneralReset failed.\n");
        return ret;
    }

    // printk("Here is the MLX90640_SetRefreshRate function.\n");
    // 设置刷新率
    ret = MLX90640_SetRefreshRate(pdev->i2c_driver_test.client->addr, refresh_rate);
    if (ret < 0)
    {
        printk("MLX90640_SetRefreshRate failed.\n");
        return ret;
    }

    return 0;
}

static int mlx90640_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;

    if (!client)
    {
        pr_err("Invalid I2C client\n");
        return -ENODEV;
    }

    mlx90640.i2c_driver_test.client = client;

    /* 结构体里的两个数组进行初始化 */
    mlx90640.eeMLX90640 = kzalloc(sizeof(uint16_t) * EE_MLX90640_SIZE, GFP_KERNEL);
    if (!mlx90640.eeMLX90640)
    {
        pr_err("Failed to allocate memory for eeMLX90640\n");
        return -ENOMEM;
    }
    mlx90640.mlx90640Frame = kzalloc(sizeof(uint16_t) * MLX90640FRAME_SIZE, GFP_KERNEL);
    if (!mlx90640.mlx90640Frame)
    {
        pr_err("Failed to allocate memory for mlx90640Frame\n");
        return -ENOMEM;
    }

    // printk("Here is the mlx90640_i2c_probe function.\n");
    mlx90640_i2c_init(&mlx90640, MLX90640_REFRESHRATE_16HZ);

    /* 1、构建设备号 */
    if (mlx90640.major)
    {
        mlx90640.devid = MKDEV(mlx90640.major, 0);
        ret = register_chrdev_region(mlx90640.devid, MLX90640_CNT, MLX90640_NAME);
        if (ret < 0)
        {
            pr_err("Failed to register chrdev region\n");
            return ret;
        }
    }
    else
    {
        ret = alloc_chrdev_region(&mlx90640.devid, 0, MLX90640_CNT, MLX90640_NAME);
        if (ret < 0)
        {
            pr_err("Failed to allocate chrdev region\n");
            return ret;
        }
        mlx90640.major = MAJOR(mlx90640.devid);
    }

    /* 2、注册设备 */
    cdev_init(&mlx90640.cdev, &mlx90640_ops);
    ret = cdev_add(&mlx90640.cdev, mlx90640.devid, MLX90640_CNT);
    if (ret != 0)
    {
        goto cdev_add_error;
    }

    /* 3、创建类 */
    mlx90640.class = class_create(THIS_MODULE, MLX90640_NAME);
    if (IS_ERR(mlx90640.class))
    {
        ret = PTR_ERR(mlx90640.class);
        goto class_create_error;
    }

    /* 4、创建设备 */
    mlx90640.device = device_create(mlx90640.class, NULL, mlx90640.devid, NULL, MLX90640_NAME);
    if (IS_ERR(mlx90640.device))
    {
        ret = PTR_ERR(mlx90640.device);
        goto device_create_error;
    }

    printk("MLX90640 I2C driver initialized successfully\n");
    return 0;

cdev_add_error:
    cdev_del(&mlx90640.cdev);
    unregister_chrdev_region(mlx90640.devid, MLX90640_CNT);
    return ret;
device_create_error:
    device_destroy(mlx90640.class, mlx90640.devid);
class_create_error:
    class_destroy(mlx90640.class);
    return ret;
}

static int mlx90640_i2c_remove(struct i2c_client *client)
{
    /* kfree申请的内存 */
    kfree(mlx90640.eeMLX90640);
    kfree(mlx90640.mlx90640Frame);
    /* 删除设备 */
    cdev_del(&mlx90640.cdev);
    unregister_chrdev_region(mlx90640.devid, MLX90640_CNT);

    /* 注销掉类和设备 */
    device_destroy(mlx90640.class, mlx90640.devid);
    class_destroy(mlx90640.class);
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

static int __init mlx90640_init(void)
{
    int ret = 0;
    ret = i2c_add_driver(&mlx90640_i2c_driver);
    return ret;
}

static void __exit mlx90640_exit(void)
{
    i2c_del_driver(&mlx90640_i2c_driver);
}

/* module_i2c_driver(mlx90640_i2c_driver) */

module_init(mlx90640_init);
module_exit(mlx90640_exit);
MODULE_LICENSE("GPL");
