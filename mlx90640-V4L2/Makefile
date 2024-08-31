KERNELDIR := /home/rlk/linux_kernel_code/linux-imx-rel_imx_4.1.15_2.1.1_ga_alientek_v2.2
CURRENT_PATH := $(shell pwd)

# 使用 ccflags-y 添加头文件路径
ccflags-y += -I$(CURRENT_PATH)/include

# 目标模块文件的名称
obj-m := mlx90640_video_driver.o
mlx90640_video_driver-objs := mlx90640_V4L2_driver.o

# 默认目标：编译内核模块和用户空间应用程序
all: kernel_modules video_test

# 编译用户空间应用程序
video_test: video_test.o MLX90640_API.o
	arm-linux-gnueabihf-gcc -o video_test video_test.o MLX90640_API.o -pthread -lm

video_test.o: video_test.c include/MLX90640_API.h include/mlx90640_ioctl.h
	arm-linux-gnueabihf-gcc -c -o video_test.o video_test.c

MLX90640_API.o: MLX90640_API.c include/MLX90640_API.h include/mlx90640_ioctl.h
	arm-linux-gnueabihf-gcc -c -o MLX90640_API.o MLX90640_API.c

mlx90640_V4L2_driver.o: mlx90640_V4L2_driver.c include/MLX90640_API.h include/mlx90640_ioctl.h
	$(CC) $(CFLAGS) -c -o mlx90640_V4L2_driver.o mlx90640_V4L2_driver.c

# 编译内核模块
kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

# 清理目标
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
	rm -f video_test video_test.o MLX90640_API.o mlx90640_V4L2_driver.o
