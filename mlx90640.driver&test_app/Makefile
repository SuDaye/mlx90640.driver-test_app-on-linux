KERNELDIR := /home/rlk/linux_kernel_code/linux-imx-rel_imx_4.1.15_2.1.1_ga_alientek_v2.2
CURRENT_PATH := $(shell pwd)

# 目标模块文件的名称
obj-m := mlx90640_i2c_test.o

# 默认目标：编译内核模块和用户空间应用程序
all: kernel_modules app

# 编译用户空间应用程序
app: mlx90640_app

# 编译用户空间应用程序 mlx90640_app，依赖 MLX90640_API.o
mlx90640_app: mlx90640_app.o MLX90640_API.o
	arm-linux-gnueabihf-gcc -o mlx90640_app mlx90640_app.o MLX90640_API.o -lm

# 编译 MLX90640_API.c 成为目标文件
MLX90640_API.o: MLX90640_API.c MLX90640_API.h mlx90640_ioctl.h
	arm-linux-gnueabihf-gcc -c -o MLX90640_API.o MLX90640_API.c

# 编译用户空间应用程序 mlx90640_app.c
mlx90640_app.o: mlx90640_app.c MLX90640_API.h mlx90640_ioctl.h
	arm-linux-gnueabihf-gcc -c -o mlx90640_app.o mlx90640_app.c

# 编译内核模块
kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules 

# 清理目标
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
	rm -f mlx90640_app MLX90640_API.o mlx90640_app.o
