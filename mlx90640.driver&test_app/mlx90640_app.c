#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include "mlx90640_ioctl.h"
#include "MLX90640_API.h"

#define MLX90640_RESULT_SIZE 768
#define MLX90640_RESULT_SIZE_WIDTH 24
#define MLX90640_RESULT_SIZE_LENGTH 32
#define EE_MLX90640_SIZE 832
#define MLX90640FRAME_SIZE 834

#define PRINT_TEMPERATURE_MATRIX 1
#define IOCTL_TEST 0

int main(int argc, char **argv)
{
    float emissivity = 0.95;
    float temp;
    float temp_sum = 0;
    float Ta, tr;
    unsigned char slaveAddress;
    uint16_t eeMLX90640[EE_MLX90640_SIZE];
    uint16_t *mlx90640Frame;
    paramsMLX90640 mlx90640;
    static float mlx90640To[MLX90640_RESULT_SIZE];
    int status;
    int fd;
    int i, j;
    int mmap_length;

    if (argc < 2)
    {
        printf("Usage: %s <device_file>\n", argv[0]);
        return -1;
    }

    fd = open(argv[1], O_RDWR);
    if (fd == -1)
    {
        printf("can not open file %s\n", argv[1]);
        return -1;
    }

    status = ioctl(fd, MLX90640_IOCP_GET_EEPROM, eeMLX90640);
    if (status != 0)
    {
        printf("MLX90640_IOCP_GET_EEPROM failed with status %d\n", status);
        goto get_eeprom_error;
    }

#if IOCTL_TEST
    status = ioctl(fd, MLX90640_IOC_RESET);
    status = ioctl(fd, MLX90640_IOCV_SET_REFRESHRATE, MLX90640_REFRESHRATE_32HZ);
    status = ioctl(fd, MLX90640_IOCV_GET_REFRESHRATE);
    printf("Refresh rate: 0x%04x\n", status);
    status = ioctl(fd, MLX90640_IOCV_SET_RESOLUTION, 0x03);
    status = ioctl(fd, MLX90640_IOCV_GET_RESOLUTION);
    printf("Resolution: 0x%04x\n", status);
    status = ioctl(fd, MLX90640_IOCV_SET_SUBPAGE_MODE, 0);
    status = ioctl(fd, MLX90640_IOCV_GET_SUBPAGE_MODE);
    printf("SUBPAGE_MODE: %d\n", status);
    status = ioctl(fd, MLX90640_IOCV_SET_SUBPAGE_MODE, 1);
#endif

    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    mmap_length = MLX90640FRAME_SIZE * sizeof(*mlx90640Frame);
    mlx90640Frame = mmap(NULL, mmap_length, PROT_READ, MAP_SHARED, fd, 0);

    if (mlx90640Frame == MAP_FAILED)
    {
        printf("can not mmap file %s\n", argv[1]);
        goto error;
    }
    else
    {
        printf("mmap file %s success\n", argv[1]);
    }

    while (1)
    {
        status = ioctl(fd, MLX90640_IOCV_REFRESH_FRAMEDATA);
        if (status < MLX90640_NO_ERROR)
        {
            printf("MLX90640_IOCV_REFRESH_FRAMEDATA error\n");
            goto error;
        }
        printf("patten = %d\n", mlx90640Frame[833]);
        Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
        tr = Ta - 8.0;
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
#if PRINT_TEMPERATURE_MATRIX
        for (i = 0; i < MLX90640_RESULT_SIZE_WIDTH; i++)
        {
            printf("|");
            for (j = 0; j < MLX90640_RESULT_SIZE_LENGTH; j++)
            {
                temp = mlx90640To[i * MLX90640_RESULT_SIZE_LENGTH + j];
                temp_sum = temp_sum + temp;
                if (temp < 25)
                {
                    printf("%.2f\t", temp);
                }
                else
                {
                    printf("\t", temp);
                }
            }
            printf("|\n\n");
        }
        printf("Average temperature: %.2f\n", temp_sum / (MLX90640_RESULT_SIZE_WIDTH * MLX90640_RESULT_SIZE_LENGTH));
        temp_sum = 0;
#endif
        // usleep(62500);
    }

    munmap(mlx90640Frame, mmap_length);
    close(fd);
    return 0;

error:
    munmap(mlx90640Frame, mmap_length);
    close(fd);
get_eeprom_error:
    return -1;
}
