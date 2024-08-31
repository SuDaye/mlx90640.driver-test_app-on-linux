#ifndef _MLX90640_IOCTL_H_
#define _MLX90640_IOCTL_H_

#include "MLX90640_API.h"

struct eeMLX90640_ioctl_data
{
    uint16_t eeMLX90640[EE_MLX90640_SIZE];
};

#define VIDIOC_MLX90640_IOCP_GET_EEPROM _IOR('V', BASE_VIDIOC_PRIVATE + 1, struct eeMLX90640_ioctl_data)

#endif