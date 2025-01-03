
#include "system_interface.h"
#include "inv_imu_extfunc.h"

/* Get time implementation for IMU driver */
uint64_t inv_imu_get_time_us(void)
{
    return si_get_time_us();
}

/* Sleep implementation for IMU driver */
void inv_imu_sleep_us(uint32_t us)
{
    si_sleep_us(us);
}
