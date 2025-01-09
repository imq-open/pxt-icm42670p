/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "example_raw.h"

/* InvenSense utils */
#include "Message.h"
#include "ErrorHelper.h"
#include "RingBuffer.h"

#include "system_interface.h"

/* std */
#include <stdio.h>

/*
 * Select UART port on which INV_MSG() will be printed.
 */
#define LOG_UART_ID INV_UART_SENSOR_CTRL

/*
 * Define msg level
 */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG

/* --------------------------------------------------------------------------------------
 *  Global variables
 * -------------------------------------------------------------------------------------- */

#if !USE_FIFO
/*
 * Buffer to keep track of the timestamp when IMU data ready interrupt fires.
 * The buffer can contain up to 64 items in order to store one timestamp for each packet in FIFO.
 */
RINGBUFFER(timestamp_buffer, 64, uint64_t);
#endif

/* --------------------------------------------------------------------------------------
 *  Static variables
 * -------------------------------------------------------------------------------------- */

/* Flag set from IMU device irq handler */
static volatile int irq_from_device;

/* --------------------------------------------------------------------------------------
 *  Forward declaration
 * -------------------------------------------------------------------------------------- */
static int setup_mcu(struct inv_imu_serif *icm_serif);
static void ext_interrupt_cb(void *context, unsigned int int_num);
static int check_rc(int rc, const char *msg_context);
void msg_printer(int level, const char *str, va_list ap);

/* --------------------------------------------------------------------------------------
 *  Main
 * -------------------------------------------------------------------------------------- */

int main1(void)
{
    int rc = 0;
    struct inv_imu_serif icm_serif;

    rc |= setup_mcu(&icm_serif);
    rc |= setup_imu_device(&icm_serif);
    rc |= configure_imu_device();
    if (check_rc(rc, "error during initialization"))
    {
        return rc;
    }

    INV_MSG(INV_MSG_LEVEL_INFO, "IMU device successfully initialized");

    do
    {
        /* Poll device for data */
        if (irq_from_device & TO_MASK(INV_GPIO_INT1))
        {
            // inv_disable_irq();
            irq_from_device &= ~TO_MASK(INV_GPIO_INT1);
            // inv_enable_irq();

            rc = get_imu_data();
            check_rc(rc, "error while getting data");
            rc = 0;
        }

        // Yield to scheduler
        inv_delay_us(1000);

    } while (0 == rc);

    INV_MSG(INV_MSG_LEVEL_INFO, "Example exit");

    return rc;
}

/* --------------------------------------------------------------------------------------
 *  Functions definitions
 * -------------------------------------------------------------------------------------- */

/*
 * This function initializes MCU on which this software is running.
 * It configures:
 *   - a UART link used to print some messages
 *   - interrupt priority group and GPIO so that MCU can receive interrupts from IMU
 *   - a microsecond timer requested by IMU driver to compute some delay
 *   - a microsecond timer used to get some timestamps
 *   - a serial link to communicate from MCU to IMU
 */
static int setup_mcu(struct inv_imu_serif *icm_serif)
{
    int rc = 0;

    inv_io_hal_board_init();

    /* configure UART */
    config_uart();

    /* Setup message facility to see internal traces from FW */
    INV_MSG_SETUP(MSG_LEVEL, msg_printer);

    INV_MSG(INV_MSG_LEVEL_INFO, "######################");
    INV_MSG(INV_MSG_LEVEL_INFO, "#   Example Raw AG   #");
    INV_MSG(INV_MSG_LEVEL_INFO, "######################");

    /*
     * Configure input capture mode GPIO connected to pin EXT3-9 (pin PB03).
     * This pin is connected to Icm406xx INT1 output and thus will receive interrupts
     * enabled on INT1 from the device.
     * A callback function is also passed that will be executed each time an interrupt
     * fires.
     */
    inv_gpio_sensor_irq_init(INV_GPIO_INT1, ext_interrupt_cb, 0);

    /* Initialize serial interface between MCU and IMU */
    icm_serif->context = 0; /* no need */
    icm_serif->read_reg = inv_io_hal_read_reg;
    icm_serif->write_reg = inv_io_hal_write_reg;
    icm_serif->max_read = 1024 * 32;  /* maximum number of bytes allowed per serial read */
    icm_serif->max_write = 1024 * 32; /* maximum number of bytes allowed per serial write */
    icm_serif->serif_type = SERIF_TYPE;
    rc |= inv_io_hal_init();

    return rc;
}

/*
 * IMU interrupt handler.
 * Function is executed when an IMU interrupt rises on MCU.
 * This function get a timestamp and store it in the timestamp buffer.
 * Note that this function is executed in an interrupt handler and thus no protection
 * are implemented for shared variable timestamp_buffer.
 */
static void ext_interrupt_cb(void *context, unsigned int int_num)
{
    (void)context;

    // INV_MSG(INV_MSG_LEVEL_INFO, "ext_interrupt_cb");

#if !USE_FIFO
    /*
     * Read timestamp from the timer dedicated to timestamping
     */
    uint64_t timestamp = inv_timer_get_counter(TIMEBASE_TIMER);

    if (int_num == INV_GPIO_INT1)
    {
        if (!RINGBUFFER_FULL(&timestamp_buffer))
            RINGBUFFER_PUSH(&timestamp_buffer, &timestamp);
    }
#endif

    irq_from_device |= TO_MASK(int_num);
}

/*
 * Helper function to check RC value and block program exectution
 */
static int check_rc(int rc, const char *msg_context)
{
    if (rc < 0)
    {
        INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)\r\n", msg_context, rc, inv_error_str(rc));
        // while (1)
        //     ;
    }
    return rc;
}

/*
 * Printer function for message facility
 */
void msg_printer(int level, const char *str, va_list ap)
{
    static char out_str[256]; /* static to limit stack usage */
    unsigned idx = 0;
    const char *s[INV_MSG_LEVEL_MAX] = {
        "",     // INV_MSG_LEVEL_OFF
        "[E] ", // INV_MSG_LEVEL_ERROR
        "[W] ", // INV_MSG_LEVEL_WARNING
        "[I] ", // INV_MSG_LEVEL_INFO
        "[V] ", // INV_MSG_LEVEL_VERBOSE
        "[D] ", // INV_MSG_LEVEL_DEBUG
    };
    idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
    if (idx >= (sizeof(out_str)))
        return;
    idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
    if (idx >= (sizeof(out_str)))
        return;
    idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
    if (idx >= (sizeof(out_str)))
        return;

    uart_puts(out_str, idx);
}

/* --------------------------------------------------------------------------------------
 *  Extern functions definition
 * -------------------------------------------------------------------------------------- */

/* Sleep implementation */
void inv_imu_sleep_us(uint32_t us)
{
    inv_delay_us(us);
}

/* Get time implementation */
uint64_t inv_imu_get_time_us(void)
{
    return inv_get_time_us();
}
