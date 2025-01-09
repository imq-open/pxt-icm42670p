/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
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
#ifndef _SYSTEM_INTERFACE_H_
#define _SYSTEM_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "inv_imu_transport.h"

#define INV_GPIO_INT1 0 /* Connected to the INT1 pin of the Invensense chip. */

/* TODO: Move that somewhere else */
#ifndef TO_MASK
#define TO_MASK(a) (1U << (unsigned)(a))
#endif

    void config_uart();

    /* @brief      Transmits an array of characters on the UART.
     *              As the length is sent as parameter, the array can contain zeros and
     *              does no have to be NULL-terminated
     *  @param[in]  uart UART peripheral
     *  @param[in]  s Pointer to the array to be transmitted
     *  @param[in]  l Length of the array to be transmitted
     *  @return     0 on success
     *              UART_ERROR_MEMORY if the internal buffers are full
     *              UART_ERROR_BUSY if another transaction is already ongoing
     *              UART_ERROR if for an unknown reason the requested transaction did not start
     *              UART_ERROR_BAD_ARG if one of the arguments is unsupported
     *  @note       The data that is to be transfered will be copied to the internal buffers.
     */
    int uart_puts(const char *s, unsigned short l);

    void inv_io_hal_board_init();
    int inv_io_hal_init();
    int inv_io_hal_read_reg(struct inv_imu_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen);
    int inv_io_hal_write_reg(struct inv_imu_serif *serif, uint8_t reg, const uint8_t *wbuffer,
                             uint32_t wlen);

    /* @brief Init the sensor line interrupt to wake-up the MCU
     *  @param[in]	int_num         IRQ pin as defined by enum gpio_sensor_irq_num
     *  @param[in]	interrupt_cb    callback to call on interrupt
     *  @param[in]	context         context passed to callback
     */
    void inv_gpio_sensor_irq_init(unsigned pin_num,
                                  void (*interrupt_cb)(void *context, unsigned int_num), void *context);

    /*
     * @brief  Disable core interrupts while supporting nested interrupts.
     * To fully support nesting, interrupts must be enabled with enable_irq() function.
     */
    // void inv_disable_irq(void);
    // static inline void inv_disable_irq() {}

    /*
     * @brief  Enable core interrupts while supporting nested interrupts.
     * To fully support nesting, interrupts must be enabled with disable_irq() function.
     */
    // void inv_enable_irq(void);
    // static inline void inv_enable_irq() {}

    /*
     * @brief  Busy wait based on a 100MHz clock timer
     * The timer is start and stop for each call to this function to avoid power consumption
     * @warning Maximum timing value supported is 40s
     * @param  Timing in us
     */
    void inv_delay_us(uint32_t us);

    /*
     * @brief  Busy wait based on a 100MHz clock timer
     * The timer is start and stop for each call to this function to avoid power consumption
     * @warning Maximum timing value supported is 40s
     * @param  Timing in ms
     */
    void inv_delay_ms(uint32_t ms);

    /* @brief Get the counter value by reading the timer register
     *  @param	timer_num  Timer peripheral number
     *  @return timestamp  timestamp value in us
     */
    uint64_t inv_get_time_us();

#ifdef __cplusplus
} // extern "C" {
#endif

#endif /* !_SYSTEM_INTERFACE_H_ */
