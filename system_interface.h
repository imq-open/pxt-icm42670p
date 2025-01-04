/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ifndef _SYSTEM_INTERFACE_H_
#define _SYSTEM_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "inv_imu_transport.h"

	/*
	 * I/O for IMU device
	 */

	/* @brief Configure I/O for IMU device.
	 *  @param[in] serif_type  Serial interface type to be used.
	 *  @return                0 on success, negative value on error.
	 */
	int si_io_imu_init(uint32_t serif_type);

	/* @brief Read register(s) implementation for IMU device.
	 *  @param[in] serif  Serial interface.
	 *  @param[in] reg   Register address to be read.
	 *  @param[out] buf  Output data from the register.
	 *  @param[in] len   Number of byte to be read.
	 *  @return          0 on success, negative value on error.
	 */
	int si_io_imu_read_reg(struct inv_imu_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len);

	/* @brief Write register(s) implementation for IMU device.
	 *  @param[in] serif  Serial interface.
	 *  @param[in] reg    Register address to be written.
	 *  @param[in] buf    Input data to write.
	 *  @param[in] len    Number of byte to be written.
	 *  @return           0 on success, negative value on error.
	 */
	int si_io_imu_write_reg(struct inv_imu_serif *serif, uint8_t reg, const uint8_t *buf, uint32_t len);

	/*
	 * Timers
	 */

	/* @brief Initialize timers IP.
	 *  @return  0 on success, negative value on error.
	 */
	int si_init_timers();

	/* @brief Sleep function implementation.
	 *  @param[in] us  Time to sleep in microseconds.
	 */
	void si_sleep_us(uint32_t us);

	/* @brief Get current time function implementation.
	 *  @return  The current time in us or 0 if get_time_us pointer is null.
	 */
	uint64_t si_get_time_us();

	/* @brief Start a periodic timer.
	 *  @param[in] period_us  Period of the timer in us.
	 *  @param[in] callback   Function to be called when period expires.
	 *  @param[out] ch        Channel number of the timer.
	 *  @return  0 on success, negative value on error.
	 */
	int si_start_periodic_timer(const uint32_t period_us, void callback(void *context), int *ch);

	/* @brief Stop a periodic timer.
	 *  @param[in] ch  Channel number of the timer.
	 *  @return  0 on success, negative value on error.
	 */
	int si_stop_periodic_timer(int ch);

	/*
	 * GPIO
	 */

#define SI_GPIO_INT1 0
#define SI_GPIO_INT2 1

	/** @brief Initializes GPIO module to trigger callback when interrupt from IMU fires.
	 *  @param[in] int_num  Interrupt pin (`SI_GPIO_INT1` or `SI_GPIO_INT2`).
	 *  @param[in] int_cb   Callback to be called when interrupt fires.
	 *  @return             0 on success, negative value on error.
	 */
	int si_init_gpio_int(unsigned int_num, void (*int_cb)(void *context, unsigned int_num));

	/*
	 * Common
	 */

	/* @brief Disable core interrupts.
	 */
	void si_disable_irq();

	/* @brief Enable core interrupts.
	 */
	void si_enable_irq();

	/*
	 * Error codes
	 */

	/* @brief In case of error, print message, file and line
	 *  @param[in] rc   Error code (rc).
	 */
	// #define SI_CHECK_RC(rc)                                                                            \
// 	do {                                                                                           \
// 		if (si_print_error_if_any(rc)) {                                                           \
// 			INV_MSG(INV_MSG_LEVEL_ERROR, "At %s (line %d)", __FILE__, __LINE__);                   \
// 			si_sleep_us(100000);                                                                   \
// 			while (1)                                                                              \
// 				;                                                                                  \
// 		}                                                                                          \
// 	} while (0)

	/* @brief In case of error, print message, file and line
	 *  @param[in] rc   Error code (rc).
	 */
#define SI_CHECK_RC(rc) si_print_error_if_any(rc)

	/* @brief Check return code from driver and print message if it is an error.
	 *  @param[in] rc  Return code from the driver.
	 *  @return        Error code (rc).
	 */
	int si_print_error_if_any(int rc);

	/*
	 * Message
	 */

	// enum /* inv_msg_level */
	// {
	// 	INV_MSG_LEVEL_OFF = 0,
	// 	INV_MSG_LEVEL_ERROR,
	// 	INV_MSG_LEVEL_WARNING,
	// 	INV_MSG_LEVEL_INFO,
	// 	INV_MSG_LEVEL_VERBOSE,
	// 	INV_MSG_LEVEL_DEBUG,
	// 	INV_MSG_LEVEL_MAX
	// };

#define INV_MSG_LEVEL_OFF 0
#define INV_MSG_LEVEL_ERROR 1
#define INV_MSG_LEVEL_WARNING 2
#define INV_MSG_LEVEL_INFO 3
#define INV_MSG_LEVEL_VERBOSE 4
#define INV_MSG_LEVEL_DEBUG 5
#define INV_MSG_LEVEL_MAX 6

#define INV_MSG(level, ...) inv_msg(level, __VA_ARGS__)

	/* @brief Display a message.
	 *  @param[in] level  Level for the message
	 *  @param[in] str    Message string
	 *  @param[in] ...    Optional arguments
	 *  @return           none
	 */
	void inv_msg(int level, const char *str, ...);

#ifdef __cplusplus
} // extern "C" {
#endif

#endif /* !_SYSTEM_INTERFACE_H_ */
