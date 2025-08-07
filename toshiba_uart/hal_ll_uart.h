/****************************************************************************
**
** Copyright (C) ${COPYRIGHT_YEAR} MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** This file is part of the mikroSDK package
**
** Commercial License Usage
**
** Licensees holding valid commercial NECTO compilers AI licenses may use this
** file in accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The MikroElektronika Company.
** For licensing terms and conditions see
** https://www.mikroe.com/legal/software-license-agreement.
** For further information use the contact form at
** https://www.mikroe.com/contact.
**
**
** GNU Lesser General Public License Usage
**
** Alternatively, this file may be used for
** non-commercial projects under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
** OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**
****************************************************************************/
/*!
 * @file hal_ll_uart.h
 * @brief HAL LL UART MikroE module interface header file.
 */

#ifndef HAL_LL_UART_H
#define HAL_LL_UART_H

#include "mcu.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// HAL LL UART MODULE CONFIGURATION
// ============================================================================

/**
 * @brief HAL LL UART module baud rates enumeration.
 */
typedef enum
{
    UART_BAUD_2400 = 2400,
    UART_BAUD_4800 = 4800,
    UART_BAUD_9600 = 9600,
    UART_BAUD_19200 = 19200,
    UART_BAUD_38400 = 38400,
    UART_BAUD_57600 = 57600,
    UART_BAUD_115200 = 115200
} uart_baud_t;

/**
 * @brief HAL LL UART module parity enumeration.
 */
typedef enum
{
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN = 1,
    UART_PARITY_ODD = 2
} uart_parity_t;

/**
 * @brief HAL LL UART module stop bits enumeration.
 */
typedef enum
{
    UART_STOP_BITS_1 = 1,
    UART_STOP_BITS_2 = 2
} uart_stop_bits_t;

/**
 * @brief HAL LL UART module data bits enumeration.
 */
typedef enum
{
    UART_DATA_BITS_7 = 7,
    UART_DATA_BITS_8 = 8,
    UART_DATA_BITS_9 = 9
} uart_data_bits_t;

/**
 * @brief HAL LL UART module interrupt configuration enumeration.
 */
typedef enum
{
    TX_FIFO_INT_CTRL = (1 << 7),
    TX_COMPLETE_INT_CTRL = (1 << 6),
    RX_FIFO_INT_CTRL = (1 << 5),
    RX_COMPLETE_INT_CTRL = (1 << 4),
    ERROR_INT_CTRL = (1 << 2),
} uart_interrupt_t;

/**
 * @brief HAL LL UART module mikrobus enumeration.
 */
typedef enum
{
    UART_MIKROBUS_1 = 1,
    UART_MIKROBUS_2 = 2,
    UART_MIKROBUS_3 = 3,
    UART_MIKROBUS_4 = 4
} uart_mikrobus_t;

/**
 * @brief HAL LL UART module configuration structure.
 */
typedef struct
{
    uart_baud_t baud_rate;      /*!< Baud rate configuration */
    uart_parity_t parity;       /*!< Parity configuration */
    uart_stop_bits_t stop_bits; /*!< Stop bits configuration */
    uart_data_bits_t data_bits; /*!< Data bits configuration */
    uart_interrupt_t interrupt; /*!< Interrupt configuration */
} uart_config_t;

/**
 * @brief HAL LL UART module handle structure.
 */
typedef struct
{
    uart_config_t config;  /*!< Module configuration */
    bool is_initialized;   /*!< Initialization status */
    uint8_t uart_instance; /*!< UART instance used */
} uart_handle_t;

// ============================================================================
// HAL LL UART MODULE ERROR CODES
// ============================================================================
typedef enum
{
    UART_SUCCESS = 0,
    UART_ERROR_INVALID_HANDLE,
    UART_ERROR_NOT_INITIALIZED,
    UART_ERROR_INVALID_PARAMETER,
    UART_ERROR_TRANSMISSION_FAILED,
    UART_ERROR_RECEPTION_FAILED,
    UART_ERROR_BUFFER_FULL,
    UART_ERROR_TIMEOUT
} uart_error_t;

// ============================================================================
// HAL LL UART MODULE PIN DEFINITIONS
// ============================================================================
#define UART_TX_PIN 0 /*!< TX pin for HAL LL UART module (PU0) */
#define UART_RX_PIN 1 /*!< RX pin for HAL LL UART module (PU1) */

// ============================================================================
// HAL LL UART MODULE BUFFER CONFIGURATION
// ============================================================================
#define UART_TX_BUFFER_SIZE 256 /*!< Transmit buffer size */
#define UART_RX_BUFFER_SIZE 256 /*!< Receive buffer size */

// ============================================================================
// UART REGISTER DEFINITIONS
// ============================================================================
#define UART2_BASE   0x400CE800UL
#define UARTxCR0     (*(volatile uint32_t *) (UART2_BASE + 0x0004))
#define UARTxCR1     (*(volatile uint32_t *) (UART2_BASE + 0x0008))
#define UARTxCLK     (*(volatile uint32_t *) (UART2_BASE + 0x000C))
#define UARTxBRD     (*(volatile uint32_t *) (UART2_BASE + 0x0010))
#define UARTxTRANS   (*(volatile uint32_t *) (UART2_BASE + 0x0014))
#define UARTxDR      (*(volatile uint32_t *) (UART2_BASE + 0x0018))
#define UARTxSR      (*(volatile uint32_t *) (UART2_BASE + 0x001C))
#define UARTxFIFOCLR (*(volatile uint32_t *) (UART2_BASE + 0x0020))
#define UARTxERR     (*(volatile uint32_t *) (UART2_BASE + 0x0024))

// ============================================================================
// UART STATUS FLAGS
// ============================================================================
enum status_flags_e
{
    UART_SUE_BIT = (1 << 0),
    UART_TBUSY = (1 << 1),
    UART_RRDY = (1 << 0),
    UART_RXFF = (1 << 3),
    UART_RLVL_MASK = (0xF << 8)
};

// ============================================================================
// UART ERROR FLAGS
// ============================================================================
enum error_flags_e
{
    UART_PERR = (1 << 0),   /*!< Parity error */
    UART_FERR = (1 << 1),   /*!< Framing error */
    UART_BERR = (1 << 2),   /*!< Break error */
    UART_OVRERR = (1 << 3), /*!< Overrun error */
    UART_OVFERR = (1 << 4)  /*!< Overflow error */
};
#define UART_ALL_ERR_FLAGS (UART_PERR | UART_FERR | UART_BERR | UART_OVRERR | UART_OVFERR)

// ============================================================================
// PRIVATE FUNCTION DECLARATIONS
// ============================================================================

/**
 * @brief Initialize HAL LL UART module with custom configuration.
 * @param handle Pointer to HAL LL UART handle structure.
 * @param config Pointer to configuration structure.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_init (uart_handle_t *handle, const uart_config_t *config,
                               uart_mikrobus_t mikrobus);

/**
 * @brief Deinitialize HAL LL UART module.
 * @param handle Pointer to HAL LL UART handle structure.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_deinit (uart_handle_t *handle);

/**
 * @brief Send a single character via HAL LL UART.
 * @param handle Pointer to HAL LL UART handle structure.
 * @param data Character to send.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_write_char (uart_handle_t *handle, char data);

/**
 * @brief Receive a single character via HAL LL UART.
 * @param handle Pointer to HAL LL UART handle structure.
 * @param data Pointer to store received character.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_read_char (uart_handle_t *handle, char *data);

/**
 * @brief Send a string via HAL LL UART.
 * @param handle Pointer to HAL LL UART handle structure.
 * @param str Null-terminated string to send.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_write_string (uart_handle_t *handle, const char *str);

/**
 * @brief Send data buffer via HAL LL UART.
 * @param handle Pointer to HAL LL UART handle structure.
 * @param data Pointer to data buffer.
 * @param length Number of bytes to send.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_write_buffer (uart_handle_t *handle, const uint8_t *data,
                                       size_t length);

/**
 * @brief Receive data into buffer via HAL LL UART.
 * @param handle Pointer to HAL LL UART handle structure.
 * @param data Pointer to data buffer.
 * @param length Maximum number of bytes to receive.
 * @param received Pointer to store actual number of bytes received.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_read_buffer (uart_handle_t *handle, uint8_t *data, size_t length,
                                      size_t *received);

/**
 * @brief Check if data is available for reading.
 * @param handle Pointer to HAL LL UART handle structure.
 * @return true if data is available, false otherwise.
 */
bool hal_ll_uart_data_available (uart_handle_t *handle);

/**
 * @brief Check if transmit buffer is empty.
 * @param handle Pointer to HAL LL UART handle structure.
 * @return true if transmit buffer is empty, false otherwise.
 */
bool hal_ll_uart_tx_ready (uart_handle_t *handle);

/**
 * @brief Flush transmit buffer.
 * @param handle Pointer to HAL LL UART handle structure.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_flush_tx (uart_handle_t *handle);

/**
 * @brief Flush receive buffer.
 * @param handle Pointer to HAL LL UART handle structure.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_flush_rx (uart_handle_t *handle);

/**
 * @brief Configure HAL LL UART module baud rate.
 * @param handle Pointer to HAL LL UART handle structure.
 * @param baud_rate New baud rate.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_set_baud_rate (uart_handle_t *handle, uart_baud_t baud_rate);

/**
 * @brief Get current HAL LL UART module configuration.
 * @param handle Pointer to HAL LL UART handle structure.
 * @param config Pointer to store current configuration.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_get_config (uart_handle_t *handle, uart_config_t *config);

/**
 * @brief Enable or disable HAL LL UART interrupts.
 * @param interrupt Interrupt type to enable/disable.
 * @param enable true to enable, false to disable.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_enable_interrupt (uart_interrupt_t interrupt, bool enable);

/**
 * @brief UART RX interrupt handler.
 * This function should be called from the UART interrupt service routine.
 * @return Nothing.
 */
void hal_ll_uart_rx_interrupt_handler (void);

/**
 * @brief UART TX interrupt handler.
 * This function should be called from the UART interrupt service routine.
 * @return Nothing.
 */
void hal_ll_uart_tx_interrupt_handler (void);

/**
 * @brief Set callback function for received data.
 * @param callback Pointer to callback function that will be called when data is received.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_set_rx_callback (void (*callback) (char received_char));

/**
 * @brief Set callback function for transmitted data.
 * @param callback Pointer to callback function that will be called when data is
 * transmitted.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_set_tx_callback (void (*callback) (char transmitted_char));

/**
 * @brief Get number of bytes available in RX buffer.
 * @return Number of bytes available.
 */
size_t hal_ll_uart_rx_buffer_count (void);

/**
 * @brief Read data from RX buffer (non-blocking).
 * @param data Pointer to store received data.
 * @param max_length Maximum number of bytes to read.
 * @param bytes_read Pointer to store actual number of bytes read.
 * @return UART_SUCCESS if successful, error code otherwise.
 */
uart_error_t hal_ll_uart_read_from_buffer (uint8_t *data, size_t max_length,
                                           size_t *bytes_read);

#ifdef __cplusplus
}
#endif

#endif  // HAL_LL_UART_H
