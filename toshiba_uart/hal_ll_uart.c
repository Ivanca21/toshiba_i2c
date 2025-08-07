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
 * @file hal_ll_uart.c
 * @brief HAL LL UART MikroE module interface implementation.
 */

#include "hal_ll_uart.h"

#include <stdint.h>

static TSB_UART_TypeDef *UART_BASE;
static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile size_t tx_head = 0;
static volatile size_t tx_tail = 0;
static volatile size_t rx_head = 0;
static volatile size_t rx_tail = 0;
static volatile bool tx_buffer_full = false;
static volatile bool rx_buffer_full = false;

// Callback function pointer for RX interrupt
static void (*rx_callback) (char received_char) = NULL;

// Callback function pointer for RX interrupt
static void (*tx_callback) (char transmitted_char) = NULL;

// ============================================================================
// PRIVATE FUNCTION DECLARATIONS
// ============================================================================
static uart_error_t usb_uart_configure_pins (void);
static uart_error_t usb_uart_configure_uart (uart_handle_t *handle);
static void usb_uart_reset_buffers (void);
static bool usb_uart_tx_buffer_empty (void);
static bool usb_uart_rx_buffer_empty (void);
static uart_error_t usb_uart_tx_buffer_write (uint8_t data);
static uart_error_t usb_uart_rx_buffer_read (uint8_t *data);

// ============================================================================
// PIN CONFIGURATION
// ============================================================================
static uart_error_t hal_ll_uart_configure_pins (uart_mikrobus_t mikrobus)
{
    if ((mikrobus == UART_MIKROBUS_1) || (mikrobus == UART_MIKROBUS_2))
    {
        // Configure TX pin (PU0) - Output
        TSB_PU->FR1 |= (1 << 0);   // Function register
        TSB_PU->CR |= (1 << 0);    // Control register - Output
        TSB_PU->IE &= ~(1 << 0);   // Input enable - Disable
        TSB_PU->OD &= ~(1 << 0);   // Open drain - Disable
        TSB_PU->PUP &= ~(1 << 0);  // Pull up - Disable
        TSB_PU->PDN &= ~(1 << 0);  // Pull down - Disable

        // Configure RX pin (PU1) - Input
        TSB_PU->FR1 |= (1 << 1);   // Function register
        TSB_PU->CR &= ~(1 << 1);   // Control register - Input
        TSB_PU->IE |= (1 << 1);    // Input enable - Enable
        TSB_PU->OD &= ~(1 << 1);   // Open drain - Disable
        TSB_PU->PUP |= (1 << 1);   // Pull up - Enable
        TSB_PU->PDN &= ~(1 << 1);  // Pull down - Disable

        return UART_SUCCESS;
    }
    else if ((mikrobus == UART_MIKROBUS_3) || (mikrobus == UART_MIKROBUS_4))
    {
        // Configure TX pin (PF6) - Output
        TSB_PF->FR1 |= (1 << 6);   // Function register
        TSB_PF->CR |= (1 << 6);    // Control register - Output
        TSB_PF->IE &= ~(1 << 6);   // Input enable - Disable
        TSB_PF->OD &= ~(1 << 6);   // Open drain - Disable
        TSB_PF->PUP &= ~(1 << 6);  // Pull up - Disable
        TSB_PF->PDN &= ~(1 << 6);  // Pull down - Disable

        // Configure RX pin (PF7) - Input
        TSB_PF->FR1 |= (1 << 7);   // Function register
        TSB_PF->CR &= ~(1 << 7);   // Control register - Input
        TSB_PF->IE |= (1 << 7);    // Input enable - Enable
        TSB_PF->OD &= ~(1 << 7);   // Open drain - Disable
        TSB_PF->PUP |= (1 << 7);   // Pull up - Enable
        TSB_PF->PDN &= ~(1 << 7);  // Pull down - Disable
        
        return UART_SUCCESS;
    }
    else
    {
        return UART_ERROR_INVALID_PARAMETER;  // Unsupported mikrobus
    }
}

// ============================================================================
// UART CONFIGURATION
// ============================================================================
static uart_error_t hal_ll_uart_configure_uart (uart_handle_t *handle)
{
    if (!handle)
    {
        return UART_ERROR_INVALID_HANDLE;
    }

    TSB_UART_TypeDef *const UART = UART_BASE;

    // Clear errors and FIFO
    UART->ERR = UART_ALL_ERR_FLAGS;
    UARTxFIFOCLR = (1 << 0);

    // Configure clock source
    UART->CLK = (0b0000 << 4);

    // Configure baud rate
    uint32_t brd = 0;
    uint16_t N, K;
    uint8_t KEN = 1;

    switch (handle->config.baud_rate)
    {
    case UART_BAUD_2400:
        N = 0x1046;
        K = 0x15;
        break;
    case UART_BAUD_4800:
        N = 0x823;
        K = 0x2B;
        break;
    case UART_BAUD_9600:
        N = 0x441;
        K = 0x15;
        break;
    case UART_BAUD_19200:
        N = 0x208;
        K = 0x0B;
        break;
    case UART_BAUD_38400:
        N = 0x104;
        K = 0x25;
        break;
    case UART_BAUD_57600:
        N = 0x0AD;
        K = 0x19;
        break;
    case UART_BAUD_115200:
    default:
        N = 0x056;
        K = 0x0D;
        break;
    }

    brd |= (KEN << 23);
    brd |= (K << 16);
    brd |= N;
    UART->BRD = brd;

    // Configure UART control registers
    uint32_t cr0 = 0;

    // Configure data bits
    switch (handle->config.data_bits)
    {
    case UART_DATA_BITS_7:
        cr0 |= (0b00 << 0);  // 7-bit data
        break;
    case UART_DATA_BITS_8:
        cr0 |= (0b01 << 0);  // 8-bit data
        break;
    case UART_DATA_BITS_9:
        cr0 |= (0b10 << 0);  // 9-bit data
        break;
    default:
        return UART_ERROR_INVALID_PARAMETER;  // Unsupported data bits
    }

    // Configure parity
    if (handle->config.parity == UART_PARITY_EVEN)
    {
        cr0 |= (0b11 << 2);  // Even parity
    }
    else if (handle->config.parity == UART_PARITY_ODD)
    {
        cr0 |= (0b01 << 2);  // Odd parity
    }

    // Configure stop bits
    if (handle->config.stop_bits == UART_STOP_BITS_2)
    {
        cr0 |= (1 << 4);  // 2 stop bits
    }
    UART->CR0 = cr0;

    // Configure receive FIFO
    UART->CR1 |= (8 << 4);             // FIFO interrupt control - RÉACTIVÉ !
    UART->CR1 |= (1 << 0) | (1 << 1);  // DMA TX & RX control

    // Configure interrupts
    UART->CR1 |= handle->config.interrupt;

    for (volatile int i = 0; i < 5000; i++);

    // Enable transmission and reception
    UART->TRANS = (1 << 1) | (1 << 0);
    UART->ERR = UART_ALL_ERR_FLAGS;

    return UART_SUCCESS;
}

// ============================================================================
// BUFFER MANAGEMENT FUNCTIONS
// ============================================================================
static void hal_ll_uart_reset_buffers (void)
{
    tx_head = 0;
    tx_tail = 0;
    rx_head = 0;
    rx_tail = 0;
    tx_buffer_full = false;
    rx_buffer_full = false;
}

static bool hal_ll_uart_tx_buffer_empty (void)
{
    return (tx_head == tx_tail) && !tx_buffer_full;
}

static bool hal_ll_uart_rx_buffer_empty (void)
{
    return (rx_head == rx_tail) && !rx_buffer_full;
}

static uart_error_t hal_ll_uart_tx_buffer_write (uint8_t data)
{
    if (tx_buffer_full)
    {
        return UART_ERROR_BUFFER_FULL;
    }

    uart_tx_buffer[tx_head] = data;
    tx_head = (tx_head + 1) % UART_TX_BUFFER_SIZE;

    if (tx_head == tx_tail)
    {
        tx_buffer_full = true;
    }

    return UART_SUCCESS;
}

static uart_error_t hal_ll_uart_rx_buffer_read (uint8_t *data)
{
    if (hal_ll_uart_rx_buffer_empty ())
    {
        return UART_ERROR_RECEPTION_FAILED;
    }

    *data = uart_rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % UART_RX_BUFFER_SIZE;
    rx_buffer_full = false;

    return UART_SUCCESS;
}

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

uart_error_t hal_ll_uart_init (uart_handle_t *handle, const uart_config_t *config,
                               uart_mikrobus_t mikrobus)
{
    if (!handle || !config)
    {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (config->interrupt == (RX_FIFO_INT_CTRL | RX_COMPLETE_INT_CTRL))
    {
        return UART_ERROR_INVALID_PARAMETER;
    }

    // Configure clock
    TSB_CG->OSCCR = 0x01;
    TSB_CG->PLL0SEL |= (0b11 << 0);
    TSB_CG->PLL0SEL |= (0x2E9020 << 8);

    if ((mikrobus == UART_MIKROBUS_1) || (mikrobus == UART_MIKROBUS_2))
    {
        TSB_CG_FSYSMENA_IPMENA16 = 1;  // Enable clk for port U
        TSB_CG_FSYSMENA_IPMENA23 = 1;  // Enable clk for UART ch02
        handle->uart_instance = 2;     // UART2
        UART_BASE = TSB_UART2;
    }
    else if ((mikrobus == UART_MIKROBUS_3) || (mikrobus == UART_MIKROBUS_4))
    {
        TSB_CG_FSYSMENA_IPMENA05 = 1;  // Enable clk for port F
        TSB_CG_FSYSMENA_IPMENA24 = 1;  // Enable clk for UART ch03
        handle->uart_instance = 3;     // UART3
        UART_BASE = TSB_UART3;
    }

    handle->config = *config;

    hal_ll_uart_reset_buffers ();

    uart_error_t result = hal_ll_uart_configure_pins (mikrobus);
    if (result != UART_SUCCESS)
    {
        return result;
    }

    result = hal_ll_uart_configure_uart (handle);
    if (result != UART_SUCCESS)
    {
        return result;
    }

    handle->is_initialized = true;
    return UART_SUCCESS;
}

uart_error_t hal_ll_uart_deinit (uart_handle_t *handle)
{
    if (!handle)
    {
        return UART_ERROR_INVALID_HANDLE;
    }

    if (!handle->is_initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }

    // Disable UART
    TSB_UART_TypeDef *const UART = UART_BASE;
    UART->TRANS = 0;
    UART->CR0 = 0;
    UART->CR1 = 0;

    hal_ll_uart_reset_buffers ();

    handle->is_initialized = false;
    return UART_SUCCESS;
}

uart_error_t hal_ll_uart_write_char (uart_handle_t *handle, char data)
{
    if (!handle)
    {
        return UART_ERROR_INVALID_HANDLE;
    }

    if (!handle->is_initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }

    TSB_UART_TypeDef *const UART = UART_BASE;

    if (handle->config.interrupt & (TX_COMPLETE_INT_CTRL | TX_FIFO_INT_CTRL))
    {
        bool was_empty = hal_ll_uart_tx_buffer_empty ();

        uart_error_t result = hal_ll_uart_tx_buffer_write ((uint8_t) data);
        if (result != UART_SUCCESS)
        {
            return result;
        }

        if (was_empty && !(UART->SR & UART_TBUSY))
        {
            uint8_t data_to_send = uart_tx_buffer[tx_tail];
            UART->DR = data_to_send;
            tx_tail = (tx_tail + 1) % UART_TX_BUFFER_SIZE;
            tx_buffer_full = false;

            if (tx_callback != NULL)
            {
                tx_callback ((char) data_to_send);
            }
        }
    }
    else
    {
        while (UART->SR & UART_TBUSY);
        UART->DR = data;
    }

    return UART_SUCCESS;
}

uart_error_t hal_ll_uart_read_char (uart_handle_t *handle, char *data)
{
    if (!handle || !data)
    {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (!handle->is_initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }

    TSB_UART_TypeDef *const UART = UART_BASE;

    if (UART->ERR != 0)
    {
        UART->ERR = UART_ALL_ERR_FLAGS;
        UARTxFIFOCLR = (1 << 0);
        return UART_ERROR_RECEPTION_FAILED;
    }

    if (!(UART->SR & UART_RRDY))
    {
        return UART_ERROR_RECEPTION_FAILED;
    }

    *data = (char) (UART->DR);
    return UART_SUCCESS;
}

uart_error_t hal_ll_uart_write_string (uart_handle_t *handle, const char *str)
{
    if (!handle || !str)
    {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (!handle->is_initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }

    uart_error_t result;
    while (*str)
    {
        result = hal_ll_uart_write_char (handle, *str);
        if (result != UART_SUCCESS)
        {
            return result;
        }
        str++;

        for (volatile int i = 0; i < 10000; i++);
    }

    return UART_SUCCESS;
}

uart_error_t hal_ll_uart_write_buffer (uart_handle_t *handle, const uint8_t *data,
                                       size_t length)
{
    if (!handle || !data)
    {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (!handle->is_initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }

    uart_error_t result;
    for (size_t i = 0; i < length; i++)
    {
        result = hal_ll_uart_write_char (handle, (char) data[i]);
        if (result != UART_SUCCESS)
        {
            return result;
        }
    }

    return UART_SUCCESS;
}

uart_error_t hal_ll_uart_read_buffer (uart_handle_t *handle, uint8_t *data, size_t length,
                                      size_t *received)
{
    if (!handle || !data || !received)
    {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (!handle->is_initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }

    *received = 0;
    char ch = 0;
    uart_error_t result;

    for (size_t i = 0; i < length; i++)
    {
        result = hal_ll_uart_read_char (handle, &ch);
        if (result != UART_SUCCESS)
        {
            break;
        }
        data[i] = (uint8_t) ch;
        (*received)++;
    }

    return (*received > 0) ? UART_SUCCESS : UART_ERROR_RECEPTION_FAILED;
}

bool hal_ll_uart_data_available (uart_handle_t *handle)
{
    if (!handle || !handle->is_initialized)
    {
        return false;
    }

    TSB_UART_TypeDef *const UART = UART_BASE;
    return (UART->SR & UART_RRDY) != 0;
}

bool hal_ll_uart_tx_ready (uart_handle_t *handle)
{
    if (!handle || !handle->is_initialized)
    {
        return false;
    }

    TSB_UART_TypeDef *const UART = UART_BASE;
    return (UART->SR & UART_TBUSY) == 0;
}

uart_error_t hal_ll_uart_flush_tx (uart_handle_t *handle)
{
    if (!handle)
    {
        return UART_ERROR_INVALID_HANDLE;
    }

    if (!handle->is_initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }

    TSB_UART_TypeDef *const UART = UART_BASE;

    // Wait for transmission to complete
    while (UART->SR & UART_TBUSY);

    return UART_SUCCESS;
}

uart_error_t hal_ll_uart_flush_rx (uart_handle_t *handle)
{
    if (!handle)
    {
        return UART_ERROR_INVALID_HANDLE;
    }

    if (!handle->is_initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }

    TSB_UART_TypeDef *const UART = UART_BASE;

    // Clear FIFO
    UARTxFIFOCLR = (1 << 0);

    // Clear any pending data
    volatile char dummy = 0;
    while (UART->SR & UART_RRDY)
    {
        dummy = UART->DR;
    }

    // Reset internal buffers
    rx_head = 0;
    rx_tail = 0;
    rx_buffer_full = false;

    return UART_SUCCESS;
}

uart_error_t hal_ll_uart_set_baud_rate (uart_handle_t *handle, uart_baud_t baud_rate)
{
    if (!handle)
    {
        return UART_ERROR_INVALID_HANDLE;
    }

    if (!handle->is_initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }

    handle->config.baud_rate = baud_rate;
    return hal_ll_uart_configure_uart (handle);
}

uart_error_t hal_ll_uart_get_config (uart_handle_t *handle, uart_config_t *config)
{
    if (!handle || !config)
    {
        return UART_ERROR_INVALID_PARAMETER;
    }

    if (!handle->is_initialized)
    {
        return UART_ERROR_NOT_INITIALIZED;
    }

    *config = handle->config;
    return UART_SUCCESS;
}

uart_error_t hal_ll_uart_enable_interrupt (uart_interrupt_t interrupt, bool enable)
{
    if (interrupt == (RX_FIFO_INT_CTRL | RX_COMPLETE_INT_CTRL))
    {
        return UART_ERROR_INVALID_PARAMETER;  // Can't do that (check ref. manual)
    }

    TSB_UART_TypeDef *const UART = UART_BASE;

    UART->TRANS &=
      ~((1 << 1) | (1 << 0));  // Disable transmission and reception temporarily

    // check SUE bit
    while (UART->SR & (1 << 31));

    if (enable)
    {
        UART->CR1 |= interrupt;
    }
    else
    {
        UART->CR1 &= ~interrupt;
    }

    UART->TRANS |= (1 << 1) | (1 << 0);  // Re-enable transmission and reception

    return UART_SUCCESS;
}

// ============================================================================
// INTERRUPT HANDLING FUNCTIONS
// ============================================================================

void hal_ll_uart_rx_interrupt_handler (void)
{
    TSB_UART_TypeDef *const UART = UART_BASE;

    // DIAGNOSTIC: Variables statiques pour debug
    static volatile uint32_t interrupt_count = 0;
    static volatile uint32_t rrdy_count = 0;
    static volatile uint32_t chars_received = 0;
    static volatile uint32_t error_count = 0;

    interrupt_count++;

    if (UART->ERR & UART_ALL_ERR_FLAGS)
    {
        error_count++;
        UART->ERR = UART_ALL_ERR_FLAGS;
    }

    while (UART->SR & UART_RRDY)
    {
        rrdy_count++;

        uint8_t received_data = (uint8_t) (UART->DR & 0xFF);
        chars_received++;

        size_t next_head = (rx_head + 1) % UART_RX_BUFFER_SIZE;
        if (next_head != rx_tail)
        {
            uart_rx_buffer[rx_head] = received_data;
            rx_head = next_head;
            rx_buffer_full = false;
        }
        else
        {
            rx_buffer_full = true;  // Buffer overflow
        }

        if (rx_callback != NULL)
        {
            rx_callback ((char) received_data);
        }

        for (volatile int i = 0; i < 10; i++);
    }
}

void hal_ll_uart_tx_interrupt_handler (void)
{
    TSB_UART_TypeDef *const UART = UART_BASE;

    // Check if transmission is complete and buffer is not busy
    if (!(UART->SR & UART_TBUSY))
    {
        if (!hal_ll_uart_tx_buffer_empty ())
        {
            uint8_t data_to_send = uart_tx_buffer[tx_tail];
            UART->DR = data_to_send;

            tx_tail = (tx_tail + 1) % UART_TX_BUFFER_SIZE;
            tx_buffer_full = false;

            if (tx_callback != NULL)
            {
                tx_callback ((char) data_to_send);
            }
        }
    }

    if (UART->ERR & UART_ALL_ERR_FLAGS)
    {
        UART->ERR = UART_ALL_ERR_FLAGS;  // Clear error flags
    }
}

uart_error_t hal_ll_uart_set_rx_callback (void (*callback) (char received_char))
{
    rx_callback = callback;
    return UART_SUCCESS;
}

uart_error_t hal_ll_uart_set_tx_callback (void (*callback) (char transmitted_char))
{
    tx_callback = callback;
    return UART_SUCCESS;
}

size_t hal_ll_uart_rx_buffer_count (void)
{
    if (rx_head >= rx_tail)
    {
        return rx_head - rx_tail;
    }
    else
    {
        return UART_RX_BUFFER_SIZE - rx_tail + rx_head;
    }
}

uart_error_t hal_ll_uart_read_from_buffer (uint8_t *data, size_t max_length,
                                           size_t *bytes_read)
{
    if (!data || !bytes_read)
    {
        return UART_ERROR_INVALID_PARAMETER;
    }

    *bytes_read = 0;

    while (*bytes_read < max_length && rx_head != rx_tail)
    {
        data[*bytes_read] = uart_rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % UART_RX_BUFFER_SIZE;
        (*bytes_read)++;
        rx_buffer_full = false;
    }

    return UART_SUCCESS;
}