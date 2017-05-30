/*
 * Copyright 2016, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 * Defines board support package for ISM43341-M4G-EVB board
 */
#include "platform.h"
#include "platform_config.h"
#include "platform_init.h"
#include "platform_isr.h"
#include "platform_peripheral.h"
#include "wwd_platform_common.h"
#include "wwd_rtos_isr.h"
#include "wiced_defaults.h"
#include "wiced_platform.h"
#include "platform_bluetooth.h"
#include "platform_mfi.h"

#include "platform_button.h"
#include "gpio_button.h"


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/*  ***** Please note the WICED_GPIO_X definitions have change from previous ISM43341_M4G_L44
 * patch files before WICED 3.5.2 ***** */

/* GPIO pin table. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_gpio_t platform_gpio_pins[] =
{									//Signal									   L44 Pin	+10 pin
    [WICED_GPIO_1 ] = { GPIOA, 15 },//TDI 											( 6)
    [WICED_GPIO_2 ] = { GPIOB,  3 },//TDO 											( 7)
    [WICED_GPIO_3 ] = { GPIOB,  4 },//TRSTN 										( 8)
    [WICED_GPIO_4 ] = { GPIOA,  9 },//UART1_TX 										(22)
    [WICED_GPIO_5 ] = { GPIOA, 10 },//UART1_RX 										(21)
    [WICED_GPIO_6 ] = { GPIOA, 11 },//UART1_CTS/USB DM								(19)
    [WICED_GPIO_7 ] = { GPIOA, 12 },//UART1_RTS/USB_DP								(18)
    [WICED_GPIO_8 ] = { GPIOB,  5 },//GPIO0 (EVB-Button SW1) 						(23)
    [WICED_GPIO_9 ] = { GPIOB,  6 },//GPIO1 (EVB-Button SW2) 						(24)
    [WICED_GPIO_10] = { GPIOB,  7 },//GPIO2 										(25)
    [WICED_GPIO_11] = { GPIOC,  6 },//GPIO3 (EVB-LED7, Red) 						(26)
    [WICED_GPIO_12] = { GPIOC,  7 },//GPIO4 (EVB-LED6, Green) 						(27)
    [WICED_GPIO_13] = { GPIOA,  3 },//ADC0/SPI1_CMD_Data Ready  (EVB-Thermistor)	(13)
    [WICED_GPIO_14] = { GPIOA,  4 },//ADC1/SPI1_SSN									(12)
    [WICED_GPIO_15] = { GPIOA,  5 },//ADC2/SPI1_SCK									(11)
    [WICED_GPIO_16] = { GPIOA,  6 },//ADC3/SPI1_MISO								(10)
    [WICED_GPIO_17] = { GPIOA,  7 },//ADC4/SPI1_MOSI								( 9)
    [WICED_GPIO_18] = { GPIOA,  0 },//WAKEUP										(16)
    [WICED_GPIO_19] = { GPIOB, 13 },//GPIO13										(32)
    [WICED_GPIO_20] = { GPIOB, 14 },//GPIO14										(31)
    [WICED_GPIO_21] = { GPIOB, 15 },//GPIO15										(30)
    [WICED_GPIO_22] = { GPIOA,  2 },//GPIOA													(45)
    [WICED_GPIO_23] = { GPIOA,  1 },//GPIOB													(46)
    [WICED_GPIO_24] = { GPIOB, 10 },//GPIOC													(47)
    [WICED_GPIO_25] = { GPIOC,  3 },//GPIOD													(48)
    [WICED_GPIO_26] = { GPIOB, 11 },//GPIOE													(49)
    [WICED_GPIO_27] = { GPIOB, 12 },//GPIOF													(50)
    [WICED_GPIO_28] = { GPIOE, 11 },//GPIOG													(51)
    [WICED_GPIO_29] = { GPIOE, 12 },//GPIOH													(52)
    [WICED_GPIO_30] = { GPIOE, 13 },//GPIOJ													(53)
    [WICED_GPIO_31] = { GPIOE, 14 },//GPIOK													(54)
 };
/* Please note that the +10pin have been define above, but functionality has not been defined
 * ie No ADCs, PWMs, Timers, or other peripherals */


/* MFI-related variables */
const wiced_i2c_device_t auth_chip_i2c_device =
{
    .port          = WICED_I2C_1,
    .address       = 0x11,
    .address_width = I2C_ADDRESS_WIDTH_7BIT,
    .speed_mode    = I2C_STANDARD_SPEED_MODE,
};

const platform_mfi_auth_chip_t platform_auth_chip =
{
    .i2c_device = &auth_chip_i2c_device,
    .reset_pin  = WICED_GPIO_AUTH_RST
};


/* ADC peripherals. Used WICED/platform/MCU/wiced_platform_common.c */
const platform_adc_t platform_adc_peripherals[] =
{
    [WICED_ADC_1] = {ADC1, ADC_Channel_3, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_13]}, /* PA3, ADC0 */
    [WICED_ADC_2] = {ADC1, ADC_Channel_4, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_14]}, /* PA4, ADC1 */
    [WICED_ADC_3] = {ADC1, ADC_Channel_5, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_15]}, /* PA5, ADC2 */
    [WICED_ADC_4] = {ADC1, ADC_Channel_6, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_16]}, /* PA6, ADC3 */
    [WICED_ADC_5] = {ADC1, ADC_Channel_7, RCC_APB2Periph_ADC1, 1, &platform_gpio_pins[WICED_GPIO_17]}, /* PA7, ADC4 */
};


/* PWM peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */

/* Inventek Systems - Un-comment to select PWM options */
//#define PWM4OPT0	/* TIM8_CH1  */
//#define PWM5OPT0	/* TIM3_CH2  */
//#define PWM7OPT0	/* TIM1_CH2N */
//#define PWM7OPT1	/* TIM8_CH2N */
//#define PWM8OPT0	/* TIM1_CH3N */
//#define PWM8OPT1	/* TIM8_CH3N */

/* STM32F405 RCC Bus
 * APB1 - TIM 2, 3, 4, 5, 6, 7, 12, 13, 14
 * APB2 - TIM 1, 8, 9, 10, 11
 */

const platform_pwm_t platform_pwm_peripherals[] =
{
    [WICED_PWM_1]  = {TIM3,  2, RCC_APB1Periph_TIM3,  GPIO_AF_TIM3,  &platform_gpio_pins[WICED_GPIO_8 ]}, /* PB5,  GPIO0,  TIM3_CH2  */
    [WICED_PWM_2]  = {TIM4,  1, RCC_APB1Periph_TIM4,  GPIO_AF_TIM4,  &platform_gpio_pins[WICED_GPIO_9 ]}, /* PB6,  GPIO1,  TIM4_CH1  */
    [WICED_PWM_3]  = {TIM4,  2, RCC_APB1Periph_TIM4,  GPIO_AF_TIM4,  &platform_gpio_pins[WICED_GPIO_10]}, /* PB7,  GPIO2,  TIM4_CH2  */

#ifdef PWM4OPT0
    [WICED_PWM_4]  = {TIM8,  1, RCC_APB2Periph_TIM8,  GPIO_AF_TIM8,  &platform_gpio_pins[WICED_GPIO_11]}, /* PC6,  GPIO3,  TIM8_CH1  */
#else /* Default */
    [WICED_PWM_4]  = {TIM3,  1, RCC_APB1Periph_TIM3,  GPIO_AF_TIM3,  &platform_gpio_pins[WICED_GPIO_11]}, /* PC6,  GPIO3,  TIM3_CH1  */
#endif

#ifdef PWM5OPT0
    [WICED_PWM_5]  = {TIM3,  2, RCC_APB1Periph_TIM3,  GPIO_AF_TIM3,  &platform_gpio_pins[WICED_GPIO_12]}, /* PC7,  GPIO4 , TIM3_CH2  */
#else /* Default */
    [WICED_PWM_5]  = {TIM8,  2, RCC_APB2Periph_TIM8,  GPIO_AF_TIM8,  &platform_gpio_pins[WICED_GPIO_12]}, /* PC7,  GPIO4 , TIM8_CH2  */
#endif

    [WICED_PWM_6]  = {TIM1,  1, RCC_APB2Periph_TIM1,  GPIO_AF_TIM1,  &platform_gpio_pins[WICED_GPIO_19]}, /* PB13, GPIO13, TIM1_CH1N */

#ifdef PWM7OPT0
    [WICED_PWM_7]  = {TIM1,  2, RCC_APB2Periph_TIM1,  GPIO_AF_TIM1,  &platform_gpio_pins[WICED_GPIO_20]}, /* PB14, GPIO14, TIM1_CH2N */
#elif defined(PWM7OPT1)
    [WICED_PWM_7]  = {TIM8,  2, RCC_APB2Periph_TIM8,  GPIO_AF_TIM8,  &platform_gpio_pins[WICED_GPIO_20]}, /* PB14, GPIO14, TIM8_CH2N */
#else /* Default */
    [WICED_PWM_7]  = {TIM12, 1, RCC_APB1Periph_TIM12, GPIO_AF_TIM12, &platform_gpio_pins[WICED_GPIO_20]}, /* PB14, GPIO14, TIM12_CH1 */
#endif

#ifdef PWM8OPT0
    [WICED_PWM_8]  = {TIM1,  3, RCC_APB1Periph_TIM1,  GPIO_AF_TIM1,  &platform_gpio_pins[WICED_GPIO_21]}, /* PB15, GPIO15, TIM1_CH3N */
#elif defined(PWM8OPT1)
    [WICED_PWM_8]  = {TIM8,  3, RCC_APB2Periph_TIM8,  GPIO_AF_TIM8,  &platform_gpio_pins[WICED_GPIO_21]}, /* PB15, GPIO15, TIM8_CH3N */
#else /* Default */
    [WICED_PWM_8]  = {TIM12, 2, RCC_APB1Periph_TIM12, GPIO_AF_TIM12, &platform_gpio_pins[WICED_GPIO_21]}, /* PB15, GPIO15, TIM12_CH2 */
#endif
};


/* UART peripherals and runtime drivers. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_uart_t platform_uart_peripherals[] =
{
    [WICED_UART_1] =
    {
        .port               = USART1,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_4],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_5],
        .cts_pin            = &platform_gpio_pins[WICED_GPIO_6],
        .rts_pin            = &platform_gpio_pins[WICED_GPIO_7],
        .tx_dma_config =
        {
            .controller     = DMA2,
            .stream         = DMA2_Stream7,
            .channel        = DMA_Channel_4,
            .irq_vector     = DMA2_Stream7_IRQn,
            .complete_flags = DMA_HISR_TCIF7,
            .error_flags    = ( DMA_HISR_TEIF7 | DMA_HISR_FEIF7 | DMA_HISR_DMEIF7 ),
        },
        .rx_dma_config =
        {
            .controller     = DMA2,
            .stream         = DMA2_Stream2,
            .channel        = DMA_Channel_4,
            .irq_vector     = DMA2_Stream2_IRQn,
            .complete_flags = DMA_LISR_TCIF2,
            .error_flags    = ( DMA_LISR_TEIF2 | DMA_LISR_FEIF2 | DMA_LISR_DMEIF2 ),
        },
    },
    [WICED_UART_2] =
    {
        .port               = USART6,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_11],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_12],
        .cts_pin            = NULL,
        .rts_pin            = NULL,
        .tx_dma_config =
        {
            .controller     = DMA2,
            .stream         = DMA2_Stream6,
            .channel        = DMA_Channel_5,
            .irq_vector     = DMA2_Stream6_IRQn,
            .complete_flags = DMA_HISR_TCIF7,
            .error_flags    = ( DMA_HISR_TEIF7 | DMA_HISR_FEIF7 | DMA_HISR_DMEIF7 ),
        },
        .rx_dma_config =
        {
            .controller     = DMA2,
            .stream         = DMA2_Stream1,
            .channel        = DMA_Channel_5,
            .irq_vector     = DMA2_Stream1_IRQn,
            .complete_flags = DMA_LISR_TCIF2,
            .error_flags    = ( DMA_LISR_TEIF2 | DMA_LISR_FEIF2 | DMA_LISR_DMEIF2 ),
        },
    },
};
platform_uart_driver_t platform_uart_drivers[WICED_UART_MAX];

/* UART standard I/O configuration */
#ifndef WICED_DISABLE_STDIO
static const platform_uart_config_t stdio_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
#ifndef ENSTDIOFC
    .flow_control = FLOW_CONTROL_DISABLED,
#else
	.flow_control = FLOW_CONTROL_CTS_RTS,
#endif
};
#endif


/* Wi-Fi control pins. Used by WICED/platform/MCU/wwd_platform_common.c
 * SDIO: WWD_PIN_BOOTSTRAP[1:0] = b'X0
 * gSPI: WWD_PIN_BOOTSTRAP[1:0] = b'01
 */
const platform_gpio_t wifi_control_pins[] =
{
    /* Power pin unavailable */
    //[WWD_PIN_POWER      ] = { GPIOx,  x },
    [WWD_PIN_RESET      ] = { GPIOD,  15 },
    [WWD_PIN_32K_CLK    ] = { GPIOA,   8 },
    [WWD_PIN_BOOTSTRAP_0] = { GPIOD,   6 },
    [WWD_PIN_BOOTSTRAP_1] = { GPIOB,   1 },
};

/* Wi-Fi SDIO bus pins. Used by WICED/platform/STM32F4xx/WWD/wwd_SDIO.c */
const platform_gpio_t wifi_sdio_pins[] =
{
    [WWD_PIN_SDIO_OOB_IRQ] = { GPIOB, 11 },
    [WWD_PIN_SDIO_CLK    ] = { GPIOC, 12 },
    [WWD_PIN_SDIO_CMD    ] = { GPIOD,  2 },
    [WWD_PIN_SDIO_D0     ] = { GPIOC,  8 },
    [WWD_PIN_SDIO_D1     ] = { GPIOC,  9 },
    [WWD_PIN_SDIO_D2     ] = { GPIOC, 10 },
    [WWD_PIN_SDIO_D3     ] = { GPIOC, 11 },
};

/* ISM433XX platforms, not usable, not connect to radio */
/* Wi-Fi gSPI bus pins. Used by WICED/platform/STM32F4xx/WWD/wwd_SPI.c */
const platform_gpio_t wifi_spi_pins[] =
{
    [WWD_PIN_SPI_IRQ ] = { GPIOC,  9 },
    [WWD_PIN_SPI_CS  ] = { GPIOC, 11 },
    [WWD_PIN_SPI_CLK ] = { GPIOB, 13 },
    [WWD_PIN_SPI_MOSI] = { GPIOB, 15 },
    [WWD_PIN_SPI_MISO] = { GPIOB, 14 },
};

/* Bluetooth control pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
static const platform_gpio_t internal_bt_control_pins[] =
{
    /* Power pin unavailable */
    //[WICED_BT_PIN_POWER      ] = { GPIOx,  x },
    [WICED_BT_PIN_RESET      ] = { GPIOD,  5 },
    [WICED_BT_PIN_HOST_WAKE  ] = { GPIOD,  6 },
    [WICED_BT_PIN_DEVICE_WAKE] = { GPIOD,  7 }
};
const platform_gpio_t* wiced_bt_control_pins[] =
{
    /* Power pin unavailable */
    //[WICED_BT_PIN_POWER      ] = &internal_bt_control_pins[WICED_BT_PIN_POWER      ],
    [WICED_BT_PIN_RESET      ] = &internal_bt_control_pins[WICED_BT_PIN_RESET      ],
    [WICED_BT_PIN_HOST_WAKE  ] = &internal_bt_control_pins[WICED_BT_PIN_HOST_WAKE  ],
    [WICED_BT_PIN_DEVICE_WAKE] = &internal_bt_control_pins[WICED_BT_PIN_DEVICE_WAKE],
    //[WICED_BT_PIN_RESET]       = NULL,
};

/* Bluetooth UART pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
static const platform_gpio_t internal_bt_uart_pins[] =
{
    [WICED_BT_PIN_UART_TX ] = { GPIOD,  9 },
    [WICED_BT_PIN_UART_RX ] = { GPIOD,  8 },
    [WICED_BT_PIN_UART_CTS] = { GPIOD, 12 },
    [WICED_BT_PIN_UART_RTS] = { GPIOD, 11 },
};
const platform_gpio_t* wiced_bt_uart_pins[] =
{
    [WICED_BT_PIN_UART_TX ] = &internal_bt_uart_pins[WICED_BT_PIN_UART_TX ],
    [WICED_BT_PIN_UART_RX ] = &internal_bt_uart_pins[WICED_BT_PIN_UART_RX ],
    [WICED_BT_PIN_UART_CTS] = &internal_bt_uart_pins[WICED_BT_PIN_UART_CTS],
    [WICED_BT_PIN_UART_RTS] = &internal_bt_uart_pins[WICED_BT_PIN_UART_RTS],
};

/* Bluetooth UART peripheral and runtime driver. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
static const platform_uart_t internal_bt_uart_peripheral =
{
    .port               = USART3,
    .tx_pin             = &internal_bt_uart_pins[WICED_BT_PIN_UART_TX ],
    .rx_pin             = &internal_bt_uart_pins[WICED_BT_PIN_UART_RX ],
    .cts_pin            = &internal_bt_uart_pins[WICED_BT_PIN_UART_CTS],
    .rts_pin            = &internal_bt_uart_pins[WICED_BT_PIN_UART_RTS],
    .tx_dma_config =
    {
        .controller     = DMA1,
        .stream         = DMA1_Stream4,
        .channel        = DMA_Channel_7,
        .irq_vector     = DMA1_Stream4_IRQn,
        .complete_flags = DMA_HISR_TCIF4,
        .error_flags    = ( DMA_HISR_TEIF4 | DMA_HISR_FEIF4 | DMA_HISR_DMEIF4 ),
    },
    .rx_dma_config =
    {
        .controller     = DMA1,
        .stream         = DMA1_Stream1,
        .channel        = DMA_Channel_4,
        .irq_vector     = DMA1_Stream1_IRQn,
        .complete_flags = DMA_LISR_TCIF1,
        .error_flags    = ( DMA_LISR_TEIF1 | DMA_LISR_FEIF1 | DMA_LISR_DMEIF1 ),
    },
};

static platform_uart_driver_t internal_bt_uart_driver;
const platform_uart_t*        wiced_bt_uart_peripheral = &internal_bt_uart_peripheral;
platform_uart_driver_t*       wiced_bt_uart_driver     = &internal_bt_uart_driver;

/* Bluetooth UART configuration. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
const platform_uart_config_t wiced_bt_uart_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_DISABLED,
};


/*BT chip specific configuration information*/
const platform_bluetooth_config_t wiced_bt_config =
{
    .patchram_download_mode      = PATCHRAM_DOWNLOAD_MODE_MINIDRV_CMD,
    .patchram_download_baud_rate = 115200,
    .featured_baud_rate          = 115200
};

gpio_button_t platform_gpio_buttons[] =
{
    [PLATFORM_BUTTON_1] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON1,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },

    [PLATFORM_BUTTON_2] =
    {
        .polarity   = WICED_ACTIVE_HIGH,
        .gpio       = WICED_BUTTON2,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
};


/* I2C peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_i2c_t platform_i2c_peripherals[] =
{
	/* ISM4334x_M4G_L44 or ISM4334x_M4G_L44+10 */
    [WICED_I2C_1] =
    {
        .port                    = I2C1,
        .pin_scl                 = &platform_gpio_pins[WICED_GPIO_9 ], 			//GPIO1 (STM32F405 PB6, L44 pin 24)
        .pin_sda                 = &platform_gpio_pins[WICED_GPIO_10], 			//GPIO2 (STM32F405 PB7, L44 pin 25)
        .peripheral_clock_reg    = RCC_APB1Periph_I2C1,
        .tx_dma                  = DMA1,
        .tx_dma_peripheral_clock = RCC_AHB1Periph_DMA1,
        .tx_dma_stream           = DMA1_Stream7,								//Alternative: DMA1_Stream6,
        .rx_dma_stream           = DMA1_Stream0,								//Alternative: DMA1_Stream5
        .tx_dma_stream_id        = 7,											//Alternative: 6
        .rx_dma_stream_id        = 0,											//Alternative: 5
        .tx_dma_channel          = DMA_Channel_1,
        .rx_dma_channel          = DMA_Channel_1,
        .gpio_af                 = GPIO_AF_I2C1
    },
	/* ISM43341_M4G_L44 ONLY */
    [WICED_I2C_2] =
    {
        .port                    = I2C1,
#ifdef ISM43341_M4G_L44
        .pin_scl                 = &internal_nfc_i2c_pins[WICED_NFC_I2C_SCL],	//STM32F405 PB8, ISM43341 Internal connection
        .pin_sda                 = &internal_nfc_i2c_pins[WICED_NFC_I2C_SDA],	//STM32F405 PB9, ISM43341 Internal connection
#else
        .pin_scl                 = 0,
        .pin_sda                 = 0,
#endif
        .peripheral_clock_reg    = RCC_APB1Periph_I2C1,
        .tx_dma                  = DMA1,
        .tx_dma_peripheral_clock = RCC_AHB1Periph_DMA1,
        .tx_dma_stream           = DMA1_Stream7,								//Alternative: DMA1_Stream6
        .rx_dma_stream           = DMA1_Stream0,								//Alternative: DMA1_Stream5
        .tx_dma_stream_id        = 7,											//Alternative: 6
        .rx_dma_stream_id        = 0,											//Alternative: 5
        .tx_dma_channel          = DMA_Channel_1,
        .rx_dma_channel          = DMA_Channel_1,
        .gpio_af                 = GPIO_AF_I2C1
    },
	/* ISM4334x_M4G_L44+10 */
    [WICED_I2C_3] =
    {
        .port                    = I2C2,
        .pin_scl                 = &platform_gpio_pins[WICED_GPIO_24], 			//GPIOC (STM32F405 PB10, +10 pin 47)
        .pin_sda                 = &platform_gpio_pins[WICED_GPIO_26], 			//GPIOE (STM32F405 PB11, +10 pin 49)
        .peripheral_clock_reg    = RCC_APB1Periph_I2C2,
        .tx_dma                  = DMA1,
        .tx_dma_peripheral_clock = RCC_AHB1Periph_DMA1,
        .tx_dma_stream           = DMA1_Stream7,
        .rx_dma_stream           = DMA1_Stream2,								//Alternative: DMA1_Stream3
        .tx_dma_stream_id        = 7,
        .rx_dma_stream_id        = 2,											//Alternative: 3
        .tx_dma_channel          = DMA_Channel_7,
        .rx_dma_channel          = DMA_Channel_7,
        .gpio_af                 = GPIO_AF_I2C2
    }
};
/* Note: if using Alternative DMA streams please update platform_init_peripheral_irq_priorities below. */


/* SPI flash. Exposed to the applications through include/wiced_platform.h */
#if defined ( WICED_PLATFORM_INCLUDES_SPI_FLASH )
#pragma message "WICED_PLATFORM_INCLUDES_SPI_FLASH"
//const wiced_spi_device_t wiced_spi_flash =
const wiced_spi_device_t wiced_spi_flash =
{
    .port        = WICED_SPI_1,
    .chip_select = WICED_SPI_FLASH_CS,
    .speed       = 5000000,
    .mode        = (SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_HIGH | SPI_NO_DMA | SPI_MSB_FIRST),
    .bits        = 8
};
#endif

#ifdef WICED_PLATFORM_INCLUDES_SPI_FLASH
/* SPI peripherals.  Used for Serial FLASH */
const platform_spi_t platform_spi_peripherals[] =
{
    [WICED_SPI_1]  =
    {
        .port                  = SPI1,
        .gpio_af               = GPIO_AF_SPI1,
        .peripheral_clock_reg  = RCC_APB2Periph_SPI1,
        .peripheral_clock_func = RCC_APB2PeriphClockCmd,
        .pin_mosi              = &platform_gpio_pins[WICED_SPI_FLASH_MOSI],
        .pin_miso              = &platform_gpio_pins[WICED_SPI_FLASH_MISO],
        .pin_clock             = &platform_gpio_pins[WICED_SPI_FLASH_CLK],
        .tx_dma =
        {
            .controller        = DMA2,
            .stream            = DMA2_Stream5,
            .channel           = DMA_Channel_3,
            .irq_vector        = DMA2_Stream5_IRQn,
            .complete_flags    = DMA_HISR_TCIF5,
            .error_flags       = ( DMA_HISR_TEIF5 | DMA_HISR_FEIF5 | DMA_HISR_DMEIF5 ),
        },
        .rx_dma =
        {
            .controller        = DMA2,
            .stream            = DMA2_Stream0,
            .channel           = DMA_Channel_3,
            .irq_vector        = DMA2_Stream0_IRQn,
            .complete_flags    = DMA_LISR_TCIF0,
            .error_flags       = ( DMA_LISR_TEIF0 | DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 ),
        },
    },
    [WICED_SPI_2]  =
    {
        .port                  = SPI3,
        .gpio_af               = GPIO_AF_SPI3,
        .peripheral_clock_reg  = RCC_APB1Periph_SPI3,
        .peripheral_clock_func = RCC_APB1PeriphClockCmd,
        .pin_mosi              = &platform_gpio_pins[WICED_GPIO_8],	//GPIO0
        .pin_miso              = &platform_gpio_pins[WICED_GPIO_3], //TRSTN
        .pin_clock             = &platform_gpio_pins[WICED_GPIO_2], //TDO
        .tx_dma =
        {
            .controller        = DMA1,
            .stream            = DMA1_Stream5,
            .channel           = DMA_Channel_0,
            .irq_vector        = DMA1_Stream5_IRQn,
            .complete_flags    = DMA_HISR_TCIF5,
            .error_flags       = ( DMA_HISR_TEIF5 | DMA_HISR_FEIF5 | DMA_HISR_DMEIF5 ),
        },
        .rx_dma =
        {
            .controller        = DMA1,
            .stream            = DMA1_Stream0,
            .channel           = DMA_Channel_0,
            .irq_vector        = DMA1_Stream0_IRQn,
            .complete_flags    = DMA_LISR_TCIF0,
            .error_flags       = ( DMA_LISR_TEIF0 | DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 ),
        },
    }

};
#endif


/******************************************************
 *               Function Definitions
 ******************************************************/

void platform_init_peripheral_irq_priorities( void )
{
    /* Interrupt priority setup. Called by WICED/platform/MCU/STM32F4xx/platform_init.c */
    NVIC_SetPriority( RTC_WKUP_IRQn    ,  1 ); /* RTC Wake-up event   */
    NVIC_SetPriority( SDIO_IRQn        ,  2 ); /* WLAN SDIO           */
    NVIC_SetPriority( DMA2_Stream3_IRQn,  3 ); /* WLAN SDIO DMA       */
    NVIC_SetPriority( DMA1_Stream3_IRQn,  3 ); /* WLAN gSPI DMA       */
    NVIC_SetPriority( USART1_IRQn      ,  6 ); /* WICED_UART_1        */
    NVIC_SetPriority( USART3_IRQn      ,  6 ); /* Bluetooth UART      */
    NVIC_SetPriority( DMA2_Stream5_IRQn,  6 ); /* WICED_SPI_2_TX      */
    NVIC_SetPriority( DMA2_Stream0_IRQn,  6 ); /* WICED_SPI_2_RX      */
    NVIC_SetPriority( DMA2_Stream7_IRQn,  7 ); /* WICED_UART_1 TX DMA */
    NVIC_SetPriority( DMA2_Stream2_IRQn,  7 ); /* WICED_UART_1 RX DMA */
    NVIC_SetPriority( DMA1_Stream4_IRQn,  7 ); /* Bluetooth TX DMA    */
    NVIC_SetPriority( DMA1_Stream1_IRQn,  7 ); /* Bluetooth RX DMA    */
    NVIC_SetPriority( DMA1_Stream5_IRQn,  7 ); /* WICED_SPI_2_TX      */
    NVIC_SetPriority( DMA1_Stream0_IRQn,  7 ); /* WICED_SPI_2_RX or WICED_I2C_1 RX */
    NVIC_SetPriority( DMA1_Stream7_IRQn,  7 ); /* WICED_I2C_3 TX or WICED_I2C_1 TX */
    NVIC_SetPriority( DMA1_Stream2_IRQn,  7 ); /* WICED_I2C_3 RX      */
    NVIC_SetPriority( DMA2_Stream6_IRQn,  7 ); /* WICED_UART_2 TX DMA */
    NVIC_SetPriority( DMA2_Stream6_IRQn,  7 ); /* WICED_UART_2 RX DMA */
    NVIC_SetPriority( EXTI0_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI1_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI2_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI3_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI4_IRQn       , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI9_5_IRQn     , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI15_10_IRQn   , 14 ); /* GPIO                */
}

void platform_init_external_devices( void )
{
    /* Initialise LEDs and turn off by default */
    platform_gpio_init( &platform_gpio_pins[WICED_LED1], OUTPUT_PUSH_PULL );
    platform_gpio_init( &platform_gpio_pins[WICED_LED2], OUTPUT_PUSH_PULL );
    platform_gpio_output_low( &platform_gpio_pins[WICED_LED1] );
    platform_gpio_output_low( &platform_gpio_pins[WICED_LED2] );

    /* Initialise buttons to input by default */
    platform_gpio_init( &platform_gpio_pins[WICED_BUTTON1], INPUT_PULL_UP );
    platform_gpio_init( &platform_gpio_pins[WICED_BUTTON2], INPUT_PULL_UP );

#ifndef WICED_DISABLE_STDIO
	/* Initialise UART standard I/O */
    platform_stdio_init( &platform_uart_drivers[STDIO_UART], &platform_uart_peripherals[STDIO_UART], &stdio_config );
#endif
}

uint32_t  platform_get_factory_reset_button_time ( uint32_t max_time )
{
    uint32_t button_press_timer = 0;
    int led_state = 0;

    /* Initialise input */
     platform_gpio_init( &platform_gpio_pins[ PLATFORM_FACTORY_RESET_BUTTON_GPIO ], INPUT_PULL_UP );

     while ( (PLATFORM_FACTORY_RESET_PRESSED_STATE == platform_gpio_input_get(&platform_gpio_pins[ PLATFORM_FACTORY_RESET_BUTTON_GPIO ])) )
    {
         /* How long is the "Factory Reset" button being pressed. */
         host_rtos_delay_milliseconds( PLATFORM_FACTORY_RESET_CHECK_PERIOD );

         /* Toggle LED every PLATFORM_FACTORY_RESET_CHECK_PERIOD  */
        if ( led_state == 0 )
        {
            platform_gpio_output_high( &platform_gpio_pins[ PLATFORM_FACTORY_RESET_LED_GPIO ] );
            led_state = 1;
        }
        else
        {
            platform_gpio_output_low( &platform_gpio_pins[ PLATFORM_FACTORY_RESET_LED_GPIO ] );
            led_state = 0;
        }

        button_press_timer += PLATFORM_FACTORY_RESET_CHECK_PERIOD;
        if ((max_time > 0) && (button_press_timer >= max_time))
        {
            break;
        }
    }

     /* turn off the LED */
     if (PLATFORM_FACTORY_RESET_LED_ON_STATE == WICED_ACTIVE_HIGH)
     {
         platform_gpio_output_low( &platform_gpio_pins[ PLATFORM_FACTORY_RESET_LED_GPIO ] );
     }
     else
     {
         platform_gpio_output_high( &platform_gpio_pins[ PLATFORM_FACTORY_RESET_LED_GPIO ] );
     }

    return button_press_timer;
}

/******************************************************
 *           Interrupt Handler Definitions
 ******************************************************/

WWD_RTOS_DEFINE_ISR( usart1_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_1] );
}

WWD_RTOS_DEFINE_ISR( usart1_tx_dma_irq )
{
    platform_uart_tx_dma_irq( &platform_uart_drivers[WICED_UART_1] );
}

WWD_RTOS_DEFINE_ISR( usart1_rx_dma_irq )
{
    platform_uart_rx_dma_irq( &platform_uart_drivers[WICED_UART_1] );
}

WWD_RTOS_DEFINE_ISR( bt_uart_irq )
{
    platform_uart_irq( wiced_bt_uart_driver );
}

WWD_RTOS_DEFINE_ISR( bt_uart_tx_dma_irq )
{
    platform_uart_tx_dma_irq( wiced_bt_uart_driver );
}

WWD_RTOS_DEFINE_ISR( bt_uart_rx_dma_irq )
{
    platform_uart_rx_dma_irq( wiced_bt_uart_driver );
}


/******************************************************
 *            Interrupt Handlers Mapping
 ******************************************************/

/* These DMA assignments can be found STM32F4xx datasheet DMA section */
WWD_RTOS_MAP_ISR( usart1_irq         , USART1_irq       )
WWD_RTOS_MAP_ISR( usart1_tx_dma_irq  , DMA2_Stream7_irq )
WWD_RTOS_MAP_ISR( usart1_rx_dma_irq  , DMA2_Stream2_irq )
WWD_RTOS_MAP_ISR( bt_uart_irq        , USART3_irq       )
WWD_RTOS_MAP_ISR( bt_uart_tx_dma_irq , DMA1_Stream4_irq )
WWD_RTOS_MAP_ISR( bt_uart_rx_dma_irq , DMA1_Stream1_irq )
