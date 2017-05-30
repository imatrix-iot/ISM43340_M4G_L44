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
 * Defines peripherals available for use on ISM43340_M4G_L44 board
 */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif



/*
ISM43340-M4G-L44 platform pin definitions ...
+----------------------------------------------------------------------------------------------------+
|Pin |   Pin Name on           |    Module     | STM32| Peripheral      |    Board     | Peripheral  |
| #  |      Module             |  GPIO Alias   | Port | Available       |  Connection  |   Alias     |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 6  | MODULE_MICRO_JTAG_TDI   | WICED_GPIO_1  | A 15 | JTDI            |              |             |
|    |                         |               |      | SPI3_NSS        |              |             |
|    |                         |               |      | I2S3_WS         |              |             |
|    |                         |               |      | TIM2_CH1_ETR    |              |             |
|    |                         |               |      | SPI1_NSS        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 7  | MODULE_MICRO_JTAG_TDO   | WICED_GPIO_2  | B 3  | JTDO            |              |             |
|    |                         |               |      | TRACESWO        |              |             |
|    |                         |               |      | SPI3_SCK        |              |             |
|    |                         |               |      | I2S3_CK         |              |             |
|    |                         |               |      | TIM2_CH2        |              |             |
|    |                         |               |      | SPI1_SCK        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 8  |MODULE_MICRO_JTAG_TRSTN  | WICED_GPIO_3  | B 4  | NJTRST          |              |             |
|    |                         |               |      | SPI3_MISO       |              |             |
|    |                         |               |      | TIM3_CH1        |              |             |
|    |                         |               |      | SPI1_MISO       |              |             |
|    |                         |               |      | I2S3ext_SD      |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 22 | MODULE_MICRO_UART1_TX   | WICED_GPIO_4  | A 9  | USART1_TX       |              |             |
|    |                         |               |      | TIM1_CH2        |              |             |
|    |                         |               |      | I2C3_SMBA       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 21 | MODULE_MICRO_UART1_RX   | WICED_GPIO_5  | A 10 | USART1_RX       |              |             |
|    |                         |               |      | TIM1_CH3        |              |             |
|    |                         |               |      | OTG_FS_ID       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 18 | MODULE_MICRO_UART1_RTS  | WICED_GPIO_6  | A 11 | USART1_CTS      |              |             |
|    |                         |               |      | CAN1_RX         |              |             |
|    |                         |               |      | TIM1_ETR        |              |             |
|    |                         |               |      | OTG_FS_DP       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 19 | MODULE_MICRO_UART1_CTS  | WICED_GPIO_7  | A 12 | USART1_RTS      |              |             |
|    |                         |               |      | CAN1_TX         |              |             |
|    |                         |               |      | TIM1_ETR        |              |             |
|    |                         |               |      | OTG_FS_DP       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 23 | MODULE_MICRO_GPIO_0     | WICED_GPIO_8  | B 5  | I2C1_SMBA       |     SW1      | WICED_PWM_1 |
|    |                         |               |      | CAN2_RX         |              |             |
|    |                         |               |      | TIM3_CH2        |              |             |
|    |                         |               |      | SPI1_MOSI       |              |             |
|    |                         |               |      | SPI3_MOSI       |              |             |
|    |                         |               |      | I2S3_SD         |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 24 | MODULE_MICRO_GPIO_1     | WICED_GPIO_9  | B 6  | I2C1_SCL        |     SW2      | WICED_PWM_2 |
|    |                         |               |      | TIM4_CH1        |              |             |
|    |                         |               |      | CAN2_TX         |              |             |
|    |                         |               |      | USART1_TX       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 25 | MODULE_MICRO_GPIO_2     | WICED_GPIO_10 | B 7  | I2C1_SDA        |              | WICED_PWM_3 |
|    |                         |               |      | USART1_RX       |              |             |
|    |                         |               |      | TIM4_CH2        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 26 | MODULE_MICRO_GPIO_3     | WICED_GPIO_11 | C 6  | I2S2_MCK        |   RED LED    | WICED_PWM_4 |
|    |                         |               |      | TIM8_CH1        |              |             |
|    |                         |               |      | SDIO_D6         |              |             |
|    |                         |               |      | USART6_TX       |              |             |
|    |                         |               |      | TIM3_CH1        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 27 | MODULE_MICRO_GPIO_4     | WICED_GPIO_12 | C 7  | I2S3_MCK        |  GREEN LED   | WICED_PWM_5 |
|    |                         |               |      | TIM8_CH2        |              |             |
|    |                         |               |      | SDIO_D7         |              |             |
|    |                         |               |      | USART6_RX       |              |             |
|    |                         |               |      | TIM3_CH2        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 13 | MODULE_MICRO_ADC0       | WICED_GPIO_13 | A 3  | USART2_RX       |  THERMISTOR  | WICED_ADC_1 |
|    | SPI_CMD_DATA_RDY        |               |      | TIM5_CH4        |              |             |
|    |                         |               |      | TIM9_CH2        |              |             |
|    |                         |               |      | TIM2_CH4        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC123_IN3      |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 12 | MODULE_MICRO_ADC1       | WICED_GPIO_14 | A 4  | SPI1_NSS        |              | WICED_ADC_2 |
|    | MODULE_MICRO_SPI_CS     |               |      | SPI3_NSS        |              |             |
|    |                         |               |      | USART2_CK       |              |             |
|    |                         |               |      | I2S3_WS         |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|    |                         |               |      | ADC12_IN4       |              |             |
|    |                         |               |      | DAC_OUT1        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 11 | MODULE_MICRO_ADC2       | WICED_GPIO_15 | A 5  | SPI1_SCK        |              | WICED_ADC_3 |
|    | MODULE_MICRO_SPI_CLK    |               |      | TIM2_CH1_ETR    |              |             |
|    |                         |               |      | TIM8_CH1N       |              |             |
|    |                         |               |      | ADC12_IN5       |              |             |
|    |                         |               |      | DAC_OUT2        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 10 | MODULE_MICRO_ADC3       | WICED_GPIO_16 | A 6  | SPI1_MISO       |              | WICED_ADC_4 |
|    | MODULE_MICRO_SPI_MISO   |               |      | TIM8_BKIN       |              |             |
|    |                         |               |      | TIM13_CH1       |              |             |
|    |                         |               |      | TIM3_CH1        |              |             |
|    |                         |               |      | TIM1_BKIN       |              |             |
|    |                         |               |      | ADC12_IN6       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 9  | MODULE_MICRO_ADC4       | WICED_GPIO_17 | A 7  | SPI1_MOSI       |              | WICED_ADC_5 |
|    | MODULE_MICRO_SPI_MOSI   |               |      | TIM8_CH1N       |              |             |
|    |                         |               |      | TIM14_CH1       |              |             |
|    |                         |               |      | TIM3_CH2        |              |             |
|    |                         |               |      | TIM1_CH1N       |              |             |
|    |                         |               |      | ADC12_IN7       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 16 | MODULE_MICRO_WAKEUP     | WICED_GPIO_18 | A 0  | USART2_CTS      |              |             |
|    |                         |               |      | USART4_TX       |              |             |
|    |                         |               |      | TIM2_CH1_ETR    |              |             |
|    |                         |               |      | TIM5_CH1        |              |             |
|    |                         |               |      | TIM8_ETR        |              |             |
|    |                         |               |      | ADC123_IN0      |              |             |
|    |                         |               |      | WKUP            |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 32 | MODULE_MICRO_GPIO13     | WICED_GPIO_19 | B 13 | SPI2_SCK        |              | WICED_PWM_6 |
|    |                         |               |      | I2S2_CK         |              |             |
|    |                         |               |      | USART3_CTS      |              |             |
|    |                         |               |      | TIM1_CH1N       |              |             |
|    |                         |               |      | CAN2_TX         |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 31 | MODULE_MICRO_GPIO14     | WICED_GPIO_20 | B 14 | SPI2_MISO       |              | WICED_PWM_7 |
|    |                         |               |      | TIM1_CH2N       |              |             |
|    |                         |               |      | TIM12_CH1       |              |             |
|    |                         |               |      | OTG_HS_DM       |              |             |
|    |                         |               |      | USART3_RTS      |              |             |
|    |                         |               |      | TIM8_CH2N       |              |             |
|    |                         |               |      | I2S2ext_SD      |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 30 | MODULE_MICRO_GPIO15     | WICED_GPIO_21 | B 15 | SPI2_M0Si       |              | WICED_PWM_8 |
|    |                         |               |      | I2S2_SD         |              |             |
|    |                         |               |      | TIM1_CH3N       |              |             |
|    |                         |               |      | TIM8_CH3N       |              |             |
|    |                         |               |      | TIM12_CH2       |              |             |
|    |                         |               |      | OTG_HS_DP       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
+10 Pins
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 45 | GPIOA                   | WICED_GPIO_22 | A 2  | ADC1_2          |              |             |
|    |                         |               |      | USART2_TX       |              |             |
|    |                         |               |      | TIM5_CH3        |              |             |
|    |                         |               |      | TIM2_CH3        |              |             |
|    |                         |               |      | TIM9_CH1        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 46 | GPIOB                   | WICED_GPIO_23 | A 1  | ADC1_1          |              |             |
|    |                         |               |      | USART2_RTS      |              |             |
|    |                         |               |      | USART4_RX       |              |             |
|    |                         |               |      | TIM5_CH2        |              |             |
|    |                         |               |      | TIM2_CH2        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 47 | GPIOC                   | WICED_GPIO_24 | B 10 | SPI2_SCK        |              |             |
|    |                         |               |      | I2C2_SCL        |              |             |
|    |                         |               |      | USART3_TX       |              |             |
|    |                         |               |      | TIM2_CH3        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 48 | GPIOD                   | WICED_GPIO_25 | C 3  | I2S2_SD         |              |             |
|    |                         |               |      | SPI2_MOSI       |              |             |
|    |                         |               |      | ADC123_IN13     |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 49 | GPIOE                   | WICED_GPIO_26 | B 11 | I2C2SDA         |              |             |
|    |                         |               |      | USART3_RX       |              |             |
|    |                         |               |      | TIM2_CH4        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 50 | GPIOF                   | WICED_GPIO_27 | B 12 | GPIO            |              |             |
|    |                         |               |      | SPI2_NSS        |              |             |
|    |                         |               |      | I2S_WS          |              |             |
|    |                         |               |      | I2C2_SMBA       |              |             |
|    |                         |               |      | USART3_CK       |              |             |
|    |                         |               |      | CAN2_RX         |              |             |
|    |                         |               |      | TIM1_BKIN       |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 51 | GPIOG                   | WICED_GPIO_28 | E 11 | GPIO            |              |             |
|    |                         |               |      | FSMC_D8         |              |             |
|    |                         |               |      | TIM1_CH2        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 52 | GPIOH                   | WICED_GPIO_29 | E 12 | GPIO            |              |             |
|    |                         |               |      | FSMC_D9         |              |             |
|    |                         |               |      | TIM1_CH3N       |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 53 | GPIOJ                   | WICED_GPIO_30 | E 13 | GPIO            |              |             |
|    |                         |               |      | FSMC_D10        |              |             |
|    |                         |               |      | TIM1_CH3        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|
| 54 | GPIOK                   | WICED_GPIO_31 | E 14 | GPIO            |              |             |
|    |                         |               |      | FSMC_D11        |              |             |
|    |                         |               |      | TIM1_CH4        |              |             |
|    |                         |               |      | EVENTOUT        |              |             |
|----+-------------------------+---------------+------+-----------------+--------------+-------------|

Notes
1. The mappings in the table above are defined in <WICED-SDK>/platforms/ISM43340_M4G_L44/platform.c
2. STM32F4xx Datasheet  ->
3. STM32F4xx Ref Manual ->
*/


/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    WICED_GPIO_1,
    WICED_GPIO_2,
    WICED_GPIO_3,
    WICED_GPIO_4,
    WICED_GPIO_5,
    WICED_GPIO_6,
    WICED_GPIO_7,
    WICED_GPIO_8,
    WICED_GPIO_9,
    WICED_GPIO_10,
    WICED_GPIO_11,
    WICED_GPIO_12,
    WICED_GPIO_13,
    WICED_GPIO_14,
    WICED_GPIO_15,
    WICED_GPIO_16,
    WICED_GPIO_17,
    WICED_GPIO_18,
    WICED_GPIO_19,
    WICED_GPIO_20,
    WICED_GPIO_21,
    WICED_GPIO_22,
    WICED_GPIO_23,
    WICED_GPIO_24,
    WICED_GPIO_25,
    WICED_GPIO_26,
    WICED_GPIO_27,
    WICED_GPIO_28,
    WICED_GPIO_29,
    WICED_GPIO_30,
    WICED_GPIO_31,
    WICED_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
    WICED_GPIO_32BIT = 0x7FFFFFFF
} wiced_gpio_t;

typedef enum
{
    WICED_SPI_1, 	/* SPI available */
    WICED_SPI_2,
    WICED_SPI_MAX,  /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
    WICED_SPI_32BIT = 0x7FFFFFFF,
} wiced_spi_t;

typedef enum
{
    WICED_I2C_1, 	/* I2C available */
    WICED_I2C_2,
    WICED_I2C_3,
    WICED_I2C_MAX,  /* Denotes the total number of I2C port aliases. Not a valid I2C alias */
    WICED_I2C_32BIT = 0x7FFFFFFF,
} wiced_i2c_t;

typedef enum
{
    WICED_I2S_NONE, /* I2S unavailable */
    WICED_I2S_MAX,  /* Denotes the total number of I2S port aliases. Not a valid I2S alias */
    WICED_I2S_32BIT = 0x7FFFFFFF,
} wiced_i2s_t;

typedef enum
{
    WICED_PWM_1,
    WICED_PWM_2,
    WICED_PWM_3,
    WICED_PWM_4,
    WICED_PWM_5,
    WICED_PWM_6,
    WICED_PWM_7,
    WICED_PWM_8,
    //WICED_PWM_NONE, /* PWM unavailable */
    WICED_PWM_MAX,  /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
    WICED_PWM_32BIT = 0x7FFFFFFF,
} wiced_pwm_t;

typedef enum
{
//    WICED_ADC_NONE, /* ADC unavailable */
	WICED_ADC_1,
	WICED_ADC_2,
	WICED_ADC_3,
	WICED_ADC_4,
	WICED_ADC_5,
    WICED_ADC_MAX,  /* Denotes the total number of ADC port aliases. Not a valid ADC alias */
    WICED_ADC_32BIT = 0x7FFFFFFF,
} wiced_adc_t;

typedef enum
{
    WICED_UART_1,
    WICED_UART_2,
    WICED_UART_MAX, /* Denotes the total number of UART port aliases. Not a valid UART alias */
    WICED_UART_32BIT = 0x7FFFFFFF,
} wiced_uart_t;

/* Logical Button-ids which map to phyiscal buttons on the board */
typedef enum
{
    PLATFORM_BUTTON_1,
    PLATFORM_BUTTON_2,
    PLATFORM_BUTTON_MAX, /* Denotes the total number of Buttons on the board. Not a valid Button Alias */
} platform_button_t;

/******************************************************
 *                    Constants
 ******************************************************/

#define WICED_PLATFORM_BUTTON_COUNT  ( 2 )

/* UART port used for standard I/O */
#define STDIO_UART                       ( WICED_UART_1 )

/* SPI flash is present */
#define WICED_PLATFORM_INCLUDES_SPI_FLASH

/* SPI Flash. Implemented by Inventek Systems ... */
#ifdef WICED_PLATFORM_INCLUDES_SPI_FLASH
//#define WICED_SPI_FLASH_CS            	( WICED_GPIO_14 ) 		//ADC1
#define WICED_SPI_FLASH_CS            	( WICED_GPIO_8 ) 		//GPIO0
#define WICED_SPI_FLASH_CLK           	( WICED_GPIO_15 )		//ADC2
#define WICED_SPI_FLASH_MISO          	( WICED_GPIO_16 )		//ADC3
#define WICED_SPI_FLASH_MOSI          	( WICED_GPIO_17 )		//ADC4
#endif

/* Components connected to external I/Os */
#define WICED_LED1                     	( WICED_GPIO_11 )		//GPIO3
#define WICED_LED2                      ( WICED_GPIO_12 )		//GPIO4
#define WICED_LED1_ON_STATE             ( WICED_ACTIVE_HIGH )
#define WICED_LED2_ON_STATE             ( WICED_ACTIVE_HIGH )
#define WICED_BUTTON1                   ( WICED_GPIO_8  )		//GPIO0
#define WICED_BUTTON2                   ( WICED_GPIO_9  )		//GPIO1
#define WICED_THERMISTOR                ( WICED_GPIO_13 )		//ADC0

#define WICED_GPIO_AUTH_RST             ( WICED_GPIO_21 )		//GPIO15
#define WICED_GPIO_AUTH_SCL             ( WICED_GPIO_9  )		//GPIO1
#define WICED_GPIO_AUTH_SDA             ( WICED_GPIO_10 )		//GPIO2

/* Authentication Chip <-> I2C Peripheral */
#define WICED_I2C_AUTH                  ( WICED_I2C_1 )
#define WICED_I2C_AUTH_DMA              ( I2C_DEVICE_USE_DMA )

/* I/O connection <-> Peripheral Connections */
/* #define WICED_LED1_JOINS_PWM         ( WICED_PWM_1 ) */
/* #define WICED_LED2_JOINS_PWM         ( WICED_PWM_2 ) */
/* #define WICED_THERMISTOR_JOINS_ADC   ( WICED_ADC_3 ) */

/* Bootloader OTA/OTA2 LED to flash while "Factory Reset" button held           */
 #define PLATFORM_FACTORY_RESET_LED_GPIO        	( WICED_LED1 )
 #define PLATFORM_FACTORY_RESET_LED_ON_STATE        ( WICED_LED1_ON_STATE )

/* Bootloader OTA/OTA2 "Factory Reset" button */
 #define PLATFORM_FACTORY_RESET_BUTTON_GPIO      	( WICED_BUTTON1 )
 #define PLATFORM_FACTORY_RESET_PRESSED_STATE    	(   0  )
 #define PLATFORM_FACTORY_RESET_CHECK_PERIOD     	(  100 )
 #ifndef PLATFORM_FACTORY_RESET_TIMEOUT
  #define PLATFORM_FACTORY_RESET_TIMEOUT          	( 5000 )
 #endif

#ifdef __cplusplus
} /*extern "C" */
#endif
