/*
    ChibiOS - Copyright (C) 2006..2024 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * Board definition for ACS4 custom PCB.
 * MCU: STM32H725VGT6 (LQFP100)
 * HSE: 50 MHz passive crystal (ABM10W-50.0000MHZ)
 * LSE: Not present
 * Available GPIO ports: A, B, C, D, E, H (PH0-PH1 only physical)
 */

#ifndef BOARD_ACS4_H725_H
#define BOARD_ACS4_H725_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for ACS4 custom board with STM32H725VGT6.
 */

/*
 * Board identifier.
 */
#define BOARD_ACS4_H725
#define BOARD_NAME                  "ACS4 STM32H725VGT6"

/*
 * Board oscillators-related settings.
 * NOTE: HSE is a 50 MHz passive crystal, NOT bypass mode.
 * NOTE: LSE is not present on this board.
 */
#if !defined(STM32_HSECLK)
#define STM32_HSECLK                50000000U
#endif

#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#if !defined(STM32_HSE_BYPASS)
#define STM32_HSE_BYPASS            FALSE
#endif

#if !defined(STM32_LSE_BYPASS)
#define STM32_LSE_BYPASS            FALSE
#endif

/*
 * LSE drive strength (required by ChibiOS HAL even when LSE is disabled).
 */
#if !defined(STM32_LSEDRV)
#define STM32_LSEDRV                (3U << 3U)
#endif

/*
 * MCU type as defined in the ST header.
 */
#if !defined(STM32H725xx)
#define STM32H725xx
#endif

/*===========================================================================*/
/* Pin definitions.                                                          */
/*===========================================================================*/

/*
 * GPIOA pin assignments.
 *
 * PA0  - GPIO0                    (input pulldown, spare).
 * PA1  - LED_1                    (output pushpull, low).
 * PA2  - LED_2                    (output pushpull, low).
 * PA3  - LED_3                    (output pushpull, low).
 * PA4  - GPIO4                    (input pulldown, spare).
 * PA5  - GPIO3                    (input pulldown, spare).
 * PA6  - GPIO2                    (input pulldown, spare).
 * PA7  - LED_4                    (output pushpull, low).
 * PA8  - CH6_PWM                  (alternate 1, TIM1_CH1).
 * PA9  - CH5_PWM                  (alternate 1, TIM1_CH2).
 * PA10 - PIN10                    (input pulldown).
 * PA11 - USB_DM                   (alternate 10, OTG_FS).
 * PA12 - USB_DP                   (alternate 10, OTG_FS).
 * PA13 - SWDIO                    (alternate 0).
 * PA14 - SWCLK                    (alternate 0).
 * PA15 - DETECT_SD                (input pullup).
 */
#define GPIOA_GPIO0                 0U
#define GPIOA_LED_1                 1U
#define GPIOA_LED_2                 2U
#define GPIOA_LED_3                 3U
#define GPIOA_GPIO4                 4U
#define GPIOA_GPIO3                 5U
#define GPIOA_GPIO2                 6U
#define GPIOA_LED_4                 7U
#define GPIOA_CH6_PWM               8U
#define GPIOA_CH5_PWM               9U
#define GPIOA_PIN10                 10U
#define GPIOA_USB_DM                11U
#define GPIOA_USB_DP                12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_DETECT_SD             15U

/*
 * GPIOB pin assignments.
 *
 * PB0  - CH2_CONT                 (input floating, pyro continuity digital,
 *                                  ext 1k pullup to 3V3 + LED, HIGH=open).
 * PB1  - ARMING                   (input floating).
 * PB2  - CH1_CONT                 (input floating, pyro continuity digital,
 *                                  ext 1k pullup to 3V3 + LED, HIGH=open).
 * PB3  - SPI6_SCK                 (alternate 8).
 * PB4  - SPI6_MISO                (alternate 8).
 * PB5  - SPI6_MOSI                (alternate 8).
 * PB6  - I2C1_SCL                 (alternate 4, open-drain, pullup).
 * PB7  - I2C1_SDA                 (alternate 4, open-drain, pullup).
 * PB8  - CAN_RX                   (alternate 9, FDCAN1).
 * PB9  - CAN_TX                   (alternate 9, FDCAN1).
 * PB10 - I2C2_SCL                 (alternate 4, open-drain, pullup).
 * PB11 - I2C2_SDA                 (alternate 4, open-drain, pullup).
 * PB12 - MAG_CS                   (output pushpull, high, deselected).
 * PB13 - SPI2_SCK                 (alternate 5).
 * PB14 - SPI2_MISO                (alternate 5).
 * PB15 - SPI2_MOSI                (alternate 5).
 */
#define GPIOB_CH2_CONT              0U
#define GPIOB_ARMING                1U
#define GPIOB_CH1_CONT              2U
#define GPIOB_SPI6_SCK              3U
#define GPIOB_SPI6_MISO             4U
#define GPIOB_SPI6_MOSI             5U
#define GPIOB_I2C1_SCL              6U
#define GPIOB_I2C1_SDA              7U
#define GPIOB_CAN_RX                8U
#define GPIOB_CAN_TX                9U
#define GPIOB_I2C2_SCL              10U
#define GPIOB_I2C2_SDA              11U
#define GPIOB_MAG_CS                12U
#define GPIOB_SPI2_SCK              13U
#define GPIOB_SPI2_MISO             14U
#define GPIOB_SPI2_MOSI             15U

/*
 * GPIOC pin assignments.
 *
 * PC0  - VIN_MES                  (analog, ADC battery voltage).
 * PC1  - PIN1                     (input pulldown).
 * PC2  - PIN2                     (input pulldown).
 * PC3  - PIN3                     (input pulldown).
 * PC4  - GPIO1                    (input pulldown, spare).
 * PC5  - MAG_INT                  (input floating, EXTI).
 * PC6  - USART6_TX                (alternate 7).
 * PC7  - USART6_RX                (alternate 7).
 * PC8  - SDMMC1_D0                (alternate 12, high speed).
 * PC9  - SDMMC1_D1                (alternate 12, high speed).
 * PC10 - SDMMC1_D2                (alternate 12, high speed).
 * PC11 - SDMMC1_D3                (alternate 12, high speed).
 * PC12 - SDMMC1_CLK               (alternate 12, high speed).
 * PC13 - PIN13                    (input pulldown).
 * PC14 - PIN14                    (input pulldown, no LSE).
 * PC15 - PIN15                    (input pulldown, no LSE).
 */
#define GPIOC_VIN_MES               0U
#define GPIOC_PIN1                  1U
#define GPIOC_PIN2                  2U
#define GPIOC_PIN3                  3U
#define GPIOC_GPIO1                 4U
#define GPIOC_MAG_INT               5U
#define GPIOC_USART6_TX             6U
#define GPIOC_USART6_RX             7U
#define GPIOC_SDMMC1_D0             8U
#define GPIOC_SDMMC1_D1             9U
#define GPIOC_SDMMC1_D2             10U
#define GPIOC_SDMMC1_D3             11U
#define GPIOC_SDMMC1_CLK            12U
#define GPIOC_PIN13                 13U
#define GPIOC_PIN14                 14U
#define GPIOC_PIN15                 15U

/*
 * GPIOD pin assignments.
 *
 * PD0  - UART4_RX                 (alternate 8).
 * PD1  - UART4_TX                 (alternate 8).
 * PD2  - SDMMC1_CMD               (alternate 12, high speed).
 * PD3  - LORA_D0                  (input floating).
 * PD4  - LORA_RSS                 (output pushpull, high).
 * PD5  - LORA_CS                  (output pushpull, high, deselected).
 * PD6  - PIN6                     (input pulldown).
 * PD7  - PIN7                     (input pulldown).
 * PD8  - IMU_CS                   (output pushpull, high, deselected).
 * PD9  - IMU_INT2                 (input floating, EXTI).
 * PD10 - IMU_INT1                 (input floating, EXTI).
 * PD11 - BARO_CS                  (output pushpull, high, deselected).
 * PD12 - CH4_PWM                  (alternate 2, TIM4_CH1).
 * PD13 - CH3_PWM                  (alternate 2, TIM4_CH2).
 * PD14 - CH2_PWM                  (alternate 2, TIM4_CH3).
 * PD15 - CH1_PWM                  (alternate 2, TIM4_CH4).
 */
#define GPIOD_UART4_RX              0U
#define GPIOD_UART4_TX              1U
#define GPIOD_SDMMC1_CMD            2U
#define GPIOD_LORA_D0               3U
#define GPIOD_LORA_RSS              4U
#define GPIOD_LORA_CS               5U
#define GPIOD_PIN6                  6U
#define GPIOD_PIN7                  7U
#define GPIOD_IMU_CS                8U
#define GPIOD_IMU_INT2              9U
#define GPIOD_IMU_INT1              10U
#define GPIOD_BARO_CS               11U
#define GPIOD_CH4_PWM               12U
#define GPIOD_CH3_PWM               13U
#define GPIOD_CH2_PWM               14U
#define GPIOD_CH1_PWM               15U

/*
 * GPIOE pin assignments.
 *
 * PE0  - PIN0                     (input pulldown).
 * PE1  - PIN1                     (input pulldown).
 * PE2  - PIN2                     (input pulldown).
 * PE3  - PIN3                     (input pulldown).
 * PE4  - PIN4                     (input pulldown).
 * PE5  - BUZZER                   (output pushpull, low).
 * PE6  - PIN6                     (input pulldown).
 * PE7  - CH1_FIRE                 (output pushpull, low, pyro OFF).
 * PE8  - CH2_FIRE                 (output pushpull, low, pyro OFF).
 * PE9  - PIN9                     (input pulldown).
 * PE10 - PIN10                    (input pulldown).
 * PE11 - PIN11                    (input pulldown).
 * PE12 - PIN12                    (input pulldown).
 * PE13 - PIN13                    (input pulldown).
 * PE14 - PIN14                    (input pulldown).
 * PE15 - PIN15                    (input pulldown).
 */
#define GPIOE_PIN0                  0U
#define GPIOE_PIN1                  1U
#define GPIOE_PIN2                  2U
#define GPIOE_PIN3                  3U
#define GPIOE_PIN4                  4U
#define GPIOE_BUZZER                5U
#define GPIOE_PIN6                  6U
#define GPIOE_CH1_FIRE              7U
#define GPIOE_CH2_FIRE              8U
#define GPIOE_PIN9                  9U
#define GPIOE_PIN10                 10U
#define GPIOE_PIN11                 11U
#define GPIOE_PIN12                 12U
#define GPIOE_PIN13                 13U
#define GPIOE_PIN14                 14U
#define GPIOE_PIN15                 15U

/*
 * GPIOH pin assignments.
 * NOTE: On LQFP100 only PH0 and PH1 exist physically (HSE oscillator).
 *       PH2-PH15 are defined for register completeness.
 *
 * PH0  - OSC_IN                   (input floating, HSE).
 * PH1  - OSC_OUT                  (input floating, HSE).
 * PH2  - PIN2                     (input pulldown).
 * PH3  - PIN3                     (input pulldown).
 * PH4  - PIN4                     (input pulldown).
 * PH5  - PIN5                     (input pulldown).
 * PH6  - PIN6                     (input pulldown).
 * PH7  - PIN7                     (input pulldown).
 * PH8  - PIN8                     (input pulldown).
 * PH9  - PIN9                     (input pulldown).
 * PH10 - PIN10                    (input pulldown).
 * PH11 - PIN11                    (input pulldown).
 * PH12 - PIN12                    (input pulldown).
 * PH13 - PIN13                    (input pulldown).
 * PH14 - PIN14                    (input pulldown).
 * PH15 - PIN15                    (input pulldown).
 */
#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

/*===========================================================================*/
/* IO lines assignments.                                                     */
/*===========================================================================*/

/* GPIOA lines */
#define LINE_GPIO0                  PAL_LINE(GPIOA, GPIOA_GPIO0)
#define LINE_LED_1                  PAL_LINE(GPIOA, GPIOA_LED_1)
#define LINE_LED_2                  PAL_LINE(GPIOA, GPIOA_LED_2)
#define LINE_LED_3                  PAL_LINE(GPIOA, GPIOA_LED_3)
#define LINE_GPIO4                  PAL_LINE(GPIOA, GPIOA_GPIO4)
#define LINE_GPIO3                  PAL_LINE(GPIOA, GPIOA_GPIO3)
#define LINE_GPIO2                  PAL_LINE(GPIOA, GPIOA_GPIO2)
#define LINE_LED_4                  PAL_LINE(GPIOA, GPIOA_LED_4)
#define LINE_CH6_PWM                PAL_LINE(GPIOA, GPIOA_CH6_PWM)
#define LINE_CH5_PWM                PAL_LINE(GPIOA, GPIOA_CH5_PWM)
#define LINE_USB_DM                 PAL_LINE(GPIOA, GPIOA_USB_DM)
#define LINE_USB_DP                 PAL_LINE(GPIOA, GPIOA_USB_DP)
#define LINE_SWDIO                  PAL_LINE(GPIOA, GPIOA_SWDIO)
#define LINE_SWCLK                  PAL_LINE(GPIOA, GPIOA_SWCLK)
#define LINE_DETECT_SD              PAL_LINE(GPIOA, GPIOA_DETECT_SD)

/* GPIOB lines */
#define LINE_CH2_CONT               PAL_LINE(GPIOB, GPIOB_CH2_CONT)
#define LINE_ARMING                 PAL_LINE(GPIOB, GPIOB_ARMING)
#define LINE_CH1_CONT               PAL_LINE(GPIOB, GPIOB_CH1_CONT)
#define LINE_SPI6_SCK               PAL_LINE(GPIOB, GPIOB_SPI6_SCK)
#define LINE_SPI6_MISO              PAL_LINE(GPIOB, GPIOB_SPI6_MISO)
#define LINE_SPI6_MOSI              PAL_LINE(GPIOB, GPIOB_SPI6_MOSI)
#define LINE_I2C1_SCL               PAL_LINE(GPIOB, GPIOB_I2C1_SCL)
#define LINE_I2C1_SDA               PAL_LINE(GPIOB, GPIOB_I2C1_SDA)
#define LINE_CAN_RX                 PAL_LINE(GPIOB, GPIOB_CAN_RX)
#define LINE_CAN_TX                 PAL_LINE(GPIOB, GPIOB_CAN_TX)
#define LINE_I2C2_SCL               PAL_LINE(GPIOB, GPIOB_I2C2_SCL)
#define LINE_I2C2_SDA               PAL_LINE(GPIOB, GPIOB_I2C2_SDA)
#define LINE_MAG_CS                 PAL_LINE(GPIOB, GPIOB_MAG_CS)
#define LINE_SPI2_SCK               PAL_LINE(GPIOB, GPIOB_SPI2_SCK)
#define LINE_SPI2_MISO              PAL_LINE(GPIOB, GPIOB_SPI2_MISO)
#define LINE_SPI2_MOSI              PAL_LINE(GPIOB, GPIOB_SPI2_MOSI)

/* GPIOC lines */
#define LINE_VIN_MES                PAL_LINE(GPIOC, GPIOC_VIN_MES)
#define LINE_GPIO1                  PAL_LINE(GPIOC, GPIOC_GPIO1)
#define LINE_MAG_INT                PAL_LINE(GPIOC, GPIOC_MAG_INT)
#define LINE_USART6_TX              PAL_LINE(GPIOC, GPIOC_USART6_TX)
#define LINE_USART6_RX              PAL_LINE(GPIOC, GPIOC_USART6_RX)
#define LINE_SDMMC1_D0              PAL_LINE(GPIOC, GPIOC_SDMMC1_D0)
#define LINE_SDMMC1_D1              PAL_LINE(GPIOC, GPIOC_SDMMC1_D1)
#define LINE_SDMMC1_D2              PAL_LINE(GPIOC, GPIOC_SDMMC1_D2)
#define LINE_SDMMC1_D3              PAL_LINE(GPIOC, GPIOC_SDMMC1_D3)
#define LINE_SDMMC1_CLK             PAL_LINE(GPIOC, GPIOC_SDMMC1_CLK)

/* GPIOD lines */
#define LINE_UART4_RX               PAL_LINE(GPIOD, GPIOD_UART4_RX)
#define LINE_UART4_TX               PAL_LINE(GPIOD, GPIOD_UART4_TX)
#define LINE_SDMMC1_CMD             PAL_LINE(GPIOD, GPIOD_SDMMC1_CMD)
#define LINE_LORA_D0                PAL_LINE(GPIOD, GPIOD_LORA_D0)
#define LINE_LORA_RSS               PAL_LINE(GPIOD, GPIOD_LORA_RSS)
#define LINE_LORA_CS                PAL_LINE(GPIOD, GPIOD_LORA_CS)
#define LINE_IMU_CS                 PAL_LINE(GPIOD, GPIOD_IMU_CS)
#define LINE_IMU_INT2               PAL_LINE(GPIOD, GPIOD_IMU_INT2)
#define LINE_IMU_INT1               PAL_LINE(GPIOD, GPIOD_IMU_INT1)
#define LINE_BARO_CS                PAL_LINE(GPIOD, GPIOD_BARO_CS)
#define LINE_CH4_PWM                PAL_LINE(GPIOD, GPIOD_CH4_PWM)
#define LINE_CH3_PWM                PAL_LINE(GPIOD, GPIOD_CH3_PWM)
#define LINE_CH2_PWM                PAL_LINE(GPIOD, GPIOD_CH2_PWM)
#define LINE_CH1_PWM                PAL_LINE(GPIOD, GPIOD_CH1_PWM)

/* GPIOE lines */
#define LINE_BUZZER                 PAL_LINE(GPIOE, GPIOE_BUZZER)
#define LINE_CH1_FIRE               PAL_LINE(GPIOE, GPIOE_CH1_FIRE)
#define LINE_CH2_FIRE               PAL_LINE(GPIOE, GPIOE_CH2_FIRE)

/* GPIOH lines */
#define LINE_OSC_IN                 PAL_LINE(GPIOH, GPIOH_OSC_IN)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, GPIOH_OSC_OUT)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - GPIO0                    (input pulldown, spare).
 * PA1  - LED_1                    (output pushpull, verylow speed, low).
 * PA2  - LED_2                    (output pushpull, verylow speed, low).
 * PA3  - LED_3                    (output pushpull, verylow speed, low).
 * PA4  - GPIO4                    (input pulldown, spare).
 * PA5  - GPIO3                    (input pulldown, spare).
 * PA6  - GPIO2                    (input pulldown, spare).
 * PA7  - LED_4                    (output pushpull, verylow speed, low).
 * PA8  - CH6_PWM                  (alternate 1, pushpull, high speed).
 * PA9  - CH5_PWM                  (alternate 1, pushpull, high speed).
 * PA10 - PIN10                    (input pulldown).
 * PA11 - USB_DM                   (alternate 10, pushpull, high speed).
 * PA12 - USB_DP                   (alternate 10, pushpull, high speed).
 * PA13 - SWDIO                    (alternate 0, pullup).
 * PA14 - SWCLK                    (alternate 0, pulldown).
 * PA15 - DETECT_SD                (input pullup).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_GPIO0) |          \
                                     PIN_MODE_OUTPUT(GPIOA_LED_1) |         \
                                     PIN_MODE_OUTPUT(GPIOA_LED_2) |         \
                                     PIN_MODE_OUTPUT(GPIOA_LED_3) |         \
                                     PIN_MODE_INPUT(GPIOA_GPIO4) |          \
                                     PIN_MODE_INPUT(GPIOA_GPIO3) |          \
                                     PIN_MODE_INPUT(GPIOA_GPIO2) |          \
                                     PIN_MODE_OUTPUT(GPIOA_LED_4) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_CH6_PWM) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_CH5_PWM) |    \
                                     PIN_MODE_INPUT(GPIOA_PIN10) |          \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_DM) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_DP) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_INPUT(GPIOA_DETECT_SD))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_GPIO0) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_2) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_GPIO4) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_GPIO3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_GPIO2) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_4) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CH6_PWM) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CH5_PWM) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DM) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DP) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_DETECT_SD))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOA_GPIO0) |      \
                                     PIN_OSPEED_VERYLOW(GPIOA_LED_1) |      \
                                     PIN_OSPEED_VERYLOW(GPIOA_LED_2) |      \
                                     PIN_OSPEED_VERYLOW(GPIOA_LED_3) |      \
                                     PIN_OSPEED_VERYLOW(GPIOA_GPIO4) |      \
                                     PIN_OSPEED_VERYLOW(GPIOA_GPIO3) |      \
                                     PIN_OSPEED_VERYLOW(GPIOA_GPIO2) |      \
                                     PIN_OSPEED_VERYLOW(GPIOA_LED_4) |      \
                                     PIN_OSPEED_HIGH(GPIOA_CH6_PWM) |       \
                                     PIN_OSPEED_HIGH(GPIOA_CH5_PWM) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN10) |      \
                                     PIN_OSPEED_HIGH(GPIOA_USB_DM) |        \
                                     PIN_OSPEED_HIGH(GPIOA_USB_DP) |        \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_VERYLOW(GPIOA_DETECT_SD))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOA_GPIO0) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_1) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_2) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_3) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOA_GPIO4) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOA_GPIO3) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOA_GPIO2) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_4) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_CH6_PWM) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_CH5_PWM) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DM) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DP) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_DETECT_SD))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_GPIO0) |             \
                                     PIN_ODR_LOW(GPIOA_LED_1) |             \
                                     PIN_ODR_LOW(GPIOA_LED_2) |             \
                                     PIN_ODR_LOW(GPIOA_LED_3) |             \
                                     PIN_ODR_LOW(GPIOA_GPIO4) |             \
                                     PIN_ODR_LOW(GPIOA_GPIO3) |             \
                                     PIN_ODR_LOW(GPIOA_GPIO2) |             \
                                     PIN_ODR_LOW(GPIOA_LED_4) |             \
                                     PIN_ODR_LOW(GPIOA_CH6_PWM) |           \
                                     PIN_ODR_LOW(GPIOA_CH5_PWM) |           \
                                     PIN_ODR_LOW(GPIOA_PIN10) |             \
                                     PIN_ODR_LOW(GPIOA_USB_DM) |            \
                                     PIN_ODR_LOW(GPIOA_USB_DP) |            \
                                     PIN_ODR_LOW(GPIOA_SWDIO) |             \
                                     PIN_ODR_LOW(GPIOA_SWCLK) |             \
                                     PIN_ODR_LOW(GPIOA_DETECT_SD))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_GPIO0, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_LED_1, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_LED_2, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_LED_3, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_GPIO4, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_GPIO3, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_GPIO2, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_LED_4, 0U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_CH6_PWM, 1U) |      \
                                     PIN_AFIO_AF(GPIOA_CH5_PWM, 1U) |      \
                                     PIN_AFIO_AF(GPIOA_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_USB_DM, 10U) |       \
                                     PIN_AFIO_AF(GPIOA_USB_DP, 10U) |       \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_DETECT_SD, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - CH2_CONT                 (input floating, pyro continuity digital,
 *                                  ext 1k pullup to 3V3 + LED, HIGH=open).
 * PB1  - ARMING                   (input floating).
 * PB2  - CH1_CONT                 (input floating, pyro continuity digital,
 *                                  ext 1k pullup to 3V3 + LED, HIGH=open).
 * PB3  - SPI6_SCK                 (alternate 8, pushpull, high speed).
 * PB4  - SPI6_MISO                (alternate 8, pushpull, high speed).
 * PB5  - SPI6_MOSI                (alternate 8, pushpull, high speed).
 * PB6  - I2C1_SCL                 (alternate 4, open-drain, high speed, pullup).
 * PB7  - I2C1_SDA                 (alternate 4, open-drain, high speed, pullup).
 * PB8  - CAN_RX                   (alternate 9, pushpull, high speed).
 * PB9  - CAN_TX                   (alternate 9, pushpull, high speed).
 * PB10 - I2C2_SCL                 (alternate 4, open-drain, high speed, pullup).
 * PB11 - I2C2_SDA                 (alternate 4, open-drain, high speed, pullup).
 * PB12 - MAG_CS                   (output pushpull, high speed, high).
 * PB13 - SPI2_SCK                 (alternate 5, pushpull, high speed).
 * PB14 - SPI2_MISO                (alternate 5, pushpull, high speed).
 * PB15 - SPI2_MOSI                (alternate 5, pushpull, high speed).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_CH2_CONT) |       \
                                     PIN_MODE_INPUT(GPIOB_ARMING) |          \
                                     PIN_MODE_INPUT(GPIOB_CH1_CONT) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI6_SCK) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI6_MISO) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI6_MOSI) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_RX) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_TX) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C2_SCL) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C2_SDA) |    \
                                     PIN_MODE_OUTPUT(GPIOB_MAG_CS) |         \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_SCK) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MISO) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_CH1_CONT) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARMING) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CH2_CONT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI6_SCK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI6_MISO) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI6_MOSI) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_RX) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_TX) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SCL) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MAG_CS) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_SCK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MISO) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOB_CH2_CONT) |   \
                                     PIN_OSPEED_VERYLOW(GPIOB_ARMING) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_CH1_CONT) |    \
                                     PIN_OSPEED_HIGH(GPIOB_SPI6_SCK) |       \
                                     PIN_OSPEED_HIGH(GPIOB_SPI6_MISO) |      \
                                     PIN_OSPEED_HIGH(GPIOB_SPI6_MOSI) |      \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCL) |       \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA) |       \
                                     PIN_OSPEED_HIGH(GPIOB_CAN_RX) |         \
                                     PIN_OSPEED_HIGH(GPIOB_CAN_TX) |         \
                                     PIN_OSPEED_HIGH(GPIOB_I2C2_SCL) |       \
                                     PIN_OSPEED_HIGH(GPIOB_I2C2_SDA) |       \
                                     PIN_OSPEED_HIGH(GPIOB_MAG_CS) |         \
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_SCK) |       \
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_MISO) |      \
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_CH2_CONT) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_ARMING) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_CH1_CONT) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI6_SCK) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI6_MISO) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI6_MOSI) |   \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C1_SCL) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C1_SDA) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_RX) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_TX) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C2_SCL) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C2_SDA) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_MAG_CS) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_SCK) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_MISO) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_CH2_CONT) |          \
                                     PIN_ODR_LOW(GPIOB_ARMING) |             \
                                     PIN_ODR_LOW(GPIOB_CH1_CONT) |           \
                                     PIN_ODR_LOW(GPIOB_SPI6_SCK) |           \
                                     PIN_ODR_LOW(GPIOB_SPI6_MISO) |          \
                                     PIN_ODR_LOW(GPIOB_SPI6_MOSI) |          \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) |          \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) |          \
                                     PIN_ODR_LOW(GPIOB_CAN_RX) |             \
                                     PIN_ODR_LOW(GPIOB_CAN_TX) |             \
                                     PIN_ODR_HIGH(GPIOB_I2C2_SCL) |          \
                                     PIN_ODR_HIGH(GPIOB_I2C2_SDA) |          \
                                     PIN_ODR_HIGH(GPIOB_MAG_CS) |            \
                                     PIN_ODR_LOW(GPIOB_SPI2_SCK) |           \
                                     PIN_ODR_LOW(GPIOB_SPI2_MISO) |          \
                                     PIN_ODR_LOW(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_CH2_CONT, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_ARMING, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_CH1_CONT, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_SPI6_SCK, 8U) |       \
                                     PIN_AFIO_AF(GPIOB_SPI6_MISO, 8U) |      \
                                     PIN_AFIO_AF(GPIOB_SPI6_MOSI, 8U) |      \
                                     PIN_AFIO_AF(GPIOB_I2C1_SCL, 4U) |       \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 4U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_CAN_RX, 9U) |        \
                                     PIN_AFIO_AF(GPIOB_CAN_TX, 9U) |         \
                                     PIN_AFIO_AF(GPIOB_I2C2_SCL, 4U) |       \
                                     PIN_AFIO_AF(GPIOB_I2C2_SDA, 4U) |       \
                                     PIN_AFIO_AF(GPIOB_MAG_CS, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_SPI2_SCK, 5U) |       \
                                     PIN_AFIO_AF(GPIOB_SPI2_MISO, 5U) |      \
                                     PIN_AFIO_AF(GPIOB_SPI2_MOSI, 5U))

/*
 * GPIOC setup:
 *
 * PC0  - VIN_MES                  (analog, ADC battery voltage).
 * PC1  - PIN1                     (input pulldown).
 * PC2  - PIN2                     (input pulldown).
 * PC3  - PIN3                     (input pulldown).
 * PC4  - GPIO1                    (input pulldown, spare).
 * PC5  - MAG_INT                  (input floating, EXTI).
 * PC6  - USART6_TX                (alternate 7, pushpull, high speed).
 * PC7  - USART6_RX                (alternate 7, pushpull, high speed).
 * PC8  - SDMMC1_D0                (alternate 12, pushpull, high speed, pullup).
 * PC9  - SDMMC1_D1                (alternate 12, pushpull, high speed, pullup).
 * PC10 - SDMMC1_D2                (alternate 12, pushpull, high speed, pullup).
 * PC11 - SDMMC1_D3                (alternate 12, pushpull, high speed, pullup).
 * PC12 - SDMMC1_CLK               (alternate 12, pushpull, high speed).
 * PC13 - PIN13                    (input pulldown).
 * PC14 - PIN14                    (input pulldown, no LSE).
 * PC15 - PIN15                    (input pulldown, no LSE).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_VIN_MES) |        \
                                     PIN_MODE_INPUT(GPIOC_PIN1) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN2) |             \
                                     PIN_MODE_INPUT(GPIOC_PIN3) |             \
                                     PIN_MODE_INPUT(GPIOC_GPIO1) |            \
                                     PIN_MODE_INPUT(GPIOC_MAG_INT) |          \
                                     PIN_MODE_ALTERNATE(GPIOC_USART6_TX) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_USART6_RX) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_SDMMC1_D0) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_SDMMC1_D1) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_SDMMC1_D2) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_SDMMC1_D3) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_SDMMC1_CLK) |   \
                                     PIN_MODE_INPUT(GPIOC_PIN13) |            \
                                     PIN_MODE_INPUT(GPIOC_PIN14) |            \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_VIN_MES) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_GPIO1) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MAG_INT) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART6_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART6_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDMMC1_D0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDMMC1_D1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDMMC1_D2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDMMC1_D3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDMMC1_CLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOC_VIN_MES) |     \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN1) |         \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN2) |         \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN3) |         \
                                     PIN_OSPEED_VERYLOW(GPIOC_GPIO1) |        \
                                     PIN_OSPEED_VERYLOW(GPIOC_MAG_INT) |      \
                                     PIN_OSPEED_HIGH(GPIOC_USART6_TX) |       \
                                     PIN_OSPEED_HIGH(GPIOC_USART6_RX) |       \
                                     PIN_OSPEED_HIGH(GPIOC_SDMMC1_D0) |       \
                                     PIN_OSPEED_HIGH(GPIOC_SDMMC1_D1) |       \
                                     PIN_OSPEED_HIGH(GPIOC_SDMMC1_D2) |       \
                                     PIN_OSPEED_HIGH(GPIOC_SDMMC1_D3) |       \
                                     PIN_OSPEED_HIGH(GPIOC_SDMMC1_CLK) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN13) |        \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN14) |        \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_VIN_MES) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN1) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN2) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN3) |         \
                                     PIN_PUPDR_PULLDOWN(GPIOC_GPIO1) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_MAG_INT) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_USART6_TX) |    \
                                     PIN_PUPDR_FLOATING(GPIOC_USART6_RX) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_SDMMC1_D0) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_SDMMC1_D1) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_SDMMC1_D2) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_SDMMC1_D3) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_SDMMC1_CLK) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN13) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN14) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_VIN_MES) |            \
                                     PIN_ODR_LOW(GPIOC_PIN1) |               \
                                     PIN_ODR_LOW(GPIOC_PIN2) |               \
                                     PIN_ODR_LOW(GPIOC_PIN3) |               \
                                     PIN_ODR_LOW(GPIOC_GPIO1) |              \
                                     PIN_ODR_LOW(GPIOC_MAG_INT) |            \
                                     PIN_ODR_LOW(GPIOC_USART6_TX) |          \
                                     PIN_ODR_LOW(GPIOC_USART6_RX) |          \
                                     PIN_ODR_LOW(GPIOC_SDMMC1_D0) |          \
                                     PIN_ODR_LOW(GPIOC_SDMMC1_D1) |          \
                                     PIN_ODR_LOW(GPIOC_SDMMC1_D2) |          \
                                     PIN_ODR_LOW(GPIOC_SDMMC1_D3) |          \
                                     PIN_ODR_LOW(GPIOC_SDMMC1_CLK) |         \
                                     PIN_ODR_LOW(GPIOC_PIN13) |              \
                                     PIN_ODR_LOW(GPIOC_PIN14) |              \
                                     PIN_ODR_LOW(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_VIN_MES, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_PIN1, 0U) |            \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0U) |            \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0U) |            \
                                     PIN_AFIO_AF(GPIOC_GPIO1, 0U) |           \
                                     PIN_AFIO_AF(GPIOC_MAG_INT, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_USART6_TX, 7U) |       \
                                     PIN_AFIO_AF(GPIOC_USART6_RX, 7U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SDMMC1_D0, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_SDMMC1_D1, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_SDMMC1_D2, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_SDMMC1_D3, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_SDMMC1_CLK, 12U) |    \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0U) |           \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0U) |           \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - UART4_RX                 (alternate 8, pushpull, high speed).
 * PD1  - UART4_TX                 (alternate 8, pushpull, high speed).
 * PD2  - SDMMC1_CMD               (alternate 12, pushpull, high speed, pullup).
 * PD3  - LORA_D0                  (input floating).
 * PD4  - LORA_RSS                 (output pushpull, verylow speed, high).
 * PD5  - LORA_CS                  (output pushpull, high speed, high).
 * PD6  - PIN6                     (input pulldown).
 * PD7  - PIN7                     (input pulldown).
 * PD8  - IMU_CS                   (output pushpull, high speed, high).
 * PD9  - IMU_INT2                 (input floating, EXTI).
 * PD10 - IMU_INT1                 (input floating, EXTI).
 * PD11 - BARO_CS                  (output pushpull, high speed, high).
 * PD12 - CH4_PWM                  (alternate 2, pushpull, high speed).
 * PD13 - CH3_PWM                  (alternate 2, pushpull, high speed).
 * PD14 - CH2_PWM                  (alternate 2, pushpull, high speed).
 * PD15 - CH1_PWM                  (alternate 2, pushpull, high speed).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_UART4_RX) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_UART4_TX) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_SDMMC1_CMD) |  \
                                     PIN_MODE_INPUT(GPIOD_LORA_D0) |          \
                                     PIN_MODE_OUTPUT(GPIOD_LORA_RSS) |       \
                                     PIN_MODE_OUTPUT(GPIOD_LORA_CS) |         \
                                     PIN_MODE_INPUT(GPIOD_PIN6) |             \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |             \
                                     PIN_MODE_OUTPUT(GPIOD_IMU_CS) |          \
                                     PIN_MODE_INPUT(GPIOD_IMU_INT2) |         \
                                     PIN_MODE_INPUT(GPIOD_IMU_INT1) |         \
                                     PIN_MODE_OUTPUT(GPIOD_BARO_CS) |         \
                                     PIN_MODE_ALTERNATE(GPIOD_CH4_PWM) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_CH3_PWM) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_CH2_PWM) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_CH1_PWM))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_UART4_RX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_UART4_TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDMMC1_CMD) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LORA_D0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LORA_RSS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LORA_CS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOD_IMU_CS) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_IMU_INT2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_IMU_INT1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_BARO_CS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CH4_PWM) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CH3_PWM) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CH2_PWM) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CH1_PWM))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_UART4_RX) |      \
                                     PIN_OSPEED_HIGH(GPIOD_UART4_TX) |      \
                                     PIN_OSPEED_HIGH(GPIOD_SDMMC1_CMD) |    \
                                     PIN_OSPEED_VERYLOW(GPIOD_LORA_D0) |     \
                                     PIN_OSPEED_VERYLOW(GPIOD_LORA_RSS) |    \
                                     PIN_OSPEED_HIGH(GPIOD_LORA_CS) |        \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN6) |        \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN7) |        \
                                     PIN_OSPEED_HIGH(GPIOD_IMU_CS) |         \
                                     PIN_OSPEED_VERYLOW(GPIOD_IMU_INT2) |    \
                                     PIN_OSPEED_VERYLOW(GPIOD_IMU_INT1) |    \
                                     PIN_OSPEED_HIGH(GPIOD_BARO_CS) |        \
                                     PIN_OSPEED_HIGH(GPIOD_CH4_PWM) |        \
                                     PIN_OSPEED_HIGH(GPIOD_CH3_PWM) |        \
                                     PIN_OSPEED_HIGH(GPIOD_CH2_PWM) |        \
                                     PIN_OSPEED_HIGH(GPIOD_CH1_PWM))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_UART4_RX) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_UART4_TX) |   \
                                     PIN_PUPDR_PULLUP(GPIOD_SDMMC1_CMD) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_LORA_D0) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_LORA_RSS) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_LORA_CS) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN6) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN7) |        \
                                     PIN_PUPDR_FLOATING(GPIOD_IMU_CS) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_IMU_INT2) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_IMU_INT1) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_BARO_CS) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_CH4_PWM) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_CH3_PWM) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_CH2_PWM) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_CH1_PWM))
#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_UART4_RX) |          \
                                     PIN_ODR_LOW(GPIOD_UART4_TX) |          \
                                     PIN_ODR_LOW(GPIOD_SDMMC1_CMD) |        \
                                     PIN_ODR_LOW(GPIOD_LORA_D0) |            \
                                     PIN_ODR_HIGH(GPIOD_LORA_RSS) |          \
                                     PIN_ODR_HIGH(GPIOD_LORA_CS) |           \
                                     PIN_ODR_LOW(GPIOD_PIN6) |               \
                                     PIN_ODR_LOW(GPIOD_PIN7) |               \
                                     PIN_ODR_HIGH(GPIOD_IMU_CS) |            \
                                     PIN_ODR_LOW(GPIOD_IMU_INT2) |           \
                                     PIN_ODR_LOW(GPIOD_IMU_INT1) |           \
                                     PIN_ODR_HIGH(GPIOD_BARO_CS) |           \
                                     PIN_ODR_LOW(GPIOD_CH4_PWM) |            \
                                     PIN_ODR_LOW(GPIOD_CH3_PWM) |            \
                                     PIN_ODR_LOW(GPIOD_CH2_PWM) |            \
                                     PIN_ODR_LOW(GPIOD_CH1_PWM))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_UART4_RX, 8U) |      \
                                     PIN_AFIO_AF(GPIOD_UART4_TX, 8U) |      \
                                     PIN_AFIO_AF(GPIOD_SDMMC1_CMD, 12U) |   \
                                     PIN_AFIO_AF(GPIOD_LORA_D0, 0U) |        \
                                     PIN_AFIO_AF(GPIOD_LORA_RSS, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_LORA_CS, 0U) |        \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0U) |           \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_IMU_CS, 0U) |        \
                                     PIN_AFIO_AF(GPIOD_IMU_INT2, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_IMU_INT1, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_BARO_CS, 0U) |        \
                                     PIN_AFIO_AF(GPIOD_CH4_PWM, 2U) |        \
                                     PIN_AFIO_AF(GPIOD_CH3_PWM, 2U) |        \
                                     PIN_AFIO_AF(GPIOD_CH2_PWM, 2U) |        \
                                     PIN_AFIO_AF(GPIOD_CH1_PWM, 2U))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                     (input pulldown).
 * PE1  - PIN1                     (input pulldown).
 * PE2  - PIN2                     (input pulldown).
 * PE3  - PIN3                     (input pulldown).
 * PE4  - PIN4                     (input pulldown).
 * PE5  - BUZZER                   (output pushpull, verylow speed, low).
 * PE6  - PIN6                     (input pulldown).
 * PE7  - CH1_FIRE                 (output pushpull, verylow speed, low = pyro OFF).
 * PE8  - CH2_FIRE                 (output pushpull, verylow speed, low = pyro OFF).
 * PE9  - PIN9                     (input pulldown).
 * PE10 - PIN10                    (input pulldown).
 * PE11 - PIN11                    (input pulldown).
 * PE12 - PIN12                    (input pulldown).
 * PE13 - PIN13                    (input pulldown).
 * PE14 - PIN14                    (input pulldown).
 * PE15 - PIN15                    (input pulldown).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN1) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN2) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN3) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN4) |            \
                                     PIN_MODE_OUTPUT(GPIOE_BUZZER) |         \
                                     PIN_MODE_INPUT(GPIOE_PIN6) |            \
                                     PIN_MODE_OUTPUT(GPIOE_CH1_FIRE) |       \
                                     PIN_MODE_OUTPUT(GPIOE_CH2_FIRE) |       \
                                     PIN_MODE_INPUT(GPIOE_PIN9) |            \
                                     PIN_MODE_INPUT(GPIOE_PIN10) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN11) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN12) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN13) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN14) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_BUZZER) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_CH1_FIRE) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_CH2_FIRE) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOE_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_BUZZER) |     \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_CH1_FIRE) |   \
                                     PIN_OSPEED_VERYLOW(GPIOE_CH2_FIRE) |   \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOE_PIN0) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN1) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN2) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN3) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_BUZZER) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_CH1_FIRE) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_CH2_FIRE) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN9) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN10) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN11) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN12) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN13) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN14) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_LOW(GPIOE_PIN0) |              \
                                     PIN_ODR_LOW(GPIOE_PIN1) |              \
                                     PIN_ODR_LOW(GPIOE_PIN2) |              \
                                     PIN_ODR_LOW(GPIOE_PIN3) |              \
                                     PIN_ODR_LOW(GPIOE_PIN4) |              \
                                     PIN_ODR_LOW(GPIOE_BUZZER) |            \
                                     PIN_ODR_LOW(GPIOE_PIN6) |              \
                                     PIN_ODR_LOW(GPIOE_CH1_FIRE) |          \
                                     PIN_ODR_LOW(GPIOE_CH2_FIRE) |          \
                                     PIN_ODR_LOW(GPIOE_PIN9) |              \
                                     PIN_ODR_LOW(GPIOE_PIN10) |             \
                                     PIN_ODR_LOW(GPIOE_PIN11) |             \
                                     PIN_ODR_LOW(GPIOE_PIN12) |             \
                                     PIN_ODR_LOW(GPIOE_PIN13) |             \
                                     PIN_ODR_LOW(GPIOE_PIN14) |             \
                                     PIN_ODR_LOW(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_BUZZER, 0U) |        \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_CH1_FIRE, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_CH2_FIRE, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0U))

/*
 * GPIOH setup:
 * NOTE: On LQFP100 only PH0 and PH1 exist physically (HSE oscillator).
 *       PH2-PH15 register bits defined for completeness.
 *
 * PH0  - OSC_IN                   (input floating, HSE).
 * PH1  - OSC_OUT                  (input floating, HSE).
 * PH2  - PIN2                     (input pulldown).
 * PH3  - PIN3                     (input pulldown).
 * PH4  - PIN4                     (input pulldown).
 * PH5  - PIN5                     (input pulldown).
 * PH6  - PIN6                     (input pulldown).
 * PH7  - PIN7                     (input pulldown).
 * PH8  - PIN8                     (input pulldown).
 * PH9  - PIN9                     (input pulldown).
 * PH10 - PIN10                    (input pulldown).
 * PH11 - PIN11                    (input pulldown).
 * PH12 - PIN12                    (input pulldown).
 * PH13 - PIN13                    (input pulldown).
 * PH14 - PIN14                    (input pulldown).
 * PH15 - PIN15                    (input pulldown).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |          \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |         \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |            \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOH_OSC_IN) |     \
                                     PIN_OSPEED_VERYLOW(GPIOH_OSC_OUT) |    \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN2) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN3) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN4) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN5) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN6) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN7) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN8) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN9) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN10) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN11) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN12) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN13) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN14) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_LOW(GPIOH_OSC_IN) |            \
                                     PIN_ODR_LOW(GPIOH_OSC_OUT) |           \
                                     PIN_ODR_LOW(GPIOH_PIN2) |              \
                                     PIN_ODR_LOW(GPIOH_PIN3) |              \
                                     PIN_ODR_LOW(GPIOH_PIN4) |              \
                                     PIN_ODR_LOW(GPIOH_PIN5) |              \
                                     PIN_ODR_LOW(GPIOH_PIN6) |              \
                                     PIN_ODR_LOW(GPIOH_PIN7) |              \
                                     PIN_ODR_LOW(GPIOH_PIN8) |              \
                                     PIN_ODR_LOW(GPIOH_PIN9) |              \
                                     PIN_ODR_LOW(GPIOH_PIN10) |             \
                                     PIN_ODR_LOW(GPIOH_PIN11) |             \
                                     PIN_ODR_LOW(GPIOH_PIN12) |             \
                                     PIN_ODR_LOW(GPIOH_PIN13) |             \
                                     PIN_ODR_LOW(GPIOH_PIN14) |             \
                                     PIN_ODR_LOW(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*===========================================================================*/
/* GPIO F — Not bonded on LQFP100, all input pull-down.                      */
/*===========================================================================*/

#define VAL_GPIOF_MODER             0x00000000U
#define VAL_GPIOF_OTYPER            0x00000000U
#define VAL_GPIOF_OSPEEDR           0x00000000U
#define VAL_GPIOF_PUPDR             0xAAAAAAAAU
#define VAL_GPIOF_ODR               0x00000000U
#define VAL_GPIOF_AFRL              0x00000000U
#define VAL_GPIOF_AFRH              0x00000000U

/*===========================================================================*/
/* GPIO G — Not bonded on LQFP100, all input pull-down.                      */
/*===========================================================================*/

#define VAL_GPIOG_MODER             0x00000000U
#define VAL_GPIOG_OTYPER            0x00000000U
#define VAL_GPIOG_OSPEEDR           0x00000000U
#define VAL_GPIOG_PUPDR             0xAAAAAAAAU
#define VAL_GPIOG_ODR               0x00000000U
#define VAL_GPIOG_AFRL              0x00000000U
#define VAL_GPIOG_AFRH              0x00000000U

/*===========================================================================*/
/* GPIO J — Not bonded on LQFP100, all input pull-down.                      */
/*===========================================================================*/

#define VAL_GPIOJ_MODER             0x00000000U
#define VAL_GPIOJ_OTYPER            0x00000000U
#define VAL_GPIOJ_OSPEEDR           0x00000000U
#define VAL_GPIOJ_PUPDR             0xAAAAAAAAU
#define VAL_GPIOJ_ODR               0x00000000U
#define VAL_GPIOJ_AFRL              0x00000000U
#define VAL_GPIOJ_AFRH              0x00000000U

/*===========================================================================*/
/* GPIO K — Not bonded on LQFP100, all input pull-down.                      */
/*===========================================================================*/

#define VAL_GPIOK_MODER             0x00000000U
#define VAL_GPIOK_OTYPER            0x00000000U
#define VAL_GPIOK_OSPEEDR           0x00000000U
#define VAL_GPIOK_PUPDR             0xAAAAAAAAU
#define VAL_GPIOK_ODR               0x00000000U
#define VAL_GPIOK_AFRL              0x00000000U
#define VAL_GPIOK_AFRH              0x00000000U

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_ACS4_H725_H */
