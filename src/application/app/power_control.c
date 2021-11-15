/**
 ******************************************************************************
 * @file    power_control.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    26-October-2021
 * @brief   Functions for control of DC/DC converters.
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "power_control.h"
#include "delay.h"
#include "led_driver.h"
#include "debug_serial.h"

/* Private define ------------------------------------------------------------*/

/* programming pin for user regulator */
#define PRG_PIN_HIGH            gpio_bit_set(PRG_4V5_PIN_PORT, PRG_4V5_PIN)
#define PRG_PIN_LOW             gpio_bit_reset(PRG_4V5_PIN_PORT, PRG_4V5_PIN)

/* timing for logic '1' and '0' consists of only NOPs, because it must be very
precise. Pulse for logic '1' or '0' takes only 1 us */
#define SET_LOGIC_HIGH() ({PRG_PIN_HIGH; \
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    PRG_PIN_LOW;\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();})

#define SET_LOGIC_LOW() ({PRG_PIN_HIGH; \
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    PRG_PIN_LOW;\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();\
    __NOP();})

/* defines for timeout handling during regulator startup */
#define DELAY_AFTER_ENABLE      5
#define DELAY_BETWEEN_READINGS  20
#define TIMEOUT                 100 /* DELAY_BETWEEN_READINGS * 100 = 2 sec */

/* define for timeout handlling during reset */
#define RESET_STATE_READING     5 /* ms */
#define RESET_TIMEOFFSET        2

#define RGB_COLOUR_LEVELS       255

typedef enum reset_states {
    RST_INIT,
    RST_LED0,
    RST_LED1,
    RST_LED2,
    RST_LED3,
    RST_LED4,
    RST_LED5,
    RST_LED6,
    RST_LED7,
    RST_LED8,
    RST_LED9,
    RST_LED10,
    RST_LED11,
} reset_state_t;

/*******************************************************************************
  * @function   power_control_prog4v5_config
  * @brief      Configuration for programming possibility of 4V5 power source.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void power_control_prog4v5_config(void)
{    
    /* pin config for programming */
    rcu_periph_clock_enable(PRG_4V5_PIN_PERIPH_CLOCK);

    gpio_mode_set(PRG_4V5_PIN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PRG_4V5_PIN);
    gpio_output_options_set(PRG_4V5_PIN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PRG_4V5_PIN);

    PRG_PIN_LOW;
}

/*******************************************************************************
  * @function   system_control_io_config
  * @brief      GPIO config for EN, PG, Reset and USB signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_io_config(void)
{
    /* Output signals */
    rcu_periph_clock_enable(RES_RAM_PIN_PERIPH_CLOCK);
    gpio_mode_set(RES_RAM_PIN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, RES_RAM_PIN
                  | INT_MCU_PIN | ENABLE_5V_PIN | ENABLE_3V3_PIN | ENABLE_1V35_PIN
                  | ENABLE_4V5_PIN | ENABLE_1V8_PIN | ENABLE_1V5_PIN | ENABLE_1V2_PIN
                  | ENABLE_VTT_PIN | USB30_PWRON_PIN | USB31_PWRON_PIN | CFG_CTRL_PIN);

    gpio_output_options_set(RES_RAM_PIN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, RES_RAM_PIN
                            | INT_MCU_PIN | ENABLE_5V_PIN | ENABLE_3V3_PIN | ENABLE_1V35_PIN
                            | ENABLE_4V5_PIN | ENABLE_1V8_PIN | ENABLE_1V5_PIN | ENABLE_1V2_PIN
                            | ENABLE_VTT_PIN | USB30_PWRON_PIN | USB31_PWRON_PIN | CFG_CTRL_PIN);


    /* OD */
    rcu_periph_clock_enable(SYSRES_OUT_PIN_PERIPH_CLOCK);
    gpio_mode_set(SYSRES_OUT_PIN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, SYSRES_OUT_PIN | MANRES_PIN);
    gpio_output_options_set(SYSRES_OUT_PIN_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, SYSRES_OUT_PIN | MANRES_PIN);

    /* Input signals */
    rcu_periph_clock_enable(MRES_PIN_PERIPH_CLOCK);
    gpio_mode_set(MRES_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, MRES_PIN);

    rcu_periph_clock_enable(DGBRES_PIN_PERIPH_CLOCK);
    gpio_mode_set(DGBRES_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, DGBRES_PIN | PG_5V_PIN
                  | PG_3V3_PIN | PG_1V35_PIN | PG_4V5_PIN | PG_1V8_PIN | PG_1V5_PIN
                  | PG_1V2_PIN | PG_VTT_PIN | USB30_OVC_PIN | USB31_OVC_PIN | RTC_ALARM_PIN
                  | LED_BRT_PIN);

    gpio_bit_set(SYSRES_OUT_PIN_PORT, SYSRES_OUT_PIN); /* dont control this ! */
    gpio_bit_set(INT_MCU_PIN_PORT, INT_MCU_PIN);

    power_control_prog4v5_config();
}

/*******************************************************************************
  * @function   power_control_set_startup_condition
  * @brief      Set signals to reset state before board startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_set_startup_condition(void)
{
    gpio_bit_set(CFG_CTRL_PIN_PORT, CFG_CTRL_PIN); //disconnect switches
    gpio_bit_reset(MANRES_PIN_PORT, MANRES_PIN); //board reset activated

    power_control_usb(USB3_PORT0, USB_ON);
    power_control_usb(USB3_PORT1, USB_ON);
}

/*******************************************************************************
  * @function   power_control_start_regulator
  * @brief      Start DC/DC regulator and handle timeout.
  * @param      regulator: regulator type.
  * @retval     error, if problem with PG signal occures.
  *****************************************************************************/
error_type_t power_control_start_regulator(reg_type_t regulator)
{
    error_type_t error = NO_ERROR;
    uint16_t counter = 0;

    switch(regulator)
    {
        case REG_5V:
        {
            gpio_bit_set(ENABLE_5V_PIN_PORT, ENABLE_5V_PIN);
            delay(DELAY_AFTER_ENABLE);

            while(!(gpio_input_bit_get(PG_5V_PIN_PORT, PG_5V_PIN)))
            {
                delay(DELAY_BETWEEN_READINGS);
                counter++;
                if (counter >= TIMEOUT)
                {
                    error = PG_5V_ERROR;
                    break;
                }
            }
        }break;

        case REG_3V3:
        {
            gpio_bit_set(ENABLE_3V3_PIN_PORT, ENABLE_3V3_PIN);
            delay(DELAY_AFTER_ENABLE);

            while(!(gpio_input_bit_get(PG_3V3_PIN_PORT, PG_3V3_PIN)))
            {
                delay(DELAY_BETWEEN_READINGS);
                counter++;
                if (counter >= TIMEOUT)
                {
                    error = PG_3V3_ERROR;
                    break;
                }
            }

        } break;

        case REG_1V35:
        {
            gpio_bit_set(ENABLE_1V35_PIN_PORT, ENABLE_1V35_PIN);
            delay(DELAY_AFTER_ENABLE);
            while(!(gpio_input_bit_get(PG_1V35_PIN_PORT, PG_1V35_PIN)))
            {
                delay(DELAY_BETWEEN_READINGS);
                counter++;
                if (counter >= TIMEOUT)
                {
                    error = PG_1V35_ERROR;
                    break;
                }
            }
        }break;

        case REG_4V5:
        {
            gpio_bit_set(ENABLE_4V5_PIN_PORT, ENABLE_4V5_PIN);
            delay(DELAY_AFTER_ENABLE);
            while(!(gpio_input_bit_get(PG_4V5_PIN_PORT, PG_4V5_PIN)))
            {
                delay(DELAY_BETWEEN_READINGS);
                counter++;
                if (counter >= TIMEOUT)
                {
                    error = PG_4V5_ERROR;
                    break;
                }
            }
        }break;

        case REG_1V8:
        {
            gpio_bit_set(ENABLE_1V8_PIN_PORT, ENABLE_1V8_PIN);
            delay(DELAY_AFTER_ENABLE);
            while(!(gpio_input_bit_get(PG_1V8_PIN_PORT, PG_1V8_PIN)))
            {
                delay(DELAY_BETWEEN_READINGS);
                counter++;
                if (counter >= TIMEOUT)
                {
                    error = PG_1V8_ERROR;
                    break;
                }
            }
        }break;

        case REG_1V5:
        {
            gpio_bit_set(ENABLE_1V5_PIN_PORT, ENABLE_1V5_PIN);
            delay(DELAY_AFTER_ENABLE);
            while(!(gpio_input_bit_get(PG_1V5_PIN_PORT, PG_1V5_PIN)))
            {
                delay(DELAY_BETWEEN_READINGS);
                counter++;
                if (counter >= TIMEOUT)
                {
                    error = PG_1V5_ERROR;
                    break;
                }
            }
        }break;

        case REG_1V2:
        {
            gpio_bit_set(ENABLE_1V2_PIN_PORT, ENABLE_1V2_PIN);
            delay(DELAY_AFTER_ENABLE);
            while(!(gpio_input_bit_get(PG_1V2_PIN_PORT, PG_1V2_PIN)))
            {
                delay(DELAY_BETWEEN_READINGS);
                counter++;
                if (counter >= TIMEOUT)
                {
                    error = PG_1V2_ERROR;
                    break;
                }
            }
        }break;

        case REG_VTT:
        {
            gpio_bit_set(ENABLE_VTT_PIN_PORT, ENABLE_VTT_PIN);
            delay(DELAY_AFTER_ENABLE);
            while(!(gpio_input_bit_get(PG_VTT_PIN_PORT, PG_VTT_PIN)))
            {
                delay(DELAY_BETWEEN_READINGS);
                counter++;
                if (counter >= TIMEOUT)
                {
                    error = PG_VTT_ERROR;
                    break;
                }
            }
        }break;

        default:
            break;
    }

    return error;
}

/*******************************************************************************
  * @function   power_control_enable_regulators
  * @brief      Starts DC/DC regulators.
  * @param      None.
  * @retval     Error if timeout elapsed.
  *****************************************************************************/
error_type_t power_control_enable_regulators(void)
{
    error_type_t value = NO_ERROR;

    /*
     * power-up sequence:
     * 1) 5V regulator
     * 2) 4.5V regulator - user selectable
     * 3) 3.3V regulator
     * 4) 1.8V regulator
     * 5) 1.5V regulator
     * 6) 1.35V regulator
     *    VTT regulator
     * 7) 1.2V regulator
     */

    value = power_control_start_regulator(REG_5V);
    if (value != NO_ERROR)
        return value;

//    value = power_control_start_regulator(REG_4V5);
//    if (value != NO_ERROR)
//        return value;

    value = power_control_start_regulator(REG_3V3);
    if (value != NO_ERROR)
        return value;

    value = power_control_start_regulator(REG_1V8);
    if (value != NO_ERROR)
        return value;

    value = power_control_start_regulator(REG_1V5);
    if (value != NO_ERROR)
        return value;

    value = power_control_start_regulator(REG_1V35);
    if (value != NO_ERROR)
        return value;

    value = power_control_start_regulator(REG_VTT);
    if (value != NO_ERROR)
        return value;

    value = power_control_start_regulator(REG_1V2);
    if (value != NO_ERROR)
        return value;

    return value;
}

/*******************************************************************************
  * @function   power_control_disable_regulators
  * @brief      Shutdown DC/DC regulators.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_disable_regulators(void)
{
    gpio_bit_reset(ENABLE_1V2_PIN_PORT, ENABLE_1V2_PIN);
    gpio_bit_reset(ENABLE_1V35_PIN_PORT, ENABLE_1V35_PIN);
    gpio_bit_reset(ENABLE_VTT_PIN_PORT, ENABLE_VTT_PIN);
    gpio_bit_reset(ENABLE_1V5_PIN_PORT, ENABLE_1V5_PIN);
    gpio_bit_reset(ENABLE_1V8_PIN_PORT, ENABLE_1V8_PIN);
    gpio_bit_reset(ENABLE_3V3_PIN_PORT, ENABLE_3V3_PIN);
    gpio_bit_reset(ENABLE_4V5_PIN_PORT, ENABLE_4V5_PIN);
    gpio_bit_reset(ENABLE_5V_PIN_PORT, ENABLE_5V_PIN);
}

/*******************************************************************************
  * @function   power_control_usb
  * @brief      Enable / disable power supply for USB.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @param      usb_state: USB_ON or USB_OFF.
  * @retval     None.
  *****************************************************************************/
void power_control_usb(usb_ports_t usb_port, usb_state_t usb_state)
{
    if (usb_port == USB3_PORT0)
    {
        if (usb_state == USB_ON)
            gpio_bit_reset(USB30_PWRON_PIN_PORT, USB30_PWRON_PIN);
        else
            gpio_bit_set(USB30_PWRON_PIN_PORT, USB30_PWRON_PIN);
    }
    else //USB3_PORT1
    {
        if (usb_state == USB_ON)
            gpio_bit_reset(USB31_PWRON_PIN_PORT, USB31_PWRON_PIN);
        else
            gpio_bit_set(USB31_PWRON_PIN_PORT, USB31_PWRON_PIN);
    }
}

/*******************************************************************************
  * @function   power_control_get_usb_overcurrent
  * @brief      Get USB overcurrent status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB overcurrent ocurred; 0 - no USB overcurrent
  *****************************************************************************/
uint8_t power_control_get_usb_overcurrent(usb_ports_t usb_port)
{
    if (usb_port == USB3_PORT0)
        return (!(gpio_input_bit_get(USB30_OVC_PIN_PORT, USB30_OVC_PIN)));
    else //USB3_PORT1
        return (!(gpio_input_bit_get(USB31_OVC_PIN_PORT, USB31_OVC_PIN)));
}

/*******************************************************************************
  * @function   power_control_get_usb_poweron
  * @brief      Get USB poweron status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB power ON; 0 - USB power OFF
  *****************************************************************************/
uint8_t power_control_get_usb_poweron(usb_ports_t usb_port)
{
    if (usb_port == USB3_PORT0)
        return (!(gpio_input_bit_get(USB30_PWRON_PIN_PORT, USB30_PWRON_PIN)));
    else //USB3_PORT1
        return (!(gpio_input_bit_get(USB31_PWRON_PIN_PORT, USB31_PWRON_PIN)));
}

/*******************************************************************************
  * @function   power_control_usb_timeout_config
  * @brief      Timer configuration for USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_config(void)
{
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_disable(RCU_TIMER16);
    timer_deinit(USB_TIMEOUT_TIMER);

    /* Clock enable */
    rcu_periph_clock_enable(RCU_TIMER16);

    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);

    /* Time base configuration - 1sec interrupt */
    /* TIMER16CLK = SystemCoreClock/7200 = 10KHz, the period is 1s(10000/10000 = 1s).*/
    timer_initpara.prescaler         = 7199;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(USB_TIMEOUT_TIMER, &timer_initpara);


    //???TIM_ARRPreloadConfig(USB_TIMEOUT_TIMER, ENABLE);

    /* TIM Interrupts enable */
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(USB_TIMEOUT_TIMER, TIMER_INT_FLAG_UP);
    /* enable the TIMER interrupt */
    timer_interrupt_enable(USB_TIMEOUT_TIMER, TIMER_INT_UP);

timer_enable(USB_TIMEOUT_TIMER); //TODO - pak smazat tento radek - volani v kodu jinde
    nvic_irq_enable(TIMER16_IRQn, 0, 5);
}

/*******************************************************************************
  * @function   power_control_usb_timeout_enable
  * @brief      Enable USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_enable(void)
{
    /* enable a TIMER */
    timer_enable(USB_TIMEOUT_TIMER);
}

/*******************************************************************************
  * @function   power_control_usb_timeout_disable
  * @brief      Disable USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_disable(void)
{
    /* disable timer and set initial condition */
    timer_disable(USB_TIMEOUT_TIMER);
    TIMER_CNT(USB_TIMEOUT_TIMER) = 0;
}

/*******************************************************************************
  * @function   power_control_first_startup
  * @brief      Handle SYSRES_OUT, MAN_RES, CFG_CTRL signals and factory reset
  *             during startup.
  * @param      None.
  * @retval     Type of factory reset.
  *****************************************************************************/
reset_type_t power_control_first_startup(void)
{
    reset_type_t reset_type = NORMAL_RESET;
    reset_state_t reset_state = RST_INIT;
    uint16_t reset_cnt = 0;
    uint8_t red, green, idx;
    uint32_t colour = 0;
    uint16_t user_brightness;

    gpio_bit_set(CFG_CTRL_PIN_PORT, CFG_CTRL_PIN);
    delay(50);
    gpio_bit_set(MANRES_PIN_PORT, MANRES_PIN);

    /* save brightness value to restore it */
    user_brightness = led_driver_pwm_get_brightness();

    /* wait for main board reset signal */
    while (!gpio_input_bit_get(SYSRES_OUT_PIN_PORT, SYSRES_OUT_PIN))
    {
        /* handle factory reset timeouts */
        delay(RESET_STATE_READING);
        reset_cnt++;

        if (reset_cnt >= RESET_TIMEOFFSET)
        {
            switch(reset_state)
            {
                case RST_INIT:
                {
                    led_driver_set_colour(LED_COUNT, GREEN_COLOUR);
                    led_driver_set_led_state(LED_COUNT, LED_OFF);
                    led_driver_set_led_state(LED11, LED_ON);
                    led_driver_pwm_set_brightness(100);
                    reset_state = RST_LED11;
                    idx = 0;
                    red = 0;
                    green = RGB_COLOUR_LEVELS;
                } break;

                case RST_LED11:
                {
                    reset_type = NORMAL_RESET;

                    led_driver_set_led_state(LED11, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED11, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED10; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED11;
                    }
                } break;

                case RST_LED10:
                {
                    reset_type = PREVIOUS_SNAPSHOT;

                    led_driver_set_led_state(LED10, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED10, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED9; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED10;
                    }
                } break;

                case RST_LED9:
                {
                    reset_type = NORMAL_FACTORY_RESET;

                    led_driver_set_led_state(LED9, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED9, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED8; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED9;
                    }
                } break;

                case RST_LED8:
                {
                    reset_type = HARD_FACTORY_RESET;

                    led_driver_set_led_state(LED8, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED8, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED7; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED8;
                    }
                } break;

                case RST_LED7:
                {
                    reset_type = USER_RESET1;

                    led_driver_set_led_state(LED7, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED7, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED6; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED7;
                    }
                } break;

                case RST_LED6:
                {
                    reset_type = USER_RESET2;

                    led_driver_set_led_state(LED6, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED6, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED5; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED6;
                    }
                } break;

                case RST_LED5:
                {
                    reset_type = USER_RESET3;

                    led_driver_set_led_state(LED5, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED5, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED4; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED5;
                    }
                } break;

                case RST_LED4:
                {
                    reset_type = USER_RESET4;

                    led_driver_set_led_state(LED4, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED4, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED3; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED4;
                    }
                } break;

                case RST_LED3:
                {
                    reset_type = USER_RESET5;

                    led_driver_set_led_state(LED3, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED3, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED2; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED3;
                    }
                } break;

                case RST_LED2:
                {
                    reset_type = USER_RESET6;

                    led_driver_set_led_state(LED2, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED2, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED1; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED2;
                    }
                } break;

                case RST_LED1:
                {
                    reset_type = USER_RESET7;

                    led_driver_set_led_state(LED1, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED1, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED0; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED1;
                    }
                } break;

                case RST_LED0:
                {
                    reset_type = USER_RESET8;

                    led_driver_set_led_state(LED0, LED_ON);

                    idx++; /* increase colour level */
                    red++;
                    green--;
                    colour = (red << 16) | (green << 8);

                    led_driver_set_colour(LED0, colour);

                    if (idx >= RGB_COLOUR_LEVELS)
                    {
                        reset_state = RST_LED11; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOUR_LEVELS;
                        /* final state - go back to start */
                        led_driver_set_colour(LED_COUNT, GREEN_COLOUR);
                        led_driver_set_led_state(LED_COUNT, LED_OFF);
                    }
                    else
                    {
                        reset_state = RST_LED0;
                    }
                } break;
            }

            reset_cnt = 0;
        }
    }

    delay(10); /* 10 + 5ms (in while loop) delay after releasing of reset signal */
    gpio_bit_reset(CFG_CTRL_PIN_PORT, CFG_CTRL_PIN);

    if (reset_type != NORMAL_RESET)
    {
        led_driver_pwm_set_brightness(0);
        delay(300);
        led_driver_pwm_set_brightness(100);
        delay(300);
        led_driver_pwm_set_brightness(0);
        delay(300);
        led_driver_pwm_set_brightness(100);
        delay(600);
    }

    /* restore brightness and colour */
    led_driver_pwm_set_brightness(user_brightness);
    led_driver_set_led_state(LED_COUNT, LED_OFF);
    led_driver_set_colour(LED_COUNT, WHITE_COLOUR);

    return reset_type;
}

/*******************************************************************************
  * @function   power_control_set_power_led
  * @brief      Set on power LED.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_set_power_led(void)
{
    struct led_rgb *rgb_leds = leds;

    led_driver_set_colour(POWER_LED, WHITE_COLOUR);
    rgb_leds[POWER_LED].led_state_default = LED_ON;
}

/*******************************************************************************
  * @function   power_led_activity
  * @brief      Set on power LED.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_led_activity(void)
{
    struct led_rgb *rgb_leds = leds;

    if (rgb_leds[POWER_LED].led_mode == LED_DEFAULT_MODE)
    {
        rgb_leds[POWER_LED].led_state_default = LED_ON;
    }
}

/*******************************************************************************
  * @function   power_control_set_voltage33
  * @brief      Set 3.3V voltage to the user regulator.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void power_control_set_voltage33(void)
{
     /* start condition */
     SET_LOGIC_HIGH();

     /* chip select */
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();

     /* register address */
     SET_LOGIC_LOW();
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();

     /* datafield - 0xDF */
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();

     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();

     /* stop condition */
     SET_LOGIC_HIGH();
}

/*******************************************************************************
  * @function   power_control_set_voltage36
  * @brief      Set 3.63V voltage to the user regulator.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void power_control_set_voltage36(void)
{
     /* start condition */
     SET_LOGIC_HIGH();

     /* chip select */
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();

     /* register address */
     SET_LOGIC_LOW();
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();

     /* datafield - 0xEF */
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();

     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();

     /* stop condition */
     SET_LOGIC_HIGH();
}

/*******************************************************************************
  * @function   power_control_set_voltage51
  * @brief      Set 5.125V voltage to the user regulator.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void power_control_set_voltage51(void)
{
     /* start condition */
     SET_LOGIC_HIGH();

     /* chip select */
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();

     /* register address */
     SET_LOGIC_LOW();
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();

     /* datafield - 0xFC */
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();

     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();
     SET_LOGIC_LOW();

     /* stop condition */
     SET_LOGIC_HIGH();
}

/*******************************************************************************
  * @function   power_control_set_voltage45
  * @brief      Set 4.5V voltage to the user regulator.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void power_control_set_voltage45(void)
{
     /* start condition */
     SET_LOGIC_HIGH();

     /* chip select */
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();

     /* register address */
     SET_LOGIC_LOW();
     SET_LOGIC_LOW();
     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();

     /* datafield - 0xF8 */
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();
     SET_LOGIC_HIGH();

     SET_LOGIC_HIGH();
     SET_LOGIC_LOW();
     SET_LOGIC_LOW();
     SET_LOGIC_LOW();

     /* stop condition */
     SET_LOGIC_HIGH();
}

/*******************************************************************************
  * @function   power_control_set_voltage
  * @brief      Set required voltage to the user regulator.
  * @param      voltage: enum value for desired voltage.
  * @retval     None.
  *****************************************************************************/
void power_control_set_voltage(voltage_value_t voltage)
{
    /* delay at least 10us before the next sequence */
    switch (voltage)
    {
        case VOLTAGE_33: power_control_set_voltage33(); break; /* 3.3V */
        case VOLTAGE_36: power_control_set_voltage36(); break; /* 3.63V */
        case VOLTAGE_45: power_control_set_voltage45(); break; /* 4.5V */
        case VOLTAGE_51: power_control_set_voltage51(); break; /* 5.125V */
        default:
            break;
    }
}
