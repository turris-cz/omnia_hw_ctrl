/**
 ******************************************************************************
 * @file    power_control.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    22-July-2015
 * @brief   Functions for control of DC/DC converters.
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "cpu.h"
#include "power_control.h"
#include "delay.h"
#include "led_driver.h"
#include "slave_i2c_device.h"
#include "debug.h"
#include "timer.h"

#if !defined(OMNIA_BOARD_REVISION)
#error build system did not define OMNIA_BOARD_REVISION macro
#endif

#if !defined(USER_REGULATOR_ENABLED)
#error build system did not define USER_REGULATOR_ENABLED macro
#endif

#if USER_REGULATOR_ENABLED && OMNIA_BOARD_REVISION >= 32
#error user regulator not supported on board revision 32 and newer
#endif

/* Private define ------------------------------------------------------------*/

/* defines for timeout handling during regulator startup */
#define DELAY_AFTER_ENABLE      5
#define DELAY_BETWEEN_READINGS  20
#define TIMEOUT                 100 /* DELAY_BETWEEN_READINGS * 100 = 2 sec */

/* define for timeout handlling during reset */
#define RESET_STATE_READING     5 /* ms */
#define RESET_TIMEOFFSET        2

#define RGB_COLOR_LEVELS        255

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

#if USER_REGULATOR_ENABLED
/*******************************************************************************
 * @function   power_control_prog4v5_config
 * @brief      Configuration for programming possibility of 4V5 power source.
 * @param      None.
 * @retval     None.
 *****************************************************************************/
static void power_control_prog4v5_config(void)
{
    gpio_init_outputs(pin_pushpull, pin_spd_2, 0, PRG_4V5_PIN);
}
#endif

/*******************************************************************************
  * @function   system_control_io_config
  * @brief      GPIO config for EN, PG, Reset and USB signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_io_config(void)
{
    /* Output signals */
    gpio_init_outputs(pin_pushpull, pin_spd_2, 0,
                      RES_RAM_PIN, ENABLE_5V_PIN, ENABLE_3V3_PIN,
                      ENABLE_1V35_PIN, ENABLE_4V5_PIN, ENABLE_1V8_PIN,
                      ENABLE_1V5_PIN, ENABLE_1V2_PIN, ENABLE_VTT_PIN,
                      USB30_PWRON_PIN, USB31_PWRON_PIN, CFG_CTRL_PIN);
    gpio_init_outputs(pin_pushpull, pin_spd_2, 1, INT_MCU_PIN);

    gpio_init_outputs(pin_opendrain, pin_spd_2, 0, MANRES_PIN);
    gpio_init_outputs(pin_opendrain, pin_spd_2, 1, SYSRES_OUT_PIN); /* dont control this ! */

    /* Input signals */
    gpio_init_inputs(pin_pullup,
                     PG_5V_PIN, PG_3V3_PIN, PG_1V35_PIN, PG_4V5_PIN,
                     PG_1V8_PIN, PG_1V5_PIN, PG_1V2_PIN, PG_VTT_PIN,
                     USB30_OVC_PIN, USB31_OVC_PIN, LED_BRT_PIN,
                     DBGRES_PIN, MRES_PIN, RTC_ALARM_PIN);

#if USER_REGULATOR_ENABLED
    power_control_prog4v5_config();
#endif
}

/*******************************************************************************
  * @function   power_control_set_startup_condition
  * @brief      Set signals to reset state before board startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_set_startup_condition(void)
{
    gpio_write(CFG_CTRL_PIN, 1); /* disconnect switches */
    gpio_write(MANRES_PIN, 0); /* board reset activated */
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
            gpio_write(ENABLE_5V_PIN, 1);
            delay(DELAY_AFTER_ENABLE);

            while(!gpio_read(PG_5V_PIN))
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
            gpio_write(ENABLE_3V3_PIN, 1);
            delay(DELAY_AFTER_ENABLE);

            while(!gpio_read(PG_3V3_PIN))
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
            gpio_write(ENABLE_1V35_PIN, 1);
            delay(DELAY_AFTER_ENABLE);
            while(!gpio_read(PG_1V35_PIN))
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

#if USER_REGULATOR_ENABLED
        case REG_4V5:
        {
            gpio_write(ENABLE_4V5_PIN, 1);
            delay(DELAY_AFTER_ENABLE);
            while(!gpio_read(PG_4V5_PIN))
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
#endif /* USER_REGULATOR_ENABLED */

        case REG_1V8:
        {
            gpio_write(ENABLE_1V8_PIN, 1);
            delay(DELAY_AFTER_ENABLE);
            while(!gpio_read(PG_1V8_PIN))
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
            gpio_write(ENABLE_1V5_PIN, 1);
            delay(DELAY_AFTER_ENABLE);
            while(!gpio_read(PG_1V5_PIN))
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
            gpio_write(ENABLE_1V2_PIN, 1);
            delay(DELAY_AFTER_ENABLE);
            while(!gpio_read(PG_1V2_PIN))
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
            gpio_write(ENABLE_VTT_PIN, 1);
            delay(DELAY_AFTER_ENABLE);
            while(!gpio_read(PG_VTT_PIN))
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
     * 2) 4.5V regulator - user selectable - not populated on the board
     * 3) 3.3V regulator
     * 4) 1.8V regulator
     * 5) 1.5V regulator - not populated on the board
     * 6) 1.35V regulator
     *    VTT regulator
     * 7) 1.2V regulator
     */

    value = power_control_start_regulator(REG_5V);
    if (value != NO_ERROR)
        return value;

#if USER_REGULATOR_ENABLED
    value = power_control_start_regulator(REG_4V5);
    if (value != NO_ERROR)
        return value;
#endif

    value = power_control_start_regulator(REG_3V3);
    if (value != NO_ERROR)
        return value;

    value = power_control_start_regulator(REG_1V8);
    if (value != NO_ERROR)
        return value;

    if (OMNIA_BOARD_REVISION < 32) {
        value = power_control_start_regulator(REG_1V5);
        if (value != NO_ERROR)
            return value;
    }

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
    /* don't collapse this into one call of gpio_write_multi(), since these
     * should be disabled in the given order */
    gpio_write(ENABLE_1V2_PIN, 0);
    gpio_write(ENABLE_1V35_PIN, 0);
    gpio_write(ENABLE_VTT_PIN, 0);
    gpio_write(ENABLE_1V5_PIN, 0);
    gpio_write(ENABLE_1V8_PIN, 0);
    gpio_write(ENABLE_3V3_PIN, 0);
    gpio_write(ENABLE_4V5_PIN, 0);
    gpio_write(ENABLE_5V_PIN, 0);
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
    gpio_write(usb_port == USB3_PORT0 ? USB30_PWRON_PIN : USB31_PWRON_PIN,
               usb_state != USB_ON);
}

/*******************************************************************************
  * @function   power_control_get_usb_overcurrent
  * @brief      Get USB overcurrent status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB overcurrent ocurred; 0 - no USB overcurrent
  *****************************************************************************/
bool power_control_get_usb_overcurrent(usb_ports_t usb_port)
{
    return !gpio_read(usb_port == USB3_PORT0 ? USB30_OVC_PIN : USB31_OVC_PIN);
}

/*******************************************************************************
  * @function   power_control_get_usb_poweron
  * @brief      Get USB poweron status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB power ON; 0 - USB power OFF
  *****************************************************************************/
bool power_control_get_usb_poweron(usb_ports_t usb_port)
{
    return !gpio_read(usb_port == USB3_PORT0 ? USB30_PWRON_PIN : USB31_PWRON_PIN);
}

/*******************************************************************************
  * @function   power_control_usb_timeout_config
  * @brief      Timer configuration for USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_config(void)
{
    timer_init(USB_TIMEOUT_TIMER, timer_interrupt, 8000, 8000, 5);
}

/*******************************************************************************
  * @function   power_control_usb_timeout_enable
  * @brief      Enable USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_enable(void)
{
    timer_enable(USB_TIMEOUT_TIMER, 1);
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
    timer_enable(USB_TIMEOUT_TIMER, 0);
    timer_set_counter(USB_TIMEOUT_TIMER, 0);
}

#if !BOOTLOADER_BUILD
/*******************************************************************************
  * @function   power_control_usb_timeout_irq_handler
  * @brief      Handle USB timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void __irq power_control_usb_timeout_irq_handler(void)
{
    struct st_i2c_status *i2c_control = &i2c_status;

    if (!timer_irq_clear_up(USB_TIMEOUT_TIMER))
        return;

    power_control_usb(USB3_PORT0, USB_ON);
    power_control_usb(USB3_PORT1, USB_ON);

    i2c_control->status_word |= STS_USB30_PWRON | STS_USB31_PWRON;

    power_control_usb_timeout_disable();
}
#endif

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
    uint32_t color = 0;
    uint16_t user_brightness;

    gpio_write(CFG_CTRL_PIN, 1);
    delay(50);
    gpio_write(MANRES_PIN, 1);

    /* save brightness value to restore it */
    user_brightness = led_driver_get_brightness();

    /* wait for main board reset signal */
    while (!gpio_read(SYSRES_OUT_PIN))
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
                    led_set_color(LED_COUNT, GREEN_COLOR);
                    led_set_state(LED_COUNT, LED_OFF);
                    led_set_state(LED11, LED_ON);
                    led_driver_set_brightness(100);
                    reset_state = RST_LED11;
                    idx = 0;
                    red = 0;
                    green = RGB_COLOR_LEVELS;
                } break;

                case RST_LED11:
                {
                    reset_type = NORMAL_RESET;

                    led_set_state(LED11, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);

                    led_set_color(LED11, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED10; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED11;
                    }
                } break;

                case RST_LED10:
                {
                    reset_type = PREVIOUS_SNAPSHOT;

                    led_set_state(LED10, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED10, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED9; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED10;
                    }
                } break;

                case RST_LED9:
                {
                    reset_type = NORMAL_FACTORY_RESET;

                    led_set_state(LED9, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED9, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED8; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED9;
                    }
                } break;

                case RST_LED8:
                {
                    reset_type = HARD_FACTORY_RESET;

                    led_set_state(LED8, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED8, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED7; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED8;
                    }
                } break;

                case RST_LED7:
                {
                    reset_type = USER_RESET1;

                    led_set_state(LED7, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED7, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED6; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED7;
                    }
                } break;

                case RST_LED6:
                {
                    reset_type = USER_RESET2;

                    led_set_state(LED6, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED6, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED5; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED6;
                    }
                } break;

                case RST_LED5:
                {
                    reset_type = USER_RESET3;

                    led_set_state(LED5, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED5, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED4; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED5;
                    }
                } break;

                case RST_LED4:
                {
                    reset_type = USER_RESET4;

                    led_set_state(LED4, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED4, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED3; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED4;
                    }
                } break;

                case RST_LED3:
                {
                    reset_type = USER_RESET5;

                    led_set_state(LED3, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED3, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED2; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED3;
                    }
                } break;

                case RST_LED2:
                {
                    reset_type = USER_RESET6;

                    led_set_state(LED2, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED2, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED1; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED2;
                    }
                } break;

                case RST_LED1:
                {
                    reset_type = USER_RESET7;

                    led_set_state(LED1, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED1, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED0; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                    }
                    else
                    {
                        reset_state = RST_LED1;
                    }
                } break;

                case RST_LED0:
                {
                    reset_type = USER_RESET8;

                    led_set_state(LED0, LED_ON);

                    idx++; /* increase color level */
                    red++;
                    green--;
                    color = (red << 16) | (green << 8);;

                    led_set_color(LED0, color);

                    if (idx >= RGB_COLOR_LEVELS)
                    {
                        reset_state = RST_LED11; /* next state */
                        idx = 0;
                        red = 0;
                        green = RGB_COLOR_LEVELS;
                        /* final state - go back to start */
                        led_set_color(LED_COUNT, GREEN_COLOR);
                        led_set_state(LED_COUNT, LED_OFF);
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
    gpio_write(CFG_CTRL_PIN, 0);

    if (reset_type != NORMAL_RESET)
    {
        led_driver_set_brightness(0);
        delay(300);
        led_driver_set_brightness(100);
        delay(300);
        led_driver_set_brightness(0);
        delay(300);
        led_driver_set_brightness(100);
        delay(600);
    }

    /* restore brightness and color */
    led_driver_set_brightness(user_brightness);
    led_set_state(LED_COUNT, LED_OFF);
    led_set_color(LED_COUNT, WHITE_COLOR);

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
    led_set_color(POWER_LED, WHITE_COLOR);
    led_set_state(POWER_LED, LED_ON);
}

/*******************************************************************************
  * @function   power_led_activity
  * @brief      Set on power LED.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_led_activity(void)
{
    led_set_state(POWER_LED, LED_ON);
}

#if USER_REGULATOR_ENABLED
/* programming pin for user regulator */
/* timing for logic '1' and '0' consists of only NOPs, because it must be very
precise. Pulse for logic '1' or '0' takes only 1 us */
static __force_inline void user_reg_prg_logic(bool val)
{
	if (val) {
		gpio_write(PRG_4V5_PIN, 1);
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		gpio_write(PRG_4V5_PIN, 0);
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
	} else {
		gpio_write(PRG_4V5_PIN, 1);
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		gpio_write(PRG_4V5_PIN, 0);
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
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
     user_reg_prg_logic(1);

     /* chip select */
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);

     /* register address */
     user_reg_prg_logic(0);
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);

     /* datafield - 0xDF */
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);

     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);

     /* stop condition */
     user_reg_prg_logic(1);
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
     user_reg_prg_logic(1);

     /* chip select */
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);

     /* register address */
     user_reg_prg_logic(0);
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);

     /* datafield - 0xEF */
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);

     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);

     /* stop condition */
     user_reg_prg_logic(1);
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
     user_reg_prg_logic(1);

     /* chip select */
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);

     /* register address */
     user_reg_prg_logic(0);
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);

     /* datafield - 0xFC */
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);

     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);
     user_reg_prg_logic(0);

     /* stop condition */
     user_reg_prg_logic(1);
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
     user_reg_prg_logic(1);

     /* chip select */
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);

     /* register address */
     user_reg_prg_logic(0);
     user_reg_prg_logic(0);
     user_reg_prg_logic(1);
     user_reg_prg_logic(0);

     /* datafield - 0xF8 */
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);
     user_reg_prg_logic(1);

     user_reg_prg_logic(1);
     user_reg_prg_logic(0);
     user_reg_prg_logic(0);
     user_reg_prg_logic(0);

     /* stop condition */
     user_reg_prg_logic(1);
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
#endif /* USER_REGULATOR_ENABLED */

/*******************************************************************************
  * @function   periph_control_io_config
  * @brief      Configuration of new IO pins for Omnia32
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void periph_control_io_config(void)
{
    gpio_init_inputs(pin_pullup, SFP_nDET_PIN);
    gpio_init_outputs(pin_opendrain, pin_spd_2, 0,
                      nRES_MMC_PIN, nRES_LAN_PIN, nRES_PHY_PIN,
                      nPERST0_PIN, nPERST1_PIN, nPERST2_PIN,
                      nVHV_CTRL_PIN, PHY_SFP_PIN);

    gpio_write_multi(1, nVHV_CTRL_PIN, PHY_SFP_PIN);
}

/*******************************************************************************
  * @function   periph_control_rst_init
  * @brief      Set reset init states for peripherals for Omnia32
  * @param      None.
  * @retval     The corresponding initial settings for extended control word.
  *****************************************************************************/
uint16_t periph_control_rst_init(void)
{
    gpio_write_multi(0, nRES_MMC_PIN, nRES_LAN_PIN, nRES_PHY_PIN, nPERST0_PIN,
                     nPERST1_PIN, nPERST2_PIN);

    gpio_write_multi(1, nVHV_CTRL_PIN, PHY_SFP_PIN);

    return EXT_CTL_PHY_SFP | EXT_CTL_nVHV_CTRL;
}
