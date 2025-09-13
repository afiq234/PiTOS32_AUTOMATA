#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_rom_sys.h"

#define STEP_PIN 33
#define DIR_PIN 27
#define EN_PIN 25

#define RX_BUF_SIZE 1024
#define TXD_PIN 17
#define RXD_PIN 16

#define SERVO_PULSE_GPIO 32
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MIN_DEGREE -90
#define SERVO_MAX_DEGREE 90
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD 20000

static mcpwm_cmpr_handle_t comparator = NULL;

// ---------------- servo helper ----------------
static inline uint32_t angleComparator(int angle)
{
    return (uint32_t)((angle - SERVO_MIN_DEGREE) *
                          (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
                          (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) +
                      SERVO_MIN_PULSEWIDTH_US);
}

static void init_servo(void)
{
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_gen_handle_t generator;

    mcpwm_timer_config_t tcfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&tcfg, &timer);

    mcpwm_operator_config_t ocfg = {.group_id = 0};
    mcpwm_new_operator(&ocfg, &oper);
    mcpwm_operator_connect_timer(oper, timer);

    mcpwm_comparator_config_t ccfg = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(oper, &ccfg, &comparator);

    mcpwm_generator_config_t gcfg = {.gen_gpio_num = SERVO_PULSE_GPIO};
    mcpwm_new_generator(oper, &gcfg, &generator);

    mcpwm_generator_set_actions_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());

    mcpwm_generator_set_actions_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       comparator, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    // neutral position at start
    mcpwm_comparator_set_compare_value(comparator, 1500);
}

// ---------------- uart ----------------
static void initUART(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// ---------------- stepper ----------------
static inline void step_once_high_us(int high_us)
{
    gpio_set_level(STEP_PIN, 1);
    esp_rom_delay_us(high_us);
    gpio_set_level(STEP_PIN, 0);
}

static void run_stepper(void)
{
    gpio_set_level(EN_PIN, 0);  // enable
    gpio_set_level(DIR_PIN, 1); // direction
    vTaskDelay(pdMS_TO_TICKS(10));

    for (int i = 0; i < 10000; i++)
    {
        step_once_high_us(3);
        vTaskDelay(1); // ~10 ms delay -> ~100 Hz
    }

    gpio_set_level(EN_PIN, 1); // disable after done
}

// Initial State - get back to normal form
static void initial_stepper(void)
{
    gpio_set_level(EN_PIN, 0);  // enable
    gpio_set_level(DIR_PIN, 1); // direction
    vTaskDelay(pdMS_TO_TICKS(10));

    for (int i = 0; i < 5000; i++)
    {
        step_once_high_us(3);
        vTaskDelay(1); // ~10 ms delay -> ~100 Hz
    }

    gpio_set_level(EN_PIN, 1); // disable after done
}

// ---------------- RX task ----------------
static void rx_task(void *arg)
{
    uint8_t data[RX_BUF_SIZE + 1];
    while (1)
    {
        int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = '\0';
            if (strstr((char *)data, "normal"))
            {

                mcpwm_comparator_set_compare_value(comparator, angleComparator(-50));
                printf("Servo moved to normal (-45°)\r\n");
                run_stepper();
                mcpwm_comparator_set_compare_value(comparator, angleComparator(0));
            }
            else if (strstr((char *)data, "oversolder") || strstr((char *)data, "burnt"))
            {

                mcpwm_comparator_set_compare_value(comparator, angleComparator(50));
                printf("Servo moved to burnt and oversolder (+45°)\r\n");
                run_stepper();
                mcpwm_comparator_set_compare_value(comparator, angleComparator(0));
            }
        }
    }
}

// static void rx_task(void *arg)
// {
//     uint8_t data[RX_BUF_SIZE + 1];
//     while (1)
//     {
//         int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
//         if (rxBytes > 0)
//         {
//             data[rxBytes] = '\0';
//             int angle = atoi((char *)data);
//             if (angle >= SERVO_MIN_DEGREE && angle <= SERVO_MAX_DEGREE)
//             {
//                 mcpwm_comparator_set_compare_value(comparator, angleComparator(angle));
//                 printf("Servo set to %d deg, running stepper...\r\n", angle);
//                 run_stepper();
//             }
//         }
//     }
// }

void app_main(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << STEP_PIN) | (1ULL << DIR_PIN) | (1ULL << EN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io);
    gpio_set_level(EN_PIN, 1); // keep disabled initially

    initUART();
    init_servo();

    xTaskCreate(rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
}
