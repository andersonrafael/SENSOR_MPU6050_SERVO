#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"    // Necessário para NeoPixel
#include "hardware/clocks.h" // Necessário para clock do PIO

// Importa o programa PIO compilado (gerado pelo CMake)
#include "ws2812.pio.h"

#include "mpu6050/mpu6050.h"
#include "servo/servo.h"
#include "display/display.h"

#define ANGLE_ALERT_THRESHOLD 60.0f
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define BUZZER_PIN 21

// --- CONFIGURAÇÃO NEOPIXEL BITDOGLAB ---
#define NEOPIXEL_PIN 7 // Pino de dados dos LEDs (BitDogLab padrão)
#define NUM_PIXELS 25  // Matriz 5x5
#define IS_RGBW false  // WS2812 padrão é RGB apenas

// --- Constantes para controle ---
#define MOVEMENT_THRESHOLD 300
#define STABLE_TIME_MS 3000

// --- Variáveis de estado ---
static uint64_t last_movement_time = 0;
static int16_t last_ax = 0, last_ay = 0, last_az = 0;

// Variáveis para controle de tempo do LED
static uint64_t last_led_toggle_time = 0;
static bool led_on_state = false;

// --- Funções do Buzzer ---
void pwm_init_buzzer(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 12500);
    pwm_config_set_clkdiv(&config, 10);
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0);
}

void beep(uint pin, uint duration_ms)
{
    pwm_set_gpio_level(pin, 6250);
    sleep_ms(duration_ms);
    pwm_set_gpio_level(pin, 0);
}

// --- Funções para NeoPixel (PIO) ---
static inline void put_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

void set_neopixel_color(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t color = urgb_u32(r, g, b);
    // Define a cor para TODOS os LEDs da matriz para garantir visibilidade
    for (int i = 0; i < NUM_PIXELS; i++)
    {
        put_pixel(color);
    }
}

void leds_init_neopixel(void)
{
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, NEOPIXEL_PIN, 800000, IS_RGBW);
}

// --- Lógica de Controle dos LEDs ---
/**
 * @brief Controla o NeoPixel baseado nos requisitos:
 * - Normal: Verde Aceso (Sólido)
 * - Movimento: Amarelo Piscando (1s total: 500ms on, 500ms off)
 * - Alerta: Vermelho Piscando (0.5s total: 250ms on, 250ms off)
 */
void update_neopixel_state(bool alarm, bool movement, bool stable, uint64_t current_time)
{

    // Configurações de intervalo
    uint32_t interval_ms = 0;

    if (alarm)
    {
        // Alerta: Pisca a cada meio segundo (250ms on / 250ms off)
        interval_ms = 250;
    }
    else if (movement && !stable)
    {
        // Movimento: Pisca a cada 1 segundo (500ms on / 500ms off)
        interval_ms = 500;
    }
    else
    {
        // Estável: Sólido, sem piscar
        interval_ms = 0;
    }

    // Lógica de Piscagem
    if (interval_ms > 0)
    {
        if (current_time - last_led_toggle_time >= interval_ms)
        {
            led_on_state = !led_on_state;
            last_led_toggle_time = current_time;
        }
    }
    else
    {
        led_on_state = true; // Sempre aceso se for estável
    }

    // Aplicação das Cores (R, G, B)
    if (alarm)
    {
        // Vermelho Piscando
        if (led_on_state)
            set_neopixel_color(100, 0, 0); // Brilho 100/255
        else
            set_neopixel_color(0, 0, 0);
    }
    else if (movement && !stable)
    {
        // Amarelo Piscando (Amarelo = Vermelho + Verde)
        if (led_on_state)
            set_neopixel_color(80, 80, 0);
        else
            set_neopixel_color(0, 0, 0);
    }
    else
    {
        // Normal (Estável): Verde Aceso Sólido
        set_neopixel_color(0, 100, 0);
    }
}

// --- Funções Auxiliares (Display/Sensor) ---
bool check_movement(int16_t ax, int16_t ay, int16_t az)
{
    int16_t delta_x = abs(ax - last_ax);
    int16_t delta_y = abs(ay - last_ay);
    int16_t delta_z = abs(az - last_az);
    return (delta_x > MOVEMENT_THRESHOLD) || (delta_y > MOVEMENT_THRESHOLD) || (delta_z > MOVEMENT_THRESHOLD);
}

void update_display_state(float angle, bool alarm, bool stable, bool movement)
{
    char state_buffer[32];
    display_text_no_clear("                ", 0, 40, 1);

    if (alarm)
    {
        snprintf(state_buffer, sizeof(state_buffer), "ALERTA! %.1f", angle);
        display_text_no_clear(state_buffer, 0, 40, 1);
        display_text_no_clear("*** PERIGO ***", 0, 50, 1);
    }
    else if (movement)
    {
        snprintf(state_buffer, sizeof(state_buffer), "MOV: %.1f", angle);
        display_text_no_clear(state_buffer, 0, 40, 1);
        display_text_no_clear("Movimento Detectado", 0, 50, 1);
    }
    else
    {
        snprintf(state_buffer, sizeof(state_buffer), "ESTAVEL: %.1f", angle);
        display_text_no_clear(state_buffer, 0, 40, 1);
        display_text_no_clear("Sistema Normal", 0, 50, 1);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== Sistema BitDogLab - Monitor NeoPixel ===\n");

    mpu6050_init();
    servo_init();
    display_init();
    leds_init_neopixel(); // Inicializa o PIO para NeoPixel
    pwm_init_buzzer(BUZZER_PIN);

    int16_t ax, ay, az;
    float angle;
    uint angle_servo;

    mpu6050_read_raw(&last_ax, &last_ay, &last_az);
    last_movement_time = to_ms_since_boot(get_absolute_time());

    while (true)
    {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());

        mpu6050_read_raw(&ax, &ay, &az);
        angle = mpu6050_get_inclination(ax, ay, az);

        bool movement_detected = check_movement(ax, ay, az);
        if (movement_detected)
        {
            last_movement_time = current_time;
            last_ax = ax;
            last_ay = ay;
            last_az = az;
        }

        bool is_stable = (current_time - last_movement_time > STABLE_TIME_MS);
        bool alarm_triggered = (fabs(angle) > ANGLE_ALERT_THRESHOLD);

        // Controle do Servo
        angle_servo = (uint)((angle + 90.0f) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / 180.0f);
        if (angle_servo > SERVO_MAX_ANGLE)
            angle_servo = SERVO_MAX_ANGLE;
        servo_set_angle(angle_servo);

        // --- NOVA LÓGICA DE LEDS ---
        update_neopixel_state(alarm_triggered, movement_detected, is_stable, current_time);

        // Atualização do display e Log
        char main_buffer[32];
        snprintf(main_buffer, sizeof(main_buffer), "ANG: %.1f", angle);
        display_text(main_buffer, 0, 24, 2);
        update_display_state(angle, alarm_triggered, is_stable, movement_detected);

        // Alarme sonoro
        if (alarm_triggered)
        {
            beep(BUZZER_PIN, 100);
        }

        show_display();

        // Delay reduzido para garantir que a piscagem do LED seja fluida
        // Como controlamos o tempo dentro da função update_neopixel_state,
        // podemos rodar o loop um pouco mais rápido.
        sleep_ms(50);
    }
}