#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "mpu6050/mpu6050.h"
#include "servo/servo.h"
#include "display/display.h"

#define ANGLE_ALERT_THRESHOLD 60.0f
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define BUZZER_PIN 21 // Pino do buzzer (comum na BitDogLab)

// --- PINOS DOS LEDs RGB INTEGRADOS DA BITDOGLAB ---
#define LED_RED_PIN   16  // LED Vermelho do RGB integrado
#define LED_GREEN_PIN 17  // LED Verde do RGB integrado
#define LED_BLUE_PIN  25  // LED Azul do RGB integrado
// Usaremos: Verde=estável, Azul=movimento, Vermelho=alarme

// --- Constantes para controle ---
#define MOVEMENT_THRESHOLD 300   // Limite para movimento (ajustável)
#define STABLE_TIME_MS 3000      // Tempo para considerar estabilizado
#define BLINK_INTERVAL_MS 400    // Intervalo base de piscagem

// Para alarme, piscamos mais rápido
#define ALARM_BLINK_INTERVAL_MS 150

// --- Variáveis de estado ---
static uint64_t last_movement_time = 0;
static uint64_t last_blink_time = 0;
static bool led_state = false;
static int16_t last_ax = 0, last_ay = 0, last_az = 0;
static uint64_t last_stable_check = 0;

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

// --- Funções para LEDs da BitDogLab ---
void leds_init_bitdoglab(void)
{
    // Inicializa os pinos dos LEDs RGB
    gpio_init(LED_RED_PIN);
    gpio_init(LED_GREEN_PIN);
    gpio_init(LED_BLUE_PIN);
    
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    
    // Desliga todos os LEDs inicialmente
    gpio_put(LED_RED_PIN, 0);
    gpio_put(LED_GREEN_PIN, 0);
    gpio_put(LED_BLUE_PIN, 0);
    
    // Pisca uma vez para indicar inicialização
    gpio_put(LED_RED_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_RED_PIN, 0);
    gpio_put(LED_GREEN_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_GREEN_PIN, 0);
    gpio_put(LED_BLUE_PIN, 1);
    sleep_ms(100);
    gpio_put(LED_BLUE_PIN, 0);
}

/**
 * @brief Verifica se houve movimento significativo
 */
bool check_movement(int16_t ax, int16_t ay, int16_t az)
{
    int16_t delta_x = abs(ax - last_ax);
    int16_t delta_y = abs(ay - last_ay);
    int16_t delta_z = abs(az - last_az);
    
    return (delta_x > MOVEMENT_THRESHOLD) || 
           (delta_y > MOVEMENT_THRESHOLD) || 
           (delta_z > MOVEMENT_THRESHOLD);
}

/**
 * @brief Controla LEDs baseado no estado
 */
void control_leds_state(bool alarm, bool movement, bool stable, uint64_t current_time)
{
    static uint64_t last_led_update = 0;
    static uint8_t blink_pattern = 0;
    
    // Para alarme: pisca rápida
    uint32_t blink_interval = alarm ? ALARM_BLINK_INTERVAL_MS : BLINK_INTERVAL_MS;
    
    // Atualiza estado de piscagem
    if (current_time - last_led_update >= blink_interval) {
        led_state = !led_state;
        blink_pattern = (blink_pattern + 1) % 4; // Padrão de 4 fases
        last_led_update = current_time;
    }
    
    // Desliga todos os LEDs primeiro
    gpio_put(LED_RED_PIN, 0);
    gpio_put(LED_GREEN_PIN, 0);
    gpio_put(LED_BLUE_PIN, 0);
    
    if (alarm) {
        // ESTADO ALARME: LED Vermelho pisca rapidamente
        gpio_put(LED_RED_PIN, led_state);
        
        // Opcional: fazer efeito de piscagem especial
        if (blink_pattern == 0) {
            gpio_put(LED_RED_PIN, 1);
        } else if (blink_pattern == 2) {
            gpio_put(LED_RED_PIN, 1);
        }
        
    } else if (movement && !stable) {
        // ESTADO MOVIMENTO: LED Azul pisca
        gpio_put(LED_BLUE_PIN, led_state);
        
    } else if (stable) {
        // ESTADO ESTÁVEL: LED Verde pisca lentamente
        // Pisca apenas a cada 2 ciclos para ser mais lento
        bool slow_blink = (blink_pattern == 0 || blink_pattern == 1);
        gpio_put(LED_GREEN_PIN, slow_blink && led_state);
        
    } else {
        // ESTADO INTERMEDIÁRIO: LED Verde fraco (50% brilho)
        // Usamos um PWM simples por software
        static bool dim_state = false;
        if (current_time % 1000 < 500) {
            dim_state = !dim_state;
        }
        gpio_put(LED_GREEN_PIN, dim_state);
    }
}

/**
 * @brief Atualiza display com informações do estado
 */
void update_display_state(float angle, bool alarm, bool stable, bool movement)
{
    char state_buffer[32];
    
    // Limpa área de status
    display_text_no_clear("                ", 0, 40, 1);
    
    if (alarm) {
        snprintf(state_buffer, sizeof(state_buffer), "ALERTA! %.1f°", angle);
        display_text_no_clear(state_buffer, 0, 40, 1);
        display_text_no_clear("*** PERIGO ***", 0, 50, 1);
    } else if (movement) {
        snprintf(state_buffer, sizeof(state_buffer), "MOV: %.1f°", angle);
        display_text_no_clear(state_buffer, 0, 40, 1);
        display_text_no_clear("Movimento Detectado", 0, 50, 1);
    } else if (stable) {
        snprintf(state_buffer, sizeof(state_buffer), "ESTAVEL: %.1f°", angle);
        display_text_no_clear(state_buffer, 0, 40, 1);
        display_text_no_clear("Sistema OK", 0, 50, 1);
    } else {
        snprintf(state_buffer, sizeof(state_buffer), "MED: %.1f°", angle);
        display_text_no_clear(state_buffer, 0, 40, 1);
        display_text_no_clear("Monitorando...", 0, 50, 1);
    }
}

int main()
{
    // Inicialização
    stdio_init_all();
    sleep_ms(2000); // Aguarda inicialização da serial
    
    printf("\n=== Sistema BitDogLab - Monitor de Inclinacao ===\n");
    printf("LEDs: Verde=Estavel, Azul=Movimento, Vermelho=Alarme\n");
    
    mpu6050_init();
    servo_init();
    display_init();
    leds_init_bitdoglab(); // LEDs específicos da BitDogLab
    pwm_init_buzzer(BUZZER_PIN);

    int16_t ax, ay, az;
    float angle;
    uint angle_servo;
    
    // Leitura inicial
    mpu6050_read_raw(&last_ax, &last_ay, &last_az);
    last_movement_time = to_ms_since_boot(get_absolute_time());

    while (true)
    {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Leitura do sensor
        mpu6050_read_raw(&ax, &ay, &az);
        angle = mpu6050_get_inclination(ax, ay, az);
        
        // Verifica movimento
        bool movement_detected = check_movement(ax, ay, az);
        if (movement_detected) {
            last_movement_time = current_time;
            last_ax = ax;
            last_ay = ay;
            last_az = az;
        }
        
        // Verifica estabilidade
        bool is_stable = (current_time - last_movement_time > STABLE_TIME_MS);
        bool alarm_triggered = (fabs(angle) > ANGLE_ALERT_THRESHOLD);
        
        // Controle do servo
        angle_servo = (uint)((angle + 90.0f) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / 180.0f);
        if (angle_servo > SERVO_MAX_ANGLE) angle_servo = SERVO_MAX_ANGLE;
        servo_set_angle(angle_servo);
        
        // Controle dos LEDs
        control_leds_state(alarm_triggered, movement_detected, is_stable, current_time);
        
        // Atualização do display
        char main_buffer[32];
        snprintf(main_buffer, sizeof(main_buffer), "ANG: %.1f°", angle);
        display_text(main_buffer, 0, 24, 2);
        
        update_display_state(angle, alarm_triggered, is_stable, movement_detected);
        
        // Log no terminal
        printf("AX:%6d AY:%6d AZ:%6d | Ang: %6.1f° | ", ax, ay, az, angle);
        if (alarm_triggered) {
            printf("ALARME! ");
        } else if (movement_detected) {
            printf("MOVIMENTO ");
        } else if (is_stable) {
            printf("ESTAVEL ");
        } else {
            printf("NORMAL ");
        }
        printf("| Servo: %3u°\n", angle_servo);
        
        // Alarme sonoro
        if (alarm_triggered) {
            beep(BUZZER_PIN, 100); // Bipe curto a cada ciclo durante alarme
        }
        
        show_display();
        
        // Intervalo de leitura
        sleep_ms(150); // 150ms para bom balanceamento entre responsividade e estabilidade
    }
}