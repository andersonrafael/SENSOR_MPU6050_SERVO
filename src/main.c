#include <stdio.h>
#include <math.h> // Adicionado para a função fabs()
#include "pico/stdlib.h"
#include "hardware/pwm.h" // Adicionado para controlar o buzzer
#include "mpu6050/mpu6050.h"
#include "servo/servo.h"
#include "display/display.h"

#define ANGLE_ALERT_THRESHOLD 60.0f // Alerta se o ângulo exceder este valor
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define BUZZER_PIN 21 // Pino do buzzer, conforme seu exemplo

// --- Funções do Buzzer (copiadas do seu exemplo buzzer.c) ---

/**
 * @brief Inicializa o pino do buzzer para operar com PWM.
 *
 * @param pin O número do GPIO ao qual o buzzer está conectado.
 */
void pwm_init_buzzer(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);

    // Configura o PWM com uma frequência base (ex: 1kHz)
    // A frequência real será ajustada pelo duty cycle depois
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 12500); // Frequência = 125MHz / (10 * 12500) = 1kHz
    pwm_config_set_clkdiv(&config, 10);
    pwm_init(slice_num, &config, true);

    pwm_set_gpio_level(pin, 0); // Começa desligado
}

/**
 * @brief Aciona um bipe no buzzer com duração específica.
 *
 * @param pin O número do GPIO do buzzer.
 * @param duration_ms Duração do bipe em milissegundos.
 */
void beep(uint pin, uint duration_ms)
{
    // Coloca o duty cycle em 50% para gerar som (6250 de 12500)
    pwm_set_gpio_level(pin, 6250);
    sleep_ms(duration_ms);

    // Desliga o som (duty cycle 0%)
    pwm_set_gpio_level(pin, 0);
}

// --- Fim das Funções do Buzzer ---

int main()
{
    // Inicialização
    stdio_init_all();
    mpu6050_init();
    servo_init();
    display_init();
    pwm_init_buzzer(BUZZER_PIN); // Inicializa o pino do buzzer

    int16_t ax, ay, az;
    float angle;      // Angulo medido pelo MPU6050
    uint angle_servo; // Angulo a ser enviado ao servo
    char buffer[64];

    while (true)
    {
        mpu6050_read_raw(&ax, &ay, &az);
        angle = mpu6050_get_inclination(ax, ay, az);

        // Mapeia o ângulo do MPU6050 para o intervalo do servo
        angle_servo = (uint)((angle + 90.0f) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / 180.0f);
        if (angle_servo > SERVO_MAX_ANGLE)
            angle_servo = SERVO_MAX_ANGLE;
        servo_set_angle(angle_servo);

        // Exibe dados no terminal
        printf("A-X: %d | A-Y: %d | A-Z: %d | POS.ANG.: %.2f ° | POS.SERVO: %u\n", ax, ay, az, angle, angle_servo);

        // Exibe dados no display
        snprintf(buffer, sizeof(buffer), "POS.:%.1f", angle);
        display_text(buffer, 0, 24, 2);

        // Alerta
        if (fabs(angle) > ANGLE_ALERT_THRESHOLD)
        {
            display_text_no_clear("ALERT!", 20, 0, 2);
            beep(BUZZER_PIN, 200); // MODIFICADO: Aciona um bipe de 200ms
        }
        show_display();

        sleep_ms(1000); // Aguarda 1000 milissegundos antes da próxima leitura
    }
}