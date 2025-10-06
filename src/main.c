#include <stdio.h>
#include "pico/stdlib.h"
#include "mpu6050/mpu6050.h"
#include "servo/servo.h"
#include "display/display.h"

#define ANGLE_ALERT_THRESHOLD 60.0f // Alerta se o ângulo exceder este valor
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

int main()
{
    // Inicialização
    stdio_init_all();
    mpu6050_init();
    servo_init();
    display_init();

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
        }
        show_display();

        sleep_ms(1000); // Aguarda 1000 milissegundos antes da próxima leitura
    }
}
