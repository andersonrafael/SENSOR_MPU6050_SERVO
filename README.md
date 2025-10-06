# Sistema de Controle de Servo Motor com MPU6050

## Descrição

Este projeto implementa um sistema de controle de servo motor baseado na inclinação medida pelo sensor MPU6050. O sistema utiliza um Raspberry Pi Pico (RP2040) para ler dados do acelerômetro, calcular o ângulo de inclinação e controlar um servo motor SG90 em tempo real. Os dados são exibidos tanto no monitor serial quanto em um display OLED SSD1306.

## Componentes Utilizados

- **Raspberry Pi Pico (RP2040)**: Microcontrolador principal
- **MPU6050**: Sensor de aceleração e giroscópio
- **Servo Motor SG90**: Motor servo para controle de posição
- **Display OLED SSD1306**: Display I2C para visualização dos dados

## Funcionalidades

### 1. Leitura do Sensor MPU6050
- Leitura contínua dos eixos X, Y e Z do acelerômetro
- Cálculo do ângulo de inclinação em relação ao eixo X
- Conversão dos valores brutos para unidades de gravidade (g)

### 2. Controle do Servo Motor
- Mapeamento do ângulo de inclinação (-90° a +90°) para o range do servo (0° a 180°)
- Controle PWM preciso com frequência de 50Hz
- Pulso de largura variável entre 500μs e 2500μs

### 3. Interface de Visualização
- **Monitor Serial**: Exibe valores brutos dos eixos, ângulo calculado e posição do servo
- **Display OLED**: Mostra o ângulo atual em tempo real
- **Sistema de Alerta**: Exibe "ALERTA!" quando o ângulo ultrapassa 60°

## Conexões dos Pinos

### MPU6050 (I2C0)
- **SDA**: GPIO 0
- **SCL**: GPIO 1
- **VCC**: 3.3V
- **GND**: GND

### Servo Motor SG90
- **Sinal**: GPIO 2
- **VCC**: 5V
- **GND**: GND

### Display OLED SSD1306 (I2C1)
- **SDA**: GPIO 14
- **SCL**: GPIO 15
- **VCC**: 3.3V
- **GND**: GND

## Estrutura do Projeto

```
MPU6050-e-servo/
├── CMakeLists.txt              # Configuração do build
├── pico_sdk_import.cmake       # Importação do SDK
├── src/
│   ├── main.c                  # Programa principal
│   └── drivers/
│       ├── mpu6050/
│       │   ├── mpu6050.c      # Driver do sensor MPU6050
│       │   └── mpu6050.h      # Header do driver
│       ├── servo/
│       │   ├── servo.c         # Driver do servo motor
│       │   └── servo.h         # Header do driver
│       └── display/
│           ├── display.c       # Driver do display OLED
│           ├── display.h       # Header do driver
│           ├── ssd1306.c      # Driver SSD1306
│           ├── ssd1306.h      # Header SSD1306
│           └── font.h         # Fontes para o display
```

## Como Usar

### 1. Compilação
```bash
mkdir build
cd build
cmake ..
make
```

### 2. Upload para o Pico
```bash
cp u3-i4-mpu6050.uf2 /media/[USER]/RPI-RP2/
```

### 3. Monitoramento
Conecte o Pico via USB e abra o monitor serial:
```bash
minicom -D /dev/ttyACM0 -b 115200
```

## Funcionamento do Programa

### Loop Principal
1. **Leitura do Sensor**: Lê valores brutos dos eixos X, Y, Z
2. **Cálculo do Ângulo**: Converte valores para ângulo de inclinação
3. **Mapeamento**: Mapeia ângulo para posição do servo (0-180°)
4. **Controle do Servo**: Define posição do servo via PWM
5. **Exibição**: Mostra dados no serial e display OLED
6. **Alerta**: Verifica se ângulo ultrapassa limite de 60°

### Parâmetros Configuráveis
- `ANGLE_ALERT_THRESHOLD`: 60.0° (limite para alerta)
- `SERVO_MIN_ANGLE`: 0° (posição mínima do servo)
- `SERVO_MAX_ANGLE`: 180° (posição máxima do servo)
- Loop delay: 150ms (frequência de atualização)

## Características Técnicas

### MPU6050
- **Endereço I2C**: 0x68
- **Frequência I2C**: 400kHz
- **Faixa de Medição**: ±2g
- **Resolução**: 16 bits

### Servo Motor
- **Frequência PWM**: 50Hz (período 20ms)
- **Pulso Mínimo**: 500μs (0°)
- **Pulso Máximo**: 2500μs (180°)
- **Pino de Controle**: GPIO 2

### Display OLED
- **Resolução**: 128x64 pixels
- **Endereço I2C**: 0x3C
- **Frequência I2C**: 400kHz

## Saída de Dados

### Monitor Serial
```
AX: 1234 | AY: -567 | AZ: 8901 | Angulo: 15.23 | Servo: 105
```

### Display OLED
- **Linha 1**: Ângulo atual (ex: "Ang.:15.2")
- **Linha 2**: "ALERTA!" (quando aplicável)

## Aplicações

Este sistema pode ser utilizado em:
- **Robótica**: Controle de braços robóticos
- **Estabilização**: Sistemas de estabilização de plataformas
- **Educação**: Demonstração de sensores e atuadores
- **Prototipagem**: Base para projetos mais complexos

## Dependências

- Raspberry Pi Pico SDK
- Hardware I2C (para MPU6050 e display)
- Hardware PWM (para servo motor)
- Biblioteca matemática (para cálculos trigonométricos)

## Licença

Este projeto é fornecido como exemplo educacional. Sinta-se livre para modificar e adaptar conforme suas necessidades. 