# Sistema AHRS com MPU6050 e Filtro de Madgwick - Versão Simplificada

Este projeto implementa um sistema AHRS (Attitude and Heading Reference System) usando o sensor MPU6050 e o filtro de Madgwick no Raspberry Pi Pico, com arquitetura modular e main simplificado.

## Estrutura do Projeto

### Arquivos Principais

- **`main.c`**: Arquivo principal simplificado - contém apenas a inicialização e chamada do driver
- **`imu_driver.h/c`**: Driver completo do IMU com todas as funcionalidades AHRS
- **`mpu6050_i2c.h/c`**: Driver de baixo nível para comunicação I2C com o MPU6050
- **`MadgwickAHRS.h/c`**: Implementação do filtro de Madgwick
- **`exemplo_uso_avancado.c`**: Exemplos de uso mais avançado do driver

### Como o Sistema Funciona

1. **Inicialização**: O sistema configura o MPU6050, calibra os sensores e inicializa o filtro de Madgwick
2. **Aquisição**: Dados são coletados a 100Hz em timers precisos
3. **Processamento**: O filtro de Madgwick converte dados brutos em ângulos de Euler (Roll, Pitch, Yaw)
4. **Saída**: Os dados são exibidos continuamente via USB serial

## Uso Básico

O main foi simplificado para ser o mais direto possível:

```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "imu_driver.h"

int main() 
{
    stdio_init_all();                    // Inicializa comunicação serial
    printf("=== Sistema AHRS ===\n");
    
    imu_init();                          // Inicializa e calibra o IMU
    imu_start();                         // Inicia aquisição (loop infinito)
    
    return 0;
}
```

## API do Driver IMU

### Funções Principais

- **`imu_init()`**: Inicializa o hardware e calibra o sensor
- **`imu_start()`**: Inicia a aquisição contínua (loop infinito)
- **`imu_stop()`**: Para a aquisição de dados
- **`imu_get_data()`**: Retorna os dados atuais (para uso avançado)
- **`imu_print_data()`**: Imprime os dados uma vez

### Estrutura de Dados

```c
typedef struct {
    float roll;        // Ângulo de rolagem em graus
    float pitch;       // Ângulo de arfagem em graus  
    float yaw;         // Ângulo de guinada em graus
    bool data_ready;   // Indica se os dados são válidos
} imu_data_t;
```

## Uso Avançado

Para casos onde você precisa de mais controle, veja o arquivo `exemplo_uso_avancado.c` que demonstra:

- Como acessar dados pontuais
- Como implementar detecção de movimento
- Como criar seu próprio loop de controle
- Como processar os dados de forma personalizada

## Conectar o MPU6050

- **VCC** → 3V3 do Pico
- **GND** → GND do Pico  
- **SDA** → GPIO 4 (pino 6)
- **SCL** → GPIO 5 (pino 7)

## Compilação e Execução

1. Compile o projeto: Use a task "Compile Project" no VS Code
2. Grave no Pico: Use a task "Run Project" no VS Code
3. Monitore a saída: Abra o monitor serial para ver os dados de orientação

## Características do Sistema

- **Taxa de amostragem**: 100Hz (10ms entre amostras)
- **Precisão**: ±2g para acelerômetro, ±250°/s para giroscópio
- **Calibração automática**: Calcula e corrige bias dos sensores
- **Inicialização inteligente**: Estima orientação inicial baseada no acelerômetro
- **Filtro adaptativo**: Beta alto para convergência rápida, depois reduzido para estabilidade
- **Dual-core**: Usa Core 0 para processamento e Core 1 para exibição

## Saída Típica

```
=== Sistema AHRS com MPU6050 ===
Inicializando IMU...
Calibrando sensor, aguarde...
Bias acelerômetro: X=-123 Y=45 Z=678
Bias giroscópio:   X=12 Y=-34 Z=56
Sistema iniciado - Filtro de Madgwick com MPU6050
Aguarde alguns segundos para estabilização...

Roll:  -2.34°, Pitch:   1.23°, Yaw:  45.67°
Roll:  -2.35°, Pitch:   1.24°, Yaw:  45.68°
...
```
  - SDA: GPIO 2
  - SCL: GPIO 3

### Software
- **main.c**: Programa principal que integra o MPU6050 com o filtro de Madgwick
- **mpu6050_i2c.c/h**: Biblioteca para comunicação I2C com o MPU6050
- **madgwickFilter.c/h**: Implementação do filtro de Madgwick para fusão sensorial

## Funcionalidades

### Configurações do MPU6050
- **Acelerômetro**: ±2g (máxima sensibilidade)
- **Giroscópio**: ±250°/s (máxima sensibilidade)
- **Frequência de amostragem**: 100Hz (10ms entre leituras)

### Filtro de Madgwick
- Fusão de dados do acelerômetro e giroscópio
- Estimativa de orientação usando quaternions
- Conversão para ângulos de Euler (roll, pitch, yaw)
- Taxa de atualização: 100Hz (DELTA_T = 0.01s)

## Como Usar

1. **Compilação**:
   ```
   Use a task "Compile Project" no VS Code
   ```

2. **Upload para o Pico**:
   ```
   Use a task "Run Project" no VS Code
   ou copie o arquivo ACELEROMETRO.uf2 para o Pico em modo bootsel
   ```

3. **Saída Serial**:
   O programa imprime continuamente os ângulos calculados:
   ```
   Roll:  12.34°, Pitch:  -5.67°, Yaw:   89.01°
   ```

## Teoria do Filtro de Madgwick

O filtro de Madgwick é um algoritmo de fusão sensorial que combina:
- **Acelerômetro**: Fornece referência de gravidade (orientação absoluta)
- **Giroscópio**: Fornece velocidade angular (mudanças rápidas)

### Vantagens
- Computacionalmente eficiente
- Boa resposta dinâmica
- Reduz drift do giroscópio
- Filtra ruído do acelerômetro

### Parâmetros Importantes
- **BETA**: Ganho do filtro (√(3/4) * erro_médio_gyro)
- **DELTA_T**: Período de amostragem (0.01s = 100Hz)

## Ângulos de Euler

- **Roll**: Rotação em torno do eixo X (inclinação lateral)
- **Pitch**: Rotação em torno do eixo Y (inclinação frontal/traseira)  
- **Yaw**: Rotação em torno do eixo Z (direção/azimute)

## Calibração

Para melhor precisão, é recomendado:
1. Manter o sensor parado por alguns segundos na inicialização
2. O filtro converge automaticamente para a orientação correta
3. Para aplicações críticas, implementar calibração de offset dos sensores

## Conexões de Hardware

```
MPU6050    ->    Raspberry Pi Pico
VCC        ->    3.3V
GND        ->    GND
SDA        ->    GPIO 2
SCL        ->    GPIO 3
```

## Dependências

- Pico SDK
- Hardware I2C library
- Math library (libm)
