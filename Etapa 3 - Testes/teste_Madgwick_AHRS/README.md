# Projeto MPU6050 com Filtro de Madgwick

Este projeto implementa um sistema de leitura de dados do sensor MPU6050 (acelerômetro e giroscópio) usando o filtro de Madgwick para calcular ângulos de orientação (roll, pitch, yaw) em tempo real.

## Componentes

### Hardware
- Raspberry Pi Pico
- MPU6050 (acelerômetro + giroscópio)
- Conexões I2C:
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
