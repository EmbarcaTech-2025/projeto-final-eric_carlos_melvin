# Desafios Encontrados e Melhorias Planejadas

Este projeto envolveu a utilização de sensores MPU6050/MPU9250 para análise postural, e durante o desenvolvimento enfrentamos alguns desafios importantes. A seguir, descrevemos os problemas identificados e as soluções ou melhorias planejadas.

## 1. Drift nos sensores MPU6050

**Problema:** Os valores de orientação aumentavam com o tempo devido ao drift (erro acumulado do giroscópio).

**Solução:** Utilizamos a biblioteca MadgwickAHRS, que representa a orientação por quaternions e corrige o drift automaticamente.

## 2. Baixa confiabilidade do Yaw

**Problema:** O MPU6050 não possui magnetômetro, tornando o valor de Yaw pouco confiável quando convertido para ângulos de Euler (Roll, Pitch, Yaw).

**Solução:** Decidimos comprar e migrar para o sensor MPU9250, que possui magnetômetro integrado, garantindo medidas de Yaw mais confiáveis.

## 3. Travamento do sistema

**Problema:** O sistema travava após certo tempo, especialmente quando:
- Tocávamos nos sensores durante o movimento;
- Havia movimento nos cabos I2C;
- Utilizávamos simultaneamente os barramentos I2C0 e I2C1.

**Possíveis causas:**
- Ruído ou mau contato nos cabos I2C;
- Flutuações na alimentação elétrica;
- Leituras I2C bloqueantes sem timeout.

**Soluções planejadas:**
- Implementar um Watchdog Timer no loop principal, caso seja necessário e resolva o problema;
- Recuperação do barramento I2C sem desligar a placa:
  - Toggle manual dos pinos SDA/SCL para liberar o barramento;
  - Reinicialização do periférico I2C via SDK (i2c_deinit() + i2c_init());

> Observação: Mesmo com essas medidas, alguns travamentos podem exigir o ciclo de energia completo, mas a combinação de Watchdog e recuperação do barramento deve reduzir as falhas.

## 4. Reconhecimento de posturas corporais

**Problema:** O reconhecimento de posturas apenas pelos ângulos de Euler (Roll, Pitch e Yaw) mostrou-se limitado na aplicação do nosso sistema, que utiliza três sensores MPU6050 (um em cada perna e um no tronco) para monitorar movimentos pós-cirurgia de quadril. Atualmente, conseguimos detectar de forma mais confiável apenas a flexão de quadril, quando o ângulo relativo de pitch entre tronco e perna ultrapassa aproximadamente 75 graus, valor que corresponde à posição em que o sensor está perpendicular. 

**Limitações:**
- Dependência da orientação inicial do usuário;
- Ângulos de Euler isolados não representam completamente a postura relativa.

**Solução planejada:**
- Implementar cálculo de ângulos relativos usando quaternions e matrizes de rotação, permitindo identificar posturas complexas com maior precisão.

**Link do vídeo**: https://youtu.be/VhlXSUJYogY