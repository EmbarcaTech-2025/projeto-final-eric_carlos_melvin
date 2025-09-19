# Sistema de Monitoramento Ativo para Reabilitação de Quadril

![Status](https://img.shields.io/badge/Status-Ativo-green)
![Versão](https://img.shields.io/badge/Versão-0.1-blue)
![Plataforma](https://img.shields.io/badge/Plataforma-Raspberry%20Pi%20Pico%20W-red)
![Linguagem](https://img.shields.io/badge/Linguagem-C%2FC%2B%2B-orange)

## 📋 Sobre o Projeto

Sistema embarcado de monitoramento postural em tempo real desenvolvido para auxiliar no processo de reabilitação de quadril. Utilizando a placa **BitDogLab (Raspberry Pi Pico W)**, o sistema captura dados de sensores inerciais, detecta posturas perigosas e fornece feedback imediato através de alarmes sonoros e registro detalhado de dados.

### 👥 Autores
- **Eric Senne Roma**
- **Carlos Fernando Mattos do Amaral**
- **Melvin Gustavo Maradiaga Elvir**

### 📅 Data de Criação
19 de Setembro de 2025

---

## 🎯 Objetivo

Desenvolver um sistema de monitoramento que auxilie pacientes em reabilitação de quadril, fornecendo:

- ✅ **Detecção automática** de posturas perigosas
- 🔊 **Alarme sonoro imediato** em situações de risco
- 📊 **Registro histórico** de eventos para acompanhamento médico
- 🎮 **Interface de controle** através de botões físicos
- 💾 **Armazenamento de dados** em cartão SD com timestamps precisos

---

## 🛠️ Lista de Materiais

| Componente | Conexão na BitDogLab | Descrição |
|------------|---------------------|-----------|
| **BitDogLab (RP2040)** | - | Microcontrolador principal |
| **MPU9250 (Tronco)** | I2C1: SDA GPIO2 / SCL GPIO3 | Sensor inercial para pelve/tronco |
| **MPU9250 (Coxa)** | I2C1: SDA GPIO2 / SCL GPIO3 | Sensor inercial para coxa |
| **RTC DS3231** | I2C0: SDA GPIO0 / SCL GPIO1 | Relógio de tempo real |
| **Cartão SD** | SPI0: MISO GPIO16 / MOSI GPIO19 / SCK GPIO18 / CS GPIO17 | Armazenamento de dados |
| **Buzzer** | GPIO21 (PWM) | Alarme sonoro |
| **Botão A** | GPIO5 | Controle de silenciar/desilenciar alarme |

---

## 📁 Estrutura do Projeto

```
projeto_final/
├── 📄 main.cpp                    # Código principal do sistema
├── 📄 CMakeLists.txt              # Configuração de build
├── 📄 pico_sdk_import.cmake       # Importação do SDK Pico
├── 📁 src/                        # Códigos fonte principais
│   ├── analise_postural.cpp       # Análise postural e controle de alarme
│   └── evento.cpp                 # Gerenciamento de eventos
├── 📁 inc/                        # Headers do projeto
│   ├── analise_postural.h
│   ├── evento.h
│   └── estruturas_de_dados.hpp
├── 📁 drivers/                    # Drivers de hardware
│   ├── button/                    # Driver dos botões
│   ├── buzzer/                    # Driver do buzzer
│   ├── mpu9250/                   # Driver dos sensores inerciais
│   ├── madgwick/                  # Filtro Madgwick para orientação
│   ├── postura/                   # Algoritmos de análise postural
│   ├── sdcard/                    # Driver do cartão SD
│   ├── rtc/                       # Driver do RTC
│   └── watchdog/                  # Sistema de watchdog
└── 📁 build/                      # Arquivos de compilação
    └── projeto_final.uf2          # Firmware para upload
```

---

## 🚀 Como Executar

### 1. 🔧 Preparação do Ambiente

- **VS Code** com extensão do Raspberry Pi Pico
- **CMake** (versão 3.13 ou superior)
- **Compilador ARM** (incluído no Pico SDK)
- **Pico SDK** versão 1.5.1

### 2. 📦 Compilação

#### Via VS Code:
```
Ctrl + Shift + B
```

#### Via Terminal:
```bash
cd build
cmake ..
ninja
```

### 3. 📤 Upload para a Placa

1. **Conecte** a BitDogLab via USB
2. **Pressione** o botão `BOOTSEL` e conecte o cabo USB
3. **Copie** o arquivo `build/projeto_final.uf2` para a unidade `RPI-RP2`
4. A placa **reiniciará automaticamente**

### 4. 📍 Posicionamento dos Sensores

- **MPU9250 #1:** Fixar na região da **pelve/tronco** (referência)
- **MPU9250 #2:** Fixar na **coxa** do membro em reabilitação
- ⚠️ **Importante:** Sensores devem estar bem fixados e alinhados

---

## ⚙️ Fluxo de Funcionamento

### 1. 🔄 Inicialização do Sistema
- Configuração de todos os periféricos (I2C, SPI, GPIO, PWM)
- Inicialização dos sensores MPU9250
- Configuração do RTC e cartão SD
- Calibração inicial dos sensores (5 segundos)
- Ativação do sistema de watchdog

### 2. 📊 Monitoramento Contínuo
- **Aquisição de Dados:** Leitura dos sensores inerciais a 100Hz
- **Processamento:** Aplicação do filtro Madgwick para obter orientação
- **Cálculo de Ângulos:** Determinação dos ângulos de flexão, abdução e rotação do quadril
- **Análise de Risco:** Verificação se os ângulos ultrapassam limites seguros

### 3. ⚠️ Detecção de Postura Perigosa
- **Abertura de Evento:** Quando um ângulo ultrapassa o limite seguro
- **Ativação do Alarme:** Buzzer emite sinal sonoro de alerta
- **Registro Contínuo:** Atualização do ângulo máximo atingido durante o evento

### 4. 🎛️ Gerenciamento de Alarme
- **Controle Manual:**
  - `Botão A`: Silenciar/desilenciar alarme durante evento ativo
  - Alarme permanece ligado até que a postura seja corrigida
- **Desativação Automática:** Quando todos os ângulos retornam aos limites seguros

### 5. 💾 Registro de Dados
- **Formato de Dados:** Cada evento é salvo no cartão SD:
  ```csv
  Timestamp,Lado,TipoMovimento,AnguloMaximo
  2025-01-15T14:30:45Z,direita,Flexão,95.2
  ```
- **Arquivo:** `eventos.csv` na raiz do cartão SD
- **Backup de Segurança:** Sistema de watchdog previne perda de dados

### 6. 🛡️ Monitoramento de Sistema
- **Watchdog:** Reinicia o sistema em caso de travamento
- **Feedback de Status:** LEDs e mensagens seriais indicam estado do sistema
- **Recuperação Automática:** Sistema retoma operação após reinicialização

---

## ⚙️ Configurações Técnicas

### 📡 Sensores MPU9250
| Parâmetro | Valor |
|-----------|-------|
| Taxa de Amostragem | 100Hz |
| Acelerômetro | ±2g |
| Giroscópio | ±250°/s |
| Filtro Digital | 41Hz passa-baixa |
| Endereços I2C | 0x68 (tronco) e 0x69 (coxa) |

### 🚨 Limites de Segurança
| Movimento | Limite Máximo |
|-----------|---------------|
| **Flexão do Quadril** | 90° |
| **Abdução do Quadril** | 45° |
| **Rotação do Quadril** | 45° |

### 💾 Sistema de Arquivos
| Especificação | Valor |
|---------------|-------|
| **Formato** | FAT32 |
| **Arquivo Principal** | `eventos.csv` |
| **Timestamp** | ISO 8601 (YYYY-MM-DDTHH:MM:SSZ) |

---

## 🔬 Funcionalidades

- 🧮 **Filtro Madgwick:** Fusão de sensores para orientação precisa
- 🎯 **Detecção de Movimento:** Diferenciação entre repouso e movimento ativo
- 📋 **Sistema de Eventos:** Rastreamento individual de cada tipo de movimento perigoso
- 🛡️ **Resistência a Falhas:** Watchdog e recuperação automática
- 🔌 **Interface Serial:** Debug e monitoramento via USB

---

## 🔧 Dependências

### Hardware
- Raspberry Pi Pico W (BitDogLab)
- Sensores MPU9250 (2x)
- RTC DS3231
- Cartão SD (FAT32)
- Buzzer
- Botões físicos

### Software
- Pico SDK v1.5.1
- CMake 3.13+
- Biblioteca FatFs_SPI
- Filtro Madgwick AHRS

---

## 📊 Dados de Saída

### Formato do Arquivo CSV
```csv
Timestamp,Lado,TipoMovimento,AnguloMaximo
2025-09-19T14:30:45Z,direita,Flexão,95.2
2025-09-19T14:31:12Z,esquerda,Abdução,47.8
2025-09-19T14:32:03Z,direita,Rotação,48.1
```

### Campos Explicados
- **Timestamp:** Data e hora do evento (ISO 8601)
- **Lado:** Membro monitorado (direita/esquerda)
- **TipoMovimento:** Tipo de movimento perigoso detectado
- **AnguloMaximo:** Maior ângulo atingido durante o evento (graus)

---


## 📄 Licença

Este projeto está sob a licença MIT. Veja o arquivo `LICENSE` para mais detalhes.

---

<div align="center">

![Raspberry Pi Pico](https://img.shields.io/badge/Powered%20by-Raspberry%20Pi%20Pico-red?style=for-the-badge&logo=raspberry-pi)

</div>