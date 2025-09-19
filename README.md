# HipSafe - Sistema de Monitoramento Ativo para Reabilitação de Quadril

![Status](https://img.shields.io/badge/Status-Concluído-green)
![Versão](https://img.shields.io/badge/Versão-1.0-blue)
![Plataforma](https://img.shields.io/badge/Plataforma-Raspberry%20Pi%20Pico%20W-red)
![Linguagem](https://img.shields.io/badge/Linguagem-C%2FC%2B%2B-orange)

## 📋 Sobre o Projeto

O **HipSafe** é um sistema embarcado de monitoramento postural em tempo real, desenvolvido para auxiliar pacientes no processo de reabilitação de quadril. Utilizando a placa **BitDogLab (Raspberry Pi Pico W)** e sensores inerciais, o sistema oferece detecção automática de posturas perigosas (restrições pós cirúrgicas), feedback imediato e registro detalhado de dados para acompanhamento médico.

### 🎯 Objetivo Principal

Desenvolver uma solução tecnológica que melhore a segurança e eficácia do processo de reabilitação de quadril, fornecendo:

- ✅ **Monitoramento contínuo** da postura do paciente
- 🚨 **Alertas imediatos** em situações de risco
- 📊 **Registro histórico** para análise médica
- 🎮 **Interface intuitiva** de controle
- 💾 **Armazenamento seguro** de dados com timestamps

### 👥 Equipe de Desenvolvimento (Grupo 4)

- **Eric Senne Roma**
- **Carlos Fernando Mattos do Amaral**
- **Melvin Gustavo Maradiaga Elvir**

**Programa:** Residência em Software Embarcado - EmbarcaTech 2025  
**Data:** Setembro de 2025

---

## 🏗️ Estrutura do Repositório

Este projeto foi desenvolvido em **4 etapas principais**, cada uma documentada em sua respectiva pasta:

### 📁 [Etapa 1](./Etapa%201/) - Definição de Requisitos e Lista de Materiais
- **Documento:** `Projeto_Final_E1.pdf`
- **Conteúdo:** Identificar o problema a ser resolvido, definir os requisitos funcionais e não funcionais da solução, e elaborar a lista inicial de materiais necessários.

### 📁 [Etapa 2](./Etapa%202/) - Arquitetura e Modelagem
- **Documento:** `Projeto_Final_Entrega_2.pdf`
- **Conteúdo:** Apresentar a proposta de arquitetura do sistema, incluindo o diagrama de hardware, os blocos funcionais e o fluxograma do software.

### 📁 [Etapa 3](./Etapa%203/) - Prototipagem e Ajustes
- **Documento:** `README.md`
- **Conteúdo:** Construir o protótipo funcional da solução, realizar testes, e listar os ajustes necessários para a versão final.

### 📁 [Etapa 4](./Etapa%204/) - Implementação Final
- **Subpastas:**
  - `📁 projeto_final/` - Código-fonte completo do sistema | [Projeto Final](./Etapa%204/projeto_final/)
  - `📁 Apresentação/` - Material de apresentação do projeto | [Apresentação](./Etapa%204/Apresentação/)
  - `📁 Esquemas/` - Diagramas técnicos e arquitetura | [Esquemas](./Etapa%204/Esquemas/)
  - `📁 Vídeo/` - Demonstração em vídeo do sistema funcionando | [Vídeo](./Etapa%204/Vídeo/)
---

## 🚀 Quick Start

### 1. 📋 Pré-requisitos
- **Hardware:** BitDogLab (Raspberry Pi Pico W)
- **Software:** VS Code + Extensão Raspberry Pi Pico
- **SDK:** Pico SDK v1.5.1
- **Build:** CMake 3.13+

### 2. ⬇️ Clone do Repositório
```bash
git clone https://github.com/EmbarcaTech-2025/projeto-final-eric_carlos_melvin.git
cd projeto-final-eric_carlos_melvin
```

### 3. 🔧 Compilação e Upload
```bash
cd "Etapa 4/projeto_final"
# Abrir no VS Code ou compilar via terminal
```

> 📖 **Documentação Completa:** Veja o [README detalhado](./Etapa%204/projeto_final/README.md) na pasta do projeto final para instruções completas de instalação, configuração e uso.

---

## ⚙️ Características Técnicas

### 🛠️ Hardware Utilizado
| Componente | Função | Conexão |
|------------|--------|---------|
| **BitDogLab (RP2040)** | Microcontrolador principal | - |
| **2x MPU9250** | Sensores inerciais (tronco e coxa) | I2C1 |
| **RTC DS3231** | Relógio de tempo real | I2C0 |
| **Cartão SD** | Armazenamento de dados | SPI0 |
| **Buzzer** | Alarme sonoro | GPIO21 (PWM) |
| **Botões** | Interface de controle | GPIO5 |

### 📊 Funcionalidades Principais
- **Detecção de Movimento:** Análise de flexão, abdução e rotação do quadril
- **Limites de Segurança:** Flexão (90°), Abdução (45°), Rotação (45°)
- **Filtro Madgwick:** Fusão de sensores para orientação precisa
- **Sistema de Eventos:** Registro automático de posturas perigosas
- **Watchdog Timer:** Proteção contra travamentos do sistema

---

### 🔬 Principais Desafios Superados
- **Drift dos sensores:** Resolvido com filtro Madgwick
- **Confiabilidade do Yaw:** Migração para MPU9250 com magnetômetro
- **Travamentos:** Implementação de watchdog timer
- **Sincronização:** Sistema de timestamps com RTC

### 📊 Métricas de Performance
- **Taxa de amostragem:** 100Hz

---

## 📁 Documentação e Recursos

### 📋 Documentos Técnicos
- [📄 Especificação Completa](./Etapa%201/Projeto_Final_E1.pdf)
- [📄 Arquitetura e Modelagem](./Etapa%202/Projeto_Final_Entrega_2.pdf)
- [📄 Desafios Encontrados](./Etapa%203/README.md)
- [📄 Sistema](./Etapa%204/projeto_final/README.md)

### 🎨 Diagramas e Esquemas
- [🔧 Arquitetura de Software](./Etapa%204/Esquemas/Arquitetura%20de%20Software.png)
- [⚡ Diagrama de Hardware](./Etapa%204/Esquemas/Diagrama%20de%20Hardware.png)
- [🔄 Fluxograma de Software](./Etapa%204/Esquemas/Fluxograma%20de%20Software.png)
- [📊 Diagrama de Blocos Funcionais](./Etapa%204/Esquemas/Diagrama_Blocos_Funcionais.png)

### 🎥 Material de Apresentação
- [📊 Apresentação Final](./Etapa%204/Apresentação/HipSafe%20-%20Apresentação.pptx)
- [🎬 Vídeo Demonstrativo](./Etapa%204/Vídeo/)

---

## 🎯 Conclusões

O **HipSafe** representa um projeto de **alta complexidade** que integrou sensores inerciais com matemática avançada e conceitos de biomecânica para alcançar seus objetivos. Desenvolvemos um driver proprietário para o MPU9250, superamos diversos obstáculos técnicos e implementamos uma arquitetura de software modular e eficiente em C++. A **prova de conceito confirma plenamente a viabilidade** da solução: com refinamento matemático e calibração aprimorada, o sistema é totalmente aplicável. 

Atualmente, o sistema necessita de **aprimoramentos na calibração** para otimizar sua aplicação direta no corpo humano, representando uma oportunidade de desenvolvimento futuro que elevará a precisão e confiabilidade. Este projeto **interdisciplinar** possui ampla aplicabilidade em problemas de orientação espacial corporal, estabelecendo uma base sólida para futuras soluções em monitoramento biomecânico.

---

## 📜 Licença

Este projeto está licenciado sob a **Licença MIT** - veja o arquivo [LICENSE](LICENSE) para detalhes.

---

## 🙏 Agradecimentos

Agradecemos ao **Programa EmbarcaTech 2025** pela oportunidade de desenvolvimento deste projeto, aos mentores pelo suporte técnico e a todos que contribuíram para o sucesso desta iniciativa.

---

<div align="center">

![EmbarcaTech](https://img.shields.io/badge/EmbarcaTech-2025-blue?style=for-the-badge)
![Raspberry Pi Pico](https://img.shields.io/badge/Powered%20by-Raspberry%20Pi%20Pico-red?style=for-the-badge&logo=raspberry-pi)

**HipSafe – Projeto Final | Grupo 4 | Residência Tecnológica em Sistemas Embarcados – 2025 | Campinas**

</div>
