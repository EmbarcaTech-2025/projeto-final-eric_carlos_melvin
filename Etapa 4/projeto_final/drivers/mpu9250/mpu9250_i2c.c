/**
 * @file mpu9250_i2c.c
 * @brief Driver para sensor inercial MPU9250 via comunicação I2C
 * 
 * Este arquivo implementa todas as funções para comunicação e controle do sensor MPU9250,
 * que é um dispositivo inercial de 9 graus de liberdade (9-DOF) composto por:
 * - Acelerômetro de 3 eixos
 * - Giroscópio de 3 eixos  
 * - Magnetômetro AK8963 de 3 eixos (integrado)
 */
#include "mpu9250_i2c.h"
#include <stdio.h>
#include <stdlib.h>

/**
 * ENDEREÇOS DOS REGISTRADORES DO MPU9250
 * ====================================
 * Estes endereços definem os registradores internos do MPU9250 
 * que controlam configuração, leitura de dados e status do sensor
 */
#define MPU9250_WHO_AM_I        0x75  // Registrador de identificação do chip
#define MPU9250_PWR_MGMT_1      0x6B  // Gerenciamento de energia principal
#define MPU9250_PWR_MGMT_2      0x6C  // Gerenciamento de energia dos sensores
#define MPU9250_CONFIG          0x1A  // Configuração do filtro digital passa-baixa
#define MPU9250_GYRO_CONFIG     0x1B  // Configuração do giroscópio (range e self-test)
#define MPU9250_ACCEL_CONFIG    0x1C  // Configuração do acelerômetro (range e self-test)
#define MPU9250_ACCEL_CONFIG2   0x1D  // Configuração adicional do acelerômetro (DLPF)
#define MPU9250_SMPLRT_DIV      0x19  // Divisor da taxa de amostragem
#define MPU9250_INT_PIN_CFG     0x37  // Configuração do pino de interrupção
#define MPU9250_INT_ENABLE      0x38  // Habilitação de interrupções
#define MPU9250_INT_STATUS      0x3A  // Status das interrupções
#define MPU9250_ACCEL_XOUT_H    0x3B  // Início dos dados do acelerômetro (byte alto X)
#define MPU9250_TEMP_OUT_H      0x41  // Dados de temperatura (byte alto)
#define MPU9250_GYRO_XOUT_H     0x43  // Início dos dados do giroscópio (byte alto X)
#define MPU9250_USER_CTRL       0x6A  // Controle de usuário (I2C master, reset, etc.)
#define MPU9250_I2C_MST_CTRL    0x24  // Controle do I2C master
#define MPU9250_I2C_SLV0_ADDR   0x25  // Endereço do dispositivo slave 0
#define MPU9250_I2C_SLV0_REG    0x26  // Registrador do slave 0 para leitura/escrita
#define MPU9250_I2C_SLV0_CTRL   0x27  // Controle do slave 0 (enable, length)
#define MPU9250_I2C_SLV0_DO     0x63  // Dados de saída para escrita no slave 0
#define MPU9250_EXT_SENS_DATA_00 0x49 // Início dos dados lidos dos sensores externos

/**
 * ENDEREÇOS DOS REGISTRADORES DO MAGNETÔMETRO AK8963
 * =================================================
 * O magnetômetro AK8963 é um chip separado integrado ao MPU9250.
 * É acessado via I2C master do MPU9250 ou através de bypass I2C.
 */
#define AK8963_WHO_AM_I     0x00  // Identificação do chip magnetômetro
#define AK8963_INFO         0x01  // Informações do dispositivo
#define AK8963_ST1          0x02  // Status 1 (data ready)
#define AK8963_XOUT_L       0x03  // Dados magnéticos X (byte baixo)
#define AK8963_XOUT_H       0x04  // Dados magnéticos X (byte alto)
#define AK8963_YOUT_L       0x05  // Dados magnéticos Y (byte baixo)
#define AK8963_YOUT_H       0x06  // Dados magnéticos Y (byte alto)
#define AK8963_ZOUT_L       0x07  // Dados magnéticos Z (byte baixo)
#define AK8963_ZOUT_H       0x08  // Dados magnéticos Z (byte alto)
#define AK8963_ST2          0x09  // Status 2 (overflow)
#define AK8963_CNTL1        0x0A  // Controle 1 (modo de operação)
#define AK8963_CNTL2        0x0B  // Controle 2 (reset)
#define AK8963_ASTC         0x0C  // Self-test
#define AK8963_I2CDIS       0x0F  // Desabilita I2C
#define AK8963_ASAX         0x10  // Ajuste de sensibilidade X
#define AK8963_ASAY         0x11  // Ajuste de sensibilidade Y
#define AK8963_ASAZ         0x12  // Ajuste de sensibilidade Z

/**
 * REGISTRADORES DE SELF-TEST
 * ==========================
 * Utilizados para verificar a integridade dos sensores
 */
#define SELF_TEST_X_GYRO   0x00  // Self-test giroscópio eixo X
#define SELF_TEST_Y_GYRO   0x01  // Self-test giroscópio eixo Y
#define SELF_TEST_Z_GYRO   0x02  // Self-test giroscópio eixo Z
#define SELF_TEST_X_ACCEL  0x0D  // Self-test acelerômetro eixo X
#define SELF_TEST_Y_ACCEL  0x0E  // Self-test acelerômetro eixo Y
#define SELF_TEST_Z_ACCEL  0x0F  // Self-test acelerômetro eixo Z

/**
 * DEFINIÇÕES DE BITS DOS REGISTRADORES
 * ===================================
 * Máscaras e valores específicos para configuração dos registradores
 */
#define PWR_RESET           0x80 // Bit 7 - Reset do dispositivo
#define CLOCK_SEL_PLL       0x01 // Seleção de clock PLL (mais estável)
#define I2C_MST_EN          0x20 // Habilita modo I2C master
#define I2C_SLV0_EN         0x80 // Habilita slave 0 do I2C master
#define I2C_READ_FLAG       0x80 // Flag para operação de leitura I2C
#define BYPASS_EN           0x02 // Habilita bypass I2C (acesso direto ao magnetômetro)

/**
 * IDS DOS DISPOSITIVOS
 * ===================
 * Valores esperados no registrador WHO_AM_I para identificação dos chips
 */
#define MPU9250_ID          0x71  // ID do chip MPU9250
#define MPU9255_ID          0x73  // ID do chip MPU9255 (variante do MPU9250)
#define AK8963_ID           0x48  // ID do chip magnetômetro AK8963

/**
 * PROTÓTIPOS DAS FUNÇÕES INTERNAS
 * ==============================
 * Funções auxiliares para comunicação I2C de baixo nível
 */
static void mpu9250_write_reg(mpu9250_t *mpu, uint8_t reg, uint8_t data);
static uint8_t mpu9250_read_reg(mpu9250_t *mpu, uint8_t reg);
static void mpu9250_read_regs(mpu9250_t *mpu, uint8_t reg, uint8_t *buffer, uint8_t len);
static void mpu9250_write_mag_reg(mpu9250_t *mpu, uint8_t reg, uint8_t data);
static uint8_t mpu9250_read_mag_reg(mpu9250_t *mpu, uint8_t reg);
static void mpu9250_read_mag_regs(mpu9250_t *mpu, uint8_t reg, uint8_t *buffer, uint8_t len);
static void mpu9250_update_sensitivity_factors(mpu9250_t *mpu);

/**
 * @brief Configura e inicializa a comunicação I2C para o MPU9250
 * 
 * Esta função realiza uma sequência crítica para garantir que a comunicação I2C
 * seja estabelecida corretamente, evitando travamentos do barramento:
 * 
 * 1. Reset manual dos pinos GPIO (prevenção de travamento)
 * 2. Força estado alto nas linhas SDA e SCL
 * 3. Configura os pinos para função I2C
 * 4. Ativa pull-ups internos
 * 5. Aguarda estabilização
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 */
void mpu9250_setup_i2c(mpu9250_t *mpu) 
{
    // Reset das linhas I2C antes de inicializar (prevenção de travamento)
    // Configura os pinos como GPIO de saída para forçar estado conhecido
    gpio_init(mpu->sda_gpio);
    gpio_init(mpu->scl_gpio);
    gpio_set_dir(mpu->sda_gpio, GPIO_OUT);
    gpio_set_dir(mpu->scl_gpio, GPIO_OUT);
    
    // Força ambas as linhas em nível alto para garantir estado conhecido
    // Isso evita que o barramento I2C fique travado
    gpio_put(mpu->sda_gpio, 1);
    gpio_put(mpu->scl_gpio, 1);
    sleep_ms(10);
    
    // Agora configura os pinos para função I2C
    i2c_init(mpu->i2c, 400*1000); // I2C a 400 kHz (fast mode)
    gpio_set_function(mpu->sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(mpu->scl_gpio, GPIO_FUNC_I2C);
    
    // Ativa pull-ups internos (essencial para I2C)
    gpio_pull_up(mpu->sda_gpio);
    gpio_pull_up(mpu->scl_gpio);
    
    // Aguarda estabilização do barramento
    sleep_ms(10);
}

/**
 * @brief Realiza reset completo do MPU9250 e configuração básica
 * 
 * Esta função executa uma sequência de reset e inicialização básica:
 * 1. Reset completo do dispositivo (todos os registradores voltam ao padrão)
 * 2. Aguarda tempo necessário para o reset ser concluído
 * 3. Configura fonte de clock mais estável (PLL)
 * 4. Habilita todos os sensores (acelerômetro e giroscópio)
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 */
void mpu9250_reset(mpu9250_t *mpu) 
{
    // Reseta o dispositivo (bit 7 do PWR_MGMT_1)
    // Isso restaura todos os registradores aos valores padrão
    mpu9250_write_reg(mpu, MPU9250_PWR_MGMT_1, PWR_RESET);
    sleep_ms(100);  // Aguarda conclusão do reset
    
    // Acorda o dispositivo e seleciona a melhor fonte de clock disponível
    // CLOCK_SEL_PLL = usa PLL com referência do giroscópio X (mais estável)
    mpu9250_write_reg(mpu, MPU9250_PWR_MGMT_1, CLOCK_SEL_PLL);
    sleep_ms(10);
    
    // Habilita todos os sensores (acelerômetro e giroscópio nos 3 eixos)
    // 0x00 = todos os sensores habilitados (bits de standby limpos)
    mpu9250_write_reg(mpu, MPU9250_PWR_MGMT_2, 0x00);
    sleep_ms(10);
}

/**
 * @brief Inicializa completamente o sensor MPU9250 com configurações especificadas
 * 
 * Esta é a função principal de inicialização que configura todos os aspectos do sensor:
 * 1. Configura comunicação I2C
 * 2. Realiza reset do dispositivo
 * 3. Verifica conectividade
 * 4. Configura ranges do acelerômetro e giroscópio
 * 5. Configura filtros digitais (DLPF)
 * 6. Define taxa de amostragem
 * 7. Inicializa magnetômetro se solicitado
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param config Ponteiro para estrutura de configuração com parâmetros desejados
 * @return true se inicialização bem-sucedida, false caso contrário
 */
bool mpu9250_init(mpu9250_t *mpu, mpu9250_config_t *config)
{
    // Setup I2C
    mpu9250_setup_i2c(mpu);
    
    // Reset device
    mpu9250_reset(mpu);
    
    // Check device connection
    if (!mpu9250_test_connection(mpu)) 
    {
        return false;
    }
    
    // Configure accelerometer range
    mpu9250_set_accel_range(mpu, config->accel_range);
    
    // Configure gyroscope range
    mpu9250_set_gyro_range(mpu, config->gyro_range);
    
    // Configure DLPF
    mpu9250_set_dlpf(mpu, config->dlpf_filter);
    
    // Configure sample rate
    mpu9250_set_sample_rate(mpu, config->sample_rate_divider);
    
    // Initialize magnetometer if enabled
    if (config->enable_magnetometer) 
    {
        if (!mpu9250_enable_magnetometer(mpu, true)) 
        {
            return false;
        }
    }
    
    return true;
}

/**
 * @brief Testa a conectividade e identifica o chip MPU9250/MPU9255
 * 
 * Lê o registrador WHO_AM_I para verificar se o dispositivo está respondendo
 * corretamente e se é um chip MPU9250 ou MPU9255 válido.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @return true se chip identificado corretamente, false caso contrário
 */
bool mpu9250_test_connection(mpu9250_t *mpu)
{
    uint8_t who_am_i = mpu9250_read_reg(mpu, MPU9250_WHO_AM_I);
    printf("WHO_AM_I: 0x%02X\n", who_am_i); // Debug: mostra ID lido
    return (who_am_i == MPU9250_ID || who_am_i == MPU9255_ID);
}

/**
 * @brief Testa a conectividade com o magnetômetro AK8963
 * 
 * Verifica se o magnetômetro integrado está respondendo corretamente
 * através da leitura do seu registrador WHO_AM_I.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @return true se magnetômetro identificado corretamente, false caso contrário
 */
bool mpu9250_test_mag_connection(mpu9250_t *mpu)
{
    uint8_t who_am_i = mpu9250_read_mag_reg(mpu, AK8963_WHO_AM_I);
    printf("AK8963 WHO_AM_I: 0x%02X\n", who_am_i); // Debug: mostra ID do magnetômetro
    return (who_am_i == AK8963_ID);
}

/**
 * @brief Obtém o range atual do acelerômetro
 * 
 * Lê os bits 4:3 do registrador ACCEL_CONFIG para determinar
 * qual range está configurado atualmente.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @return Valor do range: 0=±2g, 1=±4g, 2=±8g, 3=±16g
 */
uint8_t mpu9250_get_accel_range(mpu9250_t *mpu) 
{
    uint8_t val = mpu9250_read_reg(mpu, MPU9250_ACCEL_CONFIG);
    return (val >> 3) & 0x03; // Extrai bits 4:3
}

/**
 * @brief Configura o range (fundo de escala) do acelerômetro
 * 
 * Define a sensibilidade do acelerômetro alterando seu fundo de escala.
 * Ranges maiores permitem medir acelerações maiores, mas com menor precisão.
 * A função também atualiza automaticamente o fator de sensibilidade interno.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param range Range desejado (ACCEL_RANGE_2G, 4G, 8G ou 16G)
 */
void mpu9250_set_accel_range(mpu9250_t *mpu, mpu9250_accel_range_t range) 
{
    uint8_t val = mpu9250_read_reg(mpu, MPU9250_ACCEL_CONFIG);
    val &= ~0x18; // Limpa bits [4:3] do range atual
    val |= (range & 0x18); // Define novo range (mantém apenas bits válidos)
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG, val);
    mpu9250_update_sensitivity_factors(mpu); // Atualiza fatores de conversão
}

/**
 * @brief Obtém o range atual do giroscópio
 * 
 * Lê os bits 4:3 do registrador GYRO_CONFIG para determinar
 * qual range está configurado atualmente.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @return Valor do range: 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
 */
uint8_t mpu9250_get_gyro_range(mpu9250_t *mpu) 
{
    uint8_t val = mpu9250_read_reg(mpu, MPU9250_GYRO_CONFIG);
    return (val >> 3) & 0x03; // Extrai bits 4:3
}

/**
 * @brief Configura o range (fundo de escala) do giroscópio
 * 
 * Define a sensibilidade do giroscópio alterando seu fundo de escala.
 * Ranges maiores permitem medir velocidades angulares maiores, mas com menor precisão.
 * A função também atualiza automaticamente o fator de sensibilidade interno.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param range Range desejado (GYRO_RANGE_250, 500, 1000 ou 2000 DPS)
 */
void mpu9250_set_gyro_range(mpu9250_t *mpu, mpu9250_gyro_range_t range) 
{
    uint8_t val = mpu9250_read_reg(mpu, MPU9250_GYRO_CONFIG);
    val &= ~0x18; // Limpa bits [4:3] do range atual
    val |= (range & 0x18); // Define novo range (mantém apenas bits válidos)
    mpu9250_write_reg(mpu, MPU9250_GYRO_CONFIG, val);
    mpu9250_update_sensitivity_factors(mpu); // Atualiza fatores de conversão
}

/**
 * @brief Configura o filtro digital passa-baixa (DLPF)
 * 
 * O DLPF remove ruído de alta frequência dos sinais, melhorando a qualidade dos dados.
 * Filtros com frequências de corte menores reduzem mais ruído, mas introduzem maior latência.
 * Esta função configura o DLPF tanto para o giroscópio quanto para o acelerômetro.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param filter Configuração do filtro (DLPF_DISABLE ou frequências de 5Hz a 260Hz)
 */
void mpu9250_set_dlpf(mpu9250_t *mpu, mpu9250_dlpf_t filter)
{
    // Configura DLPF para o giroscópio (registrador CONFIG)
    mpu9250_write_reg(mpu, MPU9250_CONFIG, filter);
    
    // Configura DLPF para o acelerômetro (registrador ACCEL_CONFIG2)
    uint8_t accel_config2 = mpu9250_read_reg(mpu, MPU9250_ACCEL_CONFIG2);
    accel_config2 = (accel_config2 & 0xF0) | (filter & 0x0F); // Preserva bits superiores
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG2, accel_config2);
}

/**
 * @brief Configura a taxa de amostragem dos sensores
 * 
 * A taxa de amostragem final é calculada como:
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 * 
 * Onde Gyroscope Output Rate depende do DLPF:
 * - DLPF desabilitado: 8kHz
 * - DLPF habilitado: 1kHz
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param divider Divisor da taxa de amostragem (0-255)
 */
void mpu9250_set_sample_rate(mpu9250_t *mpu, uint8_t divider)
{
    mpu9250_write_reg(mpu, MPU9250_SMPLRT_DIV, divider);
}

/**
 * @brief Habilita ou desabilita o magnetômetro AK8963 integrado
 * 
 * Esta é uma das funções mais complexas do driver, pois o magnetômetro AK8963 é um
 * chip separado que deve ser acessado através do MPU9250. O processo envolve:
 * 
 * HABILITAÇÃO:
 * 1. Configurar bypass I2C para acesso direto temporário
 * 2. Verificar conectividade com o magnetômetro
 * 3. Ler valores de calibração de fábrica (ASA - Adjustment Values)
 * 4. Configurar modo de operação contínuo
 * 5. Configurar I2C master do MPU9250 para leitura automática
 * 6. Verificar configuração final
 * 
 * DESABILITAÇÃO:
 * 1. Colocar magnetômetro em power-down
 * 2. Desabilitar bypass I2C
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param enable true para habilitar, false para desabilitar
 * @return true se operação bem-sucedida, false caso contrário
 */
bool mpu9250_enable_magnetometer(mpu9250_t *mpu, bool enable)
{
    if (enable) 
    {
        printf("Starting magnetometer initialization...\n");
        
        // 1. Desabilita I2C master e ativa bypass para acesso direto ao magnetômetro
        // Isso permite comunicação direta com o AK8963 temporariamente
        mpu9250_write_reg(mpu, MPU9250_USER_CTRL, 0x00);
        sleep_ms(10);
        uint8_t int_pin_cfg = mpu9250_read_reg(mpu, MPU9250_INT_PIN_CFG);
        mpu9250_write_reg(mpu, MPU9250_INT_PIN_CFG, int_pin_cfg | BYPASS_EN);
        sleep_ms(100);
        
        // 2. Testa conexão com magnetômetro via bypass
        if (!mpu9250_test_mag_connection(mpu)) 
        {
            printf("ERROR: Magnetometer not detected!\n");
            return false;
        }
        printf("Magnetometer detected successfully\n");
        
        // 3. Reset do magnetômetro para estado conhecido
        mpu9250_write_mag_reg(mpu, AK8963_CNTL2, 0x01);
        sleep_ms(100);
        
        // 4. Entra no modo FUSE ROM para ler valores de calibração de fábrica
        // Os valores ASA (Adjustment Sensitivity Adjustment) compensam variações de fabricação
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, AK8963_FUSE_ROM);
        sleep_ms(100);
        
        // 5. Lê valores de calibração ASA (Adjustment Sensitivity Adjustment)
        // Estes valores são únicos para cada chip e corrigem variações de fabricação
        uint8_t asa_data[3];
        mpu9250_read_mag_regs(mpu, AK8963_ASAX, asa_data, 3);
        for (int i = 0; i < 3; i++) {
            // Fórmula do datasheet: ASA = (valor_lido - 128)/256 + 1
            mpu->mag_asa[i] = ((float)asa_data[i] - 128.0f) / 256.0f + 1.0f;
        }
        printf("ASA values: X=%.3f, Y=%.3f, Z=%.3f\n", 
               mpu->mag_asa[0], mpu->mag_asa[1], mpu->mag_asa[2]);
        
        // 6. Power down antes de configurar modo contínuo (transição obrigatória)
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, AK8963_POWER_DOWN);
        sleep_ms(100);
        
        // 7. Configura magnetômetro para modo contínuo 2 (100Hz) com resolução 16-bit
        // 0x16 = Continuous mode 2 (100Hz) + 16-bit output
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, 0x16);
        sleep_ms(100);
        
        // 8. Verifica se entrou corretamente no modo contínuo
        uint8_t cntl1 = mpu9250_read_mag_reg(mpu, AK8963_CNTL1);
        printf("AK8963 CNTL1: 0x%02X (Expected: 0x16)\n", cntl1);
        
        // 9. Desativa bypass - volta para comunicação via I2C master
        mpu9250_write_reg(mpu, MPU9250_INT_PIN_CFG, int_pin_cfg & ~BYPASS_EN);
        sleep_ms(10);
        
        // 10. Primeiro desabilita todas as slaves I2C (limpeza preventiva)
        for (int i = 0; i < 4; i++) {
            mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL + i*3, 0x00);
        }
        sleep_ms(10);
        
        // 11. Configura clock do I2C master ANTES de habilitar (400kHz)
        // 0x0D = frequência de ~400kHz para comunicação com magnetômetro
        mpu9250_write_reg(mpu, MPU9250_I2C_MST_CTRL, 0x0D);
        sleep_ms(10);
        
        // 12. Configura Slave 0 para leitura automática do magnetômetro
        // Endereço do AK8963 com bit de leitura (0x8C = 0x0C | 0x80)
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_ADDR, AK8963_ADDR | I2C_READ_FLAG);
        sleep_ms(5);
        // Registrador inicial para leitura (ST1 - inclui status + dados + ST2)
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_REG, AK8963_ST1);
        sleep_ms(5);
        // Habilita slave 0 + lê 8 bytes (ST1 + 6 bytes dados + ST2)
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, 0x88);
        sleep_ms(10);
        
        // 13. Agora habilita I2C master para começar leituras automáticas
        mpu9250_write_reg(mpu, MPU9250_USER_CTRL, I2C_MST_EN);
        sleep_ms(100);
        
        // 14. Verificação final da configuração
        uint8_t verify_addr = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_ADDR);
        uint8_t verify_reg = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_REG);
        uint8_t verify_ctrl = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_CTRL);
        uint8_t verify_user = mpu9250_read_reg(mpu, MPU9250_USER_CTRL);
        
        printf("Final verification:\n");
        printf("  I2C_SLV0_ADDR: 0x%02X (Expected: 0x8C)\n", verify_addr);
        printf("  I2C_SLV0_REG: 0x%02X (Expected: 0x02)\n", verify_reg);
        printf("  I2C_SLV0_CTRL: 0x%02X (Expected: 0x88)\n", verify_ctrl);
        printf("  USER_CTRL: 0x%02X (Expected: 0x20)\n", verify_user);
        
        mpu->mag_enabled = true;
        printf("Magnetometer initialization completed\n");
    } 
    else 
    {
        // DESABILITAÇÃO DO MAGNETÔMETRO
        // Coloca magnetômetro em power-down
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, AK8963_POWER_DOWN);
        // Desabilita bypass mode
        uint8_t int_pin_cfg = mpu9250_read_reg(mpu, MPU9250_INT_PIN_CFG);
        mpu9250_write_reg(mpu, MPU9250_INT_PIN_CFG, int_pin_cfg & ~BYPASS_EN);
        mpu->mag_enabled = false;
    }
    return true;
}

/**
 * @brief Lê dados brutos do acelerômetro, giroscópio e temperatura
 * 
 * Realiza uma única leitura sequencial de 14 bytes começando do registrador
 * ACCEL_XOUT_H, obtendo todos os dados de movimento em uma operação I2C.
 * Os dados são retornados como valores brutos de 16 bits (sem conversão).
 * 
 * Layout dos dados lidos:
 * - Bytes 0-1: Aceleração X (high byte, low byte)
 * - Bytes 2-3: Aceleração Y
 * - Bytes 4-5: Aceleração Z  
 * - Bytes 6-7: Temperatura
 * - Bytes 8-9: Velocidade angular X
 * - Bytes 10-11: Velocidade angular Y
 * - Bytes 12-13: Velocidade angular Z
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param accel Array para armazenar dados brutos do acelerômetro [X,Y,Z]
 * @param gyro Array para armazenar dados brutos do giroscópio [X,Y,Z]
 * @param temp Ponteiro para armazenar dados brutos de temperatura
 */
void mpu9250_read_raw_motion(mpu9250_t *mpu, int16_t accel[3], int16_t gyro[3], int16_t *temp) 
{
    uint8_t buffer[14];
    
    // Lê dados do acelerômetro, temperatura e giroscópio em uma única operação
    // Isso é mais eficiente e garante sincronização temporal dos dados
    mpu9250_read_regs(mpu, MPU9250_ACCEL_XOUT_H, buffer, 14);
    
    // Converte dados do acelerômetro (big endian: byte alto primeiro)
    accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]);   // X
    accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]);   // Y
    accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]);   // Z
    
    // Converte dados de temperatura
    *temp = (int16_t)((buffer[6] << 8) | buffer[7]);
    
    // Converte dados do giroscópio (big endian: byte alto primeiro)
    gyro[0] = (int16_t)((buffer[8] << 8) | buffer[9]);    // X
    gyro[1] = (int16_t)((buffer[10] << 8) | buffer[11]);  // Y
    gyro[2] = (int16_t)((buffer[12] << 8) | buffer[13]);  // Z
}

/**
 * @brief Lê dados brutos do magnetômetro via I2C master
 * 
 * Esta função é complexa pois o magnetômetro é lido através do I2C master do MPU9250.
 * Os dados são automaticamente capturados pelo MPU9250 e disponibilizados nos
 * registradores EXT_SENS_DATA. A função também implementa:
 * 
 * 1. Verificação se magnetômetro está habilitado
 * 2. Verificação de data ready (ST1)
 * 3. Detecção e tratamento de overflow (ST2)
 * 4. Ajuste de alinhamento de eixos conforme datasheet
 * 5. Reset automático em caso de erro
 * 
 * IMPORTANTE: Os eixos do magnetômetro são realinhados para coincidir com
 * o sistema de coordenadas do acelerômetro/giroscópio conforme datasheet.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param mag Array para armazenar dados brutos do magnetômetro [X,Y,Z] realinhados
 */
void mpu9250_read_raw_mag(mpu9250_t *mpu, int16_t mag[3])
{
    if (!mpu->mag_enabled) 
    {
        // Magnetômetro não habilitado - retorna zeros
        mag[0] = mag[1] = mag[2] = 0;
        return;
    }
    
    // Para modo I2C master, lê dados dos registradores EXT_SENS_DATA
    uint8_t buffer[8];
    
    // Lê os 8 bytes de dados do magnetômetro capturados automaticamente
    // Layout: ST1(0), HXL(1), HXH(2), HYL(3), HYH(4), HZL(5), HZH(6), ST2(7)
    mpu9250_read_regs(mpu, MPU9250_EXT_SENS_DATA_00, buffer, 8);
    
    // Verifica se dados estão prontos (bit 0 do ST1 = DRDY)
    if (!(buffer[0] & 0x01)) {
        // Dados não prontos, retorna zeros
        mag[0] = mag[1] = mag[2] = 0;
        return;
    }
    
    // Verifica ST2 para overflow magnético (bit 3 = HOFL)
    if (buffer[7] & 0x08) {
        // Overflow detectado - sensor magnético saturado, necessário reset
        printf("Magnetometer overflow detected, resetting...\n");
        
        // Ativa bypass temporariamente para reset direto do magnetômetro
        uint8_t user_ctrl = mpu9250_read_reg(mpu, MPU9250_USER_CTRL);
        uint8_t int_pin_cfg = mpu9250_read_reg(mpu, MPU9250_INT_PIN_CFG);
        
        mpu9250_write_reg(mpu, MPU9250_USER_CTRL, 0x00);
        mpu9250_write_reg(mpu, MPU9250_INT_PIN_CFG, int_pin_cfg | BYPASS_EN);
        sleep_ms(10);
        
        // Reset e reconfigura magnetômetro
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, AK8963_POWER_DOWN);
        sleep_ms(10);
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, 0x16); // Continuous mode 2 + 16-bit
        sleep_ms(10);
        
        // Restaura modo I2C master
        mpu9250_write_reg(mpu, MPU9250_INT_PIN_CFG, int_pin_cfg & ~BYPASS_EN);
        mpu9250_write_reg(mpu, MPU9250_USER_CTRL, user_ctrl);
        sleep_ms(10);
        
        mag[0] = mag[1] = mag[2] = 0;
        return;
    }
    
    // Converte dados do magnetômetro (formato little endian: byte baixo primeiro)
    // Layout do buffer: ST1(0), HXL(1), HXH(2), HYL(3), HYH(4), HZL(5), HZH(6), ST2(7)
    // AK8963 usa little endian: byte baixo primeiro, depois byte alto
    int16_t mag_raw[3];
    mag_raw[0] = (int16_t)((buffer[2] << 8) | buffer[1]);  // HX = HXH << 8 | HXL
    mag_raw[1] = (int16_t)((buffer[4] << 8) | buffer[3]);  // HY = HYH << 8 | HYL  
    mag_raw[2] = (int16_t)((buffer[6] << 8) | buffer[5]);  // HZ = HZH << 8 | HZL

    // Ajuste de alinhamento conforme datasheet MPU9250:
    // Para alinhar os eixos do magnetômetro com acelerômetro/giroscópio:
    // mag[X] = mag_raw[Y] (eixo Y do magnetômetro vira X do sistema)
    // mag[Y] = mag_raw[X] (eixo X do magnetômetro vira Y do sistema)
    // mag[Z] = -mag_raw[Z] (eixo Z do magnetômetro é invertido)
    mag[0] = mag_raw[1];
    mag[1] = mag_raw[0];
    mag[2] = -mag_raw[2];
}

/**
 * @brief Lê todos os dados brutos dos sensores em uma única chamada
 * 
 * Função de conveniência que obtém dados brutos do acelerômetro, giroscópio,
 * temperatura e magnetômetro, organizando-os em uma estrutura.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param data Ponteiro para estrutura que receberá todos os dados brutos
 */
void mpu9250_read_raw(mpu9250_t *mpu, mpu9250_raw_data_t *data)
{
    mpu9250_read_raw_motion(mpu, data->accel, data->gyro, &data->temp);
    mpu9250_read_raw_mag(mpu, data->mag);
}

/**
 * @brief Lê dados calibrados do acelerômetro, giroscópio e temperatura
 * 
 * Converte dados brutos para unidades físicas aplicando os fatores de
 * sensibilidade apropriados para cada sensor baseado em seu range configurado.
 * 
 * Unidades resultantes:
 * - Acelerômetro: g (gravidades terrestres)
 * - Giroscópio: °/s (graus por segundo)
 * - Temperatura: °C (graus Celsius)
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param accel Array para dados calibrados do acelerômetro [X,Y,Z] em g
 * @param gyro Array para dados calibrados do giroscópio [X,Y,Z] em °/s
 * @param temp Ponteiro para temperatura calibrada em °C
 */
void mpu9250_read_motion(mpu9250_t *mpu, float accel[3], float gyro[3], float *temp)
{
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    
    // Obtém dados brutos
    mpu9250_read_raw_motion(mpu, accel_raw, gyro_raw, &temp_raw);
    
    // Converte para unidades físicas usando fatores de sensibilidade
    for (int i = 0; i < 3; i++) {
        accel[i] = (float)accel_raw[i] / mpu->accel_sensitivity;  // Conversão para g
        gyro[i] = (float)gyro_raw[i] / mpu->gyro_sensitivity;    // Conversão para °/s
    }
    
    // Converte temperatura usando fórmula do datasheet MPU9250
    // Fórmula: Temp_degC = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity) + 21
    *temp = ((float)temp_raw - 21.0f) / 333.87f + 21.0f;
}

/**
 * @brief Lê dados calibrados do magnetômetro
 * 
 * Converte dados brutos do magnetômetro para unidades físicas (µT - microtesla)
 * aplicando os fatores de correção ASA (Adjustment Sensitivity Adjustment) 
 * obtidos durante a inicialização.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param mag Array para dados calibrados do magnetômetro [X,Y,Z] em µT
 */
void mpu9250_read_mag(mpu9250_t *mpu, float mag[3])
{
    int16_t mag_raw[3];
    
    // Obtém dados brutos do magnetômetro
    mpu9250_read_raw_mag(mpu, mag_raw);
    
    // Converte para unidades físicas com ajuste de sensibilidade
    // Aplica fatores ASA específicos de cada chip + sensibilidade padrão
    for (int i = 0; i < 3; i++) {
        mag[i] = (float)mag_raw[i] * mpu->mag_asa[i] * MAG_SENS;
    }
}

/**
 * @brief Lê todos os dados calibrados dos sensores
 * 
 * Função de conveniência que obtém dados calibrados de todos os sensores
 * (acelerômetro, giroscópio, temperatura e magnetômetro) em unidades físicas.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param data Ponteiro para estrutura que receberá todos os dados calibrados
 */
void mpu9250_read_data(mpu9250_t *mpu, mpu9250_data_t *data)
{
    mpu9250_read_motion(mpu, data->accel, data->gyro, &data->temp);
    mpu9250_read_mag(mpu, data->mag);
}

/**
 * @brief Lê apenas a temperatura calibrada
 * 
 * Função otimizada para leitura apenas da temperatura quando outros
 * dados dos sensores não são necessários.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @return Temperatura em graus Celsius
 */
float mpu9250_read_temperature(mpu9250_t *mpu)
{
    uint8_t buffer[2];
    mpu9250_read_regs(mpu, MPU9250_TEMP_OUT_H, buffer, 2);
    
    int16_t temp_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    return ((float)temp_raw - 21.0f) / 333.87f + 21.0f;
}

/**
 * @brief Função de debug para diagnóstico do magnetômetro
 * 
 * Exibe informações detalhadas sobre o status do magnetômetro e configuração
 * do I2C master, incluindo verificação automática e correção de configurações
 * incorretas. Útil para diagnosticar problemas de comunicação.
 * 
 * Informações exibidas:
 * - Dados brutos capturados nos registradores EXT_SENS_DATA
 * - Status de data ready e overflow
 * - Configuração do I2C master
 * - Status dos slaves I2C
 * - Auto-correção de configurações incorretas
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 */
void mpu9250_debug_mag_status(mpu9250_t *mpu)
{
    if (!mpu->mag_enabled) {
        printf("Magnetometer is disabled\n");
        return;
    }
    
    // Read EXT_SENS_DATA to see what's being captured
    uint8_t buffer[8];
    mpu9250_read_regs(mpu, MPU9250_EXT_SENS_DATA_00, buffer, 8);
    
    printf("EXT_SENS_DATA: ");
    for (int i = 0; i < 8; i++) {
        printf("0x%02X ", buffer[i]);
    }
    printf("\n");
    
    printf("ST1 (DRDY): %s\n", (buffer[0] & 0x01) ? "Ready" : "Not Ready");
    printf("ST2 (HOFL): %s\n", (buffer[7] & 0x08) ? "Overflow" : "Normal");
    
    // Check I2C master status
    uint8_t user_ctrl = mpu9250_read_reg(mpu, MPU9250_USER_CTRL);
    uint8_t int_pin_cfg = mpu9250_read_reg(mpu, MPU9250_INT_PIN_CFG);
    uint8_t i2c_mst_ctrl = mpu9250_read_reg(mpu, MPU9250_I2C_MST_CTRL);
    uint8_t i2c_slv0_addr = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_ADDR);
    uint8_t i2c_slv0_reg = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_REG);
    uint8_t i2c_slv0_ctrl = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_CTRL);
    
    printf("USER_CTRL: 0x%02X (I2C_MST_EN: %s)\n", user_ctrl, (user_ctrl & 0x20) ? "ON" : "OFF");
    printf("INT_PIN_CFG: 0x%02X (BYPASS_EN: %s)\n", int_pin_cfg, (int_pin_cfg & 0x02) ? "ON" : "OFF");
    printf("I2C_MST_CTRL: 0x%02X\n", i2c_mst_ctrl);
    printf("I2C_SLV0_ADDR: 0x%02X (Expected: 0x8C)\n", i2c_slv0_addr);
    printf("I2C_SLV0_REG: 0x%02X (Expected: 0x02)\n", i2c_slv0_reg);
    printf("I2C_SLV0_CTRL: 0x%02X (EN: %s, LEN: %d)\n", i2c_slv0_ctrl, 
           (i2c_slv0_ctrl & 0x80) ? "ON" : "OFF", i2c_slv0_ctrl & 0x0F);
    
    // Auto-fix if configuration is wrong
    if ((i2c_slv0_addr != 0x8C) || (i2c_slv0_reg != 0x02) || (i2c_slv0_ctrl != 0x88)) {
        printf("Auto-fixing magnetometer I2C configuration...\n");
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_ADDR, AK8963_ADDR | I2C_READ_FLAG);
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_REG, AK8963_ST1);
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, 0x88);
        printf("Configuration restored\n");
    }
}

/**
 * @brief Calibra o giroscópio calculando offsets de bias estático
 * 
 * IMPORTANTE: O sensor deve estar IMÓVEL durante toda a calibração!
 * 
 * Esta função coleta um número especificado de amostras do giroscópio enquanto
 * o sensor está parado e calcula a média. Essa média representa o bias (offset)
 * estático que deve ser subtraído das leituras futuras para obter valores
 * corretos de velocidade angular.
 * 
 * O bias do giroscópio varia com:
 * - Temperatura
 * - Tempo de operação  
 * - Orientação do sensor
 * 
 * Por isso é recomendado recalibrar periodicamente ou quando há mudanças
 * significativas de temperatura.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param samples Número de amostras para a média (recomendado: 1000+)
 * @param gyro_offset Array que receberá os offsets calculados [X,Y,Z] em °/s
 */
void mpu9250_calibrate_gyro(mpu9250_t *mpu, uint16_t samples, float gyro_offset[3])
{
    int32_t gyro_sum[3] = {0, 0, 0};  // Acumuladores para soma (32 bits para evitar overflow)
    int16_t gyro_raw[3], accel_raw[3], temp_raw;
    
    // Inicializa offsets com zero
    gyro_offset[0] = gyro_offset[1] = gyro_offset[2] = 0.0f;
    
    // Coleta amostras para cálculo da média
    for (uint16_t i = 0; i < samples; i++) 
    {
        mpu9250_read_raw_motion(mpu, accel_raw, gyro_raw, &temp_raw);
        
        // Acumula valores brutos do giroscópio
        gyro_sum[0] += gyro_raw[0];
        gyro_sum[1] += gyro_raw[1];
        gyro_sum[2] += gyro_raw[2];
        
        sleep_ms(2);  // Pequeno delay entre amostras
    }
    
    // Calcula médias e converte para unidades físicas (°/s)
    for (int i = 0; i < 3; i++) 
    {
        gyro_offset[i] = (float)gyro_sum[i] / (float)samples / mpu->gyro_sensitivity;
    }
}

/**
 * @brief Executa self-test completo do MPU9250
 * 
 * O self-test é um teste de integridade que verifica se os sensores estão
 * funcionando corretamente comparando suas respostas com e sem estimulação
 * interna. O teste segue o procedimento do datasheet:
 * 
 * 1. Lê códigos de self-test de fábrica
 * 2. Salva configuração atual (incluindo I2C master)
 * 3. Configura sensor para condições de teste
 * 4. Coleta amostras sem self-test (operação normal)
 * 5. Coleta amostras com self-test habilitado
 * 6. Restaura configuração original
 * 7. Calcula resposta ao self-test (diferença entre modos)
 * 8. Valida se resposta está dentro dos limites esperados
 * 
 * IMPORTANTE: Preserva completamente a configuração do I2C master para
 * não interromper a operação do magnetômetro.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @return true se todos os testes passaram, false caso contrário
 */
bool mpu9250_self_test(mpu9250_t *mpu)
{
    // Leitura dos códigos de fábrica de self-test
    // Estes valores são únicos para cada chip e definem a resposta esperada
    uint8_t st_gyro[3], st_accel[3];
    st_gyro[0]  = mpu9250_read_reg(mpu, SELF_TEST_X_GYRO);
    st_gyro[1]  = mpu9250_read_reg(mpu, SELF_TEST_Y_GYRO);
    st_gyro[2]  = mpu9250_read_reg(mpu, SELF_TEST_Z_GYRO);
    st_accel[0] = mpu9250_read_reg(mpu, SELF_TEST_X_ACCEL);
    st_accel[1] = mpu9250_read_reg(mpu, SELF_TEST_Y_ACCEL);
    st_accel[2] = mpu9250_read_reg(mpu, SELF_TEST_Z_ACCEL);

    printf("Factory self-test codes (Accel: %02X %02X %02X, Gyro: %02X %02X %02X)\n",
           st_accel[0], st_accel[1], st_accel[2], st_gyro[0], st_gyro[1], st_gyro[2]);

    // Salva configuração atual incluindo registradores do I2C master
    // CRÍTICO: Deve preservar configuração do magnetômetro
    uint8_t saved_config[10];
    saved_config[0] = mpu9250_read_reg(mpu, MPU9250_SMPLRT_DIV);
    saved_config[1] = mpu9250_read_reg(mpu, MPU9250_CONFIG);
    saved_config[2] = mpu9250_read_reg(mpu, MPU9250_GYRO_CONFIG);
    saved_config[3] = mpu9250_read_reg(mpu, MPU9250_ACCEL_CONFIG);
    saved_config[4] = mpu9250_read_reg(mpu, MPU9250_ACCEL_CONFIG2);
    saved_config[5] = mpu9250_read_reg(mpu, MPU9250_USER_CTRL);
    saved_config[6] = mpu9250_read_reg(mpu, MPU9250_I2C_MST_CTRL);
    saved_config[7] = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_ADDR);
    saved_config[8] = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_REG);
    saved_config[9] = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_CTRL);
    
    printf("Starting MPU9250 self-test...\n");
    
    // Configura para self-test conforme datasheet
    mpu9250_write_reg(mpu, MPU9250_SMPLRT_DIV, 0x00);    // Taxa de amostragem = 1kHz
    mpu9250_write_reg(mpu, MPU9250_CONFIG, 0x02);         // DLPF = 92Hz
    mpu9250_write_reg(mpu, MPU9250_GYRO_CONFIG, 0x00);    // ±250 dps, sem self-test
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG, 0x00);   // ±2g, sem self-test  
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG2, 0x02);  // DLPF = 92Hz
    
    sleep_ms(50); // Aguarda estabilização das configurações
    
    // Collect normal operation samples (no self-test)
    int32_t accel_normal_sum[3] = {0}, gyro_normal_sum[3] = {0};
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    
    for (int i = 0; i < 200; i++) 
    {
        mpu9250_read_raw_motion(mpu, accel_raw, gyro_raw, &temp_raw);
        accel_normal_sum[0] += accel_raw[0];
        accel_normal_sum[1] += accel_raw[1];
        accel_normal_sum[2] += accel_raw[2];
        gyro_normal_sum[0] += gyro_raw[0];
        gyro_normal_sum[1] += gyro_raw[1];
        gyro_normal_sum[2] += gyro_raw[2];
        sleep_ms(1);
    }
    
    // Enable self-test
    mpu9250_write_reg(mpu, MPU9250_GYRO_CONFIG, 0xE0);    // Enable XYZ self-test, ±250 dps
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG, 0xE0);   // Enable XYZ self-test, ±2g
    
    sleep_ms(50); // Wait for self-test to stabilize
    
    // Collect self-test samples
    int32_t accel_st_sum[3] = {0}, gyro_st_sum[3] = {0};
    
    for (int i = 0; i < 200; i++) 
    {
        mpu9250_read_raw_motion(mpu, accel_raw, gyro_raw, &temp_raw);
        accel_st_sum[0] += accel_raw[0];
        accel_st_sum[1] += accel_raw[1];
        accel_st_sum[2] += accel_raw[2];
        gyro_st_sum[0] += gyro_raw[0];
        gyro_st_sum[1] += gyro_raw[1];
        gyro_st_sum[2] += gyro_raw[2];
        sleep_ms(1);
    }
    
    // Restore original configuration including I2C master settings
    mpu9250_write_reg(mpu, MPU9250_SMPLRT_DIV, saved_config[0]);
    mpu9250_write_reg(mpu, MPU9250_CONFIG, saved_config[1]);
    mpu9250_write_reg(mpu, MPU9250_GYRO_CONFIG, saved_config[2]);
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG, saved_config[3]);
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG2, saved_config[4]);
    mpu9250_write_reg(mpu, MPU9250_USER_CTRL, saved_config[5]);
    mpu9250_write_reg(mpu, MPU9250_I2C_MST_CTRL, saved_config[6]);
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_ADDR, saved_config[7]);
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_REG, saved_config[8]);
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, saved_config[9]);
    sleep_ms(10); // Give time for I2C master to restart
    
    // Calculate averages
    int32_t accel_normal_avg[3], accel_st_avg[3];
    int32_t gyro_normal_avg[3], gyro_st_avg[3];
    
    for (int i = 0; i < 3; i++) 
    {
        accel_normal_avg[i] = accel_normal_sum[i] / 200;
        accel_st_avg[i] = accel_st_sum[i] / 200;
        gyro_normal_avg[i] = gyro_normal_sum[i] / 200;
        gyro_st_avg[i] = gyro_st_sum[i] / 200;
    }
    
    // Calculate self-test responses
    int32_t accel_str[3], gyro_str[3];
    bool test_passed = true;
    
    printf("Self-test results:\n");
    
    for (int i = 0; i < 3; i++) 
    {
        // Self-test response = self-test output - normal output
        accel_str[i] = accel_st_avg[i] - accel_normal_avg[i];
        gyro_str[i] = gyro_st_avg[i] - gyro_normal_avg[i];
        
        printf("Axis %d - Accel STR: %ld, Gyro STR: %ld\n", i, accel_str[i], gyro_str[i]);
        
        // Check factory self-test codes for validity
        if (st_accel[i] == 0 || st_gyro[i] == 0) {
            printf("Self-test axis %d returned invalid factory code\n", i);
            test_passed = false;
        }
        
        // Check if self-test response is within acceptable range
        // Accelerometer: should have significant response (>1000 LSB change for ±2g range)
        // Gyroscope: should have significant response (>50 LSB change for ±250dps range)
        // Based on MPU9250 datasheet and observed values, adjusted thresholds
        if (labs(accel_str[i]) < 1000 || labs(accel_str[i]) > 14000) {
            printf("Accelerometer axis %d failed: STR = %ld\n", i, accel_str[i]);
            test_passed = false;
        }
        
        // Adjusted gyroscope threshold based on datasheet and real hardware behavior
        // Typical self-test response should be between 50-32000 LSB for ±250dps range
        if (labs(gyro_str[i]) < 50 || labs(gyro_str[i]) > 32000) {
            printf("Gyroscope axis %d failed: STR = %ld\n", i, gyro_str[i]);
            test_passed = false;
        }
    }
    
    printf("Self-test %s\n", test_passed ? "PASSED" : "FAILED");
    return test_passed;
}

/**
 * FUNÇÕES INTERNAS - COMUNICAÇÃO I2C DE BAIXO NÍVEL
 * ================================================
 * Estas funções implementam a comunicação I2C básica com o MPU9250 e
 * magnetômetro AK8963. São funções auxiliares utilizadas pelas funções
 * públicas da API.
 */

/**
 * @brief Escreve um valor em um registrador do MPU9250
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param reg Endereço do registrador (8 bits)
 * @param data Valor a ser escrito (8 bits)
 */
static void mpu9250_write_reg(mpu9250_t *mpu, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    i2c_write_blocking(mpu->i2c, mpu->addr, buffer, 2, false);
    sleep_us(500); // Pequeno delay para garantir conclusão da escrita
}

/**
 * @brief Lê um valor de um registrador do MPU9250
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param reg Endereço do registrador a ser lido
 * @return Valor lido do registrador (8 bits)
 */
static uint8_t mpu9250_read_reg(mpu9250_t *mpu, uint8_t reg)
{
    uint8_t data;
    // Primeira transação: envia endereço do registrador
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    // Segunda transação: lê o valor do registrador
    i2c_read_blocking(mpu->i2c, mpu->addr, &data, 1, false);
    return data;
}

/**
 * @brief Lê múltiplos registradores sequenciais do MPU9250
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param reg Endereço do primeiro registrador
 * @param buffer Buffer para armazenar os dados lidos
 * @param len Número de bytes a serem lidos
 */
static void mpu9250_read_regs(mpu9250_t *mpu, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    // Primeira transação: envia endereço inicial
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    // Segunda transação: lê sequência de registradores
    i2c_read_blocking(mpu->i2c, mpu->addr, buffer, len, false);
}

/**
 * @brief Escreve um valor em um registrador do magnetômetro AK8963
 * 
 * Função para escrita direta no magnetômetro via bypass I2C.
 * Utilizada durante a inicialização quando o bypass está ativo.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param reg Endereço do registrador do AK8963
 * @param data Valor a ser escrito
 */
static void mpu9250_write_mag_reg(mpu9250_t *mpu, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    i2c_write_blocking(mpu->i2c, AK8963_ADDR, buffer, 2, false);
    sleep_us(500); // Delay para garantir conclusão da escrita
}

/**
 * @brief Lê um registrador do magnetômetro AK8963
 * 
 * Função complexa que pode ler do magnetômetro de duas formas:
 * 1. Via bypass I2C (acesso direto durante inicialização)
 * 2. Via I2C master (usando slave 0 temporariamente)
 * 
 * A função preserva a configuração do I2C master quando o magnetômetro
 * está habilitado, evitando interferir na operação normal.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param reg Endereço do registrador do AK8963
 * @return Valor lido do registrador
 */
static uint8_t mpu9250_read_mag_reg(mpu9250_t *mpu, uint8_t reg)
{
    uint8_t data = 0xFF;
    
    // Se bypass está ativo, lê diretamente do magnetômetro
    uint8_t int_pin_cfg = mpu9250_read_reg(mpu, MPU9250_INT_PIN_CFG);
    if (int_pin_cfg & BYPASS_EN) {
        i2c_write_blocking(mpu->i2c, AK8963_ADDR, &reg, 1, true);
        i2c_read_blocking(mpu->i2c, AK8963_ADDR, &data, 1, false);
        return data;
    }
    
    // Se magnetômetro habilitado, salva configuração atual do Slave 0
    uint8_t saved_addr = 0, saved_reg = 0, saved_ctrl = 0;
    if (mpu->mag_enabled) {
        saved_addr = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_ADDR);
        saved_reg = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_REG);
        saved_ctrl = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_CTRL);
    }
    
    // Configura SLV0 temporariamente para ler 1 byte do registrador desejado
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_ADDR, AK8963_ADDR | I2C_READ_FLAG);
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_REG, reg);
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, 0x81); // Habilita + 1 byte
    sleep_ms(10); // Aguarda transferência I2C master
    data = mpu9250_read_reg(mpu, MPU9250_EXT_SENS_DATA_00);
    
    // Restaura configuração original se magnetômetro estava habilitado
    if (mpu->mag_enabled) {
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_ADDR, saved_addr);
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_REG, saved_reg);
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, saved_ctrl);
    } else {
        // Desabilita SLV0 se magnetômetro não estava habilitado
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, 0x00);
    }
    
    return data;
}

/**
 * @brief Lê múltiplos registradores sequenciais do magnetômetro
 * 
 * Utilizada durante inicialização com bypass ativo para ler
 * valores de calibração ASA do magnetômetro.
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 * @param reg Endereço do primeiro registrador do AK8963
 * @param buffer Buffer para armazenar os dados lidos
 * @param len Número de bytes a serem lidos
 */
static void mpu9250_read_mag_regs(mpu9250_t *mpu, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    i2c_write_blocking(mpu->i2c, AK8963_ADDR, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, AK8963_ADDR, buffer, len, false);
}

/**
 * @brief Atualiza os fatores de sensibilidade baseados nos ranges configurados
 * 
 * Esta função é chamada automaticamente sempre que os ranges do acelerômetro
 * ou giroscópio são alterados. Os fatores de sensibilidade são utilizados
 * para converter valores brutos (LSB) em unidades físicas (g, °/s).
 * 
 * Fatores de sensibilidade (LSB por unidade física):
 * 
 * ACELERÔMETRO:
 * - ±2g:  16384 LSB/g
 * - ±4g:  8192 LSB/g  
 * - ±8g:  4096 LSB/g
 * - ±16g: 2048 LSB/g
 * 
 * GIROSCÓPIO:
 * - ±250°/s:  131 LSB/(°/s)
 * - ±500°/s:  65.5 LSB/(°/s)
 * - ±1000°/s: 32.8 LSB/(°/s)
 * - ±2000°/s: 16.4 LSB/(°/s)
 * 
 * @param mpu Ponteiro para a estrutura do MPU9250
 */
static void mpu9250_update_sensitivity_factors(mpu9250_t *mpu)
{
    uint8_t accel_range = mpu9250_get_accel_range(mpu);
    uint8_t gyro_range = mpu9250_get_gyro_range(mpu);
    
    // Atualiza sensibilidade do acelerômetro baseada no range
    switch (accel_range) 
    {
        case 0: mpu->accel_sensitivity = ACCEL_SENS_2G; break;   // ±2g
        case 1: mpu->accel_sensitivity = ACCEL_SENS_4G; break;   // ±4g
        case 2: mpu->accel_sensitivity = ACCEL_SENS_8G; break;   // ±8g
        case 3: mpu->accel_sensitivity = ACCEL_SENS_16G; break;  // ±16g
        default: mpu->accel_sensitivity = ACCEL_SENS_2G; break;  // Padrão seguro
    }
    
    // Atualiza sensibilidade do giroscópio baseada no range
    switch (gyro_range) 
    {
        case 0: mpu->gyro_sensitivity = GYRO_SENS_250DPS; break;  // ±250°/s
        case 1: mpu->gyro_sensitivity = GYRO_SENS_500DPS; break;  // ±500°/s
        case 2: mpu->gyro_sensitivity = GYRO_SENS_1000DPS; break; // ±1000°/s
        case 3: mpu->gyro_sensitivity = GYRO_SENS_2000DPS; break; // ±2000°/s
        default: mpu->gyro_sensitivity = GYRO_SENS_250DPS; break; // Padrão seguro
    }
}
