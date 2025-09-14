/**
 * @file ds3231.c
 * @author  Alper Tunga Güven (alperguven@std.iyte.edu.tr)
 * @brief   Biblioteca de driver para o DS3231, desenvolvida para Raspberry Pi Pico.
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/DS3231.pdf
 * @version 0.1
 * @date    2023-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "ds3231.h"

/**
 * @brief               Função para ler um endereço de registrador específico via I2C.
 *
 * @param[in] i2c       Instância do barramento I2C utilizada.
 * @param[in] dev_addr  Endereço do dispositivo I2C.
 * @param[in] reg_addr  Endereço do registrador a ser lido.
 * @param[in] length    Quantidade de bytes a serem lidos.
 * @param[out] data     Buffer para armazenar os dados lidos.
 * @return              0 se bem-sucedido, -1 em caso de falha na comunicação I2C.
 */
int i2c_read_reg(i2c_inst_t * i2c, uint8_t dev_addr, uint8_t reg_addr, size_t length, uint8_t * data) 
{
    if(!length) 
        return -1;

    uint8_t reg = reg_addr; 

    if(i2c_write_blocking(i2c, dev_addr, &reg, 1, true) == PICO_ERROR_GENERIC) 
    {
        return -1;
    }

    if(i2c_read_blocking(i2c, dev_addr, data, length, false) == PICO_ERROR_GENERIC) 
    {
        return -1;
    }

    return 0;
}

/**
 * @brief               Função para escrever em um endereço de registrador específico via I2C.
 *
 * @param[in] i2c       Instância do barramento I2C utilizada.
 * @param[in] dev_addr  Endereço do dispositivo I2C.
 * @param[in] reg_addr  Endereço do registrador a ser escrito.
 * @param[in] length    Quantidade de bytes a serem escritos.
 * @param[in] data      Ponteiro para o buffer de dados a serem enviados.
 * @return              0 se bem-sucedido, -1 em caso de falha na comunicação I2C.
 */
int i2c_write_reg(i2c_inst_t * i2c, uint8_t dev_addr, uint8_t reg_addr, size_t length, uint8_t * data)
{
    if(!length) 
        return -1;

    uint8_t messeage[length + 1];

    messeage[0] = reg_addr;

    for(int i = 0; i < length; i++) 
    {
        messeage[i + 1] = data[i];
    }

    if(i2c_write_blocking(i2c, dev_addr, messeage, (length + 1), false) == PICO_ERROR_GENERIC)
        return -1;
    return 0;
}

/**
 * @brief           Converte um inteiro sem sinal de 8 bits para o formato BCD (Decimal Codificado em Binário),
 *                  utilizado nos registradores do DS3231.
 *
 * @param[in] data  Número a ser convertido.
 * @return          Valor convertido em BCD.
 */
uint8_t bin_to_bcd(const uint8_t data) 
{
    uint8_t ones_digit = (uint8_t)(data % 10);
    uint8_t twos_digit = (uint8_t)(data - ones_digit) / 10;
    return ((twos_digit << 4) + ones_digit);
}

/**
 * @brief           Converte um inteiro sem sinal de 8 bits para BCD, incluindo bits para indicar AM/PM.
 *                  Os bits 5 e 6 representam as características AM/PM.
 *
 * @param[in] data  Número a ser convertido.
 * @return          Valor em BCD com bits AM/PM.
 */
uint8_t bin_to_bcd_am_pm(uint8_t data) 
{
    uint8_t temp = bin_to_bcd(data);
    uint8_t am_pm = 0x00;

    if(data > 12) 
    {
        am_pm = 0x01;
        data -= 12;
    }

    temp |= (am_pm << 5);

    return temp;
}

/**
 * @brief                   Inicializa a estrutura ds3231 e define qual instância I2C será utilizada.
 *
 * @param[out] rtc          Ponteiro para a estrutura ds3231.
 * @param[in] i2c           Instância I2C escolhida.
 * @param[in] dev_addr      Endereço do DS3231. Use 0 para endereço padrão.
 * @param[in] eeprom_addr   Endereço do EEPROM. Use 0 para endereço padrão. Se um endereço incorreto for informado, será ajustado para o padrão.
 * @return                  0 se bem-sucedido.
 */
int ds3231_init(ds3231_t * rtc, i2c_inst_t * i2c, uint8_t dev_addr, uint8_t eeprom_addr) 
{
    rtc->am_pm_mode = false;
    rtc->i2c = i2c;
    if(dev_addr)
        rtc->ds3231_addr = dev_addr;
    else 
        rtc->ds3231_addr = DS3231_DEVICE_ADRESS;

    switch (eeprom_addr)
    {
    case AT24C32_EEPROM_ADRESS_0:
        rtc->at24c32_addr = AT24C32_EEPROM_ADRESS_0;
        break;
    case AT24C32_EEPROM_ADRESS_1:
        rtc->at24c32_addr = AT24C32_EEPROM_ADRESS_1;    
        break;
    case AT24C32_EEPROM_ADRESS_2:
        rtc->at24c32_addr = AT24C32_EEPROM_ADRESS_2;
        break;
    case AT24C32_EEPROM_ADRESS_3:
        rtc->at24c32_addr = AT24C32_EEPROM_ADRESS_3;
        break;
    case AT24C32_EEPROM_ADRESS_4:
        rtc->at24c32_addr = AT24C32_EEPROM_ADRESS_4;
        break;
    case AT24C32_EEPROM_ADRESS_5:
        rtc->at24c32_addr = AT24C32_EEPROM_ADRESS_5;
        break;
    case AT24C32_EEPROM_ADRESS_6:
        rtc->at24c32_addr = AT24C32_EEPROM_ADRESS_6;
        break;
    case AT24C32_EEPROM_ADRESS_7:
        rtc->at24c32_addr = AT24C32_EEPROM_ADRESS_7;
        break;
    default:
        rtc->at24c32_addr = AT24C32_EEPROM_ADRESS_0;
        break;
    }
    return 0;
}

/**
 * @brief               Habilita ou desabilita o modo AM/PM do DS3231. Por padrão, está desabilitado (modo 24h).
 *
 * @param[in] rtc       Estrutura DS3231.
 * @param[in] enable    Se true, ativa modo AM/PM. Se false, ativa modo 24 horas.
 * @return              0 se bem-sucedido.
 */
int ds3231_enable_am_pm_mode(ds3231_t * rtc, bool enable) 
{
    uint8_t temp = 0;

    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_HOURS_REG, 1, &temp))
        return -1;

    if(enable) 
    {
        temp |= (0x01 << 6);
        rtc->am_pm_mode = true;
    } 

    else 
    {
        rtc->am_pm_mode = false;
        temp &= ~(0x01 << 6);
    }

    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_HOURS_REG, 1, &temp))
        return -1;
    return 0;
}

/**
 * @brief               Configura a hora atual no DS3231.
 *
 * @param[in] rtc       Estrutura DS3231.
 * @param[in] data      Estrutura com os dados de tempo a serem configurados no DS3231.
 * @return              0 se bem-sucedido.
 */
int ds3231_configure_time(ds3231_t * rtc, ds3231_data_t * data) 
{
    uint8_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_SECONDS_REG, 7, temp)) 
        return -1;
    
    /* Checking if time values are within correct ranges. */
    if(data->seconds > 59) 
        data->seconds = 59;

    if(data->minutes > 59) 
        data->minutes = 59;

    if(rtc->am_pm_mode) 
    {
        if(data->hours > 12)
            data->hours = 12;
        else if(data->hours < 1)
            data->hours = 1;
    } 
    else 
    {
        if(data->hours > 23) 
            data->hours = 23;
    }

    if(data->day > 7) 
        data->day = 7;
    else if(data->day < 1)
        data->day = 1;
    
    if(data->date > 31) 
        data->date = 31;
    else if(data->date < 1)
        data->date = 1;

    if(data->month > 12) 
        data->month = 12;
    else if(data->month < 1)
        data->month = 1;
    
    if(data->year > 99) 
        data->year = 99;
        

    temp[0] = bin_to_bcd(data->seconds);

    temp[1] = bin_to_bcd(data->minutes);

    if(rtc->am_pm_mode) 
    {
        temp[2] = bin_to_bcd_am_pm(data->hours);
        temp[2] |= (0x01 << 6);
    } 
    else 
    {
        temp[2] = bin_to_bcd(data->hours);
        temp[2] &= ~(0x01 << 6);
    }

    temp[3] = bin_to_bcd(data->day);

    temp[4] = bin_to_bcd(data->date);

    temp[5] = bin_to_bcd(data->month);

    if(data->century)
        temp[5] |= (0x01 << 7);
    
    temp[6] = bin_to_bcd(data->year);
    
    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_SECONDS_REG, 7, temp)) 
        return -1;
    return 0;
}

/**
 * @brief               Lê os registradores de tempo do DS3231 e converte para valores reais (segundos, minutos, horas, etc).
 *
 * @param[in]   rtc     Estrutura ds3231.
 * @param[out]  data    Estrutura para armazenar os valores convertidos.
 * @return              0 se bem-sucedido.
 */
int ds3231_read_current_time(ds3231_t * rtc, ds3231_data_t * data) 
{
    uint8_t raw_data[7];
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_SECONDS_REG, 7, raw_data)) 
        return -1;
    
    data->seconds = 10 * ((raw_data[0] & 0x70) >> 4) + (raw_data[0] & 0x0F);

    data->minutes = 10 * ((raw_data[1] & 0x70) >> 4) + (raw_data[1] & 0x0F);
    
    if(rtc->am_pm_mode) 
    {
        data->hours   = 10 * ((raw_data[2] & 0x10) >> 4) + (raw_data[2] & 0x0F);
        data->am_pm = ((raw_data[2] & 0x20) >> 5);
    } 
    else 
    {
        data->hours   = 10 * ((raw_data[2] & 0x30) >> 4) + (raw_data[2] & 0x0F);
    }

    data->day     = (raw_data[3] & (0x07));

    data->date    = 10 * ((raw_data[4] & 0x30) >> 4) + (raw_data[4] & 0x0F);

    data->month   = 10 * ((raw_data[5] & 0x10) >> 4) + (raw_data[5] & 0x0F);

    data->century = (raw_data[5] & (0x01 << 7)) >> 7;

    data->year    = 10 * ((raw_data[6] & 0xF0) >> 4) + (raw_data[6] & 0x0F);

    return 0;
}

/**
 * @brief                   Configura o alarme 1 do DS3231. Modos de disparo válidos:
 *                          ON_EVERY_SECOND, ON_MATCHING_SECOND, ON_MATCHING_SECOND_AND_MINUTE,
 *                          ON_MATCHING_SECOND_MINUTE_AND_HOUR, ON_MATCHING_SECOND_MINUTE_HOUR_AND_DATE,
 *                          ON_MATCHING_SECOND_MINUTE_HOUR_AND_DAY.
 *
 * @param[in] rtc           Ponteiro para a estrutura DS3231.
 * @param[in] alarm_time    Estrutura com os dados de tempo para disparo do alarme.
 * @param[in] mask          Enum com o modo de disparo do alarme.
 * @return                  0 se bem-sucedido.
 */
int ds3231_set_alarm_1(ds3231_t * rtc, ds3231_alarm_1_t * alarm_time, enum ALARM_1_MASKS mask) 
{
    uint8_t temp[4] = {0, 0 , 0, 0};
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_SECONDS_ALARM_1_REG, 4, temp))
        return -1;

    if(alarm_time->seconds > 59) 
        alarm_time->seconds = 59;

    if(alarm_time->minutes > 59) 
        alarm_time->minutes = 59;

    if(rtc->am_pm_mode) 
    {
        if(alarm_time->hours > 12)
            alarm_time->hours = 12;
        else if(alarm_time->hours < 1)
            alarm_time->hours = 1;
    } 
    else 
    {
        if(alarm_time->hours > 23) 
            alarm_time->hours = 23;
    }

    if(alarm_time->day > 7) 
        alarm_time->day = 7;
    else if(alarm_time->day < 1)
        alarm_time->day = 1;
    
    if(alarm_time->date > 31) 
        alarm_time->date = 31;
    else if(alarm_time->date < 1)
        alarm_time->date = 1;

    
    switch(mask) 
    {
        case ON_EVERY_SECOND:
            for(int i = 0; i < 4; i++) 
                temp[i] |= (0x01 << 7);
        break;

        case ON_MATCHING_SECOND:
            temp[0] = bin_to_bcd(alarm_time->seconds);
            temp[0] &= ~(0x01 << 7);
            for(int i = 1; i < 4; i++) 
                temp[i] |= (0x01 << 7);
        break;

        case ON_MATCHING_SECOND_AND_MINUTE:
            temp[0] = bin_to_bcd(alarm_time->seconds);
            temp[1] = bin_to_bcd(alarm_time->minutes);
            for(int i = 0; i < 2; i++) 
                temp[i] &= ~(0x01 << 7);
            for(int i = 2; i < 4; i++) 
                temp[i] |= (0x01 << 7);
        break;

        case ON_MATCHING_SECOND_MINUTE_AND_HOUR:
            temp[0] = bin_to_bcd(alarm_time->seconds);
            temp[1] = bin_to_bcd(alarm_time->minutes);
            if(rtc->am_pm_mode) 
            {
                temp[2] = bin_to_bcd_am_pm(alarm_time->hours);
                temp[2] |= (0x01 << 6);        
            } 
            else 
            {
                temp[2] = bin_to_bcd(alarm_time->hours);
                temp[2] &= ~(0x01 << 6);        
            }     
            for(int i = 0; i < 3; i++)
                temp[i] &= ~(0x01 << 7);            
            temp[3] |= (0x01 << 7);
        break;

        case ON_MATCHING_SECOND_MINUTE_HOUR_AND_DATE:
            temp[0] = bin_to_bcd(alarm_time->seconds);
            temp[1] = bin_to_bcd(alarm_time->minutes);
            if(rtc->am_pm_mode) 
            {
                temp[2] = bin_to_bcd_am_pm(alarm_time->hours);
                temp[2] |= (0x01 << 6);        
            } 
            else 
            {
                temp[2] = bin_to_bcd(alarm_time->hours);
                temp[2] &= ~(0x01 << 6);        
            }     
            temp[3] = bin_to_bcd(alarm_time->date);
            temp[3] &= ~(0x01 << 6);
            for(int i = 0; i < 3; i++)
                temp[i] &= ~(0x01 << 7);            
            temp[3] |= (0x01 << 7);
        break;

        case ON_MATCHING_SECOND_MINUTE_HOUR_AND_DAY:
            temp[0] = bin_to_bcd(alarm_time->seconds);
            temp[1] = bin_to_bcd(alarm_time->minutes);
            if(rtc->am_pm_mode) 
            {
                temp[2] = bin_to_bcd_am_pm(alarm_time->hours);
                temp[2] |= (0x01 << 6);        
            } 
            else 
            {
                temp[2] = bin_to_bcd(alarm_time->hours);
                temp[2] &= ~(0x01 << 6);        
            }     
            temp[3] = bin_to_bcd(alarm_time->day);
            temp[3] |= (0x01 << 6);
            for(int i = 0; i < 3; i++)
                temp[i] &= ~(0x01 << 7);            
            temp[3] |= (0x01 << 7);
        break;

        default:
            return -1;
        break;
    }

    uint8_t alarm_enable = 0;
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &alarm_enable))
        return -1;    
    alarm_enable |= (0x01);
    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &alarm_enable))
        return -1;

    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_SECONDS_ALARM_1_REG, 4, temp))
        return -1;
    return 0;
}

/**
 * @brief                   Configura o alarme 2 do DS3231. Modos de disparo válidos:
 *                          ON_EVERY_MINUTE, ON_MATCHING_MINUTE, ON_MATCHING_MINUTE_AND_HOUR,
 *                          ON_MATCHING_MINUTE_HOUR_AND_DATE, ON_MATCHING_MINUTE_HOUR_AND_DAY.
 *
 * @param[in] rtc           Ponteiro para a estrutura DS3231.
 * @param[in] alarm_time    Estrutura com os dados de tempo para disparo do alarme.
 * @param[in] mask          Enum com o modo de disparo do alarme.
 * @return                  0 se bem-sucedido.
 */
int ds3231_set_alarm_2(ds3231_t * rtc, ds3231_alarm_2_t * alarm_time, enum ALARM_2_MASKS mask) 
{
    uint8_t temp[3] = {0, 0 , 0};
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_MINUTES_ALARM_2_REG, 3, temp))
        return -1;

    if(alarm_time->minutes > 59) 
        alarm_time->minutes = 59;

    if(rtc->am_pm_mode) 
    {
        if(alarm_time->hours > 12)
            alarm_time->hours = 12;
        else if(alarm_time->hours < 1)
            alarm_time->hours = 1;
    } 
    else 
    {
        if(alarm_time->hours > 23) 
            alarm_time->hours = 23;
    }

    if(alarm_time->day > 7) 
        alarm_time->day = 7;
    else if(alarm_time->day < 1)
        alarm_time->day = 1;
    
    if(alarm_time->date > 31) 
        alarm_time->date = 31;
    else if(alarm_time->date < 1)
        alarm_time->date = 1;

    switch(mask) 
    {
        case ON_EVERY_MINUTE:
            for(int i = 0; i < 3; i++)
                temp[i] |= (0x01 << 7);
        break;

        case ON_MATCHING_MINUTE:
            temp[0] = bin_to_bcd(alarm_time->minutes);
            temp[0] &= ~(0x01 << 7);
            for(int i = 1; i < 3; i++)
                temp[i] |= (0x01 << 7);
        break;

        case ON_MATCHING_MINUTE_AND_HOUR:
            temp[0] = bin_to_bcd(alarm_time->minutes);
            if(rtc->am_pm_mode) 
            {
                temp[1] = bin_to_bcd_am_pm(alarm_time->hours);
                temp[1] |= (0x01 << 6);
            } 
            else 
            {
                temp[1] = bin_to_bcd(alarm_time->hours);
                temp[1] &= ~(0x01 << 6);
            }
            for(int i = 0; i < 2; i++)
                temp[i] &= ~(0x01 << 7);
            temp[2] |= (0x01 << 7);
        break;

        case ON_MATCHING_MINUTE_HOUR_AND_DATE:
            temp[0] = bin_to_bcd(alarm_time->minutes);
            if(rtc->am_pm_mode) 
            {
                temp[1] = bin_to_bcd_am_pm(alarm_time->hours);
                temp[1] |= (0x01 << 6);
            } 
            else 
            {
                temp[1] = bin_to_bcd(alarm_time->hours);
                temp[1] &= ~(0x01 << 6);
            }
            temp[2] = bin_to_bcd(alarm_time->date);
            temp[2] &= ~(0x01 << 6);
            for(int i = 0; i < 3; i++)
                temp[i] &= ~(0x01 << 7);
        break;

        case ON_MATCHING_MINUTE_HOUR_AND_DAY:
            temp[0] = bin_to_bcd(alarm_time->minutes);
            if(rtc->am_pm_mode) 
            {
                temp[1] = bin_to_bcd_am_pm(alarm_time->hours);
                temp[1] |= (0x01 << 6);
            } 
            else 
            {
                temp[1] = bin_to_bcd(alarm_time->hours);
                temp[1] &= ~(0x01 << 6);
            }
            temp[2] = bin_to_bcd(alarm_time->date);
            temp[2] |= (0x01 << 6);
            for(int i = 0; i < 3; i++)
                temp[i] &= ~(0x01 << 7);
        break;

        default:
            return -1;
        break;
    }

    uint8_t alarm_enable = 0;
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &alarm_enable))
        return -1;    
    alarm_enable |= (0x02);
    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &alarm_enable))
        return -1;

    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_MINUTES_ALARM_2_REG, 3, temp))
        return -1;
    return 0;
}


/**
 * @brief               Habilita ou desabilita a interrupção de alarme no DS3231. Se qualquer alarme estiver ativado,
 *                      o DS3231 enviará um sinal pelo pino INT/SQW. Esta funcionalidade é exclusiva com a saída de onda quadrada.
 *
 * @param[in] rtc       Estrutura DS3231.
 * @param[in] enable    Habilita se true, desabilita se false.
 * @return              0 se bem-sucedido.
 */
int ds3231_enable_alarm_interrupt(ds3231_t * rtc, bool enable) {
    uint8_t interrupt_enable = 0;
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &interrupt_enable))
        return -1;    
    if(enable)
        interrupt_enable |= (0x01 << 2);
    else 
        interrupt_enable &= ~(0x01 << 2);
    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &interrupt_enable))
        return -1;
    return 0;
} 

/**
 * @brief               Habilita ou desabilita a saída de onda quadrada de 32.768kHz no pino 32K do DS3231.
 *                      Se desabilitado, o pino entra em modo de alta impedância.
 *
 * @param[in] rtc       Estrutura DS3231.
 * @param[in] enable    Habilita se true, desabilita se false.
 * @return              0 se bem-sucedido.
 */
int ds3231_enable_32khz_square_wave(ds3231_t * rtc, bool enable) 
{
    uint8_t status = 0;
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_STATUS_REG, 1, &status))
        return -1;
    if(enable) 
        status |= (0x01 << 3);
    else 
        status &= ~(0x01 << 3);
    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_STATUS_REG, 1, &status))
        return -1;    
    return 0;    
}

/**
 * @brief               Habilita ou desabilita o oscilador do DS3231. Por padrão, está habilitado.
 *                      Se desabilitado, o oscilador para quando alimentado apenas pela bateria (Vbat).
 *
 * @param[in] rtc       Estrutura DS3231.
 * @param[in] enable    Habilita se true, desabilita se false.
 * @return              0 se bem-sucedido.
 */
int ds3231_enable_oscillator(ds3231_t * rtc, bool enable) 
{
    uint8_t enable_byte = 0;
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &enable_byte))
        return -1;    
    if(!enable)
        enable_byte |= (0x01 << 7);
    else 
        enable_byte &= ~(0x01 << 7);
    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &enable_byte))
        return -1; 
    return 0;   
}

/**
 * @brief               Habilita ou desabilita a saída de onda quadrada no pino INT/SQW.
 *                      Esta funcionalidade é desabilitada se as interrupções de alarme estiverem ativas. Por padrão, está desabilitada.
 *
 * @param[in] rtc       Estrutura DS3231.
 * @param[in] enable    Habilita se true, desabilita se false.
 * @return              0 se bem-sucedido.
 */
int ds3231_enable_battery_backed_square_wave(ds3231_t * rtc, bool enable) 
{
    uint8_t enable_byte = 0;
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &enable_byte))
        return -1;    
    if(enable) {
        enable_byte |= (0x01 << 6);
        enable_byte &= ~(0x01 << 2);
    }
    else 
        enable_byte &= ~(0x01 << 6);
    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &enable_byte))
        return -1;    
    return 0;
}

/**
 * @brief               Define a frequência da saída de onda quadrada no pino INT/SQW.
 *                      Se a saída de onda quadrada estiver ativa, o DS3231 não pode enviar sinais de alarme.
 *                      Frequências válidas: FREQUENCY_1_HZ, FREQUENCY_1024_HZ, FREQUENCY_4096_HZ, FREQUENCY_8192_HZ.
 *
 * @param rtc           Estrutura DS3231.
 * @param sqr_frq       Enum da frequência desejada.
 * @return              0 se bem-sucedido.
 */
int ds3231_set_square_wave_frequency(ds3231_t * rtc, enum SQUARE_WAVE_FREQUENCY sqr_frq) 
{
    uint8_t enable_byte = 0;
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &enable_byte))
        return -1;    

    enable_byte &= ~(0x18);

    switch(sqr_frq) {
        case FREQUENCY_1_HZ:
        enable_byte |= (FREQUENCY_1_HZ << 3);
        break;

        case FREQUENCY_1024_HZ:
        enable_byte |= (FREQUENCY_1024_HZ << 3);
        break;

        case FREQUENCY_4096_HZ:
        enable_byte |= (FREQUENCY_4096_HZ << 3);
        break;

        case FREQUENCY_8192_HZ:
        enable_byte |= (FREQUENCY_8192_HZ << 3);
        break;

        default:
            return -1;
        break;
    }

    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &enable_byte))
        return -1;   
    return  0;
}

/**
 * @brief           Força o sensor de temperatura a realizar uma conversão e atualizar o algoritmo TCXO,
 *                  ajustando a capacitância do oscilador.
 *
 * @param[in] rtc   Estrutura DS3231.
 * @return          0 se bem-sucedido.
 */
int ds3231_force_convert_temperature(ds3231_t * rtc) 
{
    uint8_t status = 0;
    /* Read the status register to check the BSY bit. */
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_STATUS_REG, 1, &status)) 
        return -1;
    /* If BSY bit in the status register is logic 1, the temperature conversion cannot be initiated. */
    if(status & (0x01 << 2)) 
        return -1;
    /* Read the control register and set the CONV bit to logic 1. */
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &status)) 
        return -1;    
    status |= (0x01 <<  5);
    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_REG, 1, &status)) 
        return -1;
    return 0;
}

/**
 * @brief                   Lê o valor de temperatura do DS3231.
 *
 * @param[in] rtc           Estrutura DS3231.
 * @param[out] temperature  Valor de temperatura lido, com resolução de 0,25°C.
 * @return                  0 se bem-sucedido.
 */
int ds3231_read_temperature(ds3231_t * rtc, float * temperature) 
{
    uint8_t temp[2] = {0, 0};
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_TEMPERATURE_MSB_REG, 2, temp))
        return -1;
    
    *temperature = temp[0] + (float)(1 / (temp[1] >> 6));
    return 0;
}

/**
 * @brief           Verifica o registrador de status do DS3231 para saber se o oscilador está funcionando.
 *
 * @param[in] rtc   Estrutura DS3231.
 * @return          0 se o oscilador está funcionando, 1 se parou, -1 em caso de erro I2C.
 */
int ds3231_check_oscillator_stop_flag(ds3231_t * rtc) 
{
    uint8_t status = 0;
    if(i2c_read_reg(rtc->i2c, rtc->ds3231_addr, DS3231_CONTROL_STATUS_REG, 1, &status))
        return -1;
    if(status & (0x01 << 7)) 
        return 1;
    return 0;
}

/**
 * @brief               Define o valor de offset de envelhecimento para calibrar o oscilador do DS3231.
 *                      O valor é codificado em complemento de dois, sendo o bit 7 o sinal.
 *                      Consulte o datasheet para detalhes de calibração.
 *
 * @param[in] rtc       Estrutura DS3231.
 * @param[in] offset    Valor de offset a ser escrito no registrador.
 * @return              0 se bem-sucedido.
 */
int ds3231_set_aging_offset(ds3231_t * rtc, int8_t offset) 
{
    int8_t temp = offset;
    uint8_t aging_offset = *((uint8_t *)(&temp));
    if(i2c_write_reg(rtc->i2c, rtc->ds3231_addr, DS3231_AGING_OFFSET_REG, 1, &aging_offset))
        return -1;    
    return 0;
}

/**
 * @brief               Define uma função de callback para ser chamada quando o DS3231 enviar um sinal de alarme.
 *                      Cada core do RP2040 pode ter apenas uma função de callback de interrupção.
 *
 * @param[out] gpio     Pino que receberá o sinal de interrupção.
 * @param[out] callback Ponteiro para a função de callback.
 * @return              0 se bem-sucedido.
 */
int ds3231_set_interrupt_callback_function(uint gpio, gpio_irq_callback_t callback) 
{
    /* Set the pin that will trigger the interrupt as an input pull-up pin. */
    gpio_init(gpio);
    gpio_set_dir(gpio, 0);
    gpio_pull_up(gpio);

    gpio_set_irq_enabled_with_callback(gpio, GPIO_IRQ_EDGE_FALL, true, callback);
    return 0;
}
