/* hw_config.c
Hardware configuration for SD Card with SPI interface
Based on no-OS-FatFS-SD-SPI-RPi-Pico library
*/

#include <assert.h>
#include <string.h>
//
#include "hw_config.h"
//
#include "ff.h" /* Obtains integer types */
//
#include "diskio.h" /* Declarations of disk functions */

/* 
Hardware configuration for SD Card on SPI0:

|       | SPI0  | GPIO  | Pin   | SPI       | MicroSD   | Description            | 
| ----- | ----  | ----- | ---   | --------  | --------- | ---------------------- |
| MISO  | RX    | 16    | 21    | DO        | DO        | Master In, Slave Out   |
| CS0   | CSn   | 17    | 22    | SS or CS  | CS        | Slave (or Chip) Select |
| SCK   | SCK   | 18    | 24    | SCLK      | CLK       | SPI clock              |
| MOSI  | TX    | 19    | 25    | DI        | DI        | Master Out, Slave In   |
| GND   |       |       | 18,23 |           | GND       | Ground                 |
| 3v3   |       |       | 36    |           | 3v3       | 3.3 volt power         |

*/

// Hardware Configuration of SPI "objects"
static spi_t spis[] = 
{  // One for each SPI.
    {
        .hw_inst = spi0,  // SPI component
        .miso_gpio = 16,  // GPIO number (not Pico pin number)
        .mosi_gpio = 19,
        .sck_gpio = 18,
        .baud_rate = 12500 * 1000  // 12.5 MHz
    }};

// Hardware Configuration of the SD Card "objects"
static sd_card_t sd_cards[] = 
{  // One for each SD card
    {
        .pcName = "0:",   // Name used to mount device
        .spi = &spis[0],  // Pointer to the SPI driving this card
        .ss_gpio = 17,    // The SPI slave select GPIO for this SD card
        .use_card_detect = false,  // No card detect pin used
        .card_detect_gpio = -1,    // Not used
        .card_detected_true = -1   // Not used
    }};

/* ********************************************************************** */
size_t sd_get_num() { 
    return sizeof(sd_cards) / sizeof(sd_cards[0]); 
}

sd_card_t *sd_get_by_num(size_t num) 
{
    if (num < sd_get_num()) 
    {
        return &sd_cards[num];
    } 
    else 
    {
        return NULL;
    }
}

size_t spi_get_num() 
{ 
    return sizeof(spis) / sizeof(spis[0]); 
}

spi_t *spi_get_by_num(size_t num) 
{
    if (num < spi_get_num()) 
    {
        return &spis[num];
    } 
    else 
    {
        return NULL;
    }
}
