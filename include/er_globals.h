#ifndef ER_GLOBALS_H
#define ER_GLOBALS_H

#include <math.h>
#include <stdbool.h>
#include <sw/modules/iqmath/src/32b/IQmathLib.h>

#define M_2PI               (M_PI * 2)        /* 2pi */
#define M_1_2PI             (1 / (M_2PI))  /* 1/2pi */
#define M_RPM2RADS          ((M_2PI) / 60)
#define M_RADS2RPM          (60 / (M_2PI))

// PINOUT

// EEPROM
#define EEPROM_SPI_CLK      P18
#define EEPROM_SPI_MISO     P17
#define EEPROM_SPI_MOSI     P16
#define EEPROM_SPI_CS       P44
#define EEPROM_HOLD         P55
#define EEPROM_WP           P13

// BLDC

// ADC

// TACTILE

// DXL


typedef struct _ER_Configurations_t_
{
    uint16_t network_node_id;              // Node ID of microcontroller in network
    uint16_t network_broadcast_id;         // Node ID of broadcast in network

    bool     module_serial_enabled;       //
    bool     module_can_enabled;          //

    bool     module_tactile_enabled;      //
    bool     module_adc_enabled;          //
    bool     module_bldc_enabled;         // conflict with Dynamixel
    bool     module_dynamixel_enabled;    // conflict with BLDC

} ER_Configurations;


//! \brief Initialization values of global variables
//!
#define ER_Configurations_INIT {100, \
                               255, \
                               false, \
                               false, \
                               false, \
                               false, \
                               false, \
                               false}

#endif
