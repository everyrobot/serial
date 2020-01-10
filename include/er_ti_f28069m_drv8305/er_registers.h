#ifndef ER_REGISTERS_H
#define ER_REGISTERS_H

// **************************************************************************
// the includes

// **************************************************************************
// the defines

//#define ONE_RPM     16667

// configuration (W) 0x00 - 0x39

// parameters (W) 0x40 - 0x79
#define ER_REG_BLDC_SET_POSITION_SP             0x40
#define ER_REG_BLDC_SET_SPEED_SP                0x41

#define ER_REG_BLDC_SET_ACCEL                   0x42
#define ER_REG_BLDC_SET_DECEL                   0x43
#define ER_REG_BLDC_SET_POSITION_TO_ZERO        0x44

#define ER_REG_BLDC_SET_SPEED_LIMIT             0x50
#define ER_REG_BLDC_SET_CURRENT_LIMIT           0x51

#define ER_REG_BLDC_INIT_MODULE                 0x79

// results (R) 0x80 - 0xFF
#define ER_REG_BLDC_GET_POSITION                0x80
#define ER_REG_BLDC_GET_SPEED                   0x81
#define ER_REG_BLDC_GET_TORQUE                  0x82
#define ER_REG_BLDC_GET_ACCEL                   0x83
#define ER_REG_BLDC_GET_CURRENT                 0x84

#define ER_REG_ADC_1_GET_VALUE                  0x84
#define ER_REG_ADC_2_GET_VALUE                  0x84
#define ER_REG_ADC_3_GET_VALUE                  0x84
#define ER_REG_ADC_4_GET_VALUE                  0x84
#define ER_REG_ADC_5_GET_VALUE                  0x84
#define ER_REG_ADC_6_GET_VALUE                  0x84
#define ER_REG_ADC_7_GET_VALUE                  0x84
#define ER_REG_ADC_8_GET_VALUE                  0x84

#define ER_REG_PING                             0xFF

// -- EEPROM REGISTERS ------------------------------------------------------
// MODULES
#define ER_CONF_REG_MODULE_GLOBAL               0x01
#define ER_CONF_REG_MODULE_BLDC                 0x02
#define ER_CONF_REG_MODULE_ADC                  0x03
#define ER_CONF_REG_MODULE_TACTILE              0x04
//#define ER_CONF_REG_MODULE_DXL                  0x04

// GLOBAL MODULE
#define ER_CONF_REG_GLOBAL_NODE_ID              0x11
//#define ER_CONF_REG_GLOBAL_NODE_ID              0x11

// BLDC MODULE
//#define ER_CONF_REG_BLDC_RUN_ON_START           0x21
//#define ER_CONF_REG_BLDC_MOTOR_TYPE             0x21
//#define ER_CONF_REG_BLDC_MOTOR_NUM_POLE_PAIRS   0x21
//#define ER_CONF_REG_BLDC_MOTOR_Rs               0x21
//#define ER_CONF_REG_BLDC_MOTOR_Ls_d             0x21
//#define ER_CONF_REG_BLDC_MOTOR_Ls_q             0x21
//#define ER_CONF_REG_BLDC_MOTOR_RATED_FLUX       0x21
//#define ER_CONF_REG_BLDC_MOTOR_MAX_CURRENT      0x21
//#define ER_CONF_REG_BLDC_MOTOR_FLUX_EST_FREQ_Hz 0x21
//#define ER_CONF_REG_BLDC_MOTOR_ENCODER_LINES    0x21
//#define ER_CONF_REG_BLDC_MOTOR_MAX_SPEED_KRPM   0x21
//#define ER_CONF_REG_BLDC_SYSTEM_INERTIA         0x21
//#define ER_CONF_REG_BLDC_SYSTEM_FRICTION        0x21
//
//// ADC MODULE
//#define ER_CONF_REG_ADC_RUN_ON_START                0x21
//#define ER_CONF_REG_ADC_DATA_SIZE                   0x21 // Ile odczytow ma byc brane pod uwage w wyliczaniu sredniej
//#define ER_CONF_REG_ADC_1_ENABLE                    0x21
//#define ER_CONF_REG_ADC_2_ENABLE                    0x21
//#define ER_CONF_REG_ADC_3_ENABLE                    0x21
//#define ER_CONF_REG_ADC_4_ENABLE                    0x21
//#define ER_CONF_REG_ADC_5_ENABLE                    0x21
//#define ER_CONF_REG_ADC_6_ENABLE                    0x21
//#define ER_CONF_REG_ADC_7_ENABLE                    0x21
//#define ER_CONF_REG_ADC_8_ENABLE                    0x21

// TACTILE MODULE ( 0x20 PCB - 0x30 FINGER -0XA0 GRIPPER)

#define ER_TACTILE_PCB1_GET_MEDIAN                0x21
#define ER_TACTILE_PCB2_GET_MEDIAN                0x22

#define ER_TACTILE_FINGER1_GET_MEDIAN             0x31

#define ER_TACTILE_PCB3_GET_MEDIAN                0x23
#define ER_TACTILE_PCB4_GET_MEDIAN                0x24

#define ER_TACTILE_FINGER2_GET_MEDIAN             0x32

#define ER_TACTILE_GRIPPER1_GET_MEDIAN            0xA1

#define ER_TACTILE_PCB5_GET_MEDIAN                0x25
#define ER_TACTILE_PCB6_GET_MEDIAN                0x26

#define ER_TACTILE_FINGER3_GET_MEDIAN             0x33

#define ER_TACTILE_PCB7_GET_MEDIAN                0x27
#define ER_TACTILE_PCB8_GET_MEDIAN                0x28

#define ER_TACTILE_FINGER4_GET_MEDIAN             0x34

#define ER_TACTILE_GRIPPER2_GET_MEDIAN            0xA2

#define ER_TACTILE_PCB9_GET_MEDIAN                0x29
#define ER_TACTILE_PCB10_GET_MEDIAN               0x2A

#define ER_TACTILE_FINGER5_GET_MEDIAN             0x35

//#define ER_TACTILE_ALLPCB_GET_VALUE               0x61



//#define ER_CONF_REG_TACTILE_RUN_ON_START            0x21
//#define ER_CONF_REG_TACTILE_DATA_SIZE               0x21 // Ile odczytow ma byc brane pod uwage w wyliczaniu sredniej
//#define ER_CONF_REG_TACTILE_TACTILE1_ENABLE         0x21
//#define ER_CONF_REG_TACTILE_TACTILE1_TYPE           0x21
//#define ER_CONF_REG_TACTILE_TACTILE1_ORIENTATION    0x21
//#define ER_CONF_REG_TACTILE_TACTILE2_ENABLE         0x21
//#define ER_CONF_REG_TACTILE_TACTILE2_TYPE           0x21
//#define ER_CONF_REG_TACTILE_TACTILE2_ORIENTATION    0x21
//#define ER_CONF_REG_TACTILE_TACTILE3_ENABLE         0x21
//#define ER_CONF_REG_TACTILE_TACTILE3_TYPE           0x21
//#define ER_CONF_REG_TACTILE_TACTILE3_ORIENTATION    0x21
//#define ER_CONF_REG_TACTILE_TACTILE4_ENABLE         0x21
//#define ER_CONF_REG_TACTILE_TACTILE4_TYPE           0x21
//#define ER_CONF_REG_TACTILE_TACTILE4_ORIENTATION    0x21
//#define ER_CONF_REG_TACTILE_TACTILE5_ENABLE         0x21
//#define ER_CONF_REG_TACTILE_TACTILE5_TYPE           0x21
//#define ER_CONF_REG_TACTILE_TACTILE5_ORIENTATION    0x21
//#define ER_CONF_REG_TACTILE_TACTILE6_ENABLE         0x21
//#define ER_CONF_REG_TACTILE_TACTILE6_TYPE           0x21
//#define ER_CONF_REG_TACTILE_TACTILE6_ORIENTATION    0x21
//#define ER_CONF_REG_TACTILE_TACTILE7_ENABLE         0x21
//#define ER_CONF_REG_TACTILE_TACTILE7_TYPE           0x21
//#define ER_CONF_REG_TACTILE_TACTILE7_ORIENTATION    0x21
//#define ER_CONF_REG_TACTILE_TACTILE8_ENABLE         0x21
//#define ER_CONF_REG_TACTILE_TACTILE8_TYPE           0x21
//#define ER_CONF_REG_TACTILE_TACTILE8_ORIENTATION    0x21
//#define ER_CONF_REG_TACTILE_TACTILE9_ENABLE         0x21
//#define ER_CONF_REG_TACTILE_TACTILE9_TYPE           0x21
//#define ER_CONF_REG_TACTILE_TACTILE9_ORIENTATION    0x21
//#define ER_CONF_REG_TACTILE_TACTILE10_ENABLE        0x21
//#define ER_CONF_REG_TACTILE_TACTILE10_TYPE          0x21
//#define ER_CONF_REG_TACTILE_TACTILE10_ORIENTATION   0x21

// **************************************************************************
// the typedefs


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#endif
