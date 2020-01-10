#ifndef ER_REGISTERS_H
#define ER_REGISTERS_H

// **************************************************************************
// the includes

// **************************************************************************
// the defines

//#define ONE_RPM     16667

// configuration (W) 0x00 - 0x39

// parameters (W) 0x40 - 0x79
#define ER_REG_SET_POSITION_SP      0x40
#define ER_REG_SET_SPEED_SP         0x41

#define ER_REG_SET_ACCEL            0x42
#define ER_REG_SET_DECEL            0x43
#define ER_REG_SET_POSITION_TO_ZERO 0x44

#define ER_REG_SET_SPEED_LIMIT      0x50
#define ER_REG_SET_CURRENT_LIMIT    0x51

#define ER_REG_INIT_MODULE          0x79

// results (R) 0x80 - 0xFF
#define ER_REG_GET_POSITION         0x80
#define ER_REG_GET_SPEED            0x81
#define ER_REG_GET_TORQUE           0x82
#define ER_REG_GET_ACCEL            0x83
#define ER_REG_GET_CURRENT          0x84

#define ER_REG_PING                 0xFF

// TACTILE MODULE ( 0x20 - 0x30 -0XA0)

#define ER_TACTILE_PCB1_GET_MEDIAN 0x21
#define ER_TACTILE_PCB2_GET_MEDIAN 0x22

#define ER_TACTILE_FINGER1_GET_MEDIAN 0x31

#define ER_TACTILE_PCB3_GET_MEDIAN 0x23
#define ER_TACTILE_PCB4_GET_MEDIAN 0x24

#define ER_TACTILE_FINGER2_GET_MEDIAN 0x32

#define ER_TACTILE_GRIPPER1_GET_MEDIAN 0xA1

#define ER_TACTILE_PCB5_GET_MEDIAN 0x25
#define ER_TACTILE_PCB6_GET_MEDIAN 0x26

#define ER_TACTILE_FINGER3_GET_MEDIAN 0x33

#define ER_TACTILE_PCB7_GET_MEDIAN 0x27
#define ER_TACTILE_PCB8_GET_MEDIAN 0x28

#define ER_TACTILE_FINGER4_GET_MEDIAN 0x34

#define ER_TACTILE_GRIPPER2_GET_MEDIAN 0xA2

#define ER_TACTILE_PCB9_GET_MEDIAN 0x29
#define ER_TACTILE_PCB10_GET_MEDIAN 0x2A

#define ER_TACTILE_FINGER5_GET_MEDIAN 0x35

//#define ER_TACTILE_ALLPCB_GET_VALUE               0x61

// **************************************************************************
// the typedefs


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#endif
