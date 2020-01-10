#ifndef ER_GLOBALS_H
#define ER_GLOBALS_H

#include <math.h>
#include <stdbool.h>
#include <sw/modules/iqmath/src/32b/IQmathLib.h>

#define M_2PI               (M_PI * 2)        /* 2pi */
#define M_1_2PI             (1 / (M_2PI))  /* 1/2pi */
#define M_RPM2RADS          ((M_2PI) / 60)
#define M_RADS2RPM          (60 / (M_2PI))

//extern USER_Params gUserParams;

typedef struct _ER_Configurations_t_
{
    uint16_t network_node_id;               // Node ID of microcontroller in network
    uint16_t network_broadcast_id;          // Node ID of broadcast in network
//  uint16_t start_sequence = consts::calib_ss;   // Start sequence to determine whether this is a valid calibration
//  uint16_t erev_start;                      // Encoder reading at the start of an electrical revolution
//  uint8_t erevs_per_mrev = 1;                   // Electrical revolutions per mechanical revolution
//  uint8_t flip_phases = false;                  // Phases A, B, C are arranged in clockwise instead of ccw order
//  float foc_kp_d;                        // Proportional gain for FOC/d PI loop
//  float foc_ki_d;                        // Integral gain for FOC/d PI loop
//  float foc_kp_q;                        // Proportional gain for FOC/q PI loop
//  float foc_ki_q;                        // Integral gain for FOC/q PI loop
//  float velocity_kp;                     // Proportional gain for velocity PI loop
//  float velocity_kd;                    // Integral gain for velocity PI loop
//  float position_kp;                     // Proportional gain for position PI loop
//  float position_kd;                    // Integral gain for position PI loop
//  float current_limit;                   // Current limit (A)
//  float torque_limit;                    // Torque limit (N*m)
//  float velocity_limit;                 // Velocity limit (rad/s)
//  float position_lower_limit;            // Position lower limit (rad)
//  float position_upper_limit;            // Position upper limit (rad)
    uint16_t encoder_resolution_ppr;       // Encoder resolution in pulse per resolution
    uint16_t motor_pole_pairs;             // Motor pole pairs
    float    motor_resistance;             // Motor resistance (ohm)
    float    motor_inductance;             // Motor inductance (henries)

    _iq      motor_speed_limit;

    uint16_t gear_transmission;            // Gear transmission

//  float motor_torque_const;              // Motor torque constant (newton-meters per ampere)
    uint16_t control_timeout;                 // Control timeout (ms)
//  float hf_velocity_filter_param;       // Parameter for high frequency velocity estimate
//  float lf_velocity_filter_param; // Parameter for low frequency velocity estimate
//  float position_offset;                 // Position offset
//  float ia_offset;                       // Current Offset for Phase A
//  float ib_offset;                       // Current Offset for Phase B
//  float ic_offset = 0.0f;                       // Current Offset for Phase C
//  float enc_ang_corr_scale = 0.0f;              // Encoder angle correction scale (rad)
//  float enc_ang_corr_offset = 0.0f;             // Encoder angle correction offset (rad)
//  int8_t enc_ang_corr_table_values[consts::enc_ang_corr_table_size]; // Encoder angle correction table values

} ER_Configurations;


//! \brief Initialization values of global variables
//!
#define ER_Configurations_INIT {100, \
                               255, \
                               0, \
                               0, \
                               0.0, \
                               0.0, \
                               _IQ(1.6), \
                               1, \
                               100}

////! \brief      Defines the ER_Calibration handle
////! \details    The ER_Calibration handle is a pointer to a ER_Calibration object.
////!
//typedef struct _ER_Calibration_t_ *CAL_Handle;
//
//
////! \brief      Defines the ER_Calibration object
////!
//extern ER_Calibration g_calibration;


typedef struct _ER_Parameters_t_
{
  _iq torque_sp;                                  // Torque control setpoint (N*m)
  _iq15 velocity_sp;                                // Velocity control setpoint (rad/s)
  _iq15 motor_position_sp;                          // Position control setpoint (rad)
  _iq15 motor_position_sp_turns;                       // Position control setpoint (turns)
  _iq15 velocity_sp_krpm;                             // Velocity control setpoint (rpm)
//  float feed_forward;                                // Feed forward term for load compensation (A)

  _iq15   motor_speed_limit;                          // kRPM
  _iq15   motor_accel_limit;                          // kRPM/s
  _iq15   motor_decel_limit;                          // kRPM/s
  _iq20 motor_jerk_limit;                           // kRPM/s2
  _iq15   motor_temperature_limit;                    // C

  _iq15   motor_output_min_A;                         // A
  _iq15   motor_output_max_A;                         // A
  uint16_t goal_mode;                               // 3 bits are used - msb - flag (0 - mode changed, 1 - new mode),
                                                    // next - actual mode (0 - position, 1 - velocity), lsb - previous mode (0 - position, 1 - velocity)
                                                    // example 101 - new mode is position mode, previous mode was velocity mode
  bool enable_controller_flag;                      // Flag for whether the gates are active or not
  bool gate_fault;                                  // Flag for whether the gate has a fault

//  bool position_goal_changed_flag;                  // Flag for whether the new position command is set
//  bool velocity_goal_changed_flag;                  // Flag for whether the new position command is set
  bool timeout_flag;
  bool positon_set_zero;
//  bool previous_mode_velocity;                      // Flag - true if last mode was velocity, false if position_mode
} ER_Parameters;


//! \brief Initialization values of global variables
//!
#define ER_Parameters_INIT {_IQ15(0.0), \
                            _IQ15(0.0), \
                            _IQ15(0.0), \
                            _IQ15(0.0), \
                            _IQ15(0.0), \
                            _IQ15(1.6), \
                            _IQ15(4.0), \
                            _IQ15(4.0), \
                            _IQ20(50.0), \
                            _IQ15(0.0), \
                            _IQ15(-10.0), \
                            _IQ15(10.0), \
                            0, \
                            true, \
                            false, \
                            false, \
                            false}


//! \brief      Defines the ER_Parameters handle
//! \details    The ER_Parameters handle is a pointer to a ER_Parameters object.
//!
//typedef struct _ER_Parameters_t_ *PAR_Handle;


//! \brief      Defines the ER_Calibration object
//!
//extern ER_Parameters g_parameters;


typedef struct _ER_Results_t_
{
//   float foc_d_current = 0;                  // Measured FOC direct current (amperes)
//  float foc_q_current = 0;                  // Measured FOC quadrature current (amperes)
//  float foc_d_voltage = 0;                  // Measured FOC direct voltage (volts)
//  float foc_q_voltage = 0;                  // Measured FOC quadrature voltage (volts)
//
//  float id_output = 0;                      // id output from PID loop to motor
//  float iq_output = 0;                      // iq output from PID loop to motor
//
//  float duty_a = 0;                         // Calculated duty cycle for phase A
//  float duty_b = 0;                         // Calculated duty cycle for phase B
//  float duty_c = 0;                         // Calculated duty cycle for phase C

//  uint8_t encoder_mode = consts::encoder_mode_none; // Encoder mode
//  uint16_t raw_enc_value = 0;                       // Raw encoder value, wraps around
//  float enc_pos = 0;                                // Corrected encoder position, wraps around (radians)
//  uint32_t encoder_diag = 0;                        // Encoder diagnostics

    uint32_t __motor_prev_position;     // Rotor prev position [0 <-> encoder resolution - 1]
    uint32_t __motor_position;          // Rotor actual position [0 <-> encoder resolution - 1]
    int32_t  motor_position_ppr;        // Rotor position (ppr)
    _iq15    motor_position_rad;        // Rotor position (radians)
    _iq15    motor_position_turns;      // Rotor position (revolutions)

    _iq15    motor_velocity_rad;        // Rotor velocity (radians / s)
    _iq15    motor_velocity_rpm;        // Rotor velocity (revolutions / min)

    _iq15    motor_accel_rad;           // Rotor acceleration (radians / s*s)
    _iq15    motor_accel_rpm;           // Rotor acceleration (revolutions / min*min)

    _iq15    motor_torque;              // Rotor torque (Nm)

    _iq15    motor_current;             // Motor current (A)

    _iq15    motor_temperature;             // Motor temperature (C)

//  int16_t rotor_revs;                   // Total number of rotor revolutions
//  float rotor_pos;                      // Rotor position (radians)
//  float hf_rotor_vel;                   // Rotor velocity High Frequency Estimate (radians/second)
//  float lf_rotor_vel;                   // Rotor velocity Low Frequency Estimate (radians/second)

//  float va = 0;                             // voltage on phase A (volts)
//  float vb = 0;                             // voltage on phase B (volts)
//  float vc = 0;                             // voltage on phase C (volts)
//  float vin = 0;                            // supply voltage (volts)
//  float ia = 0;                             // current into phase A (amperes)
//  float ib = 0;                             // current into phase B (amperes)
//  float ic = 0;                             // current into phase C (amperes)
//
//  int32_t xl_x = 0;                         // X-acceleration in milli-g's
//  int32_t xl_y = 0;                         // Y-acceleration in milli-g's
//  int32_t xl_z = 0;                         // Z-acceleration in milli-g's
//
//  float temperature = 0;                    // Temperature in degrees Celsius

} ER_Results;


//! \brief Initialization values of global variables
//!
#define ER_Results_INIT {0, \
                         0, \
                         0, \
                         _IQ15(0.0), \
                         _IQ15(0.0), \
                         _IQ15(0.0), \
                         _IQ15(0.0), \
                         _IQ15(0.0), \
                         _IQ15(0.0), \
                         _IQ15(0.0), \
                         _IQ15(0.0), \
                         _IQ15(0.0)}


////! \brief      Defines the ER_Results handle
////! \details    The ER_Results handle is a pointer to a ER_Results object.
////!
//typedef struct _ER_Results_t_ *RES_Handle;
//
//
////! \brief      Defines the ER_Results object
////!
//extern ER_Results g_results;



//typedef struct _ER_Msg_t_
//{
//    uint16_t source_id;
//    uint16_t destination_id;
//    uint16_t register_id;
//    uint16_t flags;
//    uint16_t body_size;
//    uint16_t header_crc;
//    char * body;
//    uint16_t body_crc;
//} ER_Msg;
//! \brief Initialization values of global variables
//!

//#define ER_Msg_INIT {0, \
//                         0, \
//                         0, \
//                         0, \
//                         0, \
//                         0, \
//                         NULL, \
//                         0}

////! \brief      Defines the ER_Msg handle
////! \details    The ER_Msg handle is a pointer to a ER_Msg object.
////!
//typedef struct _ER_Msg_t_ *MSG_Handle;
//
//
////! \brief      Defines the ER_Msg object
////!
//extern ER_Msg g_msg;

//! \brief      Collects FSR's data
//! \details    After scanning all channels 5 times, average value is being calculated and stored in av_results array.

typedef struct _ER_FSR_t_{
    uint16_t results[9][5];                 // stores data from every channel from five scans
    uint16_t av_results[9];                 // stores average data calculated after 5 scans
    int orientation;                        // defines orientation of FSR
} ER_FSR;

#define ER_FSR_INIT {{{0},{0},{0},{0},{0},{0},{0},{0},{0}}, \
                     {0}, \
                     0}


#endif
