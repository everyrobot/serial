#ifndef ER_BLDC_H
#define ER_BLDC_H

// **************************************************************************
// the includes
#include <main_position.h>
#include <er_globals.h>
//#include <er_comm.h>

// **************************************************************************
// the defines


// **************************************************************************
// the typedefs
typedef volatile struct _ER_BLDC_Configurations_t_
{

    uint16_t encoder_resolution_ppr;       // Encoder resolution in pulse per resolution
    uint16_t motor_pole_pairs;             // Motor pole pairs
    float    motor_resistance;             // Motor resistance (ohm)
    float    motor_inductance;             // Motor inductance (henries)

    _iq      motor_speed_limit;

    uint16_t gear_transmission;            // Gear transmission

    uint16_t control_timeout;              // Control timeout (ms)

} ER_BLDC_Configurations;


//! \brief Initialization values of global variables
//!
#define ER_BLDC_Configurations_INIT {0, \
                                     0, \
                                     0.0, \
                                     0.0, \
                                     _IQ(1.6), \
                                     1, \
                                     100}

typedef volatile struct _ER_BLDC_Parameters_t_
{
  _iq       torque_sp;                                  // Torque control setpoint (N*m)
  _iq15     velocity_sp;                                // Velocity control setpoint (rad/s)
  _iq15     motor_position_sp;                          // Position control setpoint (rad)
  _iq15     motor_position_sp_turns;                       // Position control setpoint (turns)
  _iq15     velocity_sp_krpm;                             // Velocity control setpoint (rpm)

  _iq15     motor_speed_limit;                          // kRPM
  _iq15     motor_accel_limit;                          // kRPM/s
  _iq15     motor_decel_limit;                          // kRPM/s
  _iq20     motor_jerk_limit;                           // kRPM/s2
  _iq15     motor_temperature_limit;                    // C

  _iq15     motor_output_min_A;                         // A
  _iq15     motor_output_max_A;                         // A
  uint16_t  goal_mode;                               // 3 bits are used - msb - flag (0 - mode changed, 1 - new mode),
                                                    // next - actual mode (0 - position, 1 - velocity), lsb - previous mode (0 - position, 1 - velocity)
                                                    // example 101 - new mode is position mode, previous mode was velocity mode
  bool      enable_controller_flag;                      // Flag for whether the gates are active or not
  bool      gate_fault;                                  // Flag for whether the gate has a fault

  bool      timeout_flag;
  bool      positon_set_zero;
} ER_BLDC_Parameters;


//! \brief Initialization values of global variables
//!
#define ER_BLDC_Parameters_INIT {_IQ15(0.0), \
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


typedef volatile struct _ER_BLDC_Results_t_
{

    uint32_t    __motor_prev_position;     // Rotor prev position [0 <-> encoder resolution - 1]
    uint32_t    __motor_position;          // Rotor actual position [0 <-> encoder resolution - 1]
    int32_t     motor_position_ppr;        // Rotor position (ppr)
    _iq15       motor_position_rad;        // Rotor position (radians)
    _iq15       motor_position_turns;      // Rotor position (revolutions)

    _iq15       motor_velocity_rad;        // Rotor velocity (radians / s)
    _iq15       motor_velocity_rpm;        // Rotor velocity (revolutions / min)

    _iq15       motor_accel_rad;           // Rotor acceleration (radians / s*s)
    _iq15       motor_accel_rpm;           // Rotor acceleration (revolutions / min*min)

    _iq15       motor_torque;              // Rotor torque (Nm)

    _iq15       motor_current;             // Motor current (A)

    _iq15       motor_temperature;         // Motor temperature (C)

} ER_BLDC_Results;


////! \brief Initialization values of global variables
////!
#define ER_BLDC_Results_INIT {0, \
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



// **************************************************************************
// the globals
extern volatile MOTOR_Vars_t            gMotorVars;
extern USER_Params gUserParams;
extern ER_BLDC_Configurations  gBLDCConfigurationsVars;
extern ER_BLDC_Parameters      gBLDCParametersVars;
extern ER_BLDC_Results         gBLDCResultsVars;
extern volatile bool                    gFlag_Latch_softwareUpdate;
extern volatile uint_least16_t          gCounter_updateGlobals;
extern volatile _iq                     gMaxCurrentSlope;
extern DRV_SPI_8305_Vars_t              gDrvSpi8305Vars;
extern HAL_AdcData_t                    gAdcData;
extern HAL_PwmData_t                    gPwmData;

//extern uint_least8_t                    estNumber;
//extern uint_least8_t                    ctrlNumber;

extern _iq                              gFlux_pu_to_Wb_sf;
extern _iq                              gFlux_pu_to_VpHz_sf;
extern _iq                              gTorque_Ls_Id_Iq_pu_to_Nm_sf;
extern _iq                              gTorque_Flux_Iq_pu_to_Nm_sf;

extern CTRL_Handle                      ctrlHandle;
extern CTRL_Obj                         *controller_obj;
extern ENC_Handle encHandle;
extern ENC_Obj enc;

extern ST_Obj st_obj;
extern ST_Handle stHandle;


extern CTRL_Handle ctrlHandle;
extern HAL_Handle halHandle;
// **************************************************************************
// the function prototypes

void bldc__init();
void bldc__start();
void bldc__stop();

void bldc__update_state(HAL_Handle halHandle, CTRL_Handle ctrlHandle, ST_Handle stHandle);
void bldc__isr(HAL_Handle halHandle, CTRL_Handle ctrlHandle, ST_Handle stHandle, ENC_Handle encHandle);

void bldc__HAL_setupQEP(HAL_Handle handle,HAL_QepSelect_e qep);


//! \brief     Sets up the spiA peripheral
//! \param[in] handle  The hardware abstraction layer (HAL) handle
void bldc__HAL_setupSpiA(HAL_Handle handle);

//! \brief     Writes data to the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8305_Vars  SPI variables
void bldc__HAL_writeDrvData(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars);


//! \brief     Reads data from the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8305_Vars  SPI variables
void bldc__HAL_readDrvData(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars);

void bldc__HAL_setupGate(HAL_Handle handle);

//! \brief     Sets up the SPI interface for the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8305_Vars  SPI variables
void bldc__HAL_setupDrvSpi(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars);

void bldc__updateKpKiGains(CTRL_Handle handle);
void bldc__ST_runPosConv(ST_Handle handle, ENC_Handle encHandle, CTRL_Handle ctrlHandle);
void bldc__ST_runPosCtl(ST_Handle handle, CTRL_Handle ctrlHandle);
void bldc__ST_runPosMove(ST_Handle handle);
void bldc__updateGlobalVariables_motor(CTRL_Handle handle, ST_Handle sthandle);

#endif
