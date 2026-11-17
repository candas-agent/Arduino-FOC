#include "BLDCMotor.h"
#include "./communication/SimpleFOCDebug.h"


// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
// each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_120_map[6][3] = {
    {_HIGH_IMPEDANCE,1,-1},
    {-1,1,_HIGH_IMPEDANCE},
    {-1,_HIGH_IMPEDANCE,1},
    {_HIGH_IMPEDANCE,-1,1},
    {1,-1,_HIGH_IMPEDANCE},
    {1,_HIGH_IMPEDANCE,-1} 
};

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
// each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_150_map[12][3] = {
    {_HIGH_IMPEDANCE,1,-1},
    {-1,1,-1},
    {-1,1,_HIGH_IMPEDANCE},
    {-1,1,1},
    {-1,_HIGH_IMPEDANCE,1},
    {-1,-1,1},
    {_HIGH_IMPEDANCE,-1,1},
    {1,-1,1},
    {1,-1,_HIGH_IMPEDANCE},
    {1,-1,-1},
    {1,_HIGH_IMPEDANCE,-1},
    {1,1,-1} 
};

// BLDCMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
// - KV            - motor kv rating (rpm/V)
// - Lq            - motor q-axis inductance [H]
// - Ld            - motor d-axis inductance [H]
BLDCMotor::BLDCMotor(int pp, float _R, float _KV, float _Lq, float _Ld)
: FOCMotor()
{
  // save pole pairs number
  pole_pairs = pp;
  // save phase resistance number
  phase_resistance = _R;
  // save back emf constant KV = 1/KV
  // 1/sqrt(3) - rms value
  KV_rating = _KV;
  // save phase inductance
  axis_inductance = {_Ld, _Lq};
  phase_inductance = _Lq;  // FOR BACKWARDS COMPATIBILITY

  // torque control type is voltage by default
  torque_controller = TorqueControlType::voltage;
}


/**
	Link the driver which controls the motor
*/
void BLDCMotor::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}

// init hardware pins
int BLDCMotor::init() {
  if (!driver || !driver->initialized) {
    motor_status = FOCMotorStatus::motor_init_failed;
    SIMPLEFOC_MOTOR_ERROR("Init not possible, driver not init");
    return 0;
  }
  motor_status = FOCMotorStatus::motor_initializing;
  SIMPLEFOC_MOTOR_DEBUG("Init");

  // sanity check for the voltage limit configuration
  if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
  // constrain voltage for sensor alignment
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

  // update limits in the motor controllers
  updateCurrentLimit(current_limit);
  updateVoltageLimit(voltage_limit);
  updateVelocityLimit(velocity_limit);
  
  if(_isset(phase_inductance) && !(_isset(axis_inductance.q))) {
    // if only single inductance value is set, use it for both d and q axis
    axis_inductance = {phase_inductance, phase_inductance};
  } 
  
  // if using open loop control, set a CW as the default direction if not already set
  // only if no sensor is used
  if(!sensor){
    if ((controller==MotionControlType::angle_openloop
      ||controller==MotionControlType::velocity_openloop)
      && (sensor_direction == Direction::UNKNOWN)) {
        sensor_direction = Direction::CW;
    }
  }

  _delay(500);
  // enable motor
  SIMPLEFOC_MOTOR_DEBUG("Enable driver.");
  enable();
  _delay(500);
  motor_status = FOCMotorStatus::motor_uncalibrated;
  return 1;
}


// disable motor driver
void BLDCMotor::disable()
{
  // disable the current sense
  if(current_sense) current_sense->disable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // disable the driver
  driver->disable();
  // motor status update
  enabled = 0;
}
// enable motor driver
void BLDCMotor::enable()
{
  // enable the driver
  driver->enable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // enable the current sense
  if(current_sense) current_sense->enable();
  // reset the pids
  PID_velocity.reset();
  P_angle.reset();
  PID_current_q.reset();
  PID_current_d.reset();
  // motor status update
  enabled = 1;
}

/**
  FOC functions
*/

float BLDCMotor::estimateBEMF(float vel){
  // bemf constant is approximately 1/KV rating
  // V_bemf = K_bemf * velocity
  return vel/(KV_rating*_SQRT3)/_RPM_TO_RADS;
}


// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM, Sine PWM and Trapezoidal commutation algorithms
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {

  float center;
  int sector;
  float _ca,_sa;

  switch (foc_modulation)
  {
    case FOCModulationType::Trapezoid_120 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
      // determine the sector
      sector = 6 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_120_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_120_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON);// disable phase if possible
      }else{
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF);// disable phase if possible
      }

    break;

    case FOCModulationType::Trapezoid_150 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
      // determine the sector
      sector = 12 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_150_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][2]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF); // disable phase if possible
      }else{
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // enable all phases
      }

    break;

    case FOCModulationType::SinePWM :
    case FOCModulationType::SpaceVectorPWM :

        // Sinusoidal PWM modulation
        // Inverse Park + Clarke transformation
        _sincos(angle_el, &_sa, &_ca);

        // Inverse park transform
        Ualpha = _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
        Ubeta = _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;

        // Clarke transform
        Ua = Ualpha;
        Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
        Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

        // dead-time compensation
        // - phase current directions estimated from the dq currents (no noisy raw phase currents)
        // - linear ramp around zero current instead of sign() to avoid chattering at zero crossing
        // - a single division, reuses the modulation sin/cos (suited to MCUs without FPU)
        if (deadtime_compensation > 0 && motor_status == FOCMotorStatus::motor_ready) {
          // in voltage mode the dq "current" is the dq voltage, the resistance is folded into the gain
          float r = (torque_controller == TorqueControlType::voltage && _isset(phase_resistance) && phase_resistance > 0) ? phase_resistance : 1.0f;
          float th = deadtime_comp_threshold > 0 ? deadtime_comp_threshold : 1e-3f;
          float gain = deadtime_compensation / (th * r);
          // inverse Park + Clarke, Ic from Kirchhoff
          float Ia = _ca * deadtime_comp_current.d - _sa * deadtime_comp_current.q;
          float Ibeta = _sa * deadtime_comp_current.d + _ca * deadtime_comp_current.q;
          float Ib = -0.5f * Ia + _SQRT3_2 * Ibeta;
          float Ic = -Ia - Ib;
          Ua += _constrain(Ia * gain, -deadtime_compensation, deadtime_compensation);
          Ub += _constrain(Ib * gain, -deadtime_compensation, deadtime_compensation);
          Uc += _constrain(Ic * gain, -deadtime_compensation, deadtime_compensation);
        }

        // centering the voltages around either
        // - centered modulation: around driver.voltage_limit/2
        // - non-centered modulation: pulls the lowest voltage to 0 
        //     - Can be useful for low-side current sensing 
        //       in cases where the ADC had long sample time
        //     - The part of the duty cycle in which all phases are 
        //       off is longer than in centered modulation   
        //     - Both SinePWM and SpaceVectorPWM have the same form for non-centered modulation
      if (modulation_centered) {
        center = driver->voltage_limit/2;
        if (foc_modulation == FOCModulationType::SpaceVectorPWM){
          // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
          // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
          // Midpoint Clamp
          float Umin = min(Ua, min(Ub, Uc));
          float Umax = max(Ua, max(Ub, Uc));
          center -= (Umax+Umin) / 2;
        } 
        Ua += center;
        Ub += center;
        Uc += center;
      }else{
        float Umin = min(Ua, min(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
      }
      break;
  }

  // set the voltages in driver
  driver->setPwm(Ua, Ub, Uc);
}

// Measure the dead-time voltage error and set deadtime_compensation
// Applies a DC voltage on the d axis at electrical angle 0 (phase A axis), in both directions:
//   Ua = Ud, Ub = Uc = -Ud/2  ->  Ialpha = Ia = (Ud - U0) / R
// The dead-time error of the 3 phases seen on the alpha axis is U0 = 4/3 * Udt
// (Clarke: 2/3 * (Udt + Udt/2 + Udt/2)), so Udt = 3/4 * U0.
// Measuring both directions cancels the current sense offsets.
float BLDCMotor::calibrateDeadtimeCompensation(float max_current) {
  if (!current_sense || !current_sense->initialized) {
    SIMPLEFOC_MOTOR_ERROR("Fail. CS not init.");
    return NOT_SET;
  }
  if (max_current <= 0) {
    SIMPLEFOC_MOTOR_ERROR("Fail. I <= 0");
    return NOT_SET;
  }
  SIMPLEFOC_MOTOR_DEBUG("Meas deadtime..");

  // disable the compensation during the measurement
  FOCMotorStatus prev_status = motor_status;
  motor_status = FOCMotorStatus::motor_calibrating;

  // linear regression sums U = R*I + U0
  float sum_i = 0, sum_u = 0, sum_ii = 0, sum_iu = 0;
  int n = 0;
  const int n_points = 15;

  for (int dir = 1; dir >= -1; dir -= 2) {
    // align the rotor on the d axis so that it doesn't move during the measurement
    for (int i = 0; i <= 100; i++) {
      setPhaseVoltage(0, dir * voltage_sensor_align * i / 100.0f, 0);
      _delay(3);
    }
    _delay(300);

    // coarse ramp to find the voltage giving max_current
    float u_max = 0;
    for (float u = 0; u <= voltage_limit; u += voltage_limit / 500.0f) {
      setPhaseVoltage(0, dir * u, 0);
      _delay(2);
      DQCurrent_s c = current_sense->getDQCurrents(current_sense->getABCurrents(current_sense->getPhaseCurrents()), 0);
      u_max = u;
      if (dir * c.d >= max_current) break;
    }

    // measurement points between 30% and 100% of u_max
    for (int k = 0; k < n_points; k++) {
      float u = u_max * (0.3f + 0.7f * k / (n_points - 1));
      setPhaseVoltage(0, dir * u, 0);
      _delay(20);
      DQCurrent_s c = current_sense->getDQCurrents(current_sense->getABCurrents(current_sense->readAverageCurrents(10)), 0);
      // keep only the points well above the dead-time knee (non linear around zero current)
      if (fabsf(c.d) < 0.25f * max_current) continue;
      // |I| with the current sense offset of opposite sign in each direction:
      // fitting both directions together cancels the offset
      float i_s = dir * c.d;
      sum_i += i_s; sum_u += u; sum_ii += i_s * i_s; sum_iu += i_s * u;
      n++;
    }
    // smooth release
    for (int i = 100; i >= 0; i--) {
      setPhaseVoltage(0, dir * u_max * i / 100.0f, 0);
      _delay(2);
    }
    _delay(200);
  }
  setPhaseVoltage(0, 0, 0);
  motor_status = prev_status;

  float den = n * sum_ii - sum_i * sum_i;
  if (n < 4 || den <= 0) {
    SIMPLEFOC_MOTOR_ERROR("Fail. Not enough points");
    return NOT_SET;
  }
  float R = (n * sum_iu - sum_i * sum_u) / den;
  float U0 = (sum_u - R * sum_i) / n;
  if (R <= 0 || U0 <= 0) {
    SIMPLEFOC_MOTOR_ERROR("Fail. R or U0 <= 0");
    return NOT_SET;
  }
  float Udt = 0.75f * U0;
  SIMPLEFOC_MOTOR_DEBUG("Est. R: ", R);
  SIMPLEFOC_MOTOR_DEBUG("Est. deadtime comp: ", Udt);
  // with deadtime_comp_use_measured the ramp acts as a negative resistance, it has to stay below R
  SIMPLEFOC_MOTOR_DEBUG("Min. threshold (measured): ", Udt / R);
  deadtime_compensation = Udt;
  return Udt;
}
