#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/units.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <sys/types.h>

#include <bldc_servo.hpp>
#include <resource_list.hpp>

using namespace std::chrono_literals;

namespace sjsu::perseus {

// ...existing code...
bldc_perseus::bldc_perseus(hal::v5::strong_ptr<sjsu::drivers::h_bridge> p_hbridge,
                           hal::v5::strong_ptr<hal::rotation_sensor> p_encoder)
  : m_h_bridge(p_hbridge)
  , m_encoder(p_encoder)
  , m_clock(resources::clock())
{
  m_last_clock_check = m_clock->uptime(); 

  m_reading = {
    .position = 0,
    .power = 0, 
    .velocity = 0,  
  };
  m_target = { 
    .position = 0, 
    .power = 0.0f , 
    .velocity = 0
  };
  m_clamped_power = 0.3;
  m_prev_encoder_value = bldc_perseus::read_angle();
  m_PID_prev_velocity_values = {
    .integral = 0, 
    .last_error = 0, 
    .prev_dt_time = 0.0 
  };
  m_PID_prev_position_values = { 
    .integral = 0,              
    .last_error = 0,
    .prev_dt_time = 0.0 
  };
  // elbow 
  m_servo_values = {
    .gear_ratio = 5281.1, // 5281.1 * 2 / 2
    .feedforward_clamp = 0.2, 
    .length = 0.4826, 
    .angle_offset = -20, 
    .weight_beam = 1000, 
    .weight_end = 600 
  }; 
  // // shoulder 
  // m_servo_values = {
  //   .gear_ratio = 73935.4, // 5281.1 * 28 / 2
  //   .feedforward_clamp = 0, 
  //   .length = 0.5715, 
  //   .angle_offset = -20, 
  //   .weight_beam = 1600, 
  //   .weight_end = 1600 
  // }; 
  // // wrist 
  // m_servo_values = {
  //   .gear_ratio = 2640.55, // 5281.1 * 1 / 2
  //   .feedforward_clamp = 0.2,
  //   .length = 0.762, 
  //   .angle_offset = 0, 
  //   .weight_beam = 500, 
  //   .weight_end = 100 
  // }; 
  // // track 
  // m_servo_values = {
  //   .gear_ratio = 8.35333, // 751.8 * 1 / 2 * 8 / 360
  //   .feedforward_clamp = 0,
  //   .length = 0, 
  //   .angle_offset = 0, 
  //   .weight_beam = 0, 
  //   .weight_end = 0 
  // }; 
}

void bldc_perseus::set_target_position(float target_position)
{
  m_target.position = target_position;
}

float bldc_perseus::get_target_position()
{
  return m_target.position;
}

float bldc_perseus::get_reading_position()
{
  m_reading.position = read_angle();
  return m_reading.position;
}

void bldc_perseus::set_target_velocity(float target_velocity)
{
  m_target.velocity = target_velocity;
}

float bldc_perseus::get_target_velocity()
{
  return m_target.velocity;
}
float bldc_perseus::get_reading_velocity()
{
  // TODO! 
  return m_reading.velocity;
}

void bldc_perseus::set_power(float power) {
  m_h_bridge->power(power);
}


void bldc_perseus::stop()
{
  m_reading.power = 0.0f;
  m_h_bridge->power(0.0f);
}


bldc_perseus::PID_settings bldc_perseus::get_pid_settings()
{
  return m_reading_position_settings;
}
void bldc_perseus::update_pid_position(PID_settings settings)
{
  m_reading_position_settings = settings;
}
void bldc_perseus::update_pid_velocity(PID_settings settings)
{
  m_reading_velocity_settings = settings;
}
void bldc_perseus::home_encoder()
{
  // TODO!
  home_encoder_value = read_angle();
  m_reading.position = 0;
}

hal::degrees bldc_perseus::read_angle() {
  return m_encoder->read().angle / m_servo_values.gear_ratio; 
}

void bldc_perseus::update_velocity(int from_scratch) 
{
  // TODO : implement velocity PID control
  from_scratch = from_scratch + 1; 
}

void bldc_perseus::reset_time()
{
  m_PID_prev_velocity_values = { .integral = 0,
                                 .last_error = 0,
                                 .prev_dt_time = 0.0 };
  m_PID_prev_position_values = { .integral = 0,
                                 .last_error = 0,
                                 .prev_dt_time = 0.0 };
  m_last_clock_check = m_clock->uptime();
}

void bldc_perseus::set_pid_clamped_power(float power)
{
  m_clamped_power = power; 
}
float bldc_perseus::get_pid_clamped_power()
{
  return m_clamped_speed; 
}
hal::time_duration bldc_perseus::get_clock_time(hal::steady_clock& p_clock)
{
  hal::time_duration const period =
    sec_to_hal_time_duration(1.0 / p_clock.frequency());
  return period * p_clock.uptime();
}
// position 
void bldc_perseus::update_position(int new_pos, int servo) 
{
  // pid portion
  m_reading.position = bldc_perseus::read_angle();
  float error = m_target.position - m_reading.position;
  sec curr_time = hal_time_duration_to_sec(get_clock_time(*m_clock));
  sec dt = curr_time - m_PID_prev_position_values.prev_dt_time;
  // current_time += dt; 
  if (new_pos) {
    m_PID_prev_position_values.integral = 0;
  } 
  else {
    m_PID_prev_position_values.integral += error * dt; 
  }
  float derivative = (error - m_PID_prev_position_values.last_error) / dt; // this turns out to be negative because hopefully your current error is less than your last error
  float pTerm = m_current_position_settings.kp * error; 
  float iTerm  = m_current_position_settings.ki * m_PID_prev_position_values.integral; 
  float dTerm = m_current_position_settings.kd * derivative; 
  m_PID_prev_position_values.last_error = error; 
  m_PID_prev_position_values.prev_dt_time = curr_time;

  float proj_pid = pTerm + iTerm + dTerm;
  float proj_power = proj_pid; 
  float ff = 0.0f; 
  float ff_clamp = 0.2; 
  switch(servo) {
    // track and shoulder do not need feed forward 
    case 0: 
      proj_power = std::clamp(proj_pid, -1*m_clamped_speed, m_clamped_speed);
      break; 
    // elbow needs feed forward and upward clamp 
    case 1: 
      ff = bldc_perseus::position_feedforward() * ff_clamp;
      proj_pid += ff; 
      proj_power = std::clamp(proj_pid, -1*m_clamped_speed, -0.1f*m_clamped_speed);
      break; 
    // wrist needs feed forward 
    case 2: 
      ff = bldc_perseus::position_feedforward() * ff_clamp;
      proj_pid += ff; 
      proj_power = std::clamp(proj_pid, -1*m_clamped_speed, m_clamped_speed);
      break; 
    default: 
      proj_power = std::clamp(proj_pid, -1*m_clamped_speed, m_clamped_speed);
      break; 
  }

  m_current.power = proj_power; 
  print_csv_format(pTerm, iTerm, dTerm, proj_power, ff);
  m_h_bridge->power(m_current.power);
}

// will need to be different amounts for elbow/wrist, plus weight 
float bldc_perseus::position_feedforward() 
{
  float length = 0.4826;  // elbow_bar=19in=48.26cm=0.4826m
                          // shoulder_bar=22.5in=57.15cm=0.5715m
                          // wrist_bar=12in=30.48cm=0.3048m + 14in=76.2cm=0.762m
  float angle_offset = -20; // elbow=-20
                          // shoulder=-20
                          // wrist=0
  float weight_beam = 1000 * 9.8; // elbow=1000g
                                  // shoulder=1600g
                                  // wrist=600g
  float weight_end = 600 * 9.8;  // add together other parts
  
  float y_force = std::sin(std::numbers::pi/180 * (m_current.position + angle_offset)) 
      * length * (weight_beam/2 + weight_end);
  // for elbow 
  y_force = y_force / (length * (weight_beam/2 + weight_end));
  // // for wrist (might change)
  // if (m_current.position >= 0) y_force = -1 * y_force / (length * (weight_beam/2 + weight_end));  // >= only for elbow, > for others 
  // else if (m_current.position < 0) y_force =  y_force / (length * (weight_beam/2 + weight_end)); 
  return y_force; 
}

void bldc_perseus::print_csv_format(float pTerm, float iTerm, float dTerm, float proj_power, float ff)
{
  auto console = resources::console();
  hal::print<256>(
    *console,
    "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n",
    pTerm,
    iTerm,
    dTerm,
    proj_power,
    m_current.power,
    m_current.position,
    m_target.position,
    m_current_position_settings.kp,
    m_current_position_settings.ki,
    m_current_position_settings.kd,
    ff);
} 

// home the motor 
void bldc_perseus::homing()
{
  auto homing = resources::homing_pin(); 
  volatile auto homing_level = homing->level(); 
  while(homing_level != 0) 
  {
    homing_level = homing->level();
  }
  m_h_bridge->power(0); 
  // set "homed value" to current encoder value 
  home_encoder_value = m_encoder->read().angle;
}

}// namespace sjsu::perseus
