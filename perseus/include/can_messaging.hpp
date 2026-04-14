#pragma once
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/units.hpp>

#include <bldc_servo.hpp>

namespace sjsu::perseus {
    
class can_perseus 
{

public: 
    can_perseus(
        hal::u16 p_servo_addr,
        hal::u32 p_baudrate,
        hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
        hal::v5::strong_ptr<hal::can_bus_manager> p_can_bus_manager,
        hal::v5::strong_ptr<hal::can_interrupt> p_can_interrupt,
        hal::v5::strong_ptr<hal::can_identifier_filter> p_can_identifier_filter
    ); 


    enum class action : uint32_t
    {
    // top priority
    freeze = 0x0C,  // hard stop the servo to be 0
    heartbeat = 0x0E, // are you alive
    // actuators
    homing = 0x111, 
    set_position = 0x12,
    // readers
    read_position = 0x20,
    read_velocity = 0x21,
    // setters
    set_pid_position = 0x31,
    set_pid_velocity = 0x32,
    // servo to servo 
    prev_joint_actual_position = 0x40, 
    };
    enum servo_address : hal::u16
    {
    track_servo = 0x120,
    shoulder_servo = 0x121,
    elbow_servo = 0x122,
    wrist_pitch = 0x123,
    wrist_roll = 0x124,
    clamp = 0x125,

    from_shoulder = 0x321, 
    from_elbow = 0x322

    };

    void set_curr_servo_addr(hal::u16 servo_addr); 

    void print_can_message(hal::serial& p_console,
                        hal::can_message const& p_message); 
    float fixed_to_floating_point(hal::byte msb, hal::byte lsb, int exponent); 
    hal::i16 floating_to_fixed_point(float n, int exponent); 
    void process_can_message(hal::can_message const& p_message,
                        hal::v5::strong_ptr<bldc_perseus> bldc);
    void repeating_action_can(uint32_t action, 
                        float sending_position, 
                        hal::v5::strong_ptr<bldc_perseus> bldc); 
    std::optional<hal::can_message> check_for_mc_message(); 
    std::optional<hal::can_message> check_for_joint_message(); 

private: 
    hal::u16 m_self_servo_addr;
    hal::u32 m_baudrate;
    hal::v5::strong_ptr<hal::can_transceiver> m_can_transceiver;
    hal::v5::strong_ptr<hal::can_bus_manager> m_can_bus_manager;
    hal::v5::strong_ptr<hal::can_interrupt> m_can_interrupt;
    hal::v5::strong_ptr<hal::can_identifier_filter> m_can_identifier_filter;
    hal::can_message_finder m_mc_message_finder;
    hal::can_message_finder m_joint_message_finder;
    // hal::can_message response;
    // hal::can_message forward_to_next;
}; 
} // namespace sjsu::perseus