#include <cmath>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include <libhal/units.hpp>
#include <optional>
#include <resource_list.hpp>
#include <bldc_servo.hpp>
#include <can_messaging.hpp>

using namespace std::chrono_literals;
namespace sjsu::perseus {

can_perseus::can_perseus(
    hal::u16 p_curr_servo_addr,
    hal::u32 p_baudrate,
    hal::v5::strong_ptr<hal::can_transceiver> p_can_transceiver,
    hal::v5::strong_ptr<hal::can_bus_manager> p_can_bus_manager,
    hal::v5::strong_ptr<hal::can_interrupt> p_can_interrupt,
    hal::v5::strong_ptr<hal::can_identifier_filter> p_can_identifier_filter
  ) 
  : 
    m_self_servo_addr(p_curr_servo_addr), 
    m_baudrate(p_baudrate),
    m_can_transceiver(p_can_transceiver),
    m_can_bus_manager(p_can_bus_manager),
    m_can_interrupt(p_can_interrupt),
    m_can_identifier_filter(p_can_identifier_filter), 
    m_can_message_finder(*p_can_transceiver, p_curr_servo_addr)
{
  auto console = resources::console();
  m_can_identifier_filter->allow(m_self_servo_addr);
  m_can_bus_manager->baud_rate(m_baudrate); 
  m_can_interrupt->on_receive([&console](hal::can_interrupt::on_receive_tag,
                                       hal::can_message const& p_message) {
    hal::print<64>(
      *console, "Can message with id = 0x%lX from interrupt!\n", p_message.id);
  });  
  hal::print<32>(*console,
                 "Receiver buffer size = %zu\n",
                 m_can_transceiver->receive_buffer().size());
  hal::print<64>(
    *console, "🆔 Allowing ID [0x%lX] through the filter!\n", m_self_servo_addr);
  
};

// decode message from mission control 
float can_perseus::fixed_to_floating_point(hal::byte msb, hal::byte lsb, int exponent) {
  auto console = resources::console();
  hal::i16 initial = (msb << 8) | lsb;
  hal::print<64>(*console, "pure_binary: %x -- ", initial); 
  float shifted = initial / powf(2, exponent); 
  hal::print<64>(*console, "float cast: %x\n", shifted); 
  return shifted; 

  // return static_cast<float>(static_cast<hal::i16>((msb << 8) | lsb)) / pow(2, exponent); 
}

// endcode message to mission control 
hal::i16 can_perseus::floating_to_fixed_point(float n, int exponent) {
  return (static_cast<hal::i16>(n) << exponent); 
}



void can_perseus::print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<256>(p_console,
                  "Received Message from ID: 0x%lX, length: %u \n"
                  "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                  "0x%02X, 0x%02X, 0x%02X ]\n",
                  p_message.id,
                  p_message.length,
                  p_message.payload[0],
                  p_message.payload[1],
                  p_message.payload[2],
                  p_message.payload[3],
                  p_message.payload[4],
                  p_message.payload[5],
                  p_message.payload[6],
                  p_message.payload[7]);
}

void can_perseus::process_can_message(hal::can_message const& p_message,
                        hal::v5::strong_ptr<bldc_perseus> bldc)
{   
  auto console = resources::console();
  hal::can_message response{
    .id = 0x000,
    .extended=false,
    .remote_request=false,
    .length = 0,
    .payload = {},
  };
  switch (static_cast<action>(p_message.payload[0])) {
    case action::freeze:{
      bldc->freeze(); 
      bldc->set_reading_action(static_cast<uint32_t>(action::freeze)); 
      break;
    }
    case action::heartbeat: {
      response.id = m_self_servo_addr + 0x100;
      response.length = 1;
      response.payload[0] = m_self_servo_addr + 0x50;
      bldc->set_reading_action(static_cast<uint32_t>(action::heartbeat)); 
      break; 
    }
    case action::homing: {
      response.id = m_self_servo_addr + 0x100;
      response.length = 1;
      response.payload[0] = 0x01;
      response.payload[1] = 0x11;
      bldc->set_reading_action(static_cast<uint32_t>(action::homing)); 
      break; 
    }
    case action::set_position: {
      float target_position = fixed_to_floating_point(p_message.payload[1], p_message.payload[2], 6);
      bldc->set_target_position(target_position);
      hal::print<64>(*console, "Target = %f\n", target_position);
      response.id = m_self_servo_addr + 0x100;
      response.length = 1;
      response.payload[0] = static_cast<hal::byte>(action::set_position) + 0x50;;
      bldc->set_reading_action(static_cast<uint32_t>(action::set_position)); 
      break;
    }
    case action::read_position: {
      float reading_position = bldc->get_reading_position();
      hal::u16 t = can_perseus::floating_to_fixed_point(reading_position, 6); 
      response.id = m_self_servo_addr + 0x100;
      response.length = 8;
      response.payload[0] = 0x20 + 0x50; 
      response.payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
      response.payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
      bldc->set_reading_action(static_cast<uint32_t>(action::read_position)); 
      break;
    }
    case action::read_velocity: {
      float reading_velocity = bldc->get_reading_velocity();
      hal::u16 t = can_perseus::floating_to_fixed_point(reading_velocity, 6); 
      response.id = m_self_servo_addr + 0x100;
      response.length = 8;
      response.payload[0] = 0x20 + 0x50; 
      response.payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
      response.payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
      bldc->set_reading_action(static_cast<uint32_t>(action::read_velocity)); 
      break;
    }
    case action::set_pid_position: {
      bldc_perseus::PID_settings settings = {
        .kp = fixed_to_floating_point(p_message.payload[1], p_message.payload[2], 14),
        .ki = fixed_to_floating_point(p_message.payload[3], p_message.payload[4], 14),
        .kd = fixed_to_floating_point(p_message.payload[5], p_message.payload[6], 14)
      };
      bldc->update_pid_position(settings);
      response.id = m_self_servo_addr + 0x100;
      response.length = 1;
      response.payload[0] = static_cast<hal::byte>(action::set_pid_position) + 0x50;
      bldc->set_reading_action(static_cast<uint32_t>(action::set_pid_position)); 
      break;
    }
    case action::set_pid_velocity: {
      bldc_perseus::PID_settings settings = {
        .kp = fixed_to_floating_point(p_message.payload[1], p_message.payload[2], 14),
        .ki = fixed_to_floating_point(p_message.payload[3], p_message.payload[4], 14),
        .kd = fixed_to_floating_point(p_message.payload[5], p_message.payload[6], 14)
      };
      bldc->update_pid_velocity(settings);
      response.id = m_self_servo_addr + 0x100;
      response.length = 1;
      response.payload[0] = static_cast<hal::byte>(action::set_pid_velocity) + 0x50;
      bldc->set_reading_action(static_cast<uint32_t>(action::set_pid_velocity)); 
      break;
    }
    default:
      hal::operation_not_supported(nullptr);
  }
  m_can_message_finder.transceiver().send(response);
  print_can_message(*console, response);
  hal::print<64>(*console, "finished transmission\n");
}

void can_perseus::repeating_action_can(uint32_t curr_action, float sending_position) {
  hal::can_message response{
    .id = 0x000,
    .extended=false,
    .remote_request=false,
    .length = 0,
    .payload = {},
  };
  hal::u16 t;
  switch (static_cast<can_perseus::action>(curr_action)) {
    case can_perseus::action::homing: {
      t = can_perseus::floating_to_fixed_point(sending_position, 6); 
      response.length = 8;
      response.payload[0] = 0x20 + 0x50; 
      response.payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
      response.payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
      break; 
    }
    case can_perseus::action::set_position: {
      t = can_perseus::floating_to_fixed_point(sending_position, 6); 
      response.length = 8;
      response.payload[0] = 0x20 + 0x50; 
      response.payload[1] = static_cast<hal::byte>(t >> 8) & 0xFF; // HIGH BYTE FIRST 
      response.payload[2] = static_cast<hal::byte>(t >> 0) & 0xFF;  // LOW BYTE SECOND
      break;
    }
    default:
      break; 
  }
}

std::optional<hal::can_message> can_perseus::check_for_message() {
  auto msg = m_can_message_finder.find();
  if (msg) {
    return msg; 
  }
  return std::nullopt; 
}


} // namespace sjsu::perseus