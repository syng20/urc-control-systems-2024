#include <libhal-picosdk/i2c.hpp>
#include <libhal-picosdk/serial.hpp>
#include <libhal-picosdk/time.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <numbers>

#include <hardware/irq.h>
#include <pico/multicore.h>

#include "can.h"
#include "hbridge.hpp"

int volatile dir = 1;

// AS5600L angle reading, not very much to comment on here
static float get_angle(hal::i2c& i2c)
{
  uint8_t const addr = 0x40;
  std::array<uint8_t const, 1> reg{ 0x0e };
  std::array<uint8_t, 2> data = { 0xff, 0xff };
  i2c.transaction(addr, reg, data);
  uint16_t angle = (data[0] << 8) | data[1];
  return (float(angle) / float(0xfff) * 2.0f * std::numbers::pi_v<float>);
}

// USB stack runs on core 1, so core 2 is picked to reduce jitter
void core2()
{
  namespace rp = hal::rp;
  triple_hbridge h;
  // i2c needs to be run at very high speed or this loop gets bogged down by i2c
  auto i2c = rp::i2c(
    hal::pin<16>, hal::pin<17>, hal::bus<0>, { .clock_rate = 1'000'000 });

  // we align the motor by running a phase for 500ms before reading the zero angle
  h.set_duty(0.85, -0.85, -0.85);
  hal::rp::sleep(std::chrono::duration<hal::u32, std::milli>(500));
  h.set_duty(0.0, 0.0, 0.0);
  float const zero_angle = get_angle(i2c);

  int i = 0;
  int direction = dir;
  for (;;) {
    // we only check the dir variable once in a while to avoid too much memory contention,
    // although it might be entirely unnecessary
    if (i >= 1'000) {
      if (dir == 0) {
        // this stops the h-bridge from just being kept on constantly,
        // which causes excessive current draw
        h.set_duty(0.0, 0.0, 0.0);
        while (dir == 0)
          ;
      }
      direction = dir;
      i = 0;
    }
    ++i;
    // See Umeda et al.
    float mechanical_angle = (get_angle(i2c) - zero_angle);
    float electrical_angle = mechanical_angle * 14.0f;
    float quad_offset = std::numbers::pi_v<float> / 2.f * float(direction);
    float quadrature = electrical_angle + quad_offset;
    float const offset = std::numbers::pi_v<float> * 2.0f / 3.0f;
    float max_duty = 0.7f;
    float a = max_duty * cosf(quadrature),
          b = max_duty * cosf(quadrature - offset),
          c = max_duty * cosf(quadrature + offset);
    h.set_duty(a, b, c);
  }
}

static void can2040_cb(struct can2040*,
                       uint32_t notify,
                       struct can2040_msg* msg)
{
  if (notify != CAN2040_NOTIFY_RX) {
    return;
  }

  if (msg->id != 0x15) {
    return;
  }
  if (msg->dlc < 1) {
    return;
  }
  switch (msg->data[0]) {
    case 0:
      dir = 0;
      break;
    case 1:
      dir = 1;
      break;
    case 2:
      dir = -1;
      break;
    default:
      dir = 0;
  }
}

can2040 canbus = {};
static void pio0_irq()
{
  can2040_pio_irq_handler(&canbus);
}

static void canbus_setup()
{
  uint32_t pio_num = 0;
  can2040_setup(&canbus, pio_num);
  can2040_callback_config(&canbus, &can2040_cb);

  // Enable irqs
  irq_set_exclusive_handler(PIO0_IRQ_0, &pio0_irq);
  irq_set_priority(PIO0_IRQ_0, 1);
  irq_set_enabled(PIO0_IRQ_0, true);

  uint32_t bitrate = 1'000'000;
  int rx = 25, tx = 24;
  can2040_start(&canbus, SYS_CLK_HZ, bitrate, rx, tx);
}

int main()
{
  using namespace std::chrono_literals;
  namespace rp = hal::rp;
  auto out = rp::stdio_serial();
  auto clk = rp::clock();
  multicore_launch_core1(&core2);

  canbus_setup();

  for (;;) {
    hal::print<64>(out, "Hello world! Direction: %d\n", dir);
    hal::delay(clk, 1s);
  }
}
