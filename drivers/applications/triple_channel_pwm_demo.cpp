// derived from libhal-arm-mcu/demos/applications/pwm.cpp

#include <cmath>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <h_bridge.hpp>
#include <numbers>
#include <resource_list.hpp>
#include <serial_commands.hpp>

using namespace std::chrono_literals;
namespace sjsu::drivers {

void application()
{
    using namespace std::chrono_literals;
    using namespace hal::literals;

    auto console = resources::console();
    auto clock = resources::clock();
    hal::print(*console, "Basics created...\n");

    auto high_pin_a = resources::pwm_channel_0_h();
    auto low_pin_a = resources::pwm_channel_0_l();
    auto high_pin_b = resources::pwm_channel_1_h();
    auto low_pin_b = resources::pwm_channel_1_l();
    auto high_pin_c = resources::pwm_channel_2_h();
    auto low_pin_c = resources::pwm_channel_2_l();
    hal::print(*console, "PWM channels created...\n");

    hal::print(*console, "Triple phase GO...\n");

    volatile float curr_time = 0; 

    // reset
    high_pin_a->duty_cycle(0.0f);
    low_pin_a->duty_cycle(0.0f);
    high_pin_b->duty_cycle(0.0f);
    low_pin_b->duty_cycle(0.0f);
    high_pin_c->duty_cycle(0.0f);
    low_pin_c->duty_cycle(0.0f);

    float constexpr two_pi = 2 * std::numbers::pi; 
    float const step = 1.0f / two_pi;
    float cy_a = 0; 
    float cy_b = 0; 
    float cy_c = 0; 
    float r = clock->frequency(); 

    while (true) {

        curr_time = static_cast<float>(clock->uptime()); 
        cy_a = sinf(curr_time * r); 
        cy_b = sinf(curr_time * r + two_pi / 3); 
        cy_a = sinf(curr_time * r - two_pi / 3); 


        for (float cycle_completion = 0; cycle_completion < two_pi; cycle_completion += step) {
            hal::print<64>(*console, ">> Cycle completion: %.2f \n", cycle_completion);

            
            // sinf 
            cy_a = sinf(curr_time * r); 
            cy_b = sinf(curr_time * r + two_pi / 3); 
            cy_a = sinf(curr_time * r - two_pi / 3); 

            // a pos or neg
            if (cy_a > 0) high_pin_a->duty_cycle(0x7FFE * cy_a); 
            else if (cy_a < 0) { 
                cy_a = cy_a * -1; 
                low_pin_a->duty_cycle(0x7FFE * cy_a);
            }
            else {
                high_pin_a->duty_cycle(0x7FFE * cy_a);
                low_pin_a->duty_cycle(0x7FFE * cy_a);
            }
            // b pos or neg 
            if (cy_b > 0) high_pin_b->duty_cycle(0x7FFE * cy_b); 
            else if (cy_b < 0) { 
                cy_b = cy_b * -1; 
                low_pin_b->duty_cycle(0x7FFE * cy_b);
            }
            else {
                high_pin_b->duty_cycle(0x7FFE * cy_b);
                low_pin_b->duty_cycle(0x7FFE * cy_b);
            }
            // c pos or neg 
            if (cy_c > 0) high_pin_c->duty_cycle(0x7FFE * cy_c); 
            else if (cy_c < 0) { 
                cy_c = cy_c * -1; 
                low_pin_c->duty_cycle(0x7FFE * cy_c);
            }
            else {
                high_pin_c->duty_cycle(0x7FFE * cy_c);
                low_pin_c->duty_cycle(0x7FFE * cy_c);
            }

            // high_pin_a->duty_cycle(0x7FFE * sinf(cycle_completion));
            // high_pin_b->duty_cycle(0x7FFE * sinf(cycle_completion + two_pi / 3)); 
            // high_pin_c->duty_cycle(0x7FFE * sinf(cycle_completion - two_pi / 3));

            hal::delay(*clock, 10ms);
        }

        hal::print(*console, "From the top \n");
    }

}
}  // namespace sjsu::drivers
