#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <libhal/input_pin.hpp>

#include <switches.hpp>
#include <resource_list.hpp>


using sec = float;

namespace sjsu::perseus {

switches_bldc(hal::v5::strong_ptr<hal::input_pin> p_s1, 
                    hal::v5::strong_ptr<hal::input_pin> p_s2, 
                    hal::v5::strong_ptr<hal::input_pin> p_s3, 
                    hal::v5::strong_ptr<hal::input_pin> p_s4, 
                    hal::v5::strong_ptr<hal::input_pin> p_s5, 
                    hal::v5::strong_ptr<hal::input_pin> p_s6, 
                    hal::v5::strong_ptr<hal::input_pin> p_s7
                )
    {
        m_s1 = p_s1->level(); 
        m_s2 = p_s2->level(); 
        m_s3 = p_s3->level(); 
        m_s4 = p_s4->level(); 
        m_s5 = p_s5->level(); 
        m_s6 = p_s6->level(); 
        m_s7 = p_s7->level(); 
        switch_value = switch_value || m_s1
    }

// switch value 
hal::u8 read_switch_value() {
    return m_switch_value;
} 

} // namespace sjsu::perseus