#include <iostream>
#include "driver/gpio.h"

#include "acp_onewire.h"

namespace ACP_DS18B20
{
  class DS18B20
  {
    private:
      constexpr static uint8_t M_ONEWIRE_FAMILY { 0x28 }; 
      gpio_num_t m_gpio;
      ACP_OneWire::OneWire m_onewire;


    public:
      DS18B20(gpio_num_t gpio);

    private:
  
  };
}
