#include "acp_ds18b20.h"

namespace ACP_DS18B20
{
  DS18B20::DS18B20(gpio_num_t gpio) :
    m_onewire(gpio, M_ONEWIRE_FAMILY)
  {
    m_gpio = gpio;
  }
}
