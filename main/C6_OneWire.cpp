#include <iostream>
#include "acp_ds18b20.h" 

extern "C" void app_main(void)
{
  ACP_DS18B20::DS18B20 ds81b20(GPIO_NUM_23);
}
