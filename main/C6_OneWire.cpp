#include <iostream>
#include "acp_ds18b20.h" 


std::vector<ACP_OneWire::RomNumber> deviceIds { };

extern "C" void app_main(void)
{
  ACP_OneWire::OneWire onewire(GPIO_NUM_23, 0x28);

  deviceIds = onewire.ScanDevices();

  onewire.PrintRomNumbers();
}
