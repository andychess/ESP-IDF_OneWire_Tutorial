#include <iostream>
#include "acp_ds18b20.h" 

std::vector<ACP_OneWire::RomNumber> deviceIds { };
std::vector<ACP_DS18B20::DS18B20> sensors { };
ACP_OneWire::OneWire onewire(GPIO_NUM_23, 0x28); // Ox28 is the family name for the ds18b20.

extern "C" void app_main(void)
{
  // Search for ds18b20s.
  deviceIds = onewire.ScanDevices();

  // Print rom numbers of found devices.
  onewire.PrintRomNumbers();

  // Create and store a sensor object for each device found.
  for (auto id : deviceIds)
  {
    ACP_DS18B20::DS18B20 sensor(id, onewire); 
    sensors.push_back(sensor); 
  }

  //Loop through found devices, displaying temperatures. 
  for (auto sensor : sensors)
  {
    sensor.SetResolution(ACP_DS18B20::resolution_t::RESOLUTION_9_BIT);

    std::cout << sensor.GetTemperature() << std::endl;
  }

}
