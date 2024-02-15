#include <iostream>
#include "driver/gpio.h"

#include "acp_onewire.h"

namespace ACP_DS18B20
{
  enum class Resolution
  {
    RESOLUTION_9_BIT,
    RESOLUTION_10_BIT,
    RESOLUTION_11_BIT,
    RESOLUTION_12_BIT,
  };

  class DS18B20
  {
    private:
      constexpr static uint8_t M_CMD_CONVERT_TEMP { 0x44 };
      constexpr static uint8_t M_CMD_WRITE_SCRATCHPAD { 0x4E };
      constexpr static uint8_t M_CMD_READ_SCRATCHPAD { 0xBE };
      constexpr static uint8_t M_ONEWIRE_FAMILY { 0x28 }; 
      constexpr static uint8_t M_RESOLUTION[4] { 0x1F, 0x3F, 0x5F, 0x7F };
      constexpr static uint32_t M_CONVERSION_DELAY_MS[4] { 100, 200, 400, 800 };
      constexpr static uint8_t M_LSB_MASK[4] = { 0x07, 0x03, 0x01, 0x00 };
      constexpr static float M_TEMP_READ_ERROR { 9999 }; 

      constexpr static size_t M_INDEX_TEMP_LSB { 0 }; 
      constexpr static size_t M_INDEX_TEMP_MSB { 1 };
      constexpr static size_t M_INDEX_TH_USER_ONE { 2 };
      constexpr static size_t M_INDEX_TL_USER_TWO { 3 };
      constexpr static size_t M_INDEX_CONFIGURATION { 4 };
      constexpr static size_t M_INDEX_RESERVED_ONE { 5 };
      constexpr static size_t M_INDEX_RESERVED_TWO { 6 };
      constexpr static size_t M_INDEX_RESERVED_THREE { 7 };
      constexpr static size_t M_INDEX_CRC_VALUE { 8 };     

      uint8_t m_scratchpad[9] { };

      Resolution m_resolution { Resolution::RESOLUTION_12_BIT };

      ACP_OneWire::OneWire& m_onewire_bus;

      ACP_OneWire::RomNumber m_device_id;

    public:
      DS18B20(ACP_OneWire::RomNumber deviceId, ACP_OneWire::OneWire& onewire);

      esp_err_t SetResolution(Resolution resolution);
      float GetTemperature(void);

    private:
      esp_err_t sendCommand(uint8_t cmd);
  
  };
}

