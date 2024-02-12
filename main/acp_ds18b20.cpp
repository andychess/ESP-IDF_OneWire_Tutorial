#include "acp_ds18b20.h"
#include "esp_err.h"
#include "esp_log.h"

namespace ACP_DS18B20
{
  static const char* TAG { "DS18B20_Bus" };

  DS18B20::DS18B20(ACP_OneWire::RomNumber deviceId, ACP_OneWire::OneWire& onewire) : 
    m_onewire_bus { onewire }
  {
    m_device_id = deviceId;

    ESP_LOGI(TAG, "Constructed sensor: %02X%02X%02X%02X%02X%02X", 
        m_device_id.romNumber[6], 
        m_device_id.romNumber[5], 
        m_device_id.romNumber[4], 
        m_device_id.romNumber[3], 
        m_device_id.romNumber[2], 
        m_device_id.romNumber[1]);
  }

  esp_err_t DS18B20::sendCommand(uint8_t cmd)
  {
    esp_err_t result { ESP_OK };
    uint8_t txBuffer[10] { 0 };
    size_t length { sizeof(txBuffer) }; 
    
    txBuffer[0] = ACP_OneWire::OneWire::M_CMD_MATCH_ROM;

    for (size_t i { 0 }; i < sizeof(m_device_id); i++)
    {
      txBuffer[i + 1] = m_device_id.romNumber[i];
    }

    txBuffer[9] = cmd;

    result |= m_onewire_bus.TransmitBytes(txBuffer, length);

    return result;
  }

  esp_err_t DS18B20::SetResolution(resolution_t res)
  {
    esp_err_t result { ESP_OK };
    uint8_t txBuffer[3] { 0 };
    int resolution { static_cast<int>(res) };  // Note the need to cast a strongly typed enum!
    size_t length { sizeof(txBuffer) }; 

    // Reset bus.
    result |= m_onewire_bus.Reset(); 

    // Send command: M_CMD_WRITE_SCRATCHPAD.
    result |= sendCommand(M_CMD_WRITE_SCRATCHPAD);

    // Write new resolution to scratchpad.
    txBuffer[0] = m_scratchpad[M_INDEX_TH_USER_ONE];
    txBuffer[1] = m_scratchpad[M_INDEX_TL_USER_TWO];;
    txBuffer[2] = M_RESOLUTION[resolution];

    // Send bytes.
    result |= m_onewire_bus.TransmitBytes(txBuffer, length);

    if (ESP_OK == result) m_resolution = res;

    return result;
  }

  float DS18B20::GetTemperature(void)
  {
    esp_err_t result { ESP_OK };
    float temperature;
    int resolution { static_cast<int>(m_resolution) };
    uint32_t delay { M_CONVERSION_DELAY_MS[resolution] };
    uint8_t lsbMasked;
    
    // Reset bus.
    result |= m_onewire_bus.Reset(); 

    // Send command: DS18B20_CMD_CONVERT_TEMP.
    result |= sendCommand(M_CMD_CONVERT_TEMP);

    // Delay for temperature conversion.
    vTaskDelay(pdMS_TO_TICKS(delay));

    // Reset bus.
    result |= m_onewire_bus.Reset(); 
    
    // Send command: DS18B20_CMD_READ_SCRATCHPAD.
    result |= sendCommand(M_CMD_READ_SCRATCHPAD);

    // Read scratchpad data
    result |= m_onewire_bus.ReadBytes(m_scratchpad, sizeof(m_scratchpad));

    if (ESP_OK != result) return M_TEMP_READ_ERROR; 
                  
    // Check crc.
    if (m_onewire_bus.GetCrc8(0, m_scratchpad, 8) != m_scratchpad[M_INDEX_CRC_VALUE])
    {
      ESP_LOGW(TAG, "Scratchpad CRC Error!");
      return M_TEMP_READ_ERROR;
    }

    lsbMasked = m_scratchpad[M_INDEX_TEMP_LSB] & (~M_LSB_MASK[m_scratchpad[M_INDEX_CONFIGURATION] >> 5]);

    temperature = (((int16_t)m_scratchpad[M_INDEX_TEMP_MSB] << 8) | lsbMasked)  / 16.0f;

    return temperature;
  }
}
