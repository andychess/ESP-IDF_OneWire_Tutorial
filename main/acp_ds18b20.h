#include <iostream>
#include "driver/gpio.h"

#include "acp_onewire.h"

namespace ACP_DS18B20
{
  enum class resolution_t
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


    public:
      DS18B20();


    private:
  
  };
}

// static esp_err_t ds18b20_send_command(ds18b20_device_handle_t ds18b20, uint8_t cmd)
// {
//     // send command
//     uint8_t tx_buffer[10] = {0};
//     tx_buffer[0] = ONEWIRE_CMD_MATCH_ROM;
//     memcpy(&tx_buffer[1], &ds18b20->addr, sizeof(ds18b20->addr));
//     tx_buffer[sizeof(ds18b20->addr) + 1] = cmd;
//
//     return onewire_bus_write_bytes(ds18b20->bus, tx_buffer, sizeof(tx_buffer));
// }
//
// esp_err_t ds18b20_set_resolution(ds18b20_device_handle_t ds18b20, ds18b20_resolution_t resolution)
// {
//     ESP_RETURN_ON_FALSE(ds18b20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
//     // reset bus and check if the ds18b20 is present
//     ESP_RETURN_ON_ERROR(onewire_bus_reset(ds18b20->bus), TAG, "reset bus error");
//
//     // send command: DS18B20_CMD_WRITE_SCRATCHPAD
//     ESP_RETURN_ON_ERROR(ds18b20_send_command(ds18b20, DS18B20_CMD_WRITE_SCRATCHPAD), TAG, "send DS18B20_CMD_WRITE_SCRATCHPAD failed");
//
//     // write new resolution to scratchpad
//     const uint8_t resolution_data[] = {0x1F, 0x3F, 0x5F, 0x7F};
//     uint8_t tx_buffer[3] = {0};
//     tx_buffer[0] = ds18b20->th_user1;
//     tx_buffer[1] = ds18b20->tl_user2;
//     tx_buffer[2] = resolution_data[resolution];
//     ESP_RETURN_ON_ERROR(onewire_bus_write_bytes(ds18b20->bus, tx_buffer, sizeof(tx_buffer)), TAG, "send new resolution failed");
//
//     ds18b20->resolution = resolution;
//     return ESP_OK;
// }
//
// esp_err_t ds18b20_trigger_temperature_conversion(ds18b20_device_handle_t ds18b20)
// {
//     ESP_RETURN_ON_FALSE(ds18b20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
//     // reset bus and check if the ds18b20 is present
//     ESP_RETURN_ON_ERROR(onewire_bus_reset(ds18b20->bus), TAG, "reset bus error");
//
//     // send command: DS18B20_CMD_CONVERT_TEMP
//     ESP_RETURN_ON_ERROR(ds18b20_send_command(ds18b20, DS18B20_CMD_CONVERT_TEMP), TAG, "send DS18B20_CMD_CONVERT_TEMP failed");
//
//     // delay proper time for temperature conversion
//     const uint32_t delays_ms[] = {100, 200, 400, 800};
//     vTaskDelay(pdMS_TO_TICKS(delays_ms[ds18b20->resolution]));
//
//     return ESP_OK;
// }
//
// esp_err_t ds18b20_get_temperature(ds18b20_device_handle_t ds18b20, float *ret_temperature)
// {
//     ESP_RETURN_ON_FALSE(ds18b20 && ret_temperature, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
//     // reset bus and check if the ds18b20 is present
//     ESP_RETURN_ON_ERROR(onewire_bus_reset(ds18b20->bus), TAG, "reset bus error");
//
//     // send command: DS18B20_CMD_READ_SCRATCHPAD
//     ESP_RETURN_ON_ERROR(ds18b20_send_command(ds18b20, DS18B20_CMD_READ_SCRATCHPAD), TAG, "send DS18B20_CMD_READ_SCRATCHPAD failed");
//
//     // read scratchpad data
//     ds18b20_scratchpad_t scratchpad;
//     ESP_RETURN_ON_ERROR(onewire_bus_read_bytes(ds18b20->bus, (uint8_t *)&scratchpad, sizeof(scratchpad)),
//                         TAG, "error while reading scratchpad data");
//     // check crc
//     ESP_RETURN_ON_FALSE(onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc_value, ESP_ERR_INVALID_CRC, TAG, "scratchpad crc error");
//
//     const uint8_t lsb_mask[4] = {0x07, 0x03, 0x01, 0x00}; // mask bits not used in low resolution
//     uint8_t lsb_masked = scratchpad.temp_lsb & (~lsb_mask[scratchpad.configuration >> 5]);
//     *ret_temperature = (((int16_t)scratchpad.temp_msb << 8) | lsb_masked)  / 16.0f;
//
//     return ESP_OK;
// }
