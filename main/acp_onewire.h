#pragma once

#include <iostream>
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "vector"

namespace ACP_OneWire
{
  struct RomNumber
  {
    uint8_t romNumber[8] { }; // [0] = Device Family, [1] - [6] = 48 bit Rom Id, [7] = CRC.
  };

  class OneWire
  {
    public:
      constexpr static uint8_t M_CMD_SEARCH_NORMAL { 0xF0 };
      constexpr static uint8_t M_CMD_MATCH_ROM { 0x55 };
      constexpr static uint8_t M_CMD_SKIP_ROM { 0xCC };
      constexpr static uint8_t M_CMD_SEARCH_ALARM { 0xEC };
      constexpr static uint8_t M_CMD_READ_POWER_SUPPLY { 0xB4 };

      constexpr static uint16_t M_RESET_PULSE_DURATION { 500 };
      constexpr static uint16_t M_RESET_WAIT_DURATION { 200 };
      constexpr static uint16_t M_SLOT_START_DURATION { 2 };
      constexpr static size_t M_QUEUE_LENGTH { 1 };
      constexpr static uint16_t M_SLOT_BIT_DURATION { 60 };
      constexpr static uint16_t M_SLOT_BIT_SAMPLE_TIME { 15 };
      constexpr static uint16_t M_SLOT_RECOVERY_DURATION { 2 };
      constexpr static uint32_t M_RESOLUTION_HZ { 1'000'000 };
      constexpr static size_t M_RX_MAX_BYTES { 10 }; 
      constexpr static size_t M_RX_MEM_BLOCKS { 64 };
      constexpr static size_t M_TX_MEM_BLOCKS { 64 };
      constexpr static size_t M_TX_QUEUE_DEPTH { 4 };
      constexpr static uint16_t M_RESET_PRESENCE_WAIT_DURATION_MIN { 15 }; // Minimum duration for master to wait for device to show its presence.
      constexpr static uint16_t M_RESET_PRESENCE_DURATION_MIN { 60 }; // Minimum duration for master to recognise device as present. 
                                                                      
      constexpr static uint8_t M_CRC8_TABLE[] 
      {
        0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
        157, 195, 33, 127, 252, 162, 64, 30, 95,  1, 227, 189, 62, 96, 130, 220,
        35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93,  3, 128, 222, 60, 98,
        190, 224,  2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
        70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89,  7,
        219, 133, 103, 57, 186, 228,  6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
        101, 59, 217, 135,  4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
        248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91,  5, 231, 185,
        140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
        17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
        175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
        50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
        202, 148, 118, 40, 171, 245, 23, 73,  8, 86, 180, 234, 105, 55, 213, 139,
        87,  9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
        233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
        116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
      };

      constexpr static rmt_symbol_word_t M_RESET_PULSE_SYMBOL 
      {
        .duration0 = M_RESET_PULSE_DURATION,
        .level0 = 0,
        .duration1 = M_RESET_WAIT_DURATION,
        .level1 = 1
      };

      constexpr static rmt_symbol_word_t M_BIT0_SYMBOL
      {
        .duration0 = M_SLOT_START_DURATION + M_SLOT_BIT_DURATION,
        .level0 = 0,
        .duration1 = M_SLOT_RECOVERY_DURATION,
        .level1 = 1
      };

      constexpr static rmt_symbol_word_t M_BIT1_SYMBOL
      {
        .duration0 = M_SLOT_START_DURATION,
        .level0 = 0,
        .duration1 = M_SLOT_BIT_DURATION + M_SLOT_RECOVERY_DURATION,
        .level1 = 1
      };

      constexpr static rmt_symbol_word_t M_RELEASE_SYMBOL
      {
        .duration0 = 1,
        .level0 = 1,
        .duration1 = 0,
        .level1 = 1,
      };

      rmt_channel_handle_t m_rx_channel { nullptr };
      rmt_channel_handle_t m_tx_channel { nullptr };
      rmt_encoder_handle_t m_bytes_encoder_handle { nullptr };
      rmt_encoder_handle_t m_copy_encoder_handle { nullptr };

      rmt_rx_event_callbacks_t m_callback { };
      rmt_rx_done_event_data_t m_rx_event_data { };

      rmt_rx_channel_config_t m_rx_channel_config { };
      rmt_tx_channel_config_t m_tx_channel_config { };
      rmt_copy_encoder_config_t m_copy_encoder_config { };
      rmt_bytes_encoder_config_t m_bytes_encoder_config { };

      rmt_transmit_config_t m_tx_config { };
      rmt_receive_config_t m_rx_config { };

      rmt_symbol_word_t m_rx_buffer[8 * M_RX_MAX_BYTES] { }; // 1 symbol = 1 bit.
                                                             
      uint8_t m_device_family;
      std::vector<RomNumber> m_devices_found { }; 
      size_t m_total_devices { };
      uint8_t m_last_discrepancy { 0 };
      bool m_last_device { false };

      gpio_num_t m_gpio;

      SemaphoreHandle_t m_mutex { };
      QueueHandle_t m_receive_queue;

    public:
      OneWire(gpio_num_t gpioWire, uint8_t deviceFamily);

      esp_err_t Reset(void);
      esp_err_t TransmitBit(const uint8_t txBit);
      esp_err_t TransmitSymbol(const rmt_encoder_handle_t& encoder, const rmt_symbol_word_t& symbol);
      esp_err_t TransmitBytes(const uint8_t *txData, uint8_t txDataSize);
      esp_err_t ReadBit(uint8_t& rxBit);
      esp_err_t ReadBytes(uint8_t *rxBuffer, size_t RxBufferSize);
      std::vector<RomNumber> ScanDevices(void);
      size_t GetTotalDevices(void);
      void PrintRomNumbers(void);
      uint8_t GetCrc8Fast(uint8_t initCrc, uint8_t *input, size_t inputSize);
      uint8_t GetCrc8(uint8_t initCrc, uint8_t *input, size_t inputSize);

    private:

      bool isDevicePresent(rmt_symbol_word_t *symbols, size_t totalSymbols);
      esp_err_t setupBytesEncoder(void);
      esp_err_t setupCopyEncoder(void);
      void setupTxConfig(void);
      void setupRxConfig(void);
      esp_err_t createRxChannel(void); //Must come before Tx!
      esp_err_t createTxChannel(void); 
      esp_err_t enableRxChannel(void); 
      esp_err_t enableTxChannel(void); 
      esp_err_t setupCallback(void); 
      void decodeRmtData(rmt_symbol_word_t* rmtSymbols, size_t numSymbols, uint8_t* rxBuffer, size_t rxBufferSize);
      esp_err_t getNextDevice(RomNumber& currentRomNumber);
    };
  }
