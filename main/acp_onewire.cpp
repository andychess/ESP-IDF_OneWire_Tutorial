#include "acp_onewire.h"
#include "esp_err.h"
#include "soc/clk_tree_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

namespace ACP_OneWire
{
  /*
  Reset Pulse:

            | RESET_PULSE | RESET_WAIT_DURATION |
            | _DURATION   |                     |
            |             |   | | RESET     |   |
            |             | * | | _PRESENCE |   |
            |             |   | | _DURATION |   |
  ----------+             +-----+           +--------------
            |             |     |           |
            |             |     |           |
            |             |     |           |
            +-------------+     +-----------+
  *: RESET_PRESENCE_WAIT_DURATION

  Write 1 bit:

            | SLOT_START | SLOT_BIT  | SLOT_RECOVERY | NEXT
            | _DURATION  | _DURATION | _DURATION     | SLOT
            |            |           |               |
  ----------+            +-------------------------------------
            |            |
            |            |
            |            |
            +------------+

  Write 0 bit:

            | SLOT_START | SLOT_BIT  | SLOT_RECOVERY | NEXT
            | _DURATION  | _DURATION | _DURATION     | SLOT
            |            |           |               |
  ----------+                        +-------------------------
            |                        |
            |                        |
            |                        |
            +------------------------+

  Read 1 bit:


            | SLOT_START | SLOT_BIT_DURATION | SLOT_RECOVERY | NEXT
            | _DURATION  |                   | _DURATION     | SLOT
            |            | SLOT_BIT_   |     |               |
            |            | SAMPLE_TIME |     |               |
  ----------+            +----------------------------------------------
            |            |
            |            |
            |            |
            +------------+

  Read 0 bit:

            | SLOT_START | SLOT_BIT_DURATION | SLOT_RECOVERY | NEXT
            | _DURATION  |                   | _DURATION     | SLOT
            |            | SLOT_BIT_   |     |               |
            |            | SAMPLE_TIME |     |               |
  ----------+            |             |  +-----------------------------
            |            |                |
            |            |   PULLED DOWN  |
            |            |    BY DEVICE   |
            +-----------------------------+
    
  [0].0 means symbol[0].duration0

  First reset pulse after rmt channel init:

  Bus is low | Reset | Wait |  Device  |  Bus Idle
  after init | Pulse |      | Presence |
                     +------+          +-----------
                     |      |          |
                     |      |          |
                     |      |          |
  -------------------+      +----------+
                     1      2          3

            [0].1     [0].0     [1].1     [1].0


  Following reset pulses:

  Bus is high | Reset | Wait |  Device  |  Bus Idle
  after init  | Pulse |      | Presence |
  ------------+       +------+          +-----------
              |       |      |          |
              |       |      |          |
              |       |      |          |
              +-------+      +----------+
              1       2      3          4

                [0].0  [0].1     [1].0    [1].1
  */

  static const char* TAG { "OneWire" };

  IRAM_ATTR bool rxCallbackDone(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void* queueName)
  {
      BaseType_t taskWoken = pdFALSE;
      QueueHandle_t queue = (QueueHandle_t)queueName;

      //DEBUG
      gpio_set_level(GPIO_NUM_3, 0);

      xQueueSendFromISR(queue, edata, &taskWoken);

      return taskWoken == pdTRUE;;
  }

  OneWire::OneWire(gpio_num_t gpioWire, uint8_t deviceFamily)
  {
    esp_err_t result { ESP_OK };
    m_mutex = xSemaphoreCreateMutex();
    m_receive_queue = xQueueCreate(M_QUEUE_LENGTH, sizeof(rmt_rx_done_event_data_t));
    m_gpio = gpioWire;
    m_device_family = deviceFamily;

    // Setup Bytes Encoder.
    result |= setupBytesEncoder();

    // Setup Copy Encoder.
    result |= setupCopyEncoder();
   
    // Setup Tx Config.
    setupTxConfig();

    // Setup Rx Config.
    setupRxConfig(); 
            
    // Create Rx Channel. N.b. Ensure Rx channel is defined first (see manual). 
    result |= createRxChannel();

    // Create Tx Channel.
    result |= createTxChannel();

    // Enable Rx Channel.
    result |= enableRxChannel();

    // Enable Tx Channel.
    result |= enableTxChannel();

    // Setup Callback.
    result |= setupCallback(); 

    // Initialise Line to High. This line sends the first eot message.  Without it the line does not go high.
    result |= TransmitSymbol(m_copy_encoder_handle, m_release_symbol);

    // Look for Devices.
    result |= scanDevices();

    if (ESP_OK == result) ESP_LOGI(TAG, "OneWire initialised.");
  }

  esp_err_t OneWire::setupBytesEncoder(void)
  {
    esp_err_t result { ESP_OK };
    
    m_bytes_encoder_config.bit0 = m_bit0_symbol;
    m_bytes_encoder_config.bit1 = m_bit1_symbol;
    m_bytes_encoder_config.flags.msb_first = 0;

    result = rmt_new_bytes_encoder(&m_bytes_encoder_config, &m_bytes_encoder_handle);

    if (ESP_OK != result) ESP_LOGW(TAG, "Unable to setup bytes encoder!");

    return result;
  }

  esp_err_t OneWire::setupCopyEncoder(void)
  {
    esp_err_t result { ESP_OK };
    
    result = rmt_new_copy_encoder(&m_copy_encoder_config, &m_copy_encoder_handle);

    if (ESP_OK != result) ESP_LOGW(TAG, "Unable to setup copy encoder!");

    return result;
  }

  void OneWire::setupTxConfig(void)
  {
    m_tx_config.loop_count = 0;
    m_tx_config.flags.eot_level = 1;
  }

  void OneWire::setupRxConfig(void)
  {
    m_rx_config.signal_range_min_ns = 1'000'000'000 / M_RESOLUTION_HZ; // 1 us.
    m_rx_config.signal_range_max_ns = (M_RESET_PULSE_DURATION + M_RESET_WAIT_DURATION) * 1000; // 700 us.
  }

  esp_err_t OneWire::createRxChannel()
  {
    esp_err_t result { ESP_OK };

    m_rx_channel_config.clk_src = RMT_CLK_SRC_DEFAULT;
    m_rx_channel_config.resolution_hz = M_RESOLUTION_HZ; // 1 Mhz - 1 Tick = 1us.
    m_rx_channel_config.mem_block_symbols = M_RX_MEM_BLOCKS;
    m_rx_channel_config.gpio_num = m_gpio;
    
    result = rmt_new_rx_channel(&m_rx_channel_config, &m_rx_channel);

    if (ESP_OK != result) ESP_LOGW(TAG, "Unable to create Rx channel!");

    return result;
  }

  esp_err_t OneWire::createTxChannel()
  {
    esp_err_t result { ESP_OK };

    m_tx_channel_config.clk_src = RMT_CLK_SRC_DEFAULT;
    m_tx_channel_config.resolution_hz = M_RESOLUTION_HZ; // 1 Mhz - 1 Tick = 1us.
    m_tx_channel_config.mem_block_symbols = M_TX_MEM_BLOCKS;
    m_tx_channel_config.gpio_num = m_gpio;
    m_tx_channel_config.trans_queue_depth = M_TX_QUEUE_DEPTH;
    m_tx_channel_config.flags.io_loop_back = true;
    m_tx_channel_config.flags.io_od_mode = true;
    
    ESP_ERROR_CHECK(rmt_new_tx_channel(&m_tx_channel_config, &m_tx_channel));
    
    if (ESP_OK != result) ESP_LOGW(TAG, "Unable to create Tx channel!");
    
    return result;
  }

  esp_err_t OneWire::enableRxChannel(void)
  {
    esp_err_t result { ESP_OK };

    result = rmt_enable(m_rx_channel);

    if (ESP_OK != result) ESP_LOGW(TAG, "Unable to enable Rx channel!");

    return result;
  }

  esp_err_t OneWire::enableTxChannel(void)
  {
    esp_err_t result { ESP_OK };

    result = rmt_enable(m_tx_channel);

    if (ESP_OK != result) ESP_LOGW(TAG, "Unable to enable Rx channel!");

    return result;
  }

  esp_err_t OneWire::setupCallback(void)
  {
    esp_err_t result { ESP_OK };

    m_callback.on_recv_done = rxCallbackDone;
    
    result = rmt_rx_register_event_callbacks(m_rx_channel, &m_callback, m_receive_queue);

    if (ESP_OK != result) ESP_LOGW(TAG, "Unable to setup callback!");

    return result;
  }

  esp_err_t OneWire::scanDevices(void)
  {
    esp_err_t result { ESP_OK };
    uint8_t deviceFamilyIndex { 0 };

    while (!m_last_device)
    {
      m_rom_number_t nextRom { };
      
      // Look for next device.
      result |= getNextDevice(nextRom);

      if (ESP_OK != result)
      {
        ESP_LOGE(TAG, "Device search failed!");
        return result;
      }

      // Is the family correct.
      if (m_device_family != nextRom.romNumber[deviceFamilyIndex])
      {
        ESP_LOGW(TAG, "Device found, but family incorrect.");
        continue;
      }

      // Store device.
      m_devices_found.push_back(nextRom);

      //DEBUG
      uint64_t temp = nextRom.romNumber[6];
      temp = (temp << 8) + nextRom.romNumber[5];
      temp = (temp << 8) + nextRom.romNumber[4];
      temp = (temp << 8) + nextRom.romNumber[3];
      temp = (temp << 8) + nextRom.romNumber[2];
      temp = (temp << 8) + nextRom.romNumber[1];

      ESP_LOGI(TAG, "Device Found: %llX", temp);
    }

    return result;
  }

  bool OneWire::isDevicePresent(rmt_symbol_word_t *symbols, size_t totalSymbols)
  {
    bool result { false };

    if (totalSymbols < 2) return false; // There should be at least 2 symbols (3 or 4 edges).
          
    if (symbols[0].level1 == 1) 
    { 
      // Bus is high before reset pulse.
      if ((symbols[0].duration1 > M_RESET_PRESENCE_WAIT_DURATION_MIN) && (symbols[1].duration0 > M_RESET_PRESENCE_DURATION_MIN)) result = true;
    }
    else 
    { 
      // Bus is low before reset pulse.
      if ((symbols[0].duration0 > M_RESET_PRESENCE_WAIT_DURATION_MIN) && (symbols[1].duration1 > M_RESET_PRESENCE_DURATION_MIN)) result =  true;
    }
    
    return result;
  }

  void OneWire::decodeRmtData(rmt_symbol_word_t* rmtSymbols, size_t numSymbols, uint8_t* rxBuffer, size_t rxBufferSize)
  {
    size_t bytePos { 0 };
    size_t bitPos { 0 };
    size_t counter { 0 };

    while ((counter < numSymbols) && (bytePos < rxBufferSize)) 
    {
      if (rmtSymbols[counter].duration0 > M_SLOT_BIT_SAMPLE_TIME) 
      {
        // 0 bit case.
        rxBuffer[bytePos] &= ~(1 << bitPos); // LSB first
      } 
      else 
      { 
        // 1 bit case.
        rxBuffer[bytePos] |= 1 << bitPos;
      }

      bitPos++;

      if (bitPos >= 8)
      {
        bitPos = 0;
        bytePos++;
      }

      counter++;
    }
  }

  uint8_t OneWire::getCrc8Fast(uint8_t initCrc, uint8_t *input, size_t inputSize)
  {
      uint8_t crc = initCrc;

      for (size_t i { 0 }; i < inputSize; i++) 
      {
          crc = m_crc8_table[crc ^ input[i]];
      }

      return crc;
  }

  uint8_t OneWire::getCrc8(uint8_t initCrc, uint8_t *input, size_t inputSize)
  {
    uint8_t crc = initCrc;
    uint8_t byte { };
    uint8_t x { };

    for (size_t i { 0 }; i < inputSize; i++) 
    {
      byte = input[i];

      for (int j { 0 }; j < 8; j++) 
      {
        x = (byte ^ crc) & 0x01;

        crc >>= 1;

        if (x != 0) crc ^= 0x8C;
              
        byte >>= 1;
      }
    }

    return crc;
  }

  esp_err_t OneWire::getNextDevice(m_rom_number_t& currentRomNumber)
  {
    esp_err_t result { ESP_OK };
    
    // Last device reached?
    if (m_last_device)
    {
      ESP_LOGI(TAG, "Last device found.");
      return result;
    }

    // Reset bus.
    result |= Reset();
    if (ESP_OK != result) return result;
      
    // Send Rom search command and start search algorithm.
    uint8_t txData[1] { M_CMD_SEARCH_NORMAL };  // 0xF0.
    uint8_t txDataSize { sizeof(txData) / sizeof(txData[0]) };

    result |= TransmitBytes(txData, txDataSize);
    if (ESP_OK != result) return result;

    uint8_t lastZero { 0 };
    uint8_t romLengthBits { (sizeof(currentRomNumber.romNumber) * 8) }; // 64 bits.
    uint8_t searchDirection { };

    for (uint16_t romBitIndex { 0 }; romBitIndex < romLengthBits; romBitIndex++)
    {
      uint8_t romByteIndex = romBitIndex / 8;
      uint8_t romBitMask = 1 << (romBitIndex % 8); 

      uint8_t romBit { 0 };
      uint8_t romBitComplement { 0 };
      
      result |= ReadBit(romBit);
      if (ESP_OK != result) return result;

      result |= ReadBit(romBitComplement);
      if (ESP_OK != result) return result;

      // Case 1,1 = No devices replied.
      if (romBit && romBitComplement) 
      {
        ESP_LOGE(TAG, "No devices replied!");
        return ESP_ERR_NOT_FOUND;
      }
      
      // Case 0,1 or 1,0.  All bits 0 or all bits 1. No discrepancy.
      if (romBit != romBitComplement) 
      {
        searchDirection = romBit;
      } 
      
      // Case 0,0.  There is a mixture pf 1s and 0s.  A discrepancy exists.
      if ((romBit == 0) && (romBitComplement == 0))
      {
        if (romBitIndex < m_last_discrepancy) 
        { 
          searchDirection = (currentRomNumber.romNumber[romByteIndex] & romBitMask) ? 0x01 : 0x00; 
        } 
        else
        {
          searchDirection = (romBitIndex == m_last_discrepancy) ? 0x01 : 0x00;
        }

        if (searchDirection == 0) 
        {
          lastZero = romBitIndex;
        }
      }

      if (searchDirection == 1) 
      { 
        currentRomNumber.romNumber[romByteIndex] |= romBitMask;
      }
      else 
      {
        currentRomNumber.romNumber[romByteIndex] &= ~romBitMask;
      }

      result |= TransmitBit(searchDirection);
      if (ESP_OK != result) return result;
    }
    
    // If the search was successful.
    m_last_discrepancy = lastZero;

    if (m_last_discrepancy == 0)
    { 
      m_last_device = true;
    }

    // Check crc.
    const uint8_t initCrc { 0 };
    const size_t crcByte { 7 }; 

    if (getCrc8(initCrc, currentRomNumber.romNumber, crcByte) != currentRomNumber.romNumber[crcByte])
    {
      ESP_LOGE(TAG, "CRC check failed!");
    }
    
    return result;
  }

  esp_err_t OneWire::Reset(void)
  {
    esp_err_t result { ESP_OK };

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    result |= rmt_receive(m_rx_channel, m_rx_buffer, sizeof(rmt_symbol_word_t) * 2, &m_rx_config);

    result |= TransmitSymbol(m_copy_encoder_handle, m_reset_pulse_symbol);
    
    xQueueReceive(m_receive_queue, &m_rx_event_data, pdMS_TO_TICKS(1000));

    xSemaphoreGive(m_mutex);

    if (!isDevicePresent(m_rx_event_data.received_symbols, m_rx_event_data.num_symbols)) 
    {
       ESP_LOGW(TAG, "No device found");

       result =  ESP_ERR_NOT_FOUND;
    }

    return result;
  }

  esp_err_t OneWire::TransmitBit(const uint8_t txBit)
  {
    esp_err_t result { ESP_OK };
    rmt_symbol_word_t symbolToTransmit = txBit ? m_bit1_symbol : m_bit0_symbol;
    int timeout { 50 }; // 50 ms.

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    result |= TransmitSymbol(m_copy_encoder_handle, symbolToTransmit);
    
    result |= rmt_tx_wait_all_done(m_tx_channel, timeout);

    xSemaphoreGive(m_mutex);

    if (ESP_OK != result) ESP_LOGW(TAG, "Bit transmission failed!");

    return result;
  }

  esp_err_t OneWire::TransmitSymbol(const rmt_encoder_handle_t& encoder, const rmt_symbol_word_t& symbol)
  {
    esp_err_t result { ESP_OK };

    result = rmt_transmit(m_tx_channel, encoder, &symbol, sizeof(symbol), &m_tx_config);

    if (ESP_OK != result) ESP_LOGW(TAG, "Unable to transmit symbol!");
    
    return result;
  }

  esp_err_t OneWire::TransmitBytes(const uint8_t *txData, uint8_t txDataSize)
  {
    esp_err_t result { ESP_OK };
    int timeout { 50 }; // 50 ms.

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    result |= rmt_transmit(m_tx_channel, m_bytes_encoder_handle, txData, txDataSize, &m_tx_config);
      
    result |= rmt_tx_wait_all_done(m_tx_channel, timeout);

    xSemaphoreGive(m_mutex);

    return result;
  }

  esp_err_t OneWire::ReadBit(uint8_t& rxBit)
  {
    esp_err_t result { ESP_OK };
    uint8_t rxBuffer { 0 };

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    result |= rmt_receive(m_rx_channel, m_rx_buffer, sizeof(rmt_symbol_word_t), &m_rx_config);

    result |= rmt_transmit(m_tx_channel, m_copy_encoder_handle, &m_bit1_symbol, sizeof(rmt_symbol_word_t), &m_tx_config);

    xQueueReceive(m_receive_queue, &m_rx_event_data, pdMS_TO_TICKS(1000));

    decodeRmtData(m_rx_event_data.received_symbols, m_rx_event_data.num_symbols, &rxBuffer, sizeof(rxBuffer));

    rxBit = rxBuffer & 0x01;

    xSemaphoreGive(m_mutex);

    return result;
  }

  esp_err_t OneWire::ReadBytes(uint8_t *rxBuffer, size_t rxBufferSize)
  {
    if (rxBufferSize > M_RX_MAX_BYTES)
    {
      ESP_LOGW(TAG, "ReadBytes: rxBufferSize too large!");
      return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result { ESP_OK };
    size_t txBufferSize { rxBufferSize };
    uint8_t txBuffer[rxBufferSize];

    for (size_t i { 0 }; i < txBufferSize; i++)
    {
      txBuffer[i] = 0xFF;  
    }

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    result |= rmt_receive(m_rx_channel, m_rx_buffer, (sizeof(rmt_symbol_word_t) * rxBufferSize * 8), &m_rx_config);

    result |= rmt_transmit(m_tx_channel, m_bytes_encoder_handle, txBuffer, sizeof(txBuffer), &m_tx_config);

    xQueueReceive(m_receive_queue, &m_rx_event_data, pdMS_TO_TICKS(1000));

    decodeRmtData(m_rx_event_data.received_symbols, m_rx_event_data.num_symbols, rxBuffer, rxBufferSize);

    xSemaphoreGive(m_mutex);

    return result;
  }

  }
