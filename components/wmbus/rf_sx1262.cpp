#include "rf_sx1262.h"

namespace esphome {
namespace wmbus {

  static const char *TAG = "rxLoop";

  bool RxLoop::init(GPIOPin *gdo0, GPIOPin *gdo2, GPIOPin *reset, float freq, bool syncMode) {
    bool retVal = false;
    this->syncMode = syncMode;
    this->gdo0 = gdo0;
    this->gdo2 = gdo2;
    this->reset = reset;
    this->gdo0->setup();
    this->gdo2->setup();
    this->reset->setup();
    this->spi_setup();

    resetDevice();
    standby(RADIOLIB_SX126X_STANDBY_RC);
    setPacketType(RADIOLIB_SX126X_PACKET_TYPE_GFSK);
    setRfFrequency(freq);
    setBufferBaseAddress(0x00, 0x00);
    setModulationParams(32.768f, 50.0f, 156.2f, RADIOLIB_SX126X_GFSK_FILTER_NONE); // bitrate, freqDeviation, Bandwidth, PulseShape
    setPacketParams(16, // bitlength of TX preamble
                    RADIOLIB_SX126X_GFSK_PREAMBLE_DETECT_8, 
                    8, // bitlength of syncword
                    RADIOLIB_SX126X_GFSK_ADDRESS_FILT_OFF, 
                    RADIOLIB_SX126X_GFSK_PACKET_VARIABLE, 
                    0xff,
                    RADIOLIB_SX126X_GFSK_CRC_OFF, 
                    RADIOLIB_SX126X_GFSK_WHITENING_OFF);
    //state = setDioIrqParams(RADIOLIB_SX126X_IRQ_RX_DONE, RADIOLIB_SX126X_IRQ_RX_DONE, 0, 0);
    setDioIrqParams(RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID, RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID, 0, 0);
    setSyncWord();
    setDIO3AsTCXOCtrl(RADIOLIB_SX126X_DIO3_OUTPUT_3_0, 64);  // Delay = 1 ms / 0.015625 ms = 64
    setFallbackMode(RADIOLIB_SX126X_RX_TX_FALLBACK_MODE_STDBY_XOSC);
    standby(RADIOLIB_SX126X_STANDBY_XOSC);
    setRx(0x000000);

    if((getStatus() & 0x70) == RADIOLIB_SX126X_STATUS_MODE_RX) {
      retVal = true;
      ESP_LOGD(TAG, "SX1262 initialized");
    }
    else {
      ESP_LOGE(TAG, "SX1262 initialization FAILED!");
    }

    return retVal;
  }

  bool RxLoop::task() {
    do {
      switch (rxLoop.state) {
        case INIT_RX:
          start();
          return false;

        // RX active, waiting for SYNC
        case WAIT_FOR_SYNC:
          if (digitalRead(this->gdo2)) {
            if (getIrqStatus() & RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID) { // assert when SYNC detected
                clearIrqStatus(RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID;)
                rxLoop.state = WAIT_FOR_DATA;
                sync_time_ = millis();
            }
          }
          break;

        // waiting for enough data in Rx FIFO buffer
        case WAIT_FOR_DATA:
          if (digitalRead(this->gdo0) && (getIrqStatus() & RADIOLIB_SX126X_IRQ_RX_DONE)) { // assert when Rx FIFO buffer threshold reached
            uint8_t bytesInFIFO = getRxPayloadLength();
            if (bytesInFIFO < 3) {
                rxLoop.state = INIT_RX;
                return task();
            }
            uint8_t preamble[2];
            // Read the 3 first bytes,
            readBuffer(rxLoop.pByteIndex, 0, 3);
            rxLoop.bytesRx = 3;
            bytesInFIFO = bytesInFIFO - 3;
            const uint8_t *currentByte = rxLoop.pByteIndex;
            // Mode C
            if (*currentByte == WMBUS_MODE_C_PREAMBLE) {
              currentByte++;
              data_in.mode = 'C';
              // Block A
              if (*currentByte == WMBUS_BLOCK_A_PREAMBLE) {
                currentByte++;
                rxLoop.lengthField = *currentByte;
                rxLoop.length = 2 + packetSize(rxLoop.lengthField);
                data_in.block = 'A';
              }
              // Block B
              else if (*currentByte == WMBUS_BLOCK_B_PREAMBLE) {
                currentByte++;
                rxLoop.lengthField = *currentByte;
                rxLoop.length = 2 + 1 + rxLoop.lengthField;
                data_in.block = 'B';
              }
              // Unknown type, reinit loop
              else {
                rxLoop.state = INIT_RX;
                return task();
              }
              // don't include C "preamble"
              *(rxLoop.pByteIndex) = rxLoop.lengthField;
              rxLoop.pByteIndex += 1;
            }
            // Mode T Block A
            else if (decode3OutOf6(rxLoop.pByteIndex, preamble)) {
              rxLoop.lengthField  = preamble[0];
              data_in.lengthField = rxLoop.lengthField;
              rxLoop.length  = byteSize(packetSize(rxLoop.lengthField));
              data_in.mode   = 'T';
              data_in.block  = 'A';
              rxLoop.pByteIndex += 3;
            }
            // Unknown mode, reinit loop
            else {
              rxLoop.state = INIT_RX;
              return task();
            }

            rxLoop.bytesLeft = rxLoop.length - 3;
/*
            if (rxLoop.length < MAX_FIXED_LENGTH) {
              // Set CC1101 into length mode
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTLEN, (uint8_t)rxLoop.length);
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
              rxLoop.cc1101Mode = FIXED;
            }
            else {
              // Set CC1101 into infinite mode
              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTLEN, (uint8_t)(rxLoop.length%MAX_FIXED_LENGTH));
            }
*/
            rxLoop.state = READ_DATA;
            max_wait_time_ += extra_time_;

            clearIrqStatus(RADIOLIB_SX126X_IRQ_RX_DONE);
//            ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, RX_FIFO_THRESHOLD);
/*            return task();
          }
          break;

        // waiting for more data in Rx FIFO buffer
        case READ_DATA:
          if (digitalRead(this->gdo0) && (getIrqStatus() & RADIOLIB_SX126X_IRQ_RX_DONE)) { // assert when Rx FIFO buffer threshold reached
            if ((rxLoop.bytesLeft < MAX_FIXED_LENGTH) && (rxLoop.cc1101Mode == INFINITE)) {
//              ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
              rxLoop.cc1101Mode = FIXED;
            }
            // Do not empty the Rx FIFO (See the CC1101 SWRZ020E errata note)
            uint8_t bytesInFIFO = ELECHOUSE_cc1101.SpiReadStatus(CC1101_RXBYTES) & 0x7F;
            ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, rxLoop.pByteIndex, bytesInFIFO - 1);
*/
            readBuffer(rxLoop.pByteIndex, rxLoop.pByteIndex, bytesInFIFO);
            rxLoop.bytesLeft  -= (bytesInFIFO - 1);
            rxLoop.pByteIndex += (bytesInFIFO - 1);
            rxLoop.bytesRx    += (bytesInFIFO - 1);
            max_wait_time_    += extra_time_;
          }
          break;
      }

      uint8_t overfl = 0;// ELECHOUSE_cc1101.SpiReadStatus(CC1101_RXBYTES) & 0x80;
      // end of packet in length mode
      if ((!overfl) && (!digitalRead(gdo2))  && (rxLoop.state > WAIT_FOR_DATA)) {
        ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, rxLoop.pByteIndex, (uint8_t)rxLoop.bytesLeft);
        rxLoop.bytesRx += rxLoop.bytesLeft;
        data_in.length  = rxLoop.bytesRx;
//        this->returnFrame.rssi  = (int8_t)ELECHOUSE_cc1101.getRssi();
//        this->returnFrame.lqi   = (uint8_t)ELECHOUSE_cc1101.getLqi();
        this->returnFrame.rssi = getRssiInst();
        this->returnFrame.lqi = 0; // find a measur to put here
        ESP_LOGV(TAG, "Have %d bytes from SX1262 Rx, RSSI: %d dBm LQI: %d", rxLoop.bytesRx, this->returnFrame.rssi, this->returnFrame.lqi);
        if (rxLoop.length != data_in.length) {
          ESP_LOGE(TAG, "Length problem: req(%d) != rx(%d)", rxLoop.length, data_in.length);
        }
        if (this->syncMode) {
          ESP_LOGV(TAG, "Synchronus mode enabled.");
        }
        if (mBusDecode(data_in, this->returnFrame)) {
          rxLoop.complete = true;
          this->returnFrame.mode  = data_in.mode;
          this->returnFrame.block = data_in.block;
        }
        rxLoop.state = INIT_RX;
        return rxLoop.complete;
      }
      start(false);
    } while ((this->syncMode) && (rxLoop.state > WAIT_FOR_SYNC));
    return rxLoop.complete;
  }

  WMbusFrame RxLoop::get_frame() {
    return this->returnFrame;
  }

  bool RxLoop::start(bool force) {
    // waiting to long for next part of data?
    bool reinit_needed = ((millis() - sync_time_) > max_wait_time_) ? true: false;
    if (!force) {
      if (!reinit_needed) {
        // already in RX?
        if ((getStatus() & 0x70) == RADIOLIB_SX126X_STATUS_MODE_RX) {
          return false;
        }
      }
    }
    // init RX here, each time we're idle
    rxLoop.state = INIT_RX;
    sync_time_ = millis();
    max_wait_time_ = extra_time_;

    setRx();

    // Initialize RX info variable
    rxLoop.lengthField = 0;              // Length Field in the wM-Bus packet
    rxLoop.length      = 0;              // Total length of bytes to receive packet
    rxLoop.bytesLeft   = 0;              // Bytes left to to be read from the Rx FIFO
    rxLoop.bytesRx     = 0;              // Bytes read from Rx FIFO
    rxLoop.pByteIndex  = data_in.data;   // Pointer to current position in the byte array
    rxLoop.complete    = false;          // Packet received
    rxLoop.cc1101Mode  = INFINITE;       // Infinite or fixed CC1101 packet mode

    this->returnFrame.frame.clear();
    this->returnFrame.rssi  = 0;
    this->returnFrame.lqi   = 0;
    this->returnFrame.mode  = 'X';
    this->returnFrame.block = 'X';

    std::fill( std::begin( data_in.data ), std::end( data_in.data ), 0 );
    data_in.length      = 0;
    data_in.lengthField = 0;
    data_in.mode        = 'X';
    data_in.block       = 'X';
/*
    // Set Rx FIFO threshold to 4 bytes
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, RX_FIFO_START_THRESHOLD);
    // Set infinite length 
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, INFINITE_PACKET_LENGTH);

    ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
    while((ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) != MARCSTATE_RX));
*/
    rxLoop.state = WAIT_FOR_SYNC;

    return true; // this will indicate we just have re-started Rx
  }

  void RxLoop::resetDevice() {
    // Reset device
    this->reset_pin_->digital_write(true);
    delay(2);
    this->reset_pin_->digital_write(false);
    delay(2);
    this->reset_pin_->digital_write(true);
  }

  uint16_t RxLoop::getStatus() {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_GET_STATUS, 0x00 };
    uint8_t respons[2];
            
    // Wait until device is not BUSY
    while(digital_read(gdo0)){
        delay(1);
    }
    this->delegate_->begin_transaction();
    this->delegate_->transfer(command, respons, sizeof(command));
    this->delegate_->end_transaction();

    return((uint16_t) respons[1]);
  }
  
  uint16_t RxLoop::getIrqStatus() {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_GET_IRQ_STATUS, 0x00, 0x00, 0x00 };
    uint8_t respons[4];

    // Wait until device is not BUSY
    while(digital_read(gdo0)){
        delay(1);
    }
    this->delegate_->begin_transaction();
    this->delegate_->transfer(command, respons, sizeof(command));
    this->delegate_->end_transaction();

    return((uint16_t)(respons[2] << 8) | respons[3]);
  }

  void RxLoop::sx1262command(uint8_t *command, uint32_t length) {
            
    // Wait until device is not BUSY
    while(digital_read(gdo0)){
      delay(1);
    }
    this->delegate_->begin_transaction();
    this->delegate_->write_array(command, length);
    this->delegate_->end_transaction();
  }
  
  uint8_t RxLoop::getRxPayloadLength() {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_GET_RX_BUFFER_STATUS, 0x00, 0x00 };
    uint8_t respons[3];

    sx1262command(command, respons, sizeof(command));
    return(respons[2]);
  }

  void RxLoop::readBuffer(uint8_t *buffer, uint8_t offset, uint8_t length) {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_READ_BUFFER, offset, 0x00 };

    this->delegate_->begin_transaction();
    this->delegate_->write_array(command, sizeof(command));
    this->delegate_->read_array(buffer, length < MAX_FIXED_LENGTH ? length : MAX_FIXED_LENGTH);
    this->delegate_->end_transaction();
  }
  
  uint8_t RxLoop::GetRssiInst() {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_GET_RSSI_INST, 0x00, 0x00 };
    uint8_t respons[3];

    sx1262command(command, respons, sizeof(command));
    return(respons[2] / 2);
  }

  void RxLoop::standby(uint8_t mode) {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_SET_STANDBY, mode };
    sx1262command(data, sizeof(command));
  }

  void RxLoop::setRx(uint32_t timeout) {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_SET_RX, 
      (uint8_t)((timeout >> 16) & 0xff),
      (uint8_t)((timeout >> 8) & 0xff),
      (uint8_t)(timeout & 0xff)
    };
    sx1262command(command, sizeof(command));
  }

  void RxLoop::setPacketType(uint8_t type) {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_SET_PACKET_TYPE, type };
    sx126Xcommand(data, sizeof(command));
  }

  void RxLoop::setRfFrequency(float freq) {
    uint32_t freqRaw = (freq * (uint32_t(1) << RADIOLIB_SX126X_DIV_EXPONENT)) / RADIOLIB_SX126X_CRYSTAL_FREQ;

    uint8_t command[] = { RADIOLIB_SX126X_CMD_SET_RF_FREQUENCY, 
      (uint8_t)((freqRaw >> 24) & 0xff),
      (uint8_t)((freqRaw >> 16) & 0xff),
      (uint8_t)((freqRaw >> 8) & 0xff),
      (uint8_t)(freqRaw & 0xff)
    };
    sx1262command(command, sizeof(command));
  }

  void RxLoop::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_SET_BUFFER_BASE_ADDRESS, txBaseAddress, rxBaseAddress };
    sx1262command(command, sizeof(command));
  }

  void RxLoop::setModulationParams(float br, float freqDev, float rxBw, uint32_t pulseShape) {

    // calculate raw bit rate value
    uint32_t brRaw = (uint32_t)((RADIOLIB_SX126X_CRYSTAL_FREQ * 1000000.0f * 32.0f) / (br * 1000.0f));

    // calculate raw frequency deviation value
    uint32_t freqDevRaw = (uint32_t)(((freqDev * 1000.0f) * (float)((uint32_t)(1) << 25)) / (RADIOLIB_SX126X_CRYSTAL_FREQ * 1000000.0f));

    // check allowed receiver bandwidth values

    uint8_t rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_467_0;

    if(fabsf(rxBw - 4.8f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_4_8;
    } else if(fabsf(rxBw - 5.8f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_5_8;
    } else if(fabsf(rxBw - 7.3f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_7_3;
    } else if(fabsf(rxBw - 9.7f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_9_7;
    } else if(fabsf(rxBw - 11.7f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_11_7;
    } else if(fabsf(rxBw - 14.6f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_14_6;
    } else if(fabsf(rxBw - 19.5f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_19_5;
    } else if(fabsf(rxBw - 23.4f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_23_4;
    } else if(fabsf(rxBw - 29.3f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_29_3;
    } else if(fabsf(rxBw - 39.0f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_39_0;
    } else if(fabsf(rxBw - 46.9f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_46_9;
    } else if(fabsf(rxBw - 58.6f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_58_6;
    } else if(fabsf(rxBw - 78.2f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_78_2;
    } else if(fabsf(rxBw - 93.8f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_93_8;
    } else if(fabsf(rxBw - 117.3f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_117_3;
    } else if(fabsf(rxBw - 156.2f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_156_2;
    } else if(fabsf(rxBw - 187.2f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_187_2;
    } else if(fabsf(rxBw - 234.3f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_234_3;
    } else if(fabsf(rxBw - 312.0f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_312_0;
    } else if(fabsf(rxBw - 373.6f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_373_6;
    } else if(fabsf(rxBw - 467.0f) <= 0.001f) {
      rxBandwidth = RADIOLIB_SX126X_GFSK_RX_BW_467_0;
    }
  
    uint8_t command[] = { RADIOLIB_SX126X_CMD_SET_MODULATION_PARAMS,
      (uint8_t)((brRaw >> 16) & 0xFF), (uint8_t)((brRaw >> 8) & 0xFF), (uint8_t)(brRaw & 0xFF),
      (uint8_t) pulseShape, (uint8_t) rxBandwidth,
      (uint8_t)((freqDevRaw >> 16) & 0xFF), (uint8_t)((freqDevRaw >> 8) & 0xFF), (uint8_t)(freqDevRaw & 0xFF)
    };

    sx1262command(command, sizeof(command));
  }

  void RxLoop::setPacketParams(uint16_t preambleLen, uint8_t preambleDetectorLen, 
    uint8_t syncWordLen, uint8_t addrComp, uint8_t packType, uint8_t payloadLen,
    uint8_t crcType, uint8_t whiten) {

    uint8_t command[] = {
        RADIOLIB_SX126X_CMD_SET_PACKET_PARAMS,
        (uint8_t)((preambleLen >> 8) & 0xFF), (uint8_t)(preambleLen & 0xFF),
        preambleDetectorLen, syncWordLen, addrComp,
        packType, payloadLen, crcType, whiten
    };
    sx1262command(command, sizeof(command));
  }

  void RxLoop::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
    uint8_t command[] = {RADIOLIB_SX126X_CMD_SET_DIO_IRQ_PARAMS,
                       (uint8_t)((irqMask >> 8) & 0xFF), (uint8_t)(irqMask & 0xFF),
                       (uint8_t)((dio1Mask >> 8) & 0xFF), (uint8_t)(dio1Mask & 0xFF),
                       (uint8_t)((dio2Mask >> 8) & 0xFF), (uint8_t)(dio2Mask & 0xFF),
                       (uint8_t)((dio3Mask >> 8) & 0xFF), (uint8_t)(dio3Mask & 0xFF)
    };
    sx1262command(command,sizeof(command));
  }

  void RxLoop::clearIrqStatus(uint16_t clearIrqParams) {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_CLEAR_IRQ_STATUS, (uint8_t)((clearIrqParams >> 8) & 0xFF), (uint8_t)(clearIrqParams & 0xFF) };
    sx1262command(command, sizeof(command)));
  }

  void RxLoop::setSyncWord() {
    uint8_t command[] = { 
      RADIOLIB_SX126X_CMD_WRITE_REGISTER,
      RADIOLIB_SX126X_REG_SYNC_WORD_0 >> 8 & 0xff, RADIOLIB_SX126X_REG_SYNC_WORD_0  & 0xff,
      0x54, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    sx1262command(command, sizeof(command));
  }

  void RxLoop::setDIO3AsTCXOCtrl(uint8_t tcxoVoltage, uint32_t delay) {
    uint8_t command[] = {RADIOLIB_SX126X_CMD_SET_DIO3_AS_TCXO_CTRL,
                       tcxoVoltage,
                       (uint8_t)((delay >> 24) & 0xFF), (uint8_t)((delay >> 16) & 0xFF), (uint8_t)(delay & 0xff)
    };
    sx1262command(command, sizeof(command));
  }

  void RxLoop::setFallbackMode(uint8_t mode) {
    uint8_t command[] = { RADIOLIB_SX126X_CMD_SET_RX_TX_FALLBACK_MODE, mode };
    sx1262command(command, sizeof(command));
  }

}
}