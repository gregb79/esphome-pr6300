#pragma once
#include "esphome.h"
#include <sstream>

namespace esphome {
namespace remote_transmitter {

class IrisProtocol : public Component {
 public:
  RemoteTransmitterComponent *parent;

  void set_parent(RemoteTransmitterComponent *p) { parent = p; }

  void send_frame(int id0, int id1, int instruction, int mode) {
    std::vector<int> raw = get_data(id0, id1, instruction, mode);
    if (parent != nullptr) {
      parent->transmit_raw(raw, 4, 11);  // repeat 4x, 11ms gap
    }
  }

 private:
  std::vector<int> get_data(int id0, int id1, int instruction, int mode) {
    uint8_t DATA_TABLE[12]    = {0xAA,0xAA,0xAA,0xAA,0x2D,0xD4,0xF9,203,0x00,17,3,0x00};
    uint8_t DATACRC_TABLE[12] = {0};

    // fill dynamic fields
    DATA_TABLE[6]  = id0;
    DATA_TABLE[7]  = id1;
    DATA_TABLE[9]  = instruction;
    DATA_TABLE[10] = mode;

    // checksum (two's complement of sum from index 4 to 10)
    unsigned int sum = 0;
    for (int i = 4; i < 11; i++) sum += DATA_TABLE[i];
    DATA_TABLE[11] = static_cast<unsigned char>(-sum);

    // copy to DATACRC_TABLE
    for (int i = 0; i < 12; i++) DATACRC_TABLE[i] = DATA_TABLE[i];

    // convert bytes to waveform
    std::vector<int> DataVector;
    for (int i = 0; i < 12; ++i) {
      uint8_t byte = DATACRC_TABLE[i];
      for (int j = 7; j >= 0; --j) {
        if (byte & (1 << j)) DataVector.push_back(105);
        else                 DataVector.push_back(-104);
      }
    }

    // optional debug
    std::ostringstream oss;
    for (int i = 0; i < 12; i++) {
      oss << "0x" << std::hex << std::uppercase << static_cast<int>(DATACRC_TABLE[i]);
      if (i < 11) oss << ", ";
    }
    ESP_LOGI("iris", "DATACRC_TABLE: %s", oss.str().c_str());

    return DataVector;
  }
};

}  // namespace remote_transmitter
}  // namespace esphome
