#pragma once
#include "esphome/core/automation.h"
#include "iris_protocol.h"

namespace esphome {
namespace remote_transmitter {

static const char *const TAG = "transmit_iris";

// allowed instructions for autocomplete
enum class IrisInstruction {
  Power, Blue, Magenta, Red, Lime, Green, Aqua, White,
  Mode1, Mode2, Mode3, Mode4, Brightness
};

inline int instruction_to_code(IrisInstruction instr) {
  switch(instr) {
    case IrisInstruction::Power: return 17;
    case IrisInstruction::Blue: return 33;
    case IrisInstruction::Magenta: return 34;
    case IrisInstruction::Red: return 35;
    case IrisInstruction::Lime: return 36;
    case IrisInstruction::Green: return 37;
    case IrisInstruction::Aqua: return 38;
    case IrisInstruction::White: return 39;
    case IrisInstruction::Mode1: return 49;
    case IrisInstruction::Mode2: return 50;
    case IrisInstruction::Mode3: return 51;
    case IrisInstruction::Mode4: return 52;
    case IrisInstruction::Brightness: return 65;
  }
  return 17;
}

// allowed modes for autocomplete
enum class IrisMode { pool, spa, poolspa };
inline int mode_to_code(IrisMode mode) {
  switch(mode) {
    case IrisMode::pool: return 1;
    case IrisMode::spa: return 2;
    case IrisMode::poolspa: return 3;
  }
  return 3;
}

class TransmitIrisAction : public Action<> {
 public:
  void set_id0(int id0) { id0_ = id0; }
  void set_id1(int id1) { id1_ = id1; }
  void set_instruction(IrisInstruction instr) { instr_ = instruction_to_code(instr); }
  void set_mode(IrisMode mode) { mode_ = mode_to_code(mode); }
  void set_parent(IrisProtocol *proto) { proto_ = proto; }

  void play() override {
    if (proto_ == nullptr) return;
    ESP_LOGI(TAG, "Sending Iris packet: ID0=%d ID1=%d Mode=%d Instr=%d",
             id0_, id1_, mode_, instr_);
    proto_->send_frame(id0_, id1_, instr_, mode_);
  }

 protected:
  int id0_{249};      // default
  int id1_{203};      // default
  int mode_{3};       // default poolspa
  int instr_{17};     // default Power

  IrisProtocol *proto_{nullptr};
};

}  // namespace remote_transmitter
}  // namespace esphome
