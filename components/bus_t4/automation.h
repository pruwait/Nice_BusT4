#pragma once


#include "esphome/core/automation.h"

namespace esphome {
namespace bus_t4 {

template<typename... Ts> class RawCmdAction : public Action<Ts...> {

  

  void play(Ts... x) override {
    }
  
 protected:

};

}  // namespace bus_t4
}  // namespace esphome