#include <N2kMessages.h>
#include <NMEA2000.h>

#include "sensesp/system/saveable.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/repeat.h"
#include "sensesp_base_app.h"

namespace halmet {
class N2kDCStatusSender : public sensesp::FileSystemSaveable {  // 127506
 public:
  N2kDCStatusSender(String config_path, uint8_t battery_instance,
                    tNMEA2000* nmea2000)
      : sensesp::FileSystemSaveable{config_path},
        battery_instance_{battery_instance},
        nmea2000_{nmea2000},
        repeat_interval_{1000},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{30000}           // In ms. When the inputs expire.
  {
    this->load();
    this->initialize_members(repeat_interval_, expiry_);
    if (this->enabled_)
      sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
        tN2kMsg N2kMsg;
        // At the moment, the PGN is sent regardless of whether all the values
        // are invalid or not.
        SetN2kDCStatus(N2kMsg, 0, this->battery_instance_, N2kDCt_Battery,
                       this->stateOfCharge->get(), this->stateOfHealth->get(),
                       this->timeRemaining->get(), this->rippleVoltage->get(),
                       this->capacity->get());
        this->nmea2000_->SendMsg(N2kMsg);
      });
  }

  virtual bool from_json(const JsonObject& config) override {
    if (!config["enabled"].is<bool>()) {
      return false;
    }
    enabled_ = config["enabled"];

    if (!config["battery_instance"].is<int>()) {
      return false;
    }
    battery_instance_ = config["battery_instance"];
    return true;
  }

  virtual bool to_json(JsonObject& config) override {
    config["enabled"] = enabled_;
    config["battery_instance"] = battery_instance_;
    return true;
  }

  std::shared_ptr<sensesp::RepeatExpiring<unsigned char>> stateOfCharge;
  std::shared_ptr<sensesp::RepeatExpiring<unsigned char>> stateOfHealth;
  std::shared_ptr<sensesp::RepeatExpiring<double>> timeRemaining;
  std::shared_ptr<sensesp::RepeatExpiring<double>> rippleVoltage;
  std::shared_ptr<sensesp::RepeatExpiring<double>> capacity;

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  bool enabled_ = false;
  uint8_t battery_instance_ = 0;

 private:
  void initialize_members(unsigned int repeat_interval, unsigned int expiry) {
    // Initialize the RepeatExpiring objects
    stateOfCharge = std::make_shared<sensesp::RepeatExpiring<unsigned char>>(
        repeat_interval, expiry);
    stateOfHealth = std::make_shared<sensesp::RepeatExpiring<unsigned char>>(
        repeat_interval, expiry);
    timeRemaining = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
    rippleVoltage = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
    capacity = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
  }
};

const String ConfigSchema(const N2kDCStatusSender& obj) {
  return R"###({
     "type": "object",
     "properties": {
       "enabled": { "title": "enabled", "type": "bool", "description": "enable sending" },
       "battery_instance": { "title": "battery instance", "type": "integer", "description": "battery NMEA 2000 instance number (0-253)" }
     }
   })###";
}

}  // namespace halmet
