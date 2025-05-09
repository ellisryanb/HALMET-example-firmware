#include <N2kMessages.h>
#include <NMEA2000.h>

#include "sensesp/system/saveable.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/repeat.h"
#include "sensesp_base_app.h"

namespace halmet {
enum tN2kInverterOperatingState {
  tN2kInverterOperatingState_Invert = 0,       ///< No, Off, Disabled
  tN2kInverterOperatingState_AC_Passthru = 1,  ///< No, Off, Disabled
  tN2kInverterOperatingState_Load_Sense = 2,   ///< No, Off, Disabled
  tN2kInverterOperatingState_Fault = 3,        ///< No, Off, Disabled
  tN2kInverterOperatingState_Disabled = 4,     ///< No, Off, Disabled
  tN2kInverterOperatingState_Error = 14,       ///< No, Off, Disabled
};

/************************************************************************/ /**
* \brief Setting up PGN 127509 Message "Inverter Status"
* \ingroup group_msgSetUp
*
* Provides parametric data for a specific DC Source, indicated by the
* instance field. The type of DC Source can be identified from the
* DC Detailed Status PGN. Used primarily by display or instrumentation
* devices, but may also be used by power management.

* \param N2kMsg              Reference to a N2kMsg Object,
*                            Output: NMEA2000 message ready to be send.
* \param InverterInstance     BatteryInstance.
* \param ACInstance       BatteryInstance.
* \param BatteryInstance     BatteryInstance.
* \param OperatingState      Battery voltage in V
* \param InverterEnabled      Current in A
*/
void SetN2kPGN127509(tN2kMsg& N2kMsg, unsigned char InverterInstance,
                     unsigned char ACInstance, unsigned char BatteryInstance,
                     tN2kInverterOperatingState OperatingState,
                     tN2kOnOff InverterEnabled) {
  N2kMsg.SetPGN(127509L);
  N2kMsg.Priority = 6;
  N2kMsg.AddByte(InverterInstance);
  N2kMsg.AddByte(ACInstance);
  N2kMsg.AddByte(BatteryInstance);
  N2kMsg.AddByte((OperatingState & 0x0f) << 4 |
                 ((InverterEnabled & 0x03) << 2));
}

/************************************************************************/ /**
                                                                            * \brief
                                                                            * Setting
                                                                            * up
                                                                            * Message
                                                                            * "Battery
                                                                            * Status"
                                                                            * -
                                                                            * PGN
                                                                            * 127509
                                                                            * \ingroup
                                                                            * group_msgSetUp
                                                                            *
                                                                            * Alias
                                                                            * of
                                                                            * PGN
                                                                            * 127509.
                                                                            * This
                                                                            * alias
                                                                            * was
                                                                            * introduced
                                                                            * to
                                                                            * improve
                                                                            * the
                                                                            * readability
                                                                            * of
                                                                            * the
                                                                            * source
                                                                            * code.
                                                                            * See
                                                                            * parameter
                                                                            * details
                                                                            * on
                                                                            * \ref
                                                                            * SetN2kPGN127508
                                                                            */
inline void SetN2kInverterStatus(tN2kMsg& N2kMsg,
                                 unsigned char InverterInstance,
                                 unsigned char ACInstance,
                                 unsigned char BatteryInstance,
                                 tN2kInverterOperatingState OperatingState,
                                 tN2kOnOff InverterEnabled) {
  SetN2kPGN127509(N2kMsg, InverterInstance, ACInstance, BatteryInstance,
                  OperatingState, InverterEnabled);
}

class N2kInverterSender : public sensesp::FileSystemSaveable {  // 127509
 public:
  N2kInverterSender(String config_path, uint8_t inverter_instance,
                    uint8_t ac_instance, uint8_t battery_instance,
                    tNMEA2000* nmea2000)
      : sensesp::FileSystemSaveable{config_path},
        inverter_instance_{inverter_instance},
        battery_instance_{battery_instance},
        ac_instance_{ac_instance},
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
        SetN2kInverterStatus(N2kMsg, inverter_instance_, this->ac_instance_,
                             this->battery_instance_,
                             this->operatingState->get(),
                             this->inverterEnabled->get());
        this->nmea2000_->SendMsg(N2kMsg);
      });
  }

  virtual bool from_json(const JsonObject& config) override {
    if (!config["enabled"].is<bool>()) {
      return false;
    }
    enabled_ = config["enabled"];

    if (!config["inverter_instance"].is<int>()) {
      return false;
    }
    inverter_instance_ = config["inverter_instance"];

    if (!config["battery_instance"].is<int>()) {
      return false;
    }
    battery_instance_ = config["battery_instance"];

    if (!config["ac_instance"].is<int>()) {
      return false;
    }
    ac_instance_ = config["ac_instance"];

    return true;
  }

  virtual bool to_json(JsonObject& config) override {
    config["enabled"] = enabled_;
    config["inverter_instance"] = inverter_instance_;
    config["battery_instance"] = battery_instance_;
    config["ac_instance"] = ac_instance_;
    return true;
  }

  std::shared_ptr<sensesp::RepeatStopping<tN2kInverterOperatingState>>
      operatingState;
  std::shared_ptr<sensesp::RepeatStopping<tN2kOnOff>> inverterEnabled;

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  bool enabled_ = false;
  uint8_t inverter_instance_ = 0;
  uint8_t battery_instance_ = 0;
  uint8_t ac_instance_ = 0;

 private:
  void initialize_members(unsigned int repeat_interval, unsigned int expiry) {
    // Initialize the RepeatExpiring objects
    inverterEnabled = std::make_shared<sensesp::RepeatStopping<tN2kOnOff>>(
        repeat_interval, expiry);
    operatingState =
        std::make_shared<sensesp::RepeatStopping<tN2kInverterOperatingState>>(
            repeat_interval, expiry);
  };
};

const String ConfigSchema(const N2kInverterSender& obj) {
  return R"###({
       "type": "object",
       "properties": {
         "enabled": { "title": "enabled", "type": "bool", "description": "enable sending" },
         "inverter_instance": { "title": "Inverter instance", "type": "integer", "description": "battery NMEA 2000 instance number (0-253)" },
         "battery_instance": { "title": "battery instance", "type": "integer", "description": "battery NMEA 2000 instance number (0-253)" },
         "ac_instance": { "title": "AC instance", "type": "integer", "description": "battery NMEA 2000 instance number (0-253)" }
       }
     })###";
}
}  // namespace halmet
