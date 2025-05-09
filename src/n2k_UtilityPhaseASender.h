#include <N2kMessages.h>
#include <NMEA2000.h>

#include "sensesp/system/saveable.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/repeat.h"
#include "sensesp_base_app.h"

namespace halmet {
/************************************************************************/ /**
* \brief Setting up PGN 65013  Message "Utility Phase A AC Power"
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
void SetN2kPGN65013(tN2kMsg& N2kMsg, double RealPower, double ApparentPower) {
  N2kMsg.SetPGN(65013L);
  N2kMsg.Priority = 6;
  N2kMsg.Add4ByteDouble(RealPower, 1);
  N2kMsg.Add4ByteDouble(ApparentPower, 1);
}

/************************************************************************/ /**
                                                                            * \brief
                                                                            * Setting
                                                                            * up
                                                                            * Message
                                                                            * "Utility
                                                                            * Phase
                                                                            * A
                                                                            * AC
                                                                            * Power"
                                                                            * -
                                                                            * PGN
                                                                            * 65013
                                                                            * \ingroup
                                                                            * group_msgSetUp
                                                                            *
                                                                            * Alias
                                                                            * of
                                                                            * PGN
                                                                            * 65013
                                                                            * .
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
                                                                            * SetN2kPGN65013
                                                                            */
inline void SetN2kUtilityPhaseAPower(tN2kMsg& N2kMsg, double RealPower,
                                     double ApparentPower) {
  SetN2kPGN65013(N2kMsg, RealPower, ApparentPower);
}

/* \param N2kMsg              Reference to a N2kMsg Object,
 *                            Output: NMEA2000 message ready to be send.
 * \param InverterInstance     BatteryInstance.
 * \param ACInstance       BatteryInstance.
 * \param BatteryInstance     BatteryInstance.
 * \param OperatingState      Battery voltage in V
 * \param InverterEnabled      Current in A
 */
void SetN2kPGN65014(tN2kMsg& N2kMsg, double LineLineACRmsVoltage,
                    double LineNeutralACRmsVoltage, double ACFrequency,
                    double ACRmsCurrent) {
  N2kMsg.SetPGN(65014L);
  N2kMsg.Priority = 6;
  N2kMsg.Add2ByteUDouble(LineLineACRmsVoltage, 1);
  N2kMsg.Add2ByteUDouble(LineNeutralACRmsVoltage, 1);
  N2kMsg.Add2ByteUDouble(ACFrequency, 0.0078125);
  N2kMsg.Add2ByteUDouble(ACRmsCurrent, 1);
}

/************************************************************************/ /**
                                                                            * \brief
                                                                            * Setting
                                                                            * up
                                                                            * Message
                                                                            * "Utility
                                                                            * Phase
                                                                            * A
                                                                            * Basic
                                                                            * AC
                                                                            * Quantities"
                                                                            * -
                                                                            * PGN
                                                                            * 65014
                                                                            * \ingroup
                                                                            * group_msgSetUp
                                                                            *
                                                                            * Alias
                                                                            * of
                                                                            * PGN
                                                                            * 65014.
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
inline void SetN2kUtilityPhaseABasicACQuantities(tN2kMsg& N2kMsg,
                                                 double LineLineACRmsVoltage,
                                                 double LineNeutralACRmsVoltage,
                                                 double ACFrequency,
                                                 double ACRmsCurrent) {
  SetN2kPGN65014(N2kMsg, LineLineACRmsVoltage, LineNeutralACRmsVoltage,
                 ACFrequency, ACRmsCurrent);
}

class N2kUtilityPhaseASender
    : public sensesp::FileSystemSaveable {  // 65013 + 65014
 public:
  N2kUtilityPhaseASender(String config_path, int deviceIndex,
                         tNMEA2000* nmea2000)
      : sensesp::FileSystemSaveable{config_path},
        deviceIndex_{deviceIndex},
        nmea2000_{nmea2000},
        repeat_interval_{1000},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{30000}           // In ms. When the inputs expire.
  {
    this->load();
    this->initialize_members(repeat_interval_, expiry_);
    if (this->enabled_)
      sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
        tN2kMsg N2kMsg, N2kMsg2;
        // At the moment, the PGN is sent regardless of whether all the values
        // are invalid or not.
        SetN2kUtilityPhaseAPower(N2kMsg, this->RealPower->get(),
                                 this->ApparentPower->get());
        this->nmea2000_->SendMsg(N2kMsg, deviceIndex_);

        SetN2kUtilityPhaseABasicACQuantities(
            N2kMsg2, this->LineLineACRmsVoltage->get(),
            this->LineNeutralACRmsVoltage->get(), this->ACFrequency->get(),
            this->ACRmsCurrent->get());
        this->nmea2000_->SendMsg(N2kMsg2, deviceIndex_);
      });
  }

  virtual bool from_json(const JsonObject& config) override {
    if (!config["enabled"].is<bool>()) {
      return false;
    }
    enabled_ = config["enabled"];

    return true;
  }

  virtual bool to_json(JsonObject& config) override {
    config["enabled"] = enabled_;
    return true;
  }

  std::shared_ptr<sensesp::RepeatExpiring<double>> RealPower;
  std::shared_ptr<sensesp::RepeatExpiring<double>> ApparentPower;
  std::shared_ptr<sensesp::RepeatExpiring<double>> LineLineACRmsVoltage;
  std::shared_ptr<sensesp::RepeatExpiring<double>> LineNeutralACRmsVoltage;
  std::shared_ptr<sensesp::RepeatExpiring<double>> ACFrequency;
  std::shared_ptr<sensesp::RepeatExpiring<double>> ACRmsCurrent;

 protected:
  bool enabled_ = false;
  int deviceIndex_;
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

 private:
  void initialize_members(unsigned int repeat_interval, unsigned int expiry) {
    // Initialize the RepeatExpiring objects
    RealPower = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
    ApparentPower = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
    LineLineACRmsVoltage = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
    LineNeutralACRmsVoltage = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
    ACFrequency = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
    ACRmsCurrent = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
  };
};

const String ConfigSchema(const N2kUtilityPhaseASender& obj) {
  return R"###({
       "type": "object",
       "properties": {
         "enabled": { "title": "enabled", "type": "bool", "description": "enable sending" }
       }
     })###";
}

}  // namespace halmet

// 127503
// 127504

/*https://github.com/canboat/canboat/blob/master/analyzer/pgn.h*/