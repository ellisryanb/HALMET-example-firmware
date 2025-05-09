#include <N2kMessages.h>
#include <NMEA2000.h>

#include "sensesp/system/saveable.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/repeat.h"
#include "sensesp_base_app.h"

namespace halmet {
void SetN2kPGN127751(tN2kMsg& N2kMsg, unsigned char SID,
                     unsigned char ConnectionNumber, double DcVoltage,
                     double DcCurrent) {
  N2kMsg.SetPGN(127751L);
  N2kMsg.Priority = 6;
  N2kMsg.AddByte(SID);
  N2kMsg.AddByte(ConnectionNumber);
  N2kMsg.Add2ByteUDouble(DcVoltage, 0.1);
  N2kMsg.Add3ByteUDouble(DcCurrent, 0.01);
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
                                                                            * 127751
                                                                            * \ingroup
                                                                            * group_msgSetUp
                                                                            *
                                                                            * Alias
                                                                            * of
                                                                            * PGN
                                                                            * 127751.
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
inline void SetN2kDCVoltageCurrentStatus(tN2kMsg& N2kMsg, unsigned char SID,
                                         unsigned char ConnectionNumber,
                                         double DcVoltage, double DcCurrent) {
  SetN2kPGN127751(N2kMsg, SID, ConnectionNumber, DcVoltage, DcCurrent);
}

class n2k_DCVoltageCurrentSender
    : public sensesp::FileSystemSaveable {  // 127506
 public:
  n2k_DCVoltageCurrentSender(String config_path, uint8_t connection_number,
                             tNMEA2000* nmea2000)
      : sensesp::FileSystemSaveable{config_path},
        connection_number_{connection_number},
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
        SetN2kDCVoltageCurrentStatus(
            N2kMsg, 0, this->connection_number_, this->DcVoltage->get(),
            this->DcCurrent->get() > 0
                ? (double)this->DcCurrent->get()
                : this->DcPower->get() / this->DcVoltage->get());
        this->nmea2000_->SendMsg(N2kMsg);
      });
  }

  virtual bool from_json(const JsonObject& config) override {
    if (!config["enabled"].is<bool>()) {
      return false;
    }
    enabled_ = config["enabled"];

    if (!config["connection_number"].is<int>()) {
      return false;
    }
    connection_number_ = config["connection_number"];
    return true;
  }

  virtual bool to_json(JsonObject& config) override {
    config["enabled"] = enabled_;
    config["connection_number"] = connection_number_;
    return true;
  }

  std::shared_ptr<sensesp::RepeatExpiring<double>> DcVoltage;
  std::shared_ptr<sensesp::RepeatExpiring<double>> DcCurrent;
  std::shared_ptr<sensesp::RepeatExpiring<double>> DcPower;

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  bool enabled_ = false;
  uint8_t connection_number_ = 0;

 private:
  void initialize_members(unsigned int repeat_interval, unsigned int expiry) {
    // Initialize the RepeatExpiring objects
    DcVoltage = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
    DcCurrent = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry);
    DcPower = std::make_shared<sensesp::RepeatExpiring<double>>(repeat_interval,
                                                                expiry);
  }
};

const String ConfigSchema(const n2k_DCVoltageCurrentSender& obj) {
  return R"###({
     "type": "object",
     "properties": {
         "enabled": { "title": "enabled", "type": "bool", "description": "enable sending" },
       "connection_number": { "title": "connection_number", "type": "integer", "description": "battery NMEA 2000 instance number (0-253)" }
     }
   })###";
}

}  // namespace halmet
