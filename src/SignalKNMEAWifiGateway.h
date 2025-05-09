// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <NMEA2000_esp32.h>

#include "n2k_senders.h"
#include "sensesp/net/discovery.h"
#include "sensesp/net/networking.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/ui/config_item.h"

using namespace sensesp;
using namespace halmet;

namespace halmet {

class SignalKNMEAWifiGateway : public sensesp::FileSystemSaveable {
 public:
  SignalKNMEAWifiGateway(String config_path, tNMEA2000* nmea2000,
                         bool enabled = false, int ACInputDev = 0,
                         int InverterDev = 0,
                         String StartBattName = "279-second",
                         String HouseBattName = "279",
                         String HouseBattChargerName = "276")
      : sensesp::FileSystemSaveable{config_path},
        enabled{enabled},
        startBattName{StartBattName},
        houseBattName{HouseBattName},
        houseBattChargerName{HouseBattChargerName},
        nmea2000_{nmea2000},
        repeat_interval_{1000},  // In ms. Dictated by NMEA 2000 standard!
        expiry_{30000}           // In ms. When the inputs expire.
  {
    this->load();
    this->initialize_members(repeat_interval_, expiry_);

    if (this->enabled) {
      auto* startBatSender = new N2kDCBatStatusSender(
          this->config_path_ + "/Start Batt Status", 0, nmea2000);

      ConfigItem(startBatSender)
          ->set_title("Start Batt Status")
          ->set_description(
              "NMEA 2000 dynamic engine parameters for start Batt")
          ->set_sort_order(3010);

      auto* startBattVoltage = new FloatSKListener(
          "electrical.batteries." + startBattName + ".voltage", 1000,
          "/SKPath/StartBatt/Voltage");
      startBattVoltage->connect_to(startBatSender->batteryVoltage_);

      // House Batt
      auto* houseBatSender = new N2kDCBatStatusSender(
          this->config_path_ + "/House Batt Status", 1, nmea2000);

      ConfigItem(houseBatSender)
          ->set_title("House Batt Status")
          ->set_description(
              "NMEA 2000 dynamic engine parameters for House Batt")
          ->set_sort_order(3010);

      (new FloatSKListener("electrical.batteries." + houseBattName +
                           ".voltage"))
          ->connect_to(houseBatSender->batteryVoltage_);
      (new FloatSKListener("electrical.batteries." + houseBattName +
                           ".current"))
          ->connect_to(houseBatSender->batteryCurrent_);
      (new FloatSKListener("electrical.batteries." + houseBattName +
                           ".temperature"))
          ->connect_to(houseBatSender->batteryTemperature_);

      auto* houseBatDetailSender = new N2kDCStatusSender(
          this->config_path_ + "/House Batt Detail", 1, nmea2000);

      ConfigItem(houseBatDetailSender)
          ->set_title("House Batt Detail")
          ->set_description(
              "NMEA 2000 DC Detailed Status parameters for House Batt")
          ->set_sort_order(3020);

      (new FloatSKListener("electrical.batteries." + houseBattName +
                           ".capacity.stateOfCharge"))
          ->connect_to(houseBatDetailSender->stateOfCharge);
      (new FloatSKListener("electrical.batteries." + houseBattName +
                           ".capacity.timeRemaining"))
          ->connect_to(houseBatDetailSender->timeRemaining);
      (new FloatSKListener("electrical.batteries." + houseBattName +
                           ".temperature"))
          ->connect_to(houseBatDetailSender->capacity);

      // TODO: Change 12507 to 127750 maybe
      auto* houseBatChargerSender = new N2kChargerSender(
          this->config_path_ + "/House Batt Charger", 1, 1, nmea2000);

      ConfigItem(houseBatChargerSender)
          ->set_title("House Charger Detail")
          ->set_description(
              "NMEA 2000 Charger Status parameters for House Batt")
          ->set_sort_order(3020);

      (new IntSKListener("electrical.batteries." + houseBattName +
                         ".chargingModeNumber"))
          ->connect_to(new LambdaTransform<int, tN2kChargeState>(
              [](int s) -> tN2kChargeState {
                switch (s) {
                  case 0:
                    return N2kCS_Not_Charging;
                  case 1:
                    return N2kCS_Fault;
                  case 2:
                    return N2kCS_Fault;
                  case 3:
                    return N2kCS_Bulk;
                  case 4:
                    return N2kCS_Absorption;
                  case 5:
                    return N2kCS_Float;
                  case 6:
                    return N2kCS_Float;
                  case 7:
                    return N2kCS_Equalise;
                  case 8:
                    return N2kCS_Disabled;
                  case 9:
                    return N2kCS_Not_Charging;
                  case 10:
                    return N2kCS_Not_Charging;
                  case 11:
                    return N2kCS_Constant_VI;
                  default:
                    return N2kCS_Not_Charging;
                }
              }))
          ->connect_to(houseBatChargerSender->chargeState);
      (new ValueProducer<tN2kChargerMode>(N2kCM_Standalone))
          ->connect_to(houseBatChargerSender->chargerMode);
      (new FloatSKListener("electrical.batteries." + houseBattName +
                           ".modeNumber"))
          ->connect_to(
              new LambdaTransform<int, tN2kOnOff>([](int s) -> tN2kOnOff {
                if (s == 0)
                  return N2kOnOff_Off;
                else
                  return N2kOnOff_On;
              }))
          ->connect_to(houseBatChargerSender->enabled);

      auto* inverterSender = new N2kInverterSender(
          this->config_path_ + "/Inverter", 1, 1, 1, nmea2000);

      ConfigItem(inverterSender)
          ->set_title("Inverter Detail")
          ->set_description("NMEA 2000 Charger Status parameters for Inverter")
          ->set_sort_order(3020);

      (new IntSKListener("electrical.inverters." + HouseBattChargerName +
                         ".inverterModeNumber"))
          ->connect_to(new LambdaTransform<int, tN2kInverterOperatingState>(
              [](int s) -> tN2kInverterOperatingState {
                switch (s) {
                  case 0:
                    return tN2kInverterOperatingState_Disabled;
                  case 1:
                  case 2:
                    return tN2kInverterOperatingState_Fault;
                  case 8:
                    return tN2kInverterOperatingState_AC_Passthru;
                  case 9:
                    return tN2kInverterOperatingState_Invert;
                  case 10:
                    return tN2kInverterOperatingState_Load_Sense;
                  default:
                    return tN2kInverterOperatingState_Disabled;
                }
              }))
          ->connect_to(inverterSender->operatingState);
      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".inverterModeNumber"))
          ->connect_to(
              new LambdaTransform<int, tN2kOnOff>([](int s) -> tN2kOnOff {
                if (s == 0)
                  return N2kOnOff_Off;
                else
                  return N2kOnOff_On;
              }))
          ->connect_to(inverterSender->inverterEnabled);

      auto* inverterInputSender = new N2kUtilityPhaseASender(
          this->config_path_ + "/Inverter", 1, nmea2000);
      ConfigItem(inverterInputSender)
          ->set_title("Inverter Detail")
          ->set_description("NMEA 2000 Charger Status parameters for Inverter")
          ->set_sort_order(3020);

      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".acin.power"))
          ->connect_to(inverterInputSender->RealPower);
      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".acin.power"))
          ->connect_to(inverterInputSender->ApparentPower);
      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".acin.frequency"))
          ->connect_to(inverterInputSender->ACFrequency);
      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".acin.current"))
          ->connect_to(inverterInputSender->ACRmsCurrent);
      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".acin.voltage"))
          ->connect_to(inverterInputSender->LineNeutralACRmsVoltage);

      auto* inverterOutputSender = new N2kUtilityPhaseASender(
          this->config_path_ + "/Inverter", 2, nmea2000);
      ConfigItem(inverterOutputSender)
          ->set_title("Inverter Detail")
          ->set_description("NMEA 2000 Charger Status parameters for Inverter")
          ->set_sort_order(3020);

      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".acout.power"))
          ->connect_to(inverterOutputSender->RealPower);
      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".acout.power"))
          ->connect_to(inverterOutputSender->ApparentPower);
      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".acout.frequency"))
          ->connect_to(inverterOutputSender->ACFrequency);
      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".acout.current"))
          ->connect_to(inverterOutputSender->ACRmsCurrent);
      (new FloatSKListener("electrical.inverters." + HouseBattChargerName +
                           ".acout.voltage"))
          ->connect_to(inverterOutputSender->LineNeutralACRmsVoltage);

      auto* dcBusSender = new n2k_DCVoltageCurrentSender(
          this->config_path_ + "/DC", 3, nmea2000);
      ConfigItem(dcBusSender)
          ->set_title("DC Bus Detail")
          ->set_description(
              "NMEA 2000 Charger Status parameters for House Batt")
          ->set_sort_order(3020);

      (new FloatSKListener("electrical.batteries." + houseBattName +
                           ".voltage"))
          ->connect_to(dcBusSender->DcVoltage);
      (new FloatSKListener(
           "electrical.venus.dcPower"))  
          ->connect_to(dcBusSender->DcPower);
    }
  }

  virtual bool from_json(const JsonObject& config) override {
    if (!config["enabled"].is<bool>())
      return false;
    else
      enabled = config["enabled"];

    if (!config["startBattName"].is<String>())
      return false;
    else
      startBattName = config["startBattName"].as<String>();

    if (!config["houseBattName"].is<String>())
      return false;
    else
      houseBattName = config["houseBattName"].as<String>();

    if (!config["houseBattChargerName"].is<String>())
      return false;
    else
      houseBattChargerName = config["houseBattChargerName"].as<String>();

    return true;
  }

  virtual bool to_json(JsonObject& config) override {
    config["enabled"] = enabled;
    config["startBattName"] = startBattName;
    config["houseBattName"] = houseBattName;
    config["houseBattChargerName"] = houseBattChargerName;
    return true;
  }

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  bool enabled;
  String startBattName;
  String houseBattName;
  String houseBattChargerName;

 private:
  void initialize_members(unsigned int repeat_interval, unsigned int expiry) {
    // Initialize the RepeatExpiring objects
  }
};

const String ConfigSchema(const SignalKNMEAWifiGateway& obj) {
  return R"###({
      "type": "object",
      "properties": {
        "enabled": { "title": "enabled", "type": "bool", "description": "enable Gateway" },
        "startBattName": { "title": "startBattName", "type": "String", "description": "" },
        "houseBattName": { "title": "houseBattName", "type": "String", "description": "" },
        "houseBattChargerName": { "title": "houseBattChargerName", "type": "String", "description": "" }
      }
    })###";
}
}  // namespace halmet

// Not Converted
#pragma region
// 04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.current
// 5
// A	04/21 11:19:08	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.leds.absorption
// 0
// 04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.leds.bulk
// 0
// 04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.leds.float
// 1
// 04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.leds.inverter
// 0
// 04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.leds.lowBattery
// 0
// 04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.leds.mains
// 1
// 04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.leds.overload
// 0
// 04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.leds.temperature
// 0
// electrical.chargers.276.power
// 80
// W	04/21 11:19:05	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.temperature
// 299.15
// K	04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.chargers.276.voltage
// 13.170000076293945
// V	04/21 11:19:05	venus.com.victronenergy.vebus.ttyS4
// 04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.inverters.276.acState.ignoreAcIn1.state
// 0
// electrical.inverters.276.acState.acIn1Available
// 1
// 04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.inverters.276.name
// "Inverter"
// 04/21 11:19:03	venus.com.victronenergy.system
// electrical.venus.0.acin.acSource
// "grid"
// 04/21 11:19:03	venus.com.victronenergy.system
// electrical.venus.0.acin.acSourceNumber
// 1
// 04/21 11:19:03	venus.com.victronenergy.system
// electrical.venus.acSource
// "grid"
// 04/21 11:19:03	venus.com.victronenergy.system
// electrical.venus.acSourceNumber
// 1
#pragma endregion
// A	04/21 11:19:07	venus.com.victronenergy.vebus.ttyS4
// electrical.inverters.276.acin.currentLimit
// 16.5
// V	04/21 11:19:03	venus.com.victronenergy.vebus.ttyS4
// electrical.switches.venus-0.state
// 0
// 04/21 11:19:03	venus.com.victronenergy.system
// electrical.switches.venus-1.state
// 0
