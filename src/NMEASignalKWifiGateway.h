// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <N2kMessages.h>
#include <N2kTypes.h>
#include <NMEA2000.h>

#include "sensesp/net/discovery.h"
#include "sensesp/net/networking.h"
#include "sensesp/ui/config_item.h"

using namespace sensesp;

namespace halmet {

class NMEASignalKWifiGateway : public sensesp::FileSystemSaveable {
 public:
  NMEASignalKWifiGateway(String config_path, tNMEA2000* nmea2000, String skHost,
                         bool enabled = false)
      : sensesp::FileSystemSaveable{config_path}, enabled{enabled} {
    char skHost_[100];
    skHost.toCharArray(skHost_, 100);
    this->load();

    if (this->enabled) {
      nmea2000->AttachMsgHandler(
          new MyMessageHandler(nmea2000, skHost_, &nodeAddress));
    }
  }
  virtual ~NMEASignalKWifiGateway() { this->save(); }

  virtual bool from_json(const JsonObject& config) override {
    if (!config["enabled"].is<bool>())
      return false;
    else
      enabled = config["enabled"];

    if (!config["nodeAddress"].is<int>())
      return false;
    else
      nodeAddress = config["nodeAddress"];

    return true;
  }

  virtual bool to_json(JsonObject& config) override {
    config["enabled"] = enabled;
    config["nodeAddress"] = nodeAddress;
    return true;
  }

 protected:
  class MyMessageHandler : public tNMEA2000::tMsgHandler {
   public:
    MyMessageHandler(tNMEA2000* _pNMEA2000, char* _skHost, int* _nodeAddress)
        : tNMEA2000::tMsgHandler(0, _pNMEA2000),
          skHost{_skHost},
          nodeAddress{_nodeAddress} {}

   protected:
    char* skHost;
    int* nodeAddress;
    uint16_t DaysSince1970 = 0;
    double SecondsSinceMidnight = 0;

    void HandleMsg(const tN2kMsg& N2kMsg) override {
      CheckSourceAddressChange();

      WiFiUDP udp;
      const int udpPort = 4444;  // YD UDP port

#define Max_YD_Message_Size 500
      char YD_msg[Max_YD_Message_Size] = "";

      if (N2kMsg.PGN == 126992L)
        HandleGNSS(N2kMsg);  // Just to get time from GNSS
      if (N2kMsg.PGN == 129029L) HandleSytemTime(N2kMsg);  // or this way

      N2kToYD_Can(N2kMsg, YD_msg);  // Create YD message from PGN

      udp.beginPacket(this->skHost, udpPort);  // Send to UDP
      udp.println(YD_msg);
      udp.endPacket();
    }

    void CheckSourceAddressChange() {
      int SourceAddress = this->GetNMEA2000()->GetN2kSource();
      if (SourceAddress !=
          *(this->nodeAddress)) {  // Save potentially changed Source Address to
                                   // NVS memory
        *(this->nodeAddress) =
            SourceAddress;  // Set new Node Address (to save only once)
        Serial.printf("Address Change: New Address=%d\n", SourceAddress);
      }
    }

    //*****************************************************************************
    void HandleSytemTime(const tN2kMsg& N2kMsg) {
      unsigned char SID;
      tN2kTimeSource TimeSource;

      ParseN2kSystemTime(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight,
                         TimeSource);
    }

    //*****************************************************************************
    void HandleGNSS(const tN2kMsg& N2kMsg) {
      unsigned char SID;
      double Latitude;
      double Longitude;
      double Altitude;
      tN2kGNSStype GNSStype;
      tN2kGNSSmethod GNSSmethod;
      unsigned char nSatellites;
      double HDOP;
      double PDOP;
      double GeoidalSeparation;
      unsigned char nReferenceStations;
      tN2kGNSStype ReferenceStationType;
      uint16_t ReferenceSationID;
      double AgeOfCorrection;

      if (ParseN2kGNSS(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight,
                       Latitude, Longitude, Altitude, GNSStype, GNSSmethod,
                       nSatellites, HDOP, PDOP, GeoidalSeparation,
                       nReferenceStations, ReferenceStationType,
                       ReferenceSationID, AgeOfCorrection)) {
      }
    }

    // Example Output: 16:29:27.082 R 09F8017F 50 C3 B8 13 47 D8 2B C6
    //*****************************************************************************
    void N2kToYD_Can(const tN2kMsg& msg, char* MsgBuf) {
      int i, len;
      uint32_t canId = 0;
      char time_str[20];
      char Byte[5];
      unsigned int PF;
      time_t rawtime;
      struct tm ts;

      len = msg.DataLen;
      if (len > 134) len = 134;

      // Set CanID

      canId = msg.Source & 0xff;
      PF = (msg.PGN >> 8) & 0xff;

      if (PF < 240) {
        canId = (canId | ((msg.Destination & 0xff) << 8));
        canId = (canId | (msg.PGN << 8));
      } else {
        canId = (canId | (msg.PGN << 8));
      }

      canId = (canId | (msg.Priority << 26));

      rawtime = (DaysSince1970 * 3600 * 24) +
                SecondsSinceMidnight;  // Create time from GNSS time;
      ts = *localtime(&rawtime);
      strftime(time_str, sizeof(time_str), "%T.000",
               &ts);  // Create time string

      snprintf(MsgBuf, 25, "%s R %0.8x", time_str,
               canId);  // Set time and canID

      for (i = 0; i < len; i++) {
        snprintf(Byte, 4, " %0.2x", msg.Data[i]);  // Add data fields
        strcat(MsgBuf, Byte);
      }
    }
  };

  bool enabled;
  int nodeAddress;
};

const String ConfigSchema(const NMEASignalKWifiGateway& obj) {
  return R"###({
      "type": "object",
      "properties": {
        "enabled": { "title": "enabled", "type": "bool", "description": "enable Gateway" },
        "nodeAddress": { "title": "nodeAddress", "type": "int", "description": "LastNodeAddress for NMEA" }
      }
    })###";
}
}  // namespace halmet
