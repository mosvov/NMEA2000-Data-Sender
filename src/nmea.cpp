#include <Arduino.h>
#include <Preferences.h>
#include <NMEA2000_CAN.h> // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>

Preferences preferences; // Nonvolatile storage on ESP32 - To store LastDeviceAddress
int NodeAddress;         // To store last Node Address
// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {127505L, // Fluid Level
                                                  130311L, // Temperature  (or alternatively 130312L or 130316L)
                                                  127488L, // Engine Rapid / RPM
                                                  127508L, // Battery Status
                                                  0};

// Send time offsets
#define TempSendOffset 0
#define TankSendOffset 40
#define RPM_SendOffset 80
#define BatterySendOffset 100

#define SlowDataUpdatePeriod 1000 // Time between CAN Messages sent

// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
tN2kSyncScheduler DCBatStatusScheduler(false, 1500, 500);
tN2kSyncScheduler DCStatusScheduler(false, 1500, 510);
tN2kSyncScheduler BatConfScheduler(false, 5000, 520); // Non periodic

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()

void OnN2kOpen()
{
    // Start schedulers now.
    DCBatStatusScheduler.UpdateNextTime();
    DCStatusScheduler.UpdateNextTime();
    BatConfScheduler.UpdateNextTime();
}

const tNMEA2000::tProductInformation ProductInformation PROGMEM = {
    2100,                     // N2kVersion
    100,                      // Manufacturer's product code
    "Simple battery monitor", // Manufacturer's Model ID
    "1.2.0.16 (2022-10-01)",  // Manufacturer's Software version code
    "1.2.0.0 (2022-10-01)",   // Manufacturer's Model version
    "00000001",               // Manufacturer's Model serial code
    0,                        // SertificationLevel
    1                         // LoadEquivalency
};

const char ManufacturerInformation[] PROGMEM = "John Doe, john.doe@unknown.com";
const char InstallationDescription1[] PROGMEM = "Just for sample";
const char InstallationDescription2[] PROGMEM = "No real information send to bus";

void setupNMEA()
{
    uint8_t chipid[6];
    uint32_t id = 0;
    int i = 0;

    esp_efuse_mac_get_default(chipid);
    for (i = 0; i < 6; i++)
        id += (chipid[i] << (7 * i));

    preferences.begin("nvs", false);                         // Open nonvolatile storage (nvs)
    NodeAddress = preferences.getInt("LastNodeAddress", 25); // Read stored last NodeAddress, default 25
    preferences.end();
    Serial.printf("NodeAddress=%d\n", NodeAddress);

    // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
    NMEA2000.SetN2kCANMsgBufSize(8);
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);
    NMEA2000.SetN2kCANSendFrameBufSize(250);

    // Set Product information
    NMEA2000.SetProductInformation(&ProductInformation);

    // Set Configuration information
    NMEA2000.SetProgmemConfigurationInformation(ManufacturerInformation, InstallationDescription1, InstallationDescription2);

    // Set device information
    NMEA2000.SetDeviceInformation(id,  // Unique number. Use e.g. Serial number.
                                  132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  25,  // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
    );

    NMEA2000.SetForwardStream(&Serial);
    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);
    // NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega

    NMEA2000.ExtendTransmitMessages(TransmitMessages);

    // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
    NMEA2000.SetOnOpen(OnN2kOpen);

    NMEA2000.Open();
}

bool IsTimeToUpdate(unsigned long NextUpdate)
{
    return (NextUpdate < millis());
}

unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset = 0)
{
    return millis() + Period + Offset;
}

void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period)
{
    while (NextUpdate < millis())
        NextUpdate += Period;
}

void SendN2kBattery(double BatteryVoltage)
{
    static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, BatterySendOffset);
    tN2kMsg N2kMsg;

    if (IsTimeToUpdate(SlowDataUpdated))
    {
        SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

        Serial.printf("Voltage     : %3.0f ", BatteryVoltage);
        Serial.println("%");

        SetN2kDCBatStatus(N2kMsg, 0, BatteryVoltage, N2kDoubleNA, N2kDoubleNA, 1);
        NMEA2000.SendMsg(N2kMsg);
    }
}

void SendN2kTankLevel(double level, double capacity)
{
    static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, TankSendOffset);
    tN2kMsg N2kMsg;

    if (IsTimeToUpdate(SlowDataUpdated))
    {
        SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

        Serial.printf("Fuel Level  : %3.0f ", level);
        Serial.println("%");

        SetN2kFluidLevel(N2kMsg, 0, N2kft_Fuel, level, capacity);
        NMEA2000.SendMsg(N2kMsg);
    }
}

void SendN2kExhaustTemp(double temp)
{
    static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, TempSendOffset);
    tN2kMsg N2kMsg;

    if (IsTimeToUpdate(SlowDataUpdated))
    {
        SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

        Serial.printf("Exhaust Temp: %3.0f °C \n", temp);

        // Select the right PGN for your MFD and set the PGN value also in "TransmitMessages[]"

        SetN2kEnvironmentalParameters(N2kMsg, 0, N2kts_ExhaustGasTemperature, CToKelvin(temp), // PGN130311, uncomment the PGN to be used
                                      N2khs_Undef, N2kDoubleNA, N2kDoubleNA);

        // SetN2kTemperature(N2kMsg, 0, 0, N2kts_ExhaustGasTemperature, CToKelvin(temp), N2kDoubleNA);   // PGN130312, uncomment the PGN to be used

        // SetN2kTemperatureExt(N2kMsg, 0, 0, N2kts_ExhaustGasTemperature,CToKelvin(temp), N2kDoubleNA); // PGN130316, uncomment the PGN to be used

        NMEA2000.SendMsg(N2kMsg);
    }
}

void SendN2kEngineRPM(double RPM)
{
    static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, RPM_SendOffset);
    tN2kMsg N2kMsg;

    if (IsTimeToUpdate(SlowDataUpdated))
    {
        SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

        Serial.printf("Engine RPM  :%4.0f RPM \n", RPM);

        SetN2kEngineParamRapid(N2kMsg, 0, RPM, N2kDoubleNA, N2kInt8NA);

        NMEA2000.SendMsg(N2kMsg);
    }
}

void updateNMEAdress()
{
    int SourceAddress = NMEA2000.GetN2kSource();
    if (SourceAddress != NodeAddress)
    {                                // Save potentially changed Source Address to NVS memory
        NodeAddress = SourceAddress; // Set new Node Address (to save only once)
        preferences.begin("nvs", false);
        preferences.putInt("LastNodeAddress", SourceAddress);
        preferences.end();
        Serial.printf("Address Change: New Address=%d\n", SourceAddress);
    }
}