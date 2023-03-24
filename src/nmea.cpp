#include <Arduino.h>
#include <Preferences.h>

#include <NMEA2000_CAN.h> // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include "driver/twai.h"

Preferences preferences; // Nonvolatile storage on ESP32 - To store LastDeviceAddress
int NodeAddress;         // To store last Node Address
// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {127505L, // Fluid Level
                                                  130312L, // Temperature
                                                  127488L, // Engine Rapid / RPM
                                                  // 127508L, // Battery Status
                                                  0};

// Send time offsets
#define TempSendOffset 0
#define Temp2SendOffset 40
#define TankSendOffset 80
#define RPM_SendOffset 100
#define BatterySendOffset 120

#define SlowDataUpdatePeriod 1000 // Time between CAN Messages sent
#define FastDataUpdatePeriod 400

const tNMEA2000::tProductInformation ProductInformation PROGMEM = {
    2100,                    // N2kVersion
    100,                     // Manufacturer's product code
    "ESP32 monitor",         // Manufacturer's Model ID
    "1.2.0.16 (2022-10-01)", // Manufacturer's Software version code
    "1.2.0.0 (2022-10-01)",  // Manufacturer's Model version
    "00000001",              // Manufacturer's Model serial code
    0,                       // SertificationLevel
    1                        // LoadEquivalency
};

const char ManufacturerInformation[] PROGMEM = "John Doe, john.doe@unknown.com";
const char InstallationDescription1[] PROGMEM = "Just for sample";
const char InstallationDescription2[] PROGMEM = "No real information send to bus";

// NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{

    Serial.printf("HandleNMEA2000Msg N2kMsg.PGN= %lu\n", N2kMsg.PGN);
}

void setupNMEA()
{
    uint8_t chipid[6];
    uint32_t id = 0;
    int i = 0;

    preferences.begin("nvs", false);                         // Open nonvolatile storage (nvs)
    NodeAddress = preferences.getInt("LastNodeAddress", 32); // Read stored last NodeAddress, default 32
    preferences.end();

    // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
    NMEA2000.SetN2kCANMsgBufSize(8);
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);
    NMEA2000.SetN2kCANSendFrameBufSize(250);

    // Set Product information
    NMEA2000.SetProductInformation(&ProductInformation);

    // Set Configuration information
    NMEA2000.SetProgmemConfigurationInformation(ManufacturerInformation, InstallationDescription1, InstallationDescription2);

    // Set device information
    NMEA2000.SetDeviceInformation(123, // Unique number. Use e.g. Serial number.
                                  132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  25,  // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
    );

    NMEA2000.SetForwardStream(&Serial);
    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);
    // NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega

    NMEA2000.ExtendTransmitMessages(TransmitMessages);
    NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

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
    static unsigned long FastDataUpdated = InitNextUpdate(FastDataUpdatePeriod, BatterySendOffset);
    tN2kMsg N2kMsg;

    if (IsTimeToUpdate(FastDataUpdated))
    {
        SetNextUpdate(FastDataUpdated, FastDataUpdatePeriod);

        // Serial.printf("Voltage     : %3.0f ", BatteryVoltage);
        // Serial.println("V");

        SetN2kDCBatStatus(N2kMsg, 0, BatteryVoltage, N2kDoubleNA, N2kDoubleNA, 1);
        NMEA2000.SendMsg(N2kMsg);
    }
}

void SendN2kTankLevel(double level, double capacity)
{
    static unsigned long FastDataUpdated = InitNextUpdate(FastDataUpdatePeriod, TankSendOffset);
    tN2kMsg N2kMsg;

    if (IsTimeToUpdate(FastDataUpdated))
    {
        SetNextUpdate(FastDataUpdated, FastDataUpdatePeriod);

        // Serial.printf("Fuel Level  : %3.0f ", level);
        // Serial.println("%");

        SetN2kFluidLevel(N2kMsg, 0, N2kft_Fuel, level, capacity);
        NMEA2000.SendMsg(N2kMsg);
    }
}

void SendN2kInternalTemp(double temp)
{
    static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, Temp2SendOffset);
    tN2kMsg N2kMsg;

    if (IsTimeToUpdate(SlowDataUpdated))
    {
        SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

        // Serial.printf("Internal Temp: %3.0f °C \n", temp);

        SetN2kTemperature(N2kMsg, 0, 0, N2kts_HeatIndexTemperature, CToKelvin(temp), N2kDoubleNA);

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

        // Serial.printf("Exhaust Temp: %3.0f °C \n", temp);

        SetN2kTemperature(N2kMsg, 0, 0, N2kts_EngineRoomTemperature, CToKelvin(temp), N2kDoubleNA);
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

        // Serial.printf("Engine RPM  :%4.0f RPM \n", RPM);

        SetN2kEngineParamRapid(N2kMsg, 0, RPM, N2kDoubleNA, N2kInt8NA);

        NMEA2000.SendMsg(N2kMsg);
    }
}

void updateNMEAdress()
{
    int SourceAddress = NMEA2000.GetN2kSource();
    // Save potentially changed Source Address to NVS memory
    if (SourceAddress != NodeAddress)
    {
        NodeAddress = SourceAddress; // Set new Node Address (to save only once)
        preferences.begin("nvs", false);
        preferences.putInt("LastNodeAddress", SourceAddress);
        preferences.end();
        Serial.printf("Address Change: New Address=%d\n", SourceAddress);
    }
}

void checkNmeaErrors(void *parameter)
{
    while (true)
    {
        uint32_t alerts;
        ESP_ERROR_CHECK(twai_read_alerts(&alerts, portMAX_DELAY));
        if (alerts)
        {

            if (alerts & TWAI_ALERT_RX_DATA || alerts & TWAI_ALERT_TX_IDLE || alerts & TWAI_ALERT_TX_SUCCESS)
            {
                continue;
            }

            twai_status_info_t twai_status;
            twai_get_status_info(&twai_status);

            Serial.printf("TWAI ALERT: --->>>>\n");
            Serial.printf("TWAI Status: %i\n", twai_status.state);
            Serial.printf("TWAI Messages to Receive: %li\n", twai_status.msgs_to_rx);
            Serial.printf("TWAI Messages to Send: %li\n", twai_status.msgs_to_tx);
            Serial.printf("TWAI Messages Receive Errors: %li\n", twai_status.rx_error_counter);
            Serial.printf("TWAI Messages Receive Missed: %li\n", twai_status.rx_missed_count);
            Serial.printf("TWAI Messages Bus errors: %li\n", twai_status.bus_error_count);
            Serial.printf("TWAI Messages ARB Lost: %li\n", twai_status.arb_lost_count);

            if (alerts & TWAI_ALERT_BUS_OFF)
            {
                Serial.println("ERROR: Bus Off state");
                // Prepare to initiate bus recovery, reconfigure alerts to detect bus recovery completion
                twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);
                twai_initiate_recovery(); // Needs 128 occurrences of bus free signal
                Serial.println("ERROR: Initiate bus recovery");
            }

            if (alerts & TWAI_ALERT_BUS_RECOVERED)
            {
                // Bus recovery was successful,
                Serial.println("ERROR: Bus Recovered");
                // Start TWAI driver
                ESP_ERROR_CHECK(twai_start());
                Serial.println("TWAI Driver started");
                ESP_ERROR_CHECK(twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL));
            }

            if (alerts & TWAI_ALERT_TX_IDLE)
            {
                Serial.println("Alert: No more messages to transmit");
            }

            if (alerts & TWAI_ALERT_TX_SUCCESS)
            {
                Serial.println("Alert: The previous transmission was successful");
            }

            if (alerts & TWAI_ALERT_RX_DATA)
            {
                Serial.println("Alert: A frame has been received and added to the RX queue");
            }

            if (alerts & TWAI_ALERT_BELOW_ERR_WARN)
            {
                Serial.println("Alert: Both error counters have dropped below error warning limit");
            }

            if (alerts & TWAI_ALERT_ERR_ACTIVE)
            {
                Serial.println("Alert: TWAI controller has become error active");
            }

            if (alerts & TWAI_ALERT_ARB_LOST)
            {
                Serial.println("Alert: The previous transmission lost arbitration");
            }

            if (alerts & TWAI_ALERT_ABOVE_ERR_WARN)
            {
                Serial.println("Alert: One of the error counters have exceeded the error warning limit");
            }

            if (alerts & TWAI_ALERT_BUS_ERROR)
            {
                Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus");
            }

            if (alerts & TWAI_ALERT_ERR_PASS)
            {
                Serial.println("Alert: TWAI controller has become error passive");
            }

            if (alerts & TWAI_ALERT_TX_FAILED)
            {
                Serial.println("Alert: The previous transmission has failed (for single shot transmission)");
            }

            if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
            {
                Serial.println("Alert: The RX queue is full causing a frame to be lost");
            }

            if (alerts & TWAI_ALERT_RX_FIFO_OVERRUN)
            {
                Serial.println("Alert: An RX FIFO overrun has occurred");
            }

            if (alerts & TWAI_ALERT_TX_RETRIED)
            {
                Serial.println("Alert: An message transmission was cancelled and retried due to an errata workaround");
            }

            if (alerts & TWAI_ALERT_PERIPH_RESET)
            {
                Serial.println("Alert: The TWAI controller was reset");
            }

            vTaskDelay(10);
        }
    }
}

void receiveNmeaMessage(void *parameter)
{
    while (true)
    {
        // ESP32_CAN_read_frame();
    }
}

void loopNMEA()
{
    updateNMEAdress();
    NMEA2000.ParseMessages();
}
