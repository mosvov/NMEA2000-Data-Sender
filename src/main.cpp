#include <Arduino.h>
#define ESP32_CAN_TX_PIN GPIO_NUM_5 // Set CAN TX port
#define ESP32_CAN_RX_PIN GPIO_NUM_4 // Set CAN RX port

#include <nmea.h>
#include <math.h>
#include <Wire.h>

#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/startable.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/typecast.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp_minimal_app_builder.h"

using namespace sensesp;

#define SENSE_ALT_PIN GPIO_NUM_33         // Engine RPM is measured as interrupt
#define SENSE_DS18B20_PIN = GPIO_NUM_25;  // Data wire for teperature (Dallas DS18B20)
#define SENSE_FUEL_T_PIN = GPIO_NUM_34;   // Tank fluid level measure
#define SENSE_COOLANT_PIN = GPIO_NUM_32;  // Coolant temperature sender
#define SENSE_V_ANALOG_PIN = GPIO_NUM_35; // Analog voltage
#define BUZZER_PIN = GPIO_NUM_15;         // BUZZER

const unsigned int read_delay = 500;

const float kDefaultFrequencyScale = 1. / 97;

FloatProducer *ConnectTachoSender(int pin, String name)
{
    char config_path[80];
    char sk_path[80];

    snprintf(config_path, sizeof(config_path), "", name.c_str());
    auto tacho_input = new DigitalInputCounter(pin, INPUT, RISING, 500, config_path);

    snprintf(config_path, sizeof(config_path), "/Tacho %s/Revolution Multiplier", name.c_str());
    auto tacho_frequency = new Frequency(kDefaultFrequencyScale, config_path);

    snprintf(config_path, sizeof(config_path), "/Tacho %s/Revolutions SK Path", name.c_str());
    snprintf(sk_path, sizeof(sk_path), "propulsion.%s.revolutions", name.c_str());
    auto tacho_frequency_sk_output = new SKOutputFloat(sk_path, config_path);

    tacho_input
        ->connect_to(tacho_frequency)
        ->connect_to(tacho_frequency_sk_output);

    tacho_input->attach([name, tacho_input]()
                        { debugD("Input %s counter: %d", name.c_str(), tacho_input->get()); });

    return tacho_frequency;
}

// I2C pins on SH-ESP32
const int kSDAPin = 21;
const int kSCLPin = 22;

TwoWire *i2c;
#define WIRE Wire

// Convenience function to print the addresses found on the I2C bus
void ScanI2C(TwoWire *i2c)
{
    uint8_t error, address;

    Serial.println("Scanning...");

    for (address = 1; address < 127; address++)
    {
        i2c->beginTransmission(address);
        error = i2c->endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("");
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
}

ReactESP app;

void setup()
{
#ifndef SERIAL_DEBUG_DISABLED
    SetupSerialDebug(115200);
#endif

    // Create a unique hostname for the device.
    uint8_t mac[6];
    WiFi.macAddress(mac);

    String hostname = "NMEA2000-Data-Sender";

    SensESPMinimalAppBuilder builder;
    auto sensesp_app = builder.set_hostname(hostname)->get_app();

    // manually create Networking and HTTPServer objects to enable the HTTP configuration interface
    auto *networking = new Networking("/system/net", "Pixel", "123456789",
                                      SensESPBaseApp::get_hostname(),
                                      "thisisfine");
    auto *http_server = new HTTPServer();

    setupNMEA();

    // No need to parse the messages at every single loop iteration; 1 ms will do
    app.onRepeat(1, []()
                 { loopNMEA(); });

    // initialize the I2C bus
    i2c = new TwoWire(0);
    i2c->begin(kSDAPin, kSCLPin);

    ScanI2C(i2c);

    // DallasTemperatureSensors *dts = new DallasTemperatureSensors(ONEWIRE_PIN);

    // Connect the tank senders
    // auto tank_a_volume = ConnectTankSender(ads1115, 0, "A");

    // Connect the tacho senders
    // auto tacho_1_frequency = ConnectTachoSender(Eingine_RPM_Pin, "1");

    // define three 1-Wire temperature sensors that update every 1000 ms
    // and have specific web UI configuration paths
    // auto main_engine_oil_temperature =
    //    new OneWireTemperature(dts, 1000, "/mainEngineOilTemp/oneWire");

    // main_engine_oil_temperature->connect_to(
    //     new LambdaConsumer<float>([](float temperature)
    //                               {
    //                                   debugD("temperature %d ", temperature);
    //   tN2kMsg N2kMsg;
    //   SetN2kTemperature(N2kMsg,
    //                     1,                            // SID
    //                     2,                            // TempInstance
    //                     N2kts_ExhaustGasTemperature,  // TempSource
    //                     temperature                   // actual temperature
    //);
    //   nmea2000->SendMsg(N2kMsg);
    //                              }));

    // alarm_2_input->connect_to(
    //    new LambdaConsumer<bool>([](bool value)
    //                             { alarm_states[1] = value; }));

    // auto *digin1 = new DigitalInputCounter(input_pin1, INPUT, RISING, read_delay);
    // auto *digin2 = new DigitalInputCounter(input_pin2, INPUT, CHANGE, read_delay);

    // auto *scaled1 = new Linear(2, 1, "/digin1/scale");
    // auto *scaled2 = new Linear(4, -1, "/digin2/scale");
    // digin1->connect_to(scaled1);

    // scaled1->connect_to(new LambdaTransform<int, int>([](int input)
    //{
    //    Serial.printf("millis: %d\n", millis());
    //    Serial.printf("Counter 1: %d\n", input);
    //    return input;
    //}));

    // digin2->connect_to(scaled2)->connect_to(
    //     new LambdaTransform<int, int>([](int input)
    //{
    //    Serial.printf("Counter 2: %d\n", input);
    //    return input; }));

    sensesp_app->start();
}

// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop()
{
    app.tick();
}
