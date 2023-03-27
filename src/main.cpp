#include <Arduino.h>
#define ESP32_CAN_TX_PIN GPIO_NUM_5 // Set CAN TX port
#define ESP32_CAN_RX_PIN GPIO_NUM_4 // Set CAN RX port

#include <nmea.h>
#include <sensors.h>

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
// #include "sensesp_onewire/onewire_temperature.h"
#include "sensesp_minimal_app_builder.h"

#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // I2C

using namespace sensesp;

#define SENSE_ALT_PIN GPIO_NUM_33      // Engine RPM is measured as interrupt
#define SENSE_DS18B20_PIN GPIO_NUM_25  // Data wire for teperature (Dallas DS18B20)
#define SENSE_FUEL_T_PIN GPIO_NUM_34   // Tank fluid level measure
#define SENSE_COOLANT_PIN GPIO_NUM_32  // Coolant temperature sender
#define SENSE_V_ANALOG_PIN GPIO_NUM_35 // Analog voltage
#define BUZZER_PIN GPIO_NUM_15         // BUZZER

const unsigned int read_delay = 500;

// I2C pins on SH-ESP32
const int kSDAPin = 21;
const int kSCLPin = 22;

TwoWire *i2c;
#define WIRE Wire

ReactESP app;

#define SEALEVELPRESSURE_HPA (1019.8)

void printValues()
{
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

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

    // Init BME280
    if (!bme.begin(0x76, i2c))
    {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    }

    pinMode(BUZZER_PIN, OUTPUT);
    // digitalWrite(BUZZER_PIN, HIGH);

    // DallasTemperatureSensors *dts = new DallasTemperatureSensors(ONEWIRE_PIN);

    // Connect the tank senders
    auto tank_a_volume = ConnectTankSender(SENSE_FUEL_T_PIN, "Fuel");

    // Connect the tacho senders
    auto tacho_1_frequency = ConnectTachoSender(SENSE_ALT_PIN, "1");

    // define three 1-Wire temperature sensors that update every 1000 ms
    // and have specific web UI configuration paths
    // auto main_engine_oil_temperature =
    //    new OneWireTemperature(dts, 1000, "/mainEngineOilTemp/oneWire");

    tank_a_volume->connect_to(
        new LambdaConsumer<float>([](float value)
                                  { debugD("tank_a_volume %d ", value); }));

    tacho_1_frequency->connect_to(
        new LambdaConsumer<float>([](float value)
                                  { debugD("tacho_1_frequency %d ", value); }));

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

    // No need to parse the messages at every single loop iteration; 1 ms will do
    app.onRepeat(1, []()
                 {
                     // nmea2000->ParseMessages();
                 });

    // enable CAN status polling
    app.onRepeat(500, []()
                 {
                     printValues();
                     // PollCANStatus();
                 });

    sensesp_app->start();
}
// The loop function is called in an endless loop during program execution.
// It simply calls `app.tick()` which will then execute all reactions as needed.
void loop()
{
    app.tick();
}
