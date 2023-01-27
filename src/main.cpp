#include <Arduino.h>

#define ESP32_CAN_TX_PIN GPIO_NUM_18
#define ESP32_CAN_RX_PIN GPIO_NUM_19

#include <nmea.h>
#include <DallasTemperature.h>
#include <memory>

#include "driver/temp_sensor.h" //legacy esp32 temp sensor driver. https://github.com/espressif/esp-idf/blob/master/components/driver/test_apps/legacy_rtc_temp_driver/main/test_rtc_temp_driver.c

#include <BleSerial.h>
BleSerial ble;

#define REDPIN 3
#define GREENPIN 4
#define BLUEPIN 5
#define COLD_WHITE 18
#define WARM_WHITE 19

/**
 * Controll onboard RGB light
 * setColor(255, 0, 0);  // red
 * setColor(0, 255, 0);  // green
 * setColor(0, 0, 255);  // blue
 * setColor(255, 255, 0);  // yellow
 * setColor(80, 0, 80);  // purple
 * setColor(0, 255, 255);  // aqua
 */
void setColor(int redValue = 0, int greenValue = 0, int blueValue = 0, int coldWhite = 0, int warmWhite = 0)
{
  analogWrite(REDPIN, redValue);
  analogWrite(GREENPIN, greenValue);
  analogWrite(BLUEPIN, blueValue);

  analogWrite(COLD_WHITE, 0);
  analogWrite(WARM_WHITE, 0);
}

void initTempSensor()
{
  temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
  temp_sensor.dac_offset = TSENS_DAC_L2; // TSENS_DAC_L2 is default   L4(-40℃ ~ 20℃), L2(-10℃ ~ 80℃) L1(20℃ ~ 100℃) L0(50℃ ~ 125℃)
  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();
}

#define ENABLE_DEBUG_LOG true // flag to turn on/off debugging
// println all messages to serial console and BLE
// to see all messages via phone app Serial Bluetooth terminal
#define debug_log(...)             \
  do                               \
  {                                \
    if (ENABLE_DEBUG_LOG)          \
    {                              \
      Serial.println(__VA_ARGS__); \
      ble.println(__VA_ARGS__);    \
    }                              \
  } while (0)

#define ADC_Calibration_Value1 250.0 // For resistor measure 5 Volt and 180 Ohm equals 100% plus 1K resistor.
#define ADC_Calibration_Value2 34.3  // The real value depends on the true resistor values for the ADC input (100K / 27 K).

// RPM data. Generator RPM is measured on connector "W"
#define RPM_Calibration_Value 1.0 // Translates Generator RPM to Engine RPM
#define Eingine_RPM_Pin 20        // Engine RPM is measured as interrupt on GPIO 20

volatile uint64_t StartValue = 0;  // First interrupt value
volatile uint64_t PeriodCount = 0; // period in counts of 0.000001 of a second
unsigned long Last_int_time = 0;

hw_timer_t *timer = NULL;                        // pointer to a variable of type hw_timer_t
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // synchs between maon cose and interrupt?

// Data wire for teperature (Dallas DS18B20) is plugged into GPIO 1 on the ESP32
#define ONE_WIRE_BUS 1
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Voltage measure is connected GPIO 2 (Analog ADC1_1)
const int ADCpin2 = 2;

// Tank fluid level measure is connected GPIO 6 (Analog ADC1_4)
const int ADCpin1 = 0;

// Global Data
float FuelLevel = 0;
float ExhaustTemp = 0;
float EngineRPM = 0;
float BatteryVolt = 0;

// Task handle for OneWire read (Core 0 on ESP32)
TaskHandle_t Task1;

// Serial port 2 config (GPIO 16)
const int baudrate = 38400;
const int rs_config = SERIAL_8N1;

// RPM Event Interrupt
// Enters on falling edge
//=======================================
void IRAM_ATTR handleInterrupt()
{
  portENTER_CRITICAL_ISR(&mux);
  uint64_t TempVal = timerRead(timer); // value of timer at interrupt
  PeriodCount = TempVal - StartValue;  // period count between rising edges in 0.000001 of a second
  StartValue = TempVal;                // puts latest reading as start for next calculation
  portEXIT_CRITICAL_ISR(&mux);
  Last_int_time = millis();
}

// This task runs isolated on core 0 because sensors.requestTemperatures() is slow and blocking for about 750 ms
void GetTemperature(void *parameter)
{
  float tmp = 0;
  for (;;)
  {
    setColor(40, 40, 0);
    sensors.requestTemperatures(); // Send the command to get temperatures
    vTaskDelay(100);
    tmp = sensors.getTempCByIndex(0);
    if (tmp != -127)
      ExhaustTemp = tmp;

    setColor(0, 40, 0);
    vTaskDelay(100);

    // debug_log("ExhaustTemp=" + String(ExhaustTemp));

    // Get internal temp
    float result = 0;
    ESP_ERROR_CHECK(temp_sensor_read_celsius(&result));
    debug_log("InternalTemp=C" + String(result));
  }
}

// Calculate engine RPM from number of interupts per time
double ReadRPM()
{
  double RPM = 0;

  portENTER_CRITICAL(&mux);
  if (PeriodCount != 0)
  {                                 // 0 means no signals measured
    RPM = 1000000.00 / PeriodCount; // PeriodCount in 0.000001 of a second
  }
  portEXIT_CRITICAL(&mux);
  if (millis() > Last_int_time + 200)
    RPM = 0; // No signals RPM=0;
  return (RPM);
}

// ReadVoltage is used to improve the linearity of the ESP32 ADC see: https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function
double ReadVoltage(byte pin)
{
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095)
    return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return (-0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089) * 1000;
} // Added an improved polynomial, use either, comment out as required

void setup()
{

  // Init USB serial port
  Serial.begin(115200);

  // Start the BLE Serial
  ble.begin("ESP_MONITOR");

  // Init internal temp sensor
  initTempSensor();

  //
  setupNMEA();

  // onboard RGB
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);

  // Init RPM measure
  pinMode(Eingine_RPM_Pin, INPUT_PULLUP);                                            // sets pin high
  attachInterrupt(digitalPinToInterrupt(Eingine_RPM_Pin), handleInterrupt, FALLING); // attaches pin to interrupt on Falling Edge
  timer = timerBegin(0, 80, true);                                                   // this returns a pointer to the hw_timer_t global variable
  // 0 = first timer
  // 80 is prescaler so 80MHZ divided by 80 = 1MHZ signal ie 0.000001 of a second
  // true - counts up
  timerStart(timer); // starts the timer

  // Start OneWire
  sensors.begin();

  // Create task for core 0, loop() runs on core 1
  xTaskCreatePinnedToCore(
      GetTemperature, /* Function to implement the task */
      "Task1",        /* Name of the task */
      10000,          /* Stack size in words */
      NULL,           /* Task input parameter */
      0,              /* Priority of the task */
      &Task1,         /* Task handle. */
      0);             /* Core where the task should run */

  delay(200);
}

void loop()
{

  unsigned int size;

  BatteryVolt = ((BatteryVolt * 15) + (ReadVoltage(ADCpin2) * ADC_Calibration_Value2 / 4096)) / 16; // This implements a low pass filter to eliminate spike for ADC readings

  FuelLevel = ((FuelLevel * 15) + (ReadVoltage(ADCpin1) * ADC_Calibration_Value1 / 4096)) / 16; // This implements a low pass filter to eliminate spike for ADC readings

  EngineRPM = ((EngineRPM * 5) + ReadRPM() * RPM_Calibration_Value) / 6; // This implements a low pass filter to eliminate spike for RPM measurements

  if (FuelLevel > 100)
    FuelLevel = 100;

  // SendN2kTankLevel(FuelLevel, 200); // Adjust max tank capacity.  Is it 200 ???
  // SendN2kExhaustTemp(ExhaustTemp);
  // SendN2kEngineRPM(EngineRPM);
  // SendN2kBattery(BatteryVolt);

   loopNMEA();

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if (ble.available())
  {
    Serial.write(ble.read());
  }
}
