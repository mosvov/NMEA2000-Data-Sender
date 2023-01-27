/*
NMEA2000_esp32.cpp

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inherited NMEA2000 object for ESP32 modules. See also NMEA2000 library.

Thanks to Thomas Barth, barth-dev.de, who has written ESP32 CAN code. To avoid extra
libraries, I implemented his code directly to the NMEA2000_esp32 to avoid extra
can.h library, which may cause even naming problem.
*/

#include "driver/periph_ctrl.h"
// include "soc/dport_reg.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "NMEA2000_esp32.h"
#define EXAMPLE_TAG "TWAI Master"

#if !defined(round)
#include <math.h>
#endif

bool tNMEA2000_esp32::CanInUse = false;
tNMEA2000_esp32 *pNMEA2000_esp32 = 0;

// void ESP32Can1Interrupt(void *);

//*****************************************************************************
tNMEA2000_esp32::tNMEA2000_esp32(gpio_num_t _TxPin, gpio_num_t _RxPin) : tNMEA2000(), IsOpen(false),
                                                                         speed(CAN_SPEED_250KBPS), TxPin(_TxPin), RxPin(_RxPin)
{
}

//*****************************************************************************
bool tNMEA2000_esp32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool /*wait_sent*/)
{

  twai_message_t message;
  message.identifier = id;
  message.data_length_code = len > 8 ? 8 : len;
  memcpy(message.data, buf, len);

  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
  {
    printf("Message queued for transmission\n");
    return true;
  }
  else
  {
    printf("Failed to queue message for transmission\n");
    return false;
  }
}

//*****************************************************************************
void tNMEA2000_esp32::InitCANFrameBuffers()
{
  // if (MaxCANReceiveFrames < 10)
  //   MaxCANReceiveFrames = 50; // ESP32 has plenty of RAM
  // if (MaxCANSendFrames < 10)
  //   MaxCANSendFrames = 40;
  // uint16_t CANGlobalBufSize = MaxCANSendFrames - 4;
  // MaxCANSendFrames = 4; // we do not need much libary internal buffer since driver has them.
  // RxQueue = xQueueCreate(MaxCANReceiveFrames, sizeof(tCANFrame));
  // TxQueue = xQueueCreate(CANGlobalBufSize, sizeof(tCANFrame));

  tNMEA2000::InitCANFrameBuffers(); // call main initialization
}

//*****************************************************************************
bool tNMEA2000_esp32::CANOpen()
{
  if (IsOpen)
    return true;

  if (CanInUse)
    return false; // currently prevent accidental second instance. Maybe possible in future.

  pNMEA2000_esp32 = this;
  IsOpen = true;
  CAN_init();

  CanInUse = IsOpen;

  return IsOpen;
}

//*****************************************************************************
bool tNMEA2000_esp32::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{
  bool HasFrame = false;

  // Wait for message to be received
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK)
  {
    HasFrame = true;
    printf("Message received\n");
  }
  else
  {
    printf("Failed to receive message\n");
    return false;
  }

  // Process received message
  if (message.extd)
  {
    printf("Message is in Extended Format\n");
  }
  else
  {
    printf("Message is in Standard Format\n");
  }
  printf("ID is %d\n", message.identifier);
  if (!(message.rtr))
  {
    for (int i = 0; i < message.data_length_code; i++)
    {
      printf("Data byte %d = %d\n", i, message.data[i]);
    }
  }

  id = message.identifier;
  len = message.data_length_code;
  memcpy(buf, message.data, message.data_length_code);

  return HasFrame;
}

//*****************************************************************************
void tNMEA2000_esp32::CAN_init()
{

  // Time quantum
  double __tq;

  // A soft reset of the ESP32 leaves it's CAN controller in an undefined state so a reset is needed.
  // Reset CAN controller to same state as it would be in after a power down reset.
  // periph_module_reset(PERIPH_CAN_MODULE);

  // enable module
  // DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
  // DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_18, GPIO_NUM_19, TWAI_MODE_NORMAL);
  // g_config.tx_queue_len = 20;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));

  // configure RX pin
  // gpio_set_direction(RxPin, GPIO_MODE_INPUT);
  // gpio_matrix_in(RxPin, TWAI_RX_IDX, 0);
  // gpio_pad_select_gpio(RxPin);

  // set to PELICAN mode
  // MODULE_CAN->clock_divider_reg.cd = 0x1;

  // synchronization jump width is the same for all baud rates
  // MODULE_CAN->bus_timing_0_reg.sjw = 0x1;

  // TSEG2 is the same for all baud rates
  // MODULE_CAN->bus_timing_1_reg.tseg2 = 0x1;

  // select time quantum and set TSEG1
  switch (speed)
  {
  case CAN_SPEED_1000KBPS:
    // MODULE_CAN->bus_timing_1_reg.tseg1 = 0x4;
    __tq = 0.125;
    break;

  case CAN_SPEED_800KBPS:
    // MODULE_CAN->bus_timing_1_reg.tseg1 = 0x6;
    __tq = 0.125;
    break;
  default:
    // MODULE_CAN->bus_timing_1_reg.tseg1 = 0xc;
    __tq = ((float)1000 / speed) / 16;
  }

  // set baud rate prescaler
  // MODULE_CAN->bus_timing_0_reg.brp = (uint8_t)round((((APB_CLK_FREQ * __tq) / 2) - 1) / 1000000) - 1;

  /* Set sampling
   * 1 -> triple; the bus is sampled three times; recommended for low/medium speed buses     (class A and B) where filtering spikes on the bus line is beneficial
   * 0 -> single; the bus is sampled once; recommended for high speed buses (SAE class C)*/
  // MODULE_CAN->bus_timing_1_reg.sam = 0x1;

  // enable all interrupts
  // MODULE_CAN->interrupt_enable_reg.val = 0xef; // bit 0x10 contains Baud Rate Prescaler Divider (BRP_DIV) bit

  // no acceptance filtering, as we want to fetch all messages
  // MODULE_CAN->MBX_CTRL.acceptance_filter.acr[0] = 0;
  // MODULE_CAN->MBX_CTRL.ACC.CODE[1] = 0;
  // MODULE_CAN->MBX_CTRL.ACC.CODE[2] = 0;
  // MODULE_CAN->MBX_CTRL.ACC.CODE[3] = 0;
  // MODULE_CAN->MBX_CTRL.ACC.MASK[0] = 0xff;
  // MODULE_CAN->MBX_CTRL.ACC.MASK[1] = 0xff;
  // MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0xff;
  // MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0xff;

  // set to normal mode
  // MODULE_CAN->OCR.B.OCMODE = __CAN_OC_NOM; //Output control not supported

  // clear error counters
  //(void)MODULE_CAN->tx_error_counter_reg.txerr;
  //(void)MODULE_CAN->rx_error_counter_reg;
  //(void)MODULE_CAN->error_code_capture_reg;

  // clear interrupt flags
  //(void)MODULE_CAN->interrupt_reg.val;

  // install CAN ISR
  // esp_intr_alloc(ETS_TWAI_INTR_SOURCE, 0, ESP32Can1Interrupt, NULL, NULL);

  // configure TX pin
  //  We do late configure, since some initialization above caused CAN Tx flash
  //  shortly causing one error frame on startup. By setting CAN pin here
  //  it works right.
  // gpio_set_direction(TxPin, GPIO_MODE_OUTPUT);
  // gpio_matrix_out(TxPin, TWAI_TX_IDX, 0, 0);
  // gpio_pad_select_gpio(TxPin);

  // Showtime. Release Reset Mode.
  // MODULE_CAN->mode_reg.rm = 0;

  // Start TWAI driver
  ESP_ERROR_CHECK(twai_start());
  Serial.println("TWAI Driver started");

  // TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF
  ESP_ERROR_CHECK(twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL));
}
