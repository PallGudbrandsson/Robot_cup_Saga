/***************************************************************************
*   Copyright (C) 2019 by DTU                             *
*   jca@elektro.dtu.dk                                                    *
*
*   Main function for small regulation control object (regbot)
*   build on Teensy 3.1 or higher,
*   intended for 31300/1 Linear control
*   has an IMU and a dual motor controller with current feedback.
*
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

// #define REV_ID "$Id: regbot.cpp 1281 2021-07-11 07:56:25Z jcan $"

#include <malloc.h>
#include <ADC.h>
// #include "../teensy3/kinetis.h"
// #include "../teensy3/pins_arduino.h"
// #include "../teensy3/core_pins.h"
#include "pins.h"
#include "IntervalTimer.h"
#include "mpu9150.h"
#include "motor_controller.h"
#include "data_logger.h"
#include "control.h"
#include "robot.h"
#include "src/main.h"
#include "mission.h"
#include "linesensor.h"
#include "dist_sensor.h"
#include "eeconfig.h"
#include "wifi8266.h"
#include "command.h"
#include "servo.h"
#include "subscribe.h"
#include "tunes.h"
//#include <../Snooze/Snooze.h>
//#define __MK20DX256__

#ifndef REGBOT_HW4
#pragma message "REGBOT v3 and older COMPILE"
#elif defined(REGBOT_HW4)
#pragma message "REGBOT v4 COMPILE"
//#define ENABLE_TUNES
// for on-board SD card
#endif

int16_t robotId = 0;
uint8_t robotHWversion = 6;

// main heartbeat timer to service source data and control loop interval
IntervalTimer hbTimer;
/// has positive reply been received frrom IMU
bool imuAvailable = false;
// battery low filter
uint16_t batVoltInt = 0;
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (control_period)
volatile uint32_t hb10us = 0;     /// heart beat timer count (10 us)
// flag for start of new control period
volatile bool startNewCycle = false;
uint32_t startCycleCPU;
//
uint32_t controlUsedTime[3] = { 0, 0, 0 }; // third item is acceleration limit is reached (active)
int pushHBlast = 0;
bool batteryHalt = false;
float steerWheelAngle = 0;
// Heart beat interrupt service routine
void hbIsr ( void );
///
ADC * adc = new ADC();
uint16_t adcInt0Cnt = 0;
uint16_t adcInt1Cnt = 0;
uint16_t adcStartCnt = 0, adcHalfCnt = 0;
// Destination for the first 5 ADC conversions. Value is set in ADC interrupt routine
uint16_t * adcDest[ADC_NUM_NO_LS] =
{
  &irRawAD[0],
  &irRawAD[1],
  &batVoltInt,
  &motorCurrentM[0],
  &motorCurrentM[1]

};
// List of AD numbers. First 5 values are ID, Battery and motor current. Remaining are the 8 line-sensor values
int adcPin[ADC_NUM_ALL] =
{
  PIN_IR_RAW_1,
  PIN_IR_RAW_2,
  PIN_BATTERY_VOLTAGE,
  PIN_LEFT_MOTOR_CURRENT,
  PIN_RIGHT_MOTOR_CURRENT,
  PIN_LINE_SENSOR_0,
  PIN_LINE_SENSOR_1,
  PIN_LINE_SENSOR_2,
  PIN_LINE_SENSOR_3,
  PIN_LINE_SENSOR_4,
  PIN_LINE_SENSOR_5,
  PIN_LINE_SENSOR_6,
  PIN_LINE_SENSOR_7
};
int adcSeq = 0;
bool adcHalf; // for double ADC conversion for LS
uint32_t adcStartTime, adcConvertTime;
uint32_t adcHalfStartTime, adcHalfConvertTime;

/// all control implementations (tick etc)
UControl control;
// the main mission class
UMission userMission;

UServo servo;

USubscribe subscribe;
/// main loop counter
uint32_t mainLoop = 0;

#ifdef REGBOT_HW4
  const int EEPROM_SIZE = 4048;
#else
  const int EEPROM_SIZE = 2024;
#endif


// ////////////////////////////////////////

void setup()   // INITIALIZATION
{
  pinMode ( PIN_LED_DEBUG, OUTPUT );
  // init USB connection (parameter is not used - always 12MB/s)
  Serial.begin ( 115200 ); // USB init serial
  Serial1.begin ( 115200 ); // connection to wifi serial connection (should be fast and no error) init serial to ardulog
  Serial3.begin ( 115200 ); // connection to serial connection init motor control
  motorInit();           // init ports PWM out, direction out, encoder in
  servo.initServo();     // set PWM for available servo pins
  analogWriteResolution ( 12 ); // set PWM resolution
  // init AD converter
  if ( useADCInterrupt ) // AD using interrupt
  {
    //adc->setResolution ( useADCresolution, ADC_0 );
    adc->adc0->setResolution ( useADCresolution); // , ADC_0 );
    //adc->setResolution ( useADCresolution, ADC_1 );
    adc->adc1->setResolution ( useADCresolution);
    //
    // adc->setReference ( ADC_REFERENCE::REF_1V2, ADC_0 );
    adc->adc0->setReference ( ADC_REFERENCE::REF_1V2);
    // adc->setReference ( ADC_REFERENCE::REF_1V2, ADC_1 );
    adc->adc1->setReference ( ADC_REFERENCE::REF_1V2);
    //
    //adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_0);
    //adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_1);
    //
    //adc->setConversionSpeed ( ADC_CONVERSION_SPEED::MED_SPEED, ADC_0 );
    adc->adc0->setConversionSpeed ( ADC_CONVERSION_SPEED::MED_SPEED);
    //adc->setConversionSpeed ( ADC_CONVERSION_SPEED::MED_SPEED, ADC_1 );
    adc->adc1->setConversionSpeed ( ADC_CONVERSION_SPEED::MED_SPEED);
  }
  else     // AD poll
  {
    analogReference ( INTERNAL );
  }
  // more pins
  pinMode ( PIN_START_BUTTON, INPUT ); // start switch - version 2B
  pinMode ( PIN_BATTERY_VOLTAGE, INPUT ); // battery voltage (A9)
  pinMode ( PIN_DISABLE2, INPUT ); // start switch - version 1 - moved to 6 on hardware 2 og 3
  pinMode ( PIN_LINE_LED_HIGH, OUTPUT ); // line sensor LED full power
  pinMode ( PIN_LINE_LED_LOW, OUTPUT ); // LED line sensor - half power (HW3)
  pinMode ( PIN_POWER_IR, OUTPUT ); // line sensor LED half power (HW2) - or power to IR (with new power board) (output anyhow)
  pinModeLed = OUTPUT; // switch to input for half power
  pinMode ( PIN_LINE_SENSOR_0, INPUT ); // Line sensor sensor value
  pinMode ( PIN_LINE_SENSOR_1, INPUT ); // Line sensor sensor value
  pinMode ( PIN_LINE_SENSOR_2, INPUT ); // Line sensor sensor value
  pinMode ( PIN_LINE_SENSOR_3, INPUT ); // Line sensor sensor value
  pinMode ( PIN_LINE_SENSOR_4, INPUT ); // Line sensor sensor value
  pinMode ( PIN_LINE_SENSOR_5, INPUT ); // Line sensor sensor value
  pinMode ( PIN_LINE_SENSOR_6, INPUT ); // Line sensor sensor value
  pinMode ( PIN_LINE_SENSOR_7, INPUT ); // Line sensor sensor value
  lineSensorOn = false;
  digitalWriteFast ( PIN_LINE_LED_HIGH, lineSensorOn );
  // IR power off (hw version 3)
  digitalWriteFast ( PIN_POWER_IR, false );
  // start 10us timer (heartbeat timer)
  hbTimer.begin ( hbIsr, ( unsigned int ) 10 ); // heartbeat timer, value in usec
  // data logger init
  setLogFlagDefault();
  initLogStructure ( 100000 / CONTROL_PERIOD_10us );
  // init encoder interrupts
  attachInterrupt ( M1ENC_A, m1EncoderA, CHANGE );
  attachInterrupt ( M2ENC_A, m2EncoderA, CHANGE );
  attachInterrupt ( M1ENC_B, m1EncoderB, CHANGE );
  attachInterrupt ( M2ENC_B, m2EncoderB, CHANGE );
  // I2C init
  // Setup for Master mode, pins 18/19 (V4, older versions use pins 16/17) external pullups, 400kHz
#ifdef REGBOT_HW4
  Wire.begin ( I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400 );
#else
  Wire.begin ( I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400 );
#endif
  // Initialization of MPU9150
  digitalWriteFast ( PIN_LED_DEBUG, 1 );
  imuAvailable = MPU9150_init();
  digitalWriteFast ( PIN_LED_DEBUG, 0 );
  if ( not imuAvailable )
    usb_send_str ( "# failed to find IMU\r\n" );
  // read configuration from EE-prom (if ever saved)
  // this overwrites the just set configuration for e.g. logger
  // if a configuration is saved
  if ( true )
    eeConfig.eePromLoadStatus ( NULL );
  //
  if ( imuAvailable )
  {
    if ( true )
    {
      bool isOK = mpuRequestData();
      if ( not isOK )
        usb_send_str ( "#first ACC read request failed\r\n" );
      isOK = mpuReadData();
      if ( not isOK )
        usb_send_str ( "#first ACC read failed (too)\r\n" );
    }
    readMagnetometer();
  }
  if ( useADCInterrupt ) // initialize analog values
  {
    motorCurrentM[0] = adc->analogRead ( PIN_LEFT_MOTOR_CURRENT );
    motorCurrentM[1] = adc->analogRead ( PIN_RIGHT_MOTOR_CURRENT );
    batVoltInt = adc->analogRead ( PIN_BATTERY_VOLTAGE );
    // initialize low-pass filter for current offset
    motorCurrentMLowPass[0] = adc->analogRead ( PIN_LEFT_MOTOR_CURRENT ) * 100;
    motorCurrentMLowPass[1] = adc->analogRead ( PIN_RIGHT_MOTOR_CURRENT ) * 100;
    // enable interrupt for the remaining ADC oprations
    adc->adc0->enableInterrupts(adc0_isr); // ( ADC_0 );
    adc->adc1->enableInterrupts(adc1_isr); // ( ADC_1 );
  }
  else
  {
    motorCurrentM[0] = analogRead ( PIN_LEFT_MOTOR_CURRENT );
    motorCurrentM[1] = analogRead ( PIN_RIGHT_MOTOR_CURRENT );
    // battery voltage
    batVoltInt = analogRead ( PIN_BATTERY_VOLTAGE );
    // initialize low-pass filter for current offset
    motorCurrentMLowPass[0] = analogRead ( PIN_LEFT_MOTOR_CURRENT ) * 100;
    motorCurrentMLowPass[1] = analogRead ( PIN_RIGHT_MOTOR_CURRENT ) * 100;
  }
#ifdef ENABLE_TUNES
  tunes_init ( PIN_BUZZER );
  if ( tunes_is_ready() )
  {
    sing ( SAX_MELODY_ID );
  }
#endif
  // init CPU cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

/**
* Main loop
* primarily for initialization,
* non-real time services and
* synchronisation with heartbeat.*/
void loop ( void )
{
  uint8_t m = 0;
  int ltc, lastTimerCnt = 0; /// heartbeat timer loop count
  uint32_t loggerRowWait = 0;
  int row = -1;
  uint32_t thisCycleStartTime = 0;
  uint32_t debug1adcIntCntLast = 0, adcIntErrCnt = 0;
  int adcResetCnt = 0;
  // power on
  pinMode ( PIN_POWER_ROBOT, OUTPUT );
  digitalWriteFast ( PIN_POWER_ROBOT, true );
  // wait a bit - to allow usb to connect in order to see init errors
  delay ( 300 ); // ms
  // run initialization
  setup();
  //n = 0;
  motorAnkerVoltage[0] = 0.0;
  motorAnkerVoltage[1] = 0.0;
  // addMotorVoltage(0, 0);
  motorSetAnchorVoltage();
  control.resetControl();

  // time starts now (in seconds)
  missionTime = 0.0;
  bool cycleStarted = false;
  // then just:
  // - listen for incoming commands
  // then every 1 ms (new cycle):
  // - read sensors,
  // - run control
  // - implement on actuators
  // - do data logging
  while ( true ) 
  { // main loop
    // get data from usb//wifi - if available
    handleIncoming(mainLoop);
    //
    ltc = hbTimerCnt;
    // startNewCycle is set by 10us timer interrupt every 1 ms
//     if (startNewCycle and (ARM_DWT_CYCCNT - startCycleCPU) > (F_CPU / 10000))
//       // if all too late, then skip a cycle
//       startNewCycle = false;
    if ( startNewCycle ) // start of new control cycle
    { // error detect
      // test for ADC interrupt disabled (how? - typically seen after a mission end)
      if (debug1adcIntCntLast == adcInt0Cnt)
      {
        if (adcIntErrCnt % 100 == 0)
        { // disabled for 100ms
          const int MSL  = 54;
          char s[MSL];
          snprintf(s, MSL, "# ADC use=%d, seq=%d, resetcnt=%d, reset\n", useADCInterrupt, adcSeq, adcResetCnt);
          usb_send_str(s);
          // after a mission this could happen
          // I haven't found a better solution
          adc->adc0->enableInterrupts(adc0_isr); // ( ADC_0 );
          adc->adc1->enableInterrupts(adc1_isr); // ( ADC_1 );
          adcResetCnt++;
        }
        adcIntErrCnt++;
      }
      else
        adcIntErrCnt = 0;
      // error detect end
      debug1adcIntCntLast = adcInt0Cnt;
      // debug end
      startNewCycle = false;
      cycleStarted = true;
      thisCycleStartTime = startCycleCPU;
      // timing start - not counting wifi setup
      // wifi setup
      if ( wifi.setup != 0 and wifi.setup < 99 and wifi.wifiConnectTryCount < 20) // setup needed
      {
        if ( hbTimerCnt > 2000 ) // wait a few seconds before setup of wifi, it may start by itself (last configuration)
        {
          if ( wifi.setup < 0 ) // error in communication with wifi chip
          {
            if ( not wifi.goingToSleep )
              // start setup of wifi
              wifi.restart();
          }
          // wifi configuration is in setup mode, so continue setup
          wifi.serialSetup();
        }
      }
      // read new sensor values
      readSensors();
      // battery flat check
      if ( batteryUse ) // keep an eye on battery voltage
      {
        batteryMonitoring();
      }
      // record read sensor time
      controlUsedTime[0] = ARM_DWT_CYCCNT - thisCycleStartTime;
      //
      // calculate sensor-related values
      updatePose(mainLoop);
      estimateTilt();
      estimateIrDistance();
      estimteLineEdgePosition();
      if ( not encTimeScaleCalibrated and encTimeTryCalibrate and ( ( hbTimerCnt % 21 ) == 0 ) )
        calibrateEncoderTest();
      //
      // mission time control
      if ( control.missionState == 0 ) // we are waiting to start, so time since start is zero
      {
        timeAtMissionStart = hb10us;
        if ( missionTime > 18000.0 )
          // restart time (after 5 hours) - max usable time is 32768, as added in 1ms bits
          missionTime = 0.0;
      }
      // do control
      control.controlTick();
      // Implement on actuators
      servo.servoTick();
      // implement
      motorSetAnchorVoltage();
      // record read sensor time + control time
      controlUsedTime[1] = ARM_DWT_CYCCNT - thisCycleStartTime;
      //     if ((pushInterval > 0) && (ltc - pushTimeLast) >= pushInterval)
      //     { // send to USB and first active wifi client
      //       requestingClient = -2;
      //       pushTimeLast = ltc;
      //       // pack and send one status message
      //       sendStatus();
      //     }
      // send subscribed messages
      subscribe.sendToSubscriber();

      // log data at requested interval
      if ((ltc - lastTimerCnt ) >= logInterval or control.chirpRun)
      {
        bool doLog = not control.chirpRun;
        if (not doLog)
        { // we log anyhow, if we are dooing chirp modulation
          if (control.chirpLog)
          { // time to do a log action
            control.chirpLog = false;
            doLog = true;
          }
        }
        if (doLog)
        {
          lastTimerCnt = ltc;
          m++;
          if (loggerLogging())
          {
            setStatusLed ( ( m & 0xff ) < 128 );
            stateToLog();
          }
        }
      }
    }
    // send logged data to client
    if (logToUSB) // send log to USB or wifi
    {
      if ((hbTimerCnt - loggerRowWait ) > 10 ) // attempt to wait a bit after a few lines send to logger
      {
        // but do not seem to work, so set to just 10ms wait after 8 rows
        int row20 = 0;
        // signal log read using on-board LED
        setStatusLed ( HIGH );
        // transfer 8 rows at a time
        for (row20 = 0; row < logRowCnt /*and row20 < 8*/; row20++ ) // write buffer log to destination
        {
          row = logWriteBufferTo (row);
          row++;
          if ( not logToUSB )
            break;
        }
        // set pause time
        loggerRowWait = hbTimerCnt;
        if ( row >= logRowCnt ) // finished
        {
          logToUSB = false;
          row = -1;
        }
        setStatusLed ( LOW );
      }
    }
    mainLoop++;
    if ( loggerLogging() )
      setStatusLed ( ( ltc & 0xff ) < 127 );
    else
      setStatusLed ( ( ltc & 0x3ff ) < 10 );
    // loop end time
    if (cycleStarted)
    {
      controlUsedTime[2] = ARM_DWT_CYCCNT - thisCycleStartTime;
      cycleStarted = false;
    }
  }
//   return 0;
}

/**
* Heartbeat interrupt routine
* schedules data collection and control loop timing.
* */
void hbIsr ( void ) // called every 10 microsecond
{
  // as basis for all timing
  hb10us++;
  if ( hb10us % CONTROL_PERIOD_10us == 0 ) // 1ms timing - main control period start
  {
    missionTime += 1e-5 * CONTROL_PERIOD_10us;
    hbTimerCnt++;
    startNewCycle = true;
    startCycleCPU = ARM_DWT_CYCCNT;
    if (useADCInterrupt)
    { // start regular AC conversion
      adcSeq = 0;
      adcHalf = false;
      adc->startSingleRead(adcPin[0]); // + 400;
      adcStartTime = hb10us;
      adcStartCnt++;
    }
    //
    // overload for encoder speed based on period
    //     if (not encTimeOverload[0])
    //       if ((int32_t)hb10us - (int32_t)encStartTime[0] > 256*8*256)
    //         encTimeOverload[0] = true;
    //     if (not encTimeOverload[1])
    //       if ((int32_t)hb10us - (int32_t)encStartTime[1] > 256*8*256)
    //         encTimeOverload[1] = true;
  }
  if ( hb10us % CONTROL_PERIOD_10us == 60 ) // start half-time ad conversion
  {
    if ( adcSeq >= ADC_NUM_ALL )
    {
      adcHalfStartTime = hb10us;
      adcSeq = 0;
      adc->startSingleRead ( adcPin[0] );
      adcHalf = true;
      adcHalfCnt++;
    }
  }
}

/////////////////////////////////////////////////////////////////

// If you enable interrupts make sure to call readSingle() to clear the interrupt.
void adc0_isr()
{
  uint16_t v = adc->readSingle ( ADC_0 );
  //uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS];   // <- For ADC Debug
  //Serial.printf("ADC0 %d pin %d\n\r", v, pin);                        // <- For ADC Debug
  if ( adcSeq < ADC_NUM_IR_SENSORS )
  {
    *adcDest[adcSeq] = v;
  }
  else if ( adcSeq < ADC_NUM_NO_LS )
  { // low-pass filter battery and current sensor values at about 2ms time constant
    // result is in range 0..8196 (for measured between 0v and 1.2V)
    *adcDest[adcSeq] = ( ( *adcDest[adcSeq] ) >> 1 ) + v;
  }
  else if ( adcHalf )
  { // line sensor raw value
    adcLSH[adcSeq - ADC_NUM_NO_LS] = v;
  }
  else
  {
    adcLSL[adcSeq - ADC_NUM_NO_LS] = v;
  }
  adcSeq++;
  if ( adcSeq < ADC_NUM_ALL ) // start new and re-enable interrupt
  {
    adc->startSingleRead ( adcPin[adcSeq] );
  }
  else     // finished
  {
    if ( adcHalf )
    {
      adcHalfConvertTime = hb10us - adcHalfStartTime;
      digitalWriteFast ( PIN_LINE_LED_HIGH, lineSensorOn );
      if ( robotHWversion > 2 )
        digitalWriteFast ( PIN_LINE_LED_LOW, lineSensorOn );
      else
        digitalWriteFast ( OLD_PIN_LINE_LED, lineSensorOn );
    }
    else
    {
      adcConvertTime = hb10us - adcStartTime;
      digitalWriteFast ( PIN_LINE_LED_HIGH, LOW );
      if ( robotHWversion > 2 )
        digitalWriteFast ( PIN_LINE_LED_LOW, LOW );
      else
        digitalWriteFast ( OLD_PIN_LINE_LED, LOW );
    }
  }
  adcInt0Cnt++;
}

//////////////////////////////////////////////////////////

void adc1_isr()
{
  uint16_t v = adc->readSingle ( ADC_1 );
  //uint8_t pin = ADC::sc1a2channelADC0[ADC1_SC1A&ADC_SC1A_CHANNELS];   // <- For ADC Debug
  //Serial.printf("ADC1 %d pin %d\n\r", v, pin);                        // <- For ADC Debug
  if ( adcSeq < ADC_NUM_IR_SENSORS )
  { // IR sensor use RAW AD - averaged later
    *adcDest[adcSeq] = v;
  }
  else if ( adcSeq < ADC_NUM_NO_LS )
  { // low-pass filter non-line sensor values at about 2ms time constant
    // result is in range 0..8196 (for measured between 0v and 1.2V)
    *adcDest[adcSeq] = ( ( *adcDest[adcSeq] ) >> 1 ) + v;
  }
  else if ( adcHalf )
  { // line sensor raw value
    adcLSH[adcSeq - ADC_NUM_NO_LS] = v;
  }
  else
  {
    adcLSL[adcSeq - ADC_NUM_NO_LS] = v;
  }
  adcSeq++;
  if ( adcSeq < ADC_NUM_ALL ) // start new and re-enable interrupt
  {
    adc->startSingleRead ( adcPin[adcSeq] );
  }
  else     // finished
  {
    if ( adcHalf )
    {
      adcHalfConvertTime = hb10us - adcHalfStartTime;
      digitalWriteFast ( PIN_LINE_LED_HIGH, lineSensorOn );
      if ( robotHWversion > 2 )
        digitalWriteFast ( PIN_LINE_LED_LOW, lineSensorOn );
      else
        digitalWriteFast ( OLD_PIN_LINE_LED, lineSensorOn );
    }
    else
    {
      adcConvertTime = hb10us - adcStartTime;
      digitalWriteFast ( PIN_LINE_LED_HIGH, LOW );
      if ( robotHWversion > 2 )
        digitalWriteFast ( PIN_LINE_LED_LOW, LOW );
      else
        digitalWriteFast ( OLD_PIN_LINE_LED, LOW );
    }
  }
  adcInt1Cnt++;
}
