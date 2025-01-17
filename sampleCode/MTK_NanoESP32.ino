/***************************************************
 This is a library for the Multi-Touch Kit
 Designed and tested to work with Arduino Uno, MEGA2560, LilyPad(ATmega 328P)
 Note: Please remind to disconnect AREF pin from AVCC on Lilypad
 
 For details on using this library see the tutorial at:
 ----> https://hci.cs.uni-saarland.de/multi-touch-kit/
 
 Written by Narjes Pourjafarian, Jan Dickmann, Juergen Steimle (Saarland University), 
            Anusha Withana (University of Sydney), Joe Paradiso (MIT)
 MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <MTKNanoESP32.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
#define BLINK_PIN 8

//----- Multiplexer input pins (for UNO) -----
int s0 = 7;
int s1 = 6;
int s2 = 5;
int s3 = 4;
int s4 = 3;

int muxPins[5] = {s0, s1, s2, s3, s4};

int analogPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int IMUPins[2] = {D11, D12};

rmt_data_t pwm_50duty[] = {
  {1, 1, 0, 0},  
  {0, 0, 0, 0}   
};

//----- Number of receiver (RX) and transmitter (TX) lines -----
int RX_num = 8;
int TX_num = 23;

//----- Receive raw capacitance data or touch up/down states -----
boolean raw_data = true;  // true: receive raw capacitance data, false: receive touch up/down states
int threshold = 130;  // Threshold for detecting touch down state (only required if raw_data = false). 
                    // Change this variable based on your sensor. (for more info. check the tutorial)
boolean toBLE = true; // BLE出力モード

MTKNanoESP32 mtk;

void setup() {
  // シリアル通信
  Serial.begin(115200);
  sleep(5); // 接続待ち
  Serial.println("S");

  // センサのセットアップ
  mtk.setup_sensor(RX_num,TX_num,muxPins,raw_data,threshold, 5, analogPins, toBLE, IMUPins);

  // RMTの処理 10MHz50%
  rmtInit(BLINK_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 20000000);
  rmtWriteLooping(BLINK_PIN, pwm_50duty, RMT_SYMBOLS_OF(pwm_50duty));
}

void loop() {
  if(toBLE){
    BLEDevice central = BLE.central();
    if (central) {
      // while the central is still connected to peripheral:
      while (central.connected()) {
        mtk.read();
        //delay(50);
      }
    }
  }else{
    mtk.read();
    delay(50);
  }
}
