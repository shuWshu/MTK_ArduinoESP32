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

#ifndef MTKNanoESP32_H
#define MTKNanoESP32_H

#include <Arduino.h>
#include <ArduinoBLE.h>
#include<Wire.h>

// Constants
#define MTK_DEFAULT_RX 6        // Default RX channels
#define MTK_DEFAULT_TX 6        // Default TX channels
#define MTK_MAX_TX 16           // Max TX channels
#define MTK_DEFAULT_MUX_CNTRL 4 // Default Mux control channels
#define MTK_MAX_MUX_CNTRL 4     // Max Mux control channels

class MTKNanoESP32
{
private:
  // Dimensions
  int _numRx = MTK_DEFAULT_RX;
  int _numTx = MTK_DEFAULT_TX;
  int _numMuxPins = MTK_DEFAULT_MUX_CNTRL;
  char *_adcPins;
  int *_muxPins;
  bool _correct;
  int _threshold;

  int *_analogPins;
  bool toBLE;

  int16_t accel[7]; // IMU関連
  int MPU_addr;
  int *_IMUPins;

  // // BLE関連
  // BLEService bleService;
  // BLEUnsignedLongCharacteristic bleCharacteristic;

  int (*timelineDatas)[8][10]; // 入出力端子数を自作デバイス専用にしている 配列用ポインタを受け取る
  int* writeID; // 書き込み先のインデックス 参照渡し
  int shift;
  //---------- Converting Decimal to Binary -----------

  int (*noise)[8]; // 入出力端子数を自作デバイス専用にしている

  void setupPWM();
  void selectChannelOut(int channel);
  void getIMU();

public:
  // Prototypes
  MTKNanoESP32();

  // Setup
  void setup_sensor(int *muxPins, bool correct, int threshold, int numMuxPins, int *analogPins, bool toBLE, int *IMUPins, int timelineDatas[][8][10], int* writeID, int noise[][8]);

  // Run measurements and send them via Serial
  void read();
};

#endif // MTKNanoESP32_H
