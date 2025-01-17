/***************************************************
 This is a library for the Multi-Touch Kit
 Designed and tested to work with Arduino Uno, MEGA2560, LilyPad(ATmega 328P)
 Note: Please remind to disconnect AREF pin from AVCC on Lilypad

 For details on using this library see the tutorial at:
 ----> https://hci.cs.uni-saarland.de/multi-touch-kit/

 Written by Narjes Pourjafarian, Jan Dickmann, Juergen Steimle (Saarland University),
            Anusha Withana (University of Sydney), Joe Paradiso (MIT)
 MIT license, all text above must be included in any redistribution

 Arduino Nano ESP32に対応させたバージョン

 コードの修正記録（田村）
  yyyy/mm/dd：ArduinoNanoへの対応
  2024/04/22：日本語コメントを付ける
  2024/04/22：出力tx上限を32に増やす
  　　　　　　　・setup_sensor：引数に配列要素数を追加，
  　　　　　　　・selectChannelOut：32出力に対応させた
  2024/10/04：コードをESP32へ対応
  2024/10/06：BLEへの対応
  2024/10/07：データサイズ不足だったので，キャラクタリスティックを6つに増やした
  2024/12/06：IMUを用いた加速度取得機能を追加
  2024/12/18：ローパスフィルタの追加→削除
  2025/01/17：不要な部分を削る
 ****************************************************/

#include "MTKNanoESP32.h"

MTKNanoESP32::MTKNanoESP32(void)
  : bleService("8fc03459-5012-dcc8-f6c5-3425f9bb10ed"),
    charArray {
      BLECharacteristic("b70c5e87-fcd8-eec1-bf23-6e98d6b38730", BLERead | BLENotify, 180),
      BLECharacteristic("b70c5e87-fcd8-eec1-bf23-6e98d6b38731", BLERead | BLENotify, 180),
      BLECharacteristic("b70c5e87-fcd8-eec1-bf23-6e98d6b38732", BLERead | BLENotify, 180),
      BLECharacteristic("b70c5e87-fcd8-eec1-bf23-6e98d6b38733", BLERead | BLENotify, 180),
      BLECharacteristic("b70c5e87-fcd8-eec1-bf23-6e98d6b38734", BLERead | BLENotify, 180),
      BLECharacteristic("b70c5e87-fcd8-eec1-bf23-6e98d6b38735", BLERead | BLENotify, 180)
    }
{
}
/**
 *   @brief  Setup the sensor
 *
 *   @return void
 */
// スケッチ内のsetup()内にて呼び出される関数
void MTKNanoESP32::setup_sensor(int rx, int tx, int *muxPins, bool raw_data, int threshold, int numMuxPins, int *analogPins, bool toBLE, int *IMUPins)
{
    this->_numRx = rx;              // 入力端子数
    this->_numTx = tx;              // 出力端子数
    this->_muxPins = muxPins;       // 各桁を示すピンの内容
    this->_raw_data = raw_data;     // 出力の種類（値int or 判定bool）
    this->_threshold = threshold;   // 閾値
    this->_numMuxPins = numMuxPins; // 桁数
    this->_analogPins = analogPins; // アナログ入力ピンの内訳
    this->toBLE = toBLE; // 出力先をBLEにするか
    this->_IMUPins = IMUPins;

    // set the PWM values
    // setupPWM(); // セットアップはコードの方で

    // Set mulptiplexer
    for (int k = 0; k < this->_numMuxPins; k++)
    {
        pinMode(this->_muxPins[k], OUTPUT); // 信号出力ピンの設
    }

    pinMode(A0, INPUT); // this is a workaround for a bug

    // タッチ認識用の基底値（=noise）の取得
    // TODO:形式を変えるべき
    if (this->_raw_data == false)
    {
        for (int ii = 0; ii < 10; ii++)
        {
            for (int i = 0; i < this->_numTx; i++)
            {
                selectChannelOut(i);
                // Read RX
                for (int j = 0; j < this->_numRx; j++)
                {
                    this->noise[i][j] = analogRead(_analogPins[j]);
                }
            }
        }
    }

    // BLEの初期化とサービスの追加
    if (toBLE) {
        if(BLE.begin()){
            Serial.println("BLEStart");
        }
        BLE.setDeviceName("ArduinoNanoESP32");
        for (int i = 0; i < 6; i++) {
            bleService.addCharacteristic(charArray[i]);
        }
        BLE.addService(bleService);
        BLE.setLocalName("CylindricalController");
        BLE.setAdvertisedService(bleService);
        BLE.advertise();
    }

    // 一旦コメントアウト
    // IMUの起動
    // MPU_addr=0x68; // I2C address of the MPU-6050
    // Wire1.begin(_IMUPins[0], _IMUPins[1]); //sda, scl
    // Wire1.beginTransmission(MPU_addr);
    // Wire1.write(0x6B);  // PWR_MGMT_1 register
    // Wire1.write(0);     // set to zero (wakes up the MPU-6050)
    // Wire1.endTransmission(true);
}

/**
 *   @brief  Read multi-touch data from sensor and write it to Serial
 *
 *   @return void
 */
// データを読み取り，シリアルに書き込む
// loopb内で呼び出される
void MTKNanoESP32::read()
{
    if (toBLE){
        const int bufferSize = 180;
        char sendValues[bufferSize] = {0};
        int index;
        for (int i = 0; i < this->_numTx; i++){
            // 送信先キャラクタリスティックを変更する
            if(i % 4 == 0){
                memset(sendValues, '\0', sizeof(sendValues)); // バッファの初期化
                index = 0; // インデックスの初期化
            }
            selectChannelOut(i); // 信号送信先の変更
            int length = snprintf(sendValues + index, bufferSize - index, "%d", i); // 行番号の出力
            index += length;

            // Read RX
            for (int j = 0; j < this->_numRx; j++)
            {
                int rawValue = analogRead(_analogPins[j]);
                // float filteredValue = lowpassFilters[j]->input(rawValue); // フィルタを適用
                length = snprintf(sendValues + index, bufferSize - index, ",%03d", rawValue); // %03dゼロ埋めフォーマット
                index += length;
            }

            length = snprintf(sendValues + index, bufferSize - index, ":"); // 行終端の出力
            index += length;

            if(i == 22){
                // getIMU();
                // 加速度
                for(int k = 0; k < 3; ++k){
                    length = snprintf(sendValues + index, bufferSize - index, "_%d", this->accel[k]); // 行終端の出力
                    index += length;
                }
                // 角速度
                for(int k = 0; k < 3; ++k){
                    length = snprintf(sendValues + index, bufferSize - index, "_%d", this->accel[k+4]); // 行終端の出力
                    index += length;
                }
            }

            if(i % 4 == 3 || i == 22){ // 4行ごとにキャラクタリスティックに登録
                charArray[i/5].writeValue((const uint8_t*)sendValues, index);
            }
        }
    }else if (Serial){
        // データの出力方式によって変化→シリアル通信は生の値のみにした
        for (int i = 0; i < this->_numTx; i++) // 各出力先につき繰り返し
        {
            selectChannelOut(i); // 信号送信先の変更
            Serial.print(i);     // 行番号をシリアル出力
            // Read RX
            for (int j = 0; j < this->_numRx; j++)
            {
                Serial.print(",");
                Serial.print(analogRead(_analogPins[j]));
            }
            Serial.println();
        }
    }
}

/**
 *   @brief  Set select_pins of the multiplexer
 *
 *   @return void
 */
// 出力先のチャンネルを設定，変更する関数．引数のチャンネルへ送信できるようにする．
// 32入力まで拡張，muxPinsのインデックスを反転
void MTKNanoESP32::selectChannelOut(int channel)
{
    int r0 = channel & 1;
    int r1 = channel & 2;
    int r2 = channel & 4;
    int r3 = channel & 8;
    int r4 = channel & 16;

    digitalWrite(this->_muxPins[0], (r0 == 0 ? LOW : HIGH));
    digitalWrite(this->_muxPins[1], (r1 == 0 ? LOW : HIGH));
    digitalWrite(this->_muxPins[2], (r2 == 0 ? LOW : HIGH));
    digitalWrite(this->_muxPins[3], (r3 == 0 ? LOW : HIGH));
    digitalWrite(this->_muxPins[4], (r4 == 0 ? LOW : HIGH));
}

/* 
加速度の取得
引数：返値を格納する配列
 */
void MTKNanoESP32::getIMU()
{
    Wire1.beginTransmission(MPU_addr);
    Wire1.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire1.endTransmission(false);
    Wire1.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    this->accel[0]=Wire1.read()<<8|Wire1.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    this->accel[1]=Wire1.read()<<8|Wire1.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    this->accel[2]=Wire1.read()<<8|Wire1.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    this->accel[3]=Wire1.read()<<8|Wire1.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    this->accel[4]=Wire1.read()<<8|Wire1.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    this->accel[5]=Wire1.read()<<8|Wire1.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    this->accel[6]=Wire1.read()<<8|Wire1.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
