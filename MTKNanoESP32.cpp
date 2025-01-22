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
  2025/01/17：並列処理モードを作成
 ****************************************************/

#include "MTKNanoESP32.h"

MTKNanoESP32::MTKNanoESP32(void)
{
}
/**
 *   @brief  Setup the sensor
 *
 *   @return void
 */
// スケッチ内のsetup()内にて呼び出される関数
void MTKNanoESP32::setup_sensor(int *muxPins, bool correct, int threshold, int numMuxPins, int *analogPins, bool toBLE, int *IMUPins, int timelineDatas[][8][10], int* writeID, int noise[][8])
{
    this->_numRx = 8;              // 入力端子数
    this->_numTx = 23;              // 出力端子数
    this->_muxPins = muxPins;       // 各桁を示すピンの内容
    this->_correct = correct;     // 補正の有無
    this->_threshold = threshold;   // 閾値
    this->_numMuxPins = numMuxPins; // 桁数
    this->_analogPins = analogPins; // アナログ入力ピンの内訳
    this->toBLE = toBLE; // 出力先をBLEにするか
    this->_IMUPins = IMUPins;
    this->timelineDatas = timelineDatas;
    this->writeID = writeID; // 書き込み先インデックスを決定
    this->noise = noise;

     *this->writeID = 0; // 書き込み先インデックスのリセット
    
    // set the PWM values
    // setupPWM(); // セットアップはコードの方で

    // Set mulptiplexer
    for (int k = 0; k < this->_numMuxPins; k++)
    {
        pinMode(this->_muxPins[k], OUTPUT); // 信号出力ピンの設
    }

    pinMode(A0, INPUT); // this is a workaround for a bug

    // タッチ認識用の基底値の取得
    int correctFrame = 30;
    if (this->_correct == true){
        for(int i=0; i < correctFrame; ++i){
            if(i % 4 < 2){
                digitalWrite(LED_BUILTIN, HIGH);
            }else{
                digitalWrite(LED_BUILTIN, LOW);
            }
            for (size_t tx = 0; tx < this->_numTx; tx++){
                selectChannelOut(tx); // 信号送信先の変更
                for (size_t rx = 0; rx < this->_numRx; rx++){
                    int rawValue = analogRead(_analogPins[rx]);
                    if(rawValue > this->noise[tx][rx]){
                        this->noise[tx][rx] = rawValue;
                    }
                    if(i == correctFrame - 1){ // 出力
                        Serial.print(this->noise[tx][rx]);
                        Serial.print(",");
                    }
                }
                if(i == correctFrame - 1){ // 出力
                    Serial.println("");
                }
            }
        }
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
        for (size_t tx = 0; tx < this->_numTx; tx++){
            selectChannelOut(tx); // 信号送信先の変更
            for (size_t rx = 0; rx < this->_numRx; rx++){
                double rawValue = (double)analogRead(_analogPins[rx]);
                this->timelineDatas[tx][rx][*writeID] = rawValue; // 書込
            }
        }
        *writeID = (*writeID+1)%10; // インデックスの更新
        //Serial.println("read");
        // for(int i = 0; i < 10; ++i){
        //   Serial.print(this->timelineDatas[0][0][i]);
        //   Serial.print(",");
        // }
        // Serial.println("");
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
