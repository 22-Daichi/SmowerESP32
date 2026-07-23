#include <Arduino.h>

#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_INA260.h>
#include "Adafruit_INA3221.h"

#define rightPwmCh 2
#define leftPwmCh 3
#define bladeMotorCh 4 // 草刈りモーター二つのパワーは一緒でいい
#define liftMotorCh1 5
#define liftMotorCh2 6

// pwmのch0,1は使えない
// servoのせい

const int rxPin = 22;
const int txPin = 23;

const int SDAPin = 25;
const int SCLPin = 33;
Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_INA3221 ina3221;

uint32_t sequenceNumber = 0;

const int relayInputPin = 15; // 入力ピン（pullvdown）
const int relayOutputPin = 2; // 出力ピン

const int leftSwitchPin = 35;  // yellow wire
const int rightSwitchPin = 34; // red wire

const int bladeMotorPin01 = 13;
const int bladeMotorPin02 = 12;
const int liftMotorPin1 = 19;
const int liftMotorPin2 = 18;

const int rightWheelPwrPin = 5;
const int leftWheelPwrPin = 16;
const int rightWheelDirPin = 17;
const int leftWheelDirPin = 4;

const int stepperMotorStepPin = 27;
const int stepperMotorDirPin = 14;
const int stepperMotorSleepPin = 26;

Servo servo;         // サーボオブジェクトの定義
int servoPin = 21;   // サーボの制御ピン]
int servoAngle = 90; // 最初は正面向き
int servoDirection = 1;
int minUs = 500;  // 最小のパルス幅
int maxUs = 2400; // 最大のパルス幅

int rightWheelPwr = 0;
int leftWheelPwr = 0;
bool rightWheelDir = 0;
bool leftWheelDir = 0;

bool up = 0;
bool down = 0;
bool left = 0;
bool right = 0;
bool tri = 0;
bool cir = 0;
bool cross = 0;
bool square = 0;

int l2 = 0;
int r2 = 0;

int maxPwr = 250;
int state = 0;

int t = 0;
uint8_t readByte;
uint8_t buffer[3];
uint8_t data;

volatile bool triggered = true;

unsigned long stepHalfPeriod = 3000; // ステッピングモータの半周期時間（マイクロ秒）
unsigned long previousStepTime = 0;  // 最後にステッピングモータをステップさせた時間（マイクロ秒）
unsigned long currentTime = 0;       // 現在の時間（マイクロ秒）
bool stepPinState = false;           // ステッピングモータのステップピンの状態

bool slideMotorDirection = true; // true: 前進, false: 後退
bool slideMotorEnabled = false;  // スライドモータの状態（ON/OFF）

void clearSerialBuffer(HardwareSerial &serial)
{
  while (serial.available())
  {
    serial.read();
  }
}

void pinModeSetup()
{
  pinMode(rightWheelPwrPin, OUTPUT); // 右車輪のPWMピンを出力モードに設定
  pinMode(rightWheelDirPin, OUTPUT);
  pinMode(leftWheelPwrPin, OUTPUT);
  pinMode(leftWheelDirPin, OUTPUT);

  pinMode(bladeMotorPin01, OUTPUT);
  pinMode(bladeMotorPin02, OUTPUT);

  pinMode(stepperMotorStepPin, OUTPUT);
  pinMode(stepperMotorDirPin, OUTPUT);
  digitalWrite(stepperMotorDirPin, HIGH); // 初期状態を設定
  pinMode(stepperMotorSleepPin, OUTPUT);

  pinMode(liftMotorPin1, OUTPUT);
  pinMode(liftMotorPin2, OUTPUT);

  digitalWrite(rightWheelPwrPin, LOW);
  digitalWrite(leftWheelPwrPin, LOW);
  pinMode(leftSwitchPin, INPUT);
  pinMode(rightSwitchPin, INPUT);
  pinMode(relayInputPin, INPUT_PULLDOWN); // プルアップ入力
  pinMode(relayOutputPin, OUTPUT);        // 出力モード
  digitalWrite(relayOutputPin, LOW);      // 初期はLOW
}

void pwmSetup()
{
  ledcSetup(rightPwmCh, 12800, 8);             // チャンネル0、キャリア周波数1kHz、8ビットレンジ
  ledcAttachPin(rightWheelPwrPin, rightPwmCh); // PWMピンにチャンネル0を指定
  ledcSetup(leftPwmCh, 12800, 8);              // チャンネル1、キャリア周波数1kHz、16ビットレンジ
  ledcAttachPin(leftWheelPwrPin, leftPwmCh);   // PWMピンにチャンネル1を指定

  ledcSetup(bladeMotorCh, 12800, 8);            // チャンネル1、キャリア周波数1kHz、16ビットレンジ
  ledcAttachPin(bladeMotorPin01, bladeMotorCh); // PWMピンにチャンネル1を指定
  ledcAttachPin(bladeMotorPin02, bladeMotorCh); // PWMピンにチャンネル1を指定

  ledcSetup(liftMotorCh1, 12800, 8);          // チャンネル1、キャリア周波数1kHz、16ビットレンジ
  ledcAttachPin(liftMotorPin1, liftMotorCh1); //
  ledcSetup(liftMotorCh2, 12800, 8);          // チャンネル1、キャリア周波数1kHz、16ビットレンジ
  ledcAttachPin(liftMotorPin2, liftMotorCh2);
}

void servoSetup()
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);             // 50HzのPWMを出すという設定
  servo.attach(servoPin, minUs, maxUs); // servoオブジェクトに定数を設定していく。
}

void currentSensorSetup()
{
  Wire.begin(SDAPin, SCLPin);
  if (!ina260.begin(0x45, &Wire))
  {
    Serial.println("Couldn't find INA260 chip");
    while (1)
      ;
  }
  Serial.println("Found INA260 chip");
  if (!ina3221.begin(0x40, &Wire))
  { // can use other I2C addresses or buses
    Serial.println("Failed to find INA3221 chip");
    while (1)
      delay(10);
  }
  Serial.println("INA3221 Found!");
  if (!ina3221.begin(0x40, &Wire))
  { // can use other I2C addresses or buses
    Serial.println("Failed to find INA3221 chip");
    while (1)
      delay(10);
  }
  Serial.println("INA3221 Found!");

  ina3221.setAveragingMode(INA3221_AVG_16_SAMPLES);

  // Set shunt resistances for all channels to 0.05 ohms
  for (uint8_t i = 0; i < 3; i++)
  {
    ina3221.setShuntResistance(i, 0.05);
  }

  // Set a power valid alert to tell us if ALL channels are between the two
  // limits:
  ina3221.setPowerValidLimits(3.0 /* lower limit */, 15.0 /* upper limit */);
}

void currentSensorRead()
{
  Serial.print("Current: ");
  Serial.print(ina260.readCurrent());
  Serial.println(" mA");

  Serial.print("Bus Voltage: ");
  Serial.print(ina260.readBusVoltage());
  Serial.println(" mV");

  Serial.println();
  for (uint8_t i = 0; i < 3; i++)
  {
    float voltage = ina3221.getBusVoltage(i);
    float current = ina3221.getCurrentAmps(i) * 1000; // Convert to mA

    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(": Voltage = ");
    Serial.print(voltage, 2);
    Serial.print(" V, Current = ");
    Serial.print(current, 2);
    Serial.println(" mA");
  }
  Serial.println();
}
void currentSensorSendJetson()
{
  // INA260：mV → V、mA → A
  const float ina260Voltage_V =
      ina260.readBusVoltage() / 1000.0f;

  const float ina260Current_A =
      ina260.readCurrent() / 1000.0f;

  // INA3221：V、A
  const float ch0Voltage_V = ina3221.getBusVoltage(0);
  const float ch0Current_A = ina3221.getCurrentAmps(0);

  const float ch1Voltage_V = ina3221.getBusVoltage(1);
  const float ch1Current_A = ina3221.getCurrentAmps(1);

  const float ch2Voltage_V = ina3221.getBusVoltage(2);
  const float ch2Current_A = ina3221.getCurrentAmps(2);

  Serial1.print("PWR");
  Serial1.print(',');

  Serial1.print(sequenceNumber);
  Serial1.print(',');

  Serial1.print(millis());
  Serial1.print(',');

  Serial1.print(ina260Voltage_V, 3);
  Serial1.print(',');
  Serial1.print(ina260Current_A, 3);
  Serial1.print(',');

  Serial1.print(ch0Voltage_V, 3);
  Serial1.print(',');
  Serial1.print(ch0Current_A, 3);
  Serial1.print(',');

  Serial1.print(ch1Voltage_V, 3);
  Serial1.print(',');
  Serial1.print(ch1Current_A, 3);
  Serial1.print(',');

  Serial1.print(ch2Voltage_V, 3);
  Serial1.print(',');
  Serial1.print(ch2Current_A, 3);

  Serial1.println();

  sequenceNumber++;
}

void setup()
{
  pinModeSetup();
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, rxPin, txPin); // シリアル通信の初期化
  pwmSetup();
  servoSetup();
  currentSensorSetup();
  // attachInterrupt(digitalPinToInterrupt(relayInputPin), handleInterrupt, FALLING);
}

void liftMotorOn()
{
  if (cir == 1) // 上昇
  {
    ledcWrite(liftMotorCh1, 0);
    ledcWrite(liftMotorCh2, 200);
  }
  else if (square == 1) // 下降
  {
    ledcWrite(liftMotorCh1, 200);
    ledcWrite(liftMotorCh2, 0);
  }
  else
  {
    ledcWrite(liftMotorCh1, 0);
    ledcWrite(liftMotorCh2, 0);
  }
}

void slideMotorSetDirection()
{
  if (digitalRead(leftSwitchPin) == HIGH && digitalRead(rightSwitchPin) == LOW) // 右スイッチが押されたら
  {
    if (slideMotorDirection == 1) // 前進中なら
    {
      slideMotorDirection = 0;
      digitalWrite(stepperMotorDirPin, slideMotorDirection); // モータの回転方向を設定
    }
  }
  if (digitalRead(rightSwitchPin) == HIGH && digitalRead(leftSwitchPin) == LOW) // 左スイッチが押されたら
  {
    if (slideMotorDirection == 0) // 後退中なら
    {
      slideMotorDirection = 1;
      digitalWrite(stepperMotorDirPin, slideMotorDirection); // モータの回転方向を設定
    }
  }
}

void slideMotorOn()
{
  if (l2 > 0)
  {
    slideMotorEnabled = true; // スライドモータを有効にする
  }
  else
  {
    slideMotorEnabled = false; // スライドモータを無効にする
  }
  digitalWrite(stepperMotorSleepPin, slideMotorEnabled ? HIGH : LOW); // スリープモードの制御
}

void slideMotorPulseUpdate()
{
  if (!slideMotorEnabled)
  {
    stepPinState = false;
    digitalWrite(stepperMotorStepPin, LOW);
    digitalWrite(stepperMotorSleepPin, LOW); // スリープモードにする
    return;
  }
  currentTime = micros();
  if ((unsigned long)(currentTime - previousStepTime) >= stepHalfPeriod)
  {
    previousStepTime = currentTime;
    stepPinState = !stepPinState; // ステップピンの状態を反転
    digitalWrite(stepperMotorStepPin, stepPinState);
  }
}

void bladeMotorOn()
{
  ledcWrite(bladeMotorCh, r2);
}

void servoDrive()
{
  static unsigned long previousServoTime = 0;
  unsigned long now = millis();

  if (now - previousServoTime < 50)
  {
    return;
  }

  previousServoTime = now;

  if (servoAngle <= 70)
  {
    servoDirection = 1;
  }
  else if (servoAngle >= 130)
  {
    servoDirection = -1;
  }

  servoAngle += servoDirection;
  servo.write(servoAngle);
}

void WheelPwrOn()
{
  if (rightWheelPwr > 0)
  {
    rightWheelDir = 0;
    ledcWrite(rightPwmCh, rightWheelPwr);
    digitalWrite(rightWheelDirPin, rightWheelDir);
  }
  else
  {
    rightWheelDir = 1;
    ledcWrite(rightPwmCh, rightWheelPwr * (-1));
    digitalWrite(rightWheelDirPin, rightWheelDir);
  }
  if (leftWheelPwr > 0)
  {
    leftWheelDir = 0;
    ledcWrite(leftPwmCh, leftWheelPwr);
    digitalWrite(leftWheelDirPin, leftWheelDir);
  }
  else
  {
    leftWheelDir = 1;
    ledcWrite(leftPwmCh, leftWheelPwr * (-1));
    digitalWrite(leftWheelDirPin, leftWheelDir);
  }
}

void WheelPwrOff()
{
  rightWheelPwr = 0;
  leftWheelPwr = 0;
  ledcWrite(rightPwmCh, 0);
  ledcWrite(leftPwmCh, 0);
}

void getWheelPwr()
{
  if (up == 1)
  {
    rightWheelPwr += 5;
    leftWheelPwr += 5;
  }
  if (down == 1)
  {
    rightWheelPwr -= 5;
    leftWheelPwr -= 5;
  }
  if (right == 1)
  {
    if (rightWheelPwr * leftWheelPwr > 0) // 右車輪と左車輪の回転方向が同じ場合、右車輪のパワーを減らし、左車輪のパワーを増やして右折
    {
      rightWheelPwr -= 5;
      leftWheelPwr += 5;
    }
    else if (rightWheelPwr < 0)
    {                     // 後退中は左が0右がマイナスになる
      leftWheelPwr = 0;   // 念のため
      rightWheelPwr -= 5; // 左は0のまま、右車輪のパワーを増やして信地旋回
    }
    else
    {
      rightWheelPwr = 0; // 念のため
      leftWheelPwr += 5; // 左は0のまま、右車輪のパワーを増やして信地旋回
    }
  }

  if (left == 1)
  {
    if (rightWheelPwr * leftWheelPwr > 0) // 右車輪と左車輪の回転方向が同じ場合、右車輪のパワーを増やし、左車輪のパワーを減らして左折
    {
      rightWheelPwr += 5;
      leftWheelPwr -= 5;
    }
    else if (leftWheelPwr < 0)
    {                    // 後退中は右が0左がマイナスになる
      rightWheelPwr = 0; // 念のため
      leftWheelPwr -= 5; // 右車輪と左車輪の回転方向が逆の場合、右車輪のパワーを増やして信地旋回
    }
    else
    {
      leftWheelPwr = 0;   // 念のため
      rightWheelPwr += 5; // 右車輪と左車輪の回転方向が逆の場合、右車輪のパワーを増やして信地旋回
    }
  }
  if (cross == 1)
  {
    WheelPwrOff();
  }
}

void setWheelPwr()
{
  if (rightWheelPwr > maxPwr)
  {
    rightWheelPwr = maxPwr;
  }
  if (rightWheelPwr < -maxPwr)
  {
    rightWheelPwr = -maxPwr;
  }

  if (leftWheelPwr > maxPwr)
  {
    leftWheelPwr = maxPwr;
  }
  if (leftWheelPwr < -maxPwr)
  {
    leftWheelPwr = -maxPwr;
  }
  /* Serial.print("left : ");
  Serial.println(leftWheelPwr);
  Serial.print("right : ");
  Serial.println(rightWheelPwr); // for debug */
}

void emergency()
{
  triggered = true;
  digitalWrite(relayOutputPin, LOW);
  WheelPwrOff();
  slideMotorEnabled = false;
  l2 = 0;
  r2 = 0;
  ledcWrite(bladeMotorCh, 0);
  ledcWrite(liftMotorCh1, 0);
  ledcWrite(liftMotorCh2, 0);
  clearSerialBuffer(Serial1);
}

void receiveControllerData()
{
  bool frameReceived = false;

  while (Serial1.available() >= 4)
  {
    readByte = Serial1.read();

    if (readByte != 0xAA)
    {
      continue;
    }

    data = Serial1.read();
    l2 = Serial1.read();
    r2 = Serial1.read();

    up = (data >> 7) & 1;
    down = (data >> 6) & 1;
    left = (data >> 5) & 1;
    right = (data >> 4) & 1;
    tri = (data >> 3) & 1;
    cross = (data >> 2) & 1;
    square = (data >> 1) & 1;
    cir = data & 1;

    frameReceived = true;
  }

  // 今回、新しい正常フレームがなければ何もしない
  if (!frameReceived)
  {
    return;
  }

  // 複数フレームあった場合、最後の値に対して1回だけ実行
  getWheelPwr();
  setWheelPwr();

  WheelPwrOn();
  bladeMotorOn();
  liftMotorOn();
  slideMotorOn();
}

void debugPrint()

{
  static unsigned long previousPrintTime = 0;
  unsigned long now = millis();

  if (now - previousPrintTime >= 100)
  {
    previousPrintTime = now;
    // currentSensorRead();
    currentSensorSendJetson();
  }
}

void loop()
{
  if (digitalRead(relayInputPin) == 0) // スイッチが押された
  {
    emergency();
  }
  if (triggered && digitalRead(relayInputPin) == HIGH) // スイッチ押されてない
  {
    emergency(); // 緊急停止時の処理を一度実行して安全状態にする
    triggered = false;
    digitalWrite(relayOutputPin, HIGH); // 緊急停止解除
  }
  receiveControllerData();
  // Serial.println("receiveControllerData");
  slideMotorSetDirection();
  // Serial.println("slideMotorSetDirection");
  slideMotorPulseUpdate();
  // Serial.println("slideMotorPulseUpdate");

  servoDrive();
  // Serial.println("servoDrive");
  debugPrint();
  // Serial.println("debugPrint");
}