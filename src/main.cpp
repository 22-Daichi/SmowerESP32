#include <Arduino.h>
#include <ESP32Servo.h>

#define rightPwmCh 2
#define leftPwmCh 3
#define bladeMotorCh 4 // 草刈りモーター二つのパワーは一緒でいい
#define liftMotorCh1 5
#define liftMotorCh2 6

// pwmのch0,1は使えない
// servoのせい

const int rxPin = 22;
const int txPin = 23;

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

int motorB_direction = 1; // 0:後退 1:前進

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

void clearSerialBuffer(HardwareSerial &serial)
{
  while (serial.available())
  {
    serial.read();
  }
}

void pinModeSetup()
{
  pinMode(rightWheelPwrPin, OUTPUT);
  pinMode(rightWheelDirPin, OUTPUT);
  pinMode(leftWheelPwrPin, OUTPUT);
  pinMode(leftWheelDirPin, OUTPUT);

  pinMode(bladeMotorPin01, OUTPUT);
  pinMode(bladeMotorPin02, OUTPUT);

  pinMode(stepperMotorStepPin, OUTPUT);
  pinMode(stepperMotorDirPin, OUTPUT);
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

void setup()
{
  pinModeSetup();
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, rxPin, txPin); // RX=16, TX=17
  pwmSetup();
  servoSetup();
  // attachInterrupt(digitalPinToInterrupt(relayInputPin), handleInterrupt, FALLING);
}

void slideMotorSetDirection()
{
  if (digitalRead(leftSwitchPin == HIGH) && digitalRead(rightSwitchPin) == LOW) // 右スイッチが押されたら
  {
    if (motorB_direction == 1) // 前進中なら
    {
      motorB_direction = 0;
    }
  }
  if (digitalRead(rightSwitchPin) == HIGH && digitalRead(leftSwitchPin) == LOW) // 左スイッチが押されたら
  {
    if (motorB_direction == 0) // 後退中なら
    {
      motorB_direction = 1;
    }
  }
}

void slideMotorOn()
{
  /* if (r2 < 0)
  {
    r2 += 256; // これはわからん。けしてもいいと信じている。jetsonからくるr2の値を確認。
  }
  if (motorB_direction == 1) // 前進
  {
    ledcWrite(slideMotorCh1, r2);
    ledcWrite(slideMotorCh2, 0);
  }
  else // 後退
  {
    ledcWrite(slideMotorCh1, 0);
    ledcWrite(slideMotorCh2, r2);
  } */
}

void bladeMotorOn()
{
  if (l2 < 0)
  {
    l2 += 256; // これはわからん。けしてもいいと信じている。jetsonからくるl2の値を確認。
  }
  ledcWrite(bladeMotorCh, l2);
}

void servoDrive()
{
  if (servoAngle == 70)
  {
    servoDirection = 1;
  }
  else if (servoAngle == 130)
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
    rightWheelPwr -= 5;
    leftWheelPwr += 5;
  }

  if (left == 1)
  {
    rightWheelPwr += 5;
    leftWheelPwr -= 5;
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
}

void loop()
{
  slideMotorSetDirection();
  if (digitalRead(relayInputPin) == 0) // スイッチが押された
  {
    emergency();
  }
  if (triggered && digitalRead(relayInputPin) == HIGH) // スイッチ押されてない
  {
    triggered = false;
    digitalWrite(relayOutputPin, HIGH);
  }
  if (Serial1.available())
  {
    readByte = Serial1.read();
    if (readByte == 0xAA && Serial1.available() >= 3)
    { // スタートバイト検出
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
      cir = (data >> 0) & 1;
      getWheelPwr();
      setWheelPwr();
      WheelPwrOn();
      bladeMotorOn();
      slideMotorOn();
    }
    else
    {
      clearSerialBuffer(Serial1);
    }
  }
  servoDrive();
  Serial.print(l2);
  Serial.println(r2);
  delay(50);
}