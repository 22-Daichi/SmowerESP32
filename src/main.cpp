#include <Arduino.h>
#include <ESP32Servo.h>

#define rightPwmCh 2
#define leftPwmCh 3
// pwmのch0,1は使えない
// servoのせい

const int rxPin = 22;
const int txPin = 23;

const int inputPin = 15; // 入力ピン（pullvdown）
const int outputPin = 2; // 出力ピン

Servo servo;         // サーボオブジェクトの定義
int servoPin = 21;   // サーボの制御ピン]
int servoAngle = 90; // 最初は正面向き
int servoDirection = 1;
int minUs = 500;  // 最小のパルス幅
int maxUs = 2400; // 最大のパルス幅

const int rightWheelPwrPin = 5;
const int leftWheelPwrPin = 16;
const int rightWheelDirPin = 17;
const int leftWheelDirPin = 4;

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

int maxPwr = 250;

int t = 0;
uint8_t data;

volatile bool triggered = true;

void pinModeSetup()
{
  pinMode(rightWheelPwrPin, OUTPUT);
  pinMode(rightWheelDirPin, OUTPUT);
  pinMode(leftWheelPwrPin, OUTPUT);
  pinMode(leftWheelDirPin, OUTPUT);
  digitalWrite(rightWheelPwrPin, LOW);
  digitalWrite(leftWheelPwrPin, LOW);
  pinMode(inputPin, INPUT_PULLDOWN); // プルアップ入力
  pinMode(outputPin, OUTPUT);        // 出力モード
  digitalWrite(outputPin, LOW);      // 初期はLOW
}

void pwmSetup()
{
  ledcSetup(rightPwmCh, 12800, 8);             // チャンネル0、キャリア周波数1kHz、8ビットレンジ
  ledcAttachPin(rightWheelPwrPin, rightPwmCh); // PWMピンにチャンネル0を指定
  ledcSetup(leftPwmCh, 12800, 8);              // チャンネル1、キャリア周波数1kHz、16ビットレンジ
  ledcAttachPin(leftWheelPwrPin, leftPwmCh);   // PWMピンにチャンネル1を指定
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
  // attachInterrupt(digitalPinToInterrupt(inputPin), handleInterrupt, FALLING);
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
  if (cir == 1)
  {
    rightWheelPwr -= 5;
    leftWheelPwr += 5;
  }

  if (square == 1)
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
  digitalWrite(outputPin, LOW);
  WheelPwrOff();
}

void loop()
{
  if (digitalRead(inputPin) == 0) // スイッチが押された
  {
    emergency();
  }
  if (triggered && digitalRead(inputPin) == HIGH) // スイッチ押されてない
  {
    triggered = false;
    digitalWrite(outputPin, HIGH);
  }
  if (Serial1.available())
  {
    // Serial.println("DataReceived");
    data = Serial1.read();
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
  }
  servoDrive();
  delay(50);
}