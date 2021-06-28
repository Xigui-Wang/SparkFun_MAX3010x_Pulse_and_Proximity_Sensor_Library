/*
  Optical SP02 Detection (SPK Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 19th, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This demo shows heart rate and SPO2 levels.

  It is best to attach the sensor to your finger using a rubber band or other tightening 
  device. Humans are generally bad at applying constant pressure to a thing. When you 
  press your finger against the sensor it varies enough to cause the blood in your 
  finger to flow differently which causes the sensor readings to go wonky.

  This example is based on MAXREFDES117 and RD117_LILYPAD.ino from Maxim. Their example
  was modified to work with the SparkFun MAX30105 library and to compile under Arduino 1.6.11
  Please see license file for more info.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
 
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <Wire.h>
#include "MAX30102.h"
#include "spo2_algorithm.h"

MAX30102 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //红外LED传感器数据
uint16_t redBuffer[100];  //红色LED传感器数据
#else
uint32_t irBuffer[100]; //红外LED传感器数据
uint32_t redBuffer[100];  //红色LED传感器数据
#endif

int32_t bufferLength; //缓冲器数据长度 
int32_t spo2; //SPO2值
int8_t validSPO2; //显示SPO2计算是否有效
int32_t heartRate; //心率值 
int8_t validHeartRate; //显示心率计算是否有效

byte pulseLED = 11; //必须在PWM引脚上
byte readLED = 13; //每次读取数据时闪烁

void setup()
{
  Serial.begin(115200); // 以每秒115200位的速度初始化串行通信

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // 初始化传感器
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //使用默认的I2C端口，400kHz速度
  {
    Serial.println(F("MAX30102 was not found. Please check wiring/power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //等到用户按下一个键
  Serial.read();

  byte ledBrightness = 60; //选项: 0=Off to 255=50mA
  byte sampleAverage = 4; //选项: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //选项: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //选项: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //选项: 69, 118, 215, 411
  int adcRange = 4096; //选项: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop()
{
  bufferLength = 100; //缓冲区长度为100，以25sps的速度存储4秒的样本

  //读取前100个样本，并确定信号范围
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //是否有新数据 
      particleSensor.check(); //检查传感器是否有新数据

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //我们已经完成了此样本，因此请移至下一个样本

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //在前100次采样（前4秒采样）后计算心率和SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //从MAX30102连续采样。 每1秒计算一次心率和SpO2
  while (1)
  {
    //将前25组样本转储到内存中，并将后75组样本移至顶部
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //在计算心率之前先采集25组样本。
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //是否有新数据 
        particleSensor.check(); //检查传感器是否有新数据

      digitalWrite(readLED, !digitalRead(readLED)); //每次读取数据时，板上LED闪烁

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //我们已经完成了此样本，因此请移至下一个样本

      //通过UART发送样本和计算结果到终端程序
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    //收集25个新样本后，重新计算HR和SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}
