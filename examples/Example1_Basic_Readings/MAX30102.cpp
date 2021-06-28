/**************************************************
  This is a library written for the Maxim MAX30102 Optical Smoke Detector
  It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.

  These sensors use I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.

  Written by Peter Jansen and Nathan Seidle (SparkFun)
  BSD license, all text above must be included in any redistribution.
 *****************************************************/

#include "MAX30102.h"

// Status Registers 状态寄存器
static const uint8_t MAX30102_INTSTAT1 =		0x00;
static const uint8_t MAX30102_INTSTAT2 =		0x01;
static const uint8_t MAX30102_INTENABLE1 =		0x02;
static const uint8_t MAX30102_INTENABLE2 =		0x03;

// FIFO Registers FIFO寄存器
static const uint8_t MAX30102_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30102_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30102_FIFOREADPTR = 	0x06;
static const uint8_t MAX30102_FIFODATA =		0x07;

// Configuration Registers 配置寄存器
static const uint8_t MAX30102_FIFOCONFIG = 		0x08;
static const uint8_t MAX30102_MODECONFIG = 		0x09;
static const uint8_t MAX30102_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30102_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30102_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30102_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30102_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30102_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers 模具温度寄存器
static const uint8_t MAX30102_DIETEMPINT = 		0x1F;
static const uint8_t MAX30102_DIETEMPFRAC = 	0x20;
static const uint8_t MAX30102_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers  接近功能寄存器
static const uint8_t MAX30102_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const uint8_t MAX30102_REVISIONID = 		0xFE;
static const uint8_t MAX30102_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30102 Commands MAX30102命令
// Interrupt configuration (pg 13, 14) 中断配置
static const uint8_t MAX30102_INT_A_FULL_MASK =		(byte)~0b10000000;
static const uint8_t MAX30102_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30102_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX30102_INT_DATA_RDY_MASK = (byte)~0b01000000;
static const uint8_t MAX30102_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30102_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_INT_ALC_OVF_MASK = (byte)~0b00100000;
static const uint8_t MAX30102_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30102_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30102_INT_PROX_INT_MASK = (byte)~0b00010000;
static const uint8_t MAX30102_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30102_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30102_INT_DIE_TEMP_RDY_MASK = (byte)~0b00000010;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_SAMPLEAVG_MASK =	(byte)~0b11100000;
static const uint8_t MAX30102_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30102_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30102_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30102_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30102_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30102_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30102_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30102_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30102_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30102_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19) 模式配置命令
static const uint8_t MAX30102_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30102_SHUTDOWN = 		0x80;
static const uint8_t MAX30102_WAKEUP = 			0x00;

static const uint8_t MAX30102_RESET_MASK = 		0xBF;
static const uint8_t MAX30102_RESET = 			0x40;

static const uint8_t MAX30102_MODE_MASK = 		0xF8;
static const uint8_t MAX30102_MODE_REDONLY = 	0x02;
static const uint8_t MAX30102_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30102_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20) 粒子感应配置命令
static const uint8_t MAX30102_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30102_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30102_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30102_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30102_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30102_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30102_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30102_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30102_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30102_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30102_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30102_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30102_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30102_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30102_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30102_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30102_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30102_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30102_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22) 多LED模式配置
static const uint8_t MAX30102_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30102_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30102_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30102_SLOT4_MASK = 		0x8F;

static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_NONE_PILOT = 			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT = 			0x06;

static const uint8_t MAX_30102_EXPECTEDPARTID = 0x15;

MAX30102::MAX30102() {
  // Constructor
}

boolean MAX30102::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr) {

  _i2cPort = &wirePort; //Grab which port the user wants us to use抓住用户希望我们使用的端口

  _i2cPort->begin();

  _i2caddr = i2caddr;

  // Step 1: Initial Communication and Verification 步骤1：初步沟通和验证
  // Check that a MAX30102 is connected 检查是否已连接MAX30102
  if (readPartID() != MAX_30102_EXPECTEDPARTID) {
    // Error -- Part ID read from MAX30102 does not match expected part ID.错误-从MAX30102读取的部件ID与预期的部件ID不匹配。
    // This may mean there is a physical connectivity problem (broken wire, unpowered, etc).这可能意味着存在物理连接问题（断线，未供电等）。
    return false;
  }

  // Populate revision ID填写修订版ID
  readRevisionID();
  
  return true;
}



//Begin Interrupt configuration开始中断配置
uint8_t MAX30102::getINT1(void) {
  return (readRegister8(_i2caddr, MAX30102_INTSTAT1));
}
uint8_t MAX30102::getINT2(void) {
  return (readRegister8(_i2caddr, MAX30102_INTSTAT2));
}

void MAX30102::enableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_ENABLE);
}
void MAX30102::disableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

void MAX30102::enableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}
void MAX30102::disableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

void MAX30102::enableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}
void MAX30102::disableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

void MAX30102::enablePROXINT(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_ENABLE);
}
void MAX30102::disablePROXINT(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_DISABLE);
}

void MAX30102::enableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30102::disableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}

//End Interrupt configuration结束中断配置

void MAX30102::softReset(void) {
  bitMask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);

  // Poll for bit to clear, reset is then complete轮询清除，然后重置完成
  // Timeout after 100ms  100ms后超时
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    uint8_t response = readRegister8(_i2caddr, MAX30102_MODECONFIG);
    if ((response & MAX30102_RESET) == 0) break; //We're done!完成 
    delay(1); //Let's not over burden the I2C bus不要过分负担I2C总线
  }
}

void MAX30102::shutDown(void) {
  // Put IC into low power mode (datasheet pg. 19)将IC置于低功耗模式
  // During shutdown the IC will continue to respond to I2C commands but will关断期间，IC将继续响应I2C命令，但会
  // not update with or take new readings (such as temperature)不更新或获取新的读数
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

void MAX30102::wakeUp(void) {
  // Pull IC out of low power mode (datasheet pg. 19)将IC退出低功耗模式（数据表第19页）
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

void MAX30102::setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.设置用于采样的LED-仅红色，仅RED + IR或自定义。
  // See datasheet, page 19
  bitMask(MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);
}

void MAX30102::setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30102_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}

void MAX30102::setSampleRate(uint8_t sampleRate) {
  // sampleRate采样率: one of MAX30102_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

void MAX30102::setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth脉宽: one of MAX30102_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values振幅值: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX30102::setPulseAmplitudeRed(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30102_LED1_PULSEAMP, amplitude);
}

void MAX30102::setPulseAmplitudeIR(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30102_LED2_PULSEAMP, amplitude);
}

void MAX30102::setPulseAmplitudeProximity(uint8_t amplitude) {
  writeRegister8(_i2caddr, MAX30102_LED_PROX_AMP, amplitude);
}

void MAX30102::setProximityThreshold(uint8_t threshMSB) {
  // Set the IR ADC count that will trigger the beginning of particle-sensing mode.设置将触发颗粒感测模式开始的IR ADC计数。
  // The threshMSB signifies only the 8 most significant-bits of the ADC count. threshMSB仅表示ADC计数的8个最高有效位。
  // See datasheet, page 24.
  writeRegister8(_i2caddr, MAX30102_PROXINTTHRESH, threshMSB);
}

//Given a slot number assign a thing to it给定插槽号，为其分配一个东西
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)设备为SLOT_RED_LED或SLOT_RED_PILOT（邻近）
//Assigning a SLOT_RED_LED will pulse LED 分配SLOT_RED_LED将使LED闪烁
//Assigning a SLOT_RED_PILOT will ??
void MAX30102::enableSlot(uint8_t slotNumber, uint8_t device) {

  uint8_t originalContents;

  switch (slotNumber) {
    case (1):
      bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

//Clears all slot assignments清除所有插槽分配
void MAX30102::disableSlots(void) {
  writeRegister8(_i2caddr, MAX30102_MULTILEDCONFIG1, 0);
  writeRegister8(_i2caddr, MAX30102_MULTILEDCONFIG2, 0);
}

//
// FIFO Configuration FIFO配置
//

//Set sample average (Table 3, Page 18) 设定样本平均值
void MAX30102::setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state 重置所有点以在已知状态下开始
//Page 15 recommends clearing FIFO before beginning a read 第15页建议在开始读取之前清除FIFO
void MAX30102::clearFIFO(void) {
  writeRegister8(_i2caddr, MAX30102_FIFOWRITEPTR, 0);
  writeRegister8(_i2caddr, MAX30102_FIFOOVERFLOW, 0);
  writeRegister8(_i2caddr, MAX30102_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows如果FIFO溢出则启用翻转
void MAX30102::enableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows如果FIFO溢出则禁用翻转
void MAX30102::disableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)设置样本数量以触发几乎全部中断（页18）
//Power on default is 32 samples开机默认为32个样本
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples  0x00是32个样本，0x0F是17个样本
void MAX30102::setFIFOAlmostFull(uint8_t numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

//Read the FIFO Write Pointer读取FIFO写指针
uint8_t MAX30102::getWritePointer(void) {
  return (readRegister8(_i2caddr, MAX30102_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer读取FIFO读指针
uint8_t MAX30102::getReadPointer(void) {
  return (readRegister8(_i2caddr, MAX30102_FIFOREADPTR));
}


// Die Temperature温度 
// Returns temp in C 以C返回温度
float MAX30102::readTemperature() {
	
  //DIE_TEMP_RDY interrupt must be enabled 必须启用DIE_TEMP_RDY中断
  //See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19
  
  // Step 1: Config die temperature register to take 1 temperature sample 步骤1：配置芯片温度寄存器以获取1个温度样本
  writeRegister8(_i2caddr, MAX30102_DIETEMPCONFIG, 0x01);

  // Poll for bit to clear, reading is then complete一点点清除，然后阅读完成
  // Timeout after 100ms 100ms后超时
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    
	//Check to see if DIE_TEMP_RDY interrupt is set 检查是否设置了DIE_TEMP_RDY中断
	uint8_t response = readRegister8(_i2caddr, MAX30102_INTSTAT2);
    if ((response & MAX30102_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!完成 
    delay(1); //Let's not over burden the I2C bus不要过分负担I2C总线
  }
  //TODO How do we want to fail? With what type of error?TODO我们要如何失败？ 有什么类型的错误？
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)步骤2：读取芯片温度寄存器（整数）
  int8_t tempInt = readRegister8(_i2caddr, MAX30102_DIETEMPINT);
  uint8_t tempFrac = readRegister8(_i2caddr, MAX30102_DIETEMPFRAC); //Causes the clearing of the DIE_TEMP_RDY interrupt导致清除DIE_TEMP_RDY中断

  // Step 3: Calculate temperature (datasheet pg. 23)计算温度
  return (float)tempInt + ((float)tempFrac * 0.0625);
}

// Returns die temp in F 返回F中的模具温度
float MAX30102::readTemperatureF() {
  float temp = readTemperature();

  if (temp != -999.0) temp = temp * 1.8 + 32.0;

  return (temp);
}

// Set the PROX_INT_THRESHold设置PROX_INT_THRESHold
void MAX30102::setPROXINTTHRESH(uint8_t val) {
  writeRegister8(_i2caddr, MAX30102_PROXINTTHRESH, val);
}


//
// Device ID and 设备ID和修订
//
uint8_t MAX30102::readPartID() {
  return readRegister8(_i2caddr, MAX30102_PARTID);
}

void MAX30102::readRevisionID() {
  revisionID = readRegister8(_i2caddr, MAX30102_REVISIONID);
}

uint8_t MAX30102::getRevisionID() {
  return revisionID;
}


//Setup the sensor设置传感器
//The MAX30102 has many settings. By default we select: MAX30102具有许多设置。 默认情况下，我们选择：
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the MAX30102 sensor
void MAX30102::setup(byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange) {
  softReset(); //Reset all configuration, threshold, and data registers to POR values将所有配置，阈值和数据寄存器重置为POR值

  //FIFO Configuration FIFO配置
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //The chip will average multiple samples of same type together if you wish如果您愿意，该芯片将对多个相同类型的样本求平均
  if (sampleAverage == 1) setFIFOAverage(MAX30102_SAMPLEAVG_1); //No averaging per FIFO record 每个FIFO记录无平均值
  else if (sampleAverage == 2) setFIFOAverage(MAX30102_SAMPLEAVG_2);
  else if (sampleAverage == 4) setFIFOAverage(MAX30102_SAMPLEAVG_4);
  else if (sampleAverage == 8) setFIFOAverage(MAX30102_SAMPLEAVG_8);
  else if (sampleAverage == 16) setFIFOAverage(MAX30102_SAMPLEAVG_16);
  else if (sampleAverage == 32) setFIFOAverage(MAX30102_SAMPLEAVG_32);
  else setFIFOAverage(MAX30102_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover(); //Allow FIFO to wrap/roll over 允许FIFO包装/翻转
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration模式配置
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 3) setLEDMode(MAX30102_MODE_MULTILED); //Watch all three LED channels观看所有三个LED通道
  else if (ledMode == 2) setLEDMode(MAX30102_MODE_REDIRONLY); //Red and IR
  else setLEDMode(MAX30102_MODE_REDONLY); //Red only
  activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer 用于控制要从FIFO缓冲区读取的字节数
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration 颗粒感测配置
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if(adcRange < 4096) setADCRange(MAX30102_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < 8192) setADCRange(MAX30102_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < 16384) setADCRange(MAX30102_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == 16384) setADCRange(MAX30102_ADCRANGE_16384); //62.5pA per LSB
  else setADCRange(MAX30102_ADCRANGE_2048);

  if (sampleRate < 100) setSampleRate(MAX30102_SAMPLERATE_50); //Take 50 samples per second
  else if (sampleRate < 200) setSampleRate(MAX30102_SAMPLERATE_100);
  else if (sampleRate < 400) setSampleRate(MAX30102_SAMPLERATE_200);
  else if (sampleRate < 800) setSampleRate(MAX30102_SAMPLERATE_400);
  else if (sampleRate < 1000) setSampleRate(MAX30102_SAMPLERATE_800);
  else if (sampleRate < 1600) setSampleRate(MAX30102_SAMPLERATE_1000);
  else if (sampleRate < 3200) setSampleRate(MAX30102_SAMPLERATE_1600);
  else if (sampleRate == 3200) setSampleRate(MAX30102_SAMPLERATE_3200);
  else setSampleRate(MAX30102_SAMPLERATE_50);

  //The longer the pulse width the longer range of detection you'll have脉冲宽度越长，检测范围就越长
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118) setPulseWidth(MAX30102_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution为我们提供15位分辨率
  else if (pulseWidth < 215) setPulseWidth(MAX30102_PULSEWIDTH_118); //16 bit resolution
  else if (pulseWidth < 411) setPulseWidth(MAX30102_PULSEWIDTH_215); //17 bit resolution
  else if (pulseWidth == 411) setPulseWidth(MAX30102_PULSEWIDTH_411); //18 bit resolution
  else setPulseWidth(MAX30102_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude ConfigurationLED脉冲幅度配置
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is 0x1F which gets us 6.4mA 默认值为0x1F，可获取6.4mA电流
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
  setPulseAmplitudeProximity(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Multi-LED Mode Configuration, Enable the reading of the three LEDs 多LED模式配置，启用三个LED的读取
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1) enableSlot(2, SLOT_IR_LED);
  //enableSlot(1, SLOT_RED_PILOT);
  //enableSlot(2, SLOT_IR_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  clearFIFO(); //Reset the FIFO before we begin checking the sensor开始检查传感器之前，请重置FIFO
}

//
// Data Collection数据采集
//

//Tell caller how many samples are available告诉来电者有多少样品可用
uint8_t MAX30102::available(void)
{
  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

//Report the most recent red value报告最新的红色值
uint32_t MAX30102::getRed(void)
{
  //Check the sensor for new data for 250ms 在250ms内检查传感器是否有新数据
  if(safeCheck(250))
    return (sense.red[sense.head]);
  else
    return(0); //Sensor failed to find new data传感器找不到新数据
}

//Report the most recent IR value报告最新的IR值
uint32_t MAX30102::getIR(void)
{
  //Check the sensor for new data for 250ms 在250ms内检查传感器是否有新数据
  if(safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data传感器找不到新数据
}


//Report the next Red value in the FIFO 报告FIFO中的下一个红色值
uint32_t MAX30102::getFIFORed(void)
{
  return (sense.red[sense.tail]);
}

//Report the next IR value in the FIFO报告FIFO中的下一个IR值
uint32_t MAX30102::getFIFOIR(void)
{
  return (sense.IR[sense.tail]);
}


//Advance the tail前进尾巴
void MAX30102::nextSample(void)
{
  if(available()) //Only advance the tail if new data is available仅当有新数据可用时才尾巴突出
  {
    sense.tail++;
    sense.tail %= STORAGE_SIZE; //Wrap condition包装条件
  }
}

//Polls the sensor for new data轮询传感器以获取新数据
//Call regularly定期致电
//If new data is available, it updates the head and tail in the main struct如果有新数据可用，它将更新主结构中的头和尾
//Returns number of new samples obtained返回获得的新样本数
uint16_t MAX30102::check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks 以（3字节*活动LED的数量）块读取寄存器FIDO_DATA
  //Until FIFO_RD_PTR = FIFO_WR_PTR 直到FIFO_RD_PTR = FIFO_WR_PTR

  byte readPointer = getReadPointer();
  byte writePointer = getWritePointer();

  int numberOfSamples = 0;

  //是否有新数据 
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor计算我们需要从传感器获取的读数数量
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition包装条件

    //We now have the number of readings, now calc bytes to read现在我们有了读数的数量，现在有要读取的计算字节
    //For this example we are just doing Red and IR (3 bytes each)只是在进行红色和IR（每个3字节）
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register 准备好从FIFO寄存器读取突发数据
    _i2cPort->beginTransmission(MAX30102_ADDRESS);
    _i2cPort->write(MAX30102_FIFODATA);
    _i2cPort->endTransmission();

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH可能需要读取多达288个字节，因此我们读取的块不超过I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.I2C_BUFFER_LENGTH会根据平台进行更改。
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the UnoWire.requestFrom（）限制为BUFFER_LENGTH，在Uno上为32
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time如果toGet为32，则这很不好，因为我们一次读取6个字节（Red + IR * 3 = 6）
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30. 32％6 = 2剩余。 我们不想请求32个字节，我们想请求30个字节。
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27. 32％9（红色+ IR +绿色）= 5剩余。 我们想要求27。

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read修剪成为我们需要阅读的样本的倍数
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor请求从传感器获取字节数
      _i2cPort->requestFrom(MAX30102_ADDRESS, toGet);
      
      while (toGet > 0)
      {
        sense.head++; //Advance the head of the storage struct推进存储结构的负责人
        sense.head %= STORAGE_SIZE; //Wrap condition包装条件

        byte temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long我们将转换为long的4个字节的数组
        uint32_t tempLong;

        //Burst read three bytes - RED突发读取三个字节-红色
        temp[3] = 0;
        temp[2] = _i2cPort->read();
        temp[1] = _i2cPort->read();
        temp[0] = _i2cPort->read();

        //Convert array to long将数组转换为long
        memcpy(&tempLong, temp, sizeof(tempLong));
		
		tempLong &= 0x3FFFF; //Zero out all but 18 bits将18位以外的所有位清零

        sense.red[sense.head] = tempLong; //Store this reading into the sense array将此读数存储到Sense数组中

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - IR突发读取另外三个字节-IR
          temp[3] = 0;
          temp[2] = _i2cPort->read();
          temp[1] = _i2cPort->read();
          temp[0] = _i2cPort->read();

          //Convert array to long将数组转换为long
          memcpy(&tempLong, temp, sizeof(tempLong));

		  tempLong &= 0x3FFFF; //Zero out all but 18 bits将18位以外的所有位清零
          
		  sense.IR[sense.head] = tempLong;
        }

        toGet -= activeLEDs * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found让全世界知道我们发现了多少新数据
}

//Check for new data but give up after a certain amount of time检查新数据，但在一定时间后放弃
//Returns true if new data was found如果找到新数据，则返回true
//Returns false if new data was not found如果未找到新数据，则返回false
bool MAX30102::safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t markTime = millis();
  
  while(1)
  {
	if(millis() - markTime > maxTimeToCheck) return(false);

	if(check() == true) //发现数据
	  return(true);

	delay(1);
  }
}

//Given a register, read it, mask it, and then set the thing给定一个寄存器，读取它，对其进行遮罩，然后进行设置
void MAX30102::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context 抓取当前寄存器上下文
  uint8_t originalContents = readRegister8(_i2caddr, reg);

  // Zero-out the portions of the register we're interested in将我们感兴趣的寄存器部分清零
  originalContents = originalContents & mask;

  // Change contents变更内容
  writeRegister8(_i2caddr, reg, originalContents | thing);
}

//
// Low-level I2C Communication
//
uint8_t MAX30102::readRegister8(uint8_t address, uint8_t reg) {
  _i2cPort->beginTransmission(address);
  _i2cPort->write(reg);
  _i2cPort->endTransmission(false);

  _i2cPort->requestFrom((uint8_t)address, (uint8_t)1); // Request 1 byte
  if (_i2cPort->available())
  {
    return(_i2cPort->read());
  }

  return (0); //Fail

}

void MAX30102::writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
  _i2cPort->beginTransmission(address);
  _i2cPort->write(reg);
  _i2cPort->write(value);
  _i2cPort->endTransmission();
}
