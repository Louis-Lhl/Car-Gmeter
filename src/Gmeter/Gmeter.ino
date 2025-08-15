/*
 * Car G Meter for Arduino UNO
 *
 * 使用 MPU9xxx 三轨加速径传感器和 0.96\" I2C OLED 显示屏，测量并显示车辆的纵向、横向和垂向加速度。
 * 特性：
 *  - 自动探测 MPU 地址（0x68 或 0x69）和 OLED 地址（0x3C 或 0x3D）
 *  - 设置加速轨量罩为 ±4g（可根据需要调整）
 *  - 上电静置自动校准零偏（包含重力）
 *  - 显示即时 X/Y/Z 轨加速度、水平合成 |H|，以及峰值
 *  - 简易按钮（D2）复位峰值
 *
 * 说明：
 *  - 建议传感器模块供电 3.3V。如模块标记支持 3–5V，则可直接接 5V。
 *  - OLED 模块通常支持 3.3–5V；若不确定则接 5V。
 *  - 安装方向：X 朝前、Y 朝左、Z 朝上。
 *  - 使用时请确保上电时车辆静止，保持水平以便正确校准。
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// OLED 显示实例，I2C 接口
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// 默认地址，运行时会探测
uint8_t OLED_ADDR = 0x3C;
uint8_t mpuAddr   = 0x68;

// MPU 存储器
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

// 按钮管脚（用于清零峰值，可选）
const int PIN_BTN = 2;

// ±4g 量罩下的 LSB/g
const float ACC_SCALE = 8192.0f;

// 简易低通滤波系数 (0【1)，越小越平滑
const float ALPHA = 0.20f;

float ax_g=0, ay_g=0, az_g=0;        // 当前加速度 (g)
float ax_f=0, ay_f=0, az_f=0;        // 滤波后
float ax_off=0, ay_off=0, az_off=0;  // 零偏 (g)

float peak_horiz = 0.0f;

bool buttonLast = HIGH;
unsigned long lastDebounce = 0;

// 检测 I2C 设备是否存在
bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// 写 MPU 存储器
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// 读 MPU 多字节
void mpuReadBytes(uint8_t startReg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(mpuAddr, (uint8_t)len);
  for (size_t i=0; i<len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

// 初始化 MPU：探测地址，设罩量罩
bool initMPU() {
  // 探测 0x68 或 0x69
  if (i2cDevicePresent(0x68)) mpuAddr = 0x68;
  else if (i2cDevicePresent(0x69)) mpuAddr = 0x69;
  else return false;
  // 唤醒
  mpuWrite(REG_PWR_MGMT_1, 0x00);
  delay(50);
  // 设罩 ±4g (FS_SEL = 1)
  uint8_t cur=0;
  mpuReadBytes(REG_ACCEL_CONFIG, &cur, 1);
  cur &= ~0x18;
  cur |= (1 << 3);
  mpuWrite(REG_ACCEL_CONFIG, cur);
  delay(10);
  return true;
}

// 读取加速度（单位:g）
void readAccelG(float &ax, float &ay, float &az) {
  uint8_t buf[6];
  mpuReadBytes(REG_ACCEL_XOUT_H, buf, 6);
  int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
  ax = (float)ax_raw / ACC_SCALE;
  ay = (float)ay_raw / ACC_SCALE;
  az = (float)az_raw / ACC_SCALE;
}

// 校准零偏：静止采样若干次
void calibrateOffsets(unsigned samples=500, unsigned msWait=5) {
  float sx=0, sy=0, sz=0;
  for (unsigned i=0; i<samples; i++) {
    float x,y,z;
    readAccelG(x,y,z);
    sx += x; sy += y; sz += z;
    delay(msWait);
  }
  ax_off = sx / samples;
  ay_off = sy / samples;
  az_off = sz / samples;
}

// 绘制水平条形图（中心为零，左右两侧代表±g）
void drawBarH(int x, int y, int w, int h, float gVal, float gFullScale) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int mid = x + w/2;
  int span = (int)((gVal / gFullScale) * (w/2));
  if (span >  w/2) span =  w/2;
  if (span < -w/2) span = -w/2;
  if (span >= 0) {
    display.fillRect(mid, y+1, span, h-2, SSD1306_WHITE);
  } else {
    display.fillRect(mid+span, y+1, -span, h-2, SSD1306_WHITE);
  }
  // ±1g 制度
  int tick = (w/2)/gFullScale;
  display.drawFastVLine(mid + tick*1, y, h, SSD1306_WHITE);
  display.drawFastVLine(mid - tick*1, y, h, SSD1306_WHITE);
}

void setup() {
  pinMode(PIN_BTN, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(400000);
  // OLED 初始化（尝试 0x3C 或 0x3D）
  if (!i2cDevicePresent(OLED_ADDR)) {
    OLED_ADDR = 0x3D;
  }
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Car G-Meter"));
  display.display();
  // 初始化 MPU
  if (!initMPU()) {
    display.setCursor(0, 16);
    display.println(F("MPU NOT FOUND"));
    display.display();
    while (1) { delay(1000); }
  }
  // 校准：静置 2.5s
  display.setCursor(0, 16);
  display.println(F("Calibrating..."));
  display.display();
  calibrateOffsets();
  display.clearDisplay();
  display.display();
}

void loop() {
  float x,y,z;
  readAccelG(x,y,z);
  // 去零偏
  ax_g = x - ax_off;
  ay_g = y - ay_off;
  az_g = z - az_off;
  // EMA 滤波
  ax_f = ALPHA*ax_g + (1-ALPHA)*ax_f;
  ay_f = ALPHA*ay_g + (1-ALPHA)*ay_f;
  az_f = ALPHA*az_g + (1-ALPHA)*az_f;
  // 水平向量合成
  float g_horiz = sqrtf(ax_f*ax_f + ay_f*ay_f);
  if (g_horiz > peak_horiz) peak_horiz = g_horiz;
  // 按钮复位峰值
  bool b = digitalRead(PIN_BTN);
  if (b != buttonLast) {
    lastDebounce = millis();
    buttonLast = b;
  }
  if (buttonLast == LOW && (millis() - lastDebounce) > 30) {
    peak_horiz = 0.0f;
  }
  // OLED 显示
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(F("X(Long): ")); display.print(ax_f, 2); display.println(F("g"));
  display.print(F("Y(Lat) : ")); display.print(ay_f, 2); display.println(F("g"));
  display.print(F("Z(Vert): ")); display.print(az_f, 2); display.println(F("g"));
  display.print(F("|H|   : ")); display.print(g_horiz, 2); display.println(F("g"));
  display.print(F("Peak  : ")); display.print(peak_horiz, 2); display.println(F("g"));
  // 条形图（±2g）
  drawBarH(0, 48, 124, 14, ay_f, 2.0f);
  display.setCursor(126-6*3, 48);
  display.print(F("Lat"));
  display.display();
  delay(20);
}
