/*
 * Car G Meter for Arduino UNO
 *
 * 使用 MPU9xxx 三轴加速度传感器和 0.96" I2C OLED 显示屏，测量并显示车辆的纵向、横向和垂直加速度。
 * 特性：
 * - 自动检测 MPU 地址 (0x68 或 0x69) 和 OLED 地址 (0x3C 或 0x3D)
 * - 设定加速度量程为 ±4g（可根据需要调整）
 * - 上电自动进行零偏校准（包含重力）
 * - 显示即时 X/Y/Z 加速度值、水平合成 |H|、以及峰值
 * - 高亮按钮 (D2) 重置峰值
 *
 * 使用注意：
 * - 建议传感器模块供电 3.3V，如模块标注支持 3–5V，则可直接接 5V。
 * - OLED 模块请确保支持 3.3–5V；若不确定建议接 5V。
 * - 安装方向：X 轴朝前，Y 轴向左，Z 轴向上。
 * - 使用时请将传感器上电并保持静止，保持水平位置校准零偏。
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

uint8_t OLED_ADDR = 0x3C;
uint8_t mpuAddr   = 0x68;  // will auto-detect 0x68/0x69

// MPU registers
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

// Button pin (optional)
const int PIN_BTN = 2;

// Scale factor for ±4g: 8192 LSB/g
const float ACC_SCALE = 8192.0f;

// Simple low-pass (EMA) factor for display smoothing
const float ALPHA = 0.20f;

float ax_g=0, ay_g=0, az_g=0;        // current acceleration minus offset
float ax_f=0, ay_f=0, az_f=0;        // filtered values
float ax_off=0, ay_off=0, az_off=0;  // offsets learned at startup
float peak_horiz = 0.0f;

bool buttonLast = HIGH;
unsigned long lastDebounce = 0;

bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuReadBytes(uint8_t startReg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(mpuAddr, (uint8_t)len);
  for (size_t i=0; i<len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

bool initMPU() {
  // detect address
  if (i2cDevicePresent(0x68)) mpuAddr = 0x68;
  else if (i2cDevicePresent(0x69)) mpuAddr = 0x69;
  else return false;

  // wake up
  mpuWrite(REG_PWR_MGMT_1, 0x00);
  delay(50);

  // set accel ±4g: FS_SEL = 01 (bits 4:3)
  uint8_t cur=0;
  mpuReadBytes(REG_ACCEL_CONFIG, &cur, 1);
  cur &= ~0x18;
  cur |= (1 << 3);
  mpuWrite(REG_ACCEL_CONFIG, cur);
  delay(10);

  return true;
}

// read acceleration in g
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

// calibrate offsets with N samples (~2.5s at default)
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
  az_off = sz / samples;  // includes +1g
}

// existing bar graph function (unused if using G-ball)
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
  int tick1p = mid + (w/2)/gFullScale * 1.0f;
  int tick1n = mid - (w/2)/gFullScale * 1.0f;
  display.drawFastVLine(tick1p, y, h, SSD1306_WHITE);
  display.drawFastVLine(tick1n, y, h, SSD1306_WHITE);
}

void drawGBall(int cx, int cy, int radius, float ax, float ay, float maxVal) {
  // 圆形边框
  display.drawCircle(cx, cy, radius, SSD1306_WHITE);
  // 十字线
  display.drawLine(cx - radius, cy, cx + radius, cy, SSD1306_WHITE);
  display.drawLine(cx, cy - radius, cx, cy + radius, SSD1306_WHITE);

  // ==== 新增：前进方向标记（小三角形）====
  int fx = cx + radius;     // 圆右边
  int fy = cy;
  display.fillTriangle(fx, fy, fx-4, fy-3, fx-4, fy+3, SSD1306_WHITE);

  // 将加速度值映射到圆内
  float px = ax / maxVal;
  float py = ay / maxVal;
  if (px < -1) px = -1; if (px > 1) px = 1;
  if (py < -1) py = -1; if (py > 1) py = 1;
  int x = cx + (int)(px * radius);
  int y = cy - (int)(py * radius);
  // 加速度点
  display.fillCircle(x, y, 3, SSD1306_WHITE);
}


void setup() {
  pinMode(PIN_BTN, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000);

  // OLED init (try 0x3C/0x3D)
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

  // MPU init
  if (!initMPU()) {
    display.setCursor(0, 16);
    display.println(F("MPU NOT FOUND"));
    display.display();
    while (1) { delay(1000); }
  }

  // calibration: keep still for ~2.5 seconds
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

  // 去零偏 & 平滑
  ax_g = x - ax_off;
  ay_g = y - ay_off;
  az_g = z - az_off;

  ax_f = ALPHA*ax_g + (1-ALPHA)*ax_f;
  ay_f = ALPHA*ay_g + (1-ALPHA)*ay_f;
  az_f = ALPHA*az_g + (1-ALPHA)*az_f;

  float g_horiz = sqrtf(ax_f*ax_f + ay_f*ay_f);
  if (g_horiz > peak_horiz) peak_horiz = g_horiz;

  // 可选按钮清零峰值
  bool b = digitalRead(PIN_BTN);
  if (b != buttonLast) {
    lastDebounce = millis();
    buttonLast = b;
  }
  if (buttonLast == LOW && (millis() - lastDebounce) > 30) {
    peak_horiz = 0.0f;
  }

  // 显示 G 力球
  display.clearDisplay();
  // 在调用 drawGBall 前，加这两行坐标变换
  float ax_r = -ay_f; // 左转90°后的X
  float ay_r =  ax_f; // 左转90°后的Y

  drawGBall(64, 32, 28, ax_f, ay_f, 2.0f); // 居中，半径 28 像素
  display.display();

  delay(20); // ~50 Hz 刷新
}

