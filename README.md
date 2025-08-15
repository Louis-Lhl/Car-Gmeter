# Car G‑Meter (Arduino UNO + MPU9xxx + SSD1306)

本项目通过 Arduino UNO、MPU9xxx 三轴加速度计和 0.96\" I2C OLED 显示屏，构建一个车载 G 力计。程序读取加速度数据并实时显示纵向 (X)、横向 (Y)、垂向 (Z) G 值以及水平合成值 |H| 与峰值。适用于赛道行驶或者日常驾驶参考。

## 硬件连接

| 组件 | 引脚 | 连接到 |
|---|---|---|
| MPU9xxx 加速度计 | VCC | Arduino UNO 5V（或 3.3V） |
| MPU9xxx 加速度计 | GND | Arduino UNO GND |
| MPU9xxx 加速度计 | SDA | Arduino UNO A4 (SDA) |
| MPU9xxx 加速度计 | SCL | Arduino UNO A5 (SCL) |
| MPU9xxx 加速度计 | AD0 | Arduino UNO GND (设定地址 0x68) |
| OLED 显示屏 | VCC | Arduino UNO 5V |
| OLED 显示屏 | GND | Arduino UNO GND |
| OLED 显示屏 | SDA | Arduino UNO A4 (SDA) |
| OLED 显示屏 | SCL | Arduino UNO A5 (SCL) |

电源通过车辆 USB 5V 为 Arduino UNO 供电。建议在 UNO 5V 和 GND 之间使用 100μF 电解电容和 0.1μF 陶瓷电容做去耦。

## 使用方法

1. 确保按照上表完成硬件连接，保持传感器安装方向：X 轴朝前，Y 轴朝左，Z 轴朝上。
2. 电脑安装 Arduino IDE，并通过库管理器安装 `Adafruit SSD1306` 与 `Adafruit GFX Library`。
3. 打开 `src/Gmeter/Gmeter.ino`，选择开发板 Arduino UNO，选择正确的 COM 端口。
4. 编译并上传程序到 UNO。
5. 上电后屏幕显示 “Calibrating…”；请保持车辆静止约 2.5 秒，待校准完成。
6. 之后屏幕实时显示 G 值和水平合成值；按下接在 D2 的按钮可清零峰值。

## Git 版本控制建议

初始化仓库：

    git init
    git add .
    git commit -m "feat: 初始实现 G‑Meter"
    git branch -M main
    git remote add origin https://github.com/<你的用户名>/Car-Gmeter.git
    git push -u origin main

建议每次调整参数或添加功能后提交一次，如 `git commit -m "tune: 调整滤波参数"`。可以使用标签标记里程碑，例如 `v0.1.0`。
