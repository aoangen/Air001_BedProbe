// Air001_BedProbe_CS1237 3D打印压力传感器热床调平程序V1.2
// 可以使用的MCU  PY32F002AF15P(普冉) Air001(合宙)  本质是同一款，Arduino开发环境由合宙提供
// 新增串口命令调试功能，可通过串口输入命令进行校准、重置偏移、查看输出等操作
// 新增5档LED进度条显示功能，触发进度转换成百分比，从30-100%控制LED点亮

// 功能说明
// 实时读取CS1237 ADC芯片的压力传感器数据，并转换成重量值，速率在10、40、640、1280Hz可调
// 动态阈值，设定阈值+静态压力值=最终触发所需重量，10秒内如果没有压力突变，更新一次阈值
// 检测压力值突变，使用EMA滤波器，压力值大于阈值即触发

// 使用说明
// 参考 https://wiki.luatos.com/chips/air001/index.html 配置开发环境和烧录方法
// 编译时在 <工具> 菜单中将 <Clock Source and Frequency> 设置为 <HSI 24Mhz, HCLK 24Mhz>
// 代码烧录成功后，使用串口监视器，波特率设置为115200，输入 <HELP> 获取控制命令
// 不同的传感器读取到的数据不一样，所以为了获取准确的重量需要进行校准
// 先将数据速率设置为10Hz(低速读取较为精准)，复位重启后生效，再启用实时重量输出，这样就可以在串口监视器查看实时数据了
// 三个数据分别是 重量、滤波后的重量、阈值，打开数据输出时使用串口绘图仪可以可视化显示压力值
// 找一个已知重量的物品(以50g砝码为例)，如果传感器得到的重量不是50g，那么就使用校准命令<ADC 209>进行修改校准值，在默认的209上增加或减少，直到实时重量和所称物品的实际重量接近
// 最后将数据速率调整回1280Hz

// 添加到<文件> <首选项> <其它开发板管理器地址>
// 合宙开发板地址 https://arduino.luatos.com/package_air_cn_index.json

#include "CS1237.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <climits>

// 定义引脚
#define SCK_PIN PB_0   // CS1237_CSK  数据传输时钟引脚
#define DOUT_PIN PA_4  // CS1237_DOUT 数据输出引脚
#define PROBE_OUT1 PB_1 //  触发引脚，触发时高电平
#define PROBE_OUT0 PB_3 //  触发引脚，触发时低电平
#define PULSE_PIN PA_5   // 输出实时速度速率，使用示波器检测实时运行速率
#define BUTTON_PIN PB_6  // BOOT按键，运行中复用阈值切换功能

// 定义进度显示LED引脚
#define LED1_PIN PA_13  // LED 1  30%
#define LED2_PIN PA_14  // LED 2  47%
#define LED3_PIN PA_6   // LED 3  64%
#define LED4_PIN PA_7   // LED 4  81%
#define LED5_PIN PB_2   // LED 5  100%

// 呼吸灯输出引脚
#define LED_PWM PA_0


// 定义常量
const int THRESHOLDS[] = { 50, 150, 300 };                               // 阈值数组，可修改，可添加
const int THRESHOLD_COUNT = sizeof(THRESHOLDS) / sizeof(THRESHOLDS[0]);  // 计算数组长度

const uint8_t SPEED_SETTINGS[] = { SPEED_10, SPEED_40, SPEED_640, SPEED_1280 };
const uint8_t SPEED_SETTING_COUNT = sizeof(SPEED_SETTINGS) / sizeof(SPEED_SETTINGS[0]);


const unsigned long MUTATION_DURATION = 300000;    // LED点亮的最短时间长度，单位微秒
const unsigned long MAX_TRIGGER_TIME = 10000000;   // 触发最长时间，单位微秒
const unsigned long UPDATE_INTERVAL = 10000000;    // 参考值更新时间，单位微秒
const unsigned long SERIAL_OUTPUT_INTERVAL = 100;  // 串口输出时间间隔，单位毫秒

const int THRESHOLD_ADDRESS = 0;           // EEPROM
const int CALIBRATION_FACTOR_ADDRESS = 4;  // 校准因数在 EEPROM 中的地址
const int SPEED_SETTING_ADDRESS = 8;       // 保存数据速率设置的 EEPROM 地址
const int EMA_FILTER_ADDRESS = 12;         // 保存EMA滤波器状态的 EEPROM 地址

// 校准因子，用于将ADC值转换为重量，不同测量范围的传感器不相同
// 需要对使用的传感器进行测试测到，影响实际转换重量的结果
// 可以开启串口调试，修改这个值观察输出结果，使用已知重量的物品作对比进行调整
int calibration_factor = 209;


int threshold;           // 当前阈值
int32_t zero_offset;     // 零点偏移
int referenceValue = 0;  // 参考重量值
bool ledFlag = false;
bool serialOutput = false;
unsigned long mutationStartTime = 0;
unsigned long lastUpdateTime = 0;
unsigned long mutationCount = 0;         // 触发计数
unsigned long lastSerialOutputTime = 0;  // 串口输出时间记录

bool recordMinMax = false;  // 记录最大最小值的标志
int maxWeight = INT_MIN;    // 最大重量
int minWeight = INT_MAX;    // 最小重量

int emaWeight = 0;        // 滤波后的重量
const float alpha = 0.2;  // EMA滤波系数
bool emaFilterEnabled;    // EMA滤波器开关
int progress;             //触发进度百分比

int brightness = 0;         // Initial brightness (PWM value)
int fadeAmount = 5;         // Amount to increase/decrease brightness
unsigned long lastBreathTime = 0;  // Time tracker for breathing effect
const int breathDelay = 80;  // Delay time for smooth breathing effect (adjust for speed)





// 将CS1237传感器的ADC值转换为重量。
int adc_to_weight(int32_t adc_value) {
  return (adc_value - zero_offset) / calibration_factor;
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println("初始化...");

  recordMinMax = false;
  maxWeight = INT_MIN;
  minWeight = INT_MAX;

  // 设置LED引脚为输出模式
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(LED5_PIN, OUTPUT);
  
  // 确保所有LED最开始是关闭状态
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);
  digitalWrite(LED3_PIN, HIGH);
  digitalWrite(LED4_PIN, HIGH);
  digitalWrite(LED5_PIN, HIGH);

  // 设置LED_PWM引脚为输出模式
  pinMode(LED_PWM, OUTPUT);

  // 输出触发引脚
  pinMode(PROBE_OUT1, OUTPUT);
  digitalWrite(PROBE_OUT1, LOW);

  pinMode(PROBE_OUT0, OUTPUT);
  digitalWrite(PROBE_OUT0, HIGH);

  //运行频率输出引脚
  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW);

  // boot0复用按键功能引脚
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  CS1237_init(SCK_PIN, DOUT_PIN);

  // 在 EEPROM 中保存的数据速率索引
  uint8_t speedIndex;
  EEPROM.get(SPEED_SETTING_ADDRESS, speedIndex);
  if (speedIndex >= SPEED_SETTING_COUNT) {
    speedIndex = 3;  // 如果索引无效，则使用默认值
  }
  uint8_t storedSpeedSetting = SPEED_SETTINGS[speedIndex];

  // 修改增益  数据速率10、40、640、1280
  if (CS1237_configure(PGA_128, storedSpeedSetting, CHANNEL_A) == 0) {
    Serial.println("CS1237 配置成功!");
    Serial.print("速率: ");
    switch (storedSpeedSetting) {
      case SPEED_10:
        Serial.println("SPEED_10");
        break;
      case SPEED_40:
        Serial.println("SPEED_40");
        break;
      case SPEED_640:
        Serial.println("SPEED_640");
        break;
      case SPEED_1280:
        Serial.println("SPEED_1280");
        break;
      default:
        Serial.println("速率异常!");
        break;
    }
  } else {
    Serial.println("CS1237 配置失败! 请检查硬件问题或下载设置");
    while (true) {
      blinkLED(1);
    }
  }

  zero_offset = CS1237_read();
  Serial.print("零点偏移: ");
  Serial.println(zero_offset);

  referenceValue = adc_to_weight(zero_offset);
  Serial.print("参考重量值: ");
  Serial.println(referenceValue);

  emaWeight = adc_to_weight(CS1237_read());

  int thresholdIndex = EEPROM.read(THRESHOLD_ADDRESS);
  if (thresholdIndex < 0 || thresholdIndex >= THRESHOLD_COUNT) {
    thresholdIndex = 1;  // 如果索引无效，则使用默认值
  }
  threshold = THRESHOLDS[thresholdIndex];
  Serial.print("触发阈值: ");
  Serial.print(threshold);
  Serial.println("g");

  // 从 EEPROM 读取校准因子
  int storedCalibrationFactor;
  EEPROM.get(CALIBRATION_FACTOR_ADDRESS, storedCalibrationFactor);
  if (storedCalibrationFactor > 0) {
    calibration_factor = storedCalibrationFactor;
  }
  Serial.print("校准因子: ");
  Serial.println(calibration_factor);

  // 读取EMA滤波器状态
  uint8_t emaState;
  EEPROM.get(EMA_FILTER_ADDRESS, emaState);
  if (emaState == 0 || emaState == 1) {
    emaFilterEnabled = (emaState == 1);
  } else {
    emaFilterEnabled = true;  // 如果读取的值不是0或1，默认为启用
  }
  Serial.print("EMA滤波器: ");
  Serial.println(emaFilterEnabled ? "启用" : "禁用");

  Serial.println("> 设备准备就绪。");
  Serial.println("> 输入 HELP 查看帮助信息。");
  Serial.println("");
}

void handleWeightRecording() {
  if (!recordMinMax) return;  // 如果记录功能关闭，直接返回

  int currentWeight = emaWeight;
  bool updated = false;

  // 更新最大值和最小值
  if (currentWeight > maxWeight) {
    maxWeight = currentWeight;
    updated = true;
  }
  if (currentWeight < minWeight) {
    minWeight = currentWeight;
    updated = true;
  }

  // 如果有更新，则输出新的最大值和最小值
  if (updated) {
    Serial.print(maxWeight);
    Serial.print(" | ");
    Serial.println(minWeight);
  }
}


void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PROBE_OUT1, HIGH);
    digitalWrite(PROBE_OUT0, LOW);
    delay(200);
    digitalWrite(PROBE_OUT1, LOW);
    digitalWrite(PROBE_OUT0, HIGH);
    delay(200);
  }
}

// 更新参考值，没有触发并且超过一定时长才进行更新
void updateReferenceValue() {
  if (!ledFlag && (micros() - lastUpdateTime) >= UPDATE_INTERVAL) {
    referenceValue = emaWeight;
    lastUpdateTime = micros();
  }
}

// 测量当前的重量，并与参考重量值比较，如果变化超过阈值，就触发LED灯并更新相关变量。
void checkPressureChange() { 
  int weight = emaWeight; 
  int change = weight - referenceValue; 
  int sum = threshold + referenceValue;  // 计算动态触发阈值
  
  // 计算进度百分比
  progress = 0;
  if (weight >= sum) {
    progress = 100;  // 达到或超过阈值，进度100%
  } else {
    progress = ((float)(weight - referenceValue) / threshold) * 100;  // 计算百分比
  }

  //控制进度显示LED
  checkLEDStatus();

  
  if (change > threshold) { 
    if (!ledFlag) { 
      mutationCount++; 
      if (!serialOutput) { 
        Serial.print("触发次数: "); 
        Serial.print(mutationCount); 
        Serial.print(", 触发阈值: "); 
        Serial.print(referenceValue); 
        Serial.print("g + "); 
        Serial.print(threshold); 
        Serial.println("g"); 
      } 
      ledFlag = true; 
      mutationStartTime = micros(); 
      lastUpdateTime = micros(); 
    } else if ((micros() - mutationStartTime) >= MAX_TRIGGER_TIME) { 
      referenceValue = weight; 
      ledFlag = false; 
      digitalWrite(PROBE_OUT1, LOW); 
      digitalWrite(PROBE_OUT0, HIGH); 
    } else { 
      digitalWrite(PROBE_OUT1, HIGH); 
      digitalWrite(PROBE_OUT0, LOW); 
    } 
  } else if (ledFlag && (micros() - mutationStartTime) >= MUTATION_DURATION) { 
    digitalWrite(PROBE_OUT1, LOW); 
    digitalWrite(PROBE_OUT0, HIGH); 
    ledFlag = false; 
  } 
}


// 如果按键被按下，就切换阈值，并更新EEPROM中存储的阈值索引。
void switchThreshold() {
  static unsigned long lastButtonPressTime = 0;
  if (digitalRead(BUTTON_PIN) == HIGH && (millis() - lastButtonPressTime) > 200) {
    lastButtonPressTime = millis();

    // 切换阈值
    int thresholdIndex = -1;
    for (int i = 0; i < THRESHOLD_COUNT; i++) {
      if (abs(THRESHOLDS[i] - threshold) < 1e-6) {
        thresholdIndex = i;
        break;
      }
    }
    if (thresholdIndex == -1) {
      Serial.println("Error: Threshold not found");
      return;
    }

    thresholdIndex = (thresholdIndex + 1) % THRESHOLD_COUNT;
    threshold = THRESHOLDS[thresholdIndex];

    // 将新的阈值索引存入EEPROM
    EEPROM.write(THRESHOLD_ADDRESS, thresholdIndex);

    if (!serialOutput) {
      Serial.print("切换阈值: ");
      Serial.print(threshold);
      Serial.println("g");
    }

    // LED闪烁提示成功切换
    blinkLED(thresholdIndex + 1);
  }
}

// 控制LED的点亮情况
void checkLEDStatus() {
  // 根据progress控制每个LED的点亮情况
  if (progress > 30) {
    digitalWrite(LED1_PIN, LOW);  // 亮第一个LED
  } else {
    digitalWrite(LED1_PIN, HIGH);   // 否则熄灭
  }

  if (progress > 47) {
    digitalWrite(LED2_PIN, LOW);  // 亮第二个LED
  } else {
    digitalWrite(LED2_PIN, HIGH);   // 否则熄灭
  }

  if (progress > 64) {
    digitalWrite(LED3_PIN, LOW);  // 亮第三个LED
  } else {
    digitalWrite(LED3_PIN, HIGH);   // 否则熄灭
  }

  if (progress > 81) {
    digitalWrite(LED4_PIN, LOW);  // 亮第四个LED
  } else {
    digitalWrite(LED4_PIN, HIGH);   // 否则熄灭
  }

  if (progress >= 100) {
    digitalWrite(LED5_PIN, LOW);  // 亮第五个LED
  } else {
    digitalWrite(LED5_PIN, HIGH);   // 否则熄灭
  }
}

void breathingLED() {
  if (millis() - lastBreathTime >= breathDelay) {
    // Update brightness for breathing effect
    brightness += fadeAmount;

    // Reverse the direction of the fade at the boundaries
    if (brightness <= 0 || brightness >= 255) {
      fadeAmount = -fadeAmount;
    }

    // Adjust the PWM duty cycle to control the LED brightness
    analogWrite(LED_PWM, brightness);

    // Update the last breath time
    lastBreathTime = millis();
  }
}



// 处理串口命令
void handleSerialCommand(String command) {
  command.trim();  // 去除可能的前后空格
  if (command.startsWith("ADC ")) {
    int newFactor = command.substring(4).toInt();
    if (newFactor > 0) {
      calibration_factor = newFactor;
      EEPROM.put(CALIBRATION_FACTOR_ADDRESS, calibration_factor);
      Serial.println("校准因数已保存: " + String(calibration_factor));
    } else {
      Serial.println("无效输入");
    }
  } else if (command == "RST") {
    zero_offset = CS1237_read();
    Serial.println("偏移重置: " + String(zero_offset));
  } else if (command == "SERIAL") {
    serialOutput = !serialOutput;
    Serial.println("串口输出已" + String(serialOutput ? "开启" : "关闭"));
  } else if (command.startsWith("SET SPEED ")) {
    String speedStr = command.substring(10);
    uint8_t newIndex = speedStr.toInt();
    if (newIndex < SPEED_SETTING_COUNT) {
      EEPROM.put(SPEED_SETTING_ADDRESS, newIndex);
      Serial.print("数据速率已修改为：");
      Serial.print(newIndex);
      Serial.println(" ,请重启以应用新设置。");
    } else {
      Serial.println("无效的数据速率索引。");
    }
  } else if (command == "RECORD") {
    recordMinMax = !recordMinMax;
    if (recordMinMax) {
      maxWeight = INT_MIN;
      minWeight = INT_MAX;
    }
    Serial.println(recordMinMax ? "记录功能已开启" : "记录功能已关闭并清除数据");
  } else if (command == "EMA") {
    emaFilterEnabled = !emaFilterEnabled;
    uint8_t emaState = emaFilterEnabled ? 1 : 0;
    EEPROM.put(EMA_FILTER_ADDRESS, emaState);
    Serial.println(emaFilterEnabled ? "EMA滤波器已启用" : "EMA滤波器已禁用");
  } else if (command == "HELP") {
    Serial.println("===========HELP===========");
    Serial.println("ADC <factor> - 设置校准因数,值为整数");
    Serial.println("SET SPEED <index> - 设置数据速率,输入索引值，0=10,1=40,2=640,3=1280，重启生效");
    Serial.println("RST - 重置偏移");
    Serial.println("SERIAL - 开启/关闭实时重量值输出");
    Serial.println("RECORD - 开启/关闭记录读数跳变");
    Serial.println("EMA - 开启/关闭EMA滤波器");
    Serial.println("HELP - 显示帮助信息");
  } else {
    Serial.println("未知命令, 输入 HELP 查看帮助信息");
  }
}

void loop() {

  // 读取新的重量值
  int currentWeight = adc_to_weight(CS1237_read());

  // 应用EMA滤波
  emaWeight = emaFilterEnabled ? (alpha * currentWeight + (1 - alpha) * emaWeight) : currentWeight;


  breathingLED();

  switchThreshold();       //按键切换阈值
  checkPressureChange();   //读取传感器并检查是否触发
  updateReferenceValue();  //更新参考重量

  handleWeightRecording();  //记录最大最小值

  unsigned long currentTime = millis();
  if (currentTime - lastSerialOutputTime >= SERIAL_OUTPUT_INTERVAL) {
    if (serialOutput) {
      Serial.print(currentWeight);
      Serial.print(",");
      Serial.print(emaWeight);
      Serial.print(",");
      int sum = threshold + referenceValue;
      Serial.println(sum);
        // 输出进度百分比到串口
      // Serial.print("当前进度: ");
      // Serial.print(progress);
      // Serial.println("%");
    }
    lastSerialOutputTime = currentTime;  // 更新上次输出时间
  }

  // 串口命令
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    handleSerialCommand(command);
  }
}