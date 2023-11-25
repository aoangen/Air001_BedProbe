// Air001_BedProbe_CS1237 3D打印压力传感器热床调平程序V1.1
// 新增串口命令调试功能，可通过串口输入命令进行校准、重置偏移、切换阈值等操作
// 下载时将Air001主频设置为48MHz
#include "CS1237.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <climits>

// 定义引脚
#define SCK_PIN PB_0   // CS1237_CSK
#define DOUT_PIN PA_4  // CS1237_DOUT
#define PROBE_OUT1 PB_1
#define PROBE_OUT0 PB_3
#define PULSE_PIN PA_5   // 输出实时速度速率
#define BUTTON_PIN PB_6  // BOOT按键，运行中复用阈值切换功能


// 定义常量
const int THRESHOLDS[] = { 50, 150, 300 };                               // 阈值数组，可修改，可添加
const int THRESHOLD_COUNT = sizeof(THRESHOLDS) / sizeof(THRESHOLDS[0]);  // 计算数组长度

const uint8_t SPEED_SETTINGS[] = { SPEED_10, SPEED_40, SPEED_640, SPEED_1280 };
const uint8_t SPEED_SETTING_COUNT = sizeof(SPEED_SETTINGS) / sizeof(SPEED_SETTINGS[0]);


const unsigned long MUTATION_DURATION = 300000;   // LED点亮的最短时间长度，单位微秒
const unsigned long MAX_TRIGGER_TIME = 10000000;  // 触发最长时间，单位微秒
const unsigned long UPDATE_INTERVAL = 10000000;   // 参考值更新时间，单位微秒

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
unsigned long mutationCount = 0;  // 触发计数

bool recordMinMax = false;  // 记录最大最小值的标志
int maxWeight = INT_MIN;    // 最大重量
int minWeight = INT_MAX;    // 最小重量

float emaWeight = 0.0;      // 滤波后的重量
const float alpha = 0.1;    // EMA滤波系数
bool emaFilterEnabled;      // EMA滤波器开关



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

  pinMode(PROBE_OUT1, OUTPUT);
  digitalWrite(PROBE_OUT1, LOW);

  pinMode(PROBE_OUT0, OUTPUT);
  digitalWrite(PROBE_OUT0, HIGH);

  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW);

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
    thresholdIndex = 1;// 如果索引无效，则使用默认值
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
    emaFilterEnabled = true; // 如果读取的值不是0或1，默认为启用
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

  if (change > threshold) {
    if (!ledFlag) {
      mutationCount++;
      if (!serialOutput) {
        Serial.print("触发次数: ");
        Serial.print(mutationCount);
        Serial.print(", 参考重量: ");
        Serial.print(referenceValue);
        Serial.print("g, 当前阈值: ");
        Serial.print(threshold);
        Serial.println("g");
      }
      ledFlag = true;
      mutationStartTime = micros();
      lastUpdateTime = micros();
    } else if ((micros() - mutationStartTime) >= MAX_TRIGGER_TIME) {
      // 如果触发时间超过上限，就更新参考重量并退出触发状态
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
    Serial.print("提取的速度字符串：");
    Serial.println(speedStr);  // 打印提取的字符串

    uint8_t newIndex = speedStr.toInt();
    Serial.print("解析后的数据速率索引是：");
    Serial.println(newIndex);  // 打印解析后的索引

    if (newIndex < SPEED_SETTING_COUNT) {
      EEPROM.put(SPEED_SETTING_ADDRESS, newIndex);
      Serial.println("数据速率索引已保存。请重启以应用新设置。");
      Serial.print("保存的数据速率索引是：");
      Serial.println(newIndex);  // 打印保存的数据速率索引
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
  }
  else if (command == "HELP") {
    Serial.println("===========HELP===========");
    Serial.println("ADC <factor> - 设置校准因数,值为整数");
    Serial.println("SET SPEED <index> - 设置数据速率,输入索引值，0=10,1=40,2=640,3=1280，重启生效");
    Serial.println("RST - 重置偏移");
    Serial.println("SERIAL - 开启/关闭串口调试输出");
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

  switchThreshold();       //按键切换阈值
  checkPressureChange();   //读取传感器并检查是否触发
  updateReferenceValue();  //更新参考重量

  handleWeightRecording();

  // 串口输出调试信息
  if (serialOutput) {
    Serial.print("重量: ");
    Serial.print(emaWeight);
    Serial.print("g | 参考值: ");
    Serial.print(referenceValue);
    Serial.print(" | 触发次数: ");
    Serial.print(mutationCount);
    Serial.print(" | 阈值: ");
    Serial.println(threshold);
  }

  // 串口命令
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    handleSerialCommand(command);
  }
}
