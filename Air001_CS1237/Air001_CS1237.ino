#include "CS1237.h"
#include <Arduino.h>
#include <EEPROM.h>

// 定义引脚
#define SCK_PIN PB_0   //CS1237_CSK
#define DOUT_PIN PA_4  //CS1237_DOUT
#define PROBE_OUT1 PB_1
#define PROBE_OUT0 PB_3
#define PULSE_PIN PA_5   //输出实时速度速率
#define BUTTON_PIN PB_6  //BOOT按键，运行中复用阈值切换功能



// 定义常量
const float thresholds[] = { 50.0, 150.0, 300.0 };                      // 阈值数组，可修改，可添加
const int thresholdCount = sizeof(thresholds) / sizeof(thresholds[0]);  //计算数组长度

const unsigned long mutationDuration = 300000;  // LED点亮的最短时间长度，单位微秒
const unsigned long updateInterval = 10000000;  //参考值更新时间，单位微秒

// 校准因子，用于将ADC值转换为重量，不同测量范围的传感器不相同
// 需要对使用的传感器进行测试测到，影响实际转换重量的结果
// 可以开启串口调试，修改这个值观察输出结果，使用已知重量的物品作对比进行调整
const float calibration_factor = 209.5;

bool serialOutput = false;  // 串口输出调试信息


// 定义全局变量
float threshold;             // 当前阈值
int32_t zero_offset;         // 零点偏移
float referenceValue = 0.0;  // 参考重量值
bool ledFlag = false;
unsigned long mutationStartTime = 0;
unsigned long lastUpdateTime = 0;
unsigned long mutationCount = 0;  // 触发计数
const int thresholdAddress = 0;   // EEPROM

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println("初始化...");

  pinMode(PROBE_OUT1, OUTPUT);
  digitalWrite(PROBE_OUT1, LOW);

  pinMode(PROBE_OUT0, OUTPUT);
  digitalWrite(PROBE_OUT0, HIGH);

  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  CS1237_init(SCK_PIN, DOUT_PIN);

  // 修改增益  数据速率10、40、640、1280
  if (CS1237_configure(PGA_128, SPEED_1280, CHANNEL_A) == 0) {
    Serial.println("CS1237 配置成功");
  } else {
    Serial.println("CS1237 配置失败");
    return;  // 配置失败，直接返回，不继续后续步骤
  }

  zero_offset = CS1237_read();
  Serial.print("零点偏移: ");
  Serial.println(zero_offset);

  referenceValue = adc_to_weight(zero_offset);
  Serial.print("参考重量值: ");
  Serial.println(referenceValue, 2);

  int thresholdIndex = EEPROM.read(thresholdAddress);
  if (thresholdIndex < 0 || thresholdIndex >= thresholdCount) {
    Serial.println("EEPROM 中的阈值索引无效，使用默认索引 0");
    thresholdIndex = 0;  // 如果索引无效，则使用默认索引 0
  }
  threshold = thresholds[thresholdIndex];
  Serial.print("阈值索引: ");
  Serial.print(thresholdIndex);
  Serial.print(", 阈值: ");
  Serial.print(threshold);
  Serial.println("g");

  Serial.println("> 设备准备就绪。");
}

//将CS1237传感器的ADC值转换为重量。
float adc_to_weight(int32_t adc_value) {
  return (adc_value - zero_offset) / calibration_factor;
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

//更新参考值，没有触发并且超过一定时长才进行更新
void updateReferenceValue() {
  if (!ledFlag && (micros() - lastUpdateTime) >= updateInterval) {
    referenceValue = adc_to_weight(CS1237_read());
    lastUpdateTime = micros();
  }
}

//测量当前的重量，并与参考重量值比较，如果变化超过阈值，就触发LED灯并更新相关变量。
void checkPressureChange() {
  float weight = adc_to_weight(CS1237_read());
  float change = abs(weight - referenceValue);

  if (change > threshold) {
    digitalWrite(PROBE_OUT1, HIGH);
    digitalWrite(PROBE_OUT0, LOW);
    if (!ledFlag) {
      mutationCount++;
      if (!serialOutput) {
        Serial.print("触发次数: ");
        Serial.print(mutationCount);
        Serial.print(", 参考重量: ");
        Serial.print(referenceValue, 2);
        Serial.print("g, 当前阈值: ");
        Serial.print(threshold, 2);
        Serial.println("g");
      }
      ledFlag = true;
      mutationStartTime = micros();
      lastUpdateTime = micros();
    }
  } else if (ledFlag && (micros() - mutationStartTime) >= mutationDuration) {
    digitalWrite(PROBE_OUT1, LOW);
    digitalWrite(PROBE_OUT0, HIGH);
    ledFlag = false;
  }
}

//如果按键被按下，就切换阈值，并更新EEPROM中存储的阈值索引。
void switchThreshold() {
  static unsigned long lastButtonPressTime = 0;
  if (digitalRead(BUTTON_PIN) == HIGH && (millis() - lastButtonPressTime) > 200) {
    lastButtonPressTime = millis();

    // 切换阈值
    int thresholdIndex = -1;
    for (int i = 0; i < thresholdCount; i++) {
      if (abs(thresholds[i] - threshold) < 1e-6) {
        thresholdIndex = i;
        break;
      }
    }
    if (thresholdIndex == -1) {
      Serial.println("Error: Threshold not found");
      return;
    }

    thresholdIndex = (thresholdIndex + 1) % thresholdCount;
    threshold = thresholds[thresholdIndex];

    // 将新的阈值索引存入EEPROM
    EEPROM.write(thresholdAddress, thresholdIndex);

    if (!serialOutput) {
      Serial.print("切换阈值: ");
      Serial.print(threshold);
      Serial.println("g");
    }

    // LED闪烁提示成功切换
    blinkLED(thresholdIndex + 1);
  }
}


void loop() {

  switchThreshold();       //按键切换阈值
  checkPressureChange();   //读取传感器并检查是否触发
  updateReferenceValue();  //更新参考重量

  // 串口输出调试信息
  if (serialOutput) {
    Serial.print("重量: ");
    Serial.print(adc_to_weight(CS1237_read()), 2);
    Serial.print("g | 参考值: ");
    Serial.print(referenceValue, 2);
    Serial.print(" | 触发次数: ");
    Serial.print(mutationCount);
    Serial.print(" | 阈值: ");
    Serial.println(threshold);
  }
}
