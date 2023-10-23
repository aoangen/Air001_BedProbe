#include "HX711.h"
#include <EEPROM.h>

const int EEPROM_THRESHOLD_INDEX_ADDRESS = 0;

// 引脚定义
const int HX_DOUT = PA_4;
const int HX_SCK = PB_0;
const int LED_PIN0 = PB_3;
const int LED_PIN1 = PB_1;
const int BUTTON_PIN = PB_6;

const long BASELINE = 0;
const int THRESHOLDS[] = { 50, 150, 300 };  // 按键切换按压力度阈值
unsigned int thresholdIndex = 1;
int currentThreshold = THRESHOLDS[thresholdIndex];

const unsigned long RESET_INTERVAL = 60000;  // 闲置超时归零时长

HX711 scale;
float calibration_factor = 196;  // 校准系数,10kg传感器测试为196
unsigned long previousMillis = 0;
unsigned long previousLEDStateChangeMillis = 0;
unsigned long resetMillis = 0;
boolean prevLEDState = LOW;
boolean ledStateChanged = true;
bool buttonPrevState = LOW;



void setup() {
  Serial.begin(115200);
  scale.begin(HX_DOUT, HX_SCK);
  scale.set_scale(calibration_factor);

  scale.set_gain(64);
  // scale.set_gain(128)的值 128  32  64
  //               HX711增益 128  32  64
  //      HX71708数据速率/Hz 10   20  80  320
  // HX711库不支持HX71708，想要支持320速率，需要修改库文件

  scale.tare();  // 归零

  pinMode(LED_PIN0, OUTPUT);
  pinMode(LED_PIN1, OUTPUT);
  digitalWrite(LED_PIN0, LOW);
  digitalWrite(LED_PIN1, LOW);
  pinMode(BUTTON_PIN, INPUT);




  thresholdIndex = EEPROM.read(EEPROM_THRESHOLD_INDEX_ADDRESS);
  if (thresholdIndex >= sizeof(THRESHOLDS) / sizeof(THRESHOLDS[0])) {
    thresholdIndex = 0;
  }
  currentThreshold = THRESHOLDS[thresholdIndex];

  delay(1000);
  blinkLedToIndicateThreshold(thresholdIndex);
}

void loop() {
  unsigned long currentMillis = millis();

  bool buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH && buttonPrevState == LOW) {
    thresholdIndex = (thresholdIndex + 1) % (sizeof(THRESHOLDS) / sizeof(THRESHOLDS[0]));
    currentThreshold = THRESHOLDS[thresholdIndex];

    EEPROM.write(EEPROM_THRESHOLD_INDEX_ADDRESS, thresholdIndex);
    blinkLedToIndicateThreshold(thresholdIndex);
  }
  buttonPrevState = buttonState;

  if (digitalRead(LED_PIN0) != prevLEDState) {
    previousLEDStateChangeMillis = currentMillis;
    prevLEDState = !prevLEDState;
    ledStateChanged = true;
    resetMillis = currentMillis;
  } else {
    ledStateChanged = false;
  }

  // 闲置超时归零
  if (!ledStateChanged && (currentMillis - resetMillis >= RESET_INTERVAL)) {
    //delay(5000);
    scale.tare();
    resetMillis = currentMillis;
  }

  if (scale.is_ready()) {
    long weight = scale.get_units(1);

    if (weight >= currentThreshold) {
      digitalWrite(LED_PIN0, HIGH);
      digitalWrite(LED_PIN1, LOW);
    } else {
      digitalWrite(LED_PIN0, LOW);
      digitalWrite(LED_PIN1, HIGH);
    }

    // 重量减少到负阈值*2时归零，异常情况处理，反向施加力可能会造成读数误差
    // if (weight < (BASELINE - currentThreshold * 2)) {
    //   delay(1000);
    //   scale.tare();
    // }

    //Serial.print("重量:");
    Serial.print(weight);
    Serial.print(" g | ");
    //Serial.print("数据间隔:");
    Serial.print(currentMillis - previousMillis);
    Serial.print(" ms | ");
    //Serial.print("触发阈值:");
    Serial.println(currentThreshold);

    previousMillis = currentMillis;
  }
}


void blinkLedToIndicateThreshold(int thresholdIndex) {
  const unsigned long blinkDurations[] = { 300, 150, 50 };  // 不同阈值下的闪烁间隔，单位是毫秒
  const unsigned long totalBlinkDuration = 3000;            // 总的闪烁持续时间，单位是毫秒
  unsigned long interval = blinkDurations[thresholdIndex];
  unsigned long startMillis = millis();

  while (millis() - startMillis < totalBlinkDuration) {
    unsigned long currentMillis = millis();
    if (currentMillis % (2 * interval) < interval) {
      digitalWrite(LED_PIN0, HIGH);
      digitalWrite(LED_PIN1, LOW);
    } else {
      digitalWrite(LED_PIN0, LOW);
      digitalWrite(LED_PIN1, HIGH);
    }
  }
}
