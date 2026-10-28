/****************************************************/
/*       ESP32 dev Module                           */
/****************************************************/
#include <HardwareSerial.h>

#define SENSOR_SERIAL Serial2
#define BAUD_RATE 921600

char rxBuf[64];
int rxIndex = 0;

float distance = 0;
int snr = 0;

void setup() {
  Serial.begin(115200);  // USB串口波特率
  SENSOR_SERIAL.begin(BAUD_RATE, SERIAL_8N1, 16, 17);  // 16接传感器T  黄线

  Serial.println("distance,snr");
}

void loop() {
  while (SENSOR_SERIAL.available()) {
    char c = SENSOR_SERIAL.read();

    // 收到一行结束
    if (c == '\n') {
      rxBuf[rxIndex] = '\0';  // 字符串结束

      parseLine(rxBuf);

      // 每帧打印
      Serial.print(distance, 1);
      Serial.print(",");
      Serial.println(snr);

      rxIndex = 0;  // 清空缓存
    }
    else {
      if (rxIndex < sizeof(rxBuf) - 1) {
        rxBuf[rxIndex++] = c;
      }
      else {
        // 防止溢出
        rxIndex = 0;
      }
    }
  }
}

void parseLine(char *line) {
  // 找逗号
  char *comma = strchr(line, ',');

  if (comma != NULL) {
    *comma = '\0';

    distance = atof(line);
    snr = atoi(comma + 1);
  }
}