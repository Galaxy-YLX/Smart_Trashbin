#include <AltSoftSerial.h>

AltSoftSerial SENSOR_SERIAL; // UNO: RX=8(接传感器T 黄线), TX=9   

#define BAUD_RATE 115200

#define BUF_LEN 32
char lineBuf[BUF_LEN];
uint8_t idx = 0;

uint16_t frameCount = 0;
unsigned long lastSecond = 0;

// 解析后的变量
float distance = 0;
int snr = 0;

void setup() {
  Serial.begin(115200);
  SENSOR_SERIAL.begin(BAUD_RATE);

  Serial.println("distance,snr");
  lastSecond = millis();
}

void loop() {

  while (SENSOR_SERIAL.available()) {
    char c = SENSOR_SERIAL.read();

    if (c == '\r' || c == '\n') {

      if (idx > 0) {
        lineBuf[idx] = '\0';

        parseLine(lineBuf);   //  解析

        //  打印两个变量
        Serial.print(distance, 1);
        Serial.print(",");
        Serial.println(snr);

        frameCount++;
      }

      idx = 0;
    }
    else {
      if (idx < BUF_LEN - 1) {
        lineBuf[idx++] = c;
      } else {
        idx = 0;  // 溢出丢弃
      }
    }
  }

  /*========== 帧率统计 ==========*/
  unsigned long now = millis();
  if (now - lastSecond >= 1000) {

    Serial.print("FPS=");
    Serial.println(frameCount);

    frameCount = 0;
    lastSecond = now;
  }
}

/*========== 解析函数 ==========*/
void parseLine(char *line) {
  char *comma = strchr(line, ',');

  if (comma != NULL) {
    *comma = '\0';

    distance = atof(line);
    snr = atoi(comma + 1);
  }
}