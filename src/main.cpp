#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <avr/wdt.h>
#include <limits.h>
#include "config.h"

const UartClass &bmsSerial = Serial1;

const pin_size_t MAIN_RELAIS_PIN = 2;
const pin_size_t CHARGE_RELAIS_PIN[] = {13, 12, 11, 10};
const size_t NUM_CHARGE_RELAIS_PINS = sizeof(CHARGE_RELAIS_PIN) / sizeof(pin_size_t);

int status = WL_IDLE_STATUS;
WiFiServer server(48263);

typedef struct BmsHeader {
  uint8_t magic;
  uint8_t type;
  uint8_t status;
  uint8_t dataLength;
};

typedef struct BmsBasicInfo {
  uint16_t volts;
  int16_t amps;
  uint16_t capacityRemainAh;
  // uint16_t protectionState;
  // uint16_t cycle;
  uint8_t unknown2[13];
  uint8_t capacityRemainPercent;
  uint8_t unknown3[3];
  uint16_t temp1;
  uint16_t temp2;
  uint16_t temp3;
};

typedef struct BmsPacket {
  BmsHeader header;
  BmsBasicInfo info;
  uint16_t checksum;
};

uint16_t getBmsPaketChecksum(BmsPacket &paket) {
  uint16_t sum = 0;

  for (int i = 4; i < paket.header.dataLength + 4; ++i) {
    sum += ((uint8_t*)&paket)[i];
  }

  return (sum + paket.header.dataLength - 1) ^ 0xFFFF;
}

enum BmsError {
  NO_ERROR = 0,

  MAGIC_ERROR = 1,
  TYPE_ERROR = 2,
  STATUS_ERROR = 3,
  CHECKSUM_ERROR = 4
};

BmsError getBmsBasicInfo(BmsBasicInfo &info) {
  uint8_t reqMessage[] = { 0x00, 0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77 };
  bmsSerial.write(reqMessage, sizeof(reqMessage));

  bool found = false;
  BmsPacket paket;
  uint8_t *response = (uint8_t *)&paket;
  memset(response, 0, sizeof(BmsPacket));
  unsigned long start = millis();
  for (uint8_t i = 0; i < sizeof(BmsPacket);) {
    if ((unsigned long)(millis() - start) > 1000) {
      break;
    }
    if (bmsSerial.available() > 0) {
      uint8_t u = bmsSerial.read();
      if (u == 0xDD) {
        found = true;
      }
      if (found) {
        response[i++] = u;
        // Serial.print(u, HEX);
        // Serial.print(" ");
      }
    }
  }
  // Serial.println();

  if (paket.header.magic != 0xDD) {
    Serial.println("Magic not okay");
    return MAGIC_ERROR;
  }

  if (paket.header.type != 3) {
    Serial.println("Type not okay");
    return TYPE_ERROR;
  }

  if (paket.header.status != 0) {
    Serial.println("Status not okay");
    return STATUS_ERROR;
  }

  if (getBmsPaketChecksum(paket) != __builtin_bswap16(paket.checksum)) {
    Serial.println("Checksum not okay");
    return CHECKSUM_ERROR;
  }

  info.volts = __builtin_bswap16(paket.info.volts);
  info.amps = __builtin_bswap16(paket.info.amps);
  info.capacityRemainAh = __builtin_bswap16(paket.info.capacityRemainAh);
  // info.protectionState = __builtin_bswap16(paket.info.protectionState);
  // info.cycle = __builtin_bswap16(paket.info.cycle);
  info.temp1 = __builtin_bswap16(paket.info.temp1) - 2731;
  info.temp2 = __builtin_bswap16(paket.info.temp2) - 2731;
  info.temp3 = __builtin_bswap16(paket.info.temp3) - 2731;
  info.capacityRemainPercent = paket.info.capacityRemainPercent;

  return NO_ERROR;
}

void printBmsInfo(BmsBasicInfo info) {
  Serial.print("Volts: ");
  Serial.println(info.volts / 100.f);
  Serial.print("Amps: ");
  Serial.println(info.amps / 100.f);
  // Serial.print("ProtectionState: ");
  // Serial.println(info.protectionState);
  // Serial.print("Cycle: ");
  // Serial.println(info.cycle);
  Serial.print("Temp1: ");
  Serial.println(info.temp1 / 10.f);
  Serial.print("Temp2: ");
  Serial.println(info.temp2 / 10.f);
  Serial.print("Temp3: ");
  Serial.println(info.temp3 / 10.f);
  Serial.print("Capacity (%): ");
  Serial.println(info.capacityRemainPercent);
  Serial.print("Capacity (Ah): ");
  Serial.println(info.capacityRemainAh / 100.f);
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  int statusWiFi = WiFi.status();
  if (statusWiFi==WL_CONNECTION_LOST || statusWiFi==WL_DISCONNECTED || statusWiFi==WL_SCAN_COMPLETED) {
    Serial.println("WIFI DISCONNECTED");
  } else {
    Serial.println("WIFI CONNECTED");
  }

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

bool watchdog_tripped() {
  bool res = RSTCTRL.RSTFR & RSTCTRL_WDRF_bm;
  RSTCTRL.RSTFR |= RSTCTRL_WDRF_bm;
  return res;
}

void watchdog_enable() {
  wdt_enable(0xB);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (watchdog_tripped()) {
    Serial.println("======> WATCHDOG TRIPPED");
  }

  Serial1.begin(9600);
  while(!Serial1);

  pinMode(MAIN_RELAIS_PIN, OUTPUT);
  for (size_t n = 0; n < NUM_CHARGE_RELAIS_PINS; ++n) {
    pinMode(CHARGE_RELAIS_PIN[n], OUTPUT);
    digitalWrite(CHARGE_RELAIS_PIN[n], HIGH);
  }

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  server.begin();
  watchdog_enable();
}

unsigned long lastApiCallWifiReset = 0, lastApiCallShutdown = 0;

void wifi_ensure_connected() {
  status = WiFi.status();
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    wdt_reset();
    WiFi.begin(ssid, pass);
    for (int n = 0; n < 20; ++n) {
      wdt_reset();
      delay(1000);
    }
    status = WiFi.status();
    if (status == WL_CONNECTED) {
      printWifiStatus();
      lastApiCallWifiReset = millis();
      lastApiCallShutdown = millis();
    }
  }
}

void loop() {
  wdt_reset();

  if ((unsigned long)(millis() - lastApiCallShutdown) >= 5ul*60ul*1000ul) {
    Serial.println("===> 5 mins without api call, turning everything off");
    digitalWrite(MAIN_RELAIS_PIN, LOW);
    for (size_t n = 0; n < NUM_CHARGE_RELAIS_PINS; ++n) {
      digitalWrite(CHARGE_RELAIS_PIN[n], HIGH);
    }
    lastApiCallShutdown = millis();
  }

  if ((unsigned long)(millis() - lastApiCallWifiReset) >= 1ul*60ul*1000ul) {
    Serial.println("===> 1 min without api call, reconnecting to wifi");
    WiFi.disconnect();
  }

  wifi_ensure_connected();

  WiFiClient client = server.available();
  if (client) {
    Serial.println("new client");
    lastApiCallWifiReset = millis();
    lastApiCallShutdown = millis();
    String currentLine = "";
    int statusCode = 500;
    String response = "";
    while (client.connected()) {
      wdt_reset();
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.print("HTTP/1.1 ");
            client.print(statusCode);
            if (statusCode == 200) {
              client.println(" OK");
            } else if (statusCode == 404) {
              client.println(" Not Found");
            } else if (statusCode == 500) {
              client.println(" Internal Server Error");
            } else {
              client.println("");
            }
            client.println("Content-type:text/html");
            client.println();
            client.print(response);
            client.println();
            break;
          } else {
            // Serial.println(currentLine);

            /////// POST
            if (currentLine.startsWith("POST")) {
              if (currentLine.indexOf("/cr/") > -1) {
                int d;
                if (sscanf(currentLine.c_str(), "POST /cr/%d", &d) != 1) {
                  statusCode = 404;
                  response = "parameter malformed";
                } else {
                  if (d < 0 || d >= pow(2, NUM_CHARGE_RELAIS_PINS)) {
                    statusCode = 404;
                    response = "parameter out of range";
                  } else {
                    for (size_t n = 0; n < NUM_CHARGE_RELAIS_PINS; ++n) {
                      digitalWrite(CHARGE_RELAIS_PIN[n], !(d & (1 << n)));
                    }
                    statusCode = 200;
                  }
                }
              } else if (currentLine.indexOf("/r/") > -1) {
                int r = 0, n = -1;
                if (sscanf(currentLine.c_str(), "POST /r/%d/%d", &n, &r) != 2) {
                  statusCode = 404;
                  response = "parameter malformed";
                } else {
                  if (n == 0) {
                    digitalWrite(MAIN_RELAIS_PIN, r == 1 ? HIGH : LOW);
                    statusCode = 200;
                  } else {
                    statusCode = 404;
                    response = "relais not found";
                  }
                }
              } else {
                statusCode = 404;
                response = "POST not supported";
              }

            /////// GET
            } else if (currentLine.startsWith("GET")) {
              if (currentLine.indexOf("/b ") > -1) {
                BmsBasicInfo info;
                BmsError err = getBmsBasicInfo(info);
                if (err == NO_ERROR) {
                  statusCode = 200;
                  response = "{ \"b\": " + String(info.capacityRemainPercent) + ", \"v\":" + String(info.volts) + ", \"c\":" + String(info.amps) + " }";
                } else {
                  statusCode = 500;
                  if (err == MAGIC_ERROR) {
                    response = "bms communication error: magic not okay";
                  } else if (err == TYPE_ERROR) {
                    response = "bms communication error: type not okay";
                  } else if (err == STATUS_ERROR) {
                    response = "bms communication error: status not okay";
                  } else if (err == CHECKSUM_ERROR) {
                    response = "bms communication error: checksum not okay";
                  } else {
                    response = "bms communication error: unknown";
                  }
                }
              } else {
                statusCode = 404;
                response = "route does not exist";
              }
            }

            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }

    client.stop();
    Serial.println("client disconnected");
  }
}
