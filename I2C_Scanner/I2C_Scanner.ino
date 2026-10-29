/*
 * ============================================================
 *  I2C Scanner para ESP32
 *  Escanea todas las direcciones 0x01..0x7F y reporta
 *  cuáles responden.
 *
 *  Conexiones:
 *  GPIO 25 → SDA
 *  GPIO 27 → SCL
 *  Velocidad Serial: 115200
 * ============================================================
 */

#include <Wire.h>

#define SDA_PIN 25
#define SCL_PIN 27

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==============================");
  Serial.println("       I2C SCANNER ESP32");
  Serial.println("==============================");
  Serial.println("SDA: GPIO25  |  SCL: GPIO27");
  Serial.println("------------------------------");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  int found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo encontrado en: 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);

      // Identificar dispositivos conocidos
      Serial.print("  →  ");
      switch (addr) {
        case 0x40: Serial.println("PT2258 (CSB2=VCC, CSB1=GND)  /  0x80 en 8-bit"); break;
        case 0x42: Serial.println("PT2258 (CSB2=GND, CSB1=VCC)  /  0x84 en 8-bit"); break;
        case 0x44: Serial.println("PT2258 (CSB2=GND, CSB1=GND)  /  0x88 en 8-bit"); break;
        case 0x46: Serial.println("PT2258 (CSB2=VCC, CSB1=VCC)  /  0x8C en 8-bit"); break;
        case 0x3C:
        case 0x3D: Serial.println("OLED SSD1306 / SH1106"); break;
        case 0x68:
        case 0x69: Serial.println("MPU6050 / DS3231 RTC / PCF8523"); break;
        case 0x76:
        case 0x77: Serial.println("BME280 / BMP280 / BMP180"); break;
        case 0x27:
        case 0x3F: Serial.println("LCD I2C PCF8574"); break;
        default:   Serial.println("Dispositivo desconocido"); break;
      }
      found++;
    }
    else if (error == 4) {
      Serial.print("Error en direccion 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }

  Serial.println("------------------------------");
  if (found == 0) {
    Serial.println("No se encontro ningun dispositivo.");
    Serial.println("");
    Serial.println("Verificar:");
    Serial.println("  - Conexiones SDA/SCL");
    Serial.println("  - Pull-ups 4.7k");
    Serial.println("  - Alimentacion del dispositivo");
    Serial.println("  - Nivel logico (ESP32=3.3V, PT2258=5V)");
  } else {
    Serial.print("Total encontrados: ");
    Serial.println(found);
    Serial.println("");
    Serial.println("Usa la direccion 0xXX en tu codigo:");
    Serial.println("  #define PT2258_ADDR 0xXX");
  }
  Serial.println("==============================");
  Serial.println("Reinicia para escanear de nuevo");
}

void loop() {
  // Nada — el escaneo es de una sola vez en setup()
  // Reiniciá el ESP32 para repetir
}
