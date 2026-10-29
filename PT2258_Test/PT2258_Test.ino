/*
 * PT2258 Test v4 — registros correctos (sunrutcon library)
 * SDA=GPIO25, SCL=GPIO27, Addr=0x44
 */
#include <Wire.h>

#define SDA_PIN     25
#define SCL_PIN     27
#define PT2258_ADDR 0x44

// Registros correctos según sunrutcon/PT2258
//                     CH1    CH2    CH3    CH4    CH5    CH6
uint8_t TENS[]  = {0x80, 0x40, 0x00, 0x20, 0x60, 0xA0};
uint8_t UNITS[] = {0x90, 0x50, 0x10, 0x30, 0x70, 0xB0};

bool w(uint8_t d) {
  Wire.beginTransmission(PT2258_ADDR);
  Wire.write(d);
  uint8_t e = Wire.endTransmission();
  delay(10);
  if(e) { Serial.print("NACK 0x"); Serial.println(d,HEX); }
  return e == 0;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(200);

  Serial.println("\n=== PT2258 TEST v4 ===");

  // Reset
  Serial.print("Reset 0xC0... "); w(0xC0); delay(300);

  // Mute OFF
  Serial.println("Mute OFF 0x00...");  w(0x00); delay(10);

  // Todos los canales a 0dB
  for(int i=0;i<6;i++){
    Serial.print("CH"); Serial.print(i+1); Serial.print(" 0dB: ");
    bool ok = w(TENS[i]|0) && w(UNITS[i]|0);
    Serial.println(ok?"OK":"NACK");
  }

  Serial.println("\n→ Deberias escuchar audio en todos los canales");
  Serial.println("Esperando 5s...");
  delay(5000);

  Serial.println("\nAplicando 40dB atenuacion en todos...");
  for(int i=0;i<6;i++){ w(TENS[i]|4); w(UNITS[i]|0); }
  Serial.println("→ El volumen deberia bajar notablemente");
  delay(5000);

  Serial.println("\nVolviendo a 0dB...");
  for(int i=0;i<6;i++){ w(TENS[i]|0); w(UNITS[i]|0); }
  Serial.println("Done.");
}

void loop(){}
