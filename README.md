# volume_control_wifi

Controlador de volumen de 6 canales con **ESP32 (WROOM) + PT2258** (I2C 0x44), con panel web tipo mixer, WiFi por portal cautivo, BLE, control remoto IR y escenas.

## Características
- 6 volúmenes individuales + **master relativo** (mantiene la relación entre canales).
- Sub-masters: **Frontales** (CH1+CH2) y **Traseros** (CH3+CH4). CH5 (Voces) y CH6 (Subwoofer) dependen solo del master.
- Mute no destructivo por canal, grupo y master (no altera los niveles guardados).
- Persistencia en NVS: volúmenes, mutes, escenas y códigos IR.
- **4 escenas** configurables (guardar estado actual, cargar, siguiente).
- **IR NEC** con códigos aprendibles (VOLUP, VOLDOWN, MUTE, S1–S4, NEXT).
- WiFi automático con WiFiManager (portal desde el celular si no hay red).
- Control por **BLE UART** (Nordic NUS) con comandos de texto; muestra la IP al conectar.

## Hardware
| Señal | ESP32 |
|---|---|
| SDA | GPIO 25 |
| SCL | GPIO 27 |
| Receptor IR | GPIO 34 |

- PT2258 en dirección `0x44` (CSB1=GND, CSB2=GND). I2C a 10 kHz.
- Pull-ups de 4.7k en SDA/SCL (probado a 5 V). A largo plazo se recomienda pull-ups a 3.3 V con un conversor de nivel 3.3V↔5V.
- Masa común entre ESP32, PT2258 y la fuente de audio.

## Compilar
Arduino IDE con soporte ESP32 y las librerías: **WiFiManager**, **ArduinoJson**, **IRremote 4.x**.
Partición recomendada: **Huge APP (3MB No OTA/1MB SPIFFS)**.

## Primer uso
1. Cargar el sketch y abrir el monitor serie a 115200.
2. Si no hay red guardada, conectarse al AP que crea el ESP32 y elegir la red WiFi.
3. Abrir la IP indicada en el navegador.

## Panel web
Mixer con faders, mutes, escenas y aprendizaje de IR. Funciona sin internet (sin CDN).

## Endpoints HTTP
`/`, `/status`, `/set?ch=1-6&vol=0-79`, `/master?vol=`, `/group?g=F|T&vol=`, `/mute?t=M|F|T|1-6`,
`/scene/save?n=&name=`, `/scene/load?n=`, `/scene/next`, `/scene/clear?n=`, `/scene/list`,
`/ir/learn?action=`, `/ir/list`, `/ir/clear`, `/reset`, `/wifireset`.

## Comandos serie / BLE
`SET`, `MASTER`, `GROUP`, `MUTE`, `SCENE SAVE|LOAD|LIST|CLEAR`, `IR LEARN|LIST|CLEAR`, `GET`, `RESET`, `WIFIRESET`, `HELP`.

## Utilidades
- `I2C_Scanner/` — escanea el bus I2C para confirmar la dirección del PT2258.
- `PT2258_Test/` — prueba mínima de registros (reset, 0 dB, 40 dB).

## Notas del PT2258
Atenuación = 79 − nivel. Registros decena `{0x80,0x40,0x00,0x20,0x60,0xA0}`, unidad `{0x90,0x50,0x10,0x30,0x70,0xB0}`, reset `0xC0`, mute `0x08`, unmute `0x00`.
