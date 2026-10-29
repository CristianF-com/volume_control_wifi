/*
 * ============================================================
 *  PT2258 6-Channel Volume Controller — ESP32
 *  v5.0  Master · Grupos · Mute · IR NEC · Escenas · WiFi · NVS
 * ============================================================
 *
 *  LIBRERÍAS (Library Manager):
 *  - ArduinoJson        by Benoit Blanchon
 *  - WiFiManager        by tzapu
 *  - IRremote           by Armin Joachimsmeyer  (v4.x)
 *
 *  HARDWARE:
 *  GPIO 21 → SDA  |  GPIO 22 → SCL
 *  GPIO 34 → DATA del receptor IR (TSOP1738 o similar)
 *            VCC → 3.3V  |  GND → GND
 *  PT2258 CSB → GND  (I2C addr 0x44)
 *
 *  ── JERARQUÍA DE VOLUMEN ──────────────────────────────────
 *  MASTER (0-79)
 *   ├─ FRONTALES sub-master  →  CH1, CH2
 *   ├─ TRASEROS  sub-master  →  CH3, CH4
 *   ├─ CH5 Voces
 *   └─ CH6 Subwoofer
 *
 *  ── ESCENAS (4 ranuras) ───────────────────────────────────
 *  Cada escena guarda: relBase[6], masterVal, mutes, nombre.
 *  Se puede guardar el estado actual o editar manualmente
 *  desde el panel web o por comando.
 *  Se activan por web, BT/Serial o botones IR.
 *
 *  ── IR NEC ────────────────────────────────────────────────
 *  Primero "aprender" los códigos desde el panel web o serial,
 *  luego asignarlos a acciones. Por defecto las acciones son:
 *
 *  Acción           Código IR (programable)
 *  ──────────────────────────────────────────
 *  VOL+             (aprender)
 *  VOL-             (aprender)
 *  MUTE MASTER      (aprender)
 *  ESCENA 1         (aprender)
 *  ESCENA 2         (aprender)
 *  ESCENA 3         (aprender)
 *  ESCENA 4         (aprender)
 *  NEXT SCENE       (aprender)  cicla escenas guardadas
 *
 *  ── CONFIGURAR WiFi ───────────────────────────────────────
 *  Sin credenciales → red "PT2258_Setup" → 192.168.4.1
 *  Reset WiFi: botón en GPIO35 (3s) o comando WIFIRESET
 *
 *  ── COMANDOS SERIAL / BT ──────────────────────────────────
 *  SET <1-6> <0-79>           Canal individual
 *  MASTER <0-79>              Maestro global
 *  GROUP <F|T> <0-79>         Sub-maestro grupo
 *  MUTE <M|F|T|1-6>           Toggle mute
 *  SCENE SAVE <1-4> [nombre]  Guarda escena actual
 *  SCENE LOAD <1-4>           Carga escena
 *  SCENE LIST                 Lista escenas guardadas
 *  SCENE CLEAR <1-4>          Borra escena
 *  IR LEARN <accion>          Aprende código IR para acción
 *    acciones: VOLUP VOLDOWN MUTE S1 S2 S3 S4 NEXT
 *  IR LIST                    Lista códigos IR aprendidos
 *  IR CLEAR                   Borra todos los códigos IR
 *  GET                        Estado actual
 *  RESET                      Todo a 0
 *  WIFIRESET
 *  HELP
 *
 *  ── HTTP ──────────────────────────────────────────────────
 *  GET /set?ch=1&vol=50
 *  GET /master?vol=40
 *  GET /group?g=F&vol=30
 *  GET /mute?t=M
 *  GET /scene/save?n=1&name=Cine
 *  GET /scene/load?n=1
 *  GET /scene/clear?n=1
 *  GET /ir/learn?action=VOLUP    (espera 10s el código IR)
 *  GET /ir/list
 *  GET /status
 *  GET /reset
 *  GET /wifireset
 * ============================================================
 */

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#define DECODE_NEC
#include <IRremote.hpp>

// ── Configuración ────────────────────────────────────────────
#define SDA_PIN         25
#define SCL_PIN         27
#define PT2258_ADDR     0x44
#define IR_PIN          34
#define NUM_CHANNELS    6
#define VOL_MAX         79
#define VOL_MIN         0
#define VOL_STEP        3     // paso de VOL+/-  por IR
#define AP_NAME         "PT2258_Setup"
#define BT_DEVICE_NAME  "PT2258_Controller"
#define PORTAL_TIMEOUT  180
#define NUM_SCENES      4
#define IR_LEARN_TIMEOUT 10000  // ms para aprender un código

// ── Grupos ───────────────────────────────────────────────────
#define GRP_FRONT  0
#define GRP_REAR   1
#define GRP_NONE   2
#define NUM_GROUPS 2
const uint8_t CH_GROUP[NUM_CHANNELS] = {GRP_FRONT,GRP_FRONT,GRP_REAR,GRP_REAR,GRP_NONE,GRP_NONE};
const uint8_t GRP_IDX[NUM_GROUPS][2] = {{0,1},{2,3}};

// ── Escenas ──────────────────────────────────────────────────
struct Scene {
  bool     valid;
  char     name[16];
  int      relBase[NUM_CHANNELS];
  int      masterVal;
  bool     muteM;
  bool     muteG[NUM_GROUPS];
  bool     muteCh[NUM_CHANNELS];
};

// ── Acciones IR ──────────────────────────────────────────────
#define IR_ACT_VOLUP   0
#define IR_ACT_VOLDOWN 1
#define IR_ACT_MUTE    2
#define IR_ACT_S1      3
#define IR_ACT_S2      4
#define IR_ACT_S3      5
#define IR_ACT_S4      6
#define IR_ACT_NEXT    7
#define IR_NUM_ACTIONS 8
const char* IR_ACT_NAMES[IR_NUM_ACTIONS] = {"VOLUP","VOLDOWN","MUTE","S1","S2","S3","S4","NEXT"};

// ── Variables globales ───────────────────────────────────────
int  relBase[NUM_CHANNELS]    = {0,0,0,0,0,0};
int  masterVal                = 0;
int  prevGroupVal[NUM_GROUPS] = {0,0};
bool muteM                    = false;
bool muteG[NUM_GROUPS]        = {false,false};
bool muteCh[NUM_CHANNELS]     = {false,false,false,false,false,false};
bool wifiConnected            = false;

Scene   scenes[NUM_SCENES];
int     activeScene           = -1;   // -1 = ninguna activa
uint32_t irCodes[IR_NUM_ACTIONS];     // 0 = no asignado
bool    irLearning            = false;
int     irLearnAction         = -1;
unsigned long irLearnStart    = 0;
Stream* irLearnResponder      = nullptr;

// ── Objetos globales de sistema ─────────────────────────────
WebServer   server(80);
Preferences prefs;
WiFiManager wifiManager;

// ── BLE UART (Nordic NUS) ────────────────────────────────────
// UUIDs estándar Nordic UART Service
#define BLE_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_RX_UUID        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // escritura (recibe)
#define BLE_CHAR_TX_UUID        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // notificación (envía)

BLEServer*          bleServer   = nullptr;
BLECharacteristic*  bleTxChar   = nullptr;
bool                bleConnected = false;
String              btBuf        = "";

// Stream proxy para BLE — permite usar la misma interfaz que Serial
class BLEStream : public Stream {
public:
  String rxBuf;
  size_t write(uint8_t c) override {
    if (bleTxChar && bleConnected) {
      uint8_t v = c;
      bleTxChar->setValue(&v, 1);
      bleTxChar->notify();
      delay(1);  // evita congestión BLE
    }
    return 1;
  }
  size_t write(const uint8_t* buf, size_t len) override {
    if (bleTxChar && bleConnected) {
      bleTxChar->setValue(const_cast<uint8_t*>(buf), len);
      bleTxChar->notify();
      delay(2);
    }
    return len;
  }
  int available() override  { return rxBuf.length(); }
  int read()      override  { if(!rxBuf.length()) return -1; char c=rxBuf[0]; rxBuf.remove(0,1); return c; }
  int peek()      override  { if(!rxBuf.length()) return -1; return rxBuf[0]; }
  void flush()    override  {}
};
BLEStream SerialBT;

class BLEServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) {
    bleConnected = true;
    Serial.println(F("[BLE] Cliente conectado"));
    // Enviar bienvenida con IP al conectarse
    delay(500);  // esperar que el cliente esté listo para recibir
    String msg = "\r\n[ PT2258 Controller ]\r\n";
    msg += "WiFi: " + (wifiConnected ? WiFi.localIP().toString() : String("sin conexion")) + "\r\n";
    msg += "Escribe HELP para ver comandos\r\n";
    SerialBT.print(msg);
  }
  void onDisconnect(BLEServer* s){ bleConnected = false; Serial.println(F("[BLE] Cliente desconectado"));
                                   s->startAdvertising(); }
};
class BLERxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    String val = c->getValue().c_str();
    SerialBT.rxBuf += val;
  }
};

// ── Helpers de volumen ───────────────────────────────────────
inline int groupOf(int i) { return CH_GROUP[i]; }

int level(int i) {
  int v = relBase[i] + masterVal;
  return (v < VOL_MIN) ? VOL_MIN : (v > VOL_MAX) ? VOL_MAX : v;
}

int displayed(int i) {
  if (muteM) return 0;
  int g = groupOf(i);
  if (g < NUM_GROUPS && muteG[g]) return 0;
  if (muteCh[i]) return 0;
  int v = relBase[i] + masterVal;
  return (v < VOL_MIN) ? VOL_MIN : (v > VOL_MAX) ? VOL_MAX : v;
}

void normalize() {
  int maxB = relBase[0];
  for (int i = 1; i < NUM_CHANNELS; i++) if (relBase[i] > maxB) maxB = relBase[i];
  if (maxB == 0) return;
  for (int i = 0; i < NUM_CHANNELS; i++) relBase[i] -= maxB;
  masterVal = constrain(masterVal + maxB, VOL_MIN, VOL_MAX);
}

int groupSliderVal(int g) {
  int maxBase = relBase[GRP_IDX[g][0]];
  for (int k = 1; k < 2; k++)
    if (relBase[GRP_IDX[g][k]] > maxBase) maxBase = relBase[GRP_IDX[g][k]];
  int v = maxBase + masterVal;
  return (v < VOL_MIN) ? VOL_MIN : (v > VOL_MAX) ? VOL_MAX : v;
}

// ── NVS ─────────────────────────────────────────────────────
void saveVolumes() {
  prefs.begin("pt2258", false);
  prefs.putInt("master", masterVal);
  for (int i = 0; i < NUM_CHANNELS; i++)
    prefs.putInt(("r"+String(i)).c_str(), relBase[i]);
  uint8_t mc = 0;
  if (muteM) mc |= 0x01;
  for (int g = 0; g < NUM_GROUPS; g++) if (muteG[g]) mc |= (0x02 << g);
  prefs.putUChar("muteCtrl", mc);
  uint8_t mch = 0;
  for (int i = 0; i < NUM_CHANNELS; i++) if (muteCh[i]) mch |= (1 << i);
  prefs.putUChar("muteCh", mch);
  prefs.putInt("activeScene", activeScene);
  prefs.end();
}

#define DEFAULT_MASTER 40   // Volumen por defecto al primer arranque (0-79)

void loadVolumes() {
  prefs.begin("pt2258", true);
  // Si nunca se guardó nada, "master" no existe → getInt devuelve -1 como sentinel
  int savedMaster = prefs.getInt("master", -1);
  if (savedMaster <= 0) {   // -1 = nunca guardado, 0 = silencio total (usar default)
    // Primer arranque o master=0: usar valores por defecto
    masterVal = DEFAULT_MASTER;
    for (int i = 0; i < NUM_CHANNELS; i++) relBase[i] = 0;
    muteM = false;
    for (int g = 0; g < NUM_GROUPS; g++) muteG[g] = false;
    for (int i = 0; i < NUM_CHANNELS; i++) muteCh[i] = false;
    activeScene = -1;
    Serial.print(F("[NVS] Usando volumen por defecto: "));
    Serial.println(DEFAULT_MASTER);
  } else {
    masterVal = savedMaster;
    for (int i = 0; i < NUM_CHANNELS; i++)
      relBase[i] = prefs.getInt(("r"+String(i)).c_str(), 0);
    uint8_t mc  = prefs.getUChar("muteCtrl", 0);
    uint8_t mch = prefs.getUChar("muteCh",   0);
    activeScene = prefs.getInt("activeScene", -1);
    muteM = mc & 0x01;
    for (int g = 0; g < NUM_GROUPS; g++) muteG[g]  = mc & (0x02 << g);
    for (int i = 0; i < NUM_CHANNELS; i++) muteCh[i] = mch & (1 << i);
  }
  prefs.end();
  normalize();
  for (int g = 0; g < NUM_GROUPS; g++) prevGroupVal[g] = groupSliderVal(g);
}

void saveScenes() {
  prefs.begin("scenes", false);
  for (int n = 0; n < NUM_SCENES; n++) {
    String pfx = "s" + String(n);
    prefs.putBool((pfx+"v").c_str(), scenes[n].valid);
    if (!scenes[n].valid) continue;
    prefs.putString((pfx+"nm").c_str(), scenes[n].name);
    prefs.putInt((pfx+"m").c_str(), scenes[n].masterVal);
    for (int i = 0; i < NUM_CHANNELS; i++)
      prefs.putInt((pfx+"r"+String(i)).c_str(), scenes[n].relBase[i]);
    uint8_t mc = 0;
    if (scenes[n].muteM) mc |= 0x01;
    for (int g = 0; g < NUM_GROUPS; g++) if (scenes[n].muteG[g]) mc |= (0x02 << g);
    prefs.putUChar((pfx+"mc").c_str(), mc);
    uint8_t mch = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) if (scenes[n].muteCh[i]) mch |= (1 << i);
    prefs.putUChar((pfx+"mch").c_str(), mch);
  }
  prefs.end();
}

void loadScenes() {
  prefs.begin("scenes", true);
  for (int n = 0; n < NUM_SCENES; n++) {
    String pfx = "s" + String(n);
    scenes[n].valid = prefs.getBool((pfx+"v").c_str(), false);
    if (!scenes[n].valid) { snprintf(scenes[n].name,16,"Escena %d",n+1); continue; }
    String nm = prefs.getString((pfx+"nm").c_str(), "Escena "+String(n+1));
    strncpy(scenes[n].name, nm.c_str(), 15); scenes[n].name[15]=0;
    scenes[n].masterVal = prefs.getInt((pfx+"m").c_str(), 0);
    for (int i = 0; i < NUM_CHANNELS; i++)
      scenes[n].relBase[i] = prefs.getInt((pfx+"r"+String(i)).c_str(), 0);
    uint8_t mc  = prefs.getUChar((pfx+"mc").c_str(),  0);
    uint8_t mch = prefs.getUChar((pfx+"mch").c_str(), 0);
    scenes[n].muteM = mc & 0x01;
    for (int g = 0; g < NUM_GROUPS; g++) scenes[n].muteG[g]  = mc & (0x02 << g);
    for (int i = 0; i < NUM_CHANNELS; i++) scenes[n].muteCh[i] = mch & (1 << i);
  }
  prefs.end();
}

void saveIRCodes() {
  prefs.begin("ircodes", false);
  for (int a = 0; a < IR_NUM_ACTIONS; a++)
    prefs.putUInt(IR_ACT_NAMES[a], irCodes[a]);
  prefs.end();
}

void loadIRCodes() {
  prefs.begin("ircodes", true);
  for (int a = 0; a < IR_NUM_ACTIONS; a++)
    irCodes[a] = prefs.getUInt(IR_ACT_NAMES[a], 0);
  prefs.end();
}

void clearAll() {
  for (int i = 0; i < NUM_CHANNELS; i++) { relBase[i] = 0; muteCh[i] = false; }
  masterVal = 0;
  for (int g = 0; g < NUM_GROUPS; g++) { muteG[g] = false; prevGroupVal[g] = 0; }
  muteM = false; activeScene = -1;
  saveVolumes();
}

// ── PT2258 ───────────────────────────────────────────────────
// Registros correctos según librería sunrutcon/PT2258
// CH1   CH2   CH3   CH4   CH5   CH6
const uint8_t PT2258_TENS[NUM_CHANNELS]  = {0x80, 0x40, 0x00, 0x20, 0x60, 0xA0};
const uint8_t PT2258_UNITS[NUM_CHANNELS] = {0x90, 0x50, 0x10, 0x30, 0x70, 0xB0};
// MUTE = 0x08, SYSTEM_RESET = 0xC0

bool pt2258_write(uint8_t data) {
  Wire.beginTransmission(PT2258_ADDR);
  Wire.write(data);
  uint8_t err = Wire.endTransmission();
  delayMicroseconds(50);
  return (err == 0);
}

bool pt2258_init() {
  // Reset del sistema
  Wire.beginTransmission(PT2258_ADDR);
  Wire.write(0xC0);  // SYSTEM_RESET
  if (Wire.endTransmission() != 0) return false;
  delay(300);

  // Unmute — registro correcto es 0x00 con bit0=0 según sunrutcon lib
  // MUTE register: 0x08 = mute ON, necesitamos enviarlo para limpiar estado
  Wire.beginTransmission(PT2258_ADDR);
  Wire.write(0x00);  // mute OFF (bit0 = 0)
  if (Wire.endTransmission() != 0) return false;
  delay(10);

  return true;
}

bool pt2258_setChannel(uint8_t ch, uint8_t attenDB) {
  if (ch >= NUM_CHANNELS || attenDB > 79) return false;
  if (!pt2258_write(PT2258_TENS[ch]  | (attenDB/10))) return false;
  if (!pt2258_write(PT2258_UNITS[ch] | (attenDB%10))) return false;
  return true;
}
bool pt2258_applyChannel(uint8_t ch) {
  // Calcular atenuación directamente sin pasar por displayed()
  // para evitar que mute de software devuelva 0 y cause atenuación máxima
  // cuando el masterVal es 0 por primera vez
  uint8_t atten;
  if (muteM || (groupOf(ch) < NUM_GROUPS && muteG[groupOf(ch)]) || muteCh[ch]) {
    atten = 79;  // atenuación máxima = silencio
  } else {
    int v = relBase[ch] + masterVal;
    v = (v < 0) ? 0 : (v > 79) ? 79 : v;
    atten = (uint8_t)(79 - v);
  }
  return pt2258_setChannel(ch, atten);
}

void pt2258_applyAll() {
  Wire.beginTransmission(PT2258_ADDR);
  Wire.write(muteM ? 0x08 : 0x00);
  Wire.endTransmission();
  delayMicroseconds(100);
  for (int i = 0; i < NUM_CHANNELS; i++) { pt2258_applyChannel(i); }
}

// ── Lógica de volumen ────────────────────────────────────────
void setMasterVolume(int v) {
  masterVal = constrain(v, VOL_MIN, VOL_MAX);
  for (int g = 0; g < NUM_GROUPS; g++) prevGroupVal[g] = groupSliderVal(g);
  pt2258_applyAll(); saveVolumes();
}

void setGroupVolume(uint8_t g, int newVal) {
  newVal = constrain(newVal, VOL_MIN, VOL_MAX);
  int delta = newVal - prevGroupVal[g]; if (delta == 0) return;
  for (int k = 0; k < 2; k++) relBase[GRP_IDX[g][k]] += delta;
  prevGroupVal[g] = newVal; normalize();
  for (int gg = 0; gg < NUM_GROUPS; gg++) prevGroupVal[gg] = groupSliderVal(gg);
  pt2258_applyAll(); saveVolumes();
}

void setChannelVolume(int ch, int vol) {
  if (ch < 0 || ch >= NUM_CHANNELS) return;
  vol = constrain(vol, VOL_MIN, VOL_MAX);
  relBase[ch] = vol - masterVal; normalize();
  for (int g = 0; g < NUM_GROUPS; g++) prevGroupVal[g] = groupSliderVal(g);
  pt2258_applyAll(); saveVolumes();
}

void toggleMute(String target) {
  target.toUpperCase();
  if      (target == "M") muteM = !muteM;
  else if (target == "F") muteG[GRP_FRONT] = !muteG[GRP_FRONT];
  else if (target == "T") muteG[GRP_REAR]  = !muteG[GRP_REAR];
  else { int ch = target.toInt()-1; if (ch>=0&&ch<NUM_CHANNELS) muteCh[ch]=!muteCh[ch]; }
  pt2258_applyAll(); saveVolumes();
}

// ── Escenas ──────────────────────────────────────────────────
void sceneSave(int n, const char* name) {
  if (n < 0 || n >= NUM_SCENES) return;
  scenes[n].valid     = true;
  scenes[n].masterVal = masterVal;
  scenes[n].muteM     = muteM;
  strncpy(scenes[n].name, name, 15); scenes[n].name[15] = 0;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    scenes[n].relBase[i] = relBase[i];
    scenes[n].muteCh[i]  = muteCh[i];
  }
  for (int g = 0; g < NUM_GROUPS; g++) scenes[n].muteG[g] = muteG[g];
  activeScene = n;
  saveScenes(); saveVolumes();
}

void sceneLoad(int n) {
  if (n < 0 || n >= NUM_SCENES || !scenes[n].valid) return;
  masterVal = scenes[n].masterVal;
  muteM     = scenes[n].muteM;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    relBase[i] = scenes[n].relBase[i];
    muteCh[i]  = scenes[n].muteCh[i];
  }
  for (int g = 0; g < NUM_GROUPS; g++) muteG[g] = scenes[n].muteG[g];
  normalize();
  for (int g = 0; g < NUM_GROUPS; g++) prevGroupVal[g] = groupSliderVal(g);
  activeScene = n;
  pt2258_applyAll(); saveVolumes();
}

void sceneNext() {
  // Cicla a la siguiente escena válida
  for (int i = 1; i <= NUM_SCENES; i++) {
    int n = (activeScene + i) % NUM_SCENES;
    if (scenes[n].valid) { sceneLoad(n); return; }
  }
}

// ── IR ───────────────────────────────────────────────────────
void irStartLearn(int action, Stream* responder) {
  if (action < 0 || action >= IR_NUM_ACTIONS) return;
  irLearning      = true;
  irLearnAction   = action;
  irLearnStart    = millis();
  irLearnResponder = responder;
  IrReceiver.resume();
  String msg = "[IR] Esperando código para: " + String(IR_ACT_NAMES[action]) +
               " (" + String(IR_LEARN_TIMEOUT/1000) + "s)...";
  if (responder) responder->println(msg);
  Serial.println(msg);
}

void irProcessLearn() {
  if (!irLearning) return;
  // Timeout
  if (millis() - irLearnStart > IR_LEARN_TIMEOUT) {
    irLearning = false;
    String msg = "[IR] Timeout. No se recibió código.";
    if (irLearnResponder) irLearnResponder->println(msg);
    Serial.println(msg);
    IrReceiver.resume();
    return;
  }
  if (!IrReceiver.decode()) return;
  if (IrReceiver.decodedIRData.protocol != NEC) {
    IrReceiver.resume(); return;
  }
  uint32_t code = IrReceiver.decodedIRData.decodedRawData;
  irCodes[irLearnAction] = code;
  saveIRCodes();
  irLearning = false;
  char buf[64];
  snprintf(buf, sizeof(buf), "[IR] Aprendido: %s = 0x%08X", IR_ACT_NAMES[irLearnAction], code);
  if (irLearnResponder) irLearnResponder->println(buf);
  Serial.println(buf);
  IrReceiver.resume();
}

void irHandleCode(uint32_t code) {
  for (int a = 0; a < IR_NUM_ACTIONS; a++) {
    if (irCodes[a] == 0 || irCodes[a] != code) continue;
    switch (a) {
      case IR_ACT_VOLUP:   setMasterVolume(masterVal + VOL_STEP); break;
      case IR_ACT_VOLDOWN: setMasterVolume(masterVal - VOL_STEP); break;
      case IR_ACT_MUTE:    toggleMute("M"); break;
      case IR_ACT_S1:      sceneLoad(0); break;
      case IR_ACT_S2:      sceneLoad(1); break;
      case IR_ACT_S3:      sceneLoad(2); break;
      case IR_ACT_S4:      sceneLoad(3); break;
      case IR_ACT_NEXT:    sceneNext();  break;
    }
    Serial.println("[IR] Acción: " + String(IR_ACT_NAMES[a]));
    return;
  }
}

void irPoll() {
  if (irLearning) { irProcessLearn(); return; }
  if (!IrReceiver.decode()) return;
  if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) { IrReceiver.resume(); return; }
  if (IrReceiver.decodedIRData.protocol == NEC) {
    irHandleCode(IrReceiver.decodedIRData.decodedRawData);
  }
  IrReceiver.resume();
}

// ── JSON / Texto ─────────────────────────────────────────────
String getStatusJSON() {
  StaticJsonDocument<1024> doc;
  doc["master"]     = masterVal;
  doc["muteM"]      = muteM;
  doc["groupFront"] = groupSliderVal(GRP_FRONT);
  doc["muteF"]      = muteG[GRP_FRONT];
  doc["groupRear"]  = groupSliderVal(GRP_REAR);
  doc["muteR"]      = muteG[GRP_REAR];
  JsonArray ch  = doc.createNestedArray("channels");
  JsonArray mch = doc.createNestedArray("muteChannels");
  for (int i = 0; i < NUM_CHANNELS; i++) { ch.add(level(i)); mch.add(muteCh[i]); }
  doc["activeScene"] = activeScene;
  JsonArray sc = doc.createNestedArray("scenes");
  for (int n = 0; n < NUM_SCENES; n++) {
    JsonObject s = sc.createNestedObject();
    s["valid"] = scenes[n].valid;
    s["name"]  = scenes[n].name;
  }
  doc["ip"] = wifiConnected ? WiFi.localIP().toString() : "sin wifi";
  String out; serializeJson(doc, out); return out;
}

String getScenesJSON() {
  StaticJsonDocument<512> doc;
  JsonArray sc = doc.createNestedArray("scenes");
  for (int n = 0; n < NUM_SCENES; n++) {
    JsonObject s = sc.createNestedObject();
    s["n"]      = n+1;
    s["valid"]  = scenes[n].valid;
    s["name"]   = scenes[n].name;
    s["master"] = scenes[n].masterVal;
    JsonArray ch = s.createNestedArray("channels");
    for (int i = 0; i < NUM_CHANNELS; i++) {
      int v = scenes[n].valid ? constrain(scenes[n].relBase[i]+scenes[n].masterVal,0,79) : 0;
      ch.add(v);
    }
  }
  doc["active"] = activeScene;
  String out; serializeJson(doc, out); return out;
}

String getIRJSON() {
  StaticJsonDocument<256> doc;
  for (int a = 0; a < IR_NUM_ACTIONS; a++) {
    char hex[12]; snprintf(hex,12,"0x%08X",irCodes[a]);
    doc[IR_ACT_NAMES[a]] = (irCodes[a]==0) ? "no asignado" : hex;
  }
  String out; serializeJson(doc, out); return out;
}

String getStatusText() {
  String s = "──────── PT2258 v5 ────────\r\n";
  s += "Master     : " + String(masterVal) + (muteM?" [MUTE]":"") + "\r\n";
  const char* gn[]={"Front","Rear "};
  for (int g=0;g<NUM_GROUPS;g++)
    s += "Grupo "+String(gn[g])+" : "+String(groupSliderVal(g))+(muteG[g]?" [MUTE]":"")+"\r\n";
  const char* cn[]={"CH1 Front L","CH2 Front R","CH3 Rear  L","CH4 Rear  R","CH5 Voces  ","CH6 Sub    "};
  for (int i=0;i<NUM_CHANNELS;i++)
    s += String(cn[i])+": "+String(displayed(i))+(muteCh[i]?" [MUTE]":"")+"\r\n";
  s += "Escena     : ";
  if (activeScene>=0 && scenes[activeScene].valid)
    s += String(activeScene+1) + " - " + scenes[activeScene].name;
  else s += "ninguna";
  s += "\r\n";
  s += "WiFi       : "+(wifiConnected?WiFi.localIP().toString():String("Sin conexion"))+"\r\n";
  s += "───────────────────────────\r\n";
  return s;
}

// ── Panel Web ────────────────────────────────────────────────
void handleRoot() {
  server.send(200, "text/html", R"rawhtml(<!DOCTYPE html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>PT2258 Mixer</title>
<style>
:root{--bg:#0f1217;--card:#151a20;--bd:#232b34;--tx:#e8edf2;--mt:#8a95a1;--m:#ffb938;--f:#4cc3f5;--r:#ff7d7d;--g:#a7b0ba;--v:#a78bfa;--red:#e5484d}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;padding:12px;max-width:560px;margin:0 auto}
h1{font-size:16px;margin:4px 0 12px;display:flex;justify-content:space-between;align-items:center}
h1 small{color:var(--mt);font-weight:400;font-size:12px}
.card{background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:12px;margin-bottom:10px}
.hd{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;font-weight:600}
.hd .n{font-variant-numeric:tabular-nums}
.mb{border:1px solid var(--bd);background:#1c232b;color:var(--mt);border-radius:8px;padding:5px 10px;font-size:12px;cursor:pointer}
.mb.on{background:var(--red);border-color:var(--red);color:#fff}
.fd{position:relative;touch-action:none;cursor:pointer;--p:0;--c:var(--m);outline:none}
.h{height:36px}.h .rail{position:absolute;left:0;right:0;top:50%;height:8px;margin-top:-4px;background:#222a33;border-radius:4px}
.h .fill{position:absolute;left:0;top:0;bottom:0;width:calc(var(--p)*100%);background:var(--c);border-radius:4px}
.h .cap{position:absolute;top:50%;left:calc(var(--p)*(100% - 20px));width:20px;height:30px;margin-top:-15px;background:#e8edf2;border-radius:5px;border:2px solid var(--c)}
.h.s{height:30px}.h.s .rail{height:5px;margin-top:-2.5px}.h.s .cap{height:24px;margin-top:-12px;width:16px;left:calc(var(--p)*(100% - 16px))}
.v{width:48px;height:140px;margin:0 auto}.v .rail{position:absolute;left:50%;top:0;bottom:0;width:8px;margin-left:-4px;background:#222a33;border-radius:4px}
.v .fill{position:absolute;left:0;right:0;bottom:0;height:calc(var(--p)*100%);background:var(--c);border-radius:4px}
.v .cap{position:absolute;left:7px;width:34px;height:20px;bottom:calc(var(--p)*(100% - 20px));background:#e8edf2;border-radius:5px;border:2px solid var(--c)}
.fd:focus-visible .cap{box-shadow:0 0 0 3px #ffffff55}
.big{font-size:40px;font-weight:700;color:var(--m);font-variant-numeric:tabular-nums;line-height:1}
.sc{display:flex;justify-content:space-between;color:var(--mt);font-size:11px;margin-top:2px}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.g2 .card{margin:0}
.grp{margin-bottom:10px}
.bay{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:10px;padding-top:10px;border-top:1px dashed var(--bd)}
.ch{text-align:center}
.ch .chn{font-weight:600;font-size:12px;color:var(--mt);margin-bottom:6px}
.ch .db{font-size:12px;margin:6px 0;font-variant-numeric:tabular-nums}
.mu .body,.card.mu .body,.bay.mu,.ch.mu .chb{opacity:.35;filter:grayscale(1)}
.mm .g2 .card,.mm .grp{opacity:.4}
.sl{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
.sl button,.btn{background:#1c232b;border:1px solid var(--bd);color:var(--tx);border-radius:8px;padding:9px 4px;font-size:13px;cursor:pointer;position:relative}
.sl button.act{border-color:var(--m);color:var(--m)}
.sl button.emp{color:var(--mt)}
.sl .x{position:absolute;top:-6px;right:-6px;background:var(--red);color:#fff;border-radius:50%;width:18px;height:18px;font-size:12px;line-height:18px;display:none}
.sv .sl .x.sh{display:block}
.row{display:flex;gap:6px;margin-top:8px}
.row .btn{flex:1}
input[type=text]{background:#0f1217;border:1px solid var(--bd);color:var(--tx);border-radius:8px;padding:8px;flex:2;min-width:0}
details.ir summary{cursor:pointer;font-weight:600}
.irr{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid var(--bd);font-size:13px}
.irr code{color:var(--mt);font-size:12px}
.ft{display:flex;gap:6px}.ft .btn{flex:1}
#st{color:var(--mt);font-size:12px;margin-top:10px;min-height:16px;text-align:center}
.hid{display:none}
</style></head><body>
<h1>PT2258 Mixer <small id="ip"></small></h1>

<div id="app">
<div class="card" id="cm">
 <div class="hd"><span>Master</span><button class="mb" id="mM" data-m="M">Mute</button></div>
 <div class="big" id="vM">0</div>
 <div class="fd h" id="fM" data-k="M" data-c="var(--m)" tabindex="0" role="slider" aria-label="Master" aria-valuemin="0" aria-valuemax="79"><div class="rail"><div class="fill"></div></div><div class="cap"></div></div>
 <div class="sc"><span>0</span><span>20</span><span>40</span><span>60</span><span>79</span></div>
</div>

<div class="g2 grp" id="grpA">
 <div class="card" id="cF"><div class="body">
  <div class="hd"><span style="color:var(--f)">Frontales</span><span class="n" id="vF">0</span></div>
  <div class="fd h s" id="fF" data-k="F" data-c="var(--f)" tabindex="0" role="slider" aria-label="Frontales"><div class="rail"><div class="fill"></div></div><div class="cap"></div></div></div>
  <div class="row"><button class="mb" id="mF" data-m="F" style="flex:1">Mute</button></div>
 </div>
 <div class="card" id="cT"><div class="body">
  <div class="hd"><span style="color:var(--r)">Traseros</span><span class="n" id="vT">0</span></div>
  <div class="fd h s" id="fT" data-k="T" data-c="var(--r)" tabindex="0" role="slider" aria-label="Traseros"><div class="rail"><div class="fill"></div></div><div class="cap"></div></div></div>
  <div class="row"><button class="mb" id="mT" data-m="T" style="flex:1">Mute</button></div>
 </div>
</div>

<div class="card" id="chs"><div style="display:grid;grid-template-columns:repeat(6,1fr);gap:2px" id="chg"></div></div>
</div>

<div class="card sv" id="scn">
 <div class="hd"><span>Escenas</span></div>
 <div class="sl" id="sl"></div>
 <div class="row" id="sr1"><button class="btn" id="bSave">Guardar actual</button><button class="btn" id="bNext">Siguiente ▸</button></div>
 <div class="row hid" id="sr2"><input type="text" id="svn" maxlength="15" placeholder="Nombre"><button class="btn" id="bCancel">Cancelar</button></div>
</div>

<div class="card"><details class="ir"><summary>Control remoto IR</summary><div id="irl" style="margin-top:8px"></div>
 <div class="row"><button class="btn" id="bIRC">Borrar todos</button></div></details></div>

<div class="ft"><button class="btn" id="bRst">Reset volúmenes</button><button class="btn" id="bWifi">Reset WiFi</button></div>
<div id="st"></div>

<script>
(function(){
var NAMES=['Front L','Front R','Rear L','Rear R','Voces','Sub'];
var COLS=['var(--f)','var(--f)','var(--r)','var(--r)','var(--g)','var(--g)'];
var S=null,dragId=null,saveMode=false,timers={},$=function(i){return document.getElementById(i)};
function msg(t){$('st').textContent=t}
function api(u,cb){
  fetch(u).then(function(r){return r.json()}).then(function(j){if(cb)cb(j)}).catch(function(){msg('Sin conexión con el ESP32')});
}
function db(v){return v>=79?'0 dB':'−'+(79-v)+' dB'}
function setF(el,v){el.style.setProperty('--p',(v/79).toString());el.setAttribute('aria-valuenow',v);el.setAttribute('aria-valuetext',db(v))}
// build channel faders
var g=$('chg');
for(var i=0;i<6;i++){
  var d=document.createElement('div');d.className='ch';d.id='c'+(i+1);
  d.innerHTML='<div class="chb"><div class="chn">CH'+(i+1)+'<br>'+NAMES[i]+'</div>'+
   '<div class="fd v" id="f'+(i+1)+'" data-k="'+(i+1)+'" data-c="'+COLS[i]+'" tabindex="0" role="slider" aria-label="CH'+(i+1)+'" aria-valuemin="0" aria-valuemax="79"><div class="rail"><div class="fill"></div></div><div class="cap"></div></div>'+
   '<div class="db" id="d'+(i+1)+'">0 dB</div></div>'+
   '<button class="mb" data-m="'+(i+1)+'" id="m'+(i+1)+'">Mute</button>';
  g.appendChild(d);
}
var fds=document.querySelectorAll('.fd');
Array.prototype.forEach.call(fds,function(f){f.style.setProperty('--c',f.getAttribute('data-c'))});
function send(k,v){
  var u=k==='M'?'/master?vol='+v:(k==='F'||k==='T')?'/group?g='+k+'&vol='+v:'/set?ch='+k+'&vol='+v;
  api(u,function(j){if(dragId===null)render(j)});
}
function queue(k,v){clearTimeout(timers[k]);timers[k]=setTimeout(function(){send(k,v)},80)}
function show(k,v){
  var el=$('f'+k);setF(el,v);
  if(k==='M')$('vM').textContent=v;
  else if(k==='F'||k==='T')$('v'+k).textContent=v;
  else $('d'+k).textContent=db(v);
}
Array.prototype.forEach.call(fds,function(f){
  var k=f.getAttribute('data-k'),vert=f.classList.contains('v');
  function val(e){
    var r=f.getBoundingClientRect(),p;
    if(vert){var cap=20;p=1-(e.clientY-r.top-cap/2)/(r.height-cap)}
    else{var cw=f.classList.contains('s')?16:20;p=(e.clientX-r.left-cw/2)/(r.width-cw)}
    p=Math.max(0,Math.min(1,p));return Math.round(p*79);
  }
  f.addEventListener('pointerdown',function(e){dragId=e.pointerId;f.setPointerCapture(e.pointerId);var v=val(e);show(k,v);queue(k,v);e.preventDefault()});
  f.addEventListener('pointermove',function(e){if(dragId!==e.pointerId)return;var v=val(e);show(k,v);queue(k,v)});
  function end(e){if(dragId!==e.pointerId)return;dragId=null;}
  f.addEventListener('pointerup',end);f.addEventListener('pointercancel',end);
  f.addEventListener('keydown',function(e){
    var cur=parseInt(f.getAttribute('aria-valuenow')||'0',10),d=0;
    if(e.key==='ArrowRight'||e.key==='ArrowUp')d=1;else if(e.key==='ArrowLeft'||e.key==='ArrowDown')d=-1;
    else if(e.key==='PageUp')d=10;else if(e.key==='PageDown')d=-10;else if(e.key==='Home')d=-99;else if(e.key==='End')d=99;else return;
    var v=Math.max(0,Math.min(79,cur+d));show(k,v);queue(k,v);e.preventDefault();
  });
});
Array.prototype.forEach.call(document.querySelectorAll('[data-m]'),function(b){
  b.addEventListener('click',function(){api('/mute?t='+b.getAttribute('data-m'),render)});
});
function render(j){
  S=j;if(!j||j.master===undefined)return;
  $('ip').textContent=j.ip||'';
  if(dragId===null){
    show('M',j.master);show('F',j.groupFront);show('T',j.groupRear);
    for(var i=0;i<6;i++)show(String(i+1),j.channels[i]);
  }
  function mb(id,on){$(id).classList.toggle('on',!!on);$(id).textContent=on?'Muteado':'Mute'}
  mb('mM',j.muteM);mb('mF',j.muteF);mb('mT',j.muteR);
  $('app').classList.toggle('mm',!!j.muteM);
  $('cF').classList.toggle('mu',!!j.muteF);$('cT').classList.toggle('mu',!!j.muteR);
  for(i=0;i<6;i++){mb('m'+(i+1),j.muteChannels[i]);$('c'+(i+1)).classList.toggle('mu',!!j.muteChannels[i])}
  renderScenes();
}
function renderScenes(){
  if(!S||!S.scenes)return;
  var h='';
  S.scenes.forEach(function(s,i){
    var a=S.activeScene===i;
    h+='<button data-n="'+(i+1)+'" class="'+(a?'act ':'')+(s.valid?'':'emp')+'">'+(s.valid?esc(s.name):'Vacía '+(i+1))+(s.valid?'<span class="x" data-x="'+(i+1)+'">×</span>':'')+'</button>';
  });
  $('sl').innerHTML=h;
  $('scn').classList.toggle('sv',saveMode);
  Array.prototype.forEach.call($('sl').querySelectorAll('.x'),function(x){x.classList.toggle('sh',!saveMode&&false)});
}
function esc(t){return String(t).replace(/[&<>"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
$('sl').addEventListener('click',function(e){
  var x=e.target.getAttribute&&e.target.getAttribute('data-x');
  if(x){if(confirm('¿Borrar escena '+x+'?'))api('/scene/clear?n='+x,render);return}
  var b=e.target.closest('button');if(!b)return;
  var n=b.getAttribute('data-n');
  if(saveMode){
    var nm=$('svn').value.trim()||('Escena '+n);
    api('/scene/save?n='+n+'&name='+encodeURIComponent(nm),function(j){setSave(false);render(j);msg('Guardada en '+n)});
  }else if(S&&S.scenes[n-1].valid){api('/scene/load?n='+n,render)}
  else msg('Escena vacía: usa "Guardar actual"');
});
// mostrar × para borrar con pulsación larga (o siempre en modo normal)
var st=document.createElement('style');st.textContent='.sl .x{display:block!important}.sv .sl .x{display:none!important}';document.head.appendChild(st);
function setSave(on){saveMode=on;$('sr2').classList.toggle('hid',!on);$('sr1').classList.toggle('hid',on);$('scn').classList.toggle('sv',on);if(on){$('svn').value='';$('svn').focus();msg('Elige el slot donde guardar')}else msg('')}
$('bSave').onclick=function(){setSave(true)};
$('bCancel').onclick=function(){setSave(false)};
$('bNext').onclick=function(){api('/scene/next',render)};
$('bRst').onclick=function(){if(confirm('¿Resetear volúmenes y mutes?'))api('/reset',render)};
$('bWifi').onclick=function(){if(confirm('¿Borrar WiFi y reiniciar en modo portal?')){fetch('/wifireset').catch(function(){});msg('Reiniciando…')}};
// IR
var IRN=['VOLUP','VOLDOWN','MUTE','S1','S2','S3','S4','NEXT'];
var IRL={VOLUP:'Vol +',VOLDOWN:'Vol −',MUTE:'Mute',S1:'Escena 1',S2:'Escena 2',S3:'Escena 3',S4:'Escena 4',NEXT:'Siguiente escena'};
function irRender(j){
  var h='';IRN.forEach(function(a){h+='<div class="irr"><span>'+IRL[a]+'<br><code>'+esc(j[a]||'')+'</code></span><button class="btn" data-a="'+a+'">Aprender</button></div>'});
  $('irl').innerHTML=h;
}
$('irl').addEventListener('click',function(e){
  var a=e.target.getAttribute&&e.target.getAttribute('data-a');if(!a)return;
  msg('Apunta el control y presiona el botón (10 s)…');e.target.disabled=true;
  fetch('/ir/learn?action='+a).then(function(r){return r.json()}).then(function(j){
    msg(j.error?'Tiempo agotado':'Aprendido '+j.code);api('/ir/list',irRender);
  }).catch(function(){msg('Error IR')});
});
$('bIRC').onclick=function(){if(confirm('¿Borrar todos los códigos IR?'))api('/ir/clear',irRender)};
function poll(){if(dragId===null&&!document.hidden)api('/status',render)}
api('/status',render);api('/ir/list',irRender);
setInterval(poll,3000);
})();
</script></body></html>)rawhtml");
}

// ── Handlers HTTP ────────────────────────────────────────────
void handleStatus()  { server.send(200,"application/json",getStatusJSON()); }

void handleSet() {
  if(!server.hasArg("ch")||!server.hasArg("vol")){server.send(400,"text/plain","ch,vol");return;}
  setChannelVolume(server.arg("ch").toInt()-1, server.arg("vol").toInt());
  server.send(200,"application/json",getStatusJSON());
}
void handleMaster() {
  if(!server.hasArg("vol")){server.send(400,"text/plain","vol");return;}
  setMasterVolume(server.arg("vol").toInt());
  server.send(200,"application/json",getStatusJSON());
}
void handleGroup() {
  if(!server.hasArg("g")||!server.hasArg("vol")){server.send(400,"text/plain","g,vol");return;}
  String g=server.arg("g"); g.toUpperCase();
  uint8_t grp=(g=="F")?GRP_FRONT:(g=="T")?GRP_REAR:255;
  if(grp==255){server.send(400,"text/plain","g=F|T");return;}
  setGroupVolume(grp, server.arg("vol").toInt());
  server.send(200,"application/json",getStatusJSON());
}
void handleMute() {
  if(!server.hasArg("t")){server.send(400,"text/plain","t");return;}
  toggleMute(server.arg("t"));
  server.send(200,"application/json",getStatusJSON());
}
void handleSceneSave() {
  if(!server.hasArg("n")){server.send(400,"text/plain","n");return;}
  int n = server.arg("n").toInt()-1;
  String name = server.hasArg("name") ? server.arg("name") : ("Escena "+String(n+1));
  sceneSave(n, name.c_str());
  server.send(200,"application/json",getStatusJSON());
}
void handleSceneLoad() {
  if(!server.hasArg("n")){server.send(400,"text/plain","n");return;}
  sceneLoad(server.arg("n").toInt()-1);
  server.send(200,"application/json",getStatusJSON());
}
void handleSceneNext() { sceneNext(); server.send(200,"application/json",getStatusJSON()); }
void handleSceneClear() {
  if(!server.hasArg("n")){server.send(400,"text/plain","n");return;}
  int n=server.arg("n").toInt()-1;
  if(n>=0&&n<NUM_SCENES){ scenes[n].valid=false; snprintf(scenes[n].name,16,"Escena %d",n+1); saveScenes(); }
  server.send(200,"application/json",getStatusJSON());
}
void handleSceneList() { server.send(200,"application/json",getScenesJSON()); }

void handleIRLearn() {
  if(!server.hasArg("action")){server.send(400,"text/plain","action");return;}
  String act=server.arg("action"); act.toUpperCase();
  int action=-1;
  for(int a=0;a<IR_NUM_ACTIONS;a++) if(String(IR_ACT_NAMES[a])==act){action=a;break;}
  if(action<0){server.send(400,"text/plain","accion invalida");return;}
  // Modo bloqueante corto: esperar hasta IR_LEARN_TIMEOUT en el handler
  // Para no bloquear el servidor, usamos la variable irLearning y respondemos async
  irStartLearn(action, nullptr);
  unsigned long t0=millis();
  while(millis()-t0 < IR_LEARN_TIMEOUT) {
    if(!IrReceiver.decode()){delay(10);continue;}
    if(IrReceiver.decodedIRData.protocol==NEC &&
       !(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)){
      uint32_t code=IrReceiver.decodedIRData.decodedRawData;
      irCodes[action]=code; saveIRCodes(); irLearning=false;
      IrReceiver.resume();
      char hex[12]; snprintf(hex,12,"0x%08X",code);
      StaticJsonDocument<64> doc; doc["action"]=IR_ACT_NAMES[action]; doc["code"]=hex;
      String out; serializeJson(doc,out);
      server.send(200,"application/json",out); return;
    }
    IrReceiver.resume(); delay(10);
  }
  irLearning=false; IrReceiver.resume();
  server.send(200,"application/json","{\"error\":\"timeout\"}");
}
void handleIRList()  { server.send(200,"application/json",getIRJSON()); }
void handleIRClear() {
  for(int a=0;a<IR_NUM_ACTIONS;a++) irCodes[a]=0;
  saveIRCodes(); server.send(200,"application/json",getIRJSON());
}
void handleReset() { clearAll(); pt2258_applyAll(); server.send(200,"application/json",getStatusJSON()); }
void handleWifiReset() {
  server.send(200,"text/plain","Reiniciando...");
  delay(500); wifiManager.resetSettings(); ESP.restart();
}

void setupWebServer() {
  server.on("/",            handleRoot);
  server.on("/status",      handleStatus);
  server.on("/set",         handleSet);
  server.on("/master",      handleMaster);
  server.on("/group",       handleGroup);
  server.on("/mute",        handleMute);
  server.on("/scene/save",  handleSceneSave);
  server.on("/scene/load",  handleSceneLoad);
  server.on("/scene/next",  handleSceneNext);
  server.on("/scene/clear", handleSceneClear);
  server.on("/scene/list",  handleSceneList);
  server.on("/ir/learn",    handleIRLearn);
  server.on("/ir/list",     handleIRList);
  server.on("/ir/clear",    handleIRClear);
  server.on("/reset",       handleReset);
  server.on("/wifireset",   handleWifiReset);
  server.begin();
}

// ── Comandos Serial / BT ─────────────────────────────────────
void processCommand(String cmd, Stream& out) {
  cmd.trim(); cmd.toUpperCase();

  if (cmd.startsWith("SET ")) {
    int sp=cmd.indexOf(' ',4); if(sp==-1){out.println("ERROR: SET <1-6> <0-79>");return;}
    int ch=cmd.substring(4,sp).toInt()-1, vol=cmd.substring(sp+1).toInt();
    if(ch<0||ch>=NUM_CHANNELS){out.println("ERROR: canal 1-6");return;}
    setChannelVolume(ch,vol); out.print("OK\r\n"+getStatusText());

  } else if (cmd.startsWith("MASTER ")) {
    setMasterVolume(cmd.substring(7).toInt()); out.print("OK\r\n"+getStatusText());

  } else if (cmd.startsWith("GROUP ")) {
    int sp=cmd.indexOf(' ',6); if(sp==-1){out.println("ERROR: GROUP <F|T> <0-79>");return;}
    String g=cmd.substring(6,sp); int vol=cmd.substring(sp+1).toInt();
    uint8_t grp=(g=="F")?GRP_FRONT:(g=="T")?GRP_REAR:255;
    if(grp==255){out.println("ERROR: F o T");return;}
    setGroupVolume(grp,vol); out.print("OK\r\n"+getStatusText());

  } else if (cmd.startsWith("MUTE ")) {
    toggleMute(cmd.substring(5)); out.print("OK\r\n"+getStatusText());

  } else if (cmd.startsWith("SCENE SAVE ")) {
    // SCENE SAVE <1-4> [nombre]
    String rest=cmd.substring(11); rest.trim();
    int sp=rest.indexOf(' ');
    int n=(sp==-1?rest:rest.substring(0,sp)).toInt()-1;
    String name=(sp==-1)?"Escena "+String(n+1):rest.substring(sp+1);
    if(n<0||n>=NUM_SCENES){out.println("ERROR: ranura 1-4");return;}
    sceneSave(n,name.c_str()); out.println("OK - Escena "+String(n+1)+" guardada: "+name);

  } else if (cmd.startsWith("SCENE LOAD ")) {
    int n=cmd.substring(11).toInt()-1;
    if(n<0||n>=NUM_SCENES||!scenes[n].valid){out.println("ERROR: escena no válida");return;}
    sceneLoad(n); out.print("OK\r\n"+getStatusText());

  } else if (cmd=="SCENE LIST") {
    out.println("── Escenas ──────────────────");
    for(int n=0;n<NUM_SCENES;n++){
      out.print(String(n+1)+(activeScene==n?"*":" ")+" ");
      if(scenes[n].valid) out.println(String(scenes[n].name)+" (master:"+String(scenes[n].masterVal)+")");
      else out.println("(vacía)");
    }

  } else if (cmd.startsWith("SCENE CLEAR ")) {
    int n=cmd.substring(12).toInt()-1;
    if(n>=0&&n<NUM_SCENES){scenes[n].valid=false;snprintf(scenes[n].name,16,"Escena %d",n+1);saveScenes();}
    out.println("OK");

  } else if (cmd.startsWith("IR LEARN ")) {
    String act=cmd.substring(9); act.trim();
    int action=-1;
    for(int a=0;a<IR_NUM_ACTIONS;a++) if(String(IR_ACT_NAMES[a])==act){action=a;break;}
    if(action<0){out.println("ERROR: acciones: VOLUP VOLDOWN MUTE S1 S2 S3 S4 NEXT");return;}
    irStartLearn(action, &out);

  } else if (cmd=="IR LIST") {
    out.println("── Códigos IR ───────────────");
    for(int a=0;a<IR_NUM_ACTIONS;a++){
      char buf[32]; snprintf(buf,32,"%-10s: ",IR_ACT_NAMES[a]);
      out.print(buf);
      if(irCodes[a]) { char h[12]; snprintf(h,12,"0x%08X",irCodes[a]); out.println(h); }
      else out.println("no asignado");
    }

  } else if (cmd=="IR CLEAR") {
    for(int a=0;a<IR_NUM_ACTIONS;a++) irCodes[a]=0;
    saveIRCodes(); out.println("OK - Códigos IR borrados");

  } else if (cmd=="GET"||cmd=="STATUS") {
    out.print(getStatusText());

  } else if (cmd=="RESET") {
    clearAll(); pt2258_applyAll(); out.println("OK - Todo en 0");

  } else if (cmd=="WIFIRESET") {
    out.println("Reiniciando..."); delay(300); wifiManager.resetSettings(); ESP.restart();

  } else if (cmd=="HELP") {
    out.println("SET <1-6> <0-79>           Canal individual");
    out.println("MASTER <0-79>              Maestro global");
    out.println("GROUP <F|T> <0-79>         Sub-maestro grupo");
    out.println("MUTE <M|F|T|1-6>           Toggle mute");
    out.println("SCENE SAVE <1-4> [nombre]  Guarda escena actual");
    out.println("SCENE LOAD <1-4>           Carga escena");
    out.println("SCENE LIST                 Lista escenas");
    out.println("SCENE CLEAR <1-4>          Borra escena");
    out.println("IR LEARN <accion>          Aprende código IR");
    out.println("  acciones: VOLUP VOLDOWN MUTE S1 S2 S3 S4 NEXT");
    out.println("IR LIST                    Lista códigos IR");
    out.println("IR CLEAR                   Borra códigos IR");
    out.println("GET                        Estado actual");
    out.println("RESET                      Todo a 0");
    out.println("WIFIRESET");
  } else if (cmd.length()>0) {
    out.println("Desconocido. HELP para ayuda.");
  }
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200); delay(300);
  Serial.println(F("\n[BOOT] PT2258 ESP32 v5.0"));

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(10000);   // 10kHz — maximo para nivel logico 3.3V con PT2258 a 5V

  // Espera larga para que el PT2258 estabilice su VCC antes de cualquier comando
  // Sin esto, el reset del ESP32 deja el PT2258 en estado indeterminado
  Serial.println(F("[PT2258] Esperando estabilizacion del chip..."));
  delay(1000);

  // Reintentar hasta 10 veces con 500ms entre intentos
  bool pt_ok = false;
  for (int attempt = 1; attempt <= 10; attempt++) {
    Serial.print(F("[PT2258] Init intento ")); Serial.print(attempt); Serial.print(F("/10... "));
    pt_ok = pt2258_init();
    if (pt_ok) {
      Serial.println(F("OK"));
      break;
    }
    Serial.println(F("NACK - reintentando"));
    delay(500);
  }
  if (!pt_ok) {
    Serial.println(F("[PT2258] ERROR - verificar conexiones y nivel logico (necesita level shifter 3.3V→5V)"));
  }

  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
  Serial.println(F("[IR] Receptor activo en GPIO 34"));

  loadVolumes(); loadScenes(); loadIRCodes();
  pt2258_applyAll();
  Serial.println(F("[NVS] Restaurado:")); Serial.print(getStatusText());

  // ── WiFi PRIMERO (debe iniciar antes que BLE) ────────────────
  // Deshabilitar debug para evitar spam que congela el portal
  wifiManager.setDebugOutput(false);
  wifiManager.setTimeout(300);
  wifiManager.setConnectTimeout(20);

  // Al guardar credenciales: reiniciar limpiamente
  // Esto evita conflictos entre el portal de WiFiManager y nuestro WebServer
  wifiManager.setSaveConfigCallback([](){
    Serial.println(F("[WiFi] Credenciales guardadas. Reiniciando..."));
    delay(1000);
    ESP.restart();
  });

  wifiManager.setAPCallback([](WiFiManager*){
    Serial.println(F(""));
    Serial.println(F("*** PORTAL WiFi ACTIVO ***"));
    Serial.println(F("1. Conectate a la red:  PT2258_Setup"));
    Serial.println(F("2. Abre el navegador:   192.168.4.1"));
    Serial.println(F("3. Elige tu red WiFi y pon la contrasena"));
    Serial.println(F("4. El dispositivo se reinicia solo al guardar"));
    Serial.println(F("**************************"));
  });

  Serial.println(F("[WiFi] Buscando red o abriendo portal..."));
  if(wifiManager.autoConnect(AP_NAME)){
    wifiConnected = true;
    Serial.print(F("[WiFi] Conectado! IP: "));
    Serial.println(WiFi.localIP());
    setupWebServer();
    Serial.println("[HTTP] http://" + WiFi.localIP().toString());
  } else {
    wifiConnected = false;
    Serial.println(F("[WiFi] Sin WiFi. Solo Serial/BLE activos."));
    Serial.println(F("[WiFi] Envia WIFIRESET para reintentar."));
  }

  // ── BLE UART (inicia despues de WiFi) ────────────────────────
  // WiFi y BLE comparten la radio — BLE debe arrancar DESPUES
  // de que WiFiManager haya terminado de configurar el WiFi.
  BLEDevice::init(BT_DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BLEServerCB());
  BLEService* bleService = bleServer->createService(BLE_SERVICE_UUID);
  bleTxChar = bleService->createCharacteristic(
                BLE_CHAR_TX_UUID,
                BLECharacteristic::PROPERTY_NOTIFY);
  bleTxChar->addDescriptor(new BLE2902());
  BLECharacteristic* bleRxChar = bleService->createCharacteristic(
                BLE_CHAR_RX_UUID,
                BLECharacteristic::PROPERTY_WRITE |
                BLECharacteristic::PROPERTY_WRITE_NR);
  bleRxChar->setCallbacks(new BLERxCB());
  bleService->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Activo como: " + String(BT_DEVICE_NAME));



  Serial.println(F("[BOOT] Listo. HELP para comandos."));
}

// ── Loop ─────────────────────────────────────────────────────
String serialBuf="";

void loop() {
  if(wifiConnected) server.handleClient();

  irPoll();  // IR siempre activo



  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'||c=='\r'){ if(serialBuf.length()){processCommand(serialBuf,Serial);serialBuf="";} }
    else serialBuf+=c;
  }
  while(SerialBT.available()){
    char c=SerialBT.read();
    if(c=='\n'||c=='\r'){ if(btBuf.length()){processCommand(btBuf,SerialBT);btBuf="";} }
    else btBuf+=c;
  }

  static unsigned long lastCheck=0;
  if(wifiConnected && millis()-lastCheck>15000){
    lastCheck=millis();
    if(WiFi.status()!=WL_CONNECTED){ Serial.println(F("[WiFi] Reconectando...")); WiFi.reconnect(); }
  }
}
