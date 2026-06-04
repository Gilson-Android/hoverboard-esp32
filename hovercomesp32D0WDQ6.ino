// Bibliotecas:
//   - XboxSeriesXControllerESP32_asukiaaa (Gerenciador de bibliotecas)
//   - IBusBM by bmellink (Gerenciador de bibliotecas)

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <IBusBM.h>

// --- PINOS MOTORES TRAÇÃO (BTS7960) ---
const int MOTOR_ESQ_RPWM = 12;
const int MOTOR_ESQ_LPWM = 14;
const int MOTOR_DIR_RPWM = 16;
const int MOTOR_DIR_LPWM = 17;

// --- PINO MOTOR DA LÂMINA ---
const int MOTOR_LAMINA = 23;

// --- LED E iBUS ---
const int LED_PIN    = 2;   // LED embutido da placa
const int IBUS_RX    = 13;  // Fio S do FS-iA6B → GPIO 13

// --- MODOS DE CONTROLE ---
enum ControlMode { WAITING, XBOX, RADIO };
ControlMode mode = WAITING;

// --- OBJETOS DE CONTROLE ---
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
IBusBM ibus;
HardwareSerial ibusSerial(1); // UART1 com pino customizado

const int PWM_MAX = 220;

void setup() {
  Serial.begin(115200);
  delay(1000);

  ledcAttach(MOTOR_ESQ_RPWM, 5000, 8);
  ledcAttach(MOTOR_ESQ_LPWM, 5000, 8);
  ledcAttach(MOTOR_DIR_RPWM, 5000, 8);
  ledcAttach(MOTOR_DIR_LPWM, 5000, 8);

  pinMode(MOTOR_LAMINA, OUTPUT);
  digitalWrite(MOTOR_LAMINA, LOW);

  pinMode(LED_PIN, OUTPUT);

  // iBUS: UART1 no GPIO 13 (RX apenas, TX não usado = -1)
  ibusSerial.begin(115200, SERIAL_8N1, IBUS_RX, -1);
  ibus.begin(ibusSerial);

  xboxController.begin();

  Serial.println("[AGUARDANDO] Ligue o rádio FS-i6X ou conecte o Xbox...");
  Serial.println("  LED piscando lento = aguardando");
  Serial.println("  LED sólido         = Xbox conectado");
  Serial.println("  LED duplo pisca    = Rádio conectado");
}

void loop() {
  atualizarLed();

  switch (mode) {
    case WAITING: loopWaiting(); break;
    case XBOX:    loopXbox();    break;
    case RADIO:   loopRadio();   break;
  }
}

// ─── MODO AGUARDANDO ────────────────────────────────────────────────────────

void loopWaiting() {
  xboxController.onLoop();

  // Verifica Xbox primeiro
  if (xboxController.isConnected()) {
    mode = XBOX;
    Serial.println("\n[MODO] Xbox controller conectado!");
    return;
  }

  // Verifica iBUS: canal válido (900–2100)
  uint16_t ch1 = ibus.readChannel(0);
  uint16_t ch2 = ibus.readChannel(1);
  if (ch1 >= 900 && ch1 <= 2100 && ch2 >= 900 && ch2 <= 2100) {
    mode = RADIO;
    Serial.println("\n[MODO] Rádio FS-i6X conectado!");
    return;
  }

  delay(10);
}

// ─── MODO XBOX ──────────────────────────────────────────────────────────────

void loopXbox() {
  xboxController.onLoop();

  if (!xboxController.isConnected()) {
    Serial.println("[!] Xbox desconectado — parando, voltando a aguardar");
    parar_motores();
    digitalWrite(MOTOR_LAMINA, LOW);
    mode = WAITING;
    return;
  }

  if (xboxController.isWaitingForFirstNotification()) return;

  int32_t joyY = 32768 - (int32_t)xboxController.xboxNotif.joyLVert;
  int32_t joyX = (int32_t)xboxController.xboxNotif.joyLHori - 32768;

  if (abs(joyY) < 3500) joyY = 0;
  if (abs(joyX) < 3500) joyX = 0;

  int alvo_esq = map(joyY + (joyX / 2), -32768, 32767, -PWM_MAX, PWM_MAX);
  int alvo_dir = map(joyY - (joyX / 2), -32768, 32767, -PWM_MAX, PWM_MAX);

  digitalWrite(MOTOR_LAMINA, xboxController.xboxNotif.trigRT > 512 ? HIGH : LOW);
  acionarMotores(alvo_esq, alvo_dir);

  static unsigned long lastLog = 0;
  if (millis() - lastLog > 500) {
    Serial.printf("XBOX | Y:%d X:%d | ESQ:%d DIR:%d | LAMINA:%s\n",
      joyY, joyX, alvo_esq, alvo_dir,
      digitalRead(MOTOR_LAMINA) ? "ON" : "OFF");
    lastLog = millis();
  }
}

// ─── MODO RÁDIO (FS-i6X + FS-iA6B via iBUS) ────────────────────────────────
// Mapeamento de canais no transmissor FS-i6X:
//   CH1 = Stick esquerdo horizontal  → Steering (virar)
//   CH2 = Stick esquerdo vertical    → Throttle (frente/ré)
//   CH6 = Chave SWC ou SwD           → Lâmina ON/OFF

void loopRadio() {
  uint16_t ch1 = ibus.readChannel(0); // steering
  uint16_t ch2 = ibus.readChannel(1); // throttle
  uint16_t ch6 = ibus.readChannel(5); // blade

  // Perda de sinal: para tudo e volta a aguardar
  if (ch1 < 900 || ch1 > 2100 || ch2 < 900 || ch2 > 2100) {
    Serial.println("[!] Sinal de rádio perdido — parando, voltando a aguardar");
    parar_motores();
    digitalWrite(MOTOR_LAMINA, LOW);
    mode = WAITING;
    return;
  }

  // Centro em 1500; deadzone de ±50
  int throttle = (int)ch2 - 1500;
  int steering = (int)ch1 - 1500;
  if (abs(throttle) < 50) throttle = 0;
  if (abs(steering) < 50) steering = 0;

  // Tank drive
  int alvo_esq = constrain(map(throttle + (steering / 2), -500, 500, -PWM_MAX, PWM_MAX), -PWM_MAX, PWM_MAX);
  int alvo_dir = constrain(map(throttle - (steering / 2), -500, 500, -PWM_MAX, PWM_MAX), -PWM_MAX, PWM_MAX);

  // CH6 > 1500 = lâmina ligada
  digitalWrite(MOTOR_LAMINA, ch6 > 1500 ? HIGH : LOW);

  acionarMotores(alvo_esq, alvo_dir);

  static unsigned long lastLog = 0;
  if (millis() - lastLog > 500) {
    Serial.printf("RADIO | CH1:%d CH2:%d CH6:%d | ESQ:%d DIR:%d | LAMINA:%s\n",
      ch1, ch2, ch6, alvo_esq, alvo_dir,
      digitalRead(MOTOR_LAMINA) ? "ON" : "OFF");
    lastLog = millis();
  }
}

// ─── LED DE STATUS ──────────────────────────────────────────────────────────

void atualizarLed() {
  unsigned long now = millis();

  switch (mode) {
    case WAITING: {
      // Pisca lento: 500 ms ligado / 500 ms desligado
      digitalWrite(LED_PIN, (now / 500) % 2 == 0 ? HIGH : LOW);
      break;
    }
    case XBOX: {
      // Sólido ligado
      digitalWrite(LED_PIN, HIGH);
      break;
    }
    case RADIO: {
      // Duplo pisca rápido: ON-OFF-ON-pausa (ciclo de 1 s)
      // 0–100ms ON | 100–200ms OFF | 200–300ms ON | 300–1000ms OFF
      unsigned long t = now % 1000;
      bool on = (t < 100) || (t >= 200 && t < 300);
      digitalWrite(LED_PIN, on ? HIGH : LOW);
      break;
    }
  }
}

// ─── FUNÇÕES DE MOTOR ───────────────────────────────────────────────────────

void acionarMotores(int v_esq, int v_dir) {
  if (v_esq >= 0) {
    ledcWrite(MOTOR_ESQ_RPWM, abs(v_esq)); ledcWrite(MOTOR_ESQ_LPWM, 0);
  } else {
    ledcWrite(MOTOR_ESQ_RPWM, 0); ledcWrite(MOTOR_ESQ_LPWM, abs(v_esq));
  }
  if (v_dir >= 0) {
    ledcWrite(MOTOR_DIR_RPWM, abs(v_dir)); ledcWrite(MOTOR_DIR_LPWM, 0);
  } else {
    ledcWrite(MOTOR_DIR_RPWM, 0); ledcWrite(MOTOR_DIR_LPWM, abs(v_dir));
  }
}

void parar_motores() {
  ledcWrite(MOTOR_ESQ_RPWM, 0); ledcWrite(MOTOR_ESQ_LPWM, 0);
  ledcWrite(MOTOR_DIR_RPWM, 0); ledcWrite(MOTOR_DIR_LPWM, 0);
}
