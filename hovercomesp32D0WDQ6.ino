// Bibliotecas:
//   - XboxSeriesXControllerESP32_asukiaaa (Gerenciador de bibliotecas)
//   (iBUS implementado internamente — sem biblioteca externa)

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// ─── LEITOR iBUS (FlySky FS-iA6B) ───────────────────────────────────────────
// Protocolo: 32 bytes por pacote — 0x20 0x40 + 14 canais × 2 bytes + checksum
struct IBusSimple {
  uint16_t ch[14] = {};
  unsigned long lastPacketMs = 0;

  void update(HardwareSerial& s) {
    static uint8_t buf[32];
    static uint8_t pos = 0;

    while (s.available()) {
      uint8_t b = s.read();
      if (pos == 0 && b != 0x20) continue;
      if (pos == 1 && b != 0x40) { pos = 0; continue; }
      buf[pos++] = b;
      if (pos == 32) {
        pos = 0;
        uint16_t sum = 0;
        for (int i = 0; i < 30; i++) sum += buf[i];
        uint16_t ck = buf[30] | ((uint16_t)buf[31] << 8);
        if (ck == (uint16_t)(0xFFFF - sum)) {
          for (int i = 0; i < 14; i++)
            ch[i] = buf[2 + i*2] | ((uint16_t)(buf[3 + i*2] & 0x0F) << 8);
          lastPacketMs = millis();
        }
      }
    }
  }

  uint16_t readChannel(int c) { return (c >= 0 && c < 14) ? ch[c] : 0; }
  bool signalLost(unsigned long timeoutMs = 500) { return (millis() - lastPacketMs) > timeoutMs; }
};

// --- PINOS MOTORES TRAÇÃO (BTS7960) ---
const int MOTOR_ESQ_RPWM = 12;
const int MOTOR_ESQ_LPWM = 14;
const int MOTOR_DIR_RPWM = 16;
const int MOTOR_DIR_LPWM = 17;

// --- PINO MOTOR DA LÂMINA ---
const int MOTOR_LAMINA = 23;

// --- PINOS PISTÃO ELÉTRICO (BTS7960) ---
const int PISTON_RPWM = 25; // sobe
const int PISTON_LPWM = 26; // desce

// --- LED E iBUS ---
const int LED_PIN    = 2;   // LED embutido da placa
const int IBUS_RX    = 13;  // Fio S do FS-iA6B → GPIO 13

// --- MODOS DE CONTROLE ---
enum ControlMode { WAITING, XBOX, RADIO };
ControlMode mode = WAITING;

// --- OBJETOS DE CONTROLE ---
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
IBusSimple ibus;
HardwareSerial ibusSerial(1); // UART1 com pino customizado

const int PWM_MAX = 220;
const int PISTON_PWM = 220; // velocidade do pistão (0–255)

bool armed = false;
unsigned long armStartMs = 0;
const unsigned long ARM_HOLD_MS = 1000;

void setup() {
  Serial.begin(115200);
  delay(1000);

  ledcAttach(MOTOR_ESQ_RPWM, 5000, 8);
  ledcAttach(MOTOR_ESQ_LPWM, 5000, 8);
  ledcAttach(MOTOR_DIR_RPWM, 5000, 8);
  ledcAttach(MOTOR_DIR_LPWM, 5000, 8);
  ledcAttach(PISTON_RPWM, 5000, 8);
  ledcAttach(PISTON_LPWM, 5000, 8);
  desenergizar_pistao(); // sem rádio = nenhuma energia no atuador

  pinMode(MOTOR_LAMINA, OUTPUT);
  digitalWrite(MOTOR_LAMINA, LOW);

  pinMode(LED_PIN, OUTPUT);

  // iBUS: UART1 no GPIO 13 (RX apenas, TX não usado = -1)
  ibusSerial.begin(115200, SERIAL_8N1, IBUS_RX, -1);

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

// ─── LOG DE TODOS OS CANAIS iBUS ────────────────────────────────────────────

void logIbusChannels(const char* prefixo) {
  Serial.print(prefixo);
  for (int i = 0; i < 14; i++) {
    Serial.printf(" CH%02d:%4d", i + 1, ibus.readChannel(i));
  }
  Serial.println();
}

// Detecta mudança em qualquer canal e imprime só o que mudou.
// Use para mapear controles: mova um por vez e veja qual CH aparece.
void logIbusMudancas() {
  static uint16_t anterior[14] = {};
  static bool inicializado = false;

  if (!inicializado) {
    for (int i = 0; i < 14; i++) anterior[i] = ibus.readChannel(i);
    inicializado = true;
    return;
  }

  for (int i = 0; i < 14; i++) {
    uint16_t atual = ibus.readChannel(i);
    if (abs((int)atual - (int)anterior[i]) > 20) {
      Serial.printf("[MUDA] CH%02d: %4d -> %4d\n", i + 1, anterior[i], atual);
      anterior[i] = atual;
    }
  }
}

// ─── MODO AGUARDANDO ────────────────────────────────────────────────────────

void loopWaiting() {
  desenergizar_pistao(); // garante 0V no atuador enquanto não há controle ativo
  xboxController.onLoop();

  // Verifica Xbox primeiro
  if (xboxController.isConnected()) {
    mode = XBOX;
    Serial.println("\n[MODO] Xbox controller conectado!");
    return;
  }

  // Verifica iBUS: pacote recente + canal válido (900–2100)
  ibus.update(ibusSerial);
  uint16_t ch1 = ibus.readChannel(0);
  uint16_t ch2 = ibus.readChannel(1);

  // Mostra canais enquanto aguarda, para facilitar mapeamento do transmissor
  if (!ibus.signalLost(300) && ch1 >= 900 && ch1 <= 2100) {
    static unsigned long lastWaitLog = 0;
    if (millis() - lastWaitLog > 500) {
      logIbusChannels("[AGUARD]");
      lastWaitLog = millis();
    }
  }

  // Só entra em modo RADIO se a sentinela CH10 não indicar failsafe (rádio OFF)
  if (!ibus.signalLost(300) && ch1 >= 900 && ch1 <= 2100 && ch2 >= 900 && ch2 <= 2100
      && ibus.readChannel(9) <= 1700) {
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
    desenergizar_pistao();
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
//   CH1  = Stick esquerdo horizontal  → Steering (virar)
//   CH2  = Stick esquerdo vertical    → Throttle (frente/ré)
//   CH6  = Chave SwC (3 posições)     → Pistão: cima=estica, meio=parado, baixo=fecha
//   CH7  = Chave SwA ou SwD           → Lâmina ON/OFF
//   CH10 = SENTINELA (sem chave)      → ~1500 em operação; 2000 = failsafe (rádio OFF)
//          (CH9 não serve: no rádio ele espelha a SwA)

void loopRadio() {
  ibus.update(ibusSerial);
  uint16_t ch1  = ibus.readChannel(0); // steering
  uint16_t ch2  = ibus.readChannel(1); // throttle
  uint16_t ch6  = ibus.readChannel(5); // pistão (SwC 3 posições)
  uint16_t ch7  = ibus.readChannel(6); // lâmina ON/OFF
  uint16_t ch10 = ibus.readChannel(9); // sentinela de failsafe

  // TRAVA DE FAILSAFE: com o rádio desligado o receptor continua enviando iBUS,
  // mas o CH10 (sem chave atribuída) salta de ~1500 para 2000. Detecta e mata tudo.
  if (ch10 > 1700) {
    Serial.println("[!] FAILSAFE — radio desligado! Cortando energia de tudo.");
    parar_motores();
    desenergizar_pistao();
    digitalWrite(MOTOR_LAMINA, LOW);
    ibus.lastPacketMs = 0;
    armed = false;
    armStartMs = 0;
    mode = WAITING;
    return;
  }

  // Perda de sinal: timeout de pacotes ou valores fora de faixa
  if (ibus.signalLost() || ch1 < 900 || ch1 > 2100 || ch2 < 900 || ch2 > 2100) {
    if (armed) Serial.println("[!] Sinal perdido — sistema desarmado");
    Serial.println("[!] Sinal de rádio perdido — parando, voltando a aguardar");
    parar_motores();
    desenergizar_pistao();
    digitalWrite(MOTOR_LAMINA, LOW);
    ibus.lastPacketMs = 0;
    armed = false;
    armStartMs = 0;
    mode = WAITING;
    return;
  }

  // Lógica de armação: throttle no mínimo (CH2 ≤ 1050) por 1s
  if (!armed) {
    if (ch2 <= 1050) {
      if (armStartMs == 0) armStartMs = millis();
      if (millis() - armStartMs >= ARM_HOLD_MS) {
        armed = true;
        armStartMs = 0;
        Serial.println("[ARM] Sistema armado — controle ativo!");
      }
    } else {
      armStartMs = 0;
    }
    parar_motores();
    desenergizar_pistao();
    digitalWrite(MOTOR_LAMINA, LOW);
    static unsigned long lastArmLog = 0;
    if (millis() - lastArmLog > 2000) {
      Serial.println("[DESARMADO] Empurre throttle para o minimo por 1s para armar");
      lastArmLog = millis();
    }
    return;
  }

  // Sistema armado — operação normal
  int throttle = (int)ch2 - 1500;
  int steering = (int)ch1 - 1500;
  if (abs(throttle) < 50) throttle = 0;
  if (abs(steering) < 50) steering = 0;

  int alvo_esq = constrain(map(throttle + (steering / 2), -500, 500, -PWM_MAX, PWM_MAX), -PWM_MAX, PWM_MAX);
  int alvo_dir = constrain(map(throttle - (steering / 2), -500, 500, -PWM_MAX, PWM_MAX), -PWM_MAX, PWM_MAX);

  // SwC 3 posições: cima (~1000) = abre | meio (~1500) = parado | baixo (~2000) = fecha
  // Comportamento real do braço: saída 0 = abre | +220 = parado | -220 = fecha
  int alvo_pistao = PISTON_PWM;                   // meio  → segura parado
  if (ch6 < 1200)      alvo_pistao = 0;           // cima  → abre o braço
  else if (ch6 > 1800) alvo_pistao = -PISTON_PWM; // baixo → fecha o braço

  digitalWrite(MOTOR_LAMINA, ch7 > 1500 ? HIGH : LOW);
  acionarMotores(alvo_esq, alvo_dir);
  acionarPistao(alvo_pistao);

  // Detecta qualquer canal que mudou (para mapeamento de controles)
  logIbusMudancas();

  // Log periódico de status normal
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 1000) {
    Serial.printf("[RADIO ] ESQ:%4d DIR:%4d | PISTAO:%4d | LAMINA:%s\n",
      alvo_esq, alvo_dir, alvo_pistao,
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
      if (!armed) {
        // Pisca rápido: 100ms — aguardando armação
        digitalWrite(LED_PIN, (now / 100) % 2 == 0 ? HIGH : LOW);
      } else {
        // Duplo pisca: 0–100ms ON | 100–200ms OFF | 200–300ms ON | 300–1000ms OFF
        unsigned long t = now % 1000;
        bool on = (t < 100) || (t >= 200 && t < 300);
        digitalWrite(LED_PIN, on ? HIGH : LOW);
      }
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

void acionarPistao(int v) {
  if (v > 0) {
    ledcWrite(PISTON_RPWM, v); ledcWrite(PISTON_LPWM, 0);
  } else if (v < 0) {
    ledcWrite(PISTON_RPWM, 0); ledcWrite(PISTON_LPWM, abs(v));
  } else {
    ledcWrite(PISTON_RPWM, 0); ledcWrite(PISTON_LPWM, 0);
  }
}

void desenergizar_pistao() {
  // Corta TODA a energia do atuador (0/0). Usado sempre que não há
  // comunicação com o rádio ou o sistema está desarmado.
  // Atenção: pela característica do braço, sem energia ele tende a ABRIR.
  ledcWrite(PISTON_RPWM, 0); ledcWrite(PISTON_LPWM, 0);
}
