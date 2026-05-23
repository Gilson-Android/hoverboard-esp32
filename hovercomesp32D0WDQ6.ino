#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// --- PINOS MOTORES TRAÇÃO (IBT-2) ---
const int MOTOR_ESQ_RPWM = 12; // P12
const int MOTOR_ESQ_LPWM = 14; // P14
const int MOTOR_DIR_RPWM = 16; // P16
const int MOTOR_DIR_LPWM = 17; // P17

// --- PINO MOTOR DA LÂMINA (ROÇADEIRA) ---
const int MOTOR_LAMINA = 23;   // P23 (Ligue em um Relé ou Driver)

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

const int PWM_MAX = 220; 
bool conectado_anteriormente = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- CONFIGURANDO SENTIDO: CIMA = FRENTE ---");

  // Configuração PWM (ESP32 SDK 3.0+)
  ledcAttach(MOTOR_ESQ_RPWM, 5000, 8); 
  ledcAttach(MOTOR_ESQ_LPWM, 5000, 8);
  ledcAttach(MOTOR_DIR_RPWM, 5000, 8);
  ledcAttach(MOTOR_DIR_LPWM, 5000, 8);
  
  // Pino da Lâmina (On/Off simples ou PWM)
  pinMode(MOTOR_LAMINA, OUTPUT);
  digitalWrite(MOTOR_LAMINA, LOW);

  xboxController.begin();
}

void loop() {
  xboxController.onLoop();

  if (xboxController.isConnected()) {
    if (!conectado_anteriormente) {
      Serial.println("\n[OK] CONECTADO!");
      conectado_anteriormente = true;
    }

    if (!xboxController.isWaitingForFirstNotification()) {
      
      // --- TRATAMENTO DOS EIXOS ---
      // Invertemos o joyLVert para que CIMA seja POSITIVO
      int32_t joyY = 32768 - (int32_t)xboxController.xboxNotif.joyLVert; 
      int32_t joyX = (int32_t)xboxController.xboxNotif.joyLHori - 32768;

      // Deadzone para evitar trepidação
      if (abs(joyY) < 3500) joyY = 0;
      if (abs(joyX) < 3500) joyX = 0;

      // Mixer Tank Drive
      int alvo_esq = map(joyY + (joyX / 2), -32768, 32767, -PWM_MAX, PWM_MAX);
      int alvo_dir = map(joyY - (joyX / 2), -32768, 32767, -PWM_MAX, PWM_MAX);

      // Controle da Lâmina pelo Gatilho Direito (RT)
      // Se apertar mais que a metade do gatilho, liga a lâmina
      if (xboxController.xboxNotif.trigRT > 512) {
        digitalWrite(MOTOR_LAMINA, HIGH);
      } else {
        digitalWrite(MOTOR_LAMINA, LOW);
      }

      acionarMotores(alvo_esq, alvo_dir);

      // --- LOG DE TELEMETRIA ---
      static unsigned long lastLog = 0;
      if (millis() - lastLog > 500) {
        Serial.print("JOY [Y: "); Serial.print(joyY); Serial.print(" X: "); Serial.print(joyX); Serial.print("] ");
        Serial.print("| ESQ: "); Serial.print(alvo_esq > 0 ? "FRENTE " : "TRÁS "); Serial.print(abs(alvo_esq));
        Serial.print(" | DIR: "); Serial.print(alvo_dir > 0 ? "FRENTE " : "TRÁS "); Serial.print(abs(alvo_dir));
        Serial.print(" | LÂMINA: "); Serial.println(digitalRead(MOTOR_LAMINA) ? "LIGADA" : "DESLIGADA");
        lastLog = millis();
      }
    }
  } else {
    if (conectado_anteriormente) {
      Serial.println("\n[!] CONEXÃO PERDIDA - PARANDO TUDO");
      parar_motores();
      digitalWrite(MOTOR_LAMINA, LOW);
      conectado_anteriormente = false;
    }
    delay(500);
  }
}

void acionarMotores(int v_esq, int v_dir) {
  // Motor Esquerdo
  if (v_esq >= 0) {
    ledcWrite(MOTOR_ESQ_RPWM, abs(v_esq)); ledcWrite(MOTOR_ESQ_LPWM, 0);
  } else {
    ledcWrite(MOTOR_ESQ_RPWM, 0); ledcWrite(MOTOR_ESQ_LPWM, abs(v_esq));
  }

  // Motor Direito
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