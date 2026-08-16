// ============================================================
//  TESTE DE LINK iBUS  -  FlySky FS-iA6B  ->  ESP32
//  Objetivo: descobrir em que ponto a comunicacao esta quebrando.
//  Nao mexe em motor nenhum. Grave, abra o Serial Monitor 115200.
//
//  Ligacao esperada (mesma do firmware do projeto):
//    RX i-BUS SERVO  sinal  -> GPIO 13 do ESP32
//    RX i-BUS SERVO  GND    -> GND do ESP32   (obrigatorio, terra comum)
//    RX B/VCC ou i-BUS VCC  -> 5V (VIN) do ESP32
// ============================================================

#define IBUS_RX 13

HardwareSerial ibus(1);

uint8_t  buf[32];
int      idx = 0;
uint16_t ch[14];
unsigned long ultimoPacote = 0;
unsigned long ultimoLog    = 0;
unsigned long bytesBrutos  = 0;
unsigned long pacotesOk    = 0;
unsigned long checksumRuim = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== TESTE iBUS - GPIO13 @ 115200 ===");
  Serial.println("Ligue o radio FS-i6X ANTES de olhar o resultado.");
  ibus.begin(115200, SERIAL_8N1, IBUS_RX, -1);
}

void loop() {
  while (ibus.available()) {
    uint8_t b = ibus.read();
    bytesBrutos++;

    // Pacote iBUS = 0x20 0x40 + 28 bytes de canais + 2 de checksum
    if (idx == 0 && b != 0x20) continue;
    if (idx == 1 && b != 0x40) { idx = 0; continue; }

    buf[idx++] = b;

    if (idx == 32) {
      idx = 0;
      uint16_t soma = 0xFFFF;
      for (int i = 0; i < 30; i++) soma -= buf[i];
      uint16_t chk = buf[30] | (buf[31] << 8);

      if (soma == chk) {
        for (int i = 0; i < 14; i++) ch[i] = buf[2 + i * 2] | (buf[3 + i * 2] << 8);
        ultimoPacote = millis();
        pacotesOk++;
      } else {
        checksumRuim++;
      }
    }
  }

  if (millis() - ultimoLog > 500) {
    ultimoLog = millis();

    if (millis() - ultimoPacote < 1000) {
      Serial.print("[OK] pacotes=");
      Serial.print(pacotesOk);
      Serial.print(" | ");
      for (int i = 0; i < 10; i++) {
        Serial.print("CH");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(ch[i]);
        Serial.print(" ");
      }
      Serial.println();
    } else if (bytesBrutos > 0) {
      Serial.print("[?] Chegam bytes mas nenhum pacote iBUS valido. bytes=");
      Serial.print(bytesBrutos);
      Serial.print(" checksum_ruim=");
      Serial.println(checksumRuim);
      Serial.println("    -> provavel: fio no conector errado (canal PWM/PPM ou SENS) ou baud errado.");
    } else {
      Serial.println("[X] ZERO bytes no GPIO13.");
      Serial.println("    -> provavel: sinal fora da porta i-BUS SERVO, GND nao comum, RX sem 5V ou RX sem bind.");
    }
  }
}
