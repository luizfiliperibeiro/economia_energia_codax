#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_sleep.h"

// ============ PINAGEM (AJUSTE AQUI) ============
// Sinais do MCP73871 (open-drain com pull-ups a 3V3)
constexpr int PIN_PG  = 4;   // Power Good  (1=fonte presente, 0=on-battery)  <-- ajuste
constexpr int PIN_LBO = 5;   // Low Battery Output (~3.1V threshold)          <-- ajuste
// Leitura de VBAT (divisor resistivo até 3V3)
constexpr int PIN_VBAT_ADC = A0; // GPIO analógico do C3                        <-- ajuste

// ============ CALIBRAÇÃO DO ADC / DIVISOR ============
/*
  Exemplo de divisor: 220k (top) / 100k (bottom) -> fator = 100k / (220k+100k) = 0.3125
  Vbat = Vadc / 0.3125
*/
constexpr float ADC_REF = 3.30f;        // tensão de referência do ADC (~3.3V)
constexpr int   ADC_MAX = 4095;         // 12 bits no Arduino-ESP32C3
constexpr float DIV_FACTOR = 0.3125f;   // (Rbottom / (Rtop+Rbottom))  --> ajuste se usar outro divisor
constexpr float VBAT_FROM_ADC(float vAdc) { return vAdc / DIV_FACTOR; }

// ============ LIMIARES DE ENERGIA (AJUSTE CONFORME PROJETO) ============
constexpr float VBAT_WARN     = 3.30f;  // entrar em modo economia
constexpr float VBAT_CRITICAL = 3.10f;  // reduzir ao mínimo / preparar deep sleep
constexpr uint32_t ON_BATT_GRACE_MS = 1000;  // debounce para mudanças de PG

// ============ AMOSTRAGEM ============
constexpr uint32_t SAMPLE_MS = 1000;

// Controle interno
enum class PowerMode { NORMAL, SAVING, CRITICAL };
PowerMode currentMode = PowerMode::NORMAL;
bool onMains = true; // fonte presente
uint32_t lastPgChangeMs = 0;

// ---- Utilitários ----
float readVBAT() {
  // média simples de N leituras para reduzir ruído
  const int N = 8;
  uint32_t acc = 0;
  for (int i=0;i<N;i++) { acc += analogRead(PIN_VBAT_ADC); delay(2); }
  float raw = (float)acc / N;
  float vAdc = (raw / ADC_MAX) * ADC_REF;
  return VBAT_FROM_ADC(vAdc);
}

void wifiOff() {
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop(); // garante modem realmente parado
}

void wifiOn() {
  // Ligue apenas se precisar; aqui deixo desconectado por padrão
  WiFi.mode(WIFI_STA);
  // WiFi.begin("SSID","PASS"); // se precisar
}

void enterLightSleep(uint32_t ms) {
  // Timer wake-up
  esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);
  // Light sleep devolve para o mesmo ponto após acordar
  esp_light_sleep_start();
}

void enterDeepSleep(uint32_t ms) {
  esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);
  esp_deep_sleep_start();
}

// Reduz clock da CPU para economia (opcional; mantenha 80 MHz ou 40 MHz)
void setCpuFreq80MHz() {
  setCpuFrequencyMhz(80); // em muitos C3 funciona 80MHz; 40MHz também possível
}

// ---- Estratégias de modo ----
void toNormalMode() {
  if (currentMode == PowerMode::NORMAL) return;
  // Reativar o que for necessário
  // wifiOn();  // só se precisar mesmo
  setCpuFrequencyMhz(160); // desempenho total (se for necessário)
  currentMode = PowerMode::NORMAL;
  Serial.println(F("[MODE] NORMAL"));
}

void toSavingMode() {
  if (currentMode == PowerMode::SAVING) return;
  // Cortes típicos:
  wifiOff();          // maior ganho de consumo
  setCpuFreq80MHz();  // reduz CPU
  currentMode = PowerMode::SAVING;
  Serial.println(F("[MODE] SAVING (WiFi OFF, CPU down)"));
}

void toCriticalMode() {
  if (currentMode == PowerMode::CRITICAL) return;
  // Cortes máximos:
  wifiOff();
  setCpuFreq80MHz();
  currentMode = PowerMode::CRITICAL;
  Serial.println(F("[MODE] CRITICAL (min resources)"));
}

// ---- Setup/Loop ----
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_PG,  INPUT);    // com pull-up externo do módulo
  pinMode(PIN_LBO, INPUT);    // idem
  analogReadResolution(12);

  // Começa no modo “normal” supondo fonte presente (PG=1)
  onMains = digitalRead(PIN_PG) == HIGH;
  if (onMains) toNormalMode(); else toSavingMode();

  // Se não for usar Wi-Fi no protótipo, deixe desligado:
  wifiOff();
}

void loop() {
  // 1) Detecta presença de fonte (PG) com debounce
  bool pgNow = (digitalRead(PIN_PG) == HIGH);
  if (pgNow != onMains) {
    if (millis() - lastPgChangeMs > ON_BATT_GRACE_MS) {
      onMains = pgNow;
      lastPgChangeMs = millis();
      Serial.printf("[PG] Fonte %s\n", onMains ? "PRESENTE" : "AUSENTE (BAT)");
      if (onMains) toNormalMode(); else toSavingMode();
    }
  }

  // 2) Mede VBAT e lê LBO
  float vbat = readVBAT();
  bool lbo  = (digitalRead(PIN_LBO) == LOW); // depende do módulo; ajuste se ativo-alto

  // 3) Política de energia
  if (!onMains) {
    // Estamos em bateria → aplicar thresholds por software
    if (vbat <= VBAT_CRITICAL || lbo) {
      toCriticalMode();
      // Light-sleep entre leituras para economizar
      enterLightSleep(500);  // dorme 0,5 s e volta
      // Se permanecer crítico por muito tempo, podemos entrar em deep sleep:
      static uint32_t t0 = millis();
      if (millis() - t0 > 30000) {  // 30s crítico → hibernar 5 minutos
        Serial.println(F("[ACTION] Deep sleep 5 min (critical)"));
        enterDeepSleep(5UL * 60UL * 1000UL);
      }
    } else if (vbat <= VBAT_WARN) {
      toSavingMode();
      enterLightSleep(200); // duty-cyclezinho para economizar
    } else {
      // Em bateria mas ainda confortável
      toSavingMode();
    }
  } else {
    // Fonte presente → modo normal
    toNormalMode();
  }

  // 4) Telemetria básica
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > SAMPLE_MS) {
    lastPrint = millis();
    Serial.printf("[STAT] VBAT=%.3f V, PG=%d, LBO=%d, MODE=%d\n",
                  vbat, (int)onMains, (int)lbo, (int)currentMode);
  }

  delay(50);
}
// Fim