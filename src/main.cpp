// ==== Task 76 – Economia de Energia (ESP32-C3 SuperMini) ====
// Versão com ADS1115 (I2C) para leitura de VBAT.
// Recursos: histerese, Wi-Fi OFF em bateria, light/deep-sleep, wake por PG (EXT0),
// filtro mediana no ADC externo, contadores de tempo por modo, duty-cycle por modo, logs.

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_sleep.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <algorithm>  // std::sort

// ================== PINAGEM (AJUSTE AQUI) ==================
#ifndef LED_BUILTIN
#define LED_BUILTIN 8           // Troque para o LED da sua placa (alguns SuperMini usam GPIO2)
#endif

// I2C do ESP32-C3 SuperMini (ajuste se necessário)
constexpr int I2C_SDA = 8;
constexpr int I2C_SCL = 9;

// Sinais do MCP73871 (open-drain; puxe para 3V3 com resistor/pull-up interno se necessário)
constexpr int PIN_PG  = 4;      // Power-Good: 1 = fonte presente, 0 = em bateria (precisa ser RTC-GPIO p/ wake EXT0)
constexpr int PIN_LBO = 5;      // Low Battery Output (~3.1V do módulo Adafruit), normalmente ativo em nível BAIXO

// Se o seu módulo já fornece pull-ups externos em PG/LBO, troque INPUT_PULLUP por INPUT.
constexpr bool USE_INTERNAL_PULLUPS = true;

// ================== ADS1115 ==================
Adafruit_ADS1115 ads;
constexpr uint8_t ADS_ADDR = 0x48;  // ADDR em GND → 0x48

// ================== CALIBRAÇÃO DO DIVISOR ==================
/*
  Exemplo de divisor para VBAT: 220k (top) / 100k (bottom)
  Fator = Rbottom / (Rtop + Rbottom) = 100k / 320k = 0.3125
  Vbat = Vadc / fator
*/
constexpr float DIV_FACTOR = 0.3125f;                 // Ajuste conforme seu divisor
inline float VBAT_FROM_ADC_V(float vAdc) { return vAdc / DIV_FACTOR; }

// ================== LIMIARES COM HISTERese ==================
constexpr float VBAT_WARN_ENTER = 3.60f;  // Entrar em SAVING
constexpr float VBAT_WARN_EXIT  = 3.70f;  // Sair de SAVING
constexpr float VBAT_CRIT_ENTER = 3.30f;  // Entrar em CRITICAL
constexpr float VBAT_CRIT_EXIT  = 3.40f;  // Sair de CRITICAL

// Debounce de transição PG (rede ↔ bateria)
constexpr uint32_t ON_BATT_GRACE_MS = 1000;

// Duty-cycle de amostragem por modo
constexpr uint32_t SAMPLE_LOG_MS   = 1000;
constexpr uint32_t SAVING_SLEEP_MS = 500;
constexpr uint32_t CRIT_SLEEP_MS   = 1000;

// Deep sleep em crítico persistente
constexpr uint32_t CRIT_PERSIST_MS = 30000;                           // após 30s crítico → deep sleep
constexpr uint64_t DEEP_SLEEP_US   = 5ULL * 60ULL * 1000000ULL;       // 5 minutos

// ================== CONTROLE DE MODO ==================
enum class PowerMode { NORMAL, SAVING, CRITICAL };
PowerMode currentMode = PowerMode::NORMAL;

bool onMains = true;                // verdadeiro se PG=HIGH
uint32_t lastPgChangeMs = 0;
uint32_t tEnterModeMs   = 0;
uint32_t tAccumNormal   = 0, tAccumSaving = 0, tAccumCritical = 0;
uint32_t criticalSince  = 0;

// ================== UTILITÁRIOS ==================
void wifiOff() {
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop(); // garante modem realmente parado
}

void wifiOn() {
  WiFi.mode(WIFI_STA);
  // WiFi.begin("SSID","SENHA"); // habilite se precisar
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // PS leve no NORMAL
}

void toCpu160() { setCpuFrequencyMhz(160); }
void toCpu80()  { setCpuFrequencyMhz(80);  }

void enterLightSleep(uint32_t ms) {
  esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);
  esp_light_sleep_start(); // retorna após acordar
}

void enterDeepSleepFor(uint64_t us) {
  esp_sleep_enable_timer_wakeup(us);
  esp_deep_sleep_start(); // não retorna
}

// Habilitar wake por retorno da rede (PG=HIGH). IMPORTANTE: PIN_PG deve ser RTC-capable.
void enableWakeOnPG() {
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_PG, 1); // wake quando PG=HIGH
}

// Contabiliza tempo no modo anterior e troca de modo
void switchMode(PowerMode next) {
  uint32_t now = millis();
  uint32_t delta = now - tEnterModeMs;
  if (tEnterModeMs != 0) {
    if      (currentMode == PowerMode::NORMAL)   tAccumNormal   += delta;
    else if (currentMode == PowerMode::SAVING)   tAccumSaving   += delta;
    else if (currentMode == PowerMode::CRITICAL) tAccumCritical += delta;
  }
  tEnterModeMs = now;
  currentMode = next;
}

// ================== LEITURA DO VBAT (ADS1115 + MEDIANA) ==================
float readVBAT() {
  const int N = 7;
  int16_t buf[N];

  for (int i = 0; i < N; ++i) {
    // Canal A0 (single-ended). Use A1/A2/A3 conforme sua fiação para outras tensões.
    int16_t raw = ads.readADC_SingleEnded(0);
    buf[i] = raw;
    delay(2);
  }
  std::sort(buf, buf + N);
  int16_t rawMedian = buf[N / 2];

  // Converte para Volts no pino (usa ganho configurado via ads.setGain)
  float vAdc = ads.computeVolts(rawMedian);

  // Converte para tensão real da bateria via divisor
  return VBAT_FROM_ADC_V(vAdc);
}

// ================== POLÍTICA DE MODO ==================
void applyNormalMode() {
  wifiOn();        // habilite Wi-Fi somente se necessário no seu caso
  toCpu160();      // desempenho total
  digitalWrite(LED_BUILTIN, HIGH);
}

void applySavingMode() {
  wifiOff();
  toCpu80();
  digitalWrite(LED_BUILTIN, LOW);
  enterLightSleep(SAVING_SLEEP_MS);
}

void applyCriticalMode() {
  wifiOff();
  toCpu80();
  digitalWrite(LED_BUILTIN, LOW);
  enterLightSleep(CRIT_SLEEP_MS);
}

// Regras de transição com histerese
void updateModeByVbat(float vbat) {
  if (!onMains) {
    // Estamos em bateria
    switch (currentMode) {
      case PowerMode::NORMAL:
        // Caiu pra bateria → vai para SAVING direto
        switchMode(PowerMode::SAVING);
        break;
      case PowerMode::SAVING:
        if (vbat <= VBAT_CRIT_ENTER) {
          switchMode(PowerMode::CRITICAL);
          criticalSince = millis();
        }
        break;
      case PowerMode::CRITICAL:
        if (vbat >= VBAT_CRIT_EXIT) {
          switchMode(PowerMode::SAVING);
          criticalSince = 0;
        }
        break;
    }
  } else {
    // Fonte presente → NORMAL
    if (currentMode != PowerMode::NORMAL) {
      switchMode(PowerMode::NORMAL);
      criticalSince = 0;
    }
  }
}

// ================== SETUP/LOOP ==================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n[BOOT] Task 76 - Economia de Energia (ESP32-C3 + ADS1115)"));

  // I2C + ADS1115
  Wire.begin(I2C_SDA, I2C_SCL);
  // Ganho ±2.048 V (GAIN_TWO): ótima resolução para ~1.31 V do divisor quando VBAT=4.2 V
  ads.setGain(GAIN_TWO); // LSB ~62.5 µV
  if (!ads.begin(ADS_ADDR)) {
    Serial.println(F("[ERR] ADS1115 nao encontrado (0x48). Verifique I2C/ADDR."));
    while (1) delay(10);
  }

  // Entradas digitais
  if (USE_INTERNAL_PULLUPS) {
    pinMode(PIN_PG,  INPUT_PULLUP);
    pinMode(PIN_LBO, INPUT_PULLUP);
  } else {
    pinMode(PIN_PG,  INPUT);
    pinMode(PIN_LBO, INPUT);
  }

  // Estado inicial baseado em PG (rede presente?)
  onMains = (digitalRead(PIN_PG) == HIGH);
  tEnterModeMs = millis();
  if (onMains) {
    switchMode(PowerMode::NORMAL);
    applyNormalMode();
    Serial.println(F("[MODE] NORMAL (fonte presente)"));
  } else {
    switchMode(PowerMode::SAVING);
    applySavingMode();
    Serial.println(F("[MODE] SAVING (em bateria)"));
  }

  // Wake-up por retorno da rede (PG=HIGH) em deep-sleep
  enableWakeOnPG();

  // Opcional: manter estado de GPIOs durante deep-sleep (ex.: LED)
  // gpio_hold_en((gpio_num_t)LED_BUILTIN);
  // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
}

void loop() {
  // 1) Debounce de PG (rede ↔ bateria)
  bool pgNow = (digitalRead(PIN_PG) == HIGH);
  if (pgNow != onMains) {
    if (millis() - lastPgChangeMs > ON_BATT_GRACE_MS) {
      onMains = pgNow;
      lastPgChangeMs = millis();
      Serial.printf("[PG] Fonte %s\n", onMains ? "PRESENTE" : "AUSENTE (BAT)");
    }
  }

  // 2) Leitura de VBAT e LBO (~3.1V fixo do módulo Adafruit; checar ativo-alto/baixo)
  float vbat = readVBAT();
  bool lboActive = (digitalRead(PIN_LBO) == LOW); // ajuste se seu LBO for ativo-alto

  // 3) Política baseada em VBAT + histerese + LBO
  updateModeByVbat(vbat);

  // Se LBO disparar, força CRITICAL (último estágio) mesmo que a tensão oscile
  if (!onMains && lboActive && currentMode != PowerMode::CRITICAL) {
    switchMode(PowerMode::CRITICAL);
    criticalSince = millis();
    Serial.println(F("[LBO] Low-battery asserted → CRITICAL"));
  }

  // 4) Ações por modo
  switch (currentMode) {
    case PowerMode::NORMAL:   applyNormalMode();   break;
    case PowerMode::SAVING:   applySavingMode();   break;
    case PowerMode::CRITICAL: applyCriticalMode(); break;
  }

  // 5) Deep-sleep se crítico persistir
  if (currentMode == PowerMode::CRITICAL && !onMains) {
    if (criticalSince != 0 && (millis() - criticalSince) > CRIT_PERSIST_MS) {
      Serial.println(F("[ACTION] Deep sleep (5 min) – crítico persistente"));
      // Acorda por timer OU retorno de rede (EXT0 em PG=HIGH)
      enterDeepSleepFor(DEEP_SLEEP_US);
    }
  }

  // 6) Telemetria de validação (log periódico)
  static uint32_t lastLog = 0;
  if (millis() - lastLog > SAMPLE_LOG_MS) {
    lastLog = millis();
    Serial.printf("[STAT] VBAT=%.3f V, PG=%d, LBO=%d, MODE=%d | T[N:%lus S:%lus C:%lus]\n",
                  vbat, (int)onMains, (int)lboActive, (int)currentMode,
                  tAccumNormal/1000, tAccumSaving/1000, tAccumCritical/1000);
  }

  // Pequeno atraso para evitar busy-loop quando não estamos em sleep
  delay(20);
}
// ============================================================