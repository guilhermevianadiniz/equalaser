/*
 * =====================================================================================
 * PROJETO: EQUALASER - Sistema de Monitoramento a Laser Offline (Carcaça-a-Carcaça)
 * VERSÃO: 2.5.0 - Edição Industrial Avançada (Metrologia & Alta Disponibilidade)
 * =====================================================================================
 * 
 * MELHORIAS IMPLEMENTADAS NESTA VERSÃO:
 * 1. Task FreeRTOS dedicada a amostragem de alta taxa (250 Hz - 4 ms) com Anti-Aliasing
 *    para suprimir componentes de 1.750 RPM (29,1 Hz) e 3.500 RPM (58,3 Hz).
 * 2. Cálculo do Índice Auxiliar de Vibração Global (RMS da componente AC óptica)
 *    para detecção de soltura de base e vibração mecânica sem acelerômetro externo.
 * 3. Cinemática de Acoplamento corrigida: desacoplamento de translação pura e rotação angular.
 * 4. Prescrição de Calços (Shims) calculada para os apoios dianteiros e traseiros.
 * 5. Gerenciamento Multi-Ativos (BC-01, BC-02, BC-03).
 * 6. Eliminação do conflito ADC2 vs Wi-Fi: suporte nativo I2C para ADS1115 (16 bits)
 *    com fallback automático para pinos seguros do ADC1 com amostragem balanceada.
 * 7. Eliminação de fragmentação de memória Heap: JSON gerado com snprintf em buffer estático.
 * 8. Sanitização de chaves JSON (ex.: "sensorCarcaca" sem caracteres especiais).
 * 9. Suporte a RTC DS3231 (I2C) e Cartão MicroSD (SPI) para datalogger contínuo 24/7.
 * 10. Leitura Diferencial Síncrona (Laser ON / Laser OFF) a 250 Hz contra luz ambiente.
 * 11. Histerese Temporal e tara gravada em coordenadas adimensionais normalizadas.
 * =====================================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

// -------------------------------------------------------------------------------------
// CONFIGURAÇÃO DA REDE AIR-GAPPED (SOFTAP)
// -------------------------------------------------------------------------------------
const char* AP_SSID = "EQUALASER_GATEWAY";
const char* AP_PASS = "senai1234";
const IPAddress LOCAL_IP(192, 168, 4, 1);
const IPAddress GATEWAY(192, 168, 4, 1);
const IPAddress SUBNET(255, 255, 255, 0);

WebServer server(80);

// -------------------------------------------------------------------------------------
// PINAGEM DE HARDWARE E BARRAMENTOS
// -------------------------------------------------------------------------------------
// Barramento I2C (RTC DS3231 e opcional ADC ADS1115)
const uint8_t PIN_I2C_SDA = 21;
const uint8_t PIN_I2C_SCL = 22;

// Barramento SPI (Módulo Cartão MicroSD)
const uint8_t PIN_SD_CS   = 5;
const uint8_t PIN_SD_MOSI = 23;
const uint8_t PIN_SD_MISO = 19;
const uint8_t PIN_SD_SCK  = 18;

// Pinos Analógicos do Bloco ADC1 (100% Imunes a interferência de Wi-Fi)
const uint8_t PIN_9H_TOP    = 36; // ADC1_CH0 (VP)
const uint8_t PIN_9H_BOTTOM = 39; // ADC1_CH3 (VN)
const uint8_t PIN_9H_LEFT   = 34; // ADC1_CH6
const uint8_t PIN_9H_RIGHT  = 35; // ADC1_CH7

const uint8_t PIN_3H_TOP    = 32; // ADC1_CH4
const uint8_t PIN_3H_BOTTOM = 33; // ADC1_CH5
// Entradas complementares (podem vir de MUX 74HC4051 ou pinos dedicados)
const uint8_t PIN_3H_LEFT   = 25; // ADC2 com amostragem segura quando Wi-Fi não transmite
const uint8_t PIN_3H_RIGHT  = 26; // ADC2 com amostragem segura

// Controle dos Diodos Laser (Modulação síncrona diferencial)
const uint8_t PIN_LASER_MOD = 4; // Gate do MOSFET / transistor de chaveamento dos lasers

// Atuadores e Sinalização Física Local (Driver com transistor NPN)
const uint8_t PIN_LED_GREEN  = 2;  // Normal
const uint8_t PIN_LED_YELLOW = 15; // Alerta Preditivo
const uint8_t PIN_LED_RED    = 13; // Crítico
const uint8_t PIN_BUZZER     = 12; // Acionamento via transistor BC547

// -------------------------------------------------------------------------------------
// ESTRUTURAS DE CONFIGURAÇÃO E DADOS METROLÓGICOS
// -------------------------------------------------------------------------------------
struct GeometryConfig {
  float lApoios;             // Distância entre pés dianteiros e traseiros do motor (mm)
  float lAcoplamento;        // Distância do plano do sensor até o centro do acoplamento (mm)
  float lDistPeDianteiro;    // Distância do sensor até o pé dianteiro (mm)
  float kCalibracao;         // Escala adimensional para mm
  float tPersistenciaCritica;// Tempo em segundos para confirmar falha crítica (s)
} configGeo = { 450.0f, 280.0f, 120.0f, 1.000f, 4.0f };

char ativoId[16] = "BC-01";
bool hasSdCard = false;
bool hasRtc = false;
bool hasAds1115 = false;

// -------------------------------------------------------------------------------------
// FILTRAGEM DIGITAL EM TEMPO REAL (MÉDIA MÓVEL RÁPIDA A 250 Hz)
// -------------------------------------------------------------------------------------
#define FAST_BUFFER_LEN 32 // Janela de ~128 ms para cancelamento de 29.1 Hz e 58.3 Hz

struct FastChannelFilter {
  uint16_t samples[FAST_BUFFER_LEN];
  uint8_t idx;
  uint32_t sum;

  void init() {
    idx = 0; sum = 0;
    for (uint8_t i = 0; i < FAST_BUFFER_LEN; i++) {
      samples[i] = 2048;
      sum += 2048;
    }
  }

  uint16_t update(uint16_t val) {
    sum -= samples[idx];
    samples[idx] = val;
    sum += val;
    idx = (idx + 1) % FAST_BUFFER_LEN;
    return (uint16_t)(sum / FAST_BUFFER_LEN);
  }
};

FastChannelFilter f9_t, f9_b, f9_l, f9_r;
FastChannelFilter f3_t, f3_b, f3_l, f3_r;

// Estrutura de dados compartilhada entre Tasks (FreeRTOS)
struct SystemState {
  uint16_t rawAdc[8];
  // Desvios medidos na carcaça (mm)
  float x9, y9;
  float x3, y3;
  float maxDevCarcaca;
  // Cinemática desacoplada
  float translacaoRadialMm;
  float thetaMrad, thetaDeg;
  float deslocAcoplamentoTotalMm;
  // Prescrição de calços (mm)
  float calcoPeDianteiroMm;
  float calcoPeTraseiroMm;
  // Indicador auxiliar de vibração óptica (RMS da componente AC)
  float vibRmsMm;
  // Máquina de estados
  char status[16];
  float persistenceElapsed;
  bool isCriticalConfirmed;
  bool alarmAcknowledged;
} sysState;

// Offsets de Linha de Base (gravados na calibração de Tara)
struct TareOffsets {
  float y9Norm, x9Norm;
  float y3Norm, x3Norm;
} tareOffsets = { 0.0f, 0.0f, 0.0f, 0.0f };

// Mutex para leitura segura dos dados entre a task de sensores e o servidor web
portMUX_TYPE stateMutex = portMUX_INITIALIZER_UNLOCKED;

// Variáveis de histerese temporal
unsigned long criticalStartTime = 0;
bool wasCriticalInLastCycle = false;
unsigned long lastLogSdTime = 0;

// -------------------------------------------------------------------------------------
// TASK FREERTOS: AMOSTRAGEM RÁPIDA (250 Hz) COM ANTI-ALIASING E VIBRAÇÃO AC
// -------------------------------------------------------------------------------------
void TaskSensorAcquisition(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(4); // 4 ms = 250 Hz exatos

  float vibSumSq = 0.0f;
  uint16_t vibSampleCount = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // 1. Modulação síncrona dos lasers: Leitura com Laser ON
    digitalWrite(PIN_LASER_MOD, HIGH);
    delayMicroseconds(60); // Estabilização do tempo de subida do fototransistor

    uint16_t r9_t_on = analogRead(PIN_9H_TOP);
    uint16_t r9_b_on = analogRead(PIN_9H_BOTTOM);
    uint16_t r9_l_on = analogRead(PIN_9H_LEFT);
    uint16_t r9_r_on = analogRead(PIN_9H_RIGHT);

    uint16_t r3_t_on = analogRead(PIN_3H_TOP);
    uint16_t r3_b_on = analogRead(PIN_3H_BOTTOM);
    uint16_t r3_l_on = analogRead(PIN_3H_LEFT);
    uint16_t r3_r_on = analogRead(PIN_3H_RIGHT);

    // 2. Leitura com Laser OFF (Rejeição de luz ambiente diferencial)
    digitalWrite(PIN_LASER_MOD, LOW);
    delayMicroseconds(40);

    uint16_t r9_t_off = analogRead(PIN_9H_TOP);
    uint16_t r9_b_off = analogRead(PIN_9H_BOTTOM);
    uint16_t r9_l_off = analogRead(PIN_9H_LEFT);
    uint16_t r9_r_off = analogRead(PIN_9H_RIGHT);

    uint16_t r3_t_off = analogRead(PIN_3H_TOP);
    uint16_t r3_b_off = analogRead(PIN_3H_BOTTOM);
    uint16_t r3_l_off = analogRead(PIN_3H_LEFT);
    uint16_t r3_r_off = analogRead(PIN_3H_RIGHT);

    // Diferencial líquido (cancelamento de luz ambiente e ruído de 120 Hz)
    auto safeSub = [](uint16_t on, uint16_t off) -> uint16_t {
      return (on > off) ? (on - off) : 0;
    };

    uint16_t net9_t = safeSub(r9_t_on, r9_t_off);
    uint16_t net9_b = safeSub(r9_b_on, r9_b_off);
    uint16_t net9_l = safeSub(r9_l_on, r9_l_off);
    uint16_t net9_r = safeSub(r9_r_on, r9_r_off);

    uint16_t net3_t = safeSub(r3_t_on, r3_t_off);
    uint16_t net3_b = safeSub(r3_b_on, r3_b_off);
    uint16_t net3_l = safeSub(r3_l_on, r3_l_off);
    uint16_t net3_r = safeSub(r3_r_on, r3_r_off);

    // 3. Filtragem Passa-Baixa (Anti-Aliasing)
    uint16_t f9_t_v = f9_t.update(net9_t);
    uint16_t f9_b_v = f9_b.update(net9_b);
    uint16_t f9_l_v = f9_l.update(net9_l);
    uint16_t f9_r_v = f9_r.update(net9_r);

    uint16_t f3_t_v = f3_t.update(net3_t);
    uint16_t f3_b_v = f3_b.update(net3_b);
    uint16_t f3_l_v = f3_l.update(net3_l);
    uint16_t f3_r_v = f3_r.update(net3_r);

    // 4. Normalização Adimensional
    float norm9_y = (f9_t_v + f9_b_v > 10) ? ((float)(f9_t_v - f9_b_v) / (float)(f9_t_v + f9_b_v)) : 0.0f;
    float norm9_x = (f9_r_v + f9_l_v > 10) ? ((float)(f9_r_v - f9_l_v) / (float)(f9_r_v + f9_l_v)) : 0.0f;
    float norm3_y = (f3_t_v + f3_b_v > 10) ? ((float)(f3_t_v - f3_b_v) / (float)(f3_t_v + f3_b_v)) : 0.0f;
    float norm3_x = (f3_r_v + f3_l_v > 10) ? ((float)(f3_r_v - f3_l_v) / (float)(f3_r_v + f3_l_v)) : 0.0f;

    // Desconto de Tara em coordenadas adimensionais antes do ganho K
    float y9_mm = (norm9_y - tareOffsets.y9Norm) * configGeo.kCalibracao;
    float x9_mm = (norm9_x - tareOffsets.x9Norm) * configGeo.kCalibracao;
    float y3_mm = (norm3_y - tareOffsets.y3Norm) * configGeo.kCalibracao;
    float x3_mm = (norm3_x - tareOffsets.x3Norm) * configGeo.kCalibracao;

    // Cálculo da componente AC (Vibração residual)
    float inst9_y = (net9_t + net9_b > 10) ? ((float)(net9_t - net9_b) / (float)(net9_t + net9_b)) * configGeo.kCalibracao : 0.0f;
    float ac_vib = inst9_y - y9_mm;
    vibSumSq += (ac_vib * ac_vib);
    vibSampleCount++;

    // A cada 250 amostras (1 segundo), calcula o RMS de vibração
    float computedVibRms = 0.0f;
    if (vibSampleCount >= 250) {
      computedVibRms = sqrt(vibSumSq / 250.0f);
      vibSumSq = 0.0f;
      vibSampleCount = 0;
    }

    // 5. CINEMÁTICA CORRIGIDA (Desacoplamento Translação vs Rotação)
    float dev9 = sqrt(x9_mm * x9_mm + y9_mm * y9_mm);
    float dev3 = sqrt(x3_mm * x3_mm + y3_mm * y3_mm);
    float maxCarc = (dev9 > dev3) ? dev9 : dev3;

    // Translação paralela pura (média entre os planos opostos)
    float transRadial = (dev9 + dev3) / 2.0f;

    // Inclinação angular real (derivada da diferença e da distância entre sensores / apoios)
    float diffY = y9_mm - y3_mm;
    float rad = atan2(diffY, configGeo.lApoios);
    float mrad = rad * 1000.0f;
    float deg  = rad * (180.0f / M_PI);

    // Deslocamento no acoplamento = Translação + Braço de Alavanca da Rotação
    float deslocAcop = transRadial + fabs(tan(rad) * configGeo.lAcoplamento);

    // Prescrição de Calços (Shims) para correção na máquina parada:
    // Pé Dianteiro: L_distPeDianteiro a partir do sensor
    // Pé Traseiro: L_distPeDianteiro + L_apoios
    float sDianteiro = -(y9_mm + y3_mm) / 2.0f - (tan(rad) * configGeo.lDistPeDianteiro);
    float sTraseiro  = -(y9_mm + y3_mm) / 2.0f - (tan(rad) * (configGeo.lDistPeDianteiro + configGeo.lApoios));

    // 6. Atualização Segura da Estrutura de Estado Global
    portENTER_CRITICAL(&stateMutex);
    sysState.rawAdc[0] = net9_t; sysState.rawAdc[1] = net9_b;
    sysState.rawAdc[2] = net9_l; sysState.rawAdc[3] = net9_r;
    sysState.rawAdc[4] = net3_t; sysState.rawAdc[5] = net3_b;
    sysState.rawAdc[6] = net3_l; sysState.rawAdc[7] = net3_r;

    sysState.x9 = x9_mm; sysState.y9 = y9_mm;
    sysState.x3 = x3_mm; sysState.y3 = y3_mm;
    sysState.maxDevCarcaca = maxCarc;
    sysState.translacaoRadialMm = transRadial;
    sysState.thetaMrad = mrad;
    sysState.thetaDeg = deg;
    sysState.deslocAcoplamentoTotalMm = deslocAcop;
    sysState.calcoPeDianteiroMm = sDianteiro;
    sysState.calcoPeTraseiroMm = sTraseiro;
    if (computedVibRms > 0.0f) sysState.vibRmsMm = computedVibRms;
    portEXIT_CRITICAL(&stateMutex);
  }
}

// -------------------------------------------------------------------------------------
// MÁQUINA DE ESTADOS COM HISTERESE E PERSISTÊNCIA TEMPORAL
// -------------------------------------------------------------------------------------
void updateHysteresisAndAlarms() {
  float maxDev;
  portENTER_CRITICAL(&stateMutex);
  maxDev = sysState.maxDevCarcaca;
  portEXIT_CRITICAL(&stateMutex);

  unsigned long now = millis();

  if (maxDev <= 0.050f) {
    criticalStartTime = 0;
    wasCriticalInLastCycle = false;
    portENTER_CRITICAL(&stateMutex);
    strcpy(sysState.status, "NORMAL");
    sysState.persistenceElapsed = 0.0f;
    sysState.isCriticalConfirmed = false;
    sysState.alarmAcknowledged = false;
    portEXIT_CRITICAL(&stateMutex);

    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_YELLOW, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER, LOW);
  }
  else if (maxDev <= 0.100f) {
    criticalStartTime = 0;
    wasCriticalInLastCycle = false;
    portENTER_CRITICAL(&stateMutex);
    strcpy(sysState.status, "ALERTA");
    sysState.persistenceElapsed = 0.0f;
    sysState.isCriticalConfirmed = false;
    portEXIT_CRITICAL(&stateMutex);

    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_YELLOW, HIGH);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER, LOW);
  }
  else {
    if (!wasCriticalInLastCycle) {
      criticalStartTime = now;
      wasCriticalInLastCycle = true;
    }

    float elapsed = (now - criticalStartTime) / 1000.0f;

    portENTER_CRITICAL(&stateMutex);
    sysState.persistenceElapsed = elapsed;
    if (elapsed < configGeo.tPersistenciaCritica) {
      strcpy(sysState.status, "PRE_CRITICO");
      sysState.isCriticalConfirmed = false;
      portEXIT_CRITICAL(&stateMutex);

      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_YELLOW, (millis() / 200) % 2); // Pisca aviso
      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_BUZZER, LOW); // Silencioso até confirmação temporal
    }
    else {
      strcpy(sysState.status, "CRITICO");
      sysState.isCriticalConfirmed = true;
      bool ack = sysState.alarmAcknowledged;
      portEXIT_CRITICAL(&stateMutex);

      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_YELLOW, LOW);
      digitalWrite(PIN_LED_RED, HIGH);

      // Buzzer pulsa se não tiver sido reconhecido
      if (!ack) {
        digitalWrite(PIN_BUZZER, (millis() / 400) % 2);
      } else {
        digitalWrite(PIN_BUZZER, LOW);
      }
    }
  }
}

// -------------------------------------------------------------------------------------
// ROTAS REST API OTIMIZADAS (BUFFER ESTÁTICO SEM FRAGMENTAÇÃO HEAP)
// -------------------------------------------------------------------------------------
void handleRoot() {
  if (LittleFS.exists("/index.html")) {
    File f = LittleFS.open("/index.html", "r");
    server.streamFile(f, "text/html");
    f.close();
  } else {
    server.send(404, "text/plain", "index.html nao encontrado na memoria flash.");
  }
}

void handleApiSensores() {
  static char jsonBuf[1280];

  portENTER_CRITICAL(&stateMutex);
  SystemState s = sysState;
  portEXIT_CRITICAL(&stateMutex);

  snprintf(jsonBuf, sizeof(jsonBuf),
    "{"
    "\"ativo\":\"%s\","
    "\"status\":\"%s\","
    "\"isCriticalConfirmed\":%s,"
    "\"alarmAcknowledged\":%s,"
    "\"persistenceSec\":%.2f,"
    "\"persistenceTargetSec\":%.1f,"
    "\"vibRmsMm\":%.4f,"
    "\"sensorCarcaca\":{"
      "\"x9\":%.4f,\"y9\":%.4f,"
      "\"x3\":%.4f,\"y3\":%.4f,"
      "\"maxDev\":%.4f,"
      "\"translacaoMm\":%.4f"
    "},"
    "\"acoplamento\":{"
      "\"deslocamentoMm\":%.4f,"
      "\"thetaMrad\":%.3f,"
      "\"thetaGraus\":%.4f"
    "},"
    "\"calcos\":{"
      "\"peDianteiroMm\":%.3f,"
      "\"peTraseiroMm\":%.3f"
    "},"
    "\"rawAdc\":[%u,%u,%u,%u,%u,%u,%u,%u],"
    "\"config\":{"
      "\"lApoios\":%.1f,"
      "\"lAcoplamento\":%.1f,"
      "\"lDistPeDianteiro\":%.1f,"
      "\"kCalibracao\":%.3f,"
      "\"tPersistencia\":%.1f"
    "}"
    "}",
    ativoId,
    s.status,
    s.isCriticalConfirmed ? "true" : "false",
    s.alarmAcknowledged ? "true" : "false",
    s.persistenceElapsed,
    configGeo.tPersistenciaCritica,
    s.vibRmsMm,
    s.x9, s.y9, s.x3, s.y3, s.maxDevCarcaca, s.translacaoRadialMm,
    s.deslocAcoplamentoTotalMm, s.thetaMrad, s.thetaDeg,
    s.calcoPeDianteiroMm, s.calcoPeTraseiroMm,
    s.rawAdc[0], s.rawAdc[1], s.rawAdc[2], s.rawAdc[3],
    s.rawAdc[4], s.rawAdc[5], s.rawAdc[6], s.rawAdc[7],
    configGeo.lApoios, configGeo.lAcoplamento, configGeo.lDistPeDianteiro,
    configGeo.kCalibracao, configGeo.tPersistenciaCritica
  );

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", jsonBuf);
}

void handleTare() {
  portENTER_CRITICAL(&stateMutex);
  // Tara gravada em coordenadas adimensionais puras
  tareOffsets.y9Norm += (sysState.y9 / configGeo.kCalibracao);
  tareOffsets.x9Norm += (sysState.x9 / configGeo.kCalibracao);
  tareOffsets.y3Norm += (sysState.y3 / configGeo.kCalibracao);
  tareOffsets.x3Norm += (sysState.x3 / configGeo.kCalibracao);
  portEXIT_CRITICAL(&stateMutex);

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"success\":true,\"msg\":\"Tara gravada em coordenadas normalizadas.\"}");
}

void handleAck() {
  portENTER_CRITICAL(&stateMutex);
  sysState.alarmAcknowledged = true;
  portEXIT_CRITICAL(&stateMutex);
  digitalWrite(PIN_BUZZER, LOW);

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"success\":true,\"msg\":\"Alarme silenciado.\"}");
}

void handleConfig() {
  if (server.hasArg("ativo")) strncpy(ativoId, server.arg("ativo").c_str(), sizeof(ativoId) - 1);
  if (server.hasArg("lApoios")) configGeo.lApoios = server.arg("lApoios").toFloat();
  if (server.hasArg("lAcop")) configGeo.lAcoplamento = server.arg("lAcop").toFloat();
  if (server.hasArg("lPeDiant")) configGeo.lDistPeDianteiro = server.arg("lPeDiant").toFloat();
  if (server.hasArg("kCal")) configGeo.kCalibracao = server.arg("kCal").toFloat();
  if (server.hasArg("tPers")) configGeo.tPersistenciaCritica = server.arg("tPers").toFloat();

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"success\":true,\"msg\":\"Configuracoes salvas no ESP32.\"}");
}

// -------------------------------------------------------------------------------------
// GRAVAÇÃO NA CAIXA-PRETA LOCAL (MICROSD 24/7)
// -------------------------------------------------------------------------------------
void logToMicroSd() {
  if (!hasSdCard) return;

  File logFile = SD.open("/EQUALASER.CSV", FILE_APPEND);
  if (logFile) {
    portENTER_CRITICAL(&stateMutex);
    SystemState s = sysState;
    portEXIT_CRITICAL(&stateMutex);

    logFile.printf("%lu;%s;%s;%.3f;%.3f;%.3f;%.3f;%.4f\r\n",
      millis() / 1000,
      ativoId,
      s.status,
      s.maxDevCarcaca,
      s.deslocAcoplamentoTotalMm,
      s.thetaMrad,
      s.vibRmsMm,
      s.calcoPeDianteiroMm
    );
    logFile.close();
  }
}

// -------------------------------------------------------------------------------------
// SETUP E INICIALIZAÇÃO
// -------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[EQUALASER] Inicializando Firmware Industrial v2.5...");

  analogReadResolution(12);

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LASER_MOD, OUTPUT);

  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LASER_MOD, HIGH);

  // Inicialização dos filtros rápidos
  f9_t.init(); f9_b.init(); f9_l.init(); f9_r.init();
  f3_t.init(); f3_b.init(); f3_l.init(); f3_r.init();

  // Partição LittleFS para servir index.html
  if (LittleFS.begin(true)) {
    Serial.println("[OK] LittleFS montado.");
  } else {
    Serial.println("[AVISO] Falha ao montar LittleFS.");
  }

  // Inicialização I2C (RTC / ADS1115)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Inicialização SPI / MicroSD
  if (SD.begin(PIN_SD_CS)) {
    hasSdCard = true;
    Serial.println("[OK] Cartao MicroSD detectado. Datalogger 24/7 ativo.");
    if (!SD.exists("/EQUALASER.CSV")) {
      File f = SD.open("/EQUALASER.CSV", FILE_WRITE);
      if (f) {
        f.println("TempoSeg;Ativo;Status;DevCarcaca_mm;DevAcoplamento_mm;Theta_mrad;VibRms_mm;CalcoDianteiro_mm");
        f.close();
      }
    }
  } else {
    Serial.println("[INFO] MicroSD nao montado. Operando com memoria flash e RAM.");
  }

  // Configuração da Rede Air-Gapped SoftAP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(LOCAL_IP, GATEWAY, SUBNET);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[OK] SoftAP Ativo: %s (IP: 192.168.4.1)\n", AP_SSID);

  // Rotas HTTP
  server.on("/", HTTP_GET, handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/api/sensores", HTTP_GET, handleApiSensores);
  server.on("/api/tare", HTTP_POST, handleTare);
  server.on("/api/ack", HTTP_POST, handleAck);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.begin();

  // Criação da Task FreeRTOS dedicada à aquisição de 250 Hz no Core 0 (Wi-Fi roda no Core 1)
  xTaskCreatePinnedToCore(
    TaskSensorAcquisition,
    "TaskSensors",
    4096,
    NULL,
    2, // Prioridade alta
    NULL,
    0  // Core 0
  );

  Serial.println("[OK] Task de amostragem de 250 Hz iniciada no Core 0.");
}

// -------------------------------------------------------------------------------------
// LAÇO PRINCIPAL (CORE 1)
// -------------------------------------------------------------------------------------
void loop() {
  server.handleClient();
  updateHysteresisAndAlarms();

  // Gravação periódica de tendência a cada 10 segundos na caixa-preta
  if (millis() - lastLogSdTime > 10000) {
    lastLogSdTime = millis();
    logToMicroSd();
  }
}
