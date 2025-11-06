#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ===== CONFIGURAÇÕES WIFI E MQTT =====
const char* ssid = "Henri Cruz";
const char* password = "17112018";
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_topic_sub = "enki/esp32/start";
const char* mqtt_topic_pub = "enki/esp32/data";

// ===== PINOS =====
#define STEP_XY 25
#define DIR_XY 26
#define STEP_ZA 27
#define DIR_ZA 14
#define SD_CS 5

// ===== SENSORES =====
Adafruit_VL53L0X vl1 = Adafruit_VL53L0X();
Adafruit_VL53L0X vl2 = Adafruit_VL53L0X();
Adafruit_VL53L0X vl3 = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;

#define XSHUT_1 15
#define XSHUT_2 2
#define XSHUT_3 4

// ===== VARIÁVEIS GLOBAIS =====
float distancia_cm = 0;
const float mm_per_step = 0.04;
int delayMotor = 500;
volatile bool emMovimento = false;
unsigned long passosTotais;
volatile unsigned long passoAtual = 0;
bool direcaoIda = true;  // NOVA VARIÁVEL PARA SABER SE É IDA OU RETORNO

bool sdIniciado = false;
String nomeArquivo = "";
int numeroMedicao = 0;

// ===== OBJETOS WIFI E MQTT =====
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ===== CALLBACK MQTT =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📨 Mensagem recebida no tópico: ");
  Serial.println(topic);
  
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Conteúdo: ");
  Serial.println(message);
  
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("❌ Erro ao fazer parse do JSON: ");
    Serial.println(error.c_str());
    return;
  }
  
  if (doc.containsKey("maxHeight")) {
    distancia_cm = doc["maxHeight"];
    Serial.print("✅ Altura recebida: ");
    Serial.print(distancia_cm);
    Serial.println(" cm");
    iniciarMedicao();
  } else {
    Serial.println("⚠️  JSON não contém 'maxHeight'");
  }
}

// ===== CONECTAR MQTT =====
void conectarMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("🔌 Conectando ao MQTT...");
    String clientId = "ESP32_" + String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" ✅ Conectado!");
      mqttClient.subscribe(mqtt_topic_sub);
      Serial.print("📥 Inscrito no tópico: ");
      Serial.println(mqtt_topic_sub);
    } else {
      Serial.print(" ❌ Falhou, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Tentando novamente em 5s...");
      delay(5000);
    }
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("    SISTEMA DE MEDIÇÃO COM MQTT + SERIAL");
  Serial.println("========================================\n");

  Wire.begin();

  pinMode(STEP_XY, OUTPUT);
  pinMode(DIR_XY, OUTPUT);
  pinMode(STEP_ZA, OUTPUT);
  pinMode(DIR_ZA, OUTPUT);
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);

  // ===== CONECTAR WIFI =====
  Serial.print("📡 Conectando ao WiFi ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi conectado!");
    Serial.print("IP local: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ Falha ao conectar ao WiFi. Verifique SSID e senha.");
    while (true) delay(1000);
  }

  // ===== TESTE TCP =====
  Serial.print("🧪 Testando conexão TCP com broker MQTT...");
  if (espClient.connect(mqtt_server, mqtt_port)) {
    Serial.println(" ✅ Sucesso!");
    espClient.stop();
  } else {
    Serial.println(" ❌ Falhou! Broker inacessível.");
  }

  // ===== CONFIGURA MQTT =====
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setSocketTimeout(10);
  conectarMQTT();

  // ===== INICIALIZA SD =====
  Serial.println("\n🔍 DIAGNÓSTICO DO CARTÃO SD");
  Serial.print("Pino CS configurado: GPIO ");
  Serial.println(SD_CS);
  Serial.println("Tentando inicializar SD Card...");

  if (!SD.begin(SD_CS)) {
    Serial.println("\n❌ FALHA AO INICIALIZAR SD CARD!");
    Serial.println("⚠️  Sistema continuará SEM gravação no SD.\n");
    sdIniciado = false;
  } else {
    Serial.println("✅ SD Card inicializado com sucesso!");
    uint8_t cardType = SD.cardType();
    Serial.print("Tipo do cartão: ");
    switch(cardType) {
      case CARD_NONE: Serial.println("NENHUM"); break;
      case CARD_MMC: Serial.println("MMC"); break;
      case CARD_SD: Serial.println("SD"); break;
      case CARD_SDHC: Serial.println("SDHC"); break;
      default: Serial.println("DESCONHECIDO"); break;
    }
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.print("Capacidade: ");
    Serial.print(cardSize);
    Serial.println(" MB");
    sdIniciado = true;
    criarArquivoLog();
  }

  // ===== INICIALIZA SENSORES =====
  Serial.println("\n🔧 Inicializando sensores laser...");
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(10);

  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  if (!vl1.begin(0x30)) Serial.println("❌ Falha VL53L0X #1");
  else Serial.println("✅ VL53L0X #1 OK");

  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  if (!vl2.begin(0x31)) Serial.println("❌ Falha VL53L0X #2");
  else Serial.println("✅ VL53L0X #2 OK");

  digitalWrite(XSHUT_3, HIGH);
  delay(10);
  if (!vl3.begin(0x32)) Serial.println("❌ Falha VL53L0X #3");
  else Serial.println("✅ VL53L0X #3 OK");

  Serial.println("\n🔧 Inicializando MPU6050...");
  if (!mpu.begin()) Serial.println("❌ Falha ao inicializar MPU6050");
  else Serial.println("✅ MPU6050 OK");

  Serial.println("\n========================================");
  Serial.println("✅ SISTEMA PRONTO!");
  Serial.println("Aguardando comandos via MQTT ou SERIAL...");
  Serial.println("Digite um valor (ex: 50) e pressione ENTER.");
  Serial.println("========================================\n");

  xTaskCreatePinnedToCore(leituraTask, "LeituraSensores", 4096, NULL, 1, NULL, 1);
}

// ===== LOOP =====
void loop() {
  if (!mqttClient.connected()) conectarMQTT();
  mqttClient.loop();

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      float valor = input.toFloat();
      if (valor > 0) {
        distancia_cm = valor;
        Serial.print("📏 Distância recebida via Serial: ");
        Serial.print(distancia_cm);
        Serial.println(" cm");
        iniciarMedicao();
      } else {
        Serial.print("⚠️ Entrada inválida: ");
        Serial.println(input);
        Serial.println("Digite um número positivo (cm).");
      }
    }
  }

  delay(10);
}

// ===== INICIAR MEDIÇÃO =====
void iniciarMedicao() {
  passosTotais = (distancia_cm * 10.0) / mm_per_step;
  passoAtual = 0;
  emMovimento = true;
  numeroMedicao = 0;

  Serial.print("\n➡️  Iniciando movimento de ");
  Serial.print(distancia_cm);
  Serial.print(" cm (");
  Serial.print(passosTotais);
  Serial.println(" passos)");

  if (sdIniciado) {
    Serial.print("📝 Gravando dados em: ");
    Serial.println(nomeArquivo);
  }

  direcaoIda = true;
  moverMotores(true);
  Serial.println("\n✅ Movimento concluído! Retornando...");
  delay(500);
  direcaoIda = false;
  moverMotores(false);
  Serial.println("🏁 Retorno completo!");

  emMovimento = false;

  if (sdIniciado) {
    Serial.print("💾 Total de ");
    Serial.print(numeroMedicao);
    Serial.println(" medições salvas no SD!");
  }
}

// ===== MOVER MOTORES =====
void moverMotores(bool ida) {
  digitalWrite(DIR_XY, ida ? HIGH : LOW);
  digitalWrite(DIR_ZA, ida ? LOW : HIGH);
  delayMicroseconds(1000);

  passoAtual = 0;
  for (unsigned long i = 0; i < passosTotais; i++) {
    digitalWrite(STEP_XY, HIGH);
    digitalWrite(STEP_ZA, HIGH);
    delayMicroseconds(delayMotor);
    digitalWrite(STEP_XY, LOW);
    digitalWrite(STEP_ZA, LOW);
    delayMicroseconds(delayMotor);
    passoAtual++;
  }
  digitalWrite(STEP_XY, LOW);
  digitalWrite(STEP_ZA, LOW);
}

// ===== TASK LEITURA =====
void leituraTask(void *pvParameters) {
  unsigned long lastRead = 0;
  while (true) {
    if (emMovimento && millis() - lastRead > 500) {
      lastRead = millis();
      leituraSensores();
    }
    vTaskDelay(10);
  }
}

// ===== LEITURA DOS SENSORES =====
void leituraSensores() {
  VL53L0X_RangingMeasurementData_t measure1, measure2, measure3;
  vl1.rangingTest(&measure1, false);
  vl2.rangingTest(&measure2, false);
  vl3.rangingTest(&measure3, false);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float altura_cm;
  if (direcaoIda)
    altura_cm = (passoAtual * mm_per_step) / 10.0;
  else
    altura_cm = distancia_cm - ((passoAtual * mm_per_step) / 10.0);
  if (altura_cm < 0) altura_cm = 0;

  Serial.println("\n==============================");
  Serial.println("DISTÂNCIAS (mm)");
  Serial.print("→ Esquerda: "); Serial.print(measure1.RangeMilliMeter);
  Serial.print(" | Direita: "); Serial.print(measure2.RangeMilliMeter);
  Serial.print(" | Frente: "); Serial.println(measure3.RangeMilliMeter);
  Serial.print("ALTURA: "); Serial.print(altura_cm, 2); Serial.println(" cm");
  Serial.println("\nMPU6050 (aceleração m/s²)");
  Serial.print("→ X: "); Serial.print(a.acceleration.x, 2);
  Serial.print(" | Y: "); Serial.print(a.acceleration.y, 2);
  Serial.print(" | Z: "); Serial.println(a.acceleration.z, 2);
  Serial.println("==============================");

  if (sdIniciado) {
    bool sucesso = gravarDadosSD(altura_cm, measure1.RangeMilliMeter, measure2.RangeMilliMeter,
                                 measure3.RangeMilliMeter, a.acceleration.x, a.acceleration.y, a.acceleration.z);
    if (sucesso) Serial.println("💾 SD OK");
  }

  publicarDadosMQTT(altura_cm, measure1.RangeMilliMeter, measure2.RangeMilliMeter,
                    measure3.RangeMilliMeter, a.acceleration.x, a.acceleration.y, a.acceleration.z);
}

// ===== PUBLICAR DADOS MQTT =====
void publicarDadosMQTT(float altura, int distEsq, int distDir, int distFrente,
                       float acX, float acY, float acZ) {
  StaticJsonDocument<300> doc;
  doc["alturaAtual"] = altura;
  doc["distance_left"] = distEsq / 1000.0;
  doc["distance_right"] = distDir / 1000.0;
  doc["distance_front"] = distFrente / 1000.0;

  JsonObject accel = doc.createNestedObject("accel");
  accel["x"] = acX;
  accel["y"] = acY;
  accel["z"] = acZ;

  char buffer[300];
  serializeJson(doc, buffer);

  if (mqttClient.publish(mqtt_topic_pub, buffer))
    Serial.println("📤 Dados publicados via MQTT");
  else
    Serial.println("❌ Falha ao publicar MQTT");
}

// ===== ARQUIVO SD =====
void criarArquivoLog() {
  int contador = 0;
  do {
    nomeArquivo = "/dados_" + String(contador) + ".csv";
    contador++;
  } while (SD.exists(nomeArquivo));

  Serial.print("\n📄 Criando arquivo: ");
  Serial.println(nomeArquivo);
  File arquivo = SD.open(nomeArquivo, FILE_WRITE);
  if (arquivo) {
    arquivo.println("Medicao,Tempo_ms,Altura_cm,Dist_Esq_mm,Dist_Dir_mm,Dist_Frente_mm,Acel_X,Acel_Y,Acel_Z");
    arquivo.close();
    Serial.println("✅ Arquivo criado com sucesso!");
  } else {
    Serial.println("❌ ERRO: Não foi possível criar arquivo!");
    sdIniciado = false;
  }
}

bool gravarDadosSD(float altura, int distEsq, int distDir, int distFrente,
                   float acX, float acY, float acZ) {
  File arquivo = SD.open(nomeArquivo, FILE_APPEND);
  if (!arquivo) {
    Serial.println("❌ ERRO ao abrir arquivo!");
    return false;
  }
  numeroMedicao++;
  arquivo.print(numeroMedicao);
  arquivo.print(",");
  arquivo.print(millis());
  arquivo.print(",");
  arquivo.print(altura, 2);
  arquivo.print(",");
  arquivo.print(distEsq);
  arquivo.print(",");
  arquivo.print(distDir);
  arquivo.print(",");
  arquivo.print(distFrente);
  arquivo.print(",");
  arquivo.print(acX, 2);
  arquivo.print(",");
  arquivo.print(acY, 2);
  arquivo.print(",");
  arquivo.println(acZ, 2);
  arquivo.close();
  return true;
}