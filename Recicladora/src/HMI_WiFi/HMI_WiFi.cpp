#include "HMI_WiFi.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>   // libreria "WebSockets" de Markus Sattler
#include <ArduinoJson.h>        // ArduinoJson v7
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ============================================================
// CONFIGURACION INTERNA
// ============================================================

#define WIFI_TASK_STACK        8192
#define WIFI_TASK_PRIORITY     1
#define WIFI_TASK_CORE         0

#define INTERVALO_ESTADO       500UL     // ms entre mensajes "state" (latido)
#define INTERVALO_RECONEXION   5000UL    // ms entre intentos de WiFi

// ============================================================
// VARIABLES
// ============================================================

struct MsgComando {
    HMIComando cmd;
    uint8_t    cliente;
};

struct MsgAck {
    uint8_t cliente;
    char    cmd[12];
    char    reason[32];
    bool    ok;
};

static WebSocketsServer webSocket(HMI_WS_PORT);

static bool servidorIniciado = false;

static QueueHandle_t colaComandos = NULL;   // Core 0 -> Core 1
static QueueHandle_t colaAck      = NULL;   // Core 1 -> Core 0

static HMIDatos datosHMI = {};
static portMUX_TYPE datosMux = portMUX_INITIALIZER_UNLOCKED;

static unsigned long tiempoEstado     = 0;
static unsigned long tiempoReconexion = 0;

// ============================================================
// UTILIDADES (Core 0)
// ============================================================

// Redondea a 1 decimal para un JSON corto. NaN se mantiene (sale "null").
static float r1(float v) {
    return isfinite(v) ? roundf(v * 10.0f) / 10.0f : v;
}

static void enviarAck(const MsgAck &m) {

    if (!servidorIniciado) return;

    JsonDocument doc;
    doc["type"]   = "ack";
    doc["cmd"]    = m.cmd;
    doc["ok"]     = m.ok ? 1 : 0;
    doc["reason"] = m.reason;

    String out;
    serializeJson(doc, out);

    webSocket.sendTXT(m.cliente, out);
}

static void ackDirecto(uint8_t cliente, const char *cmd, bool ok, const char *reason) {

    MsgAck m = {};
    m.cliente = cliente;
    m.ok = ok;
    strncpy(m.cmd, cmd, sizeof(m.cmd) - 1);
    strncpy(m.reason, reason, sizeof(m.reason) - 1);

    enviarAck(m);
}

// ============================================================
// RECIBIR COMANDOS (Core 0)
// ============================================================

static void procesarMensaje(uint8_t num, uint8_t *payload, size_t length) {

    JsonDocument doc;

    if (deserializeJson(doc, payload, length)) {
        ackDirecto(num, "unknown", false, "json_invalido");
        return;
    }

    const char *cmd = doc["cmd"] | "";

    MsgComando msg = {};
    msg.cliente = num;

    if      (strcmp(cmd, "start") == 0) msg.cmd = HMI_START;
    else if (strcmp(cmd, "stop")  == 0) msg.cmd = HMI_STOP;
    else if (strcmp(cmd, "bobinar") == 0) msg.cmd = HMI_BOBINAR;
    else {
        ackDirecto(num, "unknown", false, "comando_desconocido");
        return;
    }

    if (xQueueSend(colaComandos, &msg, 0) != pdPASS) {
        ackDirecto(num, cmd, false, "cola_llena");
    }
}

static void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {

    switch (type) {

        case WStype_CONNECTED:
            Serial.printf("HMI conectada (cliente %u)\n", num);
            break;

        case WStype_DISCONNECTED:
            Serial.printf("HMI desconectada (cliente %u)\n", num);
            break;

        case WStype_TEXT:
            procesarMensaje(num, payload, length);
            break;

        default:
            break;
    }
}

// ============================================================
// ENVIAR ESTADO (Core 0) - es tambien el "latido" de la HMI
// ============================================================

static void enviarEstado() {

    if (!servidorIniciado) return;
    if (webSocket.connectedClients() == 0) return;

    HMIDatos d;

    portENTER_CRITICAL(&datosMux);
    d = datosHMI;
    portEXIT_CRITICAL(&datosMux);

    JsonDocument doc;

    doc["type"] = "state";

    JsonArray micas = doc["micas"].to<JsonArray>();
    micas.add(r1(d.mica1));
    micas.add(r1(d.mica2));

    doc["rpm_ext"]    = r1(d.rpm_ext);
    doc["rpm_puller"] = r1(d.rpm_puller);
    doc["dia"]        = isfinite(d.diametro) ? roundf(d.diametro * 100.0f) / 100.0f : d.diametro;
    doc["spool_m"]    = r1(d.spool_m);

    doc["running"] = d.running ? 1 : 0;
    doc["estop"]   = d.estop   ? 1 : 0;

    JsonObject st = doc["stages"].to<JsonObject>();
    st["tritu"] = d.tritu ? 1 : 0;
    st["extru"] = d.extru ? 1 : 0;
    st["enfri"] = d.enfri ? 1 : 0;
    st["embo"]  = d.embo  ? 1 : 0;

    String out;
    serializeJson(doc, out);

    webSocket.broadcastTXT(out);
}

// ============================================================
// WIFI + SERVIDOR (Core 0)
// ============================================================

static void conectarWiFi() {

    Serial.println("Conectando a WiFi...");

    WiFi.begin(HMI_WIFI_SSID, HMI_WIFI_PASSWORD);

    tiempoReconexion = millis();
}

static void iniciarServidor() {

    if (MDNS.begin(HMI_MDNS_NAME)) {
        MDNS.addService("ws", "tcp", HMI_WS_PORT);
    } else {
        Serial.println("No se pudo iniciar mDNS");
    }

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    webSocket.enableHeartbeat(15000, 3000, 2);

    servidorIniciado = true;

    Serial.println("\nServidor WebSocket para la HMI iniciado");
    Serial.print("IP ESP32: ");
    Serial.println(WiFi.localIP());
    Serial.printf("HMI:  ws://%s.local:%d   (o ws://%s:%d)\n",
                  HMI_MDNS_NAME, HMI_WS_PORT,
                  WiFi.localIP().toString().c_str(), HMI_WS_PORT);
}

static void wifiLoop() {

    if (WiFi.status() != WL_CONNECTED) {

        if (millis() - tiempoReconexion >= INTERVALO_RECONEXION) {
            conectarWiFi();
        }

    } else if (!servidorIniciado) {

        iniciarServidor();
    }

    if (servidorIniciado) {

        webSocket.loop();

        // Respuestas (ACK) generadas por el Core 1
        MsgAck ack;
        while (xQueueReceive(colaAck, &ack, 0) == pdPASS) {
            enviarAck(ack);
        }
    }

    if (millis() - tiempoEstado >= INTERVALO_ESTADO) {
        tiempoEstado = millis();
        enviarEstado();
    }
}

static void wifiTask(void *parameter) {

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    conectarWiFi();

    for (;;) {
        wifiLoop();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ============================================================
// API PUBLICA
// ============================================================

void HMI_Setup() {

    colaComandos = xQueueCreate(8, sizeof(MsgComando));
    colaAck      = xQueueCreate(8, sizeof(MsgAck));

    if (colaComandos == NULL || colaAck == NULL) {
        Serial.println("ERROR HMI: no se pudieron crear las colas");
        return;
    }

    if (xTaskCreatePinnedToCore(
            wifiTask, "HMI_WiFi", WIFI_TASK_STACK, NULL,
            WIFI_TASK_PRIORITY, NULL, WIFI_TASK_CORE) != pdPASS) {

        Serial.println("ERROR HMI: no se pudo crear la tarea WiFi");
    }
}

void HMI_publicarDatos(const HMIDatos &datos) {

    portENTER_CRITICAL(&datosMux);
    datosHMI = datos;
    portEXIT_CRITICAL(&datosMux);
}

bool HMI_leerComando(HMIComando &cmd, uint8_t &cliente) {

    if (colaComandos == NULL) return false;

    MsgComando msg;

    if (xQueueReceive(colaComandos, &msg, 0) != pdPASS) return false;

    cmd = msg.cmd;
    cliente = msg.cliente;
    return true;
}

void HMI_responder(uint8_t cliente, const char *cmd, bool ok, const char *reason) {

    if (colaAck == NULL) return;

    MsgAck m = {};
    m.cliente = cliente;
    m.ok = ok;
    strncpy(m.cmd, cmd, sizeof(m.cmd) - 1);
    strncpy(m.reason, reason, sizeof(m.reason) - 1);

    xQueueSend(colaAck, &m, 0);
}
