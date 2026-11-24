#ifndef HMI_WIFI_H
#define HMI_WIFI_H

#include <Arduino.h>

// ============================================================
// CONFIGURACION  (EDITAR)
// ============================================================

#define HMI_WIFI_SSID      "PIPECAJICA"
#define HMI_WIFI_PASSWORD  "dafe1961"

// Nombre en la red: la HMI abre ws://recicladora.local:81
#define HMI_MDNS_NAME      "recicladora"
#define HMI_WS_PORT        81

// ============================================================
// DATOS QUE SE ENVIAN A LA HMI
// ============================================================

struct HMIDatos {

    float mica1;
    float mica2;

    float rpm_ext;
    float rpm_puller;

    float diametro;
    float spool_m;

    bool running;
    bool estop;

    bool tritu;
    bool extru;
    bool enfri;
    bool embo;
};

enum HMIComando {
    HMI_START,
    HMI_STOP,
    HMI_BOBINAR
};

// ============================================================
// API  (el servidor corre en una tarea propia en el Core 0)
// ============================================================

// Llamar una vez en setup()
void HMI_Setup();

// Core 1: publicar la ultima lectura de las variables
void HMI_publicarDatos(const HMIDatos &datos);

// Core 1: sacar un comando recibido de la HMI (false si no hay)
bool HMI_leerComando(HMIComando &cmd, uint8_t &cliente);

// Core 1: responder el resultado de un comando a la HMI
void HMI_responder(uint8_t cliente, const char *cmd, bool ok, const char *reason);

#endif
