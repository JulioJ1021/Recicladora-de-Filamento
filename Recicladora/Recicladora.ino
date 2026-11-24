
#include <Arduino.h>
#include <math.h>

// ============================================================
// MODULOS
// ============================================================

#include "src/Anti_Atasco/Anti_Atasco.h"
#include "src/Control_Motores/Control_Motores.h"
#include "src/Post_Extrusion/Post_Extrusion.h"
#include "src/Stepper/Stepper.h"
#include "src/Thermic_coils/Thermic_coils.h"

// ============================================================
// CONFIGURACION DEL PROCESO
// ============================================================

// Temperaturas
#define SP_ZONA1 230.0f
#define SP_ZONA2 210.0f

#define TOLERANCIA_TEMP 2.0f

// Tiempo de estabilidad: 5 minutos
#define TIEMPO_ESTABILIDAD 300000UL

// Extrusora
#define SP_EXTRUSOR 40.0f

// Deteccion de filamento
#define UMBRAL_DETECCION_FILAMENTO 1.0f

// Monitor Serial
#define SERIAL_BAUD 115200

// ============================================================
// ESTADOS DE LA MAQUINA
// ============================================================

enum EstadoMaquina {

    ESPERA,
    CALENTANDO,
    ESTABILIZANDO,
    EXTRUYENDO,
    DETECTANDO_FILAMENTO,
    PRODUCIENDO_FILAMENTO,
    DETENIDA
};

EstadoMaquina estadoActual = ESPERA;

// ============================================================
// VARIABLES DE CONTROL
// ============================================================

bool antiAtascoInicializado = false;
bool diametroTimerInicializado = false;

bool bobinando = false;

unsigned long tiempoEstabilidad = 0;

unsigned long tiempoMonitor = 0;

const unsigned long INTERVALO_MONITOR = 1000;

// Buffer Serial
char comandoBuffer[32];
uint8_t indiceComando = 0;

// ============================================================
// FUNCIONES AUXILIARES
// ============================================================

bool temperaturaEstable() {

    float t1 = SSR1.tempEMA;
    float t2 = SSR2.tempEMA;

    if (!isfinite(t1) || !isfinite(t2)) {
        return false;
    }

    bool zona1 =
        (t1 >= SP_ZONA1 - TOLERANCIA_TEMP) &&
        (t1 <= SP_ZONA1 + TOLERANCIA_TEMP);

    bool zona2 =
        (t2 >= SP_ZONA2 - TOLERANCIA_TEMP) &&
        (t2 <= SP_ZONA2 + TOLERANCIA_TEMP);

    return zona1 && zona2;
}

// ------------------------------------------------------------

void imprimirEstado() {

    Serial.println("\n========== RECICLADORA ==========");

    Serial.print("Estado: ");

    switch (estadoActual) {

        case ESPERA:
            Serial.println("ESPERA");
            break;

        case CALENTANDO:
            Serial.println("CALENTANDO");
            break;

        case ESTABILIZANDO:
            Serial.println("ESTABILIZANDO");
            break;

        case EXTRUYENDO:
            Serial.println("EXTRUYENDO");
            break;

        case DETECTANDO_FILAMENTO:
            Serial.println("DETECTANDO FILAMENTO");
            break;

        case PRODUCIENDO_FILAMENTO:
            Serial.println("PRODUCIENDO FILAMENTO");
            break;

        case DETENIDA:
            Serial.println("DETENIDA");
            break;
    }

    Serial.println("--------------------------------");

    Serial.printf(
        "Temperatura Z1: %.2f C\n",
        SSR1.tempEMA
    );

    Serial.printf(
        "Temperatura Z2: %.2f C\n",
        SSR2.tempEMA
    );

    Serial.printf(
        "RPM Extrusora: %.2f\n",
        getRPM_Extrusor()
    );

    Serial.printf(
        "RPM Puller: %.2f\n",
        getRPM_Puller()
    );

    Serial.printf(
        "Diametro: %.2f\n",
        getDiameter()
    );

    Serial.printf(
        "Enfriamiento: %s\n",
        getCoolingState() ? "ACTIVO" : "INACTIVO"
    );

    Serial.printf(
        "Embobinado: %s\n",
        bobinando ? "ACTIVO" : "INACTIVO"
    );

    Serial.println("================================\n");
}

// ============================================================
// INICIALIZACION DE EXTRUSION
// ============================================================

void iniciarExtrusion() {

    Serial.println("\nIniciando extrusora...");

    // Setpoint de extrusora
    setSP_Extrusor(SP_EXTRUSOR);

    // Inicializar anti-atasco una sola vez.
    // Su Setup activa automaticamente la cribadora.

    if (!antiAtascoInicializado) {

        Anti_Atasco_Setup();

        antiAtascoInicializado = true;

    } else {

        activarMotores();
    }

    // Iniciar lectura del sensor de diametro
    if (!diametroTimerInicializado) {

        Diam_timer_Setup();

        diametroTimerInicializado = true;
    }

    estadoActual = EXTRUYENDO;

    Serial.println("Extrusion y cribadora activadas.");
}

// ============================================================
// CONTROL DE EMBONINADO
// ============================================================

void iniciarBobinado() {

    if (bobinando) {
        Serial.println("El embobinador ya esta activo.");
        return;
    }

    if (estadoActual != PRODUCIENDO_FILAMENTO) {

        Serial.println(
            "No se puede bobinar: no hay filamento detectado."
        );

        return;
    }

    if (getDiameter() <= UMBRAL_DETECCION_FILAMENTO) {

        Serial.println("Lectura de diametro insuficiente.");
        return;
    }

    setEmbobinando(true);

    Stepper_ON();

    bobinando = true;

    Serial.println("EMBOBINADO ACTIVADO");
}

// ------------------------------------------------------------

void detenerBobinado() {

    setEmbobinando(false);

    Stepper_OFF();

    bobinando = false;

    Serial.println("EMBOBINADO DETENIDO");
}

// ============================================================
// START
// ============================================================

void iniciarProceso() {

    if (estadoActual != ESPERA &&
        estadoActual != DETENIDA) {

        Serial.println("La maquina ya esta trabajando.");
        return;
    }

    Serial.println("\n================================");
    Serial.println("INICIANDO RECICLADORA");
    Serial.println("================================");

    detenerBobinado();

    // Setpoints termicos
    setSP(SSR1, SP_ZONA1);
    setSP(SSR2, SP_ZONA2);

    // Asegurar motores de proceso detenidos
    setSP_Extrusor(0.0f);
    setSP_Puller(0.0f);

    if (antiAtascoInicializado) {
        apagarMotores();
    }

    tiempoEstabilidad = 0;

    estadoActual = CALENTANDO;

    Serial.println("Calentamiento iniciado.");
    Serial.println("Zona 1: 230 C");
    Serial.println("Zona 2: 210 C");
}

// ============================================================
// STOP
// ============================================================

void detenerProceso() {

    Serial.println("\n!!! STOP !!!");

    // Detener embobinado
    detenerBobinado();

    // Detener extrusora y puller
    setSP_Extrusor(0.0f);
    setSP_Puller(0.0f);

    // Detener cribadora
    if (antiAtascoInicializado) {
        apagarMotores();
    }

    // Apagar calentamiento
    setSP(SSR1, 0.0f);
    setSP(SSR2, 0.0f);

    tiempoEstabilidad = 0;

    estadoActual = DETENIDA;

    Serial.println("Setpoints en cero.");
    Serial.println("Proceso detenido.");
}

// ============================================================
// MAQUINA DE ESTADOS
// ============================================================

void actualizarProceso() {

    switch (estadoActual) {

        // ----------------------------------------------------
        // ESPERA
        // ----------------------------------------------------

        case ESPERA:
        case DETENIDA:
            break;

        // ----------------------------------------------------
        // CALENTAMIENTO
        // ----------------------------------------------------

        case CALENTANDO:

            if (temperaturaEstable()) {

                tiempoEstabilidad = millis();

                estadoActual = ESTABILIZANDO;

                Serial.println(
                    "Temperaturas alcanzadas. "
                    "Iniciando estabilidad de 5 minutos."
                );
            }

            break;

        // ----------------------------------------------------
        // ESTABILIZACION
        // ----------------------------------------------------

        case ESTABILIZANDO:

            if (!temperaturaEstable()) {

                tiempoEstabilidad = 0;

                estadoActual = CALENTANDO;

                Serial.println(
                    "Temperatura fuera de rango. "
                    "Reiniciando estabilidad."
                );

                break;
            }

            if (millis() - tiempoEstabilidad >=
                TIEMPO_ESTABILIDAD) {

                Serial.println(
                    "Temperaturas estables durante 5 minutos."
                );

                iniciarExtrusion();
            }

            break;

        // ----------------------------------------------------
        // EXTRUSION
        // ----------------------------------------------------

        case EXTRUYENDO:

            estadoActual = DETECTANDO_FILAMENTO;

            Serial.println("Esperando deteccion de filamento.");

            break;

        // ----------------------------------------------------
        // DETECCION DE FILAMENTO
        // ----------------------------------------------------

        case DETECTANDO_FILAMENTO:

            if (getDiameter() >
                UMBRAL_DETECCION_FILAMENTO) {

                estadoActual = PRODUCIENDO_FILAMENTO;

                Serial.println(
                    "Filamento detectado. Produccion activa."
                );
            }

            break;

        // ----------------------------------------------------
        // PRODUCCION
        // ----------------------------------------------------

        case PRODUCIENDO_FILAMENTO:

            if (getDiameter() <=
                UMBRAL_DETECCION_FILAMENTO) {

                estadoActual = DETECTANDO_FILAMENTO;

                Serial.println(
                    "Filamento no detectado. "
                    "Esperando lectura."
                );
            }

            break;
    }
}

// ============================================================
// LECTURA SERIAL
// ============================================================

void procesarComando(char *comando) {

    // Convertir a mayusculas
    for (uint8_t i = 0; comando[i] != '\0'; i++) {

        comando[i] = toupper(comando[i]);
    }

    if (strcmp(comando, "START") == 0) {

        iniciarProceso();
    }

    else if (strcmp(comando, "STOP") == 0) {

        detenerProceso();
    }

    else if (strcmp(comando, "BOBINAR") == 0) {

        iniciarBobinado();
    }

    else if (strcmp(comando, "DESBOBINAR") == 0) {

        detenerBobinado();
    }

    else if (strcmp(comando, "ESTADO") == 0) {

        imprimirEstado();
    }

    else {

        Serial.println("Comando no reconocido.");
    }
}

// ------------------------------------------------------------

void leerSerial() {

    while (Serial.available()) {

        char caracter = Serial.read();

        if (caracter == '\n' || caracter == '\r') {

            if (indiceComando > 0) {

                comandoBuffer[indiceComando] = '\0';

                procesarComando(comandoBuffer);

                indiceComando = 0;
            }
        }

        else if (indiceComando <
                 sizeof(comandoBuffer) - 1) {

            comandoBuffer[indiceComando++] = caracter;
        }
    }
}

// ============================================================
// SETUP
// ============================================================

void setup() {

    Serial.begin(SERIAL_BAUD);

    delay(500);

    Serial.println("\n================================");
    Serial.println("RECICLADORA DE FILAMENTO");
    Serial.println("CONTROL MAESTRO");
    Serial.println("================================");

    // Control termico
    thermalSetup();

    // Control de motores y PID
    Motor_control_Setup();

    // Post extrusion
    Post_Extrusion_Setup();

    // Stepper
    Stepper_Init();

    Stepper_OFF();

    // Setpoints iniciales
    setSP(SSR1, 0.0f);
    setSP(SSR2, 0.0f);

    setSP_Extrusor(0.0f);
    setSP_Puller(0.0f);

    setEmbobinando(false);

    estadoActual = ESPERA;

    Serial.println("\nSistema inicializado.");
    Serial.println("Esperando comando START.");

    Serial.println(
        "Comandos: START, STOP, BOBINAR, "
        "DESBOBINAR, ESTADO"
    );
}

// ============================================================
// LOOP
// ============================================================

void loop() {

    leerSerial();

    actualizarProceso();

    // Monitor de variables cada segundo
    if (millis() - tiempoMonitor >= INTERVALO_MONITOR) {

        tiempoMonitor = millis();

        if (estadoActual != ESPERA &&
            estadoActual != DETENIDA) {

            imprimirEstado();
        }
    }
}
