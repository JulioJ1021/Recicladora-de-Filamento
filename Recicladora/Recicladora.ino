
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
#include "src/HMI_WiFi/HMI_WiFi.h"

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

// HMI (pantalla web)
#define INTERVALO_HMI 100          // ms entre lecturas publicadas a la HMI

// Diametro del rodillo del puller en mm, para estimar los metros de
// filamento enrollados (spool_m). 0 = no calcular (la HMI muestra 0 m).
#define PULLER_DIAM_MM 0.0f

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

unsigned long tiempoHMI = 0;
float spoolMetros = 0.0f;

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
// HMI WIFI (se ejecuta en el Core 1; el WiFi corre en el Core 0)
// ============================================================

bool maquinaEnMarcha() {

    return estadoActual != ESPERA &&
           estadoActual != DETENIDA;
}

// ------------------------------------------------------------

void publicarDatosHMI() {

    unsigned long ahora = millis();

    // Metros enrollados: velocidad del puller x tiempo (solo si se
    // definio PULLER_DIAM_MM y el embobinado esta activo)
    static unsigned long ultimo = 0;
    float dt = (ahora - ultimo) / 1000.0f;
    ultimo = ahora;

    if (PULLER_DIAM_MM > 0.0f && bobinando) {

        spoolMetros +=
            getRPM_Puller() * 3.14159265f *
            (PULLER_DIAM_MM / 1000.0f) * dt / 60.0f;
    }

    HMIDatos d = {};

    d.mica1 = SSR1.tempEMA;
    d.mica2 = SSR2.tempEMA;

    d.rpm_ext    = getRPM_Extrusor();
    d.rpm_puller = getRPM_Puller();

    d.diametro = getDiameter();
    d.spool_m  = spoolMetros;

    d.running = maquinaEnMarcha();

    // No hay pulsador de emergencia leido por el codigo
    d.estop = false;

    // Etapas (solo indicador en la HMI)
    d.tritu = (estadoActual == EXTRUYENDO ||
               estadoActual == DETECTANDO_FILAMENTO ||
               estadoActual == PRODUCIENDO_FILAMENTO);
    d.extru = d.rpm_ext > 0.5f;
    d.enfri = getCoolingState();
    d.embo  = bobinando;

    HMI_publicarDatos(d);
}

// ------------------------------------------------------------

void procesarComandosHMI() {

    HMIComando cmd;
    uint8_t cliente;

    while (HMI_leerComando(cmd, cliente)) {

        if (cmd == HMI_START) {

            if (maquinaEnMarcha()) {

                HMI_responder(cliente, "start", false, "ya_en_marcha");

            } else {

                iniciarProceso();
                HMI_responder(cliente, "start", true, "ok");
            }
        }

        else if (cmd == HMI_STOP) {

            detenerProceso();
            HMI_responder(cliente, "stop", true, "ok");
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

    // Comunicacion con la HMI (WiFi + WebSocket en el Core 0)
    HMI_Setup();

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

    // Comandos Start / Stop recibidos de la HMI
    procesarComandosHMI();

    // Publicar variables para la HMI
    if (millis() - tiempoHMI >= INTERVALO_HMI) {

        tiempoHMI = millis();

        publicarDatosHMI();
    }

    // Monitor de variables cada segundo
    if (millis() - tiempoMonitor >= INTERVALO_MONITOR) {

        tiempoMonitor = millis();

        if (estadoActual != ESPERA &&
            estadoActual != DETENIDA) {

            imprimirEstado();
        }
    }
}
