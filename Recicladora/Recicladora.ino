
#include "src/Thermic_coils/Thermic_coils.h"
#include "src/Control_Motores/Control_Motores.h"
#include "src/Stepper/Stepper.h"

void setup() {

    Serial.begin(115200);

    // =====================================
    // CONTROL TERMICO
    // =====================================

    thermalSetup();

    setSP(SSR1, 220);
    setSP(SSR2, 120);

    // =====================================
    // CONTROL DE MOTORES
    // =====================================

    Motor_control_Setup();

    setSP_Puller(10);
    setSP_Extrusor(15);

    //Stepper_Init();
}

void loop() {

    // =====================================
    // MONITOREO TERMICO
    // =====================================

    imprimirEstadoSSR(SSR1, 1);
    imprimirEstadoSSR(SSR2, 2);

    // =====================================
    // MONITOREO DE MOTORES
    // =====================================

    /*Serial.printf(
        "PULLER | RPM: %.2f | CV: %.2f %%\n",
        getRPM_Puller(),
        getCV_Puller()
    );

    Serial.printf(
        "EXTRUSOR | RPM: %.2f | CV: %.2f %%\n",
        getRPM_Extrusor(),
        getCV_Extrusor()
    );

    Serial.println("--------------------------------");*/

    delay(500);
}
