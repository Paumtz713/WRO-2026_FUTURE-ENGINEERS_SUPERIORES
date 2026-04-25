/*!
 * MindPlus + MPU6050 - WRO Future Engineers / Open Round
 * Plataforma: Arduino Nano
 *
 * Qué hace (v3):
 *  - ALINEACIÓN inicial: centra el robot entre paredes antes de arrancar,
 *    compensando que la posición de partida dentro del área pueda variar.
 *  - ARRANQUE SUAVE (soft start): la primera ~0.8 s acelera progresivamente
 *    para no patinar ni girar bruscamente.
 *  - Sigue paredes con dos URM10 (control P sobre el servo).
 *  - Usa el MPU6050 para CONTAR vueltas (cada 360° = 1 vuelta).
 *  - Mide el tiempo de la 1ª y 2ª vuelta y predice el instante exacto en
 *    que completará las 3 vueltas de regreso al punto de salida.
 *  - Ajuste fino OFFSET_PARO_MS para quedar 0–2 cm después del origen.
 *  - Respaldo: si la predicción fallara, el detector de yaw a 1110°
 *    lo detiene igualmente.
 *
 * Conexiones MPU6050 (Arduino Nano):
 *   VCC -> 3.3V o 5V   GND -> GND   SDA -> A4   SCL -> A5
 *
 * Librería: MPU6050 by Electronic Cats v1.4.4
 */

#include <Wire.h>
#include <DFRobot_Servo.h>
#include <DFRobot_URM10.h>
#include <MPU6050.h>

// =================== Configuración ===================
#define ANGULO_VUELTA       360.0    // Grados por vuelta
#define VUELTAS_OBJETIVO    3        // Cuántas vueltas dar
#define ANGULO_RESPALDO     1080.0   // Yaw para corte de seguridad (1080 + 30)
#define MUESTRAS_CALIB      1000     // Muestras de calibración del giro
#define VELOCIDAD_PWM       90       // 0-100, velocidad de crucero

// --- Alineación inicial ---
#define ALINEACION_MS       800      // Duración del centrado inicial (ms)
#define TOL_ALINEACION_CM   2.0      // Si |izq-der| < esto, ya está centrado
#define VEL_ALINEACION      35       // PWM bajo mientras se alinea (0-100)

// --- Arranque suave ---
#define SOFT_START_MS       800      // Duración de la rampa de aceleración (ms)
#define VEL_INICIAL         45       // PWM de arranque (0-100)

// --- Ajuste fino del punto de paro ---
#define OFFSET_PARO_MS      50        // +ms para parar un poco más tarde
                                     // (p. ej. 150 ≈ 2 cm a velocidad de crucero)

// =================== Variables originales (Mind+) ===================
volatile float mind_n_ul_der, mind_n_ul_izq, mind_n_ERROR, mind_n_lado;

// =================== Objetos ===================
Servo         servo_8;
DFRobot_URM10 urm10;
MPU6050       mpu;

// =================== Variables del giroscopio ===================
float yaw = 0.0;
float gyroZ_offset = 0.0;
unsigned long t_anterior_us = 0;
const float GYRO_LSB_POR_DPS = 131.0;

// =================== Lógica de vueltas y parada ===================
unsigned long t_inicio = 0;
unsigned long t_vuelta[VUELTAS_OBJETIVO + 1] = {0};
int  contador_vueltas = 0;
unsigned long t_objetivo_paro = 0;
bool prediccion_lista = false;
bool detenido = false;


// ----- Calibración del giroscopio (robot inmóvil) -----
void calibrarGiroscopio() {
  Serial.println(F("Calibrando giroscopio... NO mover el robot"));
  long suma = 0;
  for (int i = 0; i < MUESTRAS_CALIB; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    suma += gz;
    delay(2);
  }
  gyroZ_offset = (float)suma / MUESTRAS_CALIB;
  Serial.print(F("Offset Z (LSB) = "));
  Serial.println(gyroZ_offset);
}

// ----- Cortar motores y centrar servo -----
void detenerMotores() {
  digitalWrite(10, LOW);
  digitalWrite(9,  LOW);
  analogWrite(11, 0);
  servo_8.angle(90);
}

// ----- Avanzar con PWM arbitrario y corrección de dirección -----
void avanzarConPWM(int pwm_0_100) {
  digitalWrite(10, HIGH);
  digitalWrite(9,  LOW);
  analogWrite(11, map(pwm_0_100, 0, 100, 0, 255));

  mind_n_ul_der = urm10.getDistanceCM(4, 5);
  mind_n_ul_izq = urm10.getDistanceCM(4, 7);
  mind_n_ERROR  = constrain(
                    map((mind_n_ul_izq - mind_n_ul_der) * 2, -100, 100, 65, 115),
                    65, 115);
  servo_8.angle(abs(mind_n_ERROR));
}

// ----- Seguimiento de paredes a velocidad de crucero -----
void seguirParedes() {
  avanzarConPWM(VELOCIDAD_PWM);
}

// ----- Alineación inicial: centrarse entre paredes SIN avanzar fuerte -----
// Durante ALINEACION_MS, el robot corrige servo a baja velocidad para
// acomodarse al carril actual. Compensa diferencias en el punto de partida.
void alinearInicio() {
  Serial.println(F("Alineando al carril..."));
  unsigned long t0 = millis();
  while (millis() - t0 < ALINEACION_MS) {
    mind_n_ul_der = urm10.getDistanceCM(4, 5);
    mind_n_ul_izq = urm10.getDistanceCM(4, 7);
    mind_n_ERROR  = constrain(
                      map((mind_n_ul_izq - mind_n_ul_der) * 2, -100, 100, 65, 115),
                      65, 115);
    servo_8.angle(abs(mind_n_ERROR));

    // Marcha lenta para que la dirección tenga efecto pero no se aleje mucho
    digitalWrite(10, HIGH);
    digitalWrite(9,  LOW);
    analogWrite(11, map(VEL_ALINEACION, 0, 100, 0, 255));

    Serial.print(F("[ALIN] der ")); Serial.print(mind_n_ul_der);
    Serial.print(F(" izq "));       Serial.print(mind_n_ul_izq);
    Serial.print(F(" err "));       Serial.println(mind_n_ERROR);

    // Salida temprana si ya está prácticamente centrado
    if (fabs(mind_n_ul_izq - mind_n_ul_der) < TOL_ALINEACION_CM) {
      Serial.println(F("[ALIN] Centrado OK, saliendo antes."));
      break;
    }
    delay(20);
  }
}

// ----- Arranque suave: rampa lineal de VEL_INICIAL -> VELOCIDAD_PWM -----
void arranqueSuave() {
  Serial.println(F("Arranque suave..."));
  unsigned long t0 = millis();
  while (millis() - t0 < SOFT_START_MS) {
    float frac = (float)(millis() - t0) / (float)SOFT_START_MS;   // 0 -> 1
    int pwm = VEL_INICIAL + (int)((VELOCIDAD_PWM - VEL_INICIAL) * frac);

    // Mientras acelera, también va integrando yaw para no perder cuenta
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    unsigned long t_ahora_us = micros();
    float dt = (t_ahora_us - t_anterior_us) / 1000000.0;
    t_anterior_us = t_ahora_us;
    float gyroZ_dps = ((float)gz - gyroZ_offset) / GYRO_LSB_POR_DPS;
    yaw += gyroZ_dps * dt;

    avanzarConPWM(pwm);
    delay(10);
  }
}

// =================== Setup ===================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(10, OUTPUT);
  pinMode(9,  OUTPUT);
  pinMode(11, OUTPUT);

  servo_8.attach(8);
  servo_8.angle(90);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("ERROR: MPU6050 no responde. Revisar SDA/SCL/Vcc."));
    while (true) { delay(500); }
  }
  Serial.println(F("MPU6050 conectado."));
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  delay(1500);
  calibrarGiroscopio();

  // ---- Secuencia de arranque ----
  alinearInicio();             // 1) Centrarse en el carril

  Serial.println(F("Iniciando recorrido..."));
  t_inicio = millis();
  t_vuelta[0] = t_inicio;
  t_anterior_us = micros();

  arranqueSuave();              // 2) Acelerar progresivamente
  // 3) A partir de aquí, loop() ya corre a velocidad de crucero
}

// =================== Loop ===================
void loop() {
  // ----- Si ya cortamos motores, mantenerse detenido -----
  if (detenido) {
    detenerMotores();
    return;
  }

  // ----- 1) Leer giro y actualizar yaw -----
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  unsigned long t_ahora_us = micros();
  float dt = (t_ahora_us - t_anterior_us) / 1000000.0;
  t_anterior_us = t_ahora_us;

  float gyroZ_dps = ((float)gz - gyroZ_offset) / GYRO_LSB_POR_DPS;
  yaw += gyroZ_dps * dt;

  float yaw_abs = fabs(yaw);

  // ----- 2) Contador de vueltas (cada 360°) -----
  int vueltas_actual = (int)(yaw_abs / ANGULO_VUELTA);
  if (vueltas_actual > contador_vueltas && vueltas_actual <= VUELTAS_OBJETIVO) {
    contador_vueltas = vueltas_actual;
    t_vuelta[contador_vueltas] = millis();

    unsigned long ms_acumulados = t_vuelta[contador_vueltas] - t_inicio;
    Serial.print(F(">>> Vuelta "));
    Serial.print(contador_vueltas);
    Serial.print(F(" completada en "));
    Serial.print(ms_acumulados / 1000.0, 2);
    Serial.println(F(" s"));

    // ----- 3) Predicción del instante de paro -----
    if (contador_vueltas == 1) {
      unsigned long T_vuelta1 = t_vuelta[1] - t_vuelta[0];
      t_objetivo_paro = t_vuelta[1] + 2UL * T_vuelta1 + (unsigned long)OFFSET_PARO_MS;
      prediccion_lista = true;

      Serial.print(F("    T_vuelta1 = "));
      Serial.print(T_vuelta1 / 1000.0, 2);
      Serial.print(F(" s | paro estimado en t = "));
      Serial.print((t_objetivo_paro - t_inicio) / 1000.0, 2);
      Serial.println(F(" s"));
    }
    else if (contador_vueltas == 2) {
      unsigned long T_vuelta2 = t_vuelta[2] - t_vuelta[1];
      t_objetivo_paro = t_vuelta[2] + T_vuelta2 + (unsigned long)OFFSET_PARO_MS;

      Serial.print(F("    T_vuelta2 = "));
      Serial.print(T_vuelta2 / 1000.0, 2);
      Serial.print(F(" s | paro REFINADO en t = "));
      Serial.print((t_objetivo_paro - t_inicio) / 1000.0, 2);
      Serial.println(F(" s"));
    }
  }

  // ----- 4) ¿Llegó el instante de paro? -----
  if (prediccion_lista && millis() >= t_objetivo_paro) {
    detenido = true;
    detenerMotores();
    Serial.print(F(">>> DETENIDO POR TIEMPO en punto de salida. Yaw = "));
    Serial.print(yaw, 2);
    Serial.print(F(" | t total = "));
    Serial.print((millis() - t_inicio) / 1000.0, 2);
    Serial.println(F(" s"));
    return;
  }

  // ----- 5) Respaldo por giro -----
  if (yaw_abs >= ANGULO_RESPALDO) {
    detenido = true;
    detenerMotores();
    Serial.print(F(">>> DETENIDO POR GIRO (respaldo). Yaw = "));
    Serial.println(yaw, 2);
    return;
  }

  // ----- 6) Seguimiento de paredes a velocidad de crucero -----
  seguirParedes();

  // ----- 7) Telemetría -----
  Serial.print(F("der "));  Serial.print(mind_n_ul_der);
  Serial.print(F(" izq ")); Serial.print(mind_n_ul_izq);
  Serial.print(F(" err ")); Serial.print(mind_n_ERROR);
  Serial.print(F(" yaw ")); Serial.print(yaw, 1);
  Serial.print(F(" v "));   Serial.print(contador_vueltas);
  if (prediccion_lista) {
    long restante = (long)t_objetivo_paro - (long)millis();
    Serial.print(F(" paro_en_ms ")); Serial.print(restante);
  }
  Serial.println();
}
