/*
  ESP32_Obstaculos_OpenMV_3_Vueltas
  Obstaculos para el robot de India con camara OpenMV.

  Variante independiente: conserva la navegacion y la reaccion a pilares,
  sale del estacionamiento, completa tres vueltas y se detiene.

  Es gemelo de los programas HuskyLens: conserva la misma ley de control,
  los mismos pines del robot y una sola orden de servo y motor por ciclo.
  La unica diferencia importante es la camara: OpenMV envia por UART el ID,
  la posicion y el area del pilar detectado.

  Formato compatible con color_corner_v6.py:
    ID,X,Y,AREA,ROI,COLISION,PARED_NEGRA,PARKING,X_PARKING,AREA_PARKING,
    HUECO_PARKING,X_HUECO,ANCHO_HUECO\n
    ID 3 = verde, ID 5 = rojo, ID 0 = sin pilar.
    X llega normalizada de -100 (izquierda) a 100 (derecha).

  Cableado UART2 a 3.3 V:
    OpenMV TX -> FireBeetle D2 / GPIO25 (RX)
    OpenMV RX <- FireBeetle D3 / GPIO26 (TX, opcional)
    OpenMV GND -- FireBeetle GND

  El Nano de ultrasonicos y el BNO085 siguen en I2C IO21/IO22 a 50 kHz.
  La camara usa UART a 19200 baudios y no comparte el bus I2C.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Preferences.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

// ====================================================================
//                  VARIABLES PARA CALIBRAR EL ROBOT
// ====================================================================

// Los Kp del pasillo y de la camara se ajustan aqui. Despues hay que volver
// a cargar el programa al ESP32. Ningun valor guardado en la memoria puede
// reemplazarlos. Los demas parametros conservan sus ajustes opcionales por
// monitor serie.

// ------------------------- Velocidad --------------------------------

// Arath rueda al 40 %. La V2 de India rueda al 70 %, pero esa velocidad
// esta calibrada contra sus maniobras de rescate, que aqui no existen.
// Confirma la trayectoria a 40 y sube de cinco en cinco.
constexpr uint8_t VELOCIDAD_DEFECTO = 70;

// El sentido fisico del motor de este robot sale invertido.
constexpr bool INVERTIR_DIRECCION_MOTOR = true;

// ---------------------- Direccion por pasillo -----------------------

// diferencia = (S1 + S2) - (S4 + S5), en centimetros.
// S1 y S2 miran a la izquierda, S4 y S5 a la derecha.
//
// Control P del pasillo:
//   error = izquierda - derecha
//   correccion = Kp * error
// Arath: angulo = 90 - 0.18 * error, con tope de 18 grados.
constexpr float KP_PASILLO = 4.0f;  // <--- AJUSTAR KP DE PASILLO AQUI
constexpr float TOPE_PASILLO_DEFECTO = 30.0f;

// Cambiar a 1 si el robot se pega a la pared en vez de separarse.
constexpr int8_t DIRECCION_SERVO_PASILLO = -1;

// Recorte de los laterales antes de restar. Ver la nota del encabezado.
constexpr float MAXIMA_DISTANCIA_LATERAL_CM = 150.0f;

// ----------------------- Direccion por camara -----------------------

// Valores de la copia estable de evasion de obstaculos.
// Rojo a la izquierda de la imagen -> el robot pasa por la derecha.
constexpr float OBJETIVO_X_DEFECTO = 45.0f;
constexpr float KP_CAMARA = 1.0f;  // <--- AJUSTAR KP DE CAMARA AQUI
constexpr float TOPE_CAMARA_DEFECTO = 30.0f;

// Cambiar a -1 si el robot rebasa por el lado equivocado.
constexpr int8_t DIRECCION_SERVO_CAMARA = 1;

// IDs enviados por OPENMV/color_corner.py.
constexpr uint8_t ID_ROJO = 5;
constexpr uint8_t ID_VERDE = 3;

// El firmware OpenMV ya filtra blobs pequenos. Este segundo filtro evita
// usar como objetivo una deteccion con area casi nula.
constexpr uint16_t AREA_MINIMA_PILAR_PX = 30;

// Una sola trama sin pilar no debe devolver el volante al control de pasillo.
// Se confirma la perdida visual durante dos tramas consecutivas; una nueva
// deteccion valida reemplaza inmediatamente la anterior.
constexpr uint8_t TRAMAS_PERDIDAS_PARA_SOLTAR_PILAR = 2;

// Despues de rebasar un pilar rojo o verde, el robot se detiene brevemente.
// Durante este tiempo OpenMV y los sensores siguen trabajando; si aparece el
// siguiente pilar, el servo se acomoda antes de reanudar el avance.
constexpr unsigned long PAUSA_DESPUES_OBSTACULO_MS = 500;

// --------------------------- Mision ---------------------------------

constexpr uint8_t VUELTAS_OBJETIVO = 3;
constexpr uint8_t VUELTAS_ANTES_BUSQUEDA_ROSA = 3;
constexpr bool HABILITAR_ESTACIONAMIENTO = false;
constexpr float GRADOS_POR_VUELTA = 360.0f;
constexpr uint8_t ESQUINAS_PARA_TERMINAR = VUELTAS_OBJETIVO * 4;
constexpr float GRADOS_POR_ESQUINA = 90.0f;

// Conteo robusto tomado de ESP32_Open_Optimizado: la primera curva fija el
// sentido de la ronda y cada cuarto de vuelta debe quedar confirmado durante
// varias lecturas. La histeresis impide contar dos veces por ruido del BNO085.
constexpr float GRADOS_PARA_FIJAR_SENTIDO_VUELTA = 45.0f;
constexpr float HISTERESIS_CONTEO_VUELTA_GRADOS = 3.0f;
constexpr uint8_t MUESTRAS_PARA_CONFIRMAR_ESQUINA = 3;

// Al completar la esquina 12 todavia falta recorrer el tramo desde esa
// esquina hasta la posicion de salida. Igual que en ESP32_Open_Optimizado,
// el ancho inicial del pasillo selecciona el tiempo de avance final.
constexpr uint8_t MUESTRAS_PARA_CLASIFICAR_PASILLO = 10;
constexpr uint16_t UMBRAL_PASILLO_ANCHO_MM = 1000;
constexpr unsigned long DURACION_AVANCE_FINAL_PASILLO_ANCHO_MS = 250;
constexpr unsigned long DURACION_AVANCE_FINAL_PASILLO_ANGOSTO_MS = 2500;

// El encoder aprende la distancia real: salida -> primera esquina 4 y las
// dos vueltas completas siguientes. Con esos datos calcula cuantos pulsos
// faltan desde la esquina 12 hasta el punto exacto de salida.
constexpr uint8_t DIFERENCIA_MAXIMA_VUELTAS_ENCODER_PORCENTAJE = 35;
constexpr unsigned long MAXIMO_AVANCE_FINAL_ENCODER_MS = 6000;
constexpr unsigned long FRENO_FINAL_TRES_VUELTAS_MS = 250;

// ------------------ Salida inicial del estacionamiento -------------
// S3 mira al frente; S1 y S5 comparan el espacio a izquierda y derecha.
// El monitor serie muestra mm/10 sin decimales: 21..29 mm aparece como 2 cm.
// Frenar a 30 mm da margen al motor y no exige un eco exacto de 20 mm.
constexpr uint16_t FRENTE_OBJETIVO_SALIDA_MM = 30;
// Si el HC-SR04 pierde el eco despues de ver la pared a 5 cm o menos,
// tratamos esa perdida como llegada al limite cercano.
constexpr uint16_t FRENTE_ECO_CERCANO_SALIDA_MM = 50;
constexpr unsigned long PAUSA_FRENTE_SALIDA_MS = 500;  // 0.5 s
// El retroceso se confirma con S3 al alejarse 70 mm de la lectura de frenado.
constexpr uint16_t RETROCESO_SALIDA_MM = 62;
constexpr uint8_t VELOCIDAD_AVANCE_SALIDA = 18;
constexpr uint8_t VELOCIDAD_REVERSA_SALIDA = 18;
constexpr uint8_t VELOCIDAD_GIRO_SALIDA = 25;
// Tres movimientos iniciales de 2 s: giro, reversa recta y segundo giro.
constexpr unsigned long TIEMPO_GIRO_SALIDA_MS = 1000;
constexpr unsigned long TIEMPO_REVERSA_CENTRO_INTERMEDIA_MS = 1400;
constexpr unsigned long TIEMPO_SEGUNDO_GIRO_SALIDA_MS = 2500;
constexpr unsigned long PAUSA_SERVO_CENTRO_SALIDA_MS = 200;
constexpr unsigned long MAXIMO_RECUPERAR_RUMBO_SALIDA_MS = 1000;
constexpr unsigned long MAXIMA_EDAD_RUMBO_SALIDA_MS = 500;
constexpr unsigned long PAUSA_SERVO_GIRADO_MS = 200;
constexpr unsigned long MAXIMO_AVANCE_SALIDA_MS = 12000;
constexpr unsigned long MAXIMO_REVERSA_SALIDA_MS = 5000;
constexpr unsigned long MAXIMO_ELEGIR_LADO_MS = 1000;
constexpr uint16_t FRENTE_EMERGENCIA_GIRO_MM = 30;

// Tras el segundo giro, el BNO085 confirma el rumbo original antes del coleo.
constexpr float TOLERANCIA_RUMBO_SALIDA_GRADOS = 5.0f;

// Despues de enderezar el rumbo, el robot hace un coleo para terminar de
// salir del cajon: avanza girando al lado contrario, retrocede girando de
// vuelta al lado original, y termina retrocediendo con las ruedas centradas.
constexpr unsigned long DURACION_AVANCE_CONTRARIO_SALIDA_MS = 2000;  // 2 s
constexpr unsigned long DURACION_RETROCESO_GIRO_SALIDA_MS = 2000;    // 2 s
constexpr unsigned long DURACION_RETROCESO_CENTRO_SALIDA_MS = 3000;  // 3 s


// Las tres primeras vueltas conservan exactamente el control normal. Al
// comenzar la cuarta no se frena: se activa la busqueda rosa y el robot sigue
// avanzando. Si no encuentra el estacionamiento, se detiene al completar la 4.

// ------------------ Final y busqueda de estacionamiento ------------

// Convencion del BNO085. Cambiar a -1 solamente si en telemetria los giros
// a la izquierda hacen disminuir giroAcumuladoGrados.
constexpr int8_t SIGNO_BNO_GIRO_IZQUIERDA = 1;

// En este robot un angulo mayor de 90 dirige las ruedas a la derecha.
// Cambiar a -1 si fisicamente ocurre lo contrario.
constexpr int8_t SIGNO_SERVO_DERECHA = 1;

// La cuarta vuelta conserva la velocidad y la conduccion normales hasta que
// la camara confirma el rosa.
constexpr uint8_t TRAMAS_PARED_ESTACIONAMIENTO = 3;
constexpr uint8_t TRAMAS_HUECO_ESTACIONAMIENTO = 2;
constexpr unsigned long INTERVALO_MODO_ESTACIONAMIENTO_MS = 250;
constexpr unsigned long MAXIMO_BUSQUEDA_ESTACIONAMIENTO_MS = 45000;
constexpr int16_t ZONA_CENTRAL_LADO_PARKING_X = 12;

// Despues de confirmar el rosa, la camara deja de dirigir por color y estos
// valores mantienen al robot a 8..10 cm mientras alinea las llantas.
constexpr uint16_t DISTANCIA_EXTERIOR_MINIMA_MM = 80;
constexpr uint16_t DISTANCIA_EXTERIOR_OBJETIVO_MM = 90;
constexpr uint16_t DISTANCIA_EXTERIOR_MAXIMA_MM = 100;
constexpr uint16_t DISTANCIA_MAXIMA_SEGUIMIENTO_EXTERIOR_MM = 300;
constexpr float KP_PARED_EXTERIOR_GRADOS_POR_MM = 0.20f;
constexpr float TOPE_PARED_EXTERIOR_GRADOS = 22.0f;

// Deteccion del primer delimitador y del hueco con S1 o S5, segun el lado
// que haya guardado la camara. La histeresis evita alternar por ruido.
constexpr uint16_t UMBRAL_DELIMITADOR_PARKING_MM = 260;
constexpr uint16_t UMBRAL_HUECO_PARKING_MM = 320;
constexpr uint16_t SALTO_MINIMO_HUECO_PARKING_MM = 80;
constexpr unsigned long BORDE_ESTABLE_PARKING_MS = 75;

// Entrada como en el video: el lateral localiza el delimitador y su borde,
// se alinea el eje delantero y se hacen solo dos movimientos de direccion:
// arco hacia el cajon y contraarco hasta recuperar el rumbo original.
constexpr uint8_t VELOCIDAD_APROXIMACION_PARKING = 20;
constexpr uint8_t VELOCIDAD_MANIOBRA_PARKING = 18;
constexpr unsigned long AVANCE_MINIMO_ALINEAR_LLANTAS_MS = 550;
constexpr uint32_t PULSOS_MINIMOS_ALINEAR_LLANTAS = 4;
constexpr unsigned long MAXIMO_ALINEAR_LLANTAS_MS = 1800;
constexpr float ANGULO_SERVO_PARKING_GRADOS = 38.0f;
constexpr float CAMBIO_RUMBO_ENTRADA_PARKING_GRADOS = 55.0f;
constexpr float TOLERANCIA_RUMBO_FINAL_PARKING_GRADOS = 3.0f;
constexpr uint16_t MARGEN_FRONTAL_FINAL_PARKING_MM = 70;
constexpr uint16_t DISTANCIA_FRONTAL_EMERGENCIA_PARKING_MM = 45;
constexpr uint16_t DISTANCIA_LATERAL_EMERGENCIA_PARKING_MM = 45;
constexpr unsigned long MAXIMO_LOCALIZAR_DELIMITADOR_MS = 6000;
constexpr unsigned long MAXIMO_ESPERAR_HUECO_MS = 3500;
constexpr unsigned long MAXIMO_ARCO_PARKING_MS = 4500;
constexpr unsigned long AVANCE_MINIMO_DENTRO_PARKING_MS = 900;
constexpr uint32_t PULSOS_MINIMOS_DENTRO_PARKING = 4;
constexpr unsigned long MAXIMO_CENTRADO_PARKING_MS = 4500;
constexpr unsigned long FRENO_FINAL_PARKING_MS = 180;

// ------------------- Escape por obstaculo frontal -----------------

// Misma estrategia del programa Open: S3 activa una reversa con histeresis.
// Primero se corta el motor, despues se retrocede guiando el volante segun el
// sentido acumulado de la ronda y finalmente se hace otra pausa antes de
// recuperar el avance normal por camara/pasillo.
// La copia estable reaccionaba a 35 cm y recuperaba el avance a 50 cm. Se
// conservan dos lecturas consecutivas para rechazar un eco aislado.
constexpr uint8_t TRAMAS_CONFIRMAR_ESCAPE_FRONTAL = 2;
constexpr uint16_t DISTANCIA_FRENTE_RETROCESO_MM = 350;
constexpr uint16_t DISTANCIA_FRENTE_SALIDA_RETROCESO_MM = 500;
constexpr uint8_t VELOCIDAD_REVERSA_ESCAPE = 45;
constexpr unsigned long FRENO_ANTES_RETROCESO_MS = 150;
constexpr unsigned long FRENO_DESPUES_RETROCESO_MS = 150;
constexpr unsigned long MAXIMO_RETROCESO_ESCAPE_MS = 2500;
constexpr float UMBRAL_SENTIDO_RETROCESO_GRADOS = 12.0f;
constexpr float ANGULO_GUIA_RETROCESO_GRADOS = 18.0f;

// Evita quedar atrapado repitiendo la reversa frente a la misma pared. Tras
// cuatro intentos consecutivos, avanza despacio con giro firme en el sentido
// de la ronda y despues reinicia el contador de escapes.
constexpr uint8_t MAXIMO_RETROCESOS_CONSECUTIVOS = 4;
constexpr uint8_t VELOCIDAD_SALIDA_FORZADA = 25;
constexpr float ANGULO_SALIDA_FORZADA_GRADOS = 30.0f;
constexpr unsigned long DURACION_SALIDA_FORZADA_MS = 1200;
constexpr unsigned long TIEMPO_DESPEJADO_REINICIAR_RETROCESOS_MS = 1000;

// Cambiar a -1 si el robot gira hacia el lado equivocado al retroceder.
constexpr int8_t DIRECCION_SERVO_REVERSA = 1;

// COLISION ya llega desde OpenMV. Primero reduce velocidad para darle tiempo
// al control de camara; S3 sigue siendo quien autoriza la reversa.
constexpr uint8_t VELOCIDAD_ALERTA_COLISION = 40;
constexpr unsigned long RETENCION_ALERTA_COLISION_MS = 200;

// Un eco perdido durante un solo ciclo conserva la ultima lectura real. Tras
// este margen ya se declara invalida y el control general frena por seguridad.
constexpr unsigned long RETENCION_ULTIMO_ECO_VALIDO_MS = 125;

// ------------------------ Ritmo del ciclo ---------------------------

// 40 Hz. OpenMV se recibe sin bloquear por UART y el paquete del Nano tarda
// unos 2 ms, asi que el ciclo cierra con margen de sobra.
constexpr unsigned long PERIODO_CICLO_MS = 25;

// ====================================================================
//                             PINES
// ====================================================================

constexpr uint8_t PIN_SERVO = 27;  // D4 / IO27
constexpr uint8_t PIN_MOTOR_PWM = D6;
constexpr uint8_t PIN_MOTOR_AIN2 = D7;
constexpr uint8_t PIN_MOTOR_AIN1 = D8;
constexpr uint8_t PIN_ENCODER_A = D5;

constexpr int8_t PIN_OPENMV_RX = 25;  // D2, recibe desde TX de OpenMV
constexpr int8_t PIN_OPENMV_TX = 26;  // D3, envia hacia RX de OpenMV

constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;

constexpr uint8_t DIRECCION_NANO = 0x08;
constexpr uint8_t DIRECCION_BNO085_PRIMARIA = 0x4B;
constexpr uint8_t DIRECCION_BNO085_SECUNDARIA = 0x4A;

// El bus va a 50 kHz por el Nano de 5 V: sus pull-ups son de 3.3 V y el
// margen de HIGH es de apenas 0.3 V. Ver extras/README_I2C_SENSORES.md.
constexpr uint32_t FRECUENCIA_I2C_HZ = 50000;
constexpr uint16_t TIMEOUT_I2C_MS = 10;

constexpr uint32_t OPENMV_BAUD = 19200;
constexpr unsigned long OPENMV_TIMEOUT_MS = 500;
constexpr int OPENMV_X_MIN = -100;
constexpr int OPENMV_X_MAX = 100;
constexpr int OPENMV_X_TOLERANCIA = 5;
constexpr int OPENMV_ANCHO_IMAGEN_PX = 320;
constexpr int OPENMV_ALTO_IMAGEN_PX = 240;
constexpr int OPENMV_CAMPOS_MINIMOS = 6;
constexpr int OPENMV_CAMPOS_MAXIMOS = 13;
constexpr size_t OPENMV_LINEA_BYTES = 128;
constexpr size_t OPENMV_BUFFER_RX_BYTES = 512;
constexpr int OPENMV_MAXIMO_ATRASO_BYTES = 128;
constexpr uint8_t OPENMV_LINEAS_POR_CICLO = 8;
constexpr unsigned long OPENMV_AVISO_INVALIDO_MS = 1000;

// ====================================================================
//                      CONSTANTES DE HARDWARE
// ====================================================================

constexpr uint8_t CANTIDAD_SENSORES = 5;
constexpr uint8_t BYTES_PAQUETE = CANTIDAD_SENSORES * 2;
constexpr uint16_t DISTANCIA_INVALIDA_MM = 0xFFFF;
constexpr uint16_t DISTANCIA_SIN_ECO_MM = 2000;
constexpr unsigned long TIMEOUT_SENSORES_MS = 250;

constexpr uint8_t INDICE_IZQ_90 = 0;   // S1
constexpr uint8_t INDICE_IZQ_25 = 1;   // S2
constexpr uint8_t INDICE_FRONTAL = 2;  // S3
constexpr uint8_t INDICE_DER_25 = 3;   // S4
constexpr uint8_t INDICE_DER_90 = 4;   // S5

constexpr int SERVO_CENTRO_GRADOS = 90;
constexpr int SERVO_CORRECCION_MAXIMA_GRADOS = 40;
constexpr uint16_t SERVO_PULSO_MINIMO_US = 500;
constexpr uint16_t SERVO_PULSO_MAXIMO_US = 2500;
constexpr uint32_t SERVO_PWM_HZ = 50;
constexpr uint8_t SERVO_PWM_BITS = 16;
constexpr uint32_t MOTOR_PWM_HZ = 20000;
constexpr uint8_t MOTOR_PWM_BITS = 10;

// Orden fuera del rango normal -100..100. En un TB6612, AIN1=AIN2=HIGH
// con PWM al 100 % cortocircuita electricamente el motor y frena en seco.
constexpr int ORDEN_FRENO_ACTIVO = 101;

#if ESP_ARDUINO_VERSION_MAJOR < 3
constexpr uint8_t CANAL_PWM_SERVO = 0;
constexpr uint8_t CANAL_PWM_MOTOR = 1;
#endif

// El interruptor del juez corta la potencia del motor, no la del ESP32. El
// PWM se prepara desde el encendido y la ronda arranca cuando el encoder
// confirma que el robot de verdad se movio.
constexpr uint32_t PULSOS_PARA_ARRANCAR = 4;

constexpr uint32_t BNO085_INTERVALO_REPORTE_US = 10000;
constexpr unsigned long BNO085_REINTENTO_MS = 1000;
constexpr unsigned long BNO085_SIN_DATOS_MS = 2000;
constexpr float MAXIMO_SALTO_RUMBO_GRADOS = 90.0f;

constexpr uint8_t BRILLO_LEDS_PORCENTAJE = 20;
constexpr unsigned long INTERVALO_BRILLO_MS = 2000;
constexpr unsigned long INTERVALO_TELEMETRIA_MS = 500;

constexpr uint8_t SERIAL_LINEA_BYTES = 48;

// Tope de bytes atendidos por ciclo: pegar un texto largo no debe retrasar
// los sensores ni el servo.
constexpr uint8_t SERIAL_BYTES_POR_CICLO = 32;

// Espacio propio para que los ajustes de OpenMV no se mezclen con los de
// HuskyLens 1 o 2.
constexpr char NVS_ESPACIO[] = "openmv";

// Version 6 recupera la evasion de la copia estable. Migra solamente los
// objetivos predeterminados anteriores; respeta otros ajustes manuales.
constexpr uint8_t VERSION_AJUSTES_NVS = 6;
constexpr float OBJETIVO_X_VERSION_ANTERIOR_V4 = 65.0f;
constexpr float OBJETIVO_X_VERSION_ANTERIOR_V3 = 58.0f;
constexpr float OBJETIVO_X_VERSION_ANTERIOR = 50.0f;
constexpr float OBJETIVO_X_VERSION_ANTERIOR_V2 = 45.0f;
constexpr float OBJETIVO_X_VERSION_ANTERIOR_V1 = 35.0f;

// ====================================================================
//                       COMPROBACIONES EN FRIO
// ====================================================================

static_assert(VELOCIDAD_DEFECTO > 0 && VELOCIDAD_DEFECTO <= 100,
              "La velocidad va de 1 a 100");
static_assert(DIRECCION_SERVO_PASILLO == 1 || DIRECCION_SERVO_PASILLO == -1,
              "La direccion del pasillo solo puede ser 1 o -1");
static_assert(DIRECCION_SERVO_CAMARA == 1 || DIRECCION_SERVO_CAMARA == -1,
              "La direccion de la camara solo puede ser 1 o -1");
static_assert(TOPE_PASILLO_DEFECTO <= SERVO_CORRECCION_MAXIMA_GRADOS,
              "El tope del pasillo no cabe en el recorrido del servo");
static_assert(TOPE_CAMARA_DEFECTO <= SERVO_CORRECCION_MAXIMA_GRADOS,
              "El tope de la camara no cabe en el recorrido del servo");
static_assert(ID_ROJO != ID_VERDE, "Los IDs de color deben ser distintos");
static_assert(KP_PASILLO >= 0.0f, "Kp de pasillo no puede ser negativo");
static_assert(KP_CAMARA >= 0.0f, "Kp de camara no puede ser negativo");
static_assert(SIGNO_BNO_GIRO_IZQUIERDA == 1 ||
                  SIGNO_BNO_GIRO_IZQUIERDA == -1,
              "El signo de giro BNO solo puede ser 1 o -1");
static_assert(SIGNO_SERVO_DERECHA == 1 || SIGNO_SERVO_DERECHA == -1,
              "El signo del servo solo puede ser 1 o -1");
static_assert(ANGULO_SERVO_PARKING_GRADOS <=
                  SERVO_CORRECCION_MAXIMA_GRADOS,
              "El angulo de parking excede el servo");
static_assert(ESQUINAS_PARA_TERMINAR > 0, "Hacen falta esquinas que contar");
static_assert(VUELTAS_OBJETIVO > 0 && GRADOS_POR_VUELTA > 0.0f,
              "El objetivo de vueltas debe ser valido");
static_assert(GRADOS_PARA_FIJAR_SENTIDO_VUELTA > 0.0f &&
                  GRADOS_PARA_FIJAR_SENTIDO_VUELTA < GRADOS_POR_ESQUINA,
              "El umbral para fijar el sentido de vuelta no es valido");
static_assert(HISTERESIS_CONTEO_VUELTA_GRADOS > 0.0f &&
                  HISTERESIS_CONTEO_VUELTA_GRADOS < GRADOS_POR_ESQUINA,
              "La histeresis del conteo de vueltas no es valida");
static_assert(MUESTRAS_PARA_CONFIRMAR_ESQUINA > 0,
              "Se necesita confirmar cada esquina");
static_assert(MUESTRAS_PARA_CLASIFICAR_PASILLO > 0,
              "Se necesitan muestras para clasificar el pasillo");
static_assert(DURACION_AVANCE_FINAL_PASILLO_ANCHO_MS > 0 &&
                  DURACION_AVANCE_FINAL_PASILLO_ANGOSTO_MS > 0,
              "Los avances finales deben ser mayores que cero");
static_assert(DIFERENCIA_MAXIMA_VUELTAS_ENCODER_PORCENTAJE <= 100,
              "La tolerancia entre vueltas del encoder no es valida");
static_assert(MAXIMO_AVANCE_FINAL_ENCODER_MS > 0 &&
                  FRENO_FINAL_TRES_VUELTAS_MS > 0,
              "Los tiempos de llegada final deben ser mayores que cero");
static_assert(RETROCESO_SALIDA_MM > 0,
              "El retroceso de salida debe ser positivo");
static_assert(TIEMPO_GIRO_SALIDA_MS > 0 &&
                  TIEMPO_REVERSA_CENTRO_INTERMEDIA_MS > 0 &&
                  TIEMPO_SEGUNDO_GIRO_SALIDA_MS > 0 &&
                  MAXIMO_RECUPERAR_RUMBO_SALIDA_MS > 0,
              "Los tiempos de salida deben ser positivos");
static_assert(DURACION_AVANCE_CONTRARIO_SALIDA_MS > 0 &&
                  DURACION_RETROCESO_GIRO_SALIDA_MS > 0 &&
                  DURACION_RETROCESO_CENTRO_SALIDA_MS > 0,
              "Las duraciones del coleo final deben ser mayores que cero");
static_assert(VELOCIDAD_AVANCE_SALIDA > 0 && VELOCIDAD_REVERSA_SALIDA > 0 &&
                  VELOCIDAD_GIRO_SALIDA > 0,
              "Las velocidades de salida deben ser positivas");
static_assert(PAUSA_DESPUES_OBSTACULO_MS > 0,
              "La pausa despues del obstaculo debe ser mayor que cero");
static_assert(!HABILITAR_ESTACIONAMIENTO ||
                  VUELTAS_ANTES_BUSQUEDA_ROSA < VUELTAS_OBJETIVO,
              "La busqueda rosa debe comenzar antes de terminar la mision");
static_assert(UMBRAL_DELIMITADOR_PARKING_MM < UMBRAL_HUECO_PARKING_MM,
              "Los umbrales de parking necesitan histeresis");
static_assert(DISTANCIA_EXTERIOR_MINIMA_MM <
                  DISTANCIA_EXTERIOR_OBJETIVO_MM &&
                  DISTANCIA_EXTERIOR_OBJETIVO_MM <
                      DISTANCIA_EXTERIOR_MAXIMA_MM,
              "El objetivo exterior debe quedar dentro de 8..10 cm");
static_assert(DISTANCIA_FRENTE_SALIDA_RETROCESO_MM >
                  DISTANCIA_FRENTE_RETROCESO_MM,
              "La salida del escape debe superar su distancia de entrada");
static_assert(VELOCIDAD_REVERSA_ESCAPE > 0 &&
                  VELOCIDAD_REVERSA_ESCAPE <= 100,
              "La velocidad de escape debe quedar entre 1 y 100");
static_assert(MAXIMO_RETROCESOS_CONSECUTIVOS > 0,
              "Debe permitirse al menos un intento de retroceso");
static_assert(VELOCIDAD_SALIDA_FORZADA > 0 &&
                  VELOCIDAD_SALIDA_FORZADA <= 100,
              "La velocidad de salida forzada debe quedar entre 1 y 100");
static_assert(ANGULO_SALIDA_FORZADA_GRADOS > 0.0f &&
                  ANGULO_SALIDA_FORZADA_GRADOS <=
                      SERVO_CORRECCION_MAXIMA_GRADOS,
              "El angulo de salida forzada excede el servo");
static_assert(DURACION_SALIDA_FORZADA_MS > 0 &&
                  TIEMPO_DESPEJADO_REINICIAR_RETROCESOS_MS > 0,
              "Los tiempos del limite de retrocesos deben ser mayores que cero");
static_assert(DIRECCION_SERVO_REVERSA == 1 ||
                  DIRECCION_SERVO_REVERSA == -1,
              "La direccion del servo en reversa debe ser 1 o -1");
static_assert(PERIODO_CICLO_MS > 0, "El periodo del ciclo no puede ser 0");

// ====================================================================
//                             ESTADO
// ====================================================================

enum class Color : uint8_t { NINGUNO, ROJO, VERDE };

enum class Fase : uint8_t {
  SALIDA_AVANZAR,
  SALIDA_PAUSA_FRENTE,
  SALIDA_RETROCEDER,
  SALIDA_ELEGIR_LADO,
  SALIDA_ORIENTAR_SERVO,
  SALIDA_GIRAR,
  SALIDA_CENTRAR_INTERMEDIO,
  SALIDA_REVERSA_CENTRO_INTERMEDIA,
  SALIDA_ORIENTAR_SEGUNDO_GIRO,
  SALIDA_SEGUNDO_GIRO,
  SALIDA_RECUPERAR_RUMBO,
  SALIDA_AVANCE_CONTRARIO,
  SALIDA_RETROCESO_GIRO,
  SALIDA_RETROCESO_CENTRO,
  ESPERANDO,
  RODANDO,
  FRENANDO_ESCAPE_FRONTAL,
  RETROCEDIENDO_ESCAPE_FRONTAL,
  ESPERANDO_DESPEJE_ESCAPE,
  FRENO_TRAS_ESCAPE,
  SALIDA_FORZADA_ESCAPE,
  AVANCE_FINAL_TRES_VUELTAS,
  FRENANDO_FIN_TRES_VUELTAS,
  BUSCANDO_ESTACIONAMIENTO,
  LOCALIZANDO_PRIMER_DELIMITADOR,
  ESPERANDO_ENTRADA_HUECO,
  AVANCE_LIBRE_ENTRADA,
  ARCO_ENTRADA_FRONTAL,
  CONTRAARCO_ENTRADA_FRONTAL,
  CENTRANDO_EN_CAJON,
  FRENO_FINAL_ESTACIONAMIENTO,
  TERMINADO
};

struct Pilar {
  Color color = Color::NINGUNO;
  float xCamara = 0.0f;  // normalizada: -100 izquierda, +100 derecha
  uint16_t areaPx = 0;
  int16_t yPx = -1;
  uint8_t roi = 0;
  uint8_t id = 0;
};

// Lo que se calibra en la mesa. Arranca en las constantes de arriba, se
// cambia por el monitor serie y, si se guarda, sobrevive al apagon. Va
// declarado aqui, antes de cualquier funcion, porque el IDE de Arduino
// genera los prototipos al principio del archivo.
struct Ajustes {
  float topePasillo = TOPE_PASILLO_DEFECTO;
  float topeCamara = TOPE_CAMARA_DEFECTO;
  float objetivoX = OBJETIVO_X_DEFECTO;
  uint8_t velocidad = VELOCIDAD_DEFECTO;
};

Adafruit_BNO08x bno085;
sh2_SensorValue_t valorBno085;
HardwareSerial openMVSerial(2);

Fase fase = Fase::SALIDA_AVANZAR;
Fase faseDespuesEscape = Fase::RODANDO;
Pilar pilar;
Pilar pilarAlIniciarEscape;
Ajustes ajustes;
Preferences memoria;
bool telemetriaActiva = true;

uint16_t distanciasMm[CANTIDAD_SENSORES] = {
    DISTANCIA_SIN_ECO_MM, DISTANCIA_SIN_ECO_MM, DISTANCIA_SIN_ECO_MM,
    DISTANCIA_SIN_ECO_MM, DISTANCIA_SIN_ECO_MM};
bool distanciaValida[CANTIDAD_SENSORES] = {false, false, false, false, false};
uint16_t ultimaDistanciaValidaMm[CANTIDAD_SENSORES] = {
    DISTANCIA_SIN_ECO_MM, DISTANCIA_SIN_ECO_MM, DISTANCIA_SIN_ECO_MM,
    DISTANCIA_SIN_ECO_MM, DISTANCIA_SIN_ECO_MM};
unsigned long ultimoEcoValidoMs[CANTIDAD_SENSORES] = {0, 0, 0, 0, 0};
unsigned long ultimoPaqueteMs = 0;
bool hayDistancias = false;

bool servoListo = false;
bool motorListo = false;
int ultimoAnguloEscrito = -1;
int ultimaVelocidadEscrita = 101;

volatile uint32_t pulsosEncoder = 0;
volatile int32_t pulsosEncoderNetos = 0;
volatile int8_t direccionEncoderActual = 0;

bool bno085Listo = false;
bool bno085ConRumbo = false;
float rumboAnteriorGrados = 0.0f;
float rumboSalidaActualGrados = 0.0f;
unsigned long ultimoRumboSalidaValidoMs = 0;
float giroAcumuladoGrados = 0.0f;
float giroConteoVueltasGrados = 0.0f;
uint8_t esquinas = 0;
uint8_t muestrasConfirmacionEsquina = 0;
uint8_t vueltasCompletadas = 0;
unsigned long ultimoReintentoBnoMs = 0;
unsigned long ultimoCuadroBnoMs = 0;
bool pasilloInicialClasificado = false;
bool pasilloInicialAncho = false;
uint8_t muestrasPasilloInicial = 0;
uint32_t acumuladoAnchoPasilloInicialMm = 0;
uint16_t anchoPasilloInicialMm = 0;
unsigned long duracionAvanceFinalSeleccionadaMs =
    DURACION_AVANCE_FINAL_PASILLO_ANGOSTO_MS;
unsigned long avanceFinalAcumuladoMs = 0;
unsigned long ultimoAvanceFinalMs = 0;
int32_t pulsosNetosInicioRonda = 0;
int32_t pulsosNetosVuelta1 = 0;
int32_t pulsosNetosVuelta2 = 0;
int32_t pulsosNetosVuelta3 = 0;
int32_t pulsosNetosInicioAvanceFinal = 0;
int32_t pulsosObjetivoAvanceFinal = 0;
bool referenciaFinalEncoderValida = false;

bool camaraLista = false;
unsigned long ultimoPaqueteCamaraMs = 0;
unsigned long ultimoAvisoOpenMvMs = 0;
uint8_t colisionVisual = 0;
unsigned long ultimaAlertaColisionMs = 0;
bool paredNegraVisible = false;
uint8_t tramasConsecutivasSinPilar = 0;
bool pausaDespuesObstaculoActiva = false;
unsigned long inicioPausaDespuesObstaculoMs = 0;
uint32_t tramasOpenMvValidas = 0;
uint32_t tramasOpenMvInvalidas = 0;
char lineaOpenMV[OPENMV_LINEA_BYTES];
size_t largoLineaOpenMV = 0;
bool descartandoLineaOpenMV = false;

bool paredEstacionamientoVisible = false;
int16_t paredEstacionamientoX = 0;
uint16_t paredEstacionamientoArea = 0;
uint8_t confirmacionesParedEstacionamiento = 0;
bool huecoEstacionamientoVisible = false;
int16_t huecoEstacionamientoX = 0;
uint16_t anchoHuecoEstacionamientoPx = 0;
uint8_t confirmacionesHuecoEstacionamiento = 0;
int16_t referenciaParkingXGuardada = 0;
uint16_t areaParkingGuardada = 0;
int16_t huecoParkingXGuardado = 0;
uint16_t anchoHuecoParkingGuardadoPx = 0;
bool huecoParkingGuardado = false;
int8_t ladoParking = 0;  // -1 = izquierda, +1 = derecha
unsigned long ultimoEnvioModoEstacionamientoMs = 0;
bool modoEstacionamientoEnviado = false;
bool ultimoModoEstacionamiento = false;

unsigned long inicioRondaMs = 0;
unsigned long inicioFaseMs = 0;
unsigned long inicioAvanceSalidaMs = 0;
bool s3VioParedCercana = false;
uint16_t ultimaDistanciaCercanaSalidaMm = FRENTE_OBJETIVO_SALIDA_MM;
uint16_t distanciaObjetivoReversaSalidaMm =
    FRENTE_OBJETIVO_SALIDA_MM + RETROCESO_SALIDA_MM;
int8_t ladoSalidaParking = 0;  // -1 izquierda, +1 derecha
bool salidaInicialCancelada = false;
unsigned long inicioRetrocesoEscapeMs = 0;
int8_t signoGiroVueltas = 0;
float giroObjetivoFinalGrados = 0.0f;
unsigned long inicioBusquedaParkingMs = 0;
float giroBaseManiobraParkingGrados = 0.0f;
// Rumbo absoluto del BNO085 antes de girar para salir del cajon.
float rumboBaseSalidaGrados = 0.0f;
uint32_t pulsosInicioMovimientoParking = 0;
uint16_t distanciaPrimerDelimitadorMm = UMBRAL_DELIMITADOR_PARKING_MM;
unsigned long inicioCondicionParkingMs = 0;
bool condicionParkingAnterior = false;
bool estacionamientoExitoso = false;
uint8_t confirmacionesObstaculoFrontal = 0;
uint8_t retrocesosConsecutivos = 0;
unsigned long inicioSalidaForzadaEscapeMs = 0;
unsigned long inicioFrenteDespejadoMs = 0;
unsigned long ultimoCicloMs = 0;
unsigned long ultimoBrilloMs = 0;
unsigned long ultimaTelemetriaMs = 0;

char lineaSerial[SERIAL_LINEA_BYTES];
uint8_t largoLineaSerial = 0;

bool busquedaRosaActiva();

// ====================================================================
//                            UTILIDADES
// ====================================================================

float normalizarDelta(float gradosDelta) {
  while (gradosDelta > 180.0f) gradosDelta -= 360.0f;
  while (gradosDelta < -180.0f) gradosDelta += 360.0f;
  return gradosDelta;
}

float distanciaLateralCm(uint16_t distanciaMm) {
  const float cm = static_cast<float>(distanciaMm) / 10.0f;
  return cm > MAXIMA_DISTANCIA_LATERAL_CM ? MAXIMA_DISTANCIA_LATERAL_CM : cm;
}

// ====================================================================
//                          SERVO Y MOTOR
// ====================================================================

bool iniciarServo() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(PIN_SERVO, SERVO_PWM_HZ, SERVO_PWM_BITS);
#else
  ledcSetup(CANAL_PWM_SERVO, SERVO_PWM_HZ, SERVO_PWM_BITS);
  ledcAttachPin(PIN_SERVO, CANAL_PWM_SERVO);
  return true;
#endif
}

void escribirServo(int grados) {
  if (!servoListo) {
    return;
  }

  grados = constrain(grados, 0, 180);

  const uint32_t pulsoUs =
      SERVO_PULSO_MINIMO_US +
      (static_cast<uint32_t>(grados) *
       (SERVO_PULSO_MAXIMO_US - SERVO_PULSO_MINIMO_US)) /
          180UL;

  constexpr uint32_t PERIODO_US = 1000000UL / SERVO_PWM_HZ;
  constexpr uint32_t DUTY_MAXIMO = (1UL << SERVO_PWM_BITS) - 1UL;

  const uint32_t duty =
      (pulsoUs * DUTY_MAXIMO + PERIODO_US / 2UL) / PERIODO_US;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO, duty);
#else
  ledcWrite(CANAL_PWM_SERVO, duty);
#endif
}

// Unico punto del programa que mueve el servo.
void aplicarDireccion(float grados) {
  grados = constrain(
      grados,
      static_cast<float>(SERVO_CENTRO_GRADOS -
                         SERVO_CORRECCION_MAXIMA_GRADOS),
      static_cast<float>(SERVO_CENTRO_GRADOS +
                         SERVO_CORRECCION_MAXIMA_GRADOS));

  const int redondeado = static_cast<int>(lroundf(grados));
  if (redondeado != ultimoAnguloEscrito) {
    escribirServo(redondeado);
    ultimoAnguloEscrito = redondeado;
  }
}

bool iniciarMotor() {
  pinMode(PIN_MOTOR_AIN1, OUTPUT);
  pinMode(PIN_MOTOR_AIN2, OUTPUT);
  digitalWrite(PIN_MOTOR_AIN1, LOW);
  digitalWrite(PIN_MOTOR_AIN2, LOW);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttach(PIN_MOTOR_PWM, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
#else
  ledcSetup(CANAL_PWM_MOTOR, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttachPin(PIN_MOTOR_PWM, CANAL_PWM_MOTOR);
  return true;
#endif
}

// Unico punto del programa que mueve el motor. Un valor positivo avanza,
// uno negativo da reversa, cero queda libre y ORDEN_FRENO_ACTIVO aplica
// frenado electrico en seco.
void aplicarMotor(int porcentaje) {
  if (!motorListo) {
    return;
  }

  const bool frenoActivo = porcentaje == ORDEN_FRENO_ACTIVO;
  if (!frenoActivo) {
    porcentaje = constrain(porcentaje, -100, 100);
  }
  direccionEncoderActual =
      frenoActivo || porcentaje == 0 ? 0 : (porcentaje > 0 ? 1 : -1);
  const uint8_t magnitud =
      frenoActivo ? 100 : static_cast<uint8_t>(abs(porcentaje));

  if (frenoActivo) {
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    digitalWrite(PIN_MOTOR_AIN2, HIGH);
  } else if (porcentaje == 0) {
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
  } else {
    const bool haciaAdelante = porcentaje > 0;
    const bool ain1Alto =
        haciaAdelante ? !INVERTIR_DIRECCION_MOTOR
                      : INVERTIR_DIRECCION_MOTOR;
    digitalWrite(PIN_MOTOR_AIN1, ain1Alto ? HIGH : LOW);
    digitalWrite(PIN_MOTOR_AIN2, ain1Alto ? LOW : HIGH);
  }

  if (porcentaje == ultimaVelocidadEscrita) {
    return;
  }

  constexpr uint32_t DUTY_MAXIMO = (1UL << MOTOR_PWM_BITS) - 1UL;
  const uint32_t duty =
      (static_cast<uint32_t>(magnitud) * DUTY_MAXIMO + 50UL) / 100UL;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_MOTOR_PWM, duty);
#else
  ledcWrite(CANAL_PWM_MOTOR, duty);
#endif
  ultimaVelocidadEscrita = porcentaje;
}

void IRAM_ATTR pulsoEncoder() {
  pulsosEncoder = pulsosEncoder + 1;
  pulsosEncoderNetos += direccionEncoderActual;
}

void iniciarEncoder() {
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), pulsoEncoder, RISING);
}

uint32_t leerPulsosEncoder() {
  noInterrupts();
  const uint32_t pulsos = pulsosEncoder;
  interrupts();
  return pulsos;
}

int32_t leerPulsosEncoderNetos() {
  noInterrupts();
  const int32_t pulsos = pulsosEncoderNetos;
  interrupts();
  return pulsos;
}

bool salidaInicialParkingActiva() {
  return fase == Fase::SALIDA_AVANZAR ||
         fase == Fase::SALIDA_PAUSA_FRENTE ||
         fase == Fase::SALIDA_RETROCEDER ||
         fase == Fase::SALIDA_ELEGIR_LADO ||
         fase == Fase::SALIDA_ORIENTAR_SERVO ||
         fase == Fase::SALIDA_GIRAR ||
         fase == Fase::SALIDA_CENTRAR_INTERMEDIO ||
         fase == Fase::SALIDA_REVERSA_CENTRO_INTERMEDIA ||
         fase == Fase::SALIDA_ORIENTAR_SEGUNDO_GIRO ||
         fase == Fase::SALIDA_SEGUNDO_GIRO ||
         fase == Fase::SALIDA_RECUPERAR_RUMBO ||
         fase == Fase::SALIDA_AVANCE_CONTRARIO ||
         fase == Fase::SALIDA_RETROCESO_GIRO ||
         fase == Fase::SALIDA_RETROCESO_CENTRO;
}

void cancelarSalidaInicial(const char *motivo) {
  salidaInicialCancelada = true;
  fase = Fase::TERMINADO;
  Serial.print("SALIDA PARKING DETENIDA: ");
  Serial.println(motivo);
}

void iniciarPausaFrontalSalida(unsigned long ahoraMs, const char *motivo) {
  fase = Fase::SALIDA_PAUSA_FRENTE;
  inicioFaseMs = ahoraMs;
  const uint16_t lectura = distanciasMm[INDICE_FRONTAL];
  const uint16_t distanciaAlFrenar =
      lectura > 0 && lectura < DISTANCIA_SIN_ECO_MM
          ? lectura
          : ultimaDistanciaCercanaSalidaMm;
  distanciaObjetivoReversaSalidaMm =
      distanciaAlFrenar + RETROCESO_SALIDA_MM;
  aplicarMotor(ORDEN_FRENO_ACTIVO);
  Serial.print("S3 ");
  Serial.print(motivo);
  Serial.print(" (lectura ");
  Serial.print(distanciasMm[INDICE_FRONTAL]);
  Serial.print(" mm)");
  Serial.print(": freno de 0.5 s; reversa hasta S3=");
  Serial.print(distanciaObjetivoReversaSalidaMm);
  Serial.println(" mm");
}

bool rumboSalidaDisponible(unsigned long ahoraMs) {
  return bno085ConRumbo && ultimoRumboSalidaValidoMs != 0 &&
         ahoraMs - ultimoRumboSalidaValidoMs <=
             MAXIMA_EDAD_RUMBO_SALIDA_MS;
}

bool rumboSalidaRecuperado(unsigned long ahoraMs) {
  if (!rumboSalidaDisponible(ahoraMs)) return false;
  const float error =
      normalizarDelta(rumboSalidaActualGrados - rumboBaseSalidaGrados);
  return fabsf(error) <= TOLERANCIA_RUMBO_SALIDA_GRADOS;
}

void actualizarSalidaInicial(unsigned long ahoraMs, bool paqueteNuevo) {
  switch (fase) {
    case Fase::SALIDA_AVANZAR:
      // El interruptor del juez puede cortar el motor mientras el ESP32
      // permanece encendido. El timeout empieza con el primer pulso real.
      if (inicioAvanceSalidaMs == 0 && leerPulsosEncoder() > 0) {
        inicioAvanceSalidaMs = ahoraMs;
      }
      if (paqueteNuevo && distanciaValida[INDICE_FRONTAL] &&
          distanciasMm[INDICE_FRONTAL] > 0 &&
          distanciasMm[INDICE_FRONTAL] <= FRENTE_ECO_CERCANO_SALIDA_MM) {
        s3VioParedCercana = true;
        ultimaDistanciaCercanaSalidaMm = distanciasMm[INDICE_FRONTAL];
      }
      if (hayDistancias && distanciaValida[INDICE_FRONTAL] &&
          distanciasMm[INDICE_FRONTAL] > 0 &&
          distanciasMm[INDICE_FRONTAL] <= FRENTE_OBJETIVO_SALIDA_MM) {
        iniciarPausaFrontalSalida(ahoraMs, "cerca de 2 cm");
      } else if (paqueteNuevo && distanciaValida[INDICE_FRONTAL] &&
                 (distanciasMm[INDICE_FRONTAL] == 0 ||
                  distanciasMm[INDICE_FRONTAL] >= DISTANCIA_SIN_ECO_MM)) {
        if (s3VioParedCercana) {
          iniciarPausaFrontalSalida(ahoraMs, "perdio el eco cerca de la pared");
        } else {
          cancelarSalidaInicial("S3 sin eco antes de acercarse a la pared");
        }
      } else if (paqueteNuevo && !distanciaValida[INDICE_FRONTAL]) {
        cancelarSalidaInicial("lectura S3 invalida durante el acercamiento");
      } else if (inicioAvanceSalidaMs != 0 &&
                 ahoraMs - inicioAvanceSalidaMs >= MAXIMO_AVANCE_SALIDA_MS) {
        cancelarSalidaInicial("S3 no detecto la pared cercana durante el avance");
      }
      break;

    case Fase::SALIDA_PAUSA_FRENTE:
      if (ahoraMs - inicioFaseMs >= PAUSA_FRENTE_SALIDA_MS) {
        fase = Fase::SALIDA_RETROCEDER;
        inicioFaseMs = ahoraMs;
        Serial.println("Pausa terminada: retrocediendo 7 cm por S3");
      }
      break;

    case Fase::SALIDA_RETROCEDER: {
      // Confirmado por S3: 7 cm mas lejos que al empezar la pausa.
      if (paqueteNuevo && distanciaValida[INDICE_FRONTAL] &&
          distanciasMm[INDICE_FRONTAL] >= distanciaObjetivoReversaSalidaMm) {
        fase = Fase::SALIDA_ELEGIR_LADO;
        inicioFaseMs = ahoraMs;
        Serial.print("Reversa terminada; S3=");
        Serial.print(distanciasMm[INDICE_FRONTAL]);
        Serial.println(" mm");
      } else if (ahoraMs - inicioFaseMs >= MAXIMO_REVERSA_SALIDA_MS) {
        cancelarSalidaInicial("S3 no confirmo los 7 cm de reversa");
      }
      break;
    }

    case Fase::SALIDA_ELEGIR_LADO:
      if (paqueteNuevo && distanciaValida[INDICE_IZQ_90] &&
          distanciaValida[INDICE_DER_90] && rumboSalidaDisponible(ahoraMs)) {
        const uint16_t izquierda = distanciasMm[INDICE_IZQ_90];
        const uint16_t derecha = distanciasMm[INDICE_DER_90];
        if (izquierda == derecha && distanciaValida[INDICE_IZQ_25] &&
            distanciaValida[INDICE_DER_25]) {
          ladoSalidaParking =
              distanciasMm[INDICE_IZQ_25] > distanciasMm[INDICE_DER_25]
                  ? -1 : 1;
        } else {
          ladoSalidaParking = izquierda > derecha ? -1 : 1;
        }
        // Guardar hacia donde mira el robot antes del primer arco.
        rumboBaseSalidaGrados = rumboSalidaActualGrados;
        fase = Fase::SALIDA_ORIENTAR_SERVO;
        inicioFaseMs = ahoraMs;
        Serial.print("Mayor espacio a la ");
        Serial.print(ladoSalidaParking < 0 ? "izquierda" : "derecha");
        Serial.print("; S1=");
        Serial.print(izquierda);
        Serial.print(" mm, S5=");
        Serial.println(derecha);
      } else if (ahoraMs - inicioFaseMs >= MAXIMO_ELEGIR_LADO_MS) {
        cancelarSalidaInicial("faltan lecturas laterales o rumbo BNO085");
      }
      break;

    case Fase::SALIDA_ORIENTAR_SERVO:
      if (ahoraMs - inicioFaseMs >= PAUSA_SERVO_GIRADO_MS) {
        fase = Fase::SALIDA_GIRAR;
        inicioFaseMs = ahoraMs;
        Serial.println("Servo al tope hacia el lado elegido: avanzando");
      }
      break;

    case Fase::SALIDA_GIRAR:
      // Primer avance de 2 s con el servo hacia el lado de mayor espacio.
      if (paqueteNuevo && distanciaValida[INDICE_FRONTAL] &&
          distanciasMm[INDICE_FRONTAL] > 0 &&
          distanciasMm[INDICE_FRONTAL] <= FRENTE_EMERGENCIA_GIRO_MM) {
        cancelarSalidaInicial("obstaculo frontal durante el giro");
      } else if (ahoraMs - inicioFaseMs >= TIEMPO_GIRO_SALIDA_MS) {
        fase = Fase::SALIDA_CENTRAR_INTERMEDIO;
        inicioFaseMs = ahoraMs;
        Serial.println("Primer avance de 2 s completo: servo a 90 grados");
      }
      break;

    case Fase::SALIDA_CENTRAR_INTERMEDIO:
      if (ahoraMs - inicioFaseMs >= PAUSA_SERVO_CENTRO_SALIDA_MS) {
        fase = Fase::SALIDA_REVERSA_CENTRO_INTERMEDIA;
        inicioFaseMs = ahoraMs;
        Serial.println("Servo centrado: retrocediendo 2 s");
      }
      break;

    case Fase::SALIDA_REVERSA_CENTRO_INTERMEDIA:
      if (ahoraMs - inicioFaseMs >= TIEMPO_REVERSA_CENTRO_INTERMEDIA_MS) {
        fase = Fase::SALIDA_ORIENTAR_SEGUNDO_GIRO;
        inicioFaseMs = ahoraMs;
        Serial.println("Reversa recta completa: servo hacia el lado elegido");
      }
      break;

    case Fase::SALIDA_ORIENTAR_SEGUNDO_GIRO:
      if (ahoraMs - inicioFaseMs >= PAUSA_SERVO_GIRADO_MS) {
        fase = Fase::SALIDA_SEGUNDO_GIRO;
        inicioFaseMs = ahoraMs;
        Serial.println("Servo al tope: segundo avance de 2 s");
      }
      break;

    case Fase::SALIDA_SEGUNDO_GIRO:
      if (paqueteNuevo && distanciaValida[INDICE_FRONTAL] &&
          distanciasMm[INDICE_FRONTAL] > 0 &&
          distanciasMm[INDICE_FRONTAL] <= FRENTE_EMERGENCIA_GIRO_MM) {
        cancelarSalidaInicial("obstaculo frontal durante el segundo giro");
      } else if (ahoraMs - inicioFaseMs >= TIEMPO_SEGUNDO_GIRO_SALIDA_MS) {
        if (!rumboSalidaDisponible(ahoraMs)) {
          cancelarSalidaInicial("BNO085 sin rumbo al terminar el segundo giro");
          break;
        }
        fase = rumboSalidaRecuperado(ahoraMs)
                   ? Fase::SALIDA_AVANCE_CONTRARIO
                   : Fase::SALIDA_RECUPERAR_RUMBO;
        inicioFaseMs = ahoraMs;
        Serial.println(fase == Fase::SALIDA_RECUPERAR_RUMBO
                           ? "Segundo avance completo: recuperando rumbo (maximo 1 s)"
                           : "Rumbo original confirmado: avance contrario 2 s");
      }
      break;

    case Fase::SALIDA_RECUPERAR_RUMBO:
      if (paqueteNuevo && distanciaValida[INDICE_FRONTAL] &&
          distanciasMm[INDICE_FRONTAL] > 0 &&
          distanciasMm[INDICE_FRONTAL] <= FRENTE_EMERGENCIA_GIRO_MM) {
        cancelarSalidaInicial("obstaculo frontal al recuperar el rumbo");
      } else if (!rumboSalidaDisponible(ahoraMs)) {
        cancelarSalidaInicial("BNO085 sin rumbo durante la alineacion");
      } else if (rumboSalidaRecuperado(ahoraMs)) {
        fase = Fase::SALIDA_AVANCE_CONTRARIO;
        inicioFaseMs = ahoraMs;
        Serial.println("Rumbo original recuperado: servo al lado contrario, avanzando 2 s");
      } else if (ahoraMs - inicioFaseMs >=
                 MAXIMO_RECUPERAR_RUMBO_SALIDA_MS) {
        fase = Fase::SALIDA_AVANCE_CONTRARIO;
        inicioFaseMs = ahoraMs;
        Serial.println("Giro de regreso limitado a 1 s; continua el coleo");
      }
      break;

    case Fase::SALIDA_AVANCE_CONTRARIO:
      // Servo al maximo hacia el lado contrario al elegido; sigue avanzando.
      if (ahoraMs - inicioFaseMs >= DURACION_AVANCE_CONTRARIO_SALIDA_MS) {
        fase = Fase::SALIDA_RETROCESO_GIRO;
        inicioFaseMs = ahoraMs;
        Serial.println("Avance contrario completo: servo al lado original, retrocediendo 2 s");
      }
      break;

    case Fase::SALIDA_RETROCESO_GIRO:
      // Servo al maximo de vuelta hacia el lado elegido originalmente;
      // ahora retrocede.
      if (ahoraMs - inicioFaseMs >= DURACION_RETROCESO_GIRO_SALIDA_MS) {
        fase = Fase::SALIDA_RETROCESO_CENTRO;
        inicioFaseMs = ahoraMs;
        Serial.println("Retroceso con giro completo: servo a 90 grados, retrocediendo 3 s");
      }
      break;

    case Fase::SALIDA_RETROCESO_CENTRO:
      if (ahoraMs - inicioFaseMs >= DURACION_RETROCESO_CENTRO_SALIDA_MS) {
        fase = Fase::ESPERANDO;
        Serial.println("Salida completa: comienza la logica normal");
      }
      break;

    default:
      break;
  }
}

// ====================================================================
//                       ULTRASONICOS POR I2C
// ====================================================================

bool leerDistancias(unsigned long ahoraMs) {
  const uint8_t recibidos =
      Wire.requestFrom(static_cast<uint8_t>(DIRECCION_NANO),
                       static_cast<uint8_t>(BYTES_PAQUETE));

  if (recibidos != BYTES_PAQUETE) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < CANTIDAD_SENSORES; ++i) {
    const uint8_t bajo = Wire.read();
    const uint8_t alto = Wire.read();
    const uint16_t valor =
        static_cast<uint16_t>(bajo) | (static_cast<uint16_t>(alto) << 8);
    if (valor != DISTANCIA_INVALIDA_MM) {
      distanciaValida[i] = true;
      distanciasMm[i] = valor;
      ultimaDistanciaValidaMm[i] = valor;
      ultimoEcoValidoMs[i] = ahoraMs;
    } else if (ultimoEcoValidoMs[i] != 0 &&
               ahoraMs - ultimoEcoValidoMs[i] <=
                   RETENCION_ULTIMO_ECO_VALIDO_MS) {
      distanciaValida[i] = true;
      distanciasMm[i] = ultimaDistanciaValidaMm[i];
    } else {
      distanciaValida[i] = false;
      distanciasMm[i] = DISTANCIA_SIN_ECO_MM;
    }
  }

  return true;
}

void actualizarClasificacionPasilloInicial() {
  if ((fase != Fase::ESPERANDO && fase != Fase::SALIDA_RETROCESO_CENTRO &&
       fase != Fase::RODANDO) || pasilloInicialClasificado ||
      !distanciaValida[INDICE_IZQ_90] ||
      !distanciaValida[INDICE_DER_90]) {
    return;
  }

  acumuladoAnchoPasilloInicialMm +=
      static_cast<uint32_t>(distanciasMm[INDICE_IZQ_90]) +
      distanciasMm[INDICE_DER_90];
  ++muestrasPasilloInicial;

  if (muestrasPasilloInicial < MUESTRAS_PARA_CLASIFICAR_PASILLO) {
    return;
  }

  anchoPasilloInicialMm = static_cast<uint16_t>(
      acumuladoAnchoPasilloInicialMm / muestrasPasilloInicial);
  pasilloInicialAncho = anchoPasilloInicialMm >= UMBRAL_PASILLO_ANCHO_MM;
  duracionAvanceFinalSeleccionadaMs = pasilloInicialAncho
      ? DURACION_AVANCE_FINAL_PASILLO_ANCHO_MS
      : DURACION_AVANCE_FINAL_PASILLO_ANGOSTO_MS;
  pasilloInicialClasificado = true;

  Serial.print("Pasillo inicial ");
  Serial.print(pasilloInicialAncho ? "ancho" : "angosto");
  Serial.print(" (S1+S5=");
  Serial.print(anchoPasilloInicialMm);
  Serial.print(" mm): avance final ");
  Serial.print(duracionAvanceFinalSeleccionadaMs);
  Serial.println(" ms");
}

void enviarBrilloLeds(uint8_t porcentaje) {
  Wire.beginTransmission(DIRECCION_NANO);
  Wire.write(static_cast<uint8_t>(constrain(porcentaje, 0, 100)));
  Wire.endTransmission();
}

// ====================================================================
//                         OPENMV POR UART
// ====================================================================

Color colorDeId(long id) {
  if (id == ID_ROJO) return Color::ROJO;
  if (id == ID_VERDE) return Color::VERDE;
  return Color::NINGUNO;
}

long limitarLong(long valor, long minimo, long maximo) {
  if (valor < minimo) return minimo;
  if (valor > maximo) return maximo;
  return valor;
}

// Separa una linea CSV numerica. Los campos posteriores al decimo se ignoran
// para no romper el enlace si el firmware agrega telemetria en el futuro.
int separarCamposOpenMV(const char *linea, long *campos, int capacidad) {
  int cantidad = 0;
  const char *cursor = linea;

  while (*cursor != '\0') {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == '\0') break;

    bool negativo = false;
    if (*cursor == '+' || *cursor == '-') {
      negativo = *cursor == '-';
      ++cursor;
    }
    if (*cursor < '0' || *cursor > '9') return -1;

    long valor = 0;
    uint8_t digitos = 0;
    while (*cursor >= '0' && *cursor <= '9') {
      if (digitos >= 9) return -1;
      valor = valor * 10L + static_cast<long>(*cursor - '0');
      ++digitos;
      ++cursor;
    }
    if (negativo) valor = -valor;
    if (cantidad < capacidad) campos[cantidad] = valor;
    ++cantidad;

    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == ',') {
      ++cursor;
    } else if (*cursor != '\0') {
      return -1;
    }
  }

  return cantidad;
}

bool procesarLineaOpenMV(const char *linea, unsigned long ahoraMs) {
  long campos[OPENMV_CAMPOS_MAXIMOS] = {0};
  const int recibidos =
      separarCamposOpenMV(linea, campos, OPENMV_CAMPOS_MAXIMOS);

  if (recibidos < OPENMV_CAMPOS_MINIMOS) return false;

  const long id = campos[0];
  const long x = campos[1];
  const long y = campos[2];
  const long area = campos[3];
  const long roi = campos[4];
  const long colision = campos[5];
  const long paredNegra = recibidos >= 7 ? campos[6] : 0;

  if ((id != 0 && id != ID_ROJO && id != ID_VERDE) ||
      roi < 0 || roi > 2 ||
      (colision != 0 && colision != ID_ROJO && colision != ID_VERDE) ||
      (paredNegra != 0 && paredNegra != 1)) {
    return false;
  }

  bool nuevaParedEstacionamiento = false;
  int16_t nuevaParedX = 0;
  uint16_t nuevaParedArea = 0;
  if (recibidos >= 10 && campos[7] != 0 &&
      campos[8] >= OPENMV_X_MIN - OPENMV_X_TOLERANCIA &&
      campos[8] <= OPENMV_X_MAX + OPENMV_X_TOLERANCIA &&
      campos[9] > 0 && campos[9] <= UINT16_MAX) {
    nuevaParedEstacionamiento = true;
    nuevaParedX = static_cast<int16_t>(
        limitarLong(campos[8], OPENMV_X_MIN, OPENMV_X_MAX));
    nuevaParedArea = static_cast<uint16_t>(campos[9]);
  }

  bool nuevoHuecoEstacionamiento = false;
  int16_t nuevoHuecoX = 0;
  uint16_t nuevoAnchoHuecoPx = 0;
  if (recibidos >= 13 && campos[10] != 0 &&
      campos[11] >= OPENMV_X_MIN - OPENMV_X_TOLERANCIA &&
      campos[11] <= OPENMV_X_MAX + OPENMV_X_TOLERANCIA &&
      campos[12] > 0 && campos[12] <= OPENMV_ANCHO_IMAGEN_PX) {
    nuevoHuecoEstacionamiento = true;
    nuevoHuecoX = static_cast<int16_t>(
        limitarLong(campos[11], OPENMV_X_MIN, OPENMV_X_MAX));
    nuevoAnchoHuecoPx = static_cast<uint16_t>(campos[12]);
  }

  Pilar nuevo;
  if (id != 0) {
    if (x < OPENMV_X_MIN - OPENMV_X_TOLERANCIA ||
        x > OPENMV_X_MAX + OPENMV_X_TOLERANCIA ||
        y < 0 || y >= OPENMV_ALTO_IMAGEN_PX ||
        area < AREA_MINIMA_PILAR_PX || area > UINT16_MAX ||
        (roi != 1 && roi != 2)) {
      return false;
    }

    nuevo.color = colorDeId(id);
    nuevo.id = static_cast<uint8_t>(id);
    nuevo.xCamara = static_cast<float>(
        limitarLong(x, OPENMV_X_MIN, OPENMV_X_MAX));
    nuevo.yPx = static_cast<int16_t>(y);
    nuevo.areaPx = static_cast<uint16_t>(area);
    nuevo.roi = static_cast<uint8_t>(roi);
  }

  if (nuevo.color != Color::NINGUNO) {
    pilar = nuevo;
    tramasConsecutivasSinPilar = 0;
  } else if (pilar.color != Color::NINGUNO) {
    if (tramasConsecutivasSinPilar <
        TRAMAS_PERDIDAS_PARA_SOLTAR_PILAR) {
      ++tramasConsecutivasSinPilar;
    }
    if (tramasConsecutivasSinPilar >=
        TRAMAS_PERDIDAS_PARA_SOLTAR_PILAR) {
      const Color colorRebasado = pilar.color;
      pilar = Pilar();
      const bool navegandoVueltas =
          fase == Fase::RODANDO ||
          fase == Fase::BUSCANDO_ESTACIONAMIENTO ||
          fase == Fase::AVANCE_FINAL_TRES_VUELTAS;
      if (navegandoVueltas) {
        pausaDespuesObstaculoActiva = true;
        inicioPausaDespuesObstaculoMs = ahoraMs;
        Serial.print("Pilar ");
        Serial.print(colorRebasado == Color::ROJO ? "rojo" : "verde");
        Serial.print(" rebasado: observando durante ");
        Serial.print(PAUSA_DESPUES_OBSTACULO_MS);
        Serial.println(" ms");
      }
    }
  } else {
    pilar = nuevo;
    tramasConsecutivasSinPilar = 0;
  }
  if (colision != 0) {
    colisionVisual = static_cast<uint8_t>(colision);
    ultimaAlertaColisionMs = ahoraMs;
  } else if (ahoraMs - ultimaAlertaColisionMs >
             RETENCION_ALERTA_COLISION_MS) {
    colisionVisual = 0;
  }
  paredNegraVisible = paredNegra != 0;
  paredEstacionamientoVisible = nuevaParedEstacionamiento;
  paredEstacionamientoX = nuevaParedX;
  paredEstacionamientoArea = nuevaParedArea;
  huecoEstacionamientoVisible = nuevoHuecoEstacionamiento;
  huecoEstacionamientoX = nuevoHuecoX;
  anchoHuecoEstacionamientoPx = nuevoAnchoHuecoPx;

  if (busquedaRosaActiva()) {
    if (paredEstacionamientoVisible) {
      if (confirmacionesParedEstacionamiento <
          TRAMAS_PARED_ESTACIONAMIENTO) {
        ++confirmacionesParedEstacionamiento;
      }
    } else {
      confirmacionesParedEstacionamiento = 0;
    }

    if (huecoEstacionamientoVisible) {
      if (confirmacionesHuecoEstacionamiento <
          TRAMAS_HUECO_ESTACIONAMIENTO) {
        ++confirmacionesHuecoEstacionamiento;
      }
      if (confirmacionesHuecoEstacionamiento >=
          TRAMAS_HUECO_ESTACIONAMIENTO) {
        huecoParkingGuardado = true;
        huecoParkingXGuardado = huecoEstacionamientoX;
        anchoHuecoParkingGuardadoPx = anchoHuecoEstacionamientoPx;
      }
    } else {
      confirmacionesHuecoEstacionamiento = 0;
    }
  }

  ultimoPaqueteCamaraMs = ahoraMs;
  ++tramasOpenMvValidas;

  if (!camaraLista) {
    camaraLista = true;
    Serial.println("OpenMV: enlace UART establecido");
  }
  return true;
}

void avisarTramaOpenMvInvalida(unsigned long ahoraMs) {
  ++tramasOpenMvInvalidas;
  if (ahoraMs - ultimoAvisoOpenMvMs >= OPENMV_AVISO_INVALIDO_MS) {
    ultimoAvisoOpenMvMs = ahoraMs;
    Serial.println("AVISO: trama OpenMV invalida");
  }
}

void iniciarCamara() {
  openMVSerial.setRxBufferSize(OPENMV_BUFFER_RX_BYTES);
  openMVSerial.begin(OPENMV_BAUD, SERIAL_8N1,
                     PIN_OPENMV_RX, PIN_OPENMV_TX);
  Serial.print("OpenMV UART2 lista: RX GPIO");
  Serial.print(PIN_OPENMV_RX);
  Serial.print(", TX GPIO");
  Serial.print(PIN_OPENMV_TX);
  Serial.print(", ");
  Serial.print(OPENMV_BAUD);
  Serial.println(" baudios");
}

void actualizarCamara(unsigned long ahoraMs) {
  // Si se acumularon demasiados bytes, son imagenes viejas. Se descarta la
  // parte atrasada y se recupera el encuadre con la siguiente linea completa.
  if (openMVSerial.available() > OPENMV_MAXIMO_ATRASO_BYTES) {
    while (openMVSerial.available() > OPENMV_MAXIMO_ATRASO_BYTES) {
      openMVSerial.read();
    }
    largoLineaOpenMV = 0;
    descartandoLineaOpenMV = true;
  }

  uint8_t lineasAtendidas = 0;
  while (openMVSerial.available() > 0 &&
         lineasAtendidas < OPENMV_LINEAS_POR_CICLO) {
    const char entrada = static_cast<char>(openMVSerial.read());

    if (entrada == '\r') continue;

    if (entrada == '\n') {
      ++lineasAtendidas;
      if (!descartandoLineaOpenMV && largoLineaOpenMV > 0) {
        lineaOpenMV[largoLineaOpenMV] = '\0';
        if (!procesarLineaOpenMV(lineaOpenMV, ahoraMs)) {
          avisarTramaOpenMvInvalida(ahoraMs);
        }
      }
      largoLineaOpenMV = 0;
      descartandoLineaOpenMV = false;
      continue;
    }

    if (descartandoLineaOpenMV) continue;

    if (largoLineaOpenMV < OPENMV_LINEA_BYTES - 1) {
      lineaOpenMV[largoLineaOpenMV++] = entrada;
    } else {
      largoLineaOpenMV = 0;
      descartandoLineaOpenMV = true;
      avisarTramaOpenMvInvalida(ahoraMs);
    }
  }

  if (camaraLista &&
      ahoraMs - ultimoPaqueteCamaraMs > OPENMV_TIMEOUT_MS) {
    camaraLista = false;
    pilar = Pilar();
    tramasConsecutivasSinPilar = 0;
    colisionVisual = 0;
    paredNegraVisible = false;
    paredEstacionamientoVisible = false;
    huecoEstacionamientoVisible = false;
    confirmacionesParedEstacionamiento = 0;
    confirmacionesHuecoEstacionamiento = 0;
    Serial.println("AVISO: enlace OpenMV perdido; sigue control por pasillo");
  }
}

void actualizarPausaDespuesObstaculo(unsigned long ahoraMs) {
  if (pausaDespuesObstaculoActiva &&
      ahoraMs - inicioPausaDespuesObstaculoMs >=
          PAUSA_DESPUES_OBSTACULO_MS) {
    pausaDespuesObstaculoActiva = false;
    Serial.println("Observacion terminada: continua la marcha");
  }
}

// color_corner_v6.py entiende P,0 para apagar la busqueda de estacionamiento y
// P,1 para encenderla. Se repite para recuperarse si OpenMV reinicia.
bool busquedaRosaActiva() {
  const bool ejecutandoEscape =
      fase == Fase::FRENANDO_ESCAPE_FRONTAL ||
      fase == Fase::RETROCEDIENDO_ESCAPE_FRONTAL ||
      fase == Fase::ESPERANDO_DESPEJE_ESCAPE ||
      fase == Fase::FRENO_TRAS_ESCAPE ||
      fase == Fase::SALIDA_FORZADA_ESCAPE;
  return fase == Fase::BUSCANDO_ESTACIONAMIENTO ||
         (ejecutandoEscape &&
          faseDespuesEscape == Fase::BUSCANDO_ESTACIONAMIENTO);
}

void enviarModoEstacionamiento(unsigned long ahoraMs) {
  const bool habilitado = busquedaRosaActiva();
  const bool cambio = !modoEstacionamientoEnviado ||
                      habilitado != ultimoModoEstacionamiento;

  if (!cambio &&
      ahoraMs - ultimoEnvioModoEstacionamientoMs <
          INTERVALO_MODO_ESTACIONAMIENTO_MS) {
    return;
  }

  openMVSerial.print("P,");
  openMVSerial.print(habilitado ? 1 : 0);
  openMVSerial.print('\n');
  ultimoEnvioModoEstacionamientoMs = ahoraMs;
  ultimoModoEstacionamiento = habilitado;
  modoEstacionamientoEnviado = true;
}

// ====================================================================
//                        BNO085 Y CONTEO
// ====================================================================

bool iniciarBnoEn(uint8_t direccion) {
  if (!bno085.begin_I2C(direccion, &Wire)) {
    return false;
  }
  if (!bno085.enableReport(SH2_GAME_ROTATION_VECTOR,
                           BNO085_INTERVALO_REPORTE_US)) {
    return false;
  }

  bno085Listo = true;
  ultimoCuadroBnoMs = millis();
  Serial.print("BNO085 listo en 0x");
  Serial.println(direccion, HEX);
  return true;
}

void iniciarBno() {
  if (iniciarBnoEn(DIRECCION_BNO085_PRIMARIA) ||
      iniciarBnoEn(DIRECCION_BNO085_SECUNDARIA)) {
    return;
  }
  Serial.println("AVISO: BNO085 no detectado en 0x4B ni 0x4A");
}

void iniciarAvanceFinalTresVueltas(unsigned long ahoraMs) {
  fase = Fase::AVANCE_FINAL_TRES_VUELTAS;
  inicioFaseMs = ahoraMs;
  avanceFinalAcumuladoMs = 0;
  ultimoAvanceFinalMs = ahoraMs;
  pulsosNetosInicioAvanceFinal = leerPulsosEncoderNetos();

  const int32_t tramoSalidaAPrimeraVuelta = pulsosNetosVuelta1;
  const int32_t longitudVuelta1 = pulsosNetosVuelta2 - pulsosNetosVuelta1;
  const int32_t longitudVuelta2 = pulsosNetosVuelta3 - pulsosNetosVuelta2;
  const int32_t longitudMaxima = max(longitudVuelta1, longitudVuelta2);
  const int32_t diferenciaVueltas = abs(longitudVuelta1 - longitudVuelta2);
  const bool vueltasConsistentes =
      longitudMaxima > 0 &&
      static_cast<int64_t>(diferenciaVueltas) * 100 <=
          static_cast<int64_t>(longitudMaxima) *
              DIFERENCIA_MAXIMA_VUELTAS_ENCODER_PORCENTAJE;

  const int32_t longitudVueltaPromedio =
      (longitudVuelta1 + longitudVuelta2) / 2;
  pulsosObjetivoAvanceFinal =
      longitudVueltaPromedio - tramoSalidaAPrimeraVuelta;
  referenciaFinalEncoderValida =
      tramoSalidaAPrimeraVuelta > 0 && longitudVuelta1 > 0 &&
      longitudVuelta2 > 0 && vueltasConsistentes &&
      pulsosObjetivoAvanceFinal > 0;

  Serial.println();
  if (referenciaFinalEncoderValida) {
    Serial.print("*** 3 VUELTAS: FALTAN ");
    Serial.print(pulsosObjetivoAvanceFinal);
    Serial.println(" PULSOS NETOS PARA VOLVER A LA SALIDA ***");
  } else {
    Serial.print("AVISO: referencia del encoder inconsistente; respaldo de ");
    Serial.print(duracionAvanceFinalSeleccionadaMs);
    Serial.println(" ms para regresar a la salida");
  }
}

void actualizarAvanceFinalTresVueltas(unsigned long ahoraMs) {
  const unsigned long transcurridoMs = ahoraMs - ultimoAvanceFinalMs;
  ultimoAvanceFinalMs = ahoraMs;

  if (fase != Fase::AVANCE_FINAL_TRES_VUELTAS) {
    return;
  }

  // Solo suma tiempo si en el ciclo anterior hubo avance real. Las pausas,
  // fallas de sensores y escapes no consumen el tramo final.
  if (ultimaVelocidadEscrita > 0 && ultimaVelocidadEscrita <= 100) {
    avanceFinalAcumuladoMs += transcurridoMs;
  }

  const int32_t avanceNeto =
      leerPulsosEncoderNetos() - pulsosNetosInicioAvanceFinal;
  const bool puntoSalidaAlcanzado =
      referenciaFinalEncoderValida &&
      avanceNeto >= pulsosObjetivoAvanceFinal;
  const bool respaldoPorTiempo =
      !referenciaFinalEncoderValida &&
      avanceFinalAcumuladoMs >= duracionAvanceFinalSeleccionadaMs;
  const bool limiteSeguridad =
      referenciaFinalEncoderValida &&
      avanceFinalAcumuladoMs >= MAXIMO_AVANCE_FINAL_ENCODER_MS;

  if (puntoSalidaAlcanzado || respaldoPorTiempo || limiteSeguridad) {
    fase = Fase::FRENANDO_FIN_TRES_VUELTAS;
    inicioFaseMs = ahoraMs;
    Serial.println();
    if (puntoSalidaAlcanzado) {
      Serial.print("Punto de salida alcanzado por encoder: ");
      Serial.print(avanceNeto);
      Serial.println(" pulsos; aplicando freno activo");
    } else if (limiteSeguridad) {
      Serial.println(
          "AVISO: limite del avance final; aplicando freno de seguridad");
    } else {
      Serial.println("Llegada final por respaldo de tiempo; aplicando freno");
    }
  }
}

void actualizarConteoVueltas(unsigned long ahoraMs) {
  // La primera curva fija el sentido. A partir de ahi se usa progreso con
  // signo; una correccion en sentido contrario nunca adelanta el contador.
  if (signoGiroVueltas == 0 &&
      fabsf(giroConteoVueltasGrados) >=
          GRADOS_PARA_FIJAR_SENTIDO_VUELTA) {
    signoGiroVueltas = giroConteoVueltasGrados >= 0.0f ? 1 : -1;
    Serial.print("Sentido de vueltas fijado: ");
    Serial.println(signoGiroVueltas > 0 ? "positivo" : "negativo");
  }

  if (signoGiroVueltas == 0 || esquinas >= ESQUINAS_PARA_TERMINAR) {
    return;
  }

  const float progresoGrados =
      static_cast<float>(signoGiroVueltas) * giroConteoVueltasGrados;
  const float objetivoSiguienteEsquina =
      static_cast<float>(esquinas + 1) * GRADOS_POR_ESQUINA;

  if (progresoGrados >= objetivoSiguienteEsquina) {
    if (muestrasConfirmacionEsquina <
        MUESTRAS_PARA_CONFIRMAR_ESQUINA) {
      ++muestrasConfirmacionEsquina;
    }
  } else if (progresoGrados <
             objetivoSiguienteEsquina - HISTERESIS_CONTEO_VUELTA_GRADOS) {
    muestrasConfirmacionEsquina = 0;
  }

  if (muestrasConfirmacionEsquina < MUESTRAS_PARA_CONFIRMAR_ESQUINA) {
    return;
  }

  muestrasConfirmacionEsquina = 0;
  ++esquinas;
  Serial.print("Esquina confirmada: ");
  Serial.print(esquinas);
  Serial.print("/");
  Serial.println(ESQUINAS_PARA_TERMINAR);

  const uint8_t vueltasNuevas = esquinas / 4;
  if (vueltasNuevas > vueltasCompletadas) {
    vueltasCompletadas = min(vueltasNuevas, VUELTAS_OBJETIVO);
    const int32_t pulsosDesdeSalida =
        leerPulsosEncoderNetos() - pulsosNetosInicioRonda;
    if (vueltasCompletadas == 1) {
      pulsosNetosVuelta1 = pulsosDesdeSalida;
    } else if (vueltasCompletadas == 2) {
      pulsosNetosVuelta2 = pulsosDesdeSalida;
    } else if (vueltasCompletadas == 3) {
      pulsosNetosVuelta3 = pulsosDesdeSalida;
    }
    Serial.print("Vuelta confirmada: ");
    Serial.print(vueltasCompletadas);
    Serial.print("/");
    Serial.print(VUELTAS_OBJETIVO);
    Serial.print(" | encoder neto ");
    Serial.println(pulsosDesdeSalida);
  }

  if (!HABILITAR_ESTACIONAMIENTO &&
      esquinas >= ESQUINAS_PARA_TERMINAR) {
    iniciarAvanceFinalTresVueltas(ahoraMs);
    return;
  }

  if (HABILITAR_ESTACIONAMIENTO && fase == Fase::RODANDO &&
      esquinas >= VUELTAS_ANTES_BUSQUEDA_ROSA * 4) {
    giroObjetivoFinalGrados =
        static_cast<float>(signoGiroVueltas) * GRADOS_POR_VUELTA *
        static_cast<float>(VUELTAS_OBJETIVO);
    fase = Fase::BUSCANDO_ESTACIONAMIENTO;
    inicioFaseMs = ahoraMs;
    inicioBusquedaParkingMs = ahoraMs;
    confirmacionesParedEstacionamiento = 0;
    confirmacionesHuecoEstacionamiento = 0;
    huecoParkingGuardado = false;
    ladoParking = 0;

    Serial.println();
    Serial.println("*** BUSQUEDA DE ESTACIONAMIENTO INICIADA ***");
  }
}

void procesarRumbo(float rumboGrados, unsigned long ahoraMs) {
  ultimoCuadroBnoMs = ahoraMs;

  if (!bno085ConRumbo) {
    bno085ConRumbo = true;
    rumboAnteriorGrados = rumboGrados;
    if (isfinite(rumboGrados)) {
      rumboSalidaActualGrados = rumboGrados;
      ultimoRumboSalidaValidoMs = ahoraMs;
    }
    return;
  }

  const float delta = normalizarDelta(rumboGrados - rumboAnteriorGrados);
  rumboAnteriorGrados = rumboGrados;
  if (isfinite(rumboGrados) && isfinite(delta) &&
      fabsf(delta) <= MAXIMO_SALTO_RUMBO_GRADOS) {
    rumboSalidaActualGrados = rumboGrados;
    ultimoRumboSalidaValidoMs = ahoraMs;
  }

  // La ronda todavia no empieza o ya termino.
  if (fase == Fase::ESPERANDO || fase == Fase::TERMINADO ||
      salidaInicialParkingActiva()) {
    return;
  }

  // Un solo cuadro con NaN dejaria el acumulado en NaN para siempre y con
  // el toda la mision.
  if (!isfinite(delta) || fabsf(delta) > MAXIMO_SALTO_RUMBO_GRADOS) {
    return;
  }

  giroAcumuladoGrados += delta;

  // Integra tambien el rumbo de freno, reversa y salida forzada. Al excluir
  // esos giros, sus correcciones no se cancelaban y el robot podia declarar
  // tres vueltas al otro lado de la pista. Las esquinas solo se confirman al
  // regresar a conduccion normal para evitar aceptar una maniobra temporal.
  const bool navegacionNormal = fase == Fase::RODANDO ||
                                fase == Fase::BUSCANDO_ESTACIONAMIENTO;
  const bool escapeDeUnaVuelta =
      (fase == Fase::FRENANDO_ESCAPE_FRONTAL ||
       fase == Fase::RETROCEDIENDO_ESCAPE_FRONTAL ||
       fase == Fase::ESPERANDO_DESPEJE_ESCAPE ||
       fase == Fase::FRENO_TRAS_ESCAPE ||
       fase == Fase::SALIDA_FORZADA_ESCAPE) &&
      (faseDespuesEscape == Fase::RODANDO ||
       faseDespuesEscape == Fase::BUSCANDO_ESTACIONAMIENTO);

  if (!navegacionNormal && !escapeDeUnaVuelta) {
    return;
  }

  giroConteoVueltasGrados += delta;
  if (navegacionNormal) {
    actualizarConteoVueltas(ahoraMs);
  }
}

void actualizarBno(unsigned long ahoraMs) {
  if (!bno085Listo) {
    if (ahoraMs - ultimoReintentoBnoMs >= BNO085_REINTENTO_MS) {
      ultimoReintentoBnoMs = ahoraMs;
      iniciarBno();
    }
    return;
  }

  // Contesta en I2C pero ya no manda cuaterniones. Reiniciarlo es la unica
  // salida: el conteo de vueltas depende por completo de el.
  if (ultimoCuadroBnoMs != 0 &&
      ahoraMs - ultimoCuadroBnoMs >= BNO085_SIN_DATOS_MS) {
    Serial.println("BNO085 sin datos: se reinicia");
    bno085Listo = false;
    bno085ConRumbo = false;
    ultimoReintentoBnoMs = ahoraMs;
    return;
  }

  if (bno085.wasReset()) {
    bno085ConRumbo = false;
    if (!bno085.enableReport(SH2_GAME_ROTATION_VECTOR,
                             BNO085_INTERVALO_REPORTE_US)) {
      bno085Listo = false;
      return;
    }
  }

  while (bno085.getSensorEvent(&valorBno085)) {
    if (valorBno085.sensorId != SH2_GAME_ROTATION_VECTOR) {
      continue;
    }

    const float qr = valorBno085.un.gameRotationVector.real;
    const float qi = valorBno085.un.gameRotationVector.i;
    const float qj = valorBno085.un.gameRotationVector.j;
    const float qk = valorBno085.un.gameRotationVector.k;
    const float rumbo = atan2f(2.0f * (qi * qj + qk * qr),
                               qi * qi - qj * qj - qk * qk + qr * qr) *
                        RAD_TO_DEG;
    procesarRumbo(rumbo, ahoraMs);
  }
}

// ====================================================================
//                       LEY DE DIRECCION
// ====================================================================

// Centrarse entre las dos paredes. Es la ley de Arath con el centro en 90.
float anguloPorPasillo() {
  const float izquierda = distanciaLateralCm(distanciasMm[INDICE_IZQ_90]) +
                          distanciaLateralCm(distanciasMm[INDICE_IZQ_25]);
  const float derecha = distanciaLateralCm(distanciasMm[INDICE_DER_25]) +
                        distanciaLateralCm(distanciasMm[INDICE_DER_90]);

  const float correccion =
      constrain(KP_PASILLO * (izquierda - derecha),
                -ajustes.topePasillo, ajustes.topePasillo);

  return static_cast<float>(SERVO_CENTRO_GRADOS) +
         static_cast<float>(DIRECCION_SERVO_PASILLO) * correccion;
}

// Empujar el pilar al costado que corresponde y sostenerlo ahi.
// Rojo: queda a la izquierda de la imagen, el robot pasa por la derecha.
// Verde: queda a la derecha de la imagen, el robot pasa por la izquierda.
float anguloPorCamara() {
  const float objetivo =
      pilar.color == Color::ROJO ? -ajustes.objetivoX : ajustes.objetivoX;

  const float correccion =
      constrain(KP_CAMARA * (pilar.xCamara - objetivo),
                -ajustes.topeCamara, ajustes.topeCamara);

  return static_cast<float>(SERVO_CENTRO_GRADOS) +
         static_cast<float>(DIRECCION_SERVO_CAMARA) * correccion;
}

bool vueltasHaciaIzquierda() {
  return signoGiroVueltas == SIGNO_BNO_GIRO_IZQUIERDA;
}

int8_t ladoParkingElegido() {
  if (ladoParking != 0) return ladoParking;

  // Respaldo si el rosa quedo justo en el centro de la imagen: el cajon esta
  // al exterior de la ronda (derecha al girar a la izquierda y viceversa).
  return vueltasHaciaIzquierda() ? 1 : -1;
}

int8_t signoServoHaciaExterior() {
  return ladoParkingElegido() > 0 ? SIGNO_SERVO_DERECHA
                                  : -SIGNO_SERVO_DERECHA;
}

uint8_t indiceParking90() {
  return ladoParkingElegido() > 0 ? INDICE_DER_90 : INDICE_IZQ_90;
}

uint8_t indiceParking25() {
  return ladoParkingElegido() > 0 ? INDICE_DER_25 : INDICE_IZQ_25;
}

float anguloPegadoParedExterior() {
  const uint8_t indice = indiceParking90();
  if (!distanciaValida[indice] ||
      distanciasMm[indice] > DISTANCIA_MAXIMA_SEGUIMIENTO_EXTERIOR_MM) {
    return anguloPorPasillo();
  }

  const uint16_t distancia = distanciasMm[indice];
  if (distancia >= DISTANCIA_EXTERIOR_MINIMA_MM &&
      distancia <= DISTANCIA_EXTERIOR_MAXIMA_MM) {
    return SERVO_CENTRO_GRADOS;
  }

  const float errorMm =
      static_cast<float>(distancia) - DISTANCIA_EXTERIOR_OBJETIVO_MM;
  const float correccion = constrain(
      KP_PARED_EXTERIOR_GRADOS_POR_MM * errorMm,
      -TOPE_PARED_EXTERIOR_GRADOS, TOPE_PARED_EXTERIOR_GRADOS);

  return static_cast<float>(SERVO_CENTRO_GRADOS) +
         static_cast<float>(signoServoHaciaExterior()) * correccion;
}

void guardarMedicionParking() {
  if (confirmacionesHuecoEstacionamiento >=
      TRAMAS_HUECO_ESTACIONAMIENTO) {
    huecoParkingGuardado = true;
    huecoParkingXGuardado = huecoEstacionamientoX;
    anchoHuecoParkingGuardadoPx = anchoHuecoEstacionamientoPx;
  }
  areaParkingGuardada = paredEstacionamientoArea;
  referenciaParkingXGuardada =
      huecoParkingGuardado ? huecoParkingXGuardado : paredEstacionamientoX;

  if (referenciaParkingXGuardada < -ZONA_CENTRAL_LADO_PARKING_X) {
    ladoParking = -1;
  } else if (referenciaParkingXGuardada > ZONA_CENTRAL_LADO_PARKING_X) {
    ladoParking = 1;
  } else {
    ladoParking = vueltasHaciaIzquierda() ? 1 : -1;
  }
}

bool lecturaPrimerDelimitadorParking() {
  const uint8_t indice = indiceParking90();
  return distanciaValida[indice] &&
         distanciasMm[indice] <= UMBRAL_DELIMITADOR_PARKING_MM;
}

bool lecturaEntradaHuecoParking() {
  const uint8_t indice = indiceParking90();
  if (!distanciaValida[indice]) return false;

  const uint16_t umbralRelativo = static_cast<uint16_t>(min(
      static_cast<uint32_t>(DISTANCIA_SIN_ECO_MM),
      static_cast<uint32_t>(distanciaPrimerDelimitadorMm) +
          SALTO_MINIMO_HUECO_PARKING_MM));
  const uint16_t umbral = max(UMBRAL_HUECO_PARKING_MM, umbralRelativo);
  return distanciasMm[indice] >= umbral;
}

void reiniciarConfirmacionParking() {
  inicioCondicionParkingMs = 0;
  condicionParkingAnterior = false;
}

bool condicionParkingEstable(bool condicion, unsigned long ahoraMs) {
  if (condicion != condicionParkingAnterior) {
    condicionParkingAnterior = condicion;
    inicioCondicionParkingMs = condicion ? ahoraMs : 0;
  }

  return condicion && inicioCondicionParkingMs != 0 &&
         ahoraMs - inicioCondicionParkingMs >= BORDE_ESTABLE_PARKING_MS;
}

int8_t signoGiroHaciaExteriorParking() {
  // Con SIGNO_BNO_GIRO_IZQUIERDA=+1, un giro a la derecha es negativo.
  return ladoParkingElegido() > 0 ? -SIGNO_BNO_GIRO_IZQUIERDA
                                  : SIGNO_BNO_GIRO_IZQUIERDA;
}

bool lateralParkingDemasiadoCerca() {
  const uint8_t indice = indiceParking25();
  return distanciaValida[indice] &&
         distanciasMm[indice] <= DISTANCIA_LATERAL_EMERGENCIA_PARKING_MM;
}

float anguloHaciaEspacioParking() {
  return static_cast<float>(SERVO_CENTRO_GRADOS) +
         static_cast<float>(signoServoHaciaExterior()) *
             ANGULO_SERVO_PARKING_GRADOS;
}

float anguloFueraDelEspacioParking() {
  return static_cast<float>(SERVO_CENTRO_GRADOS) -
         static_cast<float>(signoServoHaciaExterior()) *
             ANGULO_SERVO_PARKING_GRADOS;
}

bool arcoEntradaFrontalCompleto() {
  const float progreso =
      static_cast<float>(signoGiroHaciaExteriorParking()) *
      (giroAcumuladoGrados - giroBaseManiobraParkingGrados);
  return progreso >= CAMBIO_RUMBO_ENTRADA_PARKING_GRADOS;
}

bool contraArcoFrontalCompleto() {
  return fabsf(giroAcumuladoGrados - giroBaseManiobraParkingGrados) <=
         TOLERANCIA_RUMBO_FINAL_PARKING_GRADOS;
}

bool cuartaVueltaSinParkingCompleta(unsigned long ahoraMs) {
  return fabsf(giroConteoVueltasGrados) >=
             GRADOS_POR_VUELTA * static_cast<float>(VUELTAS_OBJETIVO) ||
         ahoraMs - inicioBusquedaParkingMs >=
             MAXIMO_BUSQUEDA_ESTACIONAMIENTO_MS;
}

void iniciarFrenoFinalParking(unsigned long ahoraMs, bool exitoso,
                              const char *motivo) {
  estacionamientoExitoso = exitoso;
  fase = Fase::FRENO_FINAL_ESTACIONAMIENTO;
  inicioFaseMs = ahoraMs;
  Serial.println(motivo);
}

bool alertaColisionVisualActiva(unsigned long ahoraMs) {
  return camaraLista && colisionVisual != 0 &&
         ahoraMs - ultimaAlertaColisionMs <= RETENCION_ALERTA_COLISION_MS;
}

float anguloReversaEscape() {
  if (pilarAlIniciarEscape.color != Color::NINGUNO) {
    const float objetivo = pilarAlIniciarEscape.color == Color::ROJO
                               ? -ajustes.objetivoX
                               : ajustes.objetivoX;
    const float correccionAdelante = constrain(
        KP_CAMARA * (pilarAlIniciarEscape.xCamara - objetivo),
        -ANGULO_GUIA_RETROCESO_GRADOS, ANGULO_GUIA_RETROCESO_GRADOS);

    // En reversa se invierte la correccion: asi el frente se aleja del mismo
    // lado donde la camara vio el pilar, en vez de regresar hacia el.
    return constrain(
        static_cast<float>(SERVO_CENTRO_GRADOS) -
            static_cast<float>(DIRECCION_SERVO_CAMARA) * correccionAdelante,
        static_cast<float>(SERVO_CENTRO_GRADOS -
                           SERVO_CORRECCION_MAXIMA_GRADOS),
        static_cast<float>(SERVO_CENTRO_GRADOS +
                           SERVO_CORRECCION_MAXIMA_GRADOS));
  }

  if (fabsf(giroAcumuladoGrados) < UMBRAL_SENTIDO_RETROCESO_GRADOS) {
    return SERVO_CENTRO_GRADOS;
  }

  const int8_t signoRonda = giroAcumuladoGrados > 0.0f ? 1 : -1;
  const float correccion =
      static_cast<float>(signoRonda * DIRECCION_SERVO_REVERSA) *
      ANGULO_GUIA_RETROCESO_GRADOS;

  return constrain(
      static_cast<float>(SERVO_CENTRO_GRADOS) + correccion,
      static_cast<float>(SERVO_CENTRO_GRADOS -
                         SERVO_CORRECCION_MAXIMA_GRADOS),
      static_cast<float>(SERVO_CENTRO_GRADOS +
                         SERVO_CORRECCION_MAXIMA_GRADOS));
}

float anguloSalidaForzadaEscape() {
  int8_t sentidoGiro = signoGiroVueltas;
  if (sentidoGiro == 0 && fabsf(giroConteoVueltasGrados) > 1.0f) {
    sentidoGiro = giroConteoVueltasGrados > 0.0f ? 1 : -1;
  }

  // Si la ronda aun no fijo sentido, conserva el lado que indico la camara
  // al comenzar el escape. Como ultimo respaldo se toma giro a la izquierda.
  int8_t signoServo = 0;
  if (sentidoGiro != 0) {
    const bool giraIzquierda =
        sentidoGiro == SIGNO_BNO_GIRO_IZQUIERDA;
    signoServo = giraIzquierda ? -SIGNO_SERVO_DERECHA
                               : SIGNO_SERVO_DERECHA;
  } else if (pilarAlIniciarEscape.color != Color::NINGUNO) {
    const float objetivo = pilarAlIniciarEscape.color == Color::ROJO
                               ? -ajustes.objetivoX
                               : ajustes.objetivoX;
    const float correccion = constrain(
        KP_CAMARA * (pilarAlIniciarEscape.xCamara - objetivo),
        -ajustes.topeCamara, ajustes.topeCamara);
    const float anguloCamara =
        static_cast<float>(SERVO_CENTRO_GRADOS) +
        static_cast<float>(DIRECCION_SERVO_CAMARA) * correccion;
    signoServo = anguloCamara >= SERVO_CENTRO_GRADOS ? 1 : -1;
  } else {
    signoServo = -SIGNO_SERVO_DERECHA;
  }

  return static_cast<float>(SERVO_CENTRO_GRADOS) +
         static_cast<float>(signoServo) * ANGULO_SALIDA_FORZADA_GRADOS;
}

void iniciarSalidaForzadaEscape(unsigned long ahoraMs) {
  fase = Fase::SALIDA_FORZADA_ESCAPE;
  inicioFaseMs = ahoraMs;
  inicioSalidaForzadaEscapeMs = ahoraMs;
  confirmacionesObstaculoFrontal = 0;
  inicioFrenteDespejadoMs = 0;

  Serial.println();
  Serial.print("Limite de ");
  Serial.print(MAXIMO_RETROCESOS_CONSECUTIVOS);
  Serial.print(" retrocesos alcanzado: salida hacia delante, servo=");
  Serial.println(anguloSalidaForzadaEscape(), 0);
}

void actualizarReinicioRetrocesos(unsigned long ahoraMs) {
  if (retrocesosConsecutivos == 0) {
    inicioFrenteDespejadoMs = 0;
    return;
  }

  const bool navegacionNormal = fase == Fase::RODANDO ||
                                fase == Fase::BUSCANDO_ESTACIONAMIENTO ||
                                fase == Fase::AVANCE_FINAL_TRES_VUELTAS ||
                                fase == Fase::LOCALIZANDO_PRIMER_DELIMITADOR ||
                                fase == Fase::ESPERANDO_ENTRADA_HUECO ||
                                fase == Fase::AVANCE_LIBRE_ENTRADA;
  if (!navegacionNormal || !frenteDespejadoTrasEscape()) {
    inicioFrenteDespejadoMs = 0;
    return;
  }

  if (inicioFrenteDespejadoMs == 0) {
    inicioFrenteDespejadoMs = ahoraMs;
  } else if (ahoraMs - inicioFrenteDespejadoMs >=
             TIEMPO_DESPEJADO_REINICIAR_RETROCESOS_MS) {
    retrocesosConsecutivos = 0;
    inicioFrenteDespejadoMs = 0;
    Serial.println("Frente estable y despejado: contador de retrocesos reiniciado");
  }
}

bool frenteActivaEscape() {
  const bool obstaculoConfirmable =
      distanciaValida[INDICE_FRONTAL] &&
      distanciasMm[INDICE_FRONTAL] <= DISTANCIA_FRENTE_RETROCESO_MM;

  if (!obstaculoConfirmable) {
    confirmacionesObstaculoFrontal = 0;
    return false;
  }

  if (confirmacionesObstaculoFrontal < TRAMAS_CONFIRMAR_ESCAPE_FRONTAL) {
    ++confirmacionesObstaculoFrontal;
  }
  return confirmacionesObstaculoFrontal >=
         TRAMAS_CONFIRMAR_ESCAPE_FRONTAL;
}

bool fasePermiteEscapeFrontal() {
  return fase == Fase::RODANDO ||
         fase == Fase::BUSCANDO_ESTACIONAMIENTO ||
         fase == Fase::AVANCE_FINAL_TRES_VUELTAS ||
         fase == Fase::LOCALIZANDO_PRIMER_DELIMITADOR ||
         fase == Fase::ESPERANDO_ENTRADA_HUECO ||
         fase == Fase::AVANCE_LIBRE_ENTRADA;
}

bool frenteDespejadoTrasEscape() {
  return distanciaValida[INDICE_FRONTAL] &&
         distanciasMm[INDICE_FRONTAL] >=
             DISTANCIA_FRENTE_SALIDA_RETROCESO_MM;
}

// ====================================================================
//                    AJUSTES POR EL MONITOR SERIE
// ====================================================================

// Los demas parametros se pueden cambiar en caliente a 115200 baudios.
// Los dos Kp no aparecen como comandos: se cambian unicamente en la lista
// superior.
//
//   ver            muestra los valores actuales
//   tope 25        tope de correccion del pasillo, en grados
//   topecam 20     tope de correccion de la camara, en grados
//   objx 45        donde debe quedar el pilar, en escala OpenMV 0..100
//   vel 45         velocidad del motor, en por ciento
//   tel 0          calla la telemetria para poder escribir; tel 1 la vuelve
//   guardar        graba los valores actuales en la memoria del ESP32
//   reset          vuelve a los valores de compilacion, SIN guardar
//
// Los cambios son inmediatos pero VOLATILES: sin "guardar", el siguiente
// encendido vuelve a lo ultimo guardado. Eso es a proposito, para que un
// experimento malo no se quede pegado.
//
// Leer el serie no toca el camino de control: cambia parametros, no manda
// al servo. La regla de una escritura por ciclo sigue intacta.

void mostrarAjustes() {
  Serial.println("--- ajustes ---");
  Serial.print("  kp      ");
  Serial.print(KP_PASILLO, 3);
  Serial.println(" (se cambia en el codigo)");
  Serial.print("  kp cam  ");
  Serial.print(KP_CAMARA, 4);
  Serial.println(" (se cambia en el codigo)");
  Serial.print("  tope    ");
  Serial.println(ajustes.topePasillo, 1);
  Serial.print("  topecam ");
  Serial.println(ajustes.topeCamara, 1);
  Serial.print("  objx    ");
  Serial.println(ajustes.objetivoX, 0);
  Serial.print("  vel     ");
  Serial.println(ajustes.velocidad);
}

void cargarAjustes() {
  // En modo lectura, begin() falla si todavia no se ha guardado nada.
  if (!memoria.begin(NVS_ESPACIO, true)) {
    Serial.println("Sin ajustes guardados: se usan los de compilacion");
    return;
  }

  const uint8_t versionGuardada = memoria.getUChar("cfgver", 0);
  ajustes.topePasillo = memoria.getFloat("tope", TOPE_PASILLO_DEFECTO);
  ajustes.topeCamara = memoria.getFloat("topecam", TOPE_CAMARA_DEFECTO);
  ajustes.objetivoX = memoria.getFloat("objx", OBJETIVO_X_DEFECTO);
  ajustes.velocidad = memoria.getUChar("vel", VELOCIDAD_DEFECTO);
  memoria.end();

  if (versionGuardada < VERSION_AJUSTES_NVS) {
    // Solo sustituye el valor predeterminado viejo. Si se habia calibrado un
    // objetivo diferente por monitor serie, se respeta sin modificarlo.
    if (fabsf(ajustes.objetivoX - OBJETIVO_X_VERSION_ANTERIOR_V4) < 0.01f ||
        fabsf(ajustes.objetivoX - OBJETIVO_X_VERSION_ANTERIOR_V3) < 0.01f ||
        fabsf(ajustes.objetivoX - OBJETIVO_X_VERSION_ANTERIOR) < 0.01f ||
        fabsf(ajustes.objetivoX - OBJETIVO_X_VERSION_ANTERIOR_V2) < 0.01f ||
        fabsf(ajustes.objetivoX - OBJETIVO_X_VERSION_ANTERIOR_V1) < 0.01f) {
      ajustes.objetivoX = OBJETIVO_X_DEFECTO;
      Serial.println("Evasion estable restaurada automaticamente a objx=45");
    }

    if (memoria.begin(NVS_ESPACIO, false)) {
      memoria.putFloat("objx", ajustes.objetivoX);
      memoria.putUChar("cfgver", VERSION_AJUSTES_NVS);
      memoria.end();
    }
  }

  Serial.println("Ajustes cargados de la memoria del ESP32");
}

void guardarAjustes() {
  if (!memoria.begin(NVS_ESPACIO, false)) {
    Serial.println("ERROR: no se pudo abrir la memoria");
    return;
  }

  memoria.putFloat("tope", ajustes.topePasillo);
  memoria.putFloat("topecam", ajustes.topeCamara);
  memoria.putFloat("objx", ajustes.objetivoX);
  memoria.putUChar("vel", ajustes.velocidad);
  memoria.putUChar("cfgver", VERSION_AJUSTES_NVS);
  memoria.end();

  Serial.println("Ajustes guardados");
}

// True si la linea empieza con el nombre y un espacio. El espacio importa:
// sin el, "tope" se comeria "topecam".
bool leerComando(const char *linea, const char *nombre, float &valor) {
  const size_t largo = strlen(nombre);

  if (strncmp(linea, nombre, largo) != 0 || linea[largo] != ' ') {
    return false;
  }

  valor = atof(linea + largo + 1);
  return true;
}

void aplicarComando(const char *linea) {
  float valor = 0.0f;

  if (strcmp(linea, "ver") == 0) {
    mostrarAjustes();
    return;
  }

  if (strcmp(linea, "guardar") == 0) {
    guardarAjustes();
    return;
  }

  if (strcmp(linea, "reset") == 0) {
    ajustes = Ajustes();
    Serial.println("Valores de compilacion restaurados, sin guardar");
    mostrarAjustes();
    return;
  }

  if (leerComando(linea, "tel", valor)) {
    telemetriaActiva = valor != 0.0f;
    Serial.println(telemetriaActiva ? "Telemetria encendida"
                                    : "Telemetria apagada");
    return;
  }

  if (leerComando(linea, "tope", valor)) {
    ajustes.topePasillo = constrain(
        valor, 0.0f, static_cast<float>(SERVO_CORRECCION_MAXIMA_GRADOS));
  } else if (leerComando(linea, "topecam", valor)) {
    ajustes.topeCamara = constrain(
        valor, 0.0f, static_cast<float>(SERVO_CORRECCION_MAXIMA_GRADOS));
  } else if (leerComando(linea, "objx", valor)) {
    ajustes.objetivoX =
        constrain(valor, 0.0f, static_cast<float>(OPENMV_X_MAX));
  } else if (leerComando(linea, "vel", valor)) {
    ajustes.velocidad = static_cast<uint8_t>(constrain(valor, 0.0f, 100.0f));
  } else {
    Serial.print("No entendi: ");
    Serial.println(linea);
    Serial.println("ver | tope | topecam | objx | vel | tel | "
                   "guardar | reset");
    return;
  }

  mostrarAjustes();
}

void atenderSerial() {
  uint8_t atendidos = 0;

  while (Serial.available() > 0 && atendidos < SERIAL_BYTES_POR_CICLO) {
    ++atendidos;
    const char entrada = static_cast<char>(Serial.read());

    if (entrada == '\r') {
      continue;
    }

    if (entrada == '\n') {
      lineaSerial[largoLineaSerial] = '\0';
      if (largoLineaSerial > 0) {
        aplicarComando(lineaSerial);
      }
      largoLineaSerial = 0;
      continue;
    }

    if (largoLineaSerial < SERIAL_LINEA_BYTES - 1) {
      lineaSerial[largoLineaSerial++] =
          static_cast<char>(tolower(static_cast<unsigned char>(entrada)));
    }
  }
}

// ====================================================================
//                          TELEMETRIA
// ====================================================================

void imprimirTelemetria(float angulo, int velocidad) {
  Serial.print("fase ");
  switch (fase) {
    case Fase::SALIDA_AVANZAR:
      Serial.print("salida_avanza");
      break;
    case Fase::SALIDA_PAUSA_FRENTE:
      Serial.print("salida_pausa_frente");
      break;
    case Fase::SALIDA_RETROCEDER:
      Serial.print("salida_reversa");
      break;
    case Fase::SALIDA_ELEGIR_LADO:
      Serial.print("salida_elige_lado");
      break;
    case Fase::SALIDA_ORIENTAR_SERVO:
      Serial.print("salida_orienta");
      break;
    case Fase::SALIDA_GIRAR:
      Serial.print("salida_gira");
      break;
    case Fase::SALIDA_CENTRAR_INTERMEDIO:
      Serial.print("salida_centra_intermedio");
      break;
    case Fase::SALIDA_REVERSA_CENTRO_INTERMEDIA:
      Serial.print("salida_reversa_recta_2s");
      break;
    case Fase::SALIDA_ORIENTAR_SEGUNDO_GIRO:
      Serial.print("salida_orienta_segundo");
      break;
    case Fase::SALIDA_SEGUNDO_GIRO:
      Serial.print("salida_segundo_giro_2s");
      break;
    case Fase::SALIDA_RECUPERAR_RUMBO:
      Serial.print("salida_recupera_rumbo");
      break;
    case Fase::SALIDA_AVANCE_CONTRARIO:
      Serial.print("salida_avanza_contrario");
      break;
    case Fase::SALIDA_RETROCESO_GIRO:
      Serial.print("salida_retrocede_giro");
      break;
    case Fase::SALIDA_RETROCESO_CENTRO:
      Serial.print("salida_retrocede_centro");
      break;
    case Fase::ESPERANDO:
      Serial.print("espera");
      break;
    case Fase::RODANDO:
      Serial.print("rodando");
      break;
    case Fase::FRENANDO_ESCAPE_FRONTAL:
      Serial.print("frena_escape");
      break;
    case Fase::RETROCEDIENDO_ESCAPE_FRONTAL:
      Serial.print("reversa_escape");
      break;
    case Fase::ESPERANDO_DESPEJE_ESCAPE:
      Serial.print("escape_bloqueado");
      break;
    case Fase::FRENO_TRAS_ESCAPE:
      Serial.print("freno_tras_escape");
      break;
    case Fase::SALIDA_FORZADA_ESCAPE:
      Serial.print("salida_forzada_escape");
      break;
    case Fase::AVANCE_FINAL_TRES_VUELTAS:
      Serial.print("avance_final_3_vueltas");
      break;
    case Fase::FRENANDO_FIN_TRES_VUELTAS:
      Serial.print("freno_final_3_vueltas");
      break;
    case Fase::BUSCANDO_ESTACIONAMIENTO:
      Serial.print("busca_parking");
      break;
    case Fase::LOCALIZANDO_PRIMER_DELIMITADOR:
      Serial.print("busca_delimitador");
      break;
    case Fase::ESPERANDO_ENTRADA_HUECO:
      Serial.print("busca_hueco");
      break;
    case Fase::AVANCE_LIBRE_ENTRADA:
      Serial.print("libera_entrada");
      break;
    case Fase::ARCO_ENTRADA_FRONTAL:
      Serial.print("arco_frontal");
      break;
    case Fase::CONTRAARCO_ENTRADA_FRONTAL:
      Serial.print("contraarco_frontal");
      break;
    case Fase::CENTRANDO_EN_CAJON:
      Serial.print("centra_cajon");
      break;
    case Fase::FRENO_FINAL_ESTACIONAMIENTO:
      Serial.print("freno_final");
      break;
    case Fase::TERMINADO:
      Serial.print("fin");
      break;
  }

  Serial.print(" | pilar ");
  switch (pilar.color) {
    case Color::ROJO:
      Serial.print("rojo");
      break;
    case Color::VERDE:
      Serial.print("verde");
      break;
    default:
      Serial.print("-");
      break;
  }

  Serial.print(" x ");
  Serial.print(pilar.xCamara, 0);
  Serial.print(" area ");
  Serial.print(pilar.areaPx);
  Serial.print(" cam ");
  Serial.print(camaraLista ? "ok" : "-");
  Serial.print(" col ");
  Serial.print(colisionVisual);
  Serial.print(" negra ");
  Serial.print(paredNegraVisible ? 1 : 0);
  Serial.print(" park ");
  Serial.print(paredEstacionamientoVisible ? "si" : "-");
  Serial.print(" conf ");
  Serial.print(confirmacionesParedEstacionamiento);

  Serial.print(" | cm 90i ");
  Serial.print(distanciasMm[INDICE_IZQ_90] / 10);
  Serial.print(" 25i ");
  Serial.print(distanciasMm[INDICE_IZQ_25] / 10);
  Serial.print(" fr ");
  Serial.print(distanciasMm[INDICE_FRONTAL] / 10);
  Serial.print(" 25d ");
  Serial.print(distanciasMm[INDICE_DER_25] / 10);
  Serial.print(" 90d ");
  Serial.print(distanciasMm[INDICE_DER_90] / 10);

  Serial.print(" | servo ");
  Serial.print(angulo, 1);
  Serial.print(" motor ");
  Serial.print(velocidad);
  Serial.print(" | giro ");
  Serial.print(giroAcumuladoGrados, 0);
  Serial.print(" cuenta ");
  Serial.print(giroConteoVueltasGrados, 0);
  Serial.print(" obj ");
  Serial.print(giroObjetivoFinalGrados, 0);
  Serial.print(" parkmm ");
  Serial.print(signoGiroVueltas == 0 ? 0
                                     : distanciasMm[indiceParking90()]);
  Serial.print(" esquinas ");
  Serial.print(esquinas);
  Serial.print(" vueltas ");
  Serial.println(vueltasCompletadas);
}

// ====================================================================
//                          SETUP Y LOOP
// ====================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("ESP32_Obstaculos_OpenMV");

  cargarAjustes();
  mostrarAjustes();

  servoListo = iniciarServo();
  aplicarDireccion(SERVO_CENTRO_GRADOS);

  motorListo = iniciarMotor();
  aplicarMotor(0);

  iniciarEncoder();

  Wire.begin(PIN_SDA, PIN_SCL, FRECUENCIA_I2C_HZ);
  Wire.setTimeOut(TIMEOUT_I2C_MS);

  iniciarBno();
  iniciarCamara();

  enviarBrilloLeds(BRILLO_LEDS_PORCENTAJE);
  ultimoBrilloMs = millis();
  ultimoCicloMs = millis();

  Serial.println("Listo. Salida inicial de parking al dar potencia al motor.");
}

void loop() {
  const unsigned long ahoraMs = millis();

  if (ahoraMs - ultimoCicloMs < PERIODO_CICLO_MS) {
    return;
  }
  ultimoCicloMs = ahoraMs;

  // ---------------- 1. percepcion ----------------

  actualizarCamara(ahoraMs);

  const bool paqueteSensoresNuevo = leerDistancias(ahoraMs);
  if (paqueteSensoresNuevo) {
    ultimoPaqueteMs = ahoraMs;
    hayDistancias = true;
    actualizarClasificacionPasilloInicial();
  } else if (hayDistancias &&
             ahoraMs - ultimoPaqueteMs > TIMEOUT_SENSORES_MS) {
    hayDistancias = false;
    Serial.println("AVISO: sin paquete del Nano");
  }

  actualizarBno(ahoraMs);
  actualizarSalidaInicial(ahoraMs, paqueteSensoresNuevo);
  actualizarAvanceFinalTresVueltas(ahoraMs);

  // ---------------- 2. arranque de la ronda ----------------

  if (fase == Fase::ESPERANDO &&
      leerPulsosEncoder() >= PULSOS_PARA_ARRANCAR) {
    fase = Fase::RODANDO;
    inicioRondaMs = ahoraMs;
    giroAcumuladoGrados = 0.0f;
    giroConteoVueltasGrados = 0.0f;
    signoGiroVueltas = 0;
    esquinas = 0;
    muestrasConfirmacionEsquina = 0;
    vueltasCompletadas = 0;
    pulsosNetosInicioRonda = leerPulsosEncoderNetos();
    pulsosNetosVuelta1 = 0;
    pulsosNetosVuelta2 = 0;
    pulsosNetosVuelta3 = 0;
    pulsosNetosInicioAvanceFinal = 0;
    pulsosObjetivoAvanceFinal = 0;
    referenciaFinalEncoderValida = false;
    avanceFinalAcumuladoMs = 0;
    pausaDespuesObstaculoActiva = false;
    retrocesosConsecutivos = 0;
    inicioFrenteDespejadoMs = 0;
    ladoParking = 0;
    huecoParkingGuardado = false;
    confirmacionesObstaculoFrontal = 0;
    Serial.println("Ronda iniciada");
  }

  // ---------------- 3. escape por obstaculo frontal ----------------

  actualizarReinicioRetrocesos(ahoraMs);

  if (fasePermiteEscapeFrontal() && frenteActivaEscape()) {
    faseDespuesEscape = fase;
    pilarAlIniciarEscape = pilar;
    if (pilarAlIniciarEscape.color == Color::NINGUNO &&
        colisionVisual != 0) {
      pilarAlIniciarEscape.color = colorDeId(colisionVisual);
      pilarAlIniciarEscape.xCamara = 0.0f;
    }

    if (retrocesosConsecutivos >= MAXIMO_RETROCESOS_CONSECUTIVOS) {
      iniciarSalidaForzadaEscape(ahoraMs);
    } else {
      ++retrocesosConsecutivos;
      fase = Fase::FRENANDO_ESCAPE_FRONTAL;
      inicioFaseMs = ahoraMs;
      confirmacionesObstaculoFrontal = 0;
      inicioFrenteDespejadoMs = 0;
      Serial.print("S3 detecta obstaculo a ");
      Serial.print(distanciasMm[INDICE_FRONTAL] / 10);
      Serial.print(" cm: retroceso ");
      Serial.print(retrocesosConsecutivos);
      Serial.print("/");
      Serial.println(MAXIMO_RETROCESOS_CONSECUTIVOS);
    }
  }

  if (fase == Fase::FRENANDO_ESCAPE_FRONTAL &&
      ahoraMs - inicioFaseMs >= FRENO_ANTES_RETROCESO_MS) {
    fase = Fase::RETROCEDIENDO_ESCAPE_FRONTAL;
    inicioFaseMs = ahoraMs;
    inicioRetrocesoEscapeMs = ahoraMs;
    Serial.print("Escape frontal: reversa al ");
    Serial.print(VELOCIDAD_REVERSA_ESCAPE);
    Serial.print("%, servo=");
    Serial.println(anguloReversaEscape(), 0);
  }

  if (fase == Fase::RETROCEDIENDO_ESCAPE_FRONTAL) {
    if (frenteDespejadoTrasEscape()) {
      if (retrocesosConsecutivos >= MAXIMO_RETROCESOS_CONSECUTIVOS) {
        iniciarSalidaForzadaEscape(ahoraMs);
      } else {
        fase = Fase::FRENO_TRAS_ESCAPE;
        inicioFaseMs = ahoraMs;
        Serial.println("S3 despejado: freno antes de recuperar el avance");
      }
    } else if (ahoraMs - inicioRetrocesoEscapeMs >=
               MAXIMO_RETROCESO_ESCAPE_MS) {
      if (retrocesosConsecutivos >= MAXIMO_RETROCESOS_CONSECUTIVOS) {
        iniciarSalidaForzadaEscape(ahoraMs);
      } else {
        fase = Fase::FRENO_TRAS_ESCAPE;
        inicioFaseMs = ahoraMs;
        Serial.println("Retroceso terminado por tiempo; se intentara avanzar");
      }
    }
  }

  if (fase == Fase::ESPERANDO_DESPEJE_ESCAPE &&
      frenteDespejadoTrasEscape()) {
    fase = Fase::FRENO_TRAS_ESCAPE;
    inicioFaseMs = ahoraMs;
    Serial.println("S3 vuelve a estar libre: preparando avance");
  }

  if (fase == Fase::FRENO_TRAS_ESCAPE &&
      ahoraMs - inicioFaseMs >= FRENO_DESPUES_RETROCESO_MS) {
    fase = faseDespuesEscape;
    inicioFaseMs = ahoraMs;
    if (fase == Fase::AVANCE_LIBRE_ENTRADA) {
      pulsosInicioMovimientoParking = leerPulsosEncoder();
    }
    Serial.println("Escape terminado: continua control por camara/pasillo");
  }

  if (fase == Fase::SALIDA_FORZADA_ESCAPE &&
      ahoraMs - inicioSalidaForzadaEscapeMs >=
          DURACION_SALIDA_FORZADA_MS) {
    fase = faseDespuesEscape;
    inicioFaseMs = ahoraMs;
    inicioFrenteDespejadoMs = 0;
    confirmacionesObstaculoFrontal = 0;
    if (fase == Fase::AVANCE_LIBRE_ENTRADA) {
      pulsosInicioMovimientoParking = leerPulsosEncoder();
    }
    Serial.println(
        "Salida forzada terminada: no habra mas reversa hasta despejar S3");
  }

  // ---------------- 4. busqueda rosa durante la cuarta vuelta -------

  if (fase == Fase::BUSCANDO_ESTACIONAMIENTO) {
    if (confirmacionesParedEstacionamiento >=
        TRAMAS_PARED_ESTACIONAMIENTO) {
      guardarMedicionParking();
      fase = Fase::LOCALIZANDO_PRIMER_DELIMITADOR;
      inicioFaseMs = ahoraMs;
      reiniciarConfirmacionParking();
      Serial.print("Rosa confirmado y guardado x=");
      Serial.print(referenciaParkingXGuardada);
      Serial.print(" area=");
      Serial.print(areaParkingGuardada);
      if (huecoParkingGuardado) {
        Serial.print(" hueco_x=");
        Serial.print(huecoParkingXGuardado);
        Serial.print(" ancho_px=");
        Serial.print(anchoHuecoParkingGuardadoPx);
      } else {
        Serial.print(" hueco=no_visible");
      }
      Serial.print("; cajon al lado ");
      Serial.println(ladoParkingElegido() > 0 ? "derecho" : "izquierdo");
      Serial.println(
          "Rosa confirmado: se ignoran colores y se alinean las llantas");
    } else if (cuartaVueltaSinParkingCompleta(ahoraMs)) {
      iniciarFrenoFinalParking(
          ahoraMs, false,
          "ERROR: no se encontro rosa antes de completar la cuarta vuelta");
    }
  }

  if (fase == Fase::LOCALIZANDO_PRIMER_DELIMITADOR) {
    if (condicionParkingEstable(lecturaPrimerDelimitadorParking(), ahoraMs)) {
      distanciaPrimerDelimitadorMm = distanciasMm[indiceParking90()];
      fase = Fase::ESPERANDO_ENTRADA_HUECO;
      inicioFaseMs = ahoraMs;
      reiniciarConfirmacionParking();
      Serial.print("Primer delimitador junto al robot a ");
      Serial.print(distanciaPrimerDelimitadorMm);
      Serial.println(" mm; buscando su borde final");
    } else if (ahoraMs - inicioFaseMs >=
               MAXIMO_LOCALIZAR_DELIMITADOR_MS) {
      iniciarFrenoFinalParking(
          ahoraMs, false,
          "ERROR: OpenMV vio rosa pero el lateral no encontro delimitador");
    }
  }

  if (fase == Fase::ESPERANDO_ENTRADA_HUECO) {
    if (condicionParkingEstable(lecturaEntradaHuecoParking(), ahoraMs)) {
      fase = Fase::AVANCE_LIBRE_ENTRADA;
      inicioFaseMs = ahoraMs;
      pulsosInicioMovimientoParking = leerPulsosEncoder();
      Serial.println("Borde del rosa confirmado; alineando eje delantero");
    } else if (ahoraMs - inicioFaseMs >= MAXIMO_ESPERAR_HUECO_MS) {
      iniciarFrenoFinalParking(
          ahoraMs, false,
          "ERROR: no se encontro la entrada despues del delimitador rosa");
    }
  }

  if (fase == Fase::AVANCE_LIBRE_ENTRADA) {
    const unsigned long tiempoAlineandoMs = ahoraMs - inicioFaseMs;
    const uint32_t pulsosAlineando =
        leerPulsosEncoder() - pulsosInicioMovimientoParking;
    if (tiempoAlineandoMs >= AVANCE_MINIMO_ALINEAR_LLANTAS_MS &&
        pulsosAlineando >= PULSOS_MINIMOS_ALINEAR_LLANTAS) {
      giroBaseManiobraParkingGrados = giroAcumuladoGrados;
      fase = Fase::ARCO_ENTRADA_FRONTAL;
      inicioFaseMs = ahoraMs;
      Serial.println("Llantas alineadas por encoder: arco hacia el cajon");
    } else if (tiempoAlineandoMs >= MAXIMO_ALINEAR_LLANTAS_MS) {
      iniciarFrenoFinalParking(
          ahoraMs, false,
          "ERROR: el encoder no confirmo el avance para alinear las llantas");
    }
  }

  if (fase == Fase::ARCO_ENTRADA_FRONTAL) {
    if (arcoEntradaFrontalCompleto()) {
      fase = Fase::CONTRAARCO_ENTRADA_FRONTAL;
      inicioFaseMs = ahoraMs;
      Serial.println("Movimiento 2: contraarco para quedar paralelo");
    } else if (ahoraMs - inicioFaseMs >= MAXIMO_ARCO_PARKING_MS) {
      iniciarFrenoFinalParking(
          ahoraMs, false, "ERROR: timeout en arco de entrada al parking");
    }
  }

  if (fase == Fase::CONTRAARCO_ENTRADA_FRONTAL) {
    if (contraArcoFrontalCompleto()) {
      fase = Fase::CENTRANDO_EN_CAJON;
      inicioFaseMs = ahoraMs;
      pulsosInicioMovimientoParking = leerPulsosEncoder();
      Serial.println(
          "Movimiento 2 completo: avanza recto hasta quedar dentro");
    } else if (ahoraMs - inicioFaseMs >= MAXIMO_ARCO_PARKING_MS) {
      iniciarFrenoFinalParking(
          ahoraMs, false, "ERROR: timeout en contraarco de parking");
    }
  }

  if (fase == Fase::CENTRANDO_EN_CAJON) {
    const unsigned long tiempoCentrandoMs = ahoraMs - inicioFaseMs;
    const uint32_t pulsosCentrando =
        leerPulsosEncoder() - pulsosInicioMovimientoParking;
    if (distanciaValida[INDICE_FRONTAL] &&
        distanciasMm[INDICE_FRONTAL] <=
            DISTANCIA_FRONTAL_EMERGENCIA_PARKING_MM) {
      iniciarFrenoFinalParking(
          ahoraMs, false,
          "EMERGENCIA: freno dentro del estacionamiento antes de chocar");
    } else if (tiempoCentrandoMs >= AVANCE_MINIMO_DENTRO_PARKING_MS &&
        pulsosCentrando >= PULSOS_MINIMOS_DENTRO_PARKING &&
        distanciaValida[INDICE_FRONTAL] &&
        distanciasMm[INDICE_FRONTAL] <= MARGEN_FRONTAL_FINAL_PARKING_MM) {
      iniciarFrenoFinalParking(
          ahoraMs, true,
          "Robot dentro del estacionamiento: freno final");
    } else if (tiempoCentrandoMs >= MAXIMO_CENTRADO_PARKING_MS) {
      iniciarFrenoFinalParking(
          ahoraMs, false,
          "ERROR: S3 no encontro el delimitador final del estacionamiento");
    }
  }

  const bool entrandoAlParking =
      fase == Fase::ARCO_ENTRADA_FRONTAL ||
      fase == Fase::CONTRAARCO_ENTRADA_FRONTAL ||
      fase == Fase::CENTRANDO_EN_CAJON;
  if (entrandoAlParking &&
      (!distanciaValida[INDICE_FRONTAL] ||
       !distanciaValida[indiceParking25()])) {
    iniciarFrenoFinalParking(
        ahoraMs, false,
        "EMERGENCIA: sensor critico invalido durante el estacionamiento");
  }

  if (entrandoAlParking && fase != Fase::FRENO_FINAL_ESTACIONAMIENTO &&
      distanciaValida[INDICE_FRONTAL] &&
      distanciasMm[INDICE_FRONTAL] <=
          DISTANCIA_FRONTAL_EMERGENCIA_PARKING_MM) {
    iniciarFrenoFinalParking(
        ahoraMs, false, "EMERGENCIA: delimitador demasiado cerca de S3");
  }

  if (entrandoAlParking && fase != Fase::FRENO_FINAL_ESTACIONAMIENTO &&
      lateralParkingDemasiadoCerca()) {
    iniciarFrenoFinalParking(
        ahoraMs, false,
        "EMERGENCIA: pilar rosa demasiado cerca del sensor lateral");
  }

  if (fase == Fase::FRENO_FINAL_ESTACIONAMIENTO &&
      ahoraMs - inicioFaseMs >= FRENO_FINAL_PARKING_MS) {
    fase = Fase::TERMINADO;
    if (estacionamientoExitoso) {
      Serial.print("*** ESTACIONAMIENTO FRONTAL TERMINADO | tiempo ");
      Serial.print((ahoraMs - inicioRondaMs) / 1000.0f, 1);
      Serial.println(" s ***");
    } else {
      Serial.println("Maniobra cancelada de forma segura");
    }
  }

  if (fase == Fase::FRENANDO_FIN_TRES_VUELTAS &&
      ahoraMs - inicioFaseMs >= FRENO_FINAL_TRES_VUELTAS_MS) {
    fase = Fase::TERMINADO;
    Serial.println(
        "*** TRES VUELTAS: ROBOT DETENIDO EN EL PUNTO DE SALIDA ***");
  }

  actualizarPausaDespuesObstaculo(ahoraMs);

  // ---------------- 4. un angulo y una velocidad ----------------

  float angulo = SERVO_CENTRO_GRADOS;
  int velocidad = 0;

  if (fase == Fase::TERMINADO) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = salidaInicialCancelada ? ORDEN_FRENO_ACTIVO : 0;
  } else if (fase == Fase::SALIDA_PAUSA_FRENTE) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = ORDEN_FRENO_ACTIVO;
  } else if (fase == Fase::SALIDA_ELEGIR_LADO) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = 0;
  } else if (fase == Fase::SALIDA_ORIENTAR_SERVO) {
    angulo = SERVO_CENTRO_GRADOS +
             ladoSalidaParking * SIGNO_SERVO_DERECHA *
                 SERVO_CORRECCION_MAXIMA_GRADOS;
    velocidad = 0;
  } else if (fase == Fase::SALIDA_CENTRAR_INTERMEDIO ||
             fase == Fase::SALIDA_ORIENTAR_SEGUNDO_GIRO) {
    angulo = fase == Fase::SALIDA_CENTRAR_INTERMEDIO
                 ? SERVO_CENTRO_GRADOS
                 : SERVO_CENTRO_GRADOS +
                       ladoSalidaParking * SIGNO_SERVO_DERECHA *
                           SERVO_CORRECCION_MAXIMA_GRADOS;
    velocidad = ORDEN_FRENO_ACTIVO;
  } else if (fase == Fase::FRENANDO_FIN_TRES_VUELTAS) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = ORDEN_FRENO_ACTIVO;
  } else if (fase == Fase::FRENO_FINAL_ESTACIONAMIENTO) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = ORDEN_FRENO_ACTIVO;
  } else if (!hayDistancias) {
    // Sin sensores el robot no sabe donde esta el pasillo. Frenar es la
    // unica respuesta honesta.
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = fase == Fase::SALIDA_AVANZAR ? ORDEN_FRENO_ACTIVO : 0;
  } else if (!distanciaValida[INDICE_FRONTAL]) {
    // Tras agotar la retencion de 125 ms, un S3 invalido nunca significa
    // camino libre. Se espera una lectura real antes de volver a moverse.
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = fase == Fase::SALIDA_AVANZAR ? ORDEN_FRENO_ACTIVO : 0;
  } else if (fase == Fase::SALIDA_AVANZAR) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = VELOCIDAD_AVANCE_SALIDA;
  } else if (fase == Fase::SALIDA_RETROCEDER) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = -static_cast<int>(VELOCIDAD_REVERSA_SALIDA);
  } else if (fase == Fase::SALIDA_GIRAR) {
    // Servo al maximo hacia el lado con mas espacio; avanza.
    angulo = SERVO_CENTRO_GRADOS +
             ladoSalidaParking * SIGNO_SERVO_DERECHA *
                 SERVO_CORRECCION_MAXIMA_GRADOS;
    velocidad = VELOCIDAD_GIRO_SALIDA;
  } else if (fase == Fase::SALIDA_REVERSA_CENTRO_INTERMEDIA) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = -static_cast<int>(VELOCIDAD_REVERSA_SALIDA);
  } else if (fase == Fase::SALIDA_SEGUNDO_GIRO) {
    angulo = SERVO_CENTRO_GRADOS +
             ladoSalidaParking * SIGNO_SERVO_DERECHA *
                 SERVO_CORRECCION_MAXIMA_GRADOS;
    velocidad = VELOCIDAD_GIRO_SALIDA;
  } else if (fase == Fase::SALIDA_RECUPERAR_RUMBO) {
    const float error =
        normalizarDelta(rumboSalidaActualGrados - rumboBaseSalidaGrados);
    const int8_t ladoCorreccion =
        error * SIGNO_BNO_GIRO_IZQUIERDA > 0.0f ? 1 : -1;
    angulo = SERVO_CENTRO_GRADOS +
             ladoCorreccion * SIGNO_SERVO_DERECHA *
                 SERVO_CORRECCION_MAXIMA_GRADOS;
    velocidad = VELOCIDAD_GIRO_SALIDA;
  } else if (fase == Fase::SALIDA_AVANCE_CONTRARIO) {
    // Servo al maximo hacia el lado contrario al elegido; sigue avanzando.
    angulo = SERVO_CENTRO_GRADOS -
             ladoSalidaParking * SIGNO_SERVO_DERECHA *
                 SERVO_CORRECCION_MAXIMA_GRADOS;
    velocidad = VELOCIDAD_GIRO_SALIDA;
  } else if (fase == Fase::SALIDA_RETROCESO_GIRO) {
    // Servo de vuelta al maximo hacia el lado elegido originalmente;
    // ahora retrocede.
    angulo = SERVO_CENTRO_GRADOS +
             ladoSalidaParking * SIGNO_SERVO_DERECHA *
                 SERVO_CORRECCION_MAXIMA_GRADOS;
    velocidad = -static_cast<int>(VELOCIDAD_REVERSA_SALIDA);
  } else if (fase == Fase::SALIDA_RETROCESO_CENTRO) {
    // Servo centrado a 90 grados; retrocede para terminar de salir.
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = -static_cast<int>(VELOCIDAD_REVERSA_SALIDA);
  } else if (fase == Fase::FRENANDO_ESCAPE_FRONTAL) {
    // Aprovecha la pausa para colocar las ruedas antes de invertir el motor.
    angulo = anguloReversaEscape();
    velocidad = 0;
  } else if (fase == Fase::RETROCEDIENDO_ESCAPE_FRONTAL) {
    angulo = anguloReversaEscape();
    velocidad = -static_cast<int>(VELOCIDAD_REVERSA_ESCAPE);
  } else if (fase == Fase::ESPERANDO_DESPEJE_ESCAPE ||
             fase == Fase::FRENO_TRAS_ESCAPE) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = 0;
  } else if (fase == Fase::SALIDA_FORZADA_ESCAPE) {
    angulo = anguloSalidaForzadaEscape();
    velocidad = VELOCIDAD_SALIDA_FORZADA;
  } else if (pausaDespuesObstaculoActiva &&
             (fase == Fase::RODANDO ||
              fase == Fase::BUSCANDO_ESTACIONAMIENTO ||
              fase == Fase::AVANCE_FINAL_TRES_VUELTAS)) {
    // OpenMV sigue leyendo. Si ya vio el siguiente pilar, coloca el servo
    // para su rebase; si no, conserva el control normal del pasillo.
    angulo = pilar.color != Color::NINGUNO ? anguloPorCamara()
                                           : anguloPorPasillo();
    velocidad = 0;
  } else if (fase == Fase::BUSCANDO_ESTACIONAMIENTO) {
    // La cuarta vuelta se conduce igual que las anteriores. El rosa se busca
    // en paralelo y no altera ni la velocidad ni el control de los pilares.
    angulo = pilar.color != Color::NINGUNO ? anguloPorCamara()
                                           : anguloPorPasillo();
    velocidad = ajustes.velocidad;
  } else if (fase == Fase::LOCALIZANDO_PRIMER_DELIMITADOR ||
             fase == Fase::ESPERANDO_ENTRADA_HUECO) {
    angulo = anguloPegadoParedExterior();
    velocidad = distanciaValida[indiceParking90()]
                    ? VELOCIDAD_APROXIMACION_PARKING
                    : 0;
  } else if (fase == Fase::AVANCE_LIBRE_ENTRADA) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = VELOCIDAD_APROXIMACION_PARKING;
  } else if (fase == Fase::ARCO_ENTRADA_FRONTAL) {
    angulo = anguloHaciaEspacioParking();
    velocidad = VELOCIDAD_MANIOBRA_PARKING;
  } else if (fase == Fase::CONTRAARCO_ENTRADA_FRONTAL) {
    angulo = anguloFueraDelEspacioParking();
    velocidad = VELOCIDAD_MANIOBRA_PARKING;
  } else if (fase == Fase::CENTRANDO_EN_CAJON) {
    angulo = SERVO_CENTRO_GRADOS;
    velocidad = VELOCIDAD_MANIOBRA_PARKING;
  } else if (pilar.color != Color::NINGUNO) {
    angulo = anguloPorCamara();
    velocidad = ajustes.velocidad;
  } else {
    angulo = anguloPorPasillo();
    velocidad = ajustes.velocidad;
  }

  const bool navegandoVueltas = fase == Fase::RODANDO ||
                                fase == Fase::BUSCANDO_ESTACIONAMIENTO ||
                                fase == Fase::AVANCE_FINAL_TRES_VUELTAS;
  if (navegandoVueltas && velocidad > VELOCIDAD_ALERTA_COLISION &&
      alertaColisionVisualActiva(ahoraMs)) {
    velocidad = VELOCIDAD_ALERTA_COLISION;
  }

  // ---------------- 5. las dos unicas escrituras ----------------

  aplicarDireccion(angulo);
  aplicarMotor(velocidad);

  // ---------------- 6. mantenimiento ----------------

  atenderSerial();
  enviarModoEstacionamiento(ahoraMs);

  if (ahoraMs - ultimoBrilloMs >= INTERVALO_BRILLO_MS) {
    ultimoBrilloMs = ahoraMs;
    enviarBrilloLeds(BRILLO_LEDS_PORCENTAJE);
  }

  if (telemetriaActiva &&
      ahoraMs - ultimaTelemetriaMs >= INTERVALO_TELEMETRIA_MS) {
    ultimaTelemetriaMs = ahoraMs;
    imprimirTelemetria(angulo, velocidad);
  }
}
