# ============================================================
# OPENMV FIRMWARE 6
# Detección de pilares rojos y verdes
#
# Cambio respecto a la 5: separacion de MAGENTA y ROJO por la media del
# canal B del blob, no por umbral de pixel. Ver el bloque CLASIFICADOR
# DE COLOR POR MEDIA. La version 5 queda intacta en color_corner.py.
#
# UART:
# ID, X, Y, AREA, ROI, COLISION, PARED_NEGRA,
# PARKING, X_PARKING, AREA_PARKING, HUECO, X_HUECO, ANCHO_HUECO
#
# X siempre conserva la esquina inferior original del color:
# - Verde: esquina inferior izquierda.
# - Rojo: esquina inferior derecha.
# Se transmite mapeada entre -100 y 100. El ESP32 convierte ese
# rango al recorrido angular seguro del servo.
#
# En ROI_HIGH el robot sigue directamente esa esquina.
# En ROI_LOW esa misma esquina sirve para medir la separacion
# respecto al ancho del robot (ROI de colision).
# ROI: 0 = ninguna, 1 = alta, 2 = baja.
#
# ID:
# 0 = Sin detección
# 3 = Verde
# 5 = Rojo
# ============================================================

import sensor
import time
import micropython

from machine import UART

try:
    import binascii
except ImportError:
    import ubinascii as binascii


# ============================================================
# CONFIGURACIÓN GENERAL
# ============================================================

SHOW_DEBUG = True
DRAW_BOXES = True
DRAW_TEXT = True

# Superpone la trayectoria que el ESP32 esta ordenando al robot. El ESP32
# devuelve por UART una linea "T,servo,motor,S1,S2,S3,S4,S5". Esta ayuda es
# solamente visual: no modifica la deteccion ni los datos de control enviados.
SHOW_PATH_PREVIEW = False

PRINT_UART_DATA = True
PRINT_FPS = False

# Imprimir la imagen completa en Base64 reduce mucho los FPS.
PRINT_BASE64_FRAME = False
BASE64_JPEG_QUALITY = 35

# ============================================================
# EXPOSICION
#
# Se quito el valor fijo de 30000 us. Ahora la elige la camara mirando
# la escena real, que es lo que hace el script por defecto del IDE y da
# una imagen con mas contraste y colores mas saturados.
#
# CAMERA_EXPOSURE_US
#   None   -> la elige el automatico y se congela ese valor.
#   30000  -> el valor fijo de antes, por si hay que volver.
#
# CAMERA_LOCK_SETTINGS
#   True   -> ganancia, balance de blancos y exposicion se congelan
#             despues del ajuste inicial. Los colores se quedan
#             quietos durante toda la carrera.
#   False  -> los tres siguen en automatico todo el rato, igual que el
#             script por defecto. La imagen se adapta sola a la luz,
#             pero los umbrales dejan de significar lo mismo de un
#             cuadro al siguiente: al entrar un pilar rojo grande el
#             automatico reacciona y mueve la L de TODO lo demas.
#
# Los umbrales actuales separan rojo y magenta principalmente por B:
# magenta termina en 5 y rojo empieza en 10. Si cambia la exposicion,
# conviene volver a comprobar esa banda muerta con los objetos reales.
# ============================================================

CAMERA_EXPOSURE_US = None

CAMERA_LOCK_SETTINGS = True

# Tiempo que se le da al automatico para converger antes de congelar.
CAMERA_AUTO_SETTLE_MS = 800

# True:
# X se envía normalizada entre -100 y 100.
#
# False:
# X se envía en píxeles entre 0 y 319.
X_AS_NORMALIZED = True

# Permite utilizar los dos umbrales de cada color.
USE_MULTI_RANGES = True


# ============================================================
# VISTA PREVIA DE TRAYECTORIA
# ============================================================

SERVO_CENTER_DEG = 90
SERVO_MAX_CORRECTION_DEG = 40

# Cambiar a -1 si la curva aparece al lado contrario del giro fisico.
PATH_SCREEN_DIRECTION = 1

# La proyeccion es intencionalmente calibrable: no pretende medir distancias
# metricas hasta realizar una calibracion de camara sobre el robot.
PATH_NEAR_Y = 236
PATH_FAR_Y = 86
PATH_NEAR_HALF_WIDTH_PX = 34
PATH_FAR_HALF_WIDTH_PX = 7
PATH_MAX_BEND_PX = 105
PATH_SEGMENTS = 12

TELEMETRY_TIMEOUT_MS = 350
PATH_FRONT_DANGER_MM = 250
PATH_SIDE_DANGER_MM = 180


# ============================================================
# UMBRALES ACTIVOS
# ============================================================

USE_RED_1 = True
USE_RED_2 = False

USE_GREEN_1 = True
USE_GREEN_2 = False

USE_MAGENTA_1 = True
USE_MAGENTA_2 = False


# ============================================================
# REGIONES DE INTERÉS
# Formato: (x, y, ancho, alto)
# ============================================================

ROI_HIGH = (60, 60, 200, 50)
ROI_LOW = (70, 110, 180, 130)
ROI_SIDE_LEFT = (0, 110, 70, 100)
ROI_SIDE_RIGHT = (250, 110, 70, 100)

COLLISION_ROI = (125, 165, 80, 10)

# ROI de calibracion, siempre centrada horizontalmente.
# Cambiar unicamente CENTER_ROI_CENTER_Y para subirla o bajarla:
# 0 es la parte superior y 239 la parte inferior de la imagen.
SHOW_CENTER_ROI = True
CENTER_ROI_WIDTH = 6
CENTER_ROI_HEIGHT = 10
CENTER_ROI_CENTER_Y = 105
# Cada pixel con luminosidad LAB menor o igual a este valor cuenta como negro.
# Subirlo acepta tonos mas claros; bajarlo hace la deteccion mas estricta.
WALL_BLACK_L_MAX = 35
# Porcentaje minimo de pixeles negros dentro del ROI para enviar Wall=1.
WALL_BLACK_MIN_PERCENT = 30

CENTER_ROI = (
    (320 - CENTER_ROI_WIDTH) // 2,
    max(
        0,
        min(
            240 - CENTER_ROI_HEIGHT,
            CENTER_ROI_CENTER_Y - CENTER_ROI_HEIGHT // 2
        )
    ),
    CENTER_ROI_WIDTH,
    CENTER_ROI_HEIGHT
)


# ============================================================
# ZONAS MUERTAS LATERALES
#
# Rechazan falsas detecciones producidas por lineas de pared/esquina. El
# margen crece hacia abajo para seguir la perspectiva de la pista.
# ============================================================

USE_CORNER_DEAD_ZONES = False
CORNER_DEAD_ZONE_TOP_Y = 80
CORNER_DEAD_ZONE_BOTTOM_Y = 214
CORNER_DEAD_ZONE_FAR_MARGIN_PX = 18
CORNER_DEAD_ZONE_NEAR_MARGIN_PX = 58


# ============================================================
# CONFIGURACIÓN UART
# ============================================================

UART_BUS = 3
UART_BAUD = 19200

uart = UART(
    UART_BUS,
    UART_BAUD,
    bits=8,
    parity=None,
    stop=1,
    timeout_char=10
)

micropython.alloc_emergency_exception_buf(200)


# ============================================================
# UMBRALES LAB
#
# Formato:
# (L mínimo, L máximo,
#  A mínimo, A máximo,
#  B mínimo, B máximo)
# ============================================================
# cerca
# lejos

# Bloque rojo real: media aproximada LAB (44, 61, 29).
TH_RED_1 = (15, 85, 40, 100, 10, 75)
TH_RED_2 = (20, 100, 36, 127, 16, 127)

# Bloque verde real: media (55, -29, 4), mediana (54, -29, 0).
TH_GREEN_1 = (35, 75, -50, -15, -20, 25)
TH_GREEN_2 = (34, 72, -74, -18, -128, 127)

# Rectangulo rosa real: media LAB (63, 50, -19), mediana (63, 53, -21).
# Incluye la lectura anterior (77, 42, -27) y excluye la mesa blanca. B termina
# en 5 para conservar una banda muerta antes del rojo, que empieza en B=10.
TH_MAGENTA_1 = (35, 90, 30, 70, -50, 5)
TH_MAGENTA_2 = (15, 80, 20, 110, -90, -20)


def active_by_flags(threshold_list, flags):
    active_thresholds = []

    for threshold, enabled in zip(threshold_list, flags):
        if enabled and threshold is not None:
            active_thresholds.append(threshold)

    if not active_thresholds:
        active_thresholds.append(threshold_list[0])

    if USE_MULTI_RANGES:
        return active_thresholds

    return [active_thresholds[0]]


THS_RED = active_by_flags(
    [TH_RED_1, TH_RED_2],
    [USE_RED_1, USE_RED_2]
)

THS_GREEN = active_by_flags(
    [TH_GREEN_1, TH_GREEN_2],
    [USE_GREEN_1, USE_GREEN_2]
)

THS_MAGENTA = active_by_flags(
    [TH_MAGENTA_1, TH_MAGENTA_2],
    [USE_MAGENTA_1, USE_MAGENTA_2]
)


# ============================================================
# CLASIFICADOR DE COLOR POR MEDIA
#
# Los umbrales por pixel actuales dejan una banda muerta entre magenta y rojo:
#
#   TH_RED_1      A 40..100   B  10..75
#   TH_MAGENTA_1  A 30..70    B -50..5
#
# No existe interseccion en B: el rosa termina en 5 y el rojo empieza en 10.
# La media del blob se conserva como una segunda comprobacion frente a bordes,
# sombras y arrastre de movimiento.
#
# La media sobre cientos de pixeles no tiene ese problema. Un pilar rojo
# entero da una media de B claramente positiva y una pared magenta entera
# una media claramente negativa, aunque sus bordes se mezclen. Asi que la
# decision se toma sobre el blob ya formado, no sobre cada pixel:
#
#   los umbrales siguen anchos  ->  el blob se forma completo
#   la media de B decide        ->  el blob se clasifica bien
#
# Cuesta una llamada a get_statistics por blob candidato, uno o dos por
# cuadro. No hace falta ningun find_blobs extra, que era justo el coste
# que habia que evitar.
# ============================================================

# Activo para separar el rosa de estacionamiento de los pilares rojos usando
# la media del canal B. SHOW_LAB_MEANS permite comprobar la calibracion real.
USE_B_MEAN_CLASSIFIER = True

# Dibuja en pantalla la media LAB de lo que se esta detectando. Es lo que
# permite rellenar las constantes de abajo sin escribir nada.
SHOW_LAB_MEANS = True

# Lo mismo pero por consola del IDE, para poder copiar y pegar los datos.
# Cuesta FPS: dejalo en False sobre el robot.
PRINT_LAB_MEANS = False

# Tamaño de la muestra, en porcentaje de la caja del blob, centrada en
# el centro de masa.
#
# Con LAB_SAMPLE_USE_THRESHOLD encendido este numero casi da igual y
# conviene que sea grande: el fondo ya se descarta por color, asi que
# cuanta mas caja se abarque, mas pixeles del objeto entran en la media
# y mas estable sale. Con el filtro apagado hay que apretarlo a 40 o 50
# para que no se cuele tapete.
LAB_SAMPLE_PERCENT = 80

# Medir SOLO los pixeles con color, ignorando el fondo.
#
# Este es el arreglo del problema de la dilucion. La caja de un blob
# poco denso es casi todo tapete: con D:28% medido en pista, un pilar
# que de verdad tiene A~73 y B~38 se leia como A:35 B:19, porque el
# tapete (A:-3 B:0) se promediaba dentro. Esa media no sirve ni para
# clasificar ni para calibrar, y es peor justo donde mas importa: en
# los pilares recortados por el borde, que son los de densidad baja.
#
# get_statistics acepta un umbral y entonces promedia unicamente los
# pixeles que caen dentro de el. Pasandole un umbral que cubre el rojo
# Y el magenta a la vez, el tapete y la pared negra quedan fuera de la
# cuenta y la media describe el objeto, no su encuadre.
#
# No es circular: el umbral de abajo no distingue rojo de magenta, solo
# separa "tiene color" de "es fondo". Quien decide el color sigue
# siendo la media de B.
LAB_SAMPLE_USE_THRESHOLD = True

# Cualquier cosa saturada, calida o fria. A minimo 20 deja fuera el
# tapete blanco y la pared negra, que rondan A cero. La L y la B van
# abiertas del todo a proposito: son justo lo que se quiere medir.
LAB_SAMPLE_THRESHOLD = (0, 100, 20, 127, -128, 127)

# --- AFINAR CON LA OPENMV Y LA HOJA REAL ---------------------------
#
# Mide la media de B de los dos objetos a varias distancias y anota el
# peor caso de cada uno:
#
#   pared magenta  ->  el valor de B MAS ALTO que llegues a ver
#   pilar rojo     ->  el valor de B MAS BAJO que llegues a ver
#
# Pon cada constante un poco por dentro de su peor caso y deja hueco
# entre las dos. Ese hueco es una banda muerta, la misma idea que ya usa
# MIN_W_OVER_H_PARKING con la proporcion: en la tierra de nadie no se
# acepta ninguna de las dos cosas, que es mejor que acertar a medias.
#
# El delimitador real dio B medio=-27, mediana=-32 y maximo observado=7.
# La media completa se acepta hasta 0; el rojo comienza en 12, dejando una
# banda muerta amplia para no confundir un pilar rojo con la pared rosa.
PARKING_B_MEAN_MAX = 0
PILLAR_B_MEAN_MIN = 12


# ============================================================
# PARÁMETROS DE DETECCIÓN
# ============================================================

AREA_TH_RG_PX = 30
AREA_TH_RG_PCT = 0

MIN_H_PILLAR = 12
MIN_H_OVER_W = 1.20
MAX_TILT_DEG = 35
MIN_PILLAR_AREA = 60

# Densidad minima: pixeles del blob divididos entre el area de su caja.
#
# Un pilar es un cuerpo solido y llena casi toda su caja (85% o mas). Una
# linea del tapete cruza la caja en diagonal y deja las dos esquinas
# vacias, asi que llena bastante menos (alrededor del 45%). Es lo unico que
# separa de verdad las dos cosas: la franja naranja pasa el filtro de
# proporcion alto/ancho y tambien el de inclinacion.
#
# El porcentaje real se dibuja en pantalla como D:xx% para poder ajustarlo.
# Subirlo es mas estricto; bajarlo acepta pilares mas recortados.
MIN_PILLAR_DENSITY_PERCENT = 60

# Seguimiento de un pilar recortado cuando entra a un lateral. No se exige que
# antes haya aparecido en el centro. Se acepta en el primer cuadro para que no
# exista un corte al cruzar la frontera; el minimo de pixeles filtra el ruido.
MIN_SIDE_PILLAR_PIXELS = 100
SIDE_CONFIRM_FRAMES = 2

# Densidad minima en los laterales.
#
# El lateral se salta a proposito la proporcion, la altura y la
# inclinacion, porque un pilar recortado por el borde sale bajo y ancho
# y esos tres filtros lo perderian siempre.
#
# La densidad es distinta: NO se rompe al recortar. Es pixeles entre
# area de la caja, y cuando el borde corta el pilar la caja se encoge
# con el, asi que un pilar solido medio recortado sigue llenando su
# caja igual de bien. Es el unico filtro de forma que sobrevive al
# recorte, y por eso se puede exigir aqui sin perder pilares buenos.
#
# Lo que si cae son las fusiones con el cable y el muro, los trozos de
# pared y las lineas del tapete: cosas que dejan la caja medio vacia.
# Medido en pista, un blob de basura daba 28%.
#
# Mas flojo que el 60% del centro porque en los bordes el ojo de pez
# curva el pilar y su caja crece un poco respecto a lo que llena.
# El valor real se dibuja como D:xx%, asi que se ajusta mirandolo.
MIN_SIDE_PILLAR_DENSITY_PERCENT = 40


# Evita confundir una línea horizontal con un pilar.
MAX_LINE_H = 12
MAX_LINE_W_OVER_H = 2.0

# Parámetros de find_blobs.
PIX_TH = 8
AREA_TH = 8
MERGE = True
MARGIN = 5

# Valor enviado cuando no se detecta una esquina válida.
CORNER_SENTINEL = -1


# ============================================================
# ESTACIONAMIENTO
#
# Las paredes son magenta, pero con esta luz el magenta y el rojo se
# confunden. La forma si los separa sin ambiguedad:
#
#   Pilar:   5 cm de ancho x 10 cm de alto -> alto/ancho = 2.0
#   Pared:  20 cm de ancho x 10 cm de alto -> ancho/alto = 2.0
#
# Son cuatro veces distintos en proporcion, asi que el color solo sirve
# para encontrar candidatos y la decision la toma la forma.
# ============================================================

DETECT_PARKING = True

# Estado de la busqueda de estacionamiento MIENTRAS no diga nada el ESP32.
#
# Va en True a proposito. El ESP32 manda "P,0" en cuanto arranca, antes de que
# el robot se mueva, asi que sobre el robot el comportamiento es el que se
# quiere: apagada durante las vueltas y encendida al terminarlas.
#
# En el banco, con el IDE y sin ESP32, no llega nada y se queda encendida:
# asi se puede calibrar TH_MAGENTA sin tocar ninguna constante ni acordarse
# de devolverla despues.
#
# Que se quede encendida por un cable suelto no rompe nada: el ESP32 ignora
# los campos de estacionamiento hasta que el mismo decide que toca aparcar.
PARKING_DETECTION_DEFAULT = True

# La calibracion rosa se usa para encontrar la pared y la FORMA sigue siendo
# una segunda proteccion independiente:
#
#   Pilar:   5 cm de ancho x 10 cm de alto -> alto/ancho = 2.0
#   Pared:  20 cm de ancho x 10 cm de alto -> ancho/alto = 2.0
#
# Se buscan candidatos tanto con rojo como con magenta; el clasificador B y
# la proporcion deciden si el candidato es pared o pilar.
PARKING_USES_MAGENTA_THRESHOLD = True

# Las paredes se ven desde mas lejos que los pilares, asi que el
# estacionamiento usa su propia ROI. Sus cuatro dimensiones se pueden
# modificar directamente. Formato: (X, Y, ANCHO, ALTO), dentro de 320 x 240.
# Estos valores conservan el rectangulo magenta original completo.
PARKING_ROI_X = 0
PARKING_ROI_Y = 70
PARKING_ROI_WIDTH = 320
PARKING_ROI_HEIGHT = 170

# Un blob ancho es pared; uno alto es pilar. Entre ambas proporciones queda una
# tierra de nadie donde no se acepta ninguna de las dos cosas.
# Medido en pista: una pared vista de frente da 2.0, pero en escorzo baja
# a 1.5. Con el umbral en 1.50 quedaba justo en el filo y cualquier
# angulo un poco peor la perdia, asi que se usa 1.40 con margen.
#
# Los pilares exigen alto/ancho >= 1.35, o sea ancho/alto <= 0.74. Entre 0.74
# y 1.40 no se acepta nada: el color y la forma deben estar de acuerdo.
MIN_W_OVER_H_PARKING = 1.40

# Ancho minimo en pixeles. Protege del caso en que un pilar cercano queda
# recortado por el borde de la ROI y su trozo visible parece ancho: la
# pared es cuatro veces mas ancha que el pilar, asi que siempre supera
# este valor con holgura.
MIN_PARKING_W = 20

MIN_PARKING_H = 6
MIN_PARKING_AREA = 120

# Un pilar cercano recortado por el borde superior de PARKING_ROI deja un
# trozo visible ancho y bajo que puede parecer una pared. Si el blob toca
# ese borde no se sabe su altura real, asi que no se acepta como pared.
# Las paredes miden 10 cm y se apoyan en el piso: quedan abajo en la
# imagen, no arriba, asi que esta guarda no les quita nada.
PARKING_REJECT_ROI_TOP_EDGE = True

# Las paredes tambien son cuerpos solidos: descarta lineas del tapete.
MIN_PARKING_DENSITY_PERCENT = 55

# Si los dos rectangulos rosas aparecen separados en el mismo cuadro, se
# envia tambien el centro y el ancho en pixeles del hueco que dejan. La
# separacion minima evita interpretar dos fragmentos del mismo rectangulo.
MIN_PARKING_GAP_PX = 12

# TH_RED y TH_MAGENTA se entregan juntos a una sola llamada de find_blobs; no
# se agrega una busqueda extra por cuadro.


# ============================================================
# INICIALIZACIÓN DE LA CÁMARA
# ============================================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

# Ajuste automático inicial.
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)

sensor.skip_frames(time=CAMERA_AUTO_SETTLE_MS)

if CAMERA_LOCK_SETTINGS:
    # Bloqueo de los valores para estabilizar los colores.
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)

    if CAMERA_EXPOSURE_US is None:
        # Sin valor: congela el que acaba de elegir el automatico.
        sensor.set_auto_exposure(False)
    else:
        sensor.set_auto_exposure(
            False,
            exposure_us=CAMERA_EXPOSURE_US
        )

# Deja ver que exposicion quedo puesta. Es el numero que habria que
# fijar como constante el dia que se quiera dejar de depender de lo
# que el automatico vea al encender.
try:
    print(
        "Exposicion aplicada: %s us" % sensor.get_exposure_us()
    )
except Exception:
    # No todas las versiones de firmware exponen el getter. No es
    # motivo para no arrancar.
    pass

sensor.skip_frames(time=300)

clock = time.clock()


# Ultima orden realmente aplicada por el ESP32. Los valores iniciales dibujan
# una trayectoria recta y amarilla hasta recibir la primera telemetria.
telemetry_servo_deg = SERVO_CENTER_DEG
telemetry_motor_percent = 0
telemetry_distances_mm = [2000, 2000, 2000, 2000, 2000]
telemetry_last_ms = 0
telemetry_rx_line = ""

# Lo enciende el ESP32 al terminar las vueltas.
parking_detection_enabled = PARKING_DETECTION_DEFAULT

# Estado local para confirmar el color lateral durante varios cuadros.
# No cambia el protocolo UART.
side_candidate_id = 0
side_candidate_zone = 0
side_candidate_frames = 0


# ============================================================
# DIMENSIONES DE LA IMAGEN
# ============================================================

IMG_W = 320
IMG_H = 240


# ============================================================
# ROI UNIFICADA PARA LOS PILARES
# ============================================================

pillars_top = min(
    ROI_HIGH[1],
    ROI_LOW[1]
)

pillars_bottom = max(
    ROI_HIGH[1] + ROI_HIGH[3],
    ROI_LOW[1] + ROI_LOW[3]
)

PILLARS_ROI = (
    0,
    pillars_top,
    IMG_W,
    pillars_bottom - pillars_top
)


# ============================================================
# ROI DEL ESTACIONAMIENTO
#
# Se arma con los cuatro valores configurables de la seccion ESTACIONAMIENTO.
# ============================================================

PARKING_ROI = (
    PARKING_ROI_X,
    PARKING_ROI_Y,
    PARKING_ROI_WIDTH,
    PARKING_ROI_HEIGHT
)


# ============================================================
# FUNCIONES AUXILIARES
# ============================================================

def best_blob_area(blobs):
    if not blobs:
        return None

    return max(
        blobs,
        key=lambda blob: blob.area
    )


def pillar_area_ok(blob, roi):
    if blob is None:
        return False

    ok_pixels = blob.area >= AREA_TH_RG_PX

    if AREA_TH_RG_PCT > 0:
        roi_area = max(
            1,
            roi[2] * roi[3]
        )

        percentage = (
            blob.area * 100
        ) // roi_area

        ok_percentage = percentage >= AREA_TH_RG_PCT

    else:
        ok_percentage = True

    return ok_pixels and ok_percentage


def corner_lower_left(blob):
    x, y, width, height = blob.rect

    corner_x = x
    corner_y = y + height - 1

    return int(corner_x), int(corner_y)


def corner_lower_right(blob):
    x, y, width, height = blob.rect

    corner_x = x + width - 1
    corner_y = y + height - 1

    return int(corner_x), int(corner_y)


def pillar_reference_point(blob, detected_id):
    if detected_id == 3:
        return corner_lower_left(blob)

    return corner_lower_right(blob)


def point_in_roi(x, y, roi):
    roi_x, roi_y, roi_width, roi_height = roi

    inside_x = roi_x <= x < roi_x + roi_width
    inside_y = roi_y <= y < roi_y + roi_height

    return inside_x and inside_y


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


def clamp_i16(value):
    if value < -32768:
        return -32768

    if value > 32767:
        return 32767

    return int(value)


def normalize_x100(x_pixel):
    # Mapeo lineal con extremos exactos:
    # 0 px -> -100, centro -> 0, 319 px -> 100.
    x_pixel = clamp(
        int(x_pixel),
        0,
        IMG_W - 1
    )

    normalized_x = (
        (
            x_pixel * 200
            + (IMG_W - 1) // 2
        )
        // (IMG_W - 1)
    ) - 100

    return clamp(
        normalized_x,
        -100,
        100
    )


def blob_threshold_index(blob):
    code = blob.code

    if code == 0:
        return 0

    # Verifica si solamente existe un bit activo.
    if (code & (code - 1)) == 0:
        index = 1

        while ((code >> (index - 1)) & 1) == 0:
            index += 1

        return index

    return 0


def radians_to_degrees(radians):
    if radians is None:
        return 0.0

    return radians * 57.29578


def looks_like_horizontal_line(blob):
    if blob.h <= MAX_LINE_H:
        return True

    width_over_height = (
        blob.w
        / max(1, blob.h)
    )

    return width_over_height >= MAX_LINE_W_OVER_H


def blob_density_percent(blob):
    # En OpenMV blob.area es el area de la CAJA (ancho por alto) y
    # blob.pixels es cuantos pixeles pertenecen de verdad al blob.
    if blob is None:
        return 0

    box_area = max(1, int(blob.w) * int(blob.h))

    try:
        filled_pixels = int(blob.pixels)
    except Exception:
        # Sin el dato no se puede juzgar: se da por lleno para no
        # descartar pilares buenos por una diferencia de firmware.
        return 100

    return (filled_pixels * 100) // box_area


def blob_attribute(blob, name, fallback):
    # Segun la version del firmware estos datos son atributos o metodos.
    # El resto del programa los usa como atributos, asi que aqui basta
    # con cubrir el caso de que no existan.
    value = getattr(blob, name, None)

    if value is None:
        return fallback

    if callable(value):
        try:
            return value()
        except Exception:
            return fallback

    return value


def blob_lab_means(img, blob):
    # Media LAB del INTERIOR del blob, no de su caja completa.
    # Devuelve (L, A, B) o None si no se pudo medir.
    if blob is None:
        return None

    box_x = int(blob.rect[0])
    box_y = int(blob.rect[1])
    box_w = int(blob.rect[2])
    box_h = int(blob.rect[3])

    # Centro de MASA, no centro de la caja.
    #
    # Cuando el blob viene recortado por el borde de la imagen o sale
    # inclinado, el centro de su caja puede caer fuera del objeto y la
    # muestra se llena de tapete. Eso no rompe la medida, la DILUYE:
    # tira la media de B hacia el color del fondo y hace que un pilar
    # rojo de un valor mucho mas flojo del que tiene en realidad.
    # Se nota justo en los blobs con densidad baja, los que el HUD
    # muestra con una D pequeña.
    center_x = int(
        blob_attribute(blob, "cx", box_x + box_w // 2)
    )
    center_y = int(
        blob_attribute(blob, "cy", box_y + box_h // 2)
    )

    half_w = max(1, (box_w * LAB_SAMPLE_PERCENT) // 200)
    half_h = max(1, (box_h * LAB_SAMPLE_PERCENT) // 200)

    sample_x = max(box_x, center_x - half_w)
    sample_y = max(box_y, center_y - half_h)

    sample_roi = (
        sample_x,
        sample_y,
        max(1, min(box_x + box_w, center_x + half_w) - sample_x),
        max(1, min(box_y + box_h, center_y + half_h) - sample_y)
    )

    stats = None

    if LAB_SAMPLE_USE_THRESHOLD:
        try:
            stats = img.get_statistics(
                roi=sample_roi,
                thresholds=[LAB_SAMPLE_THRESHOLD]
            )
        except Exception:
            # Firmware que no acepta el argumento: se mide sin filtrar.
            stats = None

        # Si el umbral no cogio ni un pixel, el resultado sale en cero y
        # no significa nada. Vale mas una media diluida que un cero.
        if stats is not None:
            if (int(stats.l_mean) == 0
                    and int(stats.a_mean) == 0
                    and int(stats.b_mean) == 0):
                stats = None

    if stats is None:
        try:
            stats = img.get_statistics(roi=sample_roi)
        except Exception:
            # Sin el dato no se puede juzgar. Quien llama decide que
            # hacer; aqui no se inventa un color.
            return None

    return (
        int(stats.l_mean),
        int(stats.a_mean),
        int(stats.b_mean)
    )


def lab_means_string(lab):
    if lab is None:
        return "L:-- A:-- B:--"

    return "L:%d A:%d B:%d" % lab


def blob_b_mean(img, blob):
    lab = blob_lab_means(img, blob)

    if lab is None:
        return None

    return lab[2]


def keep_red_pillar_color_candidates(img, blobs):
    # El listado rojo tambien contiene candidatos magenta para que la misma
    # pasada sirva al parking. Se clasifican ANTES de elegir el blob mayor:
    # asi una pared magenta grande no puede ocultar un pilar rojo valido que
    # aparezca simultaneamente. La lista original se conserva para parking.
    if not USE_B_MEAN_CLASSIFIER:
        return blobs

    accepted = []

    for blob in blobs:
        b_mean = blob_b_mean(img, blob)

        # Igual que antes, si el firmware no permite medir la media manda la
        # forma: el clasificador solo rechaza colores que pudo comprobar.
        if b_mean is None or b_mean >= PILLAR_B_MEAN_MIN:
            accepted.append(blob)

    return accepted


def is_vertical_pillar(blob):
    if blob is None:
        return False

    if blob.area < MIN_PILLAR_AREA:
        return False

    if blob.h < MIN_H_PILLAR:
        return False

    # Descarta las lineas del tapete: cruzan su caja en diagonal y la
    # dejan medio vacia, mientras que un pilar la llena.
    if blob_density_percent(blob) < MIN_PILLAR_DENSITY_PERCENT:
        return False

    height_over_width = (
        blob.h
        / max(1, blob.w)
    )

    if height_over_width < MIN_H_OVER_W:
        return False

    try:
        rotation_degrees = (
            abs(
                radians_to_degrees(
                    blob.rotation
                )
            )
            % 180.0
        )

        tilt_from_zero = abs(rotation_degrees)
        tilt_from_vertical = abs(rotation_degrees - 90.0)

        tilt = min(
            tilt_from_zero,
            tilt_from_vertical
        )

        if tilt > MAX_TILT_DEG:
            return False

    except Exception:
        # La proporción alto/ancho permanece como filtro.
        pass

    return True


def blob_in_center_pillar_roi(blob, detected_id):
    reference_x, reference_y = pillar_reference_point(
        blob,
        detected_id
    )

    return (
        point_in_roi(reference_x, reference_y, ROI_HIGH)
        or point_in_roi(reference_x, reference_y, ROI_LOW)
    )


def rectangle_intersection_area(rectangle, roi):
    rect_x, rect_y, rect_w, rect_h = rectangle
    roi_x, roi_y, roi_w, roi_h = roi

    left = max(rect_x, roi_x)
    top = max(rect_y, roi_y)
    right = min(rect_x + rect_w, roi_x + roi_w)
    bottom = min(rect_y + rect_h, roi_y + roi_h)

    if right <= left or bottom <= top:
        return 0

    return (right - left) * (bottom - top)


def side_zone_for_blob(blob, detected_id):
    # Se usa toda la caja y no solamente la esquina de referencia. Cuando un
    # pilar cruza la frontera entre dos ROI, su esquina puede estar en el
    # centro aunque buena parte del cuerpo ya este recortada por un lateral.
    left_overlap = rectangle_intersection_area(
        blob.rect,
        ROI_SIDE_LEFT
    )
    right_overlap = rectangle_intersection_area(
        blob.rect,
        ROI_SIDE_RIGHT
    )

    if left_overlap <= 0 and right_overlap <= 0:
        return 0

    if left_overlap >= right_overlap:
        return -1

    if right_overlap > 0:
        return 1

    return 0


def blob_color_pixels(blob):
    try:
        return int(blob.pixels)
    except Exception:
        return int(blob.area)


def keep_valid_center_pillars(blobs, detected_id):
    valid_blobs = []

    for blob in blobs:
        valid_area = pillar_area_ok(
            blob,
            PILLARS_ROI
        )

        valid_shape = is_vertical_pillar(blob)

        horizontal_line = looks_like_horizontal_line(
            blob
        )

        if (
            valid_area
            and valid_shape
            and not horizontal_line
            and blob_in_center_pillar_roi(blob, detected_id)
        ):
            valid_blobs.append(blob)

    return valid_blobs


def keep_side_color_candidates(blobs, detected_id):
    accepted = []

    for blob in blobs:
        if blob_color_pixels(blob) < MIN_SIDE_PILLAR_PIXELS:
            continue

        # Ver MIN_SIDE_PILLAR_DENSITY_PERCENT: es el unico filtro de
        # forma que se le puede pedir a un blob recortado.
        if blob_density_percent(blob) < MIN_SIDE_PILLAR_DENSITY_PERCENT:
            continue

        if side_zone_for_blob(blob, detected_id) == 0:
            continue

        accepted.append(blob)

    return accepted


def clear_side_tracking():
    global side_candidate_id
    global side_candidate_zone
    global side_candidate_frames

    side_candidate_id = 0
    side_candidate_zone = 0
    side_candidate_frames = 0


def update_side_tracking(
    red_blobs_raw,
    green_blobs_raw
):
    global side_candidate_id
    global side_candidate_zone
    global side_candidate_frames

    side_red = best_blob_area(
        keep_side_color_candidates(red_blobs_raw, 5)
    )
    side_green = best_blob_area(
        keep_side_color_candidates(green_blobs_raw, 3)
    )

    # Si ya se esta confirmando un color, se conserva mientras siga visible.
    # De lo contrario se inicia con el blob lateral de mayor area.
    if side_candidate_id == 5 and side_red:
        side_blob = side_red
        candidate_id = 5
    elif side_candidate_id == 3 and side_green:
        side_blob = side_green
        candidate_id = 3
    elif side_green and (
        not side_red
        or side_green.area >= side_red.area
    ):
        side_blob = side_green
        candidate_id = 3
    elif side_red:
        side_blob = side_red
        candidate_id = 5
    else:
        side_blob = None
        candidate_id = 0

    if side_blob is None:
        clear_side_tracking()
        return None, None

    candidate_zone = side_zone_for_blob(
        side_blob,
        candidate_id
    )

    if (
        candidate_id == side_candidate_id
        and candidate_zone == side_candidate_zone
    ):
        side_candidate_frames += 1
    else:
        side_candidate_id = candidate_id
        side_candidate_zone = candidate_zone
        side_candidate_frames = 1

    if side_candidate_frames < SIDE_CONFIRM_FRAMES:
        return None, None

    if candidate_id == 3:
        return None, side_blob

    return side_blob, None


def corner_dead_zone_margin(y_pixel):
    y_pixel = clamp(
        int(y_pixel),
        CORNER_DEAD_ZONE_TOP_Y,
        CORNER_DEAD_ZONE_BOTTOM_Y
    )

    y_span = max(
        1,
        CORNER_DEAD_ZONE_BOTTOM_Y - CORNER_DEAD_ZONE_TOP_Y
    )

    margin_span = (
        CORNER_DEAD_ZONE_NEAR_MARGIN_PX
        - CORNER_DEAD_ZONE_FAR_MARGIN_PX
    )

    return (
        CORNER_DEAD_ZONE_FAR_MARGIN_PX
        + (
            (y_pixel - CORNER_DEAD_ZONE_TOP_Y)
            * margin_span
        ) // y_span
    )


def blob_in_corner_dead_zone(blob, detected_id):
    if not USE_CORNER_DEAD_ZONES:
        return False

    if detected_id == 3:
        reference_x, reference_y = corner_lower_left(blob)
    else:
        reference_x, reference_y = corner_lower_right(blob)

    margin = corner_dead_zone_margin(reference_y)

    return (
        reference_x < margin
        or reference_x >= IMG_W - margin
    )


def remove_corner_dead_zone_blobs(blobs, detected_id):
    accepted = []

    for blob in blobs:
        if not blob_in_corner_dead_zone(blob, detected_id):
            accepted.append(blob)

    return accepted


def is_parking_wall(img, blob, use_color=True):
    if blob is None:
        return False

    if blob.area < MIN_PARKING_AREA:
        return False

    if blob.h < MIN_PARKING_H:
        return False

    if blob.w < MIN_PARKING_W:
        return False

    if PARKING_REJECT_ROI_TOP_EDGE:
        if blob.rect[1] <= PARKING_ROI[1] + 1:
            return False

    width_over_height = (
        blob.w
        / max(1, blob.h)
    )

    if width_over_height < MIN_W_OVER_H_PARKING:
        return False

    if blob_density_percent(blob) < MIN_PARKING_DENSITY_PERCENT:
        return False

    # Confirmacion por color. La forma ya dijo que es ancho como una
    # pared; esto descarta que sea un pilar rojo recortado que por el
    # angulo ha salido ancho. Una pared magenta tiene la media de B
    # negativa y un pilar rojo la tiene positiva.
    #
    # Si no se pudo medir se deja pasar: mandan la forma y la densidad,
    # igual que en la version 5. El color solo quita, nunca añade.
    if USE_B_MEAN_CLASSIFIER and use_color:
        b_mean = blob_b_mean(img, blob)

        if b_mean is not None and b_mean > PARKING_B_MEAN_MAX:
            return False

    return True


def keep_valid_parking(img, blobs, use_color=True):
    accepted = []

    for blob in blobs:
        if is_parking_wall(img, blob, use_color):
            accepted.append(blob)

    return accepted


def find_parking_wall_candidate(img, red_blobs_raw):
    # Igual que find_parking_scene pero SIN las dos puertas que dependen
    # del estado del robot: ni la busqueda que enciende el ESP32, ni el
    # filtro de color.
    #
    # Existe solo para MEDIR. Su resultado no se envia por UART ni
    # decide nada; alimenta el "Pared B" del HUD. Sin esto el numero
    # saldria siempre en "--" mientras el ESP32 este conectado, que es
    # justo cuando hace falta calibrar, y tambien se ocultaria el valor
    # del blob que el clasificador acabara de rechazar, que es
    # precisamente el que interesa ver.
    if not DETECT_PARKING:
        return None

    return best_blob_area(
        keep_valid_parking(img, red_blobs_raw, False)
    )


def parking_thresholds():
    if PARKING_USES_MAGENTA_THRESHOLD:
        return THS_RED + THS_MAGENTA

    return THS_RED


# Las paredes salen de la MISMA lista de blobs rojos que los pilares: son
# las anchas. Una sola busqueda sirve para las dos cosas, asi que la camara
# se queda en 3 llamadas a find_blobs por cuadro, las del script original.
def parking_gap_from_walls(walls):
    best_gap = None
    best_score = -1

    for first_index in range(len(walls)):
        for second_index in range(first_index + 1, len(walls)):
            first = walls[first_index]
            second = walls[second_index]

            first_center = int(first.rect[0] + first.rect[2] // 2)
            second_center = int(second.rect[0] + second.rect[2] // 2)

            if first_center <= second_center:
                left_wall = first
                right_wall = second
            else:
                left_wall = second
                right_wall = first

            gap_left = int(left_wall.rect[0] + left_wall.rect[2])
            gap_right = int(right_wall.rect[0])
            gap_width = gap_right - gap_left

            if gap_width < MIN_PARKING_GAP_PX:
                continue

            score = int(left_wall.area + right_wall.area)
            if score > best_score:
                best_score = score
                best_gap = (
                    (gap_left + gap_right) // 2,
                    gap_width,
                    left_wall,
                    right_wall
                )

    return best_gap


def find_parking_scene(img, red_blobs_raw):
    if not DETECT_PARKING or not parking_detection_enabled:
        return None, None

    walls = keep_valid_parking(img, red_blobs_raw)
    return best_blob_area(walls), parking_gap_from_walls(walls)


def find_red_green_pillars(img):
    # El rojo se busca sobre PARKING_ROI, que contiene por completo a
    # PILLARS_ROI: la misma pasada da los pilares (blobs altos) y las
    # paredes del estacionamiento (blobs anchos). Los blobs que caen por
    # encima de PILLARS_ROI se descartan solos mas adelante, cuando se
    # comprueba que su esquina caiga en ROI_HIGH o ROI_LOW.
    red_blobs = img.find_blobs(
        parking_thresholds(),
        roi=PARKING_ROI,
        pixels_threshold=PIX_TH,
        area_threshold=AREA_TH,
        merge=MERGE,
        margin=MARGIN
    ) or []

    green_blobs = img.find_blobs(
        THS_GREEN,
        roi=PILLARS_ROI,
        pixels_threshold=PIX_TH,
        area_threshold=AREA_TH,
        merge=MERGE,
        margin=MARGIN
    ) or []

    # Los blobs sin filtrar se conservan para el estacionamiento. Para los
    # pilares rojos se elimina primero el magenta, sin otra busqueda de imagen.
    red_blobs_raw = red_blobs
    green_blobs_raw = green_blobs

    red_pillar_blobs_raw = keep_red_pillar_color_candidates(
        img,
        red_blobs_raw
    )

    red_blobs = keep_valid_center_pillars(
        red_pillar_blobs_raw,
        5
    )
    green_blobs = keep_valid_center_pillars(
        green_blobs,
        3
    )

    red_blobs = remove_corner_dead_zone_blobs(
        red_blobs,
        5
    )
    green_blobs = remove_corner_dead_zone_blobs(
        green_blobs,
        3
    )

    center_red = best_blob_area(red_blobs)
    center_green = best_blob_area(green_blobs)

    side_red, side_green = update_side_tracking(
        red_pillar_blobs_raw,
        green_blobs_raw
    )

    selected_red = best_blob_area(
        [blob for blob in (center_red, side_red) if blob]
    )

    selected_green = best_blob_area(
        [blob for blob in (center_green, side_green) if blob]
    )

    return selected_red, selected_green, red_blobs_raw


def send_uart_payload(payload):
    message = "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" % tuple(payload)

    uart.write(message.encode("ascii"))

    if PRINT_UART_DATA:
        print(
            "UART TX: %s"
            % message.strip()
        )


def reinitialize_uart():
    try:
        uart.init(
            UART_BAUD,
            bits=8,
            parity=None,
            stop=1,
            timeout_char=10
        )

    except Exception as error:
        print(
            "UART_INIT_ERR:",
            error
        )


def print_base64_frame(img):
    try:
        # copy=True evita comprimir la imagen original.
        jpeg_image = img.compress(
            quality=BASE64_JPEG_QUALITY,
            copy=True
        )

        encoded_frame = binascii.b2a_base64(
            bytes(jpeg_image)
        ).decode("utf-8").strip()

        print(
            "**FRAME**:"
            + encoded_frame
        )

    except Exception as error:
        print(
            "IMG_ERR:",
            error
        )


def parse_parking_command(fields):
    # "P,1" enciende la busqueda del estacionamiento; "P,0" la apaga.
    global parking_detection_enabled

    if len(fields) < 2:
        return

    try:
        enabled = int(fields[1]) != 0
    except Exception:
        return

    if enabled != parking_detection_enabled:
        print(
            "ESTACIONAMIENTO: deteccion "
            + ("ENCENDIDA" if enabled else "apagada")
        )

    parking_detection_enabled = enabled


def parse_telemetry_line(line):
    global telemetry_servo_deg
    global telemetry_motor_percent
    global telemetry_distances_mm
    global telemetry_last_ms

    fields = line.split(",")

    if fields[0] == "P":
        parse_parking_command(fields)
        return

    if len(fields) != 8 or fields[0] != "T":
        return

    try:
        servo_deg = int(fields[1])
        motor_percent = int(fields[2])
        distances_mm = [
            int(fields[3]),
            int(fields[4]),
            int(fields[5]),
            int(fields[6]),
            int(fields[7])
        ]
    except Exception:
        return

    telemetry_servo_deg = clamp(servo_deg, 0, 180)
    telemetry_motor_percent = clamp(motor_percent, -100, 100)
    telemetry_distances_mm = distances_mm
    telemetry_last_ms = time.ticks_ms()


def read_esp32_telemetry():
    global telemetry_rx_line

    try:
        available = uart.any()
        if not available:
            return

        incoming = uart.read(available)
        if not incoming:
            return

        for byte_value in incoming:
            if isinstance(byte_value, int):
                character = chr(byte_value)
            else:
                character = byte_value

            if character == "\n":
                if telemetry_rx_line:
                    parse_telemetry_line(telemetry_rx_line)
                telemetry_rx_line = ""
            elif character != "\r":
                if len(telemetry_rx_line) < 95:
                    telemetry_rx_line += character
                else:
                    telemetry_rx_line = ""

    except Exception as error:
        print("UART_RX_ERR:", error)


def trajectory_warning_state(now_ms):
    fresh = (
        telemetry_last_ms != 0
        and time.ticks_diff(now_ms, telemetry_last_ms)
        <= TELEMETRY_TIMEOUT_MS
    )

    if not fresh:
        return 0  # Sin telemetria: amarillo.

    correction = telemetry_servo_deg - SERVO_CENTER_DEG
    front_mm = telemetry_distances_mm[2]
    danger = 0 < front_mm <= PATH_FRONT_DANGER_MM

    if correction < -4:
        # Giro hacia la izquierda: S1/S2.
        side_mm = min(
            telemetry_distances_mm[0],
            telemetry_distances_mm[1]
        )
        danger = danger or 0 < side_mm <= PATH_SIDE_DANGER_MM
    elif correction > 4:
        # Giro hacia la derecha: S4/S5.
        side_mm = min(
            telemetry_distances_mm[3],
            telemetry_distances_mm[4]
        )
        danger = danger or 0 < side_mm <= PATH_SIDE_DANGER_MM

    return 2 if danger else 1


def trajectory_point(progress, lateral_offset_px):
    # progress=0: parte baja/frente del robot; progress=1: horizonte.
    correction = clamp(
        telemetry_servo_deg - SERVO_CENTER_DEG,
        -SERVO_MAX_CORRECTION_DEG,
        SERVO_MAX_CORRECTION_DEG
    )
    steering = correction / float(SERVO_MAX_CORRECTION_DEG)

    # Curva cuadratica: estable cerca del robot y mas visible a distancia.
    bend = (
        PATH_SCREEN_DIRECTION
        * steering
        * PATH_MAX_BEND_PX
        * progress
        * progress
    )

    half_width_scale = 1.0 - progress
    perspective_offset = (
        PATH_FAR_HALF_WIDTH_PX
        + (
            PATH_NEAR_HALF_WIDTH_PX
            - PATH_FAR_HALF_WIDTH_PX
        ) * half_width_scale
    )

    x = (IMG_W // 2) + bend + lateral_offset_px * perspective_offset
    y = PATH_NEAR_Y + (PATH_FAR_Y - PATH_NEAR_Y) * progress

    return (
        clamp(int(x), 0, IMG_W - 1),
        clamp(int(y), 0, IMG_H - 1)
    )


def draw_trajectory_preview(img):
    now_ms = time.ticks_ms()
    warning_state = trajectory_warning_state(now_ms)

    if warning_state == 2:
        path_color = (255, 40, 0)
        state_text = "RIESGO"
    elif warning_state == 1:
        path_color = (0, 255, 255)
        state_text = "RUTA OK"
    else:
        path_color = (255, 220, 0)
        state_text = "SIN TELEMETRIA"

    previous_left = trajectory_point(0.0, -1.0)
    previous_center = trajectory_point(0.0, 0.0)
    previous_right = trajectory_point(0.0, 1.0)

    for index in range(1, PATH_SEGMENTS + 1):
        progress = index / float(PATH_SEGMENTS)
        current_left = trajectory_point(progress, -1.0)
        current_center = trajectory_point(progress, 0.0)
        current_right = trajectory_point(progress, 1.0)

        img.draw_line(
            previous_left + current_left,
            color=path_color,
            thickness=2
        )
        img.draw_line(
            previous_right + current_right,
            color=path_color,
            thickness=2
        )
        img.draw_line(
            previous_center + current_center,
            color=path_color,
            thickness=3
        )

        if (index % 3) == 0:
            img.draw_line(
                current_left + current_right,
                color=path_color,
                thickness=1
            )

        previous_left = current_left
        previous_center = current_center
        previous_right = current_right

    img.draw_string(
        (196, 2),
        state_text,
        color=path_color
    )
    img.draw_string(
        (196, 12),
        "S:%d M:%d%%" % (
            telemetry_servo_deg,
            telemetry_motor_percent
        ),
        color=path_color
    )


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

while True:
    clock.tick()

    # UART es full-duplex: se recibe la orden aplicada por el ESP32 sin
    # interrumpir las tramas de vision que se envian mas abajo.
    read_esp32_telemetry()

    img = sensor.snapshot()

    # Sensor optico frontal: porcentaje real de pixeles negros dentro del ROI.
    # Esta salida no decide por si sola una esquina; el ESP32 exige tambien
    # que S3 confirme la distancia configurada.
    center_roi_stats = img.get_statistics(roi=CENTER_ROI)
    center_roi_l_mean = int(center_roi_stats.l_mean)

    # a_mean y b_mean ya venian calculados dentro de center_roi_stats:
    # leerlos no cuesta nada y convierte el cuadrito central en un
    # medidor de color que se puede apuntar a lo que sea.
    center_roi_a_mean = int(center_roi_stats.a_mean)
    center_roi_b_mean = int(center_roi_stats.b_mean)
    center_roi_black_blobs = img.find_blobs(
        [(0, WALL_BLACK_L_MAX, -128, 127, -128, 127)],
        roi=CENTER_ROI,
        pixels_threshold=1,
        area_threshold=1,
        merge=False
    ) or []
    center_roi_black_pixels = 0
    for black_blob in center_roi_black_blobs:
        center_roi_black_pixels += int(black_blob.pixels)
    center_roi_total_pixels = CENTER_ROI_WIDTH * CENTER_ROI_HEIGHT
    center_roi_black_percent = (
        center_roi_black_pixels * 100
    ) // center_roi_total_pixels
    wall_black_detected = (
        1
        if center_roi_black_percent >= WALL_BLACK_MIN_PERCENT
        else 0
    )

    # --------------------------------------------------------
    # DETECCIÓN DE PILARES
    # --------------------------------------------------------

    red_blob, green_blob, red_blobs_raw = find_red_green_pillars(
        img
    )

    # Las paredes son los blobs ANCHOS de esa misma lista, justo los que
    # la deteccion de pilares descarta.
    parking_blob, parking_gap = find_parking_scene(img, red_blobs_raw)

    # --------------------------------------------------------
    # MEDIDA DE COLOR
    #
    # Sirve para leer los valores reales con los que se afinan
    # PARKING_B_MEAN_MAX y PILLAR_B_MEAN_MIN.
    # --------------------------------------------------------

    red_lab = None
    parking_lab = None

    if SHOW_LAB_MEANS or PRINT_LAB_MEANS:
        red_lab = blob_lab_means(img, red_blob)

        # El ESP32 manda "P,0" al arrancar y apaga la busqueda de
        # estacionamiento, asi que parking_blob viene vacio durante toda
        # la fase de vueltas. Para medir se usa el candidato por forma.
        parking_lab = blob_lab_means(
            img,
            parking_blob
            or find_parking_wall_candidate(img, red_blobs_raw)
        )

        if PRINT_LAB_MEANS:
            if red_lab:
                print(
                    "ROJO  L:%d A:%d B:%d" % red_lab
                )

            if parking_lab:
                print(
                    "PARED L:%d A:%d B:%d" % parking_lab
                )

    parking_detected = 1 if parking_blob else 0
    parking_x = 0
    parking_area = 0
    parking_gap_detected = 1 if parking_gap else 0
    parking_gap_x = 0
    parking_gap_width = 0

    if parking_blob:
        parking_center_x = int(
            parking_blob.rect[0] + parking_blob.rect[2] // 2
        )

        if X_AS_NORMALIZED:
            parking_x = normalize_x100(parking_center_x)
        else:
            parking_x = parking_center_x

        parking_area = int(parking_blob.area)

    if parking_gap:
        parking_gap_center_x = int(parking_gap[0])
        parking_gap_width = int(parking_gap[1])

        if X_AS_NORMALIZED:
            parking_gap_x = normalize_x100(parking_gap_center_x)
        else:
            parking_gap_x = parking_gap_center_x

    selected_blob = None
    detected_id = 0

    # ID 3 = verde
    # ID 5 = rojo

    if green_blob and (
        not red_blob
        or green_blob.area >= red_blob.area
    ):
        selected_blob = green_blob
        detected_id = 3

    elif red_blob:
        selected_blob = red_blob
        detected_id = 5

    roi_code = 0
    area_pixels = 0
    threshold_index = 0
    selected_in_side = False

    # Solo para la pantalla: permiten ajustar MIN_PILLAR_DENSITY_PERCENT y
    # comprobar que convencion usa blob.rotation en esta camara.
    selected_density_percent = 0
    selected_rotation_degrees = 0

    x_reference = CORNER_SENTINEL
    y_reference = CORNER_SENTINEL

    # Coordenadas en pixeles usadas tambien para dibujar exactamente
    # la referencia enviada por UART.
    reference_x_pixel = CORNER_SENTINEL
    reference_y_pixel = CORNER_SENTINEL

    # --------------------------------------------------------
    # CÁLCULO DE LA ESQUINA
    # --------------------------------------------------------

    if selected_blob:

        if detected_id == 3:
            edge_x, lower_y = corner_lower_left(
                selected_blob
            )

        else:
            edge_x, lower_y = corner_lower_right(
                selected_blob
            )

        inside_low = point_in_roi(
            edge_x,
            lower_y,
            ROI_LOW
        )

        inside_high = point_in_roi(
            edge_x,
            lower_y,
            ROI_HIGH
        )

        # Un blob puede ocupar simultaneamente la ROI central y una lateral.
        # La caja completa conserva el seguimiento durante ese cruce.
        inside_side = (
            side_zone_for_blob(
                selected_blob,
                detected_id
            ) != 0
        )

        if inside_low or inside_high or inside_side:

            if inside_high:
                roi_code = 1
            else:
                roi_code = 2

            selected_in_side = inside_side

            # La referencia no cambia al pasar de HIGH a LOW.
            reference_x_pixel = edge_x
            reference_y_pixel = lower_y

            area_pixels = int(
                selected_blob.area
            )

            threshold_index = blob_threshold_index(
                selected_blob
            )

            selected_density_percent = blob_density_percent(
                selected_blob
            )

            try:
                selected_rotation_degrees = int(
                    abs(
                        radians_to_degrees(
                            selected_blob.rotation
                        )
                    )
                    % 180.0
                )
            except Exception:
                selected_rotation_degrees = 0

            if X_AS_NORMALIZED:
                x_reference = normalize_x100(
                    reference_x_pixel
                )
            else:
                x_reference = reference_x_pixel

            y_reference = reference_y_pixel

        else:
            detected_id = 0
            x_reference = CORNER_SENTINEL
            y_reference = CORNER_SENTINEL
            area_pixels = 0
            roi_code = 0
            threshold_index = 0

    # --------------------------------------------------------
    # DETECCIÓN DE COLISIÓN
    # --------------------------------------------------------

    # La colision usa la misma esquina de referencia. Aunque los laterales no
    # exigen forma vertical, sus esquinas quedan fuera de COLLISION_ROI y no
    # pueden activar por accidente esta salida central.
    collision_id = 0
    if (
        selected_blob
        and detected_id in (3, 5)
        and roi_code == 2
        and reference_x_pixel != CORNER_SENTINEL
        and point_in_roi(
            reference_x_pixel,
            reference_y_pixel,
            COLLISION_ROI
        )
    ):
        collision_id = detected_id

    # --------------------------------------------------------
    # DATOS ENVIADOS POR UART
    #
    # 0: ID del color
    # 1: X de la esquina
    # 2: Y de la esquina
    # 3: Área del blob
    # 4: Código de ROI
    # 5: ID de colisión
    # 6: Pared negra en ROI central (0/1)
    # 7: Pared de estacionamiento a la vista (0/1)
    # 8: X del centro de esa pared
    # 9: Área de esa pared
    # 10: Hueco entre dos paredes a la vista (0/1)
    # 11: X del centro del hueco
    # 12: Ancho del hueco en pixeles
    # --------------------------------------------------------

    data_to_send = [
        clamp_i16(detected_id),
        clamp_i16(x_reference),
        clamp_i16(y_reference),
        clamp_i16(area_pixels),
        clamp_i16(roi_code),
        clamp_i16(collision_id),
        clamp_i16(wall_black_detected),
        clamp_i16(parking_detected),
        clamp_i16(parking_x),
        clamp_i16(parking_area),
        clamp_i16(parking_gap_detected),
        clamp_i16(parking_gap_x),
        clamp_i16(parking_gap_width)
    ]

    try:
        send_uart_payload(
            data_to_send
        )

    except Exception as error:
        print(
            "UART_ERR:",
            error
        )

        reinitialize_uart()

    # --------------------------------------------------------
    # DIBUJOS DE DEPURACIÓN
    # --------------------------------------------------------

    if SHOW_DEBUG:
        img.draw_rectangle(
            ROI_LOW,
            color=(255, 255, 0)
        )

        img.draw_rectangle(
            ROI_HIGH,
            color=(255, 200, 0)
        )

        img.draw_rectangle(
            ROI_SIDE_LEFT,
            color=(0, 120, 255)
        )

        img.draw_rectangle(
            ROI_SIDE_RIGHT,
            color=(0, 120, 255)
        )

        img.draw_rectangle(
            PILLARS_ROI,
            color=(0, 200, 255)
        )

        if DETECT_PARKING:
            img.draw_rectangle(
                PARKING_ROI,
                color=(180, 0, 180)
            )

        img.draw_rectangle(
            COLLISION_ROI,
            color=(0, 255, 255)
        )

        if SHOW_CENTER_ROI:
            img.draw_rectangle(
                CENTER_ROI,
                color=(
                    (0, 255, 0)
                    if wall_black_detected
                    else (255, 0, 255)
                )
            )

        if USE_CORNER_DEAD_ZONES:
            # Bordes interiores de las dos zonas muertas trapezoidales.
            img.draw_line(
                (
                    CORNER_DEAD_ZONE_FAR_MARGIN_PX,
                    CORNER_DEAD_ZONE_TOP_Y,
                    CORNER_DEAD_ZONE_NEAR_MARGIN_PX,
                    CORNER_DEAD_ZONE_BOTTOM_Y
                ),
                color=(255, 40, 40),
                thickness=2
            )
            img.draw_line(
                (
                    IMG_W - 1 - CORNER_DEAD_ZONE_FAR_MARGIN_PX,
                    CORNER_DEAD_ZONE_TOP_Y,
                    IMG_W - 1 - CORNER_DEAD_ZONE_NEAR_MARGIN_PX,
                    CORNER_DEAD_ZONE_BOTTOM_Y
                ),
                color=(255, 40, 40),
                thickness=2
            )

        if DRAW_BOXES:

            if green_blob:
                img.draw_rectangle(
                    green_blob.rect,
                    color=(0, 255, 0)
                )

            if red_blob:
                img.draw_rectangle(
                    red_blob.rect,
                    color=(255, 0, 0)
                )

            if parking_blob:
                img.draw_rectangle(
                    parking_blob.rect,
                    color=(255, 0, 255),
                    thickness=2
                )

            if parking_gap:
                img.draw_cross(
                    (int(parking_gap[0]), 225),
                    color=(255, 255, 0),
                    size=7,
                    thickness=2
                )

        # ----------------------------------------------------
        # CRUZ DE LA REFERENCIA ENVIADA
        # HIGH y LOW conservan la misma esquina original.
        # ----------------------------------------------------

        if (
            detected_id != 0
            and roi_code != 0
            and reference_x_pixel != CORNER_SENTINEL
        ):
            img.draw_cross(
                (
                    int(reference_x_pixel),
                    int(reference_y_pixel)
                ),
                color=(
                    (0, 255, 0)
                    if detected_id == 3
                    else (255, 0, 0)
                ),
                size=5,
                thickness=1
            )

        # ----------------------------------------------------
        # TEXTO EN PANTALLA
        # ----------------------------------------------------

        if DRAW_TEXT:
            text_y = 2

            if X_AS_NORMALIZED:
                x_mode_text = "X100"
            else:
                x_mode_text = "X"

            img.draw_rectangle(
                (0, 0, 320, 60),
                color=(0, 0, 0),
                fill=True
            )

            img.draw_string(
                (2, text_y),
                "FPS: %.1f" % clock.fps(),
                color=(255, 255, 255)
            )

            text_y += 10

            x_show = data_to_send[1]
            y_show = data_to_send[2]

            if x_show == CORNER_SENTINEL:
                x_string = "--"
            else:
                x_string = "%4d" % x_show

            if y_show == CORNER_SENTINEL:
                y_string = "--"
            else:
                y_string = "%3d" % y_show

            img.draw_string(
                (2, text_y),
                "ID:%d %s:%s Y:%s"
                % (
                    data_to_send[0],
                    x_mode_text,
                    x_string,
                    y_string
                ),
                color=(255, 255, 255)
            )

            text_y += 10

            if data_to_send[4] == 1:
                roi_string = "HIGH"
            elif data_to_send[4] == 2:
                roi_string = (
                    "SIDE"
                    if selected_in_side
                    else "LOW"
                )
            else:
                roi_string = "--"

            img.draw_string(
                (2, text_y),
                "Area:%d D:%d%% R:%d ROI:%s TH:%d"
                % (
                    data_to_send[3],
                    selected_density_percent,
                    selected_rotation_degrees,
                    roi_string,
                    threshold_index
                ),
                color=(255, 255, 255)
            )

            text_y += 10

            img.draw_string(
                (2, text_y),
                "CollisionID:%d W:%d Blk:%d%%"
                % (
                    collision_id,
                    wall_black_detected,
                    center_roi_black_percent
                ),
                color=(255, 255, 255)
            )

            text_y += 10

            # Color medio del cuadrito central. Apuntalo a un pilar rojo
            # o a una pared magenta para leer su LAB sin tocar nada.
            # El porcentaje de negro pasa a llamarse Blk para no chocar
            # con la B del canal azul-amarillo.
            img.draw_string(
                (2, text_y),
                "ROI   L:%d A:%d B:%d"
                % (
                    center_roi_l_mean,
                    center_roi_a_mean,
                    center_roi_b_mean
                ),
                color=(255, 255, 255)
            )

            text_y += 10

            if SHOW_LAB_MEANS:
                # LAB completo de lo que se esta detectando ahora mismo.
                #
                # Van los tres canales y no solo la B porque cualquier
                # umbral que se toque hay que poder verlo: si se separa
                # por L hace falta leer la L, y sin ese numero se estaria
                # ajustando a ciegas.
                img.draw_string(
                    (2, text_y),
                    "Rojo  %s  Clf:%s"
                    % (
                        lab_means_string(red_lab),
                        "ON" if USE_B_MEAN_CLASSIFIER else "OFF"
                    ),
                    color=(255, 255, 255)
                )

                text_y += 10

                img.draw_string(
                    (2, text_y),
                    "Pared %s" % lab_means_string(parking_lab),
                    color=(255, 255, 255)
                )

                text_y += 10

            if parking_blob:
                parking_ratio_x10 = (
                    int(parking_blob.w) * 10
                ) // max(1, int(parking_blob.h))

                parking_string = (
                    "Park:SI X:%d A:%d W/H:%d.%d D:%d%%"
                    % (
                        parking_x,
                        parking_area,
                        parking_ratio_x10 // 10,
                        parking_ratio_x10 % 10,
                        blob_density_percent(parking_blob)
                    )
                )
                if parking_gap:
                    parking_string += " G:%d/%d" % (
                        parking_gap_x,
                        parking_gap_width
                    )
            elif not parking_detection_enabled:
                parking_string = "Park:OFF (esperando al ESP32)"
            else:
                parking_string = "Park:--"

            img.draw_string(
                (2, text_y),
                parking_string,
                color=(255, 120, 255)
            )

        # Se dibuja al final para que la trayectoria sea visible sobre las
        # ROIs y cajas de depuracion.
        if SHOW_PATH_PREVIEW:
            draw_trajectory_preview(img)

    # --------------------------------------------------------
    # FPS EN LA TERMINAL
    # --------------------------------------------------------

    if PRINT_FPS:
        print(
            "FPS: %.1f"
            % clock.fps()
        )

    # --------------------------------------------------------
    # IMAGEN BASE64
    #
    # Siempre debe ejecutarse al final, después de detectar
    # y dibujar.
    # --------------------------------------------------------

    if PRINT_BASE64_FRAME:
        print_base64_frame(
            img
        )
