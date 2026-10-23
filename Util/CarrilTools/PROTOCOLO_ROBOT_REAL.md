# Carrera Individual: pruebas en el robot real

Protocolo para llevar el comportamiento del carril (escenario `Carril`) a un NAO real
con seguridad. Hacer las etapas en orden; no pasar a la siguiente si la anterior falla.

## Despliegue

```bash
Make/Common/deploy Develop <ip-del-robot> -s Carril -b
```

`-s Carril` deja `scenario = Carril;` en el `settings.cfg` del robot y `-b` reinicia bhuman.
Para volver a fútbol: desplegar con `-s 4v4_Full` (o el escenario del partido).

Los parámetros están en `Config/Scenarios/Carril/`:
- `carrilBehavior.cfg`: velocidad, ganancias, espera inicial, parada en meta, paro por inclinación.
- `carrilPerceptProvider.cfg`: umbrales de color (línea negra y meta roja).

Cambiar un `.cfg` y volver a desplegar con `-nc` (sin compilar) es suficiente.

## Botones y LED del pecho

| Estado | LED del pecho | Robot | Cómo se llega |
|---|---|---|---|
| Sin rigidez (`unstiff`) | azul parpadeando | sentado/sin par | al arrancar bhuman; o mantener **los 3 botones de la cabeza** |
| De pie (`initial`) | apagado | de pie, quieto | 1 toque al pecho desde sin rigidez |
| **Carrera** (`playing`) | verde | espera `esperaInicial` (2 s) y corre | 1 toque al pecho desde de pie |
| **Paro manual** (`penalizedManual`) | rojo | de pie, quieto | 1 toque al pecho durante la carrera |

- Cada vez que se entra en carrera (verde) empieza una carrera **nueva**: se olvidan la
  meta vista y cualquier paro anterior. Para repetir: tocar el pecho (rojo), recolocar el
  robot en la salida y tocar el pecho otra vez (verde).
- **Paro por inclinación**: si el torso se inclina más de `maxInclinacion` (25°) o el robot
  cae, la carrera se aborta. Mientras cae actúa la protección de caída normal; ya en el
  suelo se queda sin rigidez (`levantarseTrasCaida = false`). Solo vuelve a moverse con
  una carrera nueva (rojo → verde).
- Con GameController en la red y el mismo número de equipo, sus estados mandan igual
  (solo se corre en PLAYING).

## Etapas

### 0. Antes de nada: foto de la cámara inferior

Con el robot de pie sobre el carril (o sobre cinta negra en madera), conectar SimRobot con
la escena `RemoteRobot`, abrir la imagen de la cámara inferior y guardarla. Si no es
posible, una foto con el móvil a la altura de la cabeza (~45 cm), mirando hacia delante y
abajo, también sirve para revisar contraste, brillos y sombras. Enviarla para revisar
`fraccionYLinea` (línea = más oscura que 0.6 × la duela) y `minVRojoMeta` (rojo de la meta).

### 1. Con arnés, sin carrera: botones y estados

1. Desplegar y esperar al LED azul parpadeando.
2. Pecho → de pie (LED apagado). Pecho → verde: debe esperar 2 s y empezar a andar.
   Sostener con el arnés y tocar el pecho: debe pararse (rojo) en menos de 1 s.
3. Mantener los 3 botones de la cabeza: debe quedarse sin rigidez.

### 2. Con arnés: paro por inclinación

Con el robot andando (verde) y bien sujeto con el arnés, inclinarlo suavemente hacia
delante más de 25°: debe dejar de andar. Comprobar que no reanuda solo al enderezarlo
(sigue parado hasta rojo → verde).

### 3. Con arnés, despacio, sobre el carril

En `carrilBehavior.cfg`: `velocidadAvance = 0.3;`. Desplegar con `-nc`. Colocar el robot
centrado tras la franja de salida, mirando a la meta. Verde y acompañarlo con el arnés
sin tirar de él. Comprobar que corrige el rumbo, ve la meta y se para pasada la franja
roja (unos 10-15 cm después, `margenParada`).

Repetir con salidas desplazadas (±10 cm) y giradas (±25°).

### 4. Velocidad de carrera

Subir `velocidadAvance` a 0.5 y luego a 0.8 (valor por defecto; en simulación: 0.5 →
36 s, 0.8 → 24 s, 1.0 → 21.5 s, sin salidas). Si a una velocidad el robot se tambalea
o se acerca a las líneas, quedarse en la anterior. Al final, probar sin arnés con
alguien al lado listo para tocar el pecho.

## Qué anotar en cada prueba

Velocidad, tiempo hasta la meta, si algún pie tocó o pisó una línea, dónde se paró, si
hubo tambaleo o caída, y condiciones de luz. Si algo falla, guardar la foto de la cámara
inferior en ese punto.
