# Carrera Individual: pruebas en el robot real

Protocolo para llevar el comportamiento del carril (escenario `Carril`) a un NAO real
con seguridad. Hacer las etapas en orden; no pasar a la siguiente si la anterior falla.
Resultados, cambios y decisiones de cada sesión: [BITACORA_ROBOT_REAL.md](BITACORA_ROBOT_REAL.md).

## Despliegue

```bash
Make/Common/deploy Develop <ip-del-robot> -s Carril -b
```

`-s Carril` deja `scenario = Carril;` en el `settings.cfg` del robot y `-b` reinicia bhuman.
Para volver a fútbol: desplegar con `-s 4v4_Full` (o el escenario del partido).

Los parámetros están en `Config/Scenarios/Carril/`:
- `carrilBehavior.cfg`: arranque (botón, silbato), velocidad, ganancias, parada en meta,
  paros y recuperación tras caída.
- `carrilPerceptProvider.cfg`: umbrales de color (línea negra y meta roja).

Cambiar un `.cfg` y volver a desplegar con `-nc` (sin compilar) es suficiente.

## Botones y LED

**El pecho nunca arranca la carrera.** Arranca un toque al **botón frontal de la cabeza**
o, si `arranquePorSilbato = true`, el silbato del juez.

| Estado | LED del pecho | Robot | Cómo se llega |
|---|---|---|---|
| Sin rigidez (`unstiff`) | azul parpadeando | sentado/sin par | al arrancar bhuman; o mantener **los 3 botones de la cabeza** |
| **Armado** (`initial`) | apagado | de pie, quieto, esperando el arranque | 1 toque al pecho desde sin rigidez |
| **Corriendo** | el mismo que tenía armado (apagado o verde) | anda (tras `esperaTrasBoton` = 0.8 s, o `esperaTrasSilbato` = 0 s) | 1 toque al **botón frontal de la cabeza** estando armado (o silbato) |
| **Paro manual** (`penalizedManual`) | rojo | de pie, quieto | 1 toque al pecho (corriendo o armado) |
| Armado de nuevo (`playing`) | verde | de pie, quieto | 1 toque al pecho desde rojo |

- El LED **no distingue armado de corriendo**: lo pone el GameState, que ya no decide la
  carrera. Que el robot ande es la única señal de que corre.
- Según el código de `GameStateProvider`, un toque al pecho desde "armado" (`initial`)
  pasa a rojo, no a verde: de sin rigidez a verde son tres toques (apagado, rojo, verde).
  Comprobarlo en la etapa 1; para la carrera basta con el primero (apagado).
- **Tras cualquier paro** (pecho, inclinación, meta, sin líneas tras una caída) la carrera
  queda *detenida*: no se reanuda sola ni con silbato. Para una carrera nueva:
  - estando armado (apagado o verde): toque al botón frontal de la cabeza; o
  - desde rojo: pecho (verde) y botón de la cabeza; o
  - 3 botones de la cabeza (sin rigidez), pecho (apagado) y botón o silbato. Pasar por
    sin rigidez es empezar de cero: el silbato vuelve a valer.
- Cada arranque es una carrera **nueva**: se olvidan la meta vista, los paros y las caídas.
- Silbato: solo cuenta uno detectado más de `margenSilbato` (0.5 s) después de armar y con
  confianza ≥ `minConfianzaSilbato` (0.5). Con GameController en la red, además solo en
  PLAYING (en la Copa no hay GameController ni Wi-Fi).
- El botón frontal se ignora si se pulsa con el central o el trasero (gesto de sin rigidez)
  y si se mantiene más de 1 s. El arranque se reconoce ~0.2 s **después de soltarlo**.
  El reconocedor publica el silbato ~0.5 s después de que deja de sonar
  (`accumulationDuration`): un pitido largo retrasa el arranque.
- **Paro por inclinación**: torso más inclinado que `maxInclinacion` (25°) → de pie, detenida.
  Si después cae, cuenta como caída (ver abajo).
- **Caída**: mientras cae actúa la protección de caída normal. Después:
  - `levantarseTrasCaida = false` (por defecto): sin rigidez en el suelo.
  - `levantarseTrasCaida = true`, `reanudarTrasCaida = false`: se levanta y se queda de pie.
  - ambos `true`: se levanta y, tras `tiempoEstableTrasCaida` (1 s) de pie con el torso a
    menos de `inclinacionEstable` (10°), **sigue la misma carrera** (conserva la meta
    vista). No anda hasta volver a ver las líneas; si no las ve en `maxEdadSinPercepto`
    (2 s), se queda de pie. Pasadas `maxCaidas` (2) caídas, sin rigidez en el suelo.

## Reglamento (sección 3.2) y cómo se cumple

- Activación con un sensor del NAO, hasta el silbato y no más de 10 s después: botón
  frontal de la cabeza (tras el silbato) o el propio silbato.
- Desactivación manual con un sensor que no está en las piernas: pecho (paro, de pie) o
  los 3 botones de la cabeza (sin rigidez).
- Sin comunicación externa: no se usa red; el silbato se oye con los micrófonos del NAO.
- Percances resueltos por el robot: recuperación tras caída (etapa 5, desactivada por
  defecto hasta probarla).

## Etapas

### 0. Antes de nada: foto de la cámara inferior

Con el robot de pie sobre el carril (o sobre cinta negra en madera), conectar SimRobot con
la escena `RemoteRobot`, abrir la imagen de la cámara inferior y guardarla. Si no es
posible, una foto con el móvil a la altura de la cabeza (~45 cm), mirando hacia delante y
abajo, también sirve para revisar contraste, brillos y sombras. Enviarla para revisar
`fraccionYLinea` (línea = más oscura que 0.6 × la duela) y `minVRojoMeta` (rojo de la meta).

### 1. Con arnés, sin carrera: botones y estados (`arranquePorSilbato = false`)

1. Desplegar y esperar al LED azul parpadeando.
2. Pecho → de pie (LED apagado). Esperar 10 s: **no debe moverse**.
3. Botón frontal de la cabeza (toque corto): ~1 s después debe empezar a andar.
   Sostener con el arnés y tocar el pecho: debe pararse (rojo) en menos de 1 s.
4. Pecho otra vez (verde): **no debe andar**. Botón de la cabeza: debe andar (carrera nueva).
   Pecho → rojo.
5. Anotar la secuencia de LED desde sin rigidez (¿apagado → rojo → verde?).
6. Mantener los 3 botones de la cabeza: debe quedarse sin rigidez **sin echar a andar**
   (probarlo también estando armado, LED apagado).

### 2. Con arnés: paro por inclinación

Con el robot andando y bien sujeto con el arnés, inclinarlo suavemente hacia delante más
de 25°: debe dejar de andar. Comprobar que no reanuda solo al enderezarlo y que un toque
al botón de la cabeza sí arranca una carrera nueva.

### 3. Con arnés, despacio, sobre el carril

En `carrilBehavior.cfg`: `velocidadAvance = 0.3;`. Desplegar con `-nc`. Colocar el robot
centrado tras la franja de salida, mirando a la meta. Armar (pecho) y arrancar con el
botón de la cabeza; acompañarlo con el arnés sin tirar de él. Comprobar que corrige el
rumbo, ve la meta y se para pasada la franja roja (unos 10-15 cm después, `margenParada`).

Repetir con salidas desplazadas (±10 cm) y giradas (±25°).

### 4. Silbato (`arranquePorSilbato = true`)

1. Robot sin rigidez. Pitar: no debe pasar nada. Pecho (armado). Antes de pitar,
   esperar 10-20 s con ruido normal de sala (voces, pasos, palmas): **no debe arrancar**.
2. Pitar con un silbato como el del juez, a la distancia a la que estará (2-5 m):
   debe arrancar. Repetir varias veces y anotar fallos.
3. Tras un paro con el pecho, volver a verde y pitar: **no debe arrancar** (solo el botón).
4. Si hay falsos positivos (arranca sin silbato), subir `minConfianzaSilbato`; si no
   arranca con el silbato real, bajarla (no por debajo de 0.25, que ya filtra el
   reconocedor). Si no es fiable, dejar `arranquePorSilbato = false` y arrancar con el
   botón tras el silbato: una salida anticipada puede anular la carrera.

### 5. Caída y recuperación (colchoneta, alguien listo para sujetar)

1. `levantarseTrasCaida = true`, `reanudarTrasCaida = false`. Con el robot andando,
   tumbarlo con cuidado sobre la colchoneta: debe levantarse y quedarse de pie quieto.
2. `reanudarTrasCaida = true`. Repetir sobre el carril: tras levantarse y ~1 s estable,
   debe seguir andando y corrigiendo el rumbo. Comprobar que no se sale del carril.
3. Tumbarlo y, al levantarse, dejarlo mirando fuera del carril (sin líneas a la vista):
   debe quedarse de pie (~2 s) y no andar.
4. Provocar tres caídas en la misma carrera (`maxCaidas = 2`): tras la tercera debe quedar
   sin rigidez en el suelo.
5. Cualquier comportamiento raro tras levantarse (anda con el torso inclinado, se vuelve
   a caer enseguida): volver a `reanudarTrasCaida = false`.

### 6. Velocidad de carrera

Subir `velocidadAvance` a 0.5 y luego a 0.8 (en simulación: 0.5 → 36 s, 0.8 → 24 s,
1.0 → 21.5 s, sin salidas). Si a una velocidad el robot se tambalea o se acerca a las
líneas, quedarse en la anterior. Al final, probar sin arnés con alguien al lado listo
para tocar el pecho.

## Qué está probado

| Qué | Simulación | Robot real |
|---|---|---|
| Visión y marcha, parada en meta | sí (regresión 12/12 a 0.3 con el flujo nuevo) | sí, a 0.3 (flujo anterior: arranque con el pecho) |
| Arranque por botón de la cabeza | lógica sí, con el disparo `dr module:CarrilBehavior:arrancar`; el botón real no se simula | no |
| Arranque por silbato | lógica sí (nuevo, viejo, tras un paro, desactivado), con el disparo `dr module:CarrilBehavior:silbato`; el reconocedor de audio no se simula | no |
| Paro manual con el pecho y detención | sí | no con el flujo nuevo |
| Paro por inclinación | sí (versión anterior); paso de inclinación a caída no | no |
| Recuperación tras caída (levantarse, reanudar, sin líneas, 3 caídas) | sí, con caídas provocadas con `mv` | no |

## Qué anotar en cada prueba

Velocidad, tiempo hasta la meta, si algún pie tocó o pisó una línea, dónde se paró, si
hubo tambaleo o caída, y condiciones de luz. En las pruebas de silbato: distancia,
ruido de la sala y cuántos silbatos detectó o no. Si algo falla, guardar la foto de la
cámara inferior en ese punto.
