# Sabana Herons — Rama `carril-vision`: Carrera Individual (Copa NAO CCM MX 2026)

Esta rama añade al código de RoboCup 2026 (descrito más abajo, en
[Código base](#código-base-sabana-herons--code-release-2026)) todo lo necesario
para que un NAO V6 corra la **Carrera Individual de la Copa NAO CCM MX 2026**
(3 de noviembre de 2026) guiándose solo con la cámara inferior.

**Prueba:** carril recto de 5 m × 70 cm sobre duela de madera, delimitado por dos
líneas negras de 1.9 cm, con franja de salida negra y meta roja. Si el robot pisa
fuera del carril, la carrera se anula.

**Estado:** desarrollo y validación en simulación terminados; compilación cruzada
para el NAO correcta. Faltan las pruebas en el robot real (protocolo en
[Util/CarrilTools/PROTOCOLO_ROBOT_REAL.md](Util/CarrilTools/PROTOCOLO_ROBOT_REAL.md))
y ajustar los umbrales de color con una foto real de la cámara inferior.

## Resumen de resultados (simulación)

| Prueba | Resultado |
| --- | --- |
| Precisión de la visión, robot quieto (10 poses, ±10 cm / ±25°, 413 frames) | 100 % de detección; error de `d` medio 3.3 mm (máx. 6.0 mm); error de `psi` medio 0.32° (máx. 1.80°) |
| Carrera completa a `velocidadAvance = 0.8` | 4.78 m en ~24 s, 0 salidas del carril, margen mínimo pie–línea 238 mm, parada de pie justo pasada la meta |
| Velocidades probadas | 0.5 → 36 s, 0.8 → 24 s, 1.0 → 21.5 s; todas sin salidas |
| Batería de robustez (salidas desplazadas/giradas × luz baja/alta, duela clara/oscura, líneas gastadas) | Todas las carreras correctas tras el umbral relativo: sin caídas, 0 salidas, margen mínimo ≥ 150 mm, parada entre x = 2577 y 2630 mm |
| Paro manual y paro por inclinación | Verificados en simulación |
| Compilación cruzada NAO (`Make/Linux/compile Develop Nao`) | Correcta; regresión en simulación 6/6 tras los cambios de seguridad |

## Cómo se hizo

El trabajo se hizo por pasos, cada uno validado con números medidos contra la pose
real del simulador (no a ojo) antes de pasar al siguiente.

### Paso 0. Entorno de compilación (Ubuntu 24.04 / WSL2)

El código base no compilaba con clang 18 y Qt ≥ 6.4. Cambios:

- `Make/CMake/CMakeLists.txt`: inclusión forzada de `<cstdint>` en todo el C++
  (los submódulos CompiledNN y SimRobot usan `uintptr_t`/`uint16_t` sin incluirlo),
  sin modificar los submódulos.
- `Make/CMake/QtBackingStoreCompat.cpp` (nuevo): definición vacía de
  `qt_registerDefaultPlatformBackingStoreOpenGLSupport()`, que Qt 6.4 eliminó y
  SimRobot sigue llamando; se añade al target SimRobot solo con Qt ≥ 6.4.
- Local, no versionable: `git config core.symlinks true` y re-checkout de los
  symlinks; dependencias apt de `Util/SimRobot/README.md` más `build-essential`,
  `ccache` y `libasound2-dev`.

Compilar: `Make/Linux/compile Develop SimRobot` (binario en
`Build/Linux/SimRobot/Develop/SimRobot`).

### Paso 1. Cámaras simuladas

Se comprobó que SimRobot renderiza imágenes reales de ambas cámaras (inferior
320×240, superior 640×480) en las escenas normales (las "Fast" desactivan las
imágenes). Las imágenes se capturan de forma automática con un `.con` que espera
frames (`for Lower get representation:FrameInfo`) y guarda con `for Lower si <ruta>.png`.

### Paso 2. Escena del carril

- `Config/Scenes/Includes/Carril/carril.rsi2`: losa de duela de 7 × 3 m, las dos
  líneas negras, franja de salida y meta roja, y el NAO colocado en la salida.
  Los colores ambientales son el 80 % del difuso (con un ambiente gris los colores
  se ven lavados).
- `Config/Scenes/Carril.ros2` / `Carril.con`: escena principal (sin árbitro
  automático, coloca el robot con `mv` y pasa a `playing` tras ~8 s).
- Supuestos: 70 cm = ancho libre entre bordes internos de las líneas; franjas de
  salida y meta de 10 cm dentro de los 5 m; en coordenadas del robot la salida está
  en x = −2450 mm y la meta en x = +2450 mm.

Detalle importante: SimRobot espeja x/y (+180°) al robot del primer equipo, tanto
en `mv` como en `GroundTruthWorldState`. Por eso en la escena la meta está en
x = −2.45 m y el robot mira hacia −x; en las coordenadas del robot y de las
herramientas todo queda como se describe arriba.

### Paso 3. Visión del carril

`CarrilPerceptProvider` (hilo Lower) produce `CarrilPercept`:

1. Recorre la imagen de la cámara inferior fila a fila (una de cada `pasoFilas`)
   buscando tramos oscuros y poco cromáticos.
2. Proyecta cada tramo al suelo con la `CameraMatrix` y descarta los que no tienen
   el ancho de una línea (`minAnchoLineaSuelo`..`maxAnchoLineaSuelo`).
3. Encadena tramos por continuidad horizontal entre filas (`maxSaltoPx`).
4. Ajusta una recta por mínimos cuadrados totales a cada cadena y elige el par de
   rectas casi paralelas cuya separación coincide con el ancho nominal.
5. Calcula `d` (desplazamiento lateral respecto al eje, mm, positivo a la izquierda)
   y `psi` (desorientación, positiva antihoraria). Si solo ve una línea (robot muy
   girado), decide el lado por el signo de su posición y usa el ancho nominal.

Todos los umbrales están en `Config/Scenarios/Carril/carrilPerceptProvider.cfg`,
ninguno fijo en el código.

Para medir la precisión se creó `CarrilEvaluador` (hilo Cognition, solo en
simulación), que escribe en un CSV el percepto junto a la pose real
(`GroundTruthWorldState`), y `Util/CarrilTools/evaluar_carril.py`, que genera
escenas de prueba y analiza el CSV.

### Paso 4. Control de marcha

`CarrilBehavior` (hilo Cognition) sustituye a `SkillBehaviorControl` solo como
proveedor de `MotionRequest` y `HeadMotionRequest` en el escenario `Carril`.
Ley de control:

```
giro   = clamp(-(kD * d + kPsi * psi), ±maxGiro)
avance = velocidadAvance * (1 - frenoPorGiro * |giro|)
```

Sin percepto fresco (`maxEdadPercepto` = 300 ms) sigue recto y despacio; si pasan
`maxEdadSinPercepto` (2 s) sin percepto, se queda de pie. La cabeza se fija para
que la cámara inferior quede 30° bajo la horizontal (`inclinacionCabeza`; con 0°
pierde las líneas cerca de la meta).

El evaluador se amplió para calcular el borde exterior de cada pie en coordenadas
del mundo en cada paso, de modo que `evaluar_carril.py marcha` cuenta salidas del
carril por pie y paso y el margen mínimo a las líneas.

Resultado de la primera carrera: 4.66 m en 38.2 s, 168 pasos, 0 salidas, margen
mínimo 198 mm. La visión en marcha dio un 59 % de frames válidos (suficiente con la
tolerancia de 300 ms) y error medio de `d` de 5.1 mm.

Lección: si el robot echa a andar nada más aparecer en la escena se cae; hace falta
recolocarlo con `mv` y una espera inicial.

### Paso 5. Meta roja y parada

El percepto añade `metaVisible`, `distanciaMeta` y `numPuntosMeta` (píxeles con
V − 128 ≥ `minVRojoMeta`). El comportamiento frena a menos de `distanciaFrenado`
(800 mm) y, a menos de `distanciaFijarParada` (500 mm), fija la parada por
odometría a la distancia medida más `margenParada` (200 mm); después se queda de pie.

Resultado: 4.78 m recorridos, parada en x ≈ 2630 mm (la franja de meta ocupa
2400..2500), 0 salidas, margen mínimo 238 mm. La meta se detecta desde ~2.5 m con
error medio de ~30 mm.

### Paso 6. Robustez

`evaluar_carril.py bateria` lanza una instancia de SimRobot por caso, genera una
escena temporal (`Config/Scenes/CarrilEval.*`, `Includes/Carril/carrilEval.rsi2`)
y resume en una tabla con veredicto (sin caída, 0 salidas, parada entre 2500 y
3000 mm). Casos:

- 9 salidas: centrada, ±10 cm, ±25° y combinaciones;
- 5 variantes de escena: luz baja (25 %), luz alta (70 %), duela clara, duela
  oscura y líneas gastadas (gris, en tramos de 40 cm con huecos de 5 cm).

Todo pasó salvo las líneas gastadas: su luminancia (~70) superaba el umbral
absoluto `maxYLinea` = 60 y el robot no arrancaba. Se cambió a un **umbral
relativo**: un píxel es línea si su luminancia es menor que
`min(maxYLinea = 110, fraccionYLinea = 0.6 × mediana de la fila)`. Con ello la
batería completa pasó 12/12, incluidas las líneas gastadas.

### Paso 7. Seguridad y paso al robot real

`CarrilBehavior` usa los estados del `GameState` (los mismos botones que en un
partido):

| Estado | LED del pecho | Robot | Cómo se llega |
| --- | --- | --- | --- |
| Sin rigidez | azul parpadeando | sin par | al arrancar, o los 3 botones de la cabeza |
| De pie (`initial`) | apagado | quieto | 1 toque al pecho |
| Carrera (`playing`) | verde | espera 2 s y corre | 1 toque al pecho |
| Paro manual (`penalizedManual`) | rojo | quieto de pie | 1 toque al pecho durante la carrera |

- Cada entrada en `playing` es una carrera nueva (se reinician meta y paros).
- **Paro por inclinación:** si el torso supera `maxInclinacion` (25°) o el robot
  cae, la carrera se aborta. Mientras cae actúa la protección normal (no se pide
  `playDead`, que desactivaría el FallEngine); ya en el suelo queda sin rigidez
  hasta una carrera nueva (`levantarseTrasCaida = false`). El estado "en el suelo"
  se recuerda porque en `playDead` el `FallDownState` vuelve a `upright` aunque el
  robot siga tumbado.
- En simulación: Shift+Ctrl+P = playing, Shift+Ctrl+M = paro, Shift+Ctrl+U = quitar paro.
- `CarrilEvaluador` solo escribe el CSV en simulación.

## Uso

```bash
# Simulación
Make/Linux/compile Develop SimRobot
Build/Linux/SimRobot/Develop/SimRobot Config/Scenes/Carril.ros2

# Evaluación
python3 Util/CarrilTools/evaluar_carril.py generar      # precisión con robot quieto
python3 Util/CarrilTools/evaluar_carril.py analizar
python3 Util/CarrilTools/evaluar_carril.py marcha       # analiza una carrera completa
python3 Util/CarrilTools/evaluar_carril.py bateria todas todas

# Robot real
Make/Linux/compile Develop Nao
Make/Common/deploy Develop <ip-del-robot> -s Carril -b
```

Para cambiar solo parámetros basta con editar el `.cfg` y desplegar con `-nc`.

## Archivos de la rama

| Ruta | Contenido |
| --- | --- |
| `Src/Representations/Perception/Carril/CarrilPercept.h` | Percepto del carril (`d`, `psi`, líneas, meta) |
| `Src/Modules/Perception/Carril/CarrilPerceptProvider.*` | Detector de líneas y meta (cámara inferior) |
| `Src/Modules/BehaviorControl/CarrilBehavior/*` | Control de marcha, parada en meta y paros |
| `Src/Modules/Infrastructure/CarrilEvaluador/*`, `Src/Representations/Infrastructure/CarrilEvaluacion.h` | Registro CSV percepto vs. pose real (solo simulación) |
| `Config/Scenarios/Carril/` | `threads.cfg` y parámetros (`carrilBehavior.cfg`, `carrilPerceptProvider.cfg`, `carrilEvaluador.cfg`) |
| `Config/Scenes/Carril.*`, `Config/Scenes/Includes/Carril/carril.rsi2` | Escena del carril |
| `Config/Scenes/CarrilEval.*`, `Includes/Carril/carrilEval.rsi2` | Escenas generadas por `evaluar_carril.py` |
| `Util/CarrilTools/evaluar_carril.py` | Generación de escenas, análisis y batería de robustez |
| `Util/CarrilTools/PROTOCOLO_ROBOT_REAL.md` | Protocolo de pruebas en el NAO real |
| `Make/CMake/CMakeLists.txt`, `Make/CMake/QtBackingStoreCompat.cpp` | Arreglos de compilación (clang 18 / Qt ≥ 6.4) |

## Pendiente

- Pruebas en el robot real siguiendo el protocolo: botones y paros con arnés,
  velocidad baja (0.3) y subida progresiva hasta 0.8.
- Foto real de la cámara inferior sobre el carril (o cinta negra sobre madera) para
  ajustar `fraccionYLinea` y `minVRojoMeta`.
- Confirmar la regla de meta (si basta cruzarla o hay que parar dentro); hoy se
  para unos 10–15 cm después de la franja roja.

---

# Código base: Sabana Herons — Code Release 2026


This is the code that team **Sabana Herons** (Universidad de La Sabana,
Colombia) used in the **RoboCup 2026 Humanoid Soccer League (HSL)** with NAO
V6 robots, 4v4 on the HSL Small field.

**It is based on the [B-Human Code Release 2023](https://github.com/bhuman/BHumanCodeRelease/releases/tag/coderelease2023).**
B-Human's framework, motion (walking, kicks, get-up), perception, localization,
team communication, behavior architecture, and tools (SimRobot, deploy dialog)
are the foundation of everything here. On top of it we:

1. **migrated the code from the SPL to the HSL 2026 rules** and the HSL
   GameController protocol v20;
2. **adapted the team strategies** to 4v4 and 3v3 on the HSL Small field,
   including set plays, kick-offs, dropped balls, and penalty kicks;
3. **replaced the ball detection** with a YOLO network running on the robot
   (ONNX), since the HSL ball is not the SPL ball;
4. **rewrote the whistle recognizer** with three spectral profiles;
5. **integrated reinforcement learning (RL)**: policies trained in SimRobot
   decide which skill each player executes, and run embedded on the NAO;
6. **added operation tools** for deployment, remote control, camera
   streaming, and dataset collection.

The RL training itself was developed in a separate Python repository (see
[Reinforcement learning](#5-reinforcement-learning)).

If you have used the B-Human 2023 release before, everything you know still
applies: build, deploy, SimRobot, and the module/representation architecture
are unchanged. The [B-Human 2023 documentation](https://wiki.b-human.de/coderelease2023/)
is the reference for all of that.

---

## What we changed

### 1. HSL 2026 rules and GameController

The SPL GameController protocol (v18) was replaced by the HSL protocol (v20).
`GameStateProvider` and `GameState` handle Stop Play, direct and indirect free
kicks, throw-ins, goal kicks, corner kicks, penalty kicks, dropped balls,
cautions, and sent-off players. The behavior enforces the HSL restart rules:
no direct goal from kick-off (two-touch rule, or a touch outside the center
circle when two or fewer robots are active), the 45 s free-kick limit,
opponent goal kicks with the whole penalty area cleared, and HSL penalty-kick
placement. The simulated GameController in SimRobot exposes the HSL commands.

Robots returning from a penalty are seeded by `SelfLocator` on the touchline
at the height of their own penalty mark, using a side hint remembered before
the penalty to break the left/right symmetry. For restarts, a new
`RestartBallSearchProvider` predicts where the ball will be placed, so robots
search there instead of waiting to see it.

Details: [docs/HSL2026_Migration.md](docs/HSL2026_Migration.md).

### 2. Strategies for 4v4 and 3v3

New strategies, tactics, setup poses, and locations for HSL-sized fields:
`4v4_Full` and `3v3_Full` (field dimensions of the HSL 2026 Small field). In
the final 4v4 configuration the goalkeeper dives, and the kick-off is a short
lateral pass to receivers placed away from the center circle. The classical
4v4 behavior (no RL) is the `4v4_Full` scenario.

### 3. Ball detection

The HSL uses a FIFA-style ball instead of the SPL ball, so we added:

- `YoloBallDetector`: a YOLOv8n detector (320×320) exported to ONNX and run
  asynchronously on the NAO with ONNX Runtime, with a lightweight image-space
  Kalman tracker that compensates for camera motion; its output feeds
  B-Human's `BallPerceptFilter` and `BallStateEstimator`;
- Trionda-specific candidate generation and classification modules, kept as
  an alternative to the network;
- `CameraStreamer` and `RawBallPatch` to collect images and ball patches for
  training.

The scenario used in matches selects `YoloBallDetector` with
`Config/NeuralNets/BallDetector/yolo_ball_320.onnx`.
Details: [docs/BallDetection.md](docs/BallDetection.md).

### 4. Whistle recognition

`WhistleRecognizer` was rewritten around three independent profiles
(standard referee whistle, hand-squeeze whistle, and mouth whistle), each
using Goertzel band energy, SNR, spectral flatness, and temporal gates
(onset confirmation, hang-over, gap filling). It still provides B-Human's
`Whistle` representation, so the rest of the code is unchanged.

### 5. Reinforcement learning

The policies decide **which skill** to execute (stand, walk, shoot, pass,
dribble, block, mark, observe) and four continuous parameters; B-Human still
executes every skill and every motion. On the robot,
`StrategyBehaviorControl` computes the classical decision first and replaces
it with the policy's decision only during active play, after a legal-action
gate. If a model is missing or inference fails, the classical behavior is
used.

| Policy | Observation | Used in |
| --- | --- | --- |
| Striker (`striker_base`) | 26 values | `4v4_StrikerBase` |
| Defender/support (`baseline_attack`) | 26 values | `4v4_BaselineAttack` |
| Team striker v4.2 (`mixed_attack`) | 47 values | `4v4_MixedAttack` |
| Merged field-player brain v5 (`complete`) | 47 values + role | `4v4_Complete` (our match configuration) |
| Goalkeeper | 64 values, 12 skills | any scenario, enabled with `--rl-gk on` |

In the `complete` mode one network serves all field players: a C++ role
coordinator assigns striker, open support, and off-ball support roles, and the
role is part of the observation.

The training pipeline (in the separate RL repository) is: rule-based teachers
derived from B-Human's behavior → behavioral cloning → PPO/MAPPO in SimRobot
with a curriculum and a KL anchor to the cloned policy → scenario-based
evaluation → ONNX export. SimRobot is driven from Python through the `pybh`
bindings and `RLSharedState`, so the policies are trained against the real
B-Human behavior, motion, and perception stack.

| Document | Content |
| --- | --- |
| [docs/RL/Integration.md](docs/RL/Integration.md) | How the policies run on the robot, models, deploy options, fallback |
| [docs/RL/Environment.md](docs/RL/Environment.md) | Training environment: observation, action, skill gate, reward, curriculum |
| [docs/RL/Training.md](docs/RL/Training.md) | BC + PPO/MAPPO training, evaluation, ONNX export, model lineage |
| [docs/RL/Coordination.md](docs/RL/Coordination.md) | Role coordinator, support placement, shoot-vs-pass arbitration, role-conditioned mask |

The pipeline is described in the paper *A Teacher-Guided RL Pipeline for
Role-Conditioned Football Cognition in Humanoid Robots*. The implementation
details the paper refers to are documented here:

| Topic | Document |
| --- | --- |
| SimRobot bridge and training environment | [docs/RL/Environment.md](docs/RL/Environment.md) |
| Production scenarios and curriculum stages | [docs/RL/Environment.md](docs/RL/Environment.md#curriculum-and-world-generation-curriculumpy-production_curriculumpy), [docs/RL/Training.md](docs/RL/Training.md#behavioral-evaluation-behavioral_evaluatorpy) |
| Team message structure | [docs/RL/Integration.md](docs/RL/Integration.md#team-communication) |
| Teacher dataset generation and audit | [docs/RL/Training.md](docs/RL/Training.md#production-imitation-datasets-teacher_production_datasetpy-production_curriculumpy) |
| Role-coordinator rules and constants | [docs/RL/Coordination.md](docs/RL/Coordination.md#role-coordinator) |
| Shoot-vs-pass decision | [docs/RL/Coordination.md](docs/RL/Coordination.md#shoot-vs-pass-arbitration) |

Training with the RL repository requires the multi-agent observation fields
of `RLSharedState`, which are in the
[`rl-simrobot3d`](https://github.com/juanulloa2050/SabanaHerons2026/tree/rl-simrobot3d)
branch of this repository; `master` contains everything needed to run the trained policies.

### 6. Tools and robustness

- `Make/Common/deploy` and the deploy dialog select the RL mode per player,
  the goalkeeper policy, and goalkeeper diving (see `Config/teams.cfg`).
- [Util/KeyboardControl](Util/KeyboardControl/README.md): control robots from
  a phone or browser, live camera view, recording, and ball dataset
  collection.
- Stability fixes: the debug thread no longer busy-waits without a client,
  more connection retries at startup, safe handling of `off`/`ignore` joint
  sentinels, non-negative scan-line starts, and a fallback in `Zweikampf` when
  a field-line intersection fails.

### Earlier work (2024–2025)

This code base also contains the team's earlier work: robot identities,
calibrations and network profiles, the 5v5 migration and the
attack/defensive/"Tortuga" strategies of 2024, spoken feedback, a Windows
deploy dialog, the referee-gesture pipeline (disabled by default in
`HandleRefereeSignal.h`), and optimizations in `ImageTransform.h` (polar
transform for the ball CNN) and `UKFPose2D`.

---

## Getting started

Building, deploying, and running SimRobot work exactly as in B-Human 2023;
follow the [B-Human 2023 documentation](https://wiki.b-human.de/coderelease2023/).
After cloning, initialize the submodules:

```bash
git submodule update --init
```

### Scenarios and locations

| Scenario | Description |
| --- | --- |
| `4v4_Complete` | Our match configuration: merged RL brain for field players, YOLO ball detector |
| `4v4_Full` | Classical B-Human-style 4v4 behavior, no field-player RL |
| `4v4_NoRLBlock` | Classical 4v4 with its own blocking formation; the deploy script disables all RL policies |
| `4v4_StrikerBase`, `4v4_BaselineAttack`, `4v4_MixedAttack` | Earlier RL policies, for comparison |
| `3v3_Full`, `3v3_RL_TeamV42`, `3v3_RL_MergedV5` | 3v3 variants |
| `4v4_RL2D`, `4v4_RL3D` | Scenarios driven by the Python RL environment |

Use the location `4v4_Full` (or `3v3_Full`) on the HSL Small field. Other
scenarios and locations come from B-Human or from our earlier SPL work.

### Deploying

```bash
cd Make/Common
./deploy Release \
  -r 1 <goalkeeper-ip> -r 2 <player-ip> -r 3 <player-ip> -r 4 <player-ip> \
  -t <team-number> -s 4v4_Complete -l 4v4_Full \
  --rl-complete 2,3,4 --rl-gk on --goalkeeper-dive on
```

Use `--rl-disable --rl-gk off` with `-s 4v4_Full` for the classical behavior.
All RL options are listed in [docs/RL/Integration.md](docs/RL/Integration.md#deployment).
The robots, network profiles, and team number in `Config/` are ours; replace
them with your own.

### Simulation

The usual B-Human scenes work (`Config/Scenes/*.ros2`). `RLvsBH3v3_3D.ros2`
plays RL-controlled field players against a B-Human team; the `RL*` scenes
are used by the Python training environment.

---

## Repository layout

Only the folders that differ from B-Human 2023 are listed.

| Path | Content |
| --- | --- |
| `Src/Libs/RL` | Observation encoders, skill gates, action decoders, ONNX policy wrappers |
| `Src/Libs/Python/Controller` | `pybh` extensions and `RLSharedState` (Python ↔ SimRobot) |
| `Src/Modules/BehaviorControl/StrategyBehaviorControl` | Embedded RL, role coordinator, 4v4/3v3 behavior, restart search |
| `Src/Modules/Perception/BallPerceptors` | `YoloBallDetector`, Trionda modules, external detector bridge |
| `Src/Modules/Modeling/WhistleRecognizer` | Three-profile whistle recognizer |
| `Src/Modules/Infrastructure/RLSkillProvider` | Applies actions from Python during training |
| `Src/Modules/Infrastructure/CameraStreamer` | Camera streaming for data collection |
| `Config/NeuralNets/RLPolicy` | RL policies (ONNX) and manifests |
| `Config/NeuralNets/BallDetector` | YOLO ball detector models |
| `Config/Scenarios`, `Config/Locations` | HSL 4v4/3v3 and RL scenarios |
| `Util/KeyboardControl` | Web/keyboard control and data collection tools |
| `docs/` | Documentation of our contributions |

## Known limitations

- The goalkeeper policy's save rate comes from a simulated benchmark; dive
  reach was not calibrated on the real robot.
- `--rl-complete` selects the merged model for all eligible field players; it
  does not restrict it to the listed player numbers.
