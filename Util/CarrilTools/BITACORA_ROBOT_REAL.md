# Bitácora de pruebas en el robot real (Carrera Individual)

Registro de lo que se probó en el NAO real, lo que pasó, qué se cambió y por qué. El
protocolo de pruebas está en [PROTOCOLO_ROBOT_REAL.md](PROTOCOLO_ROBOT_REAL.md); aquí van
los resultados. Las horas son las del reloj del robot (`bhumand_(N).log` en
`/home/nao/logs/`).

Robot: **juan** (192.168.49.5, Ethernet). Despliegue:
`Make/Common/deploy Develop 192.168.49.5 -s Carril -b`.

## 2026-10-28 — silbato, parada en meta y velocidad

### Configuración al terminar la sesión

| Parámetro | Antes | Ahora | Archivo |
|---|---|---|---|
| `captureVolume` | 100 | **75** | `Config/Scenarios/Carril/audioProvider.cfg` |
| `mouthWhistleEnabled` | true | **false** | `Config/Scenarios/Carril/whistleRecognizer.cfg` |
| `minConfianzaSilbato` | 0.5 | **0.35** | `Config/Scenarios/Carril/carrilBehavior.cfg` |
| `distanciaFrenado` | 800 | **1300** | ídem |
| `distanciaFijarParada` | 500 | **1200** | ídem |
| `avanceFrenado` | 0.3 | **0.4** | ídem |
| `velocidadAvance` | 0.3 | **0.8** (en prueba) | ídem |

Sin cambiar: `avanceSinPercepto = 0.3`, `margenParada = 200`, `maxEdadSinPercepto = 2000`,
`kD = 0.003`, `kPsi = 1.5`, `maxGiro = 0.6`, umbrales del perfil `rescue_whistle`.

### 1. Silbato demasiado sensible

**Qué pasaba.** El arranque por silbato se disparaba con facilidad.

**Causa.** No se habían tocado los umbrales del reconocedor; se juntaron dos cosas:
`captureVolume` estaba en 100 (el resto de escenarios usa 75) y, con
`detectInAnyState = true` y `onlySoundInSetAndPlaying = false`, el robot escucha siempre,
no solo en SET/PLAYING.

**Pruebas con el robot sentado, ganancia 75.**
- El silbato real entra por el perfil `rescue_whistle` a 3100–3700 Hz con mucho margen:
  pMax de 950 a 5400 (umbral 41) y SNR de 9 a 13 dB (umbral 3.2).
- El perfil `mouth_whistle` (silbido con la boca, 1700–4200 Hz) pasó *todos* sus umbrales
  con sonidos de la sala a 1700–2550 Hz (voces o ruido, lejos del silbato). No llegó a
  disparar porque esos sonidos no se sostuvieron, pero dio una detección dudosa
  (20:54:35, 2550 Hz, confianza 0.29).

**Decisiones.**
- Ganancia de vuelta a **75**: el silbato real sigue pasando holgado.
- **`mouthWhistleEnabled = false`** en Carril: el juez usa silbato, y este perfil era la
  fuente de casi‑disparos con voces.
- **`minConfianzaSilbato` de 0.5 a 0.35**: en el log, las detecciones reales tenían
  confianza 0.42–0.49 al abrirse. Luego se vio que la confianza que llega al
  comportamiento es mayor (0.59–0.83 en todos los arranques), porque se acumula mientras
  suena el silbato; 0.35 sigue dejando margen sobre el 0.25 del reconocedor.

### 2. Bug: el log no mostraba las detecciones del silbato

**Síntoma.** Después de varios pitazos solo aparecía un `Whistle detected` en el log.

**Causa.** `Whistle detected` y `Whistle monitor` compartían el limitador `lastLogTime`
(1 línea por segundo). El monitor escribe cada segundo, así que casi siempre se comía la
línea de detección. La detección sí ocurría; lo que no salía era el mensaje.

**Arreglo** (`WhistleRecognizer.cpp/.h`): las detecciones usan su propio limitador,
`lastDetectionLogTime`, de `minAnnotationDelay / 4` (0.5 s). Con el arreglo se vieron 4
detecciones en 11 s.

### 3. Diagnóstico de la carrera en el robot

`OUTPUT_TEXT` no llega a `bhumand.log`, y la parada en meta no escribía nada, así que no
había forma de saber por qué paraba el robot. Se añadieron líneas `OUTPUT_WARNING` en
`CarrilBehavior`:
- `arranque por silbato (confianza X)` / `arranque por botón de la cabeza`
- `meta vista a N mm, parada fijada por odometría`
- `parada en meta tras N mm desde la última lectura de la meta`
- `paro manual, carrera detenida`
- `sin líneas desde hace N ms, se queda de pie`
- Una línea `estado` por segundo: odometría recorrida, `valido`, `lineas`, `d`, `psi`,
  `meta=visible/distancia`, edad de la imagen, giro y avance pedidos, y `DE_PIE`.

### 4. Se paraba antes de la meta (104 cm y 54 cm)

**Qué se vio** (corrida de las 21:11:50). La meta aparece a 2.4 m. A ~1.2 m de la meta
dejaba de ver las líneas del carril, porque la franja roja ocupa la imagen. A los 2 s sin
líneas (`maxEdadSinPercepto`) se quedaba de pie, con la meta a 0.93 m. La parada por
odometría solo se fijaba con la meta a ≤ 500 mm, distancia a la que nunca llegaba
viendo líneas. Era una zona muerta entre 1.2 m y 0.5 m.

La distancia a la meta es estable desde 1.2 m (lecturas seguidas de 1241, 1192, 1215 y
1192 mm).

**Decisiones.**
- `distanciaFijarParada` de 500 a **1200** y `distanciaFrenado` de 800 a **1300**.
- La parada **se vuelve a fijar con cada lectura** de la meta mientras se vea, así que la
  odometría solo cubre el último tramo (~0.4 m más `margenParada`).

**Resultado.** A partir de la corrida de las 21:16:44 llega siempre a la franja y se para
justo después de cruzarla (`parada en meta tras ~600 mm`).

### 5. Casi se sale a 0.5: deriva al andar sin líneas

**Qué se vio** (corrida de las 21:22:38, paro manual). Perdió las líneas a ~1 m de la meta
y anduvo 9 s "en recto" sin corregir, con 5° de error de rumbo en la última lectura más
la deriva propia de la marcha. A 0.5 pierde las líneas más a menudo, con tramos de hasta
3 s.

**Arreglo** (`CarrilBehavior.cpp/.h`). Sin líneas, tanto con la parada fijada como en
pérdidas cortas (≤ `maxEdadSinPercepto`), `d` y `psi` se estiman por odometría desde el
último percepto válido y se aplica el mismo control:
- eje del carril = rotación odométrica en el último percepto − `psi` medido;
- `d` estimado = `d` medido + desplazamiento odométrico sobre la normal izquierda del eje;
- `psi` estimado = rotación odométrica actual − eje del carril.

**Resultado.** Con la misma desviación máxima (~11 cm), el p95 de |d| bajó de ~95 mm a
~60 mm.

### 6. "Andaba pero no avanzaba"

En una corrida (~21:04–21:06) el robot pareció marchar en el sitio un momento. No se
pudo confirmar en el log: la odometría cuenta los pasos pedidos, no lo que avanza de
verdad, así que el patinaje no se ve ahí. Pendiente de observar; si se repite, anotar la
hora para cruzarla con el log.

### Corridas

| Hora | `bhumand` | Vel. / frenado | Arranque | Fin | Tiempo | max \|d\| | p95 \|d\| | Sin líneas* |
|---|---|---|---|---|---|---|---|---|
| 21:04:57 | 24 | 0.3 / 0.3 | silbato, 0.83 | se quedó a 104 cm (sin log) | — | — | — | — |
| 21:11:50 | 25 | 0.3 / 0.3 | silbato, 0.82 | de pie sin líneas, a 54 cm | — | 86 mm | 59 mm | 33 % |
| 21:16:44 | 26 | 0.3 / 0.3 | silbato, 0.72 | **meta** | 80 s | 106 mm | 95 mm | 56 % |
| 21:22:38 | 27 | 0.5 / 0.3 | silbato, 0.59 | paro manual, casi se sale | — | 108 mm | 98 mm | 38 % |
| 21:28:36 | 28 | 0.5 / 0.3 + estimación a ciegas | silbato, 0.78 | **meta** | 59 s | 113 mm | 64 mm | 58 % |
| 21:36:01 | 29 | 0.65 / 0.4 | silbato, 0.70 | **meta** | 46 s | 115 mm | 58 mm | 54 % |
| 21:43:26 | 31 | 0.8 / 0.4 | silbato, 0.58 | **meta** | 49 s (log), ~40 s cronometrado | 118 mm | 69 mm | 49 % |

\* Porcentaje de líneas `estado` con `valido=false`. Incluye el tramo final, a partir de
~1.2 m de la meta, en el que es normal no ver las líneas.

`d` se toma solo de las lecturas válidas. Margen aproximado hasta pisar la línea: el
carril mide 70 cm (35 cm por lado) y el borde exterior del pie queda a unos 15 cm del
centro del robot, así que con |d| = 11 cm quedan ~9 cm.

Velocidad real medida por odometría: 0.3 → ~6 cm/s, 0.5 → ~10 cm/s. El máximo del motor
de marcha (`Config/Robots/Default/walkingEngine.cfg`) es 250 mm/s, y `velocidadAvance`
es una fracción de ese valor.

### Por qué a 0.8 tarda ~40–49 s y no 24 s como en simulación

Desglose de la corrida a 0.8 (21:43:26), según las líneas `estado`:

| Tramo | Tiempo | Distancia | Velocidad |
|---|---|---|---|
| Viendo líneas, a 0.8 | ~22 s | ~2.6 m | ~120 mm/s |
| Sin líneas, a `avanceSinPercepto` = 0.3 | ~8 s | ~0.8 m | ~100 mm/s |
| Tramo final, a `avanceFrenado` = 0.4 | ~19 s | ~1.6 m | ~83 mm/s |

1. **El robot real anda más despacio que lo pedido.** 0.8 son 200 mm/s nominales (como en
   simulación: 4.78 m en 24 s), pero en el robot real se miden ~120 mm/s.
2. **Pierde las líneas la mitad del tiempo** y entonces baja a 0.3. Con
   `maxAcceleration.x = 75 mm/s²` (`walkingEngineCommon.cfg`) tarda ~1.5 s en volver a
   acelerar, y como alterna sin parar entre 0.3 y 0.8, casi nunca llega a la velocidad
   máxima.
3. **El tramo final lento es largo:** ~1.6 m a 0.4. En simulación se veían las líneas
   hasta la meta y el frenado empezaba a 0.8 m.

### Decisiones de velocidad

- Se sube en escalones (0.3 → 0.5 → 0.65 → 0.8), siempre con arnés, según el paso 6 del
  protocolo. Se desplegó 0.75, pero no llegó a correrse: se pasó directo a 0.8.
- **Techo previsto: 0.8.** No se planea 1.0 en el robot real: en simulación no hay
  deriva ni patinaje, y en la duela una salida anula la carrera.
- Criterio para bajar: |d| máximo por encima de ~13 cm, tambaleo o que se acerque a una
  línea.
- `avanceSinPercepto` se deja en 0.3 para no ir rápido a ciegas.
- El tramo final (desde ~1.3 m) va a `avanceFrenado`. Con 0.3 ese tramo costaba ~20 s;
  con 0.4 algo menos. Subir `velocidadAvance` solo acelera los primeros ~3.5 m.

### Otros

- El primer despliegue de Carril borró del robot los escenarios `Maze*` que había (el
  deploy hace `rsync --del` de `Config/`). Venían de otro árbol y no están en este repo.
- Error recurrente `uppercamera: 1000 ms passed and there's still no image`: es de la
  cámara superior, que Carril no usa; se recupera sola.

### Pendiente

- Registrar las corridas posteriores a 0.8: se probó varias veces más, pero sus datos no
  están en esta tabla (hay que sacarlos de los logs del robot).
- Por qué pierde las líneas ~1/3 del tiempo a mitad de carrera (visión).
- Prueba sin arnés y prueba de falsos positivos del silbato con ruido de sala (protocolo,
  paso 4.1).

## 2026-10-29 — offset del giroscopio al arrancar

- En dos arranques seguidos (`bhumand_(33)` y `(34)`), `GyroOffsetProvider` reportó
  "Gyro has Offset … 7 degrees" y pedía reinicio cada 10 s. El segundo arranque fue un
  reinicio por software (`sudo systemctl reboot`; `sudo reboot` pide contraseña).
  El día anterior, 13 arranques sin este aviso.
- El aviso significa que, con el robot desactivado y apoyado, el giroscopio promedió más
  de 3°/s durante ~1 s. Es una lectura del sensor: flashear no lo arregla.
- **Solución:** apagado en frío con el botón del pecho, unos minutos apagado y encendido
  con el robot quieto en el suelo, sin arnés. En `bhumand_(35)` ya no salió el aviso.
- **Regla:** arrancar siempre con el robot quieto y apoyado, nunca colgando del arnés,
  y sin tocarlo hasta unos segundos después de "Lets go".
- Al final de la sesión del 28 también avisó "Heat": dejarlo enfriar entre tandas.
