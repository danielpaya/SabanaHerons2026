#!/usr/bin/env python3
"""Evaluación del percepto del carril contra la pose real de SimRobot.

Uso:
  python3 Util/CarrilTools/evaluar_carril.py generar
      Crea la escena temporal Config/Scenes/CarrilEval.{ros2,con}: coloca el robot
      en una lista de poses (desplazado ±10 cm y girado ±25°, según el plan) y deja
      que CarrilEvaluador (escenario Carril) registre /tmp/carril_eval/eval.csv.
      Al terminar el guion, guarda /tmp/carril_eval/fin.png como centinela.

  python3 Util/CarrilTools/evaluar_carril.py analizar [ruta_csv]
      Separa el CSV en segmentos (uno por pose, detectando saltos de la pose real),
      descarta los primeros frames de cada segmento (teletransporte y balanceo) y
      reporta error medio y máximo de d y psi, por pose y globales.

  python3 Util/CarrilTools/evaluar_carril.py marcha [ruta_csv]
      Analiza una carrera completa: salidas del carril por pie y paso, margen a las
      líneas, visión en marcha, meta, parada y caídas.

  python3 Util/CarrilTools/evaluar_carril.py bateria [variante,...|todas] [todas|centrada|peores]
      Robustez (paso 6): por cada variante de escena (luz, tono de duela, líneas
      gastadas) y cada salida (±10 cm, ±25°), lanza SimRobot con la carrera completa,
      guarda el CSV en /tmp/carril_eval/bateria/ y resume en una tabla.

Convenios: el eje del carril es el eje x de la escena, así que d_real = y_real y
psi_real = rotación real. Estándar: solo biblioteca estándar de Python.
"""

import csv
import math
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[2]
RUTA_CSV = "/tmp/carril_eval/eval.csv"

# Poses de prueba (x mm, y mm, giro en grados): plan del paso 3 (±10 cm, ±25°),
# cerca de la salida y en el centro del carril.
POSES = [
    (-2200, 0, 0),
    (-2200, 100, 0),
    (-2200, -100, 0),
    (-2200, 0, 25),
    (-2200, 0, -25),
    (-2200, 100, 25),
    (-2200, -100, -25),
    (0, 0, 0),
    (0, 100, -25),
    (0, -100, 25),
]
GETS_POR_POSE = 30  # cada "get" espera una respuesta del hilo Lower (al menos un frame)

# Umbrales para detectar el cambio de pose en la pose real y descartar el asentamiento.
SALTO_MM = 30.0
SALTO_GRAD = 5.0
FRAMES_DESCARTADOS = 10
MIN_FRAMES_SEGMENTO = 10


def generar():
    escenas = RAIZ / "Config/Scenes"
    ros2 = (escenas / "Carril.ros2").read_text()
    (escenas / "CarrilEval.ros2").write_text(ros2)
    lineas = [
        "# Escena TEMPORAL generada por Util/CarrilTools/evaluar_carril.py. No editar; borrar tras evaluar.",
        "call Includes/Normal",
        "dr debugDrawing3d:representation:GameState off",
        "call Includes/Views",
        "ar off",
        "",
    ]
    for x, y, giro in POSES:
        lineas.append(f"mv {x} {y} 320 0 0 {giro}")
        lineas += ["for Lower get representation:FrameInfo"] * GETS_POR_POSE
        lineas.append("")
    lineas.append("for Lower si /tmp/carril_eval/fin.png")
    (escenas / "CarrilEval.con").write_text("\n".join(lineas) + "\n")
    print(f"Escena de evaluación escrita: {escenas / 'CarrilEval.ros2'}")
    print(f"Poses: {len(POSES)}; CSV esperado: {RUTA_CSV}; centinela: /tmp/carril_eval/fin.png")


def segmentar(filas):
    segmentos = []
    actual = []
    anterior = None
    for fila in filas:
        if anterior is not None and (
            abs(fila["y_real"] - anterior["y_real"]) > SALTO_MM
            or abs(fila["x_real"] - anterior["x_real"]) > 3 * SALTO_MM
            or abs(fila["psi_real_grad"] - anterior["psi_real_grad"]) > SALTO_GRAD
        ):
            segmentos.append(actual)
            actual = []
        actual.append(fila)
        anterior = fila
    segmentos.append(actual)
    return [s[FRAMES_DESCARTADOS:] for s in segmentos if len(s) > FRAMES_DESCARTADOS + MIN_FRAMES_SEGMENTO]


def analizar(ruta):
    with open(ruta, newline="") as f:
        filas = [{k: float(v) for k, v in fila.items()} for fila in csv.DictReader(f)]
    if not filas:
        sys.exit(f"CSV vacío: {ruta}")
    segmentos = segmentar(filas)
    print(f"{len(filas)} frames, {len(segmentos)} poses tras descartar el asentamiento\n")
    cab = ("pose real (y mm, psi°)", "frames", "válidos", "líneas", "err d medio", "err d máx", "err psi medio", "err psi máx")
    print(f"{cab[0]:>24} {cab[1]:>7} {cab[2]:>8} {cab[3]:>7} {cab[4]:>12} {cab[5]:>10} {cab[6]:>14} {cab[7]:>12}")
    errores_d, errores_psi = [], []
    for seg in segmentos:
        y_m = sum(f["y_real"] for f in seg) / len(seg)
        p_m = sum(f["psi_real_grad"] for f in seg) / len(seg)
        validos = [f for f in seg if f["valido"] > 0.5]
        if validos:
            ed = [f["d_est"] - f["y_real"] for f in validos]
            ep = [f["psi_est_grad"] - f["psi_real_grad"] for f in validos]
            lineas = sum(f.get("lineas", 0) for f in validos) / len(validos)
            errores_d += ed
            errores_psi += ep
            print(f"{f'({y_m:+7.1f}, {p_m:+6.1f})':>24} {len(seg):>7} {len(validos):>8} {lineas:>7.1f} "
                  f"{sum(ed) / len(ed):>10.1f}mm {max(abs(e) for e in ed):>8.1f}mm "
                  f"{sum(ep) / len(ep):>13.2f}° {max(abs(e) for e in ep):>11.2f}°")
        else:
            print(f"{f'({y_m:+7.1f}, {p_m:+6.1f})':>24} {len(seg):>7} {0:>8} {'—':>7} {'—':>12} {'—':>10} {'—':>14} {'—':>12}")
    if errores_d:
        print(f"\nGLOBAL ({len(errores_d)} frames válidos):")
        print(f"  d:   error medio {sum(errores_d) / len(errores_d):+.1f} mm, |error| medio "
              f"{sum(abs(e) for e in errores_d) / len(errores_d):.1f} mm, máximo {max(abs(e) for e in errores_d):.1f} mm")
        print(f"  psi: error medio {sum(errores_psi) / len(errores_psi):+.2f}°, |error| medio "
              f"{sum(abs(e) for e in errores_psi) / len(errores_psi):.2f}°, máximo {max(abs(e) for e in errores_psi):.2f}°")
        tasa = len(errores_d) / sum(1 for s in segmentos for _ in s)
        print(f"  tasa de detección válida: {100 * tasa:.1f}%")
    else:
        print("\nSin frames válidos: revisar umbrales de carrilPerceptProvider.cfg")


MEDIO_CARRIL = 350.0  # mm: borde interno de cada línea respecto al eje


def marcha(ruta, imprimir=True):
    """Análisis de una corrida caminando (escena Carril.ros2): salidas por pie y por paso,
    margen mínimo a las líneas, error de la visión en movimiento, meta, parada y caídas.
    Devuelve un resumen (dict) para la batería."""
    with open(ruta, newline="") as f:
        filas = [{k: float(v) for k, v in fila.items()} for fila in csv.DictReader(f)]
    if not filas:
        sys.exit(f"CSV vacío: {ruta}")
    # Ignorar todo lo anterior al último teletransporte (mv de colocación; antes del
    # primer mv la pose llega espejada por lado de equipo).
    ultimo_salto = 0
    for i in range(1, len(filas)):
        if abs(filas[i]["x_real"] - filas[i - 1]["x_real"]) > 500:
            ultimo_salto = i
    filas = filas[ultimo_salto:]
    # Caídas: estado del cuerpo (FallDownState) 3 = cayendo, 4 = caído. En todo el
    # registro tras la colocación, también después de la parada.
    caido = any(f.get("estado_cuerpo", 1) in (3, 4) for f in filas)
    # Quedarse con la marcha: desde que el robot avanza 50 mm respecto al inicio.
    x0 = filas[0]["x_real"]
    inicio = next((i for i, f in enumerate(filas) if f["x_real"] > x0 + 50), None)
    if inicio is None:
        if imprimir:
            print("El robot no avanzó: no hay marcha que analizar")
        return {"avanzo": False, "caido": caido, "salidas": 0, "margen_min": float("nan"),
                "duracion": float("nan"), "x_final": filas[-1]["x_real"], "y_final": filas[-1]["y_real"],
                "parado": False, "validos": 0.0, "err_d": float("nan"), "meta_vista": False}
    datos = filas[inicio:]
    # Si el robot se paró (p. ej. tras la meta), analizar solo hasta la parada: el
    # tiempo quieto después diluiría velocidad y tasa de detección.
    x_fin = datos[-1]["x_real"]
    fin = next(i for i, f in enumerate(datos) if all(abs(g["x_real"] - x_fin) < 5 for g in datos[i:]))
    tiempo_quieto = (datos[-1]["t"] - datos[fin]["t"]) / 1000.0
    parado = fin > 0 and tiempo_quieto > 2.0
    if parado:
        datos = datos[:fin + 1]
    duracion = (datos[-1]["t"] - datos[0]["t"]) / 1000.0
    avance = datos[-1]["x_real"] - datos[0]["x_real"]

    # Pasos: cambio de signo del reparto de apoyo (positivo = apoyo izquierdo).
    pasos = []
    actual = [datos[0]]
    for f in datos[1:]:
        if (f["soporte"] >= 0) != (actual[-1]["soporte"] >= 0):
            pasos.append(actual)
            actual = []
        actual.append(f)
    pasos.append(actual)

    # Solo cuentan los pasos dentro del carril (antes de cruzar la meta, x < 2500).
    salidas_izq = salidas_der = 0
    margen_min = float("inf")
    margenes = []
    for paso in pasos:
        dentro = [f for f in paso if f["x_real"] < 2500]
        if not dentro:
            continue
        m_izq = min(MEDIO_CARRIL - f["y_pie_izq"] for f in dentro)
        m_der = min(f["y_pie_der"] + MEDIO_CARRIL for f in dentro)
        if m_izq <= 0:
            salidas_izq += 1
        if m_der <= 0:
            salidas_der += 1
        margenes.append(min(m_izq, m_der))
        margen_min = min(margen_min, m_izq, m_der)

    validos = [f for f in datos if f["valido"] > 0.5]
    con_meta = [f for f in datos if f.get("meta", 0) > 0.5]
    ult = datos[-1]
    ed = [f["d_est"] - f["y_real"] for f in validos]
    ep = [f["psi_est_grad"] - f["psi_real_grad"] for f in validos]
    resumen = {
        "avanzo": True, "caido": caido, "salidas": salidas_izq + salidas_der, "margen_min": margen_min,
        "duracion": duracion, "x_final": ult["x_real"], "y_final": ult["y_real"], "parado": parado,
        "validos": 100 * len(validos) / len(datos),
        "err_d": sum(abs(e) for e in ed) / len(ed) if ed else float("nan"),
        "meta_vista": bool(con_meta),
    }
    if not imprimir:
        return resumen

    print(f"Marcha: {duracion:.1f} s, avance {avance / 1000:.2f} m, velocidad media {avance / duracion / 1000:.2f} m/s")
    print(f"x final: {ult['x_real']:.0f} mm (meta en +2500), |y| máx del torso: "
          f"{max(abs(f['y_real']) for f in datos):.0f} mm")
    print(f"\nPasos: {len(pasos)}  |  SALIDAS del carril: pie izquierdo {salidas_izq}, pie derecho {salidas_der}")
    print(f"Margen mínimo pie-línea: {margen_min:.0f} mm (global); margen mínimo medio por paso: "
          f"{sum(margenes) / len(margenes):.0f} mm")
    if validos:
        print(f"\nVisión en marcha ({len(validos)}/{len(datos)} frames válidos, "
              f"{100 * len(validos) / len(datos):.1f}%):")
        print(f"  d:   |error| medio {sum(abs(e) for e in ed) / len(ed):.1f} mm, máximo {max(abs(e) for e in ed):.1f} mm")
        print(f"  psi: |error| medio {sum(abs(e) for e in ep) / len(ep):.2f}°, máximo {max(abs(e) for e in ep):.2f}°")

    # Meta: error de la distancia estimada frente a la real (centro de la franja en x=+2450).
    if con_meta:
        em = [f["dist_meta"] - (2450.0 - f["x_real"]) for f in con_meta]
        print(f"\nMeta vista en {len(con_meta)} frames (primera a {2450 - con_meta[0]['x_real']:.0f} mm reales):")
        print(f"  distancia: |error| medio {sum(abs(e) for e in em) / len(em):.1f} mm, "
              f"máximo {max(abs(e) for e in em):.1f} mm")

    # Parada: ¿terminó quieto y dónde?
    quieto = [f for f in datos if ult["t"] - f["t"] <= 2000]
    desplazamiento = max(abs(f["x_real"] - ult["x_real"]) for f in quieto)
    estado = (f"PARADO (quieto {tiempo_quieto:.0f} s al final)" if parado
              else "PARADO" if desplazamiento < 20 else f"EN MOVIMIENTO (±{desplazamiento:.0f} mm en los últimos 2 s)")
    print(f"\nFinal: {estado} en x={ult['x_real']:.0f} mm, y={ult['y_real']:.0f} mm "
          f"(franja de meta: 2400..2500, borde de la losa: 3500)")
    print(f"Caída: {'SÍ' if caido else 'no'}")
    return resumen


# --- Batería de robustez (paso 6): carrera completa por cada salida y variante de escena ---

# Salidas (y mm, giro en grados) desde x = -2200: centrada, desplazada ±10 cm, girada
# ±25° y las combinaciones peores (desplazada y girada hacia fuera) y cruzadas.
SALIDAS = [(0, 0), (100, 0), (-100, 0), (0, 25), (0, -25), (100, 25), (-100, -25), (100, -25), (-100, 25)]

# Variantes de escena: cambios de luz (Carril.ros2) y de superficies (carril.rsi2).
# Cada una es (luz difusa y ambiente en %, color de duela, color de líneas, líneas gastadas).
DUELA = "rgb(185, 140, 90)"
LINEA = "rgb(20, 20, 20)"
VARIANTES = {
    "base": (45, DUELA, LINEA, False),
    "luz_baja": (25, DUELA, LINEA, False),
    "luz_alta": (70, DUELA, LINEA, False),
    "duela_clara": (45, "rgb(225, 190, 140)", LINEA, False),
    "duela_oscura": (45, "rgb(120, 80, 45)", LINEA, False),
    "lineas_gastadas": (45, DUELA, "rgb(70, 70, 70)", True),
}
# Líneas gastadas: tramos de TRAMO_GASTADO m con huecos de HUECO_GASTADO m (desgaste/cinta despegada).
TRAMO_GASTADO = 0.40
HUECO_GASTADO = 0.05

GETS_ASENTAR = 240  # ~8 s en pie tras colocar al robot, antes de playing
GETS_CARRERA = 2400  # espera inicial + carrera + parada (~80 s simulados)
CARPETA = Path("/tmp/carril_eval")


def _ambiente(color):
    """Ambiente = 80% del difuso (mismo criterio que carril.rsi2)."""
    r, g, b = (int(c) for c in color[4:-1].split(","))
    return f"rgb({int(r * 0.8)}, {int(g * 0.8)}, {int(b * 0.8)})"


def generar_carrera(y, giro, variante):
    """Escribe Config/Scenes/CarrilEval.{ros2,con} y, si hace falta, la copia variada del carril."""
    luz, duela, linea, gastadas = VARIANTES[variante]
    escenas = RAIZ / "Config/Scenes"
    rsi2 = (escenas / "Includes/Carril/carril.rsi2").read_text()
    rsi2 = rsi2.replace(f'name="carrilDuela" diffuseColor="{DUELA}" ambientColor="{_ambiente(DUELA)}"',
                        f'name="carrilDuela" diffuseColor="{duela}" ambientColor="{_ambiente(duela)}"')
    # Superficie propia para las líneas laterales (la franja de salida conserva el negro).
    rsi2 = rsi2.replace('<Surface name="carrilMeta"',
                        f'<Surface name="carrilLateral" diffuseColor="{linea}" ambientColor="{_ambiente(linea)}" '
                        f'specularColor="rgb(5%, 5%, 5%)" shininess="2"/>\n  <Surface name="carrilMeta"')
    for lado in ("0.3595m", "-0.3595m"):
        original = (f'    <BoxAppearance depth="5m" width="0.019m" height="0.002m">\n'
                    f'      <Surface ref="carrilLinea"/>\n'
                    f'      <Translation y="{lado}" z="0.001m"/>\n'
                    f'    </BoxAppearance>\n')
        assert original in rsi2, lado
        if gastadas:
            tramos = []
            x = -2.5
            while x < 2.5 - 1e-6:
                largo = min(TRAMO_GASTADO, 2.5 - x)
                tramos.append(f'    <BoxAppearance depth="{largo:.3f}m" width="0.019m" height="0.002m">\n'
                              f'      <Surface ref="carrilLateral"/>\n'
                              f'      <Translation x="{x + largo / 2:.3f}m" y="{lado}" z="0.001m"/>\n'
                              f'    </BoxAppearance>\n')
                x += TRAMO_GASTADO + HUECO_GASTADO
            rsi2 = rsi2.replace(original, "".join(tramos))
        else:
            rsi2 = rsi2.replace(original, original.replace("carrilLinea", "carrilLateral"))
    (escenas / "Includes/Carril/carrilEval.rsi2").write_text(
        "<!-- TEMPORAL generado por Util/CarrilTools/evaluar_carril.py (variante " + variante + "). No editar. -->\n" + rsi2)

    ros2 = (escenas / "Carril.ros2").read_text()
    ros2 = ros2.replace('href="Includes/Carril/carril.rsi2"', 'href="Includes/Carril/carrilEval.rsi2"')
    ros2 = ros2.replace('<DirLight z="-1" diffuseColor="rgb(45%, 45%, 45%)" ambientColor="rgb(45%, 45%, 45%)"/>',
                        f'<DirLight z="-1" diffuseColor="rgb({luz}%, {luz}%, {luz}%)" ambientColor="rgb({luz}%, {luz}%, {luz}%)"/>')
    (escenas / "CarrilEval.ros2").write_text(ros2)

    lineas = [
        f"# Escena TEMPORAL generada por Util/CarrilTools/evaluar_carril.py (salida y={y} giro={giro}, {variante}).",
        "call Includes/Normal",
        "dr debugDrawing3d:representation:GameState off",
        "call Includes/Views",
        "ar off",
        "",
        f"mv -2200 {y} 320 0 0 {giro}",
    ]
    # Asentarse en pie (~8 s) y empezar la carrera (playing), como en Carril.con.
    lineas += ["for Lower get representation:FrameInfo"] * GETS_ASENTAR + ["gc playing"]
    lineas += ["for Lower get representation:FrameInfo"] * GETS_CARRERA
    lineas += [f"for Lower si {CARPETA}/fin_lower.png", f"for Upper si {CARPETA}/fin.png"]
    (escenas / "CarrilEval.con").write_text("\n".join(lineas) + "\n")


def correr_carrera(y, giro, variante):
    """Lanza UNA instancia de SimRobot con la carrera, espera al centinela y la cierra por PID."""
    import shutil
    import subprocess
    import time
    generar_carrera(y, giro, variante)
    for nombre in ("eval.csv", "fin.png", "fin_lower.png"):
        (CARPETA / nombre).unlink(missing_ok=True)
    simrobot = RAIZ / "Build/Linux/SimRobot/Develop/SimRobot"
    with open(CARPETA / "simrobot_bateria.log", "w") as log:
        proc = subprocess.Popen([str(simrobot), "Config/Scenes/CarrilEval.ros2"], cwd=RAIZ, stdout=log, stderr=log)
    try:
        limite = time.time() + 400
        while not (CARPETA / "fin.png").exists() and time.time() < limite and proc.poll() is None:
            time.sleep(1)
    finally:
        proc.kill()
        proc.wait()
    nombre = f"carrera_{variante}_y{y:+d}_g{giro:+d}"
    destino = CARPETA / "bateria" / f"{nombre}.csv"
    destino.parent.mkdir(exist_ok=True)
    shutil.copy(CARPETA / "eval.csv", destino)
    for png in ("fin.png", "fin_lower.png"):
        if (CARPETA / png).exists():
            shutil.copy(CARPETA / png, destino.with_name(f"{nombre}_{png}"))
    return marcha(destino, imprimir=False)


def bateria(variantes, salidas):
    print(f"{'variante':>16} {'salida':>12} {'tiempo':>7} {'salidas':>8} {'margen':>7} {'válidos':>8} "
          f"{'err d':>6} {'meta':>5} {'x final':>8} {'parado':>7} {'caída':>6}  veredicto")
    for variante in variantes:
        for y, giro in salidas:
            r = correr_carrera(y, giro, variante)
            ok = (r["avanzo"] and not r["caido"] and r["salidas"] == 0 and r["parado"]
                  and 2500 <= r["x_final"] <= 3000)
            print(f"{variante:>16} {f'{y:+d}mm {giro:+d}°':>12} {r['duracion']:>6.1f}s {r['salidas']:>8} "
                  f"{r['margen_min']:>5.0f}mm {r['validos']:>7.1f}% {r['err_d']:>4.1f}mm "
                  f"{'sí' if r['meta_vista'] else 'NO':>5} {r['x_final']:>8.0f} {'sí' if r['parado'] else 'NO':>7} "
                  f"{'SÍ' if r['caido'] else 'no':>6}  {'OK' if ok else 'FALLO'}", flush=True)


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "generar":
        generar()
    elif len(sys.argv) >= 2 and sys.argv[1] == "analizar":
        analizar(sys.argv[2] if len(sys.argv) > 2 else RUTA_CSV)
    elif len(sys.argv) >= 2 and sys.argv[1] == "marcha":
        marcha(sys.argv[2] if len(sys.argv) > 2 else RUTA_CSV)
    elif len(sys.argv) >= 2 and sys.argv[1] == "bateria":
        # bateria [variante,...|todas] [todas|centrada|peores]
        elegidas = sys.argv[2].split(",") if len(sys.argv) > 2 and sys.argv[2] != "todas" else list(VARIANTES)
        modo = sys.argv[3] if len(sys.argv) > 3 else "todas"
        salidas = {"todas": SALIDAS, "centrada": [(0, 0)], "peores": [(0, 0), (100, 25), (-100, -25)]}[modo]
        bateria(elegidas, salidas)
    else:
        sys.exit(__doc__)
