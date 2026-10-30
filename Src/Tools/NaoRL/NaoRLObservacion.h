/**
 * @file NaoRLObservacion.h
 *
 * De los sensores a la entrada de la politica, y el limitador de objetivos. Parte "pura" (sin B-Human) y compartida: la usan
 * la fase de marcha (NaoRLWalkPhase) y el modo sombra (NaoRLSombra), asi que no pueden divergir. El paso desde las
 * representaciones de B-Human a estos datos esta en NaoRLSensoresBHuman.h.
 */

#pragma once

#include "NaoRLGuardas.h" // misma carpeta: asi tambien compila en Tests, que no tiene Src/ en la ruta
#include "NaoRLPolitica.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace NaoRL
{
  // ===================================================================================================================
  // SIGNOS POR VERIFICAR CON EL ROBOT SUSPENDIDO (contrato, seccion 5). Si un canal sale al reves, invertir SOLO su
  // constante aqui (la politica nunca vio otro signo). Prueba de cada uno:
  //  - signoGyroY: inclinar el torso hacia ADELANTE (nariz abajo) -> gyro_y > 0 mientras gira.
  //  - signoGyroX: inclinar hacia la DERECHA (gira alrededor de +x, hombro izquierdo arriba) -> gyro_x > 0.
  //  - signoGyroZ: girar a la IZQUIERDA (antihorario visto desde arriba) -> gyro_z > 0.
  //  - signoArribaX: robot inclinado ADELANTE -> arriba_x < 0 (atras -> > 0).
  //  - signoArribaY: robot inclinado a la DERECHA -> arriba_y > 0 (el "arriba" del mundo queda hacia la izquierda
  //    del torso; derivado de la regla de la mano derecha, no medido).
  //  - signoPsi: girar el robot a la DERECHA sobre el carril -> psi_obs > 0. CarrilPercept.psi > 0 = robot girado a
  //    la izquierda; la politica usa psi = rumbo_deseado - rumbo_real, de ahi el -1 (contrato 5.2).
  //  - signoD: robot desplazado a la IZQUIERDA del eje -> d_obs > 0 (CarrilPercept.d y la politica coinciden).
  // ===================================================================================================================
  constexpr float signoGyroX = 1.f, signoGyroY = 1.f, signoGyroZ = 1.f;
  constexpr float signoArribaX = 1.f, signoArribaY = 1.f;
  constexpr float signoPsi = -1.f;
  constexpr float signoD = 1.f;

  constexpr int maxEdadFotoMs = 300; /**< = maxEdadPercepto de CarrilBehavior. */

  /** Los sensores de un ciclo, en crudo. */
  struct SensoresPaso
  {
    std::array<float, 3> gyro{};     /**< rad/s. */
    float qw = 1.f, qx = 0.f, qy = 0.f, qz = 0.f; /**< InertialData.orientation3D. */
    Accion angulos{};                /**< Ultimo angulo finito de los 15 motores, orden de la accion. */
    float psiCamara = 0.f;           /**< CarrilPercept.psi, rad. */
    float dCamaraMm = 0.f;           /**< CarrilPercept.d, mm. */
    bool camaraValida = false;       /**< CarrilPercept.valido. */
    bool metaVisible = false;
    float distanciaMetaMm = 0.f;
    unsigned tiempoFrame = 0;        /**< CarrilPercept.tiempoFrame. */
    int edadFrameMs = 0;             /**< Tiempo desde tiempoFrame. */
  };

  /** Lo que se recuerda de la camara entre ciclos. */
  struct EstadoVision
  {
    unsigned ultimoPerceptoValido = 0; /**< tiempoFrame del ultimo percepto con lineas validas. */
    bool metaCerca = false;            /**< La meta se vio a menos de distanciaMetaSinCamara en esta marcha. */
  };

  /**
   * Un ciclo: gyro, arriba, angulos y vision, con los signos del entrenamiento. Se mantienen el psi/d de "anterior" entre
   * fotos y con fotos sin lineas (como en el entrenamiento, cuadros perdidos = se mantiene el ultimo).
   */
  inline EntradaPaso leerSensores(const SensoresPaso& s, const EntradaPaso& anterior, const bool ignorarCamara,
                                  const float distanciaMetaSinCamara, EstadoVision& vision)
  {
    EntradaPaso e = anterior;
    e.gyro = {signoGyroX * s.gyro[0], signoGyroY * s.gyro[1], signoGyroZ * s.gyro[2]};

    // arriba = fila 3 de la matriz torso->mundo = R^T * (0, 0, 1) (misma formula que gravedadProyectada de Camilo, sin
    // el signo menos).
    const float w = s.qw, x = s.qx, y = s.qy, z = s.qz;
    e.arriba = {signoArribaX * 2.f * (x * z - w * y), signoArribaY * 2.f * (y * z + w * x), 1.f - 2.f * (x * x + y * y)};

    e.angulos = s.angulos;

    // Camara: d [mm] -> [m].
    const bool fresca = s.tiempoFrame && s.edadFrameMs <= maxEdadFotoMs;
    if(ignorarCamara)
      e.psi = e.d = 0.f;
    else if(fresca && s.camaraValida && actualizarVision(s.psiCamara, s.dCamaraMm, signoPsi, signoD, e.psi, e.d))
      vision.ultimoPerceptoValido = s.tiempoFrame; // H1: con NaN/Inf o fuera de rango es un cuadro perdido, no cuenta como valido
    if(fresca && s.metaVisible && distanciaMetaValida(s.distanciaMetaMm) && s.distanciaMetaMm <= distanciaMetaSinCamara)
      vision.metaCerca = true;
    return e;
  }

  /** Lo que falta para una decision: el mando (wz = 0: la politica nunca recibe giro) y el reloj de fase. */
  inline void completarDecision(EntradaPaso& e, const float vxCmd, const float frecuencia, const float tMarcha)
  {
    e.vxCmd = vxCmd;
    e.wzCmd = 0.f;
    e.tMarcha = tMarcha;
    e.frecuencia = frecuencia;
  }

  /** Inclinacion del torso [rad] (angulo con la vertical). */
  inline float inclinacionTorso(const float roll, const float pitch)
  {
    return std::acos(std::clamp(std::cos(roll) * std::cos(pitch), -1.f, 1.f));
  }

  // -------------------------------------------------------------------------------------------------------------------
  // Limitador (secciones 6 y 6b.5): un ciclo de 12 ms.

  /**
   * Lleva "objetivos" hacia "deseados": margen interior a los limites (si el rango es mas angosto que dos margenes, el
   * centro) y paso maximo por ciclo (por ciclo de 12 ms fijo: si se pierde un ciclo, se queda corto, no largo). La
   * articulacion "espejo" no se limita: copia a "origen" (RHipYawPitch = LHipYawPitch, un solo motor fisico).
   * @param limita false = no se toca (la cabeza).
   * @param recorteVel, recorteLim Salida: se marca (nunca se borra) lo que este ciclo recorto cada limite.
   */
  template<std::size_t N>
  void limitarObjetivos(const std::array<float, N>& deseados, std::array<float, N>& objetivos, const std::array<bool, N>& limita,
                        const std::array<float, N>& minimos, const std::array<float, N>& maximos, const std::array<float, N>& pasos,
                        const float margen, const std::size_t origen, const std::size_t espejo,
                        std::array<bool, N>& recorteVel, std::array<bool, N>& recorteLim)
  {
    for(std::size_t j = 0; j < N; ++j)
    {
      if(!limita[j] || j == espejo)
        continue;
      const float pedido = std::isfinite(deseados[j]) ? deseados[j] : objetivos[j];
      const float minimo = minimos[j] + margen, maximo = maximos[j] - margen;
      const float deseado = minimo <= maximo ? std::clamp(pedido, minimo, maximo) : .5f * (minimos[j] + maximos[j]);
      if(deseado != pedido)
        recorteLim[j] = true;
      const float delta = deseado - objetivos[j];
      if(std::abs(delta) > pasos[j])
        recorteVel[j] = true;
      objetivos[j] += std::clamp(delta, -pasos[j], pasos[j]);
    }
    objetivos[espejo] = objetivos[origen];
  }
}
