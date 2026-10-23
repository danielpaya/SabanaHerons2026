/**
 * @file CarrilBehavior.h
 *
 * Comportamiento de la Carrera Individual (Copa NAO CCM MX 2026): tras una espera
 * inicial en pie, camina siguiendo el eje del carril corrigiendo el rumbo con el
 * percepto de la cámara inferior (CarrilPercept: d, psi) y se para tras la meta roja.
 * Sustituye a SkillBehaviorControl solo como proveedor de MotionRequest y
 * HeadMotionRequest en el escenario Carril (ver Config/Scenarios/Carril/threads.cfg).
 *
 * Arranque y paros (los del GameStateProvider, los mismos botones que en un partido):
 *   - sin rigidez (unstiff): playDead. Un toque al pecho -> de pie (initial).
 *   - otro toque al pecho -> playing: empieza la carrera (cada entrada en playing es
 *     una carrera nueva: se reinicia todo, también la parada en meta y el paro).
 *   - PARO MANUAL: toque al pecho durante la carrera -> penalizedManual: de pie.
 *     Mantener los tres botones de la cabeza -> sin rigidez (playDead).
 *   - PARO POR INCLINACIÓN: si el torso se inclina más de maxInclinacion o el robot
 *     cae, la carrera se aborta hasta el siguiente playing: de pie mientras no esté
 *     en el suelo (el FallEngine sigue protegiendo la caída) y playDead ya caído,
 *     salvo que levantarseTrasCaida pida levantarse.
 *   Con GameController, sus estados mandan igual (solo se corre en playing).
 *
 * Ley de control (ganancias en carrilBehavior.cfg): giro = -(kD * d + kPsi * psi),
 * recortado a maxGiro; el avance se reduce proporcionalmente al giro. Sin percepto
 * fresco camina recto y despacio; si la antigüedad supera maxEdadSinPercepto, se para.
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/Carril/CarrilPercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"

MODULE(CarrilBehavior,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(InertialData),
  REQUIRES(LowerCarrilPercept),
  REQUIRES(OdometryData),
  PROVIDES(MotionRequest),
  PROVIDES(HeadMotionRequest),
  LOADS_PARAMETERS(
  {,
    (int) esperaInicial,          /**< Tiempo en pie desde que empieza playing hasta echar a andar (ms). */
    (float) velocidadAvance,      /**< Avance como fracción de la velocidad máxima (0..1). */
    (float) kD,                   /**< Ganancia del desplazamiento lateral (giro por mm). */
    (float) kPsi,                 /**< Ganancia de la orientación (giro por radián). */
    (float) maxGiro,              /**< Giro máximo como fracción de la velocidad máxima (0..1). */
    (float) frenoPorGiro,         /**< Reducción del avance por unidad de giro (0..1). */
    (int) maxEdadPercepto,        /**< Antigüedad máxima del percepto para corregir rumbo (ms). */
    (int) maxEdadSinPercepto,     /**< Sin percepto válido durante este tiempo, pararse (ms). */
    (float) avanceSinPercepto,    /**< Avance mientras no hay percepto fresco (0..1). */
    (Angle) inclinacionCabeza,    /**< Ángulo de la cámara inferior bajo la horizontal (positivo = abajo; 0 = mira al horizonte). */
    (float) distanciaFrenado,     /**< Con la meta más cerca que esto (mm), reducir el avance. */
    (float) avanceFrenado,        /**< Avance durante el frenado (0..1). */
    (float) distanciaFijarParada, /**< Con la meta más cerca que esto (mm), fijar la parada por odometría. */
    (float) margenParada,         /**< Distancia extra tras el centro de la meta antes de pararse (mm). */
    (Angle) maxInclinacion,       /**< Paro: inclinación máxima del torso (cabeceo o alabeo) durante la carrera. */
    (bool) levantarseTrasCaida,   /**< Tras un paro por caída: true = levantarse; false = quedarse sin rigidez. */
  }),
});

class CarrilBehavior : public CarrilBehaviorBase
{
  void update(MotionRequest& theMotionRequest) override;
  void update(HeadMotionRequest& theHeadMotionRequest) override;

  /** Deja todo listo para una carrera nueva (al entrar en playing). */
  void reiniciarCarrera();

  bool enCarrera = false;        /**< Estuvo en playing el frame anterior. */
  bool abortada = false;         /**< Paro por inclinación o caída: sin moverse hasta el siguiente playing. */
  bool enSuelo = false;          /**< Tras el paro, el robot llegó a estar caído: sin rigidez hasta el siguiente playing. */
  unsigned tiempoArranque = 0;   /**< Momento de entrada en playing, para la espera inicial. */
  unsigned ultimoValido = 0;     /**< Momento del último percepto VÁLIDO (no del último frame). */
  bool paradaFijada = false;     /**< La meta se vio cerca: parada programada por odometría. */
  bool terminado = false;        /**< Carrera terminada: quedarse en pie. */
  Vector2f odometriaAlFijar;     /**< Posición odométrica al fijar la parada. */
  float distanciaRestante = 0.f; /**< Lo que faltaba por andar al fijar la parada (mm). */
};
