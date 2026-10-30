/**
 * @file CarrilBehavior.h
 *
 * Comportamiento de la Carrera Individual (Copa NAO CCM MX 2026): tras armarse de pie,
 * camina siguiendo el eje del carril corrigiendo el rumbo con el percepto de la cámara
 * inferior (CarrilPercept: d, psi) y se para tras la meta roja.
 * Sustituye a SkillBehaviorControl solo como proveedor de MotionRequest y
 * HeadMotionRequest en el escenario Carril (ver Config/Scenarios/Carril/threads.cfg).
 *
 * Estados (el pecho NUNCA arranca la carrera):
 *   - Sin rigidez (unstiff): playDead. Un toque al pecho -> de pie y armado.
 *   - Armado (de pie, playerState active, sin carrera): espera el arranque.
 *   - Arranque: un silbato NUEVO (posterior al armado + margenSilbato, con confianza
 *     >= minConfianzaSilbato, solo si arranquePorSilbato) o un toque al botón frontal
 *     de la cabeza (ignorado si el central o el trasero están pulsados: ese es el gesto
 *     de unstiff). Tras el arranque se espera esperaTrasSilbato o esperaTrasBoton.
 *   - Corriendo: ley de control. Cada arranque es una carrera nueva.
 *   - PARO MANUAL: toque al pecho durante la carrera (penalizedManual: de pie). Deja la
 *     carrera "detenida": ni sola ni con silbato se reanuda; solo con un toque al botón
 *     frontal de la cabeza (con el robot de nuevo armado: otro toque al pecho), o pasando
 *     por sin rigidez y volviendo a armar. Los tres botones de la cabeza -> sin rigidez.
 *   - PARO POR INCLINACIÓN: torso más inclinado que maxInclinacion. Detenida.
 *   - CAÍDA: al caer se aborta la carrera (detenida). Con reanudarTrasCaida (y
 *     levantarseTrasCaida), tras levantarse y estar estable (tiempoEstableTrasCaida) se
 *     reanuda SIN reiniciar la carrera (conserva la parada fijada y el resto), hasta
 *     maxCaidas caídas; después, sin rigidez en el suelo. Tras reanudar no anda hasta
 *     ver las líneas; si no las ve en maxEdadSinPercepto, se queda de pie (detenida).
 *   - META: al llegar a la parada por odometría, de pie; detenida.
 *   Tras cualquier paro, el botón de la cabeza arranca una carrera nueva.
 *   Con GameController activo, el silbato exige además que el GC esté en playing.
 *   Solo simulación: "dr module:CarrilBehavior:arrancar" equivale al botón de la cabeza y
 *   "dr module:CarrilBehavior:silbato" a un silbato detectado en ese momento.
 *
 * Ley de control (ganancias en carrilBehavior.cfg): giro = -(kD * d + kPsi * psi),
 * recortado a maxGiro; el avance se reduce proporcionalmente al giro. Sin percepto
 * fresco camina recto y despacio; si la antigüedad supera maxEdadSinPercepto, se para.
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/Perception/Carril/CarrilPercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"

MODULE(CarrilBehavior,
{,
  REQUIRES(EnhancedKeyStates),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(InertialData),
  REQUIRES(LowerCarrilPercept),
  REQUIRES(OdometryData),
  REQUIRES(Whistle),
  PROVIDES(MotionRequest),
  PROVIDES(HeadMotionRequest),
  LOADS_PARAMETERS(
  {,
    (bool) arranquePorSilbato,       /**< Arrancar con un silbato nuevo (además del botón de la cabeza). */
    (int) margenSilbato,             /**< Un silbato solo cuenta si es posterior al armado + este margen (ms). */
    (float) minConfianzaSilbato,     /**< Confianza mínima del silbato para arrancar (0..1). */
    (int) esperaTrasBoton,           /**< Tiempo en pie tras el toque al botón de la cabeza, para retirar la mano (ms). */
    (int) esperaTrasSilbato,         /**< Tiempo en pie tras el silbato antes de andar (ms). */
    (float) velocidadAvance,         /**< Avance como fracción de la velocidad máxima (0..1). */
    (float) kD,                      /**< Ganancia del desplazamiento lateral (giro por mm). */
    (float) kPsi,                    /**< Ganancia de la orientación (giro por radián). */
    (float) maxGiro,                 /**< Giro máximo como fracción de la velocidad máxima (0..1). */
    (float) frenoPorGiro,            /**< Reducción del avance por unidad de giro (0..1). */
    (int) maxEdadPercepto,           /**< Antigüedad máxima del percepto para corregir rumbo (ms). */
    (int) maxEdadSinPercepto,        /**< Sin percepto válido durante este tiempo, pararse (ms). */
    (float) avanceSinPercepto,       /**< Avance mientras no hay percepto fresco (0..1). */
    (Angle) inclinacionCabeza,       /**< Ángulo de la cámara inferior bajo la horizontal (positivo = abajo; 0 = mira al horizonte). */
    (float) distanciaFrenado,        /**< Con la meta más cerca que esto (mm), reducir el avance. */
    (float) avanceFrenado,           /**< Avance durante el frenado (0..1). */
    (float) distanciaFijarParada,    /**< Con la meta más cerca que esto (mm), fijar la parada por odometría. */
    (float) margenParada,            /**< Distancia extra tras el centro de la meta antes de pararse (mm). */
    (Angle) maxInclinacion,          /**< Paro: inclinación máxima del torso (cabeceo o alabeo) durante la carrera. */
    (bool) levantarseTrasCaida,      /**< Tras un paro por caída: true = levantarse; false = quedarse sin rigidez. */
    (bool) reanudarTrasCaida,        /**< Tras levantarse y estar estable, seguir la carrera sin reiniciarla. */
    (int) maxCaidas,                 /**< Caídas que se reanudan; tras la siguiente, sin rigidez en el suelo. */
    (int) tiempoEstableTrasCaida,    /**< Tiempo de pie y estable antes de reanudar (ms). */
    (Angle) inclinacionEstable,      /**< Inclinación del torso que se considera estable para reanudar. */
  }),
});

class CarrilBehavior : public CarrilBehaviorBase
{
  void update(MotionRequest& theMotionRequest) override;
  void update(HeadMotionRequest& theHeadMotionRequest) override;

  /** Deja todo listo para una carrera nueva (al arrancar por silbato o botón). */
  void reiniciarCarrera();

  /** Comprueba si hay que arrancar (silbato nuevo o botón de la cabeza) estando armado. */
  bool debeArrancar(bool& porSilbato);

  /** Tras una caída: true cuando lleva el tiempo de estabilidad (se reanuda la carrera). */
  bool estableTrasCaida();

  bool armado = false;           /**< De pie y activo el frame anterior. */
  bool enCarrera = false;        /**< Corriendo: arrancada y no parada. */
  bool detenida = false;         /**< Paro (manual, inclinación, meta): solo el botón de la cabeza la reanuda. */
  bool abortada = false;         /**< Paro por inclinación o caída: sin moverse hasta reanudar o siguiente arranque. */
  bool abortadaPorCaida = false; /**< La parada abortada es por caída (y puede reanudarse). */
  bool enSuelo = false;          /**< Tras el paro, el robot llegó a estar caído: sin rigidez hasta levantarse. */
  unsigned tiempoArmado = 0;     /**< Momento de armarse: solo cuentan silbatos posteriores. */
  unsigned tiempoArranque = 0;   /**< Momento del arranque, para la espera inicial. */
  int esperaArranque = 0;        /**< Espera en pie tras el arranque elegido (ms). */
  unsigned tiempoEstable = 0;    /**< Desde cuándo está de pie y estable tras una caída (0 = no). */
  unsigned tiempoReanudacion = 0; /**< Momento de la última reanudación tras caída (0 = ninguna). */
  int caidas = 0;                /**< Caídas en la carrera actual. */
  unsigned silbatoSimulado = 0;  /**< Solo simulación: momento del último silbato simulado (0 = ninguno). */
  unsigned ultimoValido = 0;     /**< Momento del último percepto VÁLIDO (no del último frame). */
  bool paradaFijada = false;     /**< La meta se vio cerca: parada programada por odometría. */
  bool terminado = false;        /**< Carrera terminada: quedarse en pie. */
  Vector2f odometriaAlFijar;     /**< Posición odométrica al fijar la parada. */
  float distanciaRestante = 0.f; /**< Lo que faltaba por andar al fijar la parada (mm). */
  Vector2f odometriaArranque;    /**< Posición odométrica al arrancar (para el log de estado). */
  unsigned ultimoLogEstado = 0;  /**< Último log de estado de la carrera (robot real: 1 por segundo). */
  Pose2f odometriaUltimoValido;  /**< Odometría en el último percepto válido (estima d y psi a ciegas). */
  float dUltimo = 0.f;           /**< d del último percepto válido (mm). */
  Angle psiUltimo = 0_deg;       /**< psi del último percepto válido. */
  bool parado = false;           /**< El frame anterior la carrera pidió quedarse de pie (sin líneas). */
};
