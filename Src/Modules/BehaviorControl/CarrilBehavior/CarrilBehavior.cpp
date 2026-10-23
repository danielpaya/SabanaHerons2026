/**
 * @file CarrilBehavior.cpp
 *
 * Implementación del comportamiento de la carrera (ver CarrilBehavior.h).
 */

#include "CarrilBehavior.h"
#include "Streaming/Output.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(CarrilBehavior);

void CarrilBehavior::reiniciarCarrera()
{
  abortada = false;
  enSuelo = false;
  tiempoArranque = theFrameInfo.time;
  ultimoValido = 0;
  paradaFijada = false;
  terminado = false;
  distanciaRestante = 0.f;
}

void CarrilBehavior::update(MotionRequest& theMotionRequest)
{
  theMotionRequest.odometryData = theOdometryData;

  // Sin rigidez (o calibración): sentado y sin par en las articulaciones.
  if(theGameState.playerState == GameState::unstiff || theGameState.playerState == GameState::calibration)
  {
    enCarrera = false;
    theMotionRequest.motion = MotionRequest::playDead;
    return;
  }

  // Solo se corre en playing y activo; cualquier otro estado (initial, set, penalizado
  // con el pecho = PARO MANUAL, finished...) es estar de pie.
  if(!theGameState.isPlaying() || theGameState.playerState != GameState::active)
  {
    enCarrera = false;
    theMotionRequest.motion = MotionRequest::stand;
    return;
  }
  if(!enCarrera)
  {
    enCarrera = true;
    reiniciarCarrera();
  }

  // PARO POR INCLINACIÓN: se mantiene hasta la siguiente entrada en playing.
  if(!abortada
     && (std::abs(theInertialData.angle.x()) > maxInclinacion || std::abs(theInertialData.angle.y()) > maxInclinacion
         || theFallDownState.state == FallDownState::falling || theFallDownState.state == FallDownState::fallen))
  {
    abortada = true;
    OUTPUT_WARNING("CarrilBehavior: carrera abortada por inclinación/caída");
  }
  if(abortada)
  {
    // Mientras cae, "stand" deja actuar al FallEngine (playDead lo desactivaría); ya en
    // el suelo, playDead evita que se levante solo, salvo que se pida lo contrario. Se
    // memoriza: sin rigidez, el FallDownState puede volver a "upright" estando tumbado
    // (verificado en simulación) y pedir "stand" en el suelo forzaría las articulaciones.
    if(theFallDownState.state == FallDownState::fallen)
      enSuelo = true;
    theMotionRequest.motion = enSuelo && !levantarseTrasCaida ? MotionRequest::playDead : MotionRequest::stand;
    return;
  }

  // Espera inicial en pie (acaba de empezar playing: el operador retira la mano).
  if(theFrameInfo.getTimeSince(tiempoArranque) < esperaInicial)
  {
    theMotionRequest.motion = MotionRequest::stand;
    return;
  }

  // Carrera terminada: quedarse en pie.
  if(terminado)
  {
    theMotionRequest.motion = MotionRequest::stand;
    return;
  }

  if(theLowerCarrilPercept.valido)
    ultimoValido = theLowerCarrilPercept.tiempoFrame;
  // Dos relojes: la frescura de la imagen (para usar lo que dice el frame actual) y el
  // tiempo desde el último percepto de líneas VÁLIDO (tiempoFrame avanza cada imagen
  // aunque no se vea nada, así que por sí solo no dice si hay visión).
  const bool imagenFresca = theFrameInfo.getTimeSince(theLowerCarrilPercept.tiempoFrame) <= maxEdadPercepto;
  const bool lineasFrescas = imagenFresca && theLowerCarrilPercept.valido;

  // Meta: al verla cerca, programar la parada por odometría (la franja sale del campo
  // visual de la cámara inferior justo antes de pisarla, así que no se puede esperar a
  // "verla debajo"). Después, caminar lo que faltaba más el margen, y pararse. No
  // depende de ver las líneas: pegado a la meta solo se ve la franja roja.
  if(!paradaFijada && imagenFresca && theLowerCarrilPercept.metaVisible
     && theLowerCarrilPercept.distanciaMeta <= distanciaFijarParada)
  {
    paradaFijada = true;
    odometriaAlFijar = theOdometryData.translation;
    distanciaRestante = theLowerCarrilPercept.distanciaMeta + margenParada;
  }
  if(paradaFijada && (theOdometryData.translation - odometriaAlFijar).norm() >= distanciaRestante)
  {
    terminado = true;
    theMotionRequest.motion = MotionRequest::stand;
    return;
  }

  if(lineasFrescas)
  {
    // d > 0 o psi > 0 significan robot a la izquierda o girado a la izquierda:
    // en ambos casos hay que girar a la derecha (giro negativo).
    const float giro = std::clamp(-(kD * theLowerCarrilPercept.d + kPsi * theLowerCarrilPercept.psi),
                                  -maxGiro, maxGiro);
    // Cerca de la meta (o con la parada ya fijada), avanzar despacio.
    const bool frenando = paradaFijada
                          || (imagenFresca && theLowerCarrilPercept.metaVisible
                              && theLowerCarrilPercept.distanciaMeta <= distanciaFrenado);
    const float base = frenando ? avanceFrenado : velocidadAvance;
    const float avance = base * std::max(0.f, 1.f - frenoPorGiro * std::abs(giro));
    theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
    theMotionRequest.walkSpeed = Pose2f(giro, avance, 0.f);
  }
  else if(paradaFijada)
  {
    // Sin líneas (normal pegado a la meta): completar la distancia restante en recto.
    theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
    theMotionRequest.walkSpeed = Pose2f(0.f, avanceFrenado, 0.f);
  }
  else if(ultimoValido != 0 && theFrameInfo.getTimeSince(ultimoValido) <= maxEdadSinPercepto)
  {
    // Visión perdida hace poco: seguir recto y despacio hasta recuperarla.
    theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
    theMotionRequest.walkSpeed = Pose2f(0.f, avanceSinPercepto, 0.f);
  }
  else
    theMotionRequest.motion = MotionRequest::stand;
}

void CarrilBehavior::update(HeadMotionRequest& theHeadMotionRequest)
{
  // Cabeza fija: la cámara inferior ve el carril con la cabeza al frente.
  theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
  theHeadMotionRequest.cameraControlMode = HeadMotionRequest::lowerCamera;
  theHeadMotionRequest.pan = 0_deg;
  theHeadMotionRequest.tilt = inclinacionCabeza;
  theHeadMotionRequest.speed = 60_deg;
}
