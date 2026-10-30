/**
 * @file CarrilBehavior.cpp
 *
 * Implementación del comportamiento de la carrera (ver CarrilBehavior.h).
 */

#include "CarrilBehavior.h"
#include "Platform/SystemCall.h"
#include "Streaming/Output.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(CarrilBehavior);

void CarrilBehavior::reiniciarCarrera()
{
  abortada = false;
  abortadaPorCaida = false;
  enSuelo = false;
  detenida = false;
  caidas = 0;
  tiempoEstable = 0;
  tiempoReanudacion = 0;
  tiempoArranque = theFrameInfo.time;
  ultimoValido = 0;
  paradaFijada = false;
  terminado = false;
  distanciaRestante = 0.f;
  odometriaArranque = theOdometryData.translation;
  ultimoLogEstado = 0;
  parado = false;
}

bool CarrilBehavior::debeArrancar(bool& porSilbato)
{
  // Solo simulación: no hay botones ni silbato simulados. Arranca con
  // "dr module:CarrilBehavior:arrancar" en la consola de SimRobot (ver Config/Scenes/Carril.con).
  DEBUG_RESPONSE_ONCE("module:CarrilBehavior:arrancar")
  if(SystemCall::getMode() == SystemCall::simulatedRobot)
  {
    porSilbato = false;
    return true;
  }

  // Botón frontal de la cabeza (un solo toque). Con el central o el trasero pulsados
  // es el gesto de unstiff: se ignora.
  if(theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 1
     && !theEnhancedKeyStates.pressed[KeyStates::headMiddle]
     && !theEnhancedKeyStates.pressed[KeyStates::headRear])
  {
    porSilbato = false;
    return true;
  }

  // Silbato NUEVO: posterior al armado (más margen) y con confianza suficiente. Tras un
  // paro no cuenta (detenida), y con GameController activo solo en playing.
  unsigned tiempoSilbato = theWhistle.lastTimeWhistleDetected;
  float confianzaSilbato = theWhistle.confidenceOfLastWhistleDetection;
  if(silbatoSimulado > tiempoSilbato)
  {
    tiempoSilbato = silbatoSimulado;
    confianzaSilbato = 1.f;
  }
  porSilbato = arranquePorSilbato && !detenida
               && (!theGameState.gameControllerActive || theGameState.isPlaying())
               && static_cast<int>(tiempoSilbato - tiempoArmado) > margenSilbato
               && confianzaSilbato >= minConfianzaSilbato;
  return porSilbato;
}

bool CarrilBehavior::estableTrasCaida()
{
  // FallDownState "upright" también puede darse tumbado sin rigidez: por eso se exige
  // además un torso casi vertical.
  const bool estable = theFallDownState.state == FallDownState::upright
                       && std::abs(theInertialData.angle.x()) < inclinacionEstable
                       && std::abs(theInertialData.angle.y()) < inclinacionEstable;
  if(!estable)
  {
    tiempoEstable = 0;
    return false;
  }
  if(tiempoEstable == 0)
    tiempoEstable = theFrameInfo.time;
  return theFrameInfo.getTimeSince(tiempoEstable) >= tiempoEstableTrasCaida;
}

void CarrilBehavior::update(MotionRequest& theMotionRequest)
{
  // Registrar el disparo de simulación en cada frame: la consola solo acepta "dr" de
  // peticiones que ya conoce, y debeArrancar() solo se ejecuta estando armado.
  DECLARE_DEBUG_RESPONSE("module:CarrilBehavior:arrancar");

  // Solo simulación: en SimRobot el Whistle no llega al comportamiento (ni del audio ni del
  // GameController simulado). "dr module:CarrilBehavior:silbato" simula uno detectado ahora;
  // se atiende en cualquier estado, como un silbato real.
  DEBUG_RESPONSE_ONCE("module:CarrilBehavior:silbato")
    if(SystemCall::getMode() == SystemCall::simulatedRobot)
      silbatoSimulado = theFrameInfo.time;

  theMotionRequest.odometryData = theOdometryData;

  // Sin rigidez (o calibración): sentado y sin par en las articulaciones. Pasar por aquí y
  // volver a armar con el pecho es empezar de cero: el silbato vuelve a valer.
  if(theGameState.playerState == GameState::unstiff || theGameState.playerState == GameState::calibration)
  {
    detenida = false;
    enCarrera = false;
    armado = false;
    theMotionRequest.motion = MotionRequest::playDead;
    return;
  }

  // Cualquier estado no activo (penalizedManual = PARO MANUAL con el pecho, finished...)
  // es estar de pie. Si había carrera, queda detenida.
  if(theGameState.playerState != GameState::active)
  {
    if(enCarrera)
    {
      detenida = true;
      OUTPUT_WARNING("CarrilBehavior: paro manual, carrera detenida");
    }
    enCarrera = false;
    armado = false;
    theMotionRequest.motion = MotionRequest::stand;
    return;
  }

  // De pie y activo: armado. Solo cuentan silbatos posteriores a este momento.
  if(!armado)
  {
    armado = true;
    tiempoArmado = theFrameInfo.time;
  }

  // Arranque: silbato nuevo o botón de la cabeza. Cada arranque es una carrera nueva.
  // Tras un paro (detenida) sigue de pie y activo, y solo el botón la vuelve a arrancar
  // (debeArrancar no acepta el silbato con la carrera detenida).
  if(!enCarrera || detenida)
  {
    bool porSilbato = false;
    if(debeArrancar(porSilbato))
    {
      enCarrera = true;
      reiniciarCarrera();
      esperaArranque = porSilbato ? esperaTrasSilbato : esperaTrasBoton;
      if(porSilbato)
        OUTPUT_WARNING("CarrilBehavior: arranque por silbato (confianza " << theWhistle.confidenceOfLastWhistleDetection << ")");
      else
        OUTPUT_WARNING("CarrilBehavior: arranque por botón de la cabeza");
    }
    else if(!enCarrera)
    {
      theMotionRequest.motion = MotionRequest::stand;
      return;
    }
  }

  // PARO POR INCLINACIÓN O CAÍDA: se detiene la carrera (detenida). Solo una caída puede
  // reanudarse, y solo con reanudarTrasCaida.
  const bool inclinado = std::abs(theInertialData.angle.x()) > maxInclinacion
                         || std::abs(theInertialData.angle.y()) > maxInclinacion;
  const bool cayendo = theFallDownState.state == FallDownState::falling
                       || theFallDownState.state == FallDownState::fallen;
  if(!abortada && (inclinado || cayendo))
  {
    abortada = true;
    abortadaPorCaida = cayendo;
    detenida = true;
    if(cayendo)
      ++caidas;
    tiempoEstable = 0;
    OUTPUT_WARNING("CarrilBehavior: carrera abortada por " << (cayendo ? "caída" : "inclinación"));
  }
  else if(abortada && !abortadaPorCaida && cayendo)
  {
    // La inclinación suele superar maxInclinacion antes de que el FallDownState detecte la
    // caída: si el paro por inclinación acaba en caída, cuenta como caída.
    abortadaPorCaida = true;
    ++caidas;
    tiempoEstable = 0;
    OUTPUT_WARNING("CarrilBehavior: el paro por inclinación acabó en caída");
  }
  if(abortada)
  {
    // Recuperación tras caída: de pie y estable se reanuda la carrera SIN reiniciarla
    // (se conservan la parada fijada y lo demás). Sin líneas tras reanudar, se para.
    if(abortadaPorCaida && reanudarTrasCaida && levantarseTrasCaida && caidas <= maxCaidas && estableTrasCaida())
    {
      abortada = false;
      abortadaPorCaida = false;
      detenida = false;
      enSuelo = false;
      tiempoEstable = 0;
      tiempoReanudacion = theFrameInfo.time;
      ultimoValido = 0;
      OUTPUT_WARNING("CarrilBehavior: carrera reanudada tras la caída (" << caidas << ")");
    }
    else
    {
      // Mientras cae, "stand" deja actuar al FallEngine (playDead lo desactivaría); ya en
      // el suelo, playDead evita que se levante solo, salvo que se pida lo contrario. Se
      // memoriza: sin rigidez, el FallDownState puede volver a "upright" estando tumbado
      // (verificado en simulación) y pedir "stand" en el suelo forzaría las articulaciones.
      if(theFallDownState.state == FallDownState::fallen)
        enSuelo = true;
      theMotionRequest.motion = enSuelo && (!levantarseTrasCaida || caidas > maxCaidas) ? MotionRequest::playDead : MotionRequest::stand;
      return;
    }
  }

  // Espera en pie tras el arranque (el operador retira la mano).
  if(theFrameInfo.getTimeSince(tiempoArranque) < esperaArranque)
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

  if(theLowerCarrilPercept.valido && theLowerCarrilPercept.tiempoFrame != ultimoValido)
  {
    ultimoValido = theLowerCarrilPercept.tiempoFrame;
    odometriaUltimoValido = theOdometryData;
    dUltimo = theLowerCarrilPercept.d;
    psiUltimo = theLowerCarrilPercept.psi;
  }
  // Dos relojes: la frescura de la imagen (para usar lo que dice el frame actual) y el
  // tiempo desde el último percepto de líneas VÁLIDO (tiempoFrame avanza cada imagen
  // aunque no se vea nada, así que por sí solo no dice si hay visión).
  const bool imagenFresca = theFrameInfo.getTimeSince(theLowerCarrilPercept.tiempoFrame) <= maxEdadPercepto;
  const bool lineasFrescas = imagenFresca && theLowerCarrilPercept.valido;

  // Reanudada tras una caída: de pie hasta volver a ver las líneas (también con la parada
  // fijada: tras levantarse el rumbo es desconocido y no se anda a ciegas). Si no aparecen
  // en maxEdadSinPercepto, se detiene (de pie hasta un arranque nuevo).
  if(tiempoReanudacion != 0 && ultimoValido < tiempoReanudacion)
  {
    if(theFrameInfo.getTimeSince(tiempoReanudacion) > maxEdadSinPercepto)
    {
      abortada = true;
      abortadaPorCaida = false;
      detenida = true;
      OUTPUT_WARNING("CarrilBehavior: sin líneas tras la caída, se queda de pie");
    }
    theMotionRequest.motion = MotionRequest::stand;
    return;
  }

  // Meta: al verla cerca, programar la parada por odometría (la franja sale del campo
  // visual de la cámara inferior justo antes de pisarla, así que no se puede esperar a
  // "verla debajo"). Después, caminar lo que faltaba más el margen, y pararse. No
  // depende de ver las líneas: pegado a la meta solo se ve la franja roja.
  // Mientras la meta siga a la vista se vuelve a fijar con cada lectura: la odometría solo
  // cubre el último tramo, cuando la franja ya salió de la imagen.
  if(imagenFresca && theLowerCarrilPercept.metaVisible
     && theLowerCarrilPercept.distanciaMeta <= distanciaFijarParada)
  {
    if(!paradaFijada)
      OUTPUT_WARNING("CarrilBehavior: meta vista a " << static_cast<int>(theLowerCarrilPercept.distanciaMeta)
                     << " mm, parada fijada por odometría");
    paradaFijada = true;
    odometriaAlFijar = theOdometryData.translation;
    distanciaRestante = theLowerCarrilPercept.distanciaMeta + margenParada;
  }
  if(paradaFijada && (theOdometryData.translation - odometriaAlFijar).norm() >= distanciaRestante)
  {
    terminado = true;
    detenida = true;
    OUTPUT_WARNING("CarrilBehavior: parada en meta tras " << static_cast<int>((theOdometryData.translation - odometriaAlFijar).norm())
                   << " mm desde la última lectura de la meta");
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
  else if(paradaFijada || (ultimoValido != 0 && theFrameInfo.getTimeSince(ultimoValido) <= maxEdadSinPercepto))
  {
    // Sin líneas (pegado a la meta, o visión perdida hace poco): d y psi se estiman por
    // odometría desde el último percepto válido, con el mismo control. "Recto" sin
    // corregir dejaba que la deriva de la marcha lo sacara del carril (robot real, 0.5).
    const Angle ejeCarril = odometriaUltimoValido.rotation - psiUltimo; // eje del carril en el marco odométrico
    const Vector2f normalIzq(-std::sin(ejeCarril), std::cos(ejeCarril));
    const float dEst = dUltimo + (theOdometryData.translation - odometriaUltimoValido.translation).dot(normalIzq);
    const Angle psiEst = Angle::normalize(theOdometryData.rotation - ejeCarril);
    const float giro = std::clamp(-(kD * dEst + kPsi * psiEst), -maxGiro, maxGiro);
    const float base = paradaFijada ? avanceFrenado : avanceSinPercepto;
    theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
    theMotionRequest.walkSpeed = Pose2f(giro, base * std::max(0.f, 1.f - frenoPorGiro * std::abs(giro)), 0.f);
  }
  else
    theMotionRequest.motion = MotionRequest::stand;

  // Diagnóstico en el robot real (bhumand.log): por qué se queda de pie y qué ve.
  const bool dePie = theMotionRequest.motion == MotionRequest::stand;
  if(dePie && !parado)
    OUTPUT_WARNING("CarrilBehavior: sin líneas desde hace " << theFrameInfo.getTimeSince(ultimoValido)
                   << " ms, se queda de pie");
  parado = dePie;
  if(ultimoLogEstado == 0 || theFrameInfo.getTimeSince(ultimoLogEstado) >= 1000)
  {
    ultimoLogEstado = theFrameInfo.time;
    OUTPUT_WARNING("CarrilBehavior: estado odo=" << static_cast<int>((theOdometryData.translation - odometriaArranque).norm())
                   << "mm valido=" << theLowerCarrilPercept.valido
                   << " lineas=" << theLowerCarrilPercept.numLineas
                   << " d=" << static_cast<int>(theLowerCarrilPercept.d)
                   << " psi=" << static_cast<int>(theLowerCarrilPercept.psi.toDegrees())
                   << " meta=" << theLowerCarrilPercept.metaVisible
                   << "/" << static_cast<int>(theLowerCarrilPercept.distanciaMeta)
                   << " edad=" << theFrameInfo.getTimeSince(theLowerCarrilPercept.tiempoFrame)
                   << " giro=" << theMotionRequest.walkSpeed.rotation
                   << " avance=" << theMotionRequest.walkSpeed.translation.x()
                   << (dePie ? " DE_PIE" : ""));
  }
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
