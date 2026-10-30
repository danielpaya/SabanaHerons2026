/**
 * @file NaoRLWalkEngine.cpp
 *
 * Ver NaoRLWalkEngine.h y demo_carril/docs/despliegue_nao_v08.md.
 */

#include "NaoRLWalkEngine.h"
#include "Platform/SystemCall.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Streaming/Output.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/NaoRL/NaoRLPoliticaConfig.h"
#include <chrono>
#include <cstdio>
#include <iostream>

MAKE_MODULE(NaoRLWalkEngine);

namespace
{
  const char* nombreEstado(const int e)
  {
    static const char* nombres[] = {"interpolando", "enHome", "caminando", "parado"};
    return nombres[e];
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Modulo

NaoRLWalkEngine::NaoRLWalkEngine()
{
  // Parametros por motor (lo, hi, velMax vienen del golden: el .cfg de la red no los trae).
  parametrosValidos = lo.size() == NaoRL::nMotores && hi.size() == NaoRL::nMotores && velMax.size() == NaoRL::nMotores &&
                      stiffnessPiernas >= 0 && stiffnessPiernas <= 100 && stiffnessBrazos >= 0 && stiffnessBrazos <= 100;
  for(std::size_t i = 0; parametrosValidos && i < NaoRL::nMotores; ++i)
  {
    parametrosValidos = std::isfinite(lo[i]) && std::isfinite(hi[i]) && lo[i] < hi[i] && velMax[i] > 0.f && velMax[i] < 10.f;
    loAccion[i] = lo[i];
    hiAccion[i] = hi[i];
  }

  std::string errorParametros;
  if(!parametrosValidos)
    errorParametros = "lo/hi/velMax deben tener 15 valores validos y las rigideces estar en [0, 100]";
  else
  {
    // H4 + H5(b): tabla de rangos en NaoRLGuardas.h.
    NaoRL::ValoresSeguridad valores{};
    valores[NaoRL::pMaxInclinacion] = maxInclinacion;
    valores[NaoRL::pMaxTemperatura] = static_cast<float>(maxTemperatura);
    valores[NaoRL::pUmbralSeguimiento] = umbralSeguimiento;
    valores[NaoRL::pUmbralSeguimientoRoll] = umbralSeguimientoRoll;
    valores[NaoRL::pCiclosSeguimiento] = static_cast<float>(ciclosSeguimiento);
    valores[NaoRL::pUmbralCorriente] = static_cast<float>(umbralCorriente);
    valores[NaoRL::pCiclosCorriente] = static_cast<float>(ciclosCorriente);
    valores[NaoRL::pMargenLimites] = margenLimites;
    valores[NaoRL::pFactorVelRoll] = factorVelRoll;
    valores[NaoRL::pStiffnessParo] = static_cast<float>(stiffnessParo);
    valores[NaoRL::pPresupuestoInferenciaUs] = static_cast<float>(presupuestoInferenciaUs);
    valores[NaoRL::pVxCmd] = vxCmd;
    valores[NaoRL::pRetardoSeguimiento] = static_cast<float>(retardoSeguimiento);
    valores[NaoRL::pMaxFallosNaN] = static_cast<float>(maxFallosNaN);
    valores[NaoRL::pMaxExcesosInferencia] = static_cast<float>(maxExcesosInferencia);
    valores[NaoRL::pMaxEdadCamaraMs] = static_cast<float>(maxEdadCamaraMs);
    valores[NaoRL::pMaxMarchaMs] = static_cast<float>(maxMarchaMs);
    valores[NaoRL::pMaxCorrienteHome] = static_cast<float>(maxCorrienteHome);
    parametrosValidos = NaoRL::validarSeguridad(valores, ignorarLevantado, errorParametros);
  }

  // Mapeo por nombre: motor de la politica <-> articulacion de B-Human.
  const bool mapeoValido = NaoRL::mapearMotores(mapa);

  std::string error;
  NaoRL::PoliticaCargada cargada;
  if(!mapeoValido)
    error = "no se pudo mapear un motor de la politica a Joints";
  else if(!parametrosValidos)
    error = "naoRLWalkEngine.cfg: " + errorParametros;
  else if(NaoRL::cargarPolitica("NeuralNets/NaoRL/" + politica + ".cfg", cargada, error))
  {
    red = std::move(cargada.politica);
    postura = cargada.postura;
    escala = cargada.escala;
    politicaValida = true;
  }

  home = NaoRL::poseHome(postura, mapa);

  // H5(a): una bandera de prueba activa se anuncia en voz alta y queda en la cabecera del registro.
  const bool enModoPrueba = NaoRL::modoPrueba(ignorarCamara, ignorarLevantado);
  if(enModoPrueba)
  {
    OUTPUT_WARNING("NaoRLWalkEngine: MODO PRUEBA ignorarCamara=" << ignorarCamara << " ignorarLevantado=" << ignorarLevantado);
    SystemCall::say("test mode");
  }

  if(usarPoliticaRL && politicaValida)
  {
    std::string cabecera = enModoPrueba ? "# MODO PRUEBA: ignorarCamara=" + std::to_string(ignorarCamara) + " ignorarLevantado=" +
                                          std::to_string(ignorarLevantado) + "\n" : std::string();
    cabecera += "t_ms,estado,decision,t_inferencia_us,fallDown,inclinacion,gyro_x,gyro_y,gyro_z,arriba_x,arriba_y,"
                           "arriba_z,psi_obs,d_obs,edad_camara_ms,paro";
    for(int j = Joints::firstLegJoint; j < Joints::numOfJoints; ++j)
    {
      const std::string n = TypeRegistry::getEnumName(static_cast<Joints::Joint>(j));
      cabecera += ",obj_" + n + ",ang_" + n + ",corr_" + n + ",temp_" + n;
    }
    for(std::size_t i = 0; i < NaoRL::nMotores; ++i)
      cabecera += std::string(",a_") + NaoRL::nombresMotores[i];
    if(!registro.abrir(rutaRegistro, cabecera))
      OUTPUT_WARNING("NaoRLWalkEngine: no se pudo abrir el registro en " << rutaRegistro);
  }

  if(usarPoliticaRL && !politicaValida)
  {
    OUTPUT_ERROR("NaoRLWalkEngine: politica " << politica << " NO cargada: " << error);
    SystemCall::say(parametrosValidos ? "R L policy missing" : "R L parameters invalid");
  }
  std::cout << "[NaoRL] usarPoliticaRL=" << (usarPoliticaRL ? "true" : "false") << " politica=" << politica
            << (politicaValida ? " cargada" : " NO cargada (" + error + ")") << (enModoPrueba ? " MODO PRUEBA" : "") << " vxCmd=" << vxCmd
            << " registro=" << (registro.abierto() ? registro.archivo() : std::string("ninguno")) << std::endl;
}

float NaoRLWalkEngine::inclinacion() const
{
  return NaoRL::inclinacionTorso(theInertialData.angle.x(), theInertialData.angle.y());
}

void NaoRLWalkEngine::enclavar(const Paro tipo, const std::string& motivo)
{
  if(static_cast<int>(tipo) <= static_cast<int>(paro))
    return;
  paro = tipo;
  motivoParo = motivo;
  const char* tipoTexto = tipo == Paro::duro ? "DURO" : "suave";
  OUTPUT_WARNING("NaoRLWalkEngine: PARO " << tipoTexto << ": " << motivo);
  std::cout << "[NaoRL] t=" << theFrameInfo.time << " PARO " << tipoTexto << ": " << motivo << " (inclinacion "
            << inclinacion() << " rad)" << std::endl;
  registro.escribir("# t=" + std::to_string(theFrameInfo.time) + " PARO " + tipoTexto + ": " + motivo);
  SystemCall::say(tipo == Paro::duro ? "safety stop" : "stop");
}

void NaoRLWalkEngine::update(WalkAtRelativeSpeedGenerator& generator)
{
  // El paro se desenclava solo al volver a armar (flanco de active). No se puede usar "entrada en playing": al armar desde
  // sin rigidez GameStateProvider deja el estado en initial, y CarrilBehavior arranca con el boton o el silbato igualmente;
  // el paro seguiria enclavado y la carrera nueva naceria con la fase "parada", soltando a un robot de pie.
  const bool activo = theGameState.playerState == GameState::active;
  if(activo && !estabaActivo && paro != Paro::ninguno)
  {
    std::cout << "[NaoRL] robot rearmado: se desenclava el paro (" << motivoParo << ")" << std::endl;
    paro = Paro::ninguno;
    motivoParo.clear();
  }
  estabaActivo = activo;

  // Caida mientras la fase RL estaba activa: el FallEngine reemplaza la fase en ese mismo ciclo, asi que el paro se
  // enclava aqui (la siguiente fase RL nacera "parada", sin rigidez).
  if(tUltimaFase && theFrameInfo.getTimeSince(tUltimaFase) <= 100 &&
     (theFallDownState.state == FallDownState::falling || theFallDownState.state == FallDownState::fallen))
    enclavar(Paro::duro, "caida");

  generator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase) -> std::unique_ptr<MotionPhase>
  {
    // Sin politica, sin armar o tras un paro suave: el stand de B-Human. Tras un paro duro: fase RL "parada".
    if(!usarPoliticaRL || !politicaValida || paro == Paro::suave)
      return theStandGenerator.createPhase(motionRequest, lastPhase);
    return std::make_unique<NaoRLWalkPhase>(*this);
  };
}

// ---------------------------------------------------------------------------------------------------------------------
// Fase

NaoRLWalkPhase::NaoRLWalkPhase(NaoRLWalkEngine& engine) :
  MotionPhase(MotionPhase::calibration),
  engine(engine)
{
  tInicio = tAnterior = engine.theFrameInfo.time;
  for(int j = 0; j < Joints::numOfJoints; ++j)
  {
    const float angulo = engine.theJointSensorData.angles[j];
    anguloInicial[j] = anguloValido[j] = std::isfinite(angulo) && angulo != SensorData::off ? angulo : engine.home[j];
  }
  deseados = objetivos = anguloInicial;
  for(int j = 0; j < Joints::numOfJoints; ++j)
    objetivos[j] = NaoRL::objetivoInicial(anguloInicial[j], engine.theJointLimits.limits[j].min, engine.theJointLimits.limits[j].max);
  objetivosPasados.fill(anguloInicial);
  decisor.reiniciar();
  if(engine.paro == NaoRLWalkEngine::Paro::duro)
    estado = Estado::parado;
}

void NaoRLWalkPhase::revisarSeguridad()
{
  const float incl = engine.inclinacion();
  if(NaoRL::inclinacionExcedida(incl, engine.maxInclinacion)) // H5(c): ninguna bandera de prueba quita este paro
    engine.enclavar(NaoRLWalkEngine::Paro::duro, "inclinacion " + std::to_string(incl) + " rad");

  const FallDownState::State caida = engine.theFallDownState.state;
  if(caida == FallDownState::falling || caida == FallDownState::fallen)
    engine.enclavar(NaoRLWalkEngine::Paro::duro, std::string("caida (") + TypeRegistry::getEnumName(caida) + ")");
  else if(caida == FallDownState::pickedUp && !engine.ignorarLevantado)
    engine.enclavar(NaoRLWalkEngine::Paro::duro, "levantado");

  for(int j = Joints::firstLegJoint; j < Joints::numOfJoints; ++j)
  {
    const char* nombre = TypeRegistry::getEnumName(static_cast<Joints::Joint>(j));
    if(engine.theJointSensorData.temperatures[j] >= engine.maxTemperatura)
      engine.enclavar(NaoRLWalkEngine::Paro::duro, std::string("temperatura ") + nombre + " " +
                      std::to_string(engine.theJointSensorData.temperatures[j]) + " C");

    // Corriente sostenida (SensorData::off = sin sensor, p. ej. en simulacion).
    const short corriente = engine.theJointSensorData.currents[j];
    if(corriente != static_cast<short>(SensorData::off) && std::abs(corriente) > engine.umbralCorriente)
    {
      if(++cuentaCorriente[j] >= engine.ciclosCorriente)
        engine.enclavar(NaoRLWalkEngine::Paro::duro, std::string("corriente ") + nombre + " " + std::to_string(corriente) + " mA");
    }
    else
      cuentaCorriente[j] = 0;

    // Error de seguimiento contra el objetivo mandado hace retardoSeguimiento ciclos (6b.2).
    const std::size_t retardo = static_cast<std::size_t>(std::clamp(engine.retardoSeguimiento, 0, static_cast<int>(nRetardo) - 1));
    const float error = std::abs(objetivosPasados[retardo][j] - anguloValido[j]);
    const float umbral = NaoRL::esRollPierna(j) ? engine.umbralSeguimientoRoll : engine.umbralSeguimiento;
    if(!primerCiclo && error > umbral)
    {
      if(++cuentaSeguimiento[j] >= engine.ciclosSeguimiento)
        engine.enclavar(NaoRLWalkEngine::Paro::duro, std::string("seguimiento ") + nombre + " " + std::to_string(error) + " rad");
    }
    else
      cuentaSeguimiento[j] = 0;
  }

  if(estado == Estado::caminando)
  {
    if(engine.theFrameInfo.getTimeSince(tMarcha) > engine.maxMarchaMs)
      engine.enclavar(NaoRLWalkEngine::Paro::suave, "tiempo maximo de marcha");
    if(!engine.ignorarCamara && !vision.metaCerca && engine.theFrameInfo.getTimeSince(vision.ultimoPerceptoValido) > engine.maxEdadCamaraMs)
      engine.enclavar(NaoRLWalkEngine::Paro::suave, "camara: sin carril " + std::to_string(engine.theFrameInfo.getTimeSince(vision.ultimoPerceptoValido)) + " ms");
  }
}

std::string NaoRLWalkPhase::autoverificar() const
{
  std::array<float, NaoRL::nPiernas> angulos, posturaHome;
  std::array<int, NaoRL::nPiernas> temperaturas;
  for(std::size_t k = 0; k < NaoRL::nPiernas; ++k)
  {
    const int j = Joints::firstLegJoint + static_cast<int>(k);
    angulos[k] = engine.theJointSensorData.angles[j]; // crudo: un NaN o SensorData::off tambien falla
    posturaHome[k] = engine.home[j];
    temperaturas[k] = engine.theJointSensorData.temperatures[j];
  }
  const float corrienteMedia = muestrasCorrienteHome ? sumaCorrienteHome / static_cast<float>(muestrasCorrienteHome) : -1.f;
  return NaoRL::autoverificacionHome(angulos, posturaHome, temperaturas, engine.inclinacion(), entrada.gyro, corrienteMedia,
                                     SystemCall::getMode() == SystemCall::physicalRobot, static_cast<float>(engine.maxCorrienteHome));
}

void NaoRLWalkPhase::decidir()
{
  NaoRL::EntradaPaso e = entrada;
  NaoRL::completarDecision(e, engine.vxCmd, engine.frecuenciaMarcha, engine.theFrameInfo.getTimeSince(tMarcha) / 1000.f);
  const char* motivoEntrada = NaoRL::motivoEntrada(fallaAngulo, e, engine.theJointSensorData, engine.theJointLimits, engine.mapa);

  const auto t0 = std::chrono::steady_clock::now();
  const NaoRL::Decisor::Resultado resultado = decisor.decidir(engine.red, e, engine.postura, motivoEntrada, engine.maxFallosNaN);
  ultimaInferenciaUs = std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - t0).count();
  NaoRL::Accion accion = resultado.accion;

  // Sensores o red mal: se repite la ultima accion valida; maxFallosNaN seguidas -> paro.
  if(resultado.paro)
    engine.enclavar(NaoRLWalkEngine::Paro::duro, std::string(resultado.motivo) + " en " + std::to_string(decisor.fallosSeguidos()) + " decisiones seguidas");

  // Watchdog: una inferencia fuera de presupuesto se descarta (se repite la ultima accion).
  if(ultimaInferenciaUs > static_cast<float>(engine.presupuestoInferenciaUs))
  {
    accion = decisor.descartarUltima();
    if(++excesosInferencia >= engine.maxExcesosInferencia)
      engine.enclavar(NaoRLWalkEngine::Paro::duro, "watchdog: inferencia de " + std::to_string(ultimaInferenciaUs) + " us");
  }
  else
    excesosInferencia = 0;

  deseados = NaoRL::deseadosDesdeObjetivos(NaoRL::accionAObjetivos(accion, engine.postura, engine.escala, engine.loAccion, engine.hiAccion),
                                           engine.home, engine.mapa);

  ++decisiones;
}

void NaoRLWalkPhase::update()
{
  const unsigned ahora = engine.theFrameInfo.time;
  dtUltimo = std::max(0.f, engine.theFrameInfo.getTimeSince(tAnterior) / 1000.f);
  tAnterior = ahora;
  engine.tUltimaFase = ahora;

  // Angulos leidos: los no finitos se sustituyen por el ultimo valido y cuentan como falla para la decision.
  fallaAngulo = NaoRL::actualizarAngulosValidos(engine.theJointSensorData, anguloValido);
  entrada = NaoRL::leerSensores(NaoRL::sensoresPaso(engine.theInertialData, engine.theLowerCarrilPercept, engine.theFrameInfo, anguloValido, engine.mapa),
                                entrada, engine.ignorarCamara, engine.distanciaMetaSinCamara, vision);

  if(estado != Estado::parado)
    revisarSeguridad();
  if(engine.paro == NaoRLWalkEngine::Paro::duro)
    estado = Estado::parado;

  float tInferencia = -1.f;
  bool registrarAhora = ciclo % NaoRL::ciclosPorDecision == 0;
  switch(estado)
  {
    case Estado::interpolando:
    {
      const float a = std::clamp(engine.theFrameInfo.getTimeSince(tInicio) / static_cast<float>(std::max(1, engine.interpolacionMs)), 0.f, 1.f);
      const float suave = .5f - .5f * std::cos(static_cast<float>(NaoRL::pi) * a);
      for(int j = 0; j < Joints::numOfJoints; ++j)
        deseados[j] = anguloInicial[j] + suave * (engine.home[j] - anguloInicial[j]);
      if(a >= 1.f)
      {
        estado = Estado::enHome;
        tEnHome = ahora;
      }
      break;
    }
    case Estado::enHome:
    {
      deseados = engine.home;
      for(int j = Joints::firstLegJoint; j < Joints::numOfJoints; ++j)
        if(engine.theJointSensorData.currents[j] != static_cast<short>(SensorData::off)) // sin sensor en simulacion
        {
          sumaCorrienteHome += static_cast<float>(std::abs(engine.theJointSensorData.currents[j]));
          ++muestrasCorrienteHome; // una muestra por articulacion: la media es por motor
        }
      const bool registroListo = engine.registro.abierto() || !engine.exigirRegistro ||
                                 SystemCall::getMode() != SystemCall::physicalRobot;
      const bool camaraLista = engine.ignorarCamara ||
                               (vision.ultimoPerceptoValido && engine.theFrameInfo.getTimeSince(vision.ultimoPerceptoValido) <= engine.maxEdadCamaraMs);
      if(engine.theFrameInfo.getTimeSince(tEnHome) >= engine.asentarMs)
      {
        if(!registroListo)
          engine.enclavar(NaoRLWalkEngine::Paro::suave, "sin registro (" + engine.rutaRegistro + ")");
        else if(camaraLista)
        {
          // H7: con cualquier duda no se camina (paro suave = stand de B-Human).
          const std::string fallo = autoverificar();
          if(!fallo.empty())
          {
            engine.enclavar(NaoRLWalkEngine::Paro::suave, "autoverificacion: " + fallo);
            SystemCall::say("pre walk check failed");
            break;
          }
          estado = Estado::caminando;
          tMarcha = ahora;
          ciclo = 0;
          registrarAhora = true;
          decisor.reiniciar();
          excesosInferencia = 0;
          vision.metaCerca = false;
          std::cout << "[NaoRL] t=" << ahora << " marcha RL: vxCmd=" << engine.vxCmd << " f=" << engine.frecuenciaMarcha
                    << " Hz" << std::endl;
        }
      }
      break;
    }
    case Estado::caminando:
      // Una decision cada 2 ciclos de 12 ms (24 ms); entre decisiones se mantiene el objetivo.
      if(ciclo % NaoRL::ciclosPorDecision == 0)
      {
        decidir();
        tInferencia = ultimaInferenciaUs;
      }
      break;
    case Estado::parado:
      break;
  }
  // Un paro duro de la propia decision (NaN, watchdog) suelta en este mismo ciclo, no en el siguiente.
  if(engine.paro == NaoRLWalkEngine::Paro::duro)
    estado = Estado::parado;

  if(estado != Estado::parado)
  {
    NaoRL::PorArticulacion minimos, maximos;
    NaoRL::limitesArticulares(engine.theJointLimits, minimos, maximos);
    std::array<bool, Joints::numOfJoints> recorteVel{}, recorteLim{};
    NaoRL::limitarObjetivos(deseados, objetivos, NaoRL::articulacionesLimitadas(), minimos, maximos,
                            NaoRL::pasosMaximos(engine.velMax, engine.factorVelRoll, engine.mapa), engine.margenLimites,
                            Joints::lHipYawPitch, Joints::rHipYawPitch, recorteVel, recorteLim);
    std::copy_backward(objetivosPasados.begin(), objetivosPasados.end() - 1, objetivosPasados.end());
    objetivosPasados[0] = objetivos;
  }
  else
    objetivos = anguloValido; // no empujar: el objetivo sigue al angulo medido

  if(registrarAhora)
    registrar(ahora, tInferencia);
  ++ciclo;
  primerCiclo = false;
}

void NaoRLWalkPhase::registrar(const unsigned ahora, const float tInferenciaUs)
{
  if(!engine.registro.abierto())
    return;
  char buf[160];
  std::string linea;
  linea.reserve(1600);
  const auto& s = engine.theJointSensorData;
  std::snprintf(buf, sizeof(buf), "%u,%s,%u,%.0f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%s", ahora,
                nombreEstado(static_cast<int>(estado)), decisiones, tInferenciaUs,
                TypeRegistry::getEnumName(engine.theFallDownState.state), engine.inclinacion(), entrada.gyro[0],
                entrada.gyro[1], entrada.gyro[2], entrada.arriba[0], entrada.arriba[1], entrada.arriba[2], entrada.psi,
                entrada.d, vision.ultimoPerceptoValido ? engine.theFrameInfo.getTimeSince(vision.ultimoPerceptoValido) : 0u,
                engine.paro == NaoRLWalkEngine::Paro::ninguno ? "" : engine.motivoParo.c_str());
  linea += buf;
  for(int j = Joints::firstLegJoint; j < Joints::numOfJoints; ++j)
  {
    std::snprintf(buf, sizeof(buf), ",%.4f,%.4f,%d,%d", objetivos[j], anguloValido[j], s.currents[j], s.temperatures[j]);
    linea += buf;
  }
  for(const float a : decisor.ultimaAccion())
  {
    std::snprintf(buf, sizeof(buf), ",%.4f", a);
    linea += buf;
  }
  engine.registro.escribir(std::move(linea));
}

bool NaoRLWalkPhase::isDone(const MotionRequest& motionRequest) const
{
  // Paro suave: el generador entrega el stand de B-Human.
  if(engine.paro == NaoRLWalkEngine::Paro::suave)
    return true;
  // Paro duro: se queda sin rigidez hasta playDead (botones de la cabeza) o una carrera nueva. Un "stand" de
  // CarrilBehavior (paro manual o su propio paro por inclinacion) NO vuelve a dar rigidez (6b.4).
  if(estado == Estado::parado)
    return motionRequest.motion == MotionRequest::playDead || engine.paro == NaoRLWalkEngine::Paro::ninguno;
  return motionRequest.motion != MotionRequest::walkAtRelativeSpeed;
}

void NaoRLWalkPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  // La cabeza la lleva HeadMotionEngine (freeLimbs): no se toca.
  std::array<float, Joints::numOfJoints> angulos = objetivos, minimos, maximos;
  std::array<int, Joints::numOfJoints> rigideces;
  std::array<bool, Joints::numOfJoints> escribe;
  for(int j = 0; j < Joints::numOfJoints; ++j)
  {
    escribe[j] = !NaoRL::esCabeza(j);
    rigideces[j] = estado == Estado::parado ? engine.stiffnessParo : NaoRL::esPierna(j) ? engine.stiffnessPiernas : engine.stiffnessBrazos;
    minimos[j] = engine.theJointLimits.limits[j].min;
    maximos[j] = engine.theJointLimits.limits[j].max;
  }

  // H6: segundo cerrojo, no depende de que limitarObjetivos este bien.
  const std::array<float, Joints::numOfJoints> pedidos = angulos;
  const std::array<int, Joints::numOfJoints> rigidecesPedidas = rigideces;
  const int mala = NaoRL::guardaFinal(angulos, rigideces, escribe, anguloValido, minimos, maximos, engine.stiffnessParo);
  if(mala >= 0)
  {
    engine.enclavar(NaoRLWalkEngine::Paro::duro, std::string("guarda final: ") + TypeRegistry::getEnumName(static_cast<Joints::Joint>(mala)) + " = " +
                    std::to_string(pedidos[mala]) + " en [" + std::to_string(minimos[mala]) + ", " + std::to_string(maximos[mala]) + "], rigidez " +
                    std::to_string(rigidecesPedidas[mala]) + ", medido " + std::to_string(anguloValido[mala]) + ", estado " + nombreEstado(static_cast<int>(estado)));
    estado = Estado::parado;
  }

  for(int j = 0; j < Joints::numOfJoints; ++j)
    if(escribe[j])
    {
      jointRequest.angles[j] = angulos[j];
      jointRequest.stiffnessData.stiffnesses[j] = rigideces[j];
    }
  motionInfo.isMotionStable = estado != Estado::parado;
  if(estado == Estado::caminando)
  {
    // Sin modelo de odometria: se estima con vxCmd (como Camilo) para que CarrilBehavior pueda parar en la meta.
    odometryOffset.translation.x() = engine.vxCmd * 1000.f * dtUltimo;
    motionInfo.speed = Pose2f(0.f, engine.vxCmd * 1000.f, 0.f);
  }
}
