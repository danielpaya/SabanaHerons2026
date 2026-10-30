/**
 * @file NaoRLSombra.cpp
 *
 * Ver NaoRLSombra.h y demo_carril/docs/despliegue_nao_v08.md.
 */

#include "NaoRLSombra.h"
#include "Platform/SystemCall.h"
#include "Streaming/Output.h"
#include "Tools/NaoRL/NaoRLPoliticaConfig.h"
#include <chrono>
#include <iostream>
#include <limits>

MAKE_MODULE(NaoRLSombra);

NaoRLSombra::NaoRLSombra()
{
  if(!usarSombra)
    return;

  std::string error;
  NaoRL::PoliticaCargada cargada;
  if(lo.size() != NaoRL::nMotores || hi.size() != NaoRL::nMotores || velMax.size() != NaoRL::nMotores)
    error = "naoRLSombra.cfg: lo/hi/velMax deben tener 15 valores";
  else if(!NaoRL::mapearMotores(mapa))
    error = "no se pudo mapear un motor de la politica a Joints";
  else if(NaoRL::cargarPolitica("NeuralNets/NaoRL/" + politica + ".cfg", cargada, error))
  {
    red = std::move(cargada.politica);
    postura = cargada.postura;
    escala = cargada.escala;
    std::copy(lo.begin(), lo.end(), loAccion.begin());
    std::copy(hi.begin(), hi.end(), hiAccion.begin());
    home = NaoRL::poseHome(postura, mapa);
    anguloValido = deseados = objetivos = home;
    pasos = NaoRL::pasosMaximos(velMax, factorVelRoll, mapa);
    limita = NaoRL::articulacionesLimitadas();

    std::array<const char*, NaoRL::nPiernasRegistro> piernas;
    for(std::size_t k = 0; k < NaoRL::nPiernasRegistro; ++k)
      piernas[k] = TypeRegistry::getEnumName(static_cast<Joints::Joint>(Joints::firstLegJoint + static_cast<int>(k)));
    if(registro.abrir(rutaRegistro, NaoRL::cabeceraSombra(piernas), "naoRLsombra_"))
      lista = true;
    else
      error = "no se pudo abrir el registro en " + rutaRegistro;
  }

  if(!lista)
  {
    OUTPUT_ERROR("NaoRLSombra: sombra desactivada: " << error);
    SystemCall::say("shadow mode failed");
  }
  std::cout << "[NaoRLSombra] politica=" << politica << (lista ? " cargada" : " NO cargada (" + error + ")") << " siempre="
            << (siempre ? "true" : "false") << " registro=" << (registro.abierto() ? registro.archivo() : std::string("ninguno")) << std::endl;
}

void NaoRLSombra::reiniciar()
{
  tMarcha = theFrameInfo.time;
  ciclo = 0;
  hayFila = false;
  decisor.reiniciar();
  vision.metaCerca = false;
  deseados = objetivos = home;
}

void NaoRLSombra::update(NaoRLSombraEstado& estado)
{
  estado.activa = lista;
  estado.decisiones = decisiones;
  if(!lista)
    return;

  // Sensores en cada ciclo, como la fase: la vision se sigue aunque B-Human este quieto.
  fallaAngulo = NaoRL::actualizarAngulosValidos(theJointSensorData, anguloValido);
  entrada = NaoRL::leerSensores(NaoRL::sensoresPaso(theInertialData, theLowerCarrilPercept, theFrameInfo, anguloValido, mapa), entrada,
                                false, distanciaMetaSinCamara, vision);

  const bool marcha = theMotionInfo.isMotion(MotionPhase::walk);
  if(!siempre && !marcha)
  {
    corriendo = enMarcha = false;
    return;
  }
  if(!corriendo || (marcha && !enMarcha))
    reiniciar();
  corriendo = true;
  enMarcha = marcha;

  // Igual que la fase: una decision cada 2 ciclos, el limitador en todos. La linea se escribe tras los 2 pasos del limitador.
  const bool cicloDecision = ciclo % NaoRL::ciclosPorDecision == 0;
  if(cicloDecision)
    decidir();
  NaoRL::limitarObjetivos(deseados, objetivos, limita, minimos, maximos, pasos, margenLimites, Joints::lHipYawPitch,
                          Joints::rHipYawPitch, recorteVel, recorteLim);
  if(!cicloDecision && hayFila)
    registrar();
  ++ciclo;
}

void NaoRLSombra::decidir()
{
  NaoRL::limitesArticulares(theJointLimits, minimos, maximos);
  recorteVel.fill(false);
  recorteLim.fill(false);

  NaoRL::EntradaPaso e = entrada;
  NaoRL::completarDecision(e, vxCmd, frecuenciaMarcha, theFrameInfo.getTimeSince(tMarcha) / 1000.f);
  const char* motivoEntrada = NaoRL::motivoEntrada(fallaAngulo, e, theJointSensorData, theJointLimits, mapa);

  // z de lo que la red veria, aunque el filtro luego lo rechace: justo ahi es cuando interesa.
  red.estadisticasZ(decisor.observacionSiguiente(e, postura), NaoRL::umbralZ, fila.zmax, fila.zmaxIdx, fila.nZ4);

  const auto t0 = std::chrono::steady_clock::now();
  const NaoRL::Decisor::Resultado resultado = decisor.decidir(red, e, postura, motivoEntrada, std::numeric_limits<int>::max()); // sin paro: aqui nada se mueve
  fila.tInferenciaUs = std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - t0).count();
  if(resultado.motivo && decisor.fallosSeguidos() == 1)
    registro.escribir("# t=" + std::to_string(theFrameInfo.time) + " la fase repetiria la accion: " + resultado.motivo);

  const NaoRL::Accion objetivo15 = NaoRL::accionAObjetivos(resultado.accion, postura, escala, loAccion, hiAccion);
  deseados = NaoRL::deseadosDesdeObjetivos(objetivo15, home, mapa);
  for(std::size_t i = 0; i < NaoRL::nMotores; ++i)
    if(objetivo15[i] != postura[i] + resultado.accion[i] * escala)
      recorteLim[mapa.articulacion[i]] = true;
  ++decisiones;

  fila.tMs = theFrameInfo.time;
  fila.estadoBHuman = TypeRegistry::getEnumName(theMotionInfo.executedPhase);
  fila.tMarchaS = e.tMarcha;
  fila.fallDown = TypeRegistry::getEnumName(theFallDownState.state);
  fila.inclinacion = NaoRL::inclinacionTorso(theInertialData.angle.x(), theInertialData.angle.y());
  fila.gyro = e.gyro;
  fila.arriba = e.arriba;
  fila.psiObs = e.psi;
  fila.dObs = e.d;
  fila.edadCamaraMs = vision.ultimoPerceptoValido ? theFrameInfo.getTimeSince(vision.ultimoPerceptoValido) : 0u;
  fila.a = resultado.accion;
  fila.objPol = objetivo15;
  for(std::size_t i = 0; i < NaoRL::nMotores; ++i)
  {
    fila.bh[i] = theJointRequest.angles[mapa.articulacion[i]];
    fila.ang[i] = theJointSensorData.angles[mapa.articulacion[i]];
  }
  for(std::size_t k = 0; k < NaoRL::nPiernasRegistro; ++k)
  {
    const int j = Joints::firstLegJoint + static_cast<int>(k);
    fila.corr[k] = theJointSensorData.currents[j];
    fila.temp[k] = theJointSensorData.temperatures[j];
  }
  hayFila = true;
}

void NaoRLSombra::registrar()
{
  fila.nRecorteVel = fila.nRecorteLim = 0;
  for(std::size_t i = 0; i < NaoRL::nMotores; ++i)
  {
    const Joints::Joint j = mapa.articulacion[i];
    fila.objLim[i] = objetivos[j];
    fila.nRecorteVel += recorteVel[j] ? 1 : 0;
    fila.nRecorteLim += recorteLim[j] ? 1 : 0;
  }
  registro.escribir(NaoRL::lineaSombra(fila));
  hayFila = false;
}
