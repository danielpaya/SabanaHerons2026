/**
 * @file NaoRLSensoresBHuman.h
 *
 * El puente entre las representaciones de B-Human y Tools/NaoRL, compartido por la fase de marcha (NaoRLWalkPhase) y el modo
 * sombra (NaoRLSombra): mapeo de motores, pose home, angulos validos, sensores del ciclo, plausibilidad y decodificacion de
 * la accion. Solo LEE representaciones (todo llega por const&).
 */

#pragma once

#include "Math/Angle.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Perception/Carril/CarrilPercept.h"
#include "Representations/Sensing/InertialData.h"
#include "RobotParts/Joints.h"
#include "Streaming/TypeRegistry.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/NaoRL/NaoRLObservacion.h"
#include <array>
#include <cctype>
#include <string>
#include <vector>

namespace NaoRL
{
  static_assert(static_cast<float>(SensorData::off) == sensorApagado);

  using PorArticulacion = std::array<float, Joints::numOfJoints>;

  /** Por articulacion de B-Human: indice en la accion, o -1 si no la maneja la politica. Por motor: articulacion. */
  struct MapaMotores
  {
    std::array<int, Joints::numOfJoints> indiceAccion{};
    std::array<Joints::Joint, nMotores> articulacion{};
  };

  inline bool esPierna(const int j) { return j >= Joints::firstLegJoint && j < Joints::numOfJoints; }
  inline bool esCabeza(const int j) { return j == Joints::headYaw || j == Joints::headPitch; }
  inline bool esRollPierna(const int j)
  {
    return j == Joints::lHipRoll || j == Joints::rHipRoll || j == Joints::lAnkleRoll || j == Joints::rAnkleRoll;
  }

  /** Mapeo por nombre, "LHipRoll" -> lHipRoll. Devuelve false si un motor no existe en Joints. */
  inline bool mapearMotores(MapaMotores& mapa)
  {
    mapa.indiceAccion.fill(-1);
    bool valido = true;
    for(std::size_t i = 0; i < nMotores; ++i)
    {
      std::string buscado = nombresMotores[i];
      buscado[0] = static_cast<char>(std::tolower(static_cast<unsigned char>(buscado[0])));
      bool hallado = false;
      FOREACH_ENUM(Joints::Joint, j)
        if(buscado == TypeRegistry::getEnumName(j))
        {
          mapa.articulacion[i] = j;
          mapa.indiceAccion[j] = static_cast<int>(i);
          hallado = true;
        }
      valido = valido && hallado;
    }
    return valido;
  }

  /** Home de las 26: brazos/cabeza como CopaRace::poseHome de Camilo (contrato, seccion 4), piernas y hombro/codo de la postura. */
  inline PorArticulacion poseHome(const Accion& postura, const MapaMotores& mapa)
  {
    PorArticulacion home;
    home.fill(0.f);
    home[Joints::lShoulderPitch] = home[Joints::rShoulderPitch] = 1.57f;
    home[Joints::lShoulderRoll] = .2f;
    home[Joints::rShoulderRoll] = -.2f;
    home[Joints::lElbowYaw] = -1.57f;
    home[Joints::rElbowYaw] = 1.57f;
    home[Joints::lElbowRoll] = -.2f;
    home[Joints::rElbowRoll] = .2f;
    for(std::size_t i = 0; i < nMotores; ++i)
      home[mapa.articulacion[i]] = postura[i];
    home[Joints::rHipYawPitch] = home[Joints::lHipYawPitch];
    return home;
  }

  /** Los angulos no finitos se sustituyen por el ultimo valido. Devuelve true si alguno (sin contar la cabeza) fallo. */
  inline bool actualizarAngulosValidos(const JointSensorData& sensores, PorArticulacion& anguloValido)
  {
    bool falla = false;
    for(int j = 0; j < Joints::numOfJoints; ++j)
    {
      const float angulo = sensores.angles[j];
      if(std::isfinite(angulo) && angulo != SensorData::off)
        anguloValido[j] = angulo;
      else if(!esCabeza(j))
        falla = true;
    }
    return falla;
  }

  /** Los sensores de este ciclo, en crudo. */
  inline SensoresPaso sensoresPaso(const InertialData& inercial, const CarrilPercept& foto, const FrameInfo& frameInfo,
                                   const PorArticulacion& anguloValido, const MapaMotores& mapa)
  {
    SensoresPaso s;
    s.gyro = {static_cast<float>(inercial.gyro.x()), static_cast<float>(inercial.gyro.y()), static_cast<float>(inercial.gyro.z())};
    const Quaternionf& q = inercial.orientation3D;
    s.qw = q.w();
    s.qx = q.x();
    s.qy = q.y();
    s.qz = q.z();
    for(std::size_t i = 0; i < nMotores; ++i)
      s.angulos[i] = anguloValido[mapa.articulacion[i]];
    s.psiCamara = foto.psi;
    s.dCamaraMm = foto.d;
    s.camaraValida = foto.valido;
    s.metaVisible = foto.metaVisible;
    s.distanciaMetaMm = foto.distanciaMeta;
    s.tiempoFrame = foto.tiempoFrame;
    s.edadFrameMs = frameInfo.getTimeSince(foto.tiempoFrame);
    return s;
  }

  /** H2: motivo por el que los sensores de esta decision no valen, o nullptr. Usa los angulos CRUDOS. */
  inline const char* motivoEntrada(const bool fallaAngulo, const EntradaPaso& e, const JointSensorData& sensores,
                                   const JointLimits& limites, const MapaMotores& mapa)
  {
    if(fallaAngulo)
      return "angulo no finito";
    Accion crudos, minimos, maximos;
    for(std::size_t i = 0; i < nMotores; ++i)
    {
      const Joints::Joint j = mapa.articulacion[i];
      crudos[i] = sensores.angles[j];
      minimos[i] = limites.limits[j].min;
      maximos[i] = limites.limits[j].max;
    }
    return motivoNoPlausible(e.gyro, e.arriba, crudos, minimos, maximos);
  }

  /** La accion ya decodificada (seccion 4) como objetivos de las 26: lo que no maneja la politica se queda en home. */
  inline PorArticulacion deseadosDesdeObjetivos(const Accion& objetivo15, const PorArticulacion& home, const MapaMotores& mapa)
  {
    PorArticulacion deseados = home;
    for(std::size_t i = 0; i < nMotores; ++i)
      deseados[mapa.articulacion[i]] = objetivo15[i];
    deseados[Joints::rHipYawPitch] = deseados[Joints::lHipYawPitch]; // un solo motor fisico
    return deseados;
  }

  /** Limites de jointLimits.cfg de las 26. */
  inline void limitesArticulares(const JointLimits& limites, PorArticulacion& minimos, PorArticulacion& maximos)
  {
    for(int j = 0; j < Joints::numOfJoints; ++j)
    {
      minimos[j] = limites.limits[j].min;
      maximos[j] = limites.limits[j].max;
    }
  }

  /** Paso maximo de 12 ms por articulacion: velMax del .cfg (las que no maneja la politica, la minima), roll x factorVelRoll. */
  inline PorArticulacion pasosMaximos(const std::vector<float>& velMax, const float factorVelRoll, const MapaMotores& mapa)
  {
    float velMaxMinima = velMax[0];
    for(const float v : velMax)
      velMaxMinima = std::min(velMaxMinima, v);
    PorArticulacion pasos;
    for(int j = 0; j < Joints::numOfJoints; ++j)
    {
      const int i = mapa.indiceAccion[j];
      float vel = i >= 0 ? velMax[i] : velMaxMinima;
      if(esRollPierna(j))
        vel *= factorVelRoll;
      pasos[j] = vel * cicloS;
    }
    return pasos;
  }

  /** Quienes pasan por el limitador: todas menos la cabeza. */
  inline std::array<bool, Joints::numOfJoints> articulacionesLimitadas()
  {
    std::array<bool, Joints::numOfJoints> limita;
    for(int j = 0; j < Joints::numOfJoints; ++j)
      limita[j] = !esCabeza(j);
    return limita;
  }
}
