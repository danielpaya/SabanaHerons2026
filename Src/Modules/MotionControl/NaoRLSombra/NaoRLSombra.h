/**
 * @file NaoRLSombra.h
 *
 * MODO SOMBRA de la politica nao_v08: corre en el robot con los sensores reales mientras camina B-Human y registra lo que
 * HABRIA mandado, sin mandar nada. Lazo abierto: sirve para tiempos, signos, rangos y linea base; no prueba que la politica
 * mantenga el equilibrio (ver demo_carril/rl/analizar_sombra.py). Solo se instancia en el escenario CarrilSombra.
 *
 * POR QUE ES ESTRUCTURALMENTE IMPOSIBLE QUE ESCRIBA A LOS MOTORES (lo comprueba el test NaoRL.SombraSoloProveeSuRepresentacion):
 *  1. A los motores solo llega JointRequest, y JointRequest solo lo escribe su proveedor (MotionEngine en threads.cfg). Este
 *     modulo no lo PROVEE: su unico PROVIDES es NaoRLSombraEstado, y el framework solo le entrega por referencia no constante
 *     la representacion que provee (update(NaoRLSombraEstado&)).
 *  2. Todo lo demas son REQUIRES, que el framework da como referencia CONSTANTE: theJointRequest solo se puede leer. No hay
 *     MODIFIES.
 *  3. La otra via hacia los motores es que el MotionEngine ejecute una MotionPhase, y las fases entran por los *Generator
 *     (createPhase) y por MotionRequest. Este modulo no provee ninguno ni define ninguna fase.
 *  4. Lo que calcula (objetivos, limitador simulado) son variables privadas que solo salen al CSV.
 *
 * Con usarSombra = false (por defecto) no se carga la politica, no se abre ningun archivo y no hay hilo.
 *
 * Toda la cadena es la de la fase de marcha, compartida en Tools/NaoRL para que no puedan divergir: sensores -> entrada
 * (NaoRLSensoresBHuman.h, NaoRLObservacion.h), red e historia (NaoRLGuardas.h), decodificacion (NaoRLPolitica.h), limitador
 * (NaoRLObservacion.h), carga con hash y autotest (NaoRLPoliticaConfig.h).
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/NaoRLSombraEstado.h"
#include "Representations/Perception/Carril/CarrilPercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/NaoRL/NaoRLGuardas.h"
#include "Tools/NaoRL/NaoRLObservacion.h"
#include "Tools/NaoRL/NaoRLPolitica.h"
#include "Tools/NaoRL/NaoRLRegistro.h"
#include "Tools/NaoRL/NaoRLSensoresBHuman.h"
#include "Tools/NaoRL/NaoRLSombraFormato.h"
#include <array>
#include <string>
#include <vector>

MODULE(NaoRLSombra,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointLimits),
  REQUIRES(JointRequest),
  REQUIRES(JointSensorData),
  REQUIRES(LowerCarrilPercept),
  REQUIRES(MotionInfo),
  PROVIDES(NaoRLSombraEstado),
  LOADS_PARAMETERS(
  {,
    (bool)(false) usarSombra,          /**< false = no hace nada. */
    (bool)(false) siempre,             /**< true = corre tambien con B-Human quieto (robot suspendido o de pie). */
    (std::string) politica,            /**< Config/NeuralNets/NaoRL/<politica>.cfg. */
    (float) vxCmd,                     /**< Mando a la politica [m/s]. */
    (float) frecuenciaMarcha,          /**< Reloj de fase [Hz]. */
    (std::vector<float>) lo,           /**< Como naoRLWalkEngine.cfg de CarrilRL (el test lo comprueba). */
    (std::vector<float>) hi,           /**< Idem. */
    (std::vector<float>) velMax,       /**< Idem. */
    (float) factorVelRoll,             /**< Idem. */
    (float) margenLimites,             /**< Idem. */
    (float) distanciaMetaSinCamara,    /**< Idem. */
    (std::string) rutaRegistro,        /**< Directorio del CSV en el robot. */
  }),
});

class NaoRLSombra : public NaoRLSombraBase
{
public:
  NaoRLSombra();

private:
  void update(NaoRLSombraEstado& estado) override;

  /** Empieza una marcha: reloj de fase, historia en ceros y objetivos en home, como la fase al pasar a caminando. */
  void reiniciar();
  /** Una decision (cada 24 ms). Rellena fila, menos lo que aun depende del limitador. */
  void decidir();
  /** Cierra la fila con el limitador y la encola. */
  void registrar();

  bool lista = false;              /**< usarSombra, politica cargada y registro abierto. */
  NaoRL::Politica red;
  NaoRL::Accion postura{}, loAccion{}, hiAccion{};
  float escala = 0.f;
  NaoRL::MapaMotores mapa;
  NaoRL::PorArticulacion home{}, minimos{}, maximos{}, pasos{};
  std::array<bool, Joints::numOfJoints> limita{};
  NaoRLRegistro registro;

  bool corriendo = false;
  bool enMarcha = false;           /**< B-Human estaba en fase de marcha el ciclo anterior. */
  unsigned tMarcha = 0;
  unsigned ciclo = 0;
  unsigned decisiones = 0;
  bool hayFila = false;

  NaoRL::PorArticulacion anguloValido{}, deseados{}, objetivos{};
  bool fallaAngulo = false;
  NaoRL::EntradaPaso entrada;
  NaoRL::EstadoVision vision;
  NaoRL::Decisor decisor;
  std::array<bool, Joints::numOfJoints> recorteVel{}, recorteLim{};
  NaoRL::FilaSombra fila;
};
