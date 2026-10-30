/**
 * @file NaoRLWalkEngine.h
 *
 * Marcha RL (politica nao_v08) para la Carrera Individual, como alternativa a la caminata de B-Human. Contrato:
 * demo_carril/docs/despliegue_nao_v08.md (secciones 1-6c). Esqueleto tomado de CopaRLWalkEngine de Camilo
 * (CamiloGomezB/nao-v6-rl-race), pero NO su modelo de seguridad: aquella politica dano LHipRoll de un NAO.
 *
 * COMO SE ELIGE LA CAMINATA (dos llaves, las dos apagadas por defecto):
 *  1. Escenario: este modulo solo se instancia en el escenario CarrilRL (Config/Scenarios/CarrilRL/threads.cfg), donde
 *     provee WalkAtRelativeSpeedGenerator en lugar de WalkAtSpeedEngine. El escenario Carril NO lo usa: con Carril
 *     todo sigue igual que antes (B-Human + vision de carril = plan B).
 *  2. usarPoliticaRL (naoRLWalkEngine.cfg, false por defecto): aun en CarrilRL, con false el generador entrega el stand
 *     de B-Human y la politica nunca mueve el robot. Es un seguro contra desplegar CarrilRL por error.
 *
 * Fase NaoRLWalkPhase (tipo calibration, como Camilo): interpolando (postura actual -> home, coseno) -> enHome
 * (espera asentarMs) -> caminando (una decision cada 2 ciclos de 12 ms = 24 ms; entre decisiones se mantiene el
 * objetivo). La cabeza queda libre para HeadMotionEngine (CarrilBehavior la inclina para la camara inferior).
 *
 * SEGURIDAD (secciones 6 y 6b del contrato), todo en esta fase o en el modulo:
 *  - filtro NaN/Inf en la accion y en los angulos leidos (se repite la ultima accion valida; maxFallosNaN -> paro);
 *  - limite de cambio por ciclo |delta| <= velMax * 0.012 s por articulacion (roll x factorVelRoll);
 *  - margen interior a JointLimits (jointLimits.cfg) y recorte a lo/hi del modelo;
 *  - paro ENCLAVADO por inclinacion, caida (falling/fallen), levantado (pickedUp), temperatura, corriente sostenida,
 *    error de seguimiento por articulacion, NaN repetido, watchdog de inferencia repetido;
 *  - paro duro = rigidez a stiffnessParo (0 = soltar) DE INMEDIATO con objetivo = angulo medido (no empuja);
 *  - paro suave (camara perdida, tiempo maximo de marcha, sin registro) = stand de B-Human;
 *  - registro por ciclo de decision en un CSV del robot (hilo aparte, prioridad normal).
 * El paro se desenclava solo al volver a armar el robot (playerState pasa a active: toques al pecho o pasar por sin
 * rigidez). El boton de la cabeza que arranca la carrera NO lo desenclava: tras un paro de la politica hay que rearmar.
 *
 * INTEGRIDAD DE LA ORDEN (seccion 6c, H1-H7; la logica esta en Tools/NaoRL/NaoRLGuardas.h y NaoRLPoliticaConfig.h, y la
 * prueban los tests NaoRL.*):
 *  - H1 camara: NaN/Inf o fuera de rango = "sin medicion nueva", no es un fallo de la red;
 *  - H2 plausibilidad de gyro, arriba y angulos antes de evaluar la red; entrada normalizada recortada a +-10;
 *  - H3 hash de los pesos y autotest al cargar; si no, politica NO cargada (stand de B-Human + aviso hablado);
 *  - H4 rangos duros de los umbrales: fuera de rango no se arma;
 *  - H5 banderas de prueba: aviso, cabecera del registro, ignorarLevantado solo con vxCmd bajo;
 *  - H6 guarda final en calcJoints, independiente de limitarObjetivos;
 *  - H7 autoverificacion en enHome antes de caminar.
 * La velocidad y el giro de MotionRequest NO se leen aqui: la politica usa vxCmd, validado por H4 (6c).
 */

#pragma once

#include "Framework/Module.h"
#include "MathBase/BHMath.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/StandGenerator.h"
#include "Representations/MotionControl/WalkAtRelativeSpeedGenerator.h"
#include "Representations/Perception/Carril/CarrilPercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "RobotParts/Joints.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/NaoRL/NaoRLGuardas.h"
#include "Tools/NaoRL/NaoRLObservacion.h"
#include "Tools/NaoRL/NaoRLPolitica.h"
#include "Tools/NaoRL/NaoRLRegistro.h"
#include "Tools/NaoRL/NaoRLSensoresBHuman.h"
#include <array>
#include <string>

MODULE(NaoRLWalkEngine,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(InertialData),
  REQUIRES(JointLimits),
  REQUIRES(JointSensorData),
  REQUIRES(LowerCarrilPercept),
  REQUIRES(StandGenerator),
  PROVIDES(WalkAtRelativeSpeedGenerator),
  LOADS_PARAMETERS(
  {,
    (bool) usarPoliticaRL,             /**< false = la politica no se usa nunca (stand de B-Human). */
    (std::string) politica,            /**< Config/NeuralNets/NaoRL/<politica>.cfg. */
    (float) vxCmd,                     /**< Velocidad pedida a la politica [m/s] (entrenada hasta 0.5; E3: 0.2). */
    (float) frecuenciaMarcha,          /**< Reloj de fase [Hz] (contrato: 1.8). */
    (int) interpolacionMs,             /**< Postura actual -> home. */
    (int) asentarMs,                   /**< Tiempo minimo en home antes de caminar. */
    (int) stiffnessPiernas,            /**< Rigidez de las piernas [%] en marcha (variable de prueba, 6b.6). */
    (int) stiffnessBrazos,             /**< Rigidez de los brazos [%] en marcha. */
    (int) stiffnessParo,               /**< Rigidez [%] tras un paro duro (0 = soltar, 6b.4). */
    (std::vector<float>) lo,           /**< Limite inferior del modelo por motor [rad] (golden: lo). */
    (std::vector<float>) hi,           /**< Limite superior del modelo por motor [rad] (golden: hi). */
    (std::vector<float>) velMax,       /**< Velocidad maxima por motor [rad/s] (golden: vel_max). */
    (float) factorVelRoll,             /**< Factor extra de velocidad para las 4 de roll (6b.5). */
    (float) margenLimites,             /**< Margen interior a jointLimits.cfg [rad]. */
    (float) maxInclinacion,            /**< Paro: inclinacion del torso [rad]. */
    (int) maxTemperatura,              /**< Paro: temperatura de un motor de pierna [C]. */
    (int) umbralCorriente,             /**< Paro: |corriente| de un motor de pierna [mA] ... */
    (int) ciclosCorriente,             /**< ... sostenida durante estos ciclos de 12 ms. */
    (float) umbralSeguimiento,         /**< Paro: |objetivo - medido| [rad] en piernas (no roll) ... */
    (float) umbralSeguimientoRoll,     /**< ... en las 4 de roll ... */
    (int) ciclosSeguimiento,           /**< ... durante estos ciclos seguidos. */
    (int) retardoSeguimiento,          /**< Ciclos de retraso del servo: se compara con el objetivo de hace N ciclos. */
    (int) maxFallosNaN,                /**< Paro tras N decisiones seguidas con NaN/Inf. */
    (int) presupuestoInferenciaUs,     /**< Watchdog: una inferencia mas larga se descarta (se repite la anterior). */
    (int) maxExcesosInferencia,        /**< Paro tras N descartes seguidos por tiempo. */
    (int) maxEdadCamaraMs,             /**< Paro suave si el percepto valido es mas viejo que esto caminando. */
    (float) distanciaMetaSinCamara,    /**< Con la meta vista a menos de esto [mm], perder las lineas no es paro
                                            (CarrilBehavior termina la carrera a ciegas por odometria). */
    (int) maxMarchaMs,                 /**< Paro suave por tiempo de marcha (respaldo de la parada en meta). */
    (int) maxCorrienteHome,            /**< H7: corriente media de las piernas en home [mA] para poder caminar. */
    (bool) ignorarCamara,              /**< SOLO E2/E3: vis = (0, 0) y sin paro por camara (H5: se avisa). */
    (bool) ignorarLevantado,           /**< SOLO E2 (robot colgado): no parar por pickedUp (H5: exige vxCmd bajo). */
    (std::string) rutaRegistro,        /**< Directorio del CSV de registro en el robot. */
    (bool) exigirRegistro,             /**< En el robot real, no caminar si el registro no se pudo abrir. */
  }),
});

class NaoRLWalkEngine : public NaoRLWalkEngineBase
{
public:
  NaoRLWalkEngine();

  /** Tipo de paro enclavado. */
  enum class Paro { ninguno, suave, duro };

  NaoRL::Politica red;
  NaoRL::Accion postura{};
  NaoRL::Accion loAccion{}, hiAccion{}; /**< lo/hi del .cfg de parametros como arreglos. */
  float escala = 0.f;
  bool politicaValida = false;
  bool parametrosValidos = false;

  NaoRL::MapaMotores mapa;
  /** Pose home completa (26): postura del .cfg + brazos/manos de CopaRace::poseHome. */
  NaoRL::PorArticulacion home{};

  // Estado del paro (vive en el modulo: sobrevive a la fase y al FallEngine).
  Paro paro = Paro::ninguno;
  std::string motivoParo;
  bool estabaActivo = false;
  unsigned tUltimaFase = 0; /**< Ultimo ciclo en que corrio una NaoRLWalkPhase (0 = nunca). */

  NaoRLRegistro registro;

  /** Enclava un paro (el mas grave gana) y lo anuncia una sola vez. */
  void enclavar(Paro tipo, const std::string& motivo);

  /** Inclinacion del torso [rad] (angulo con la vertical). */
  float inclinacion() const;

private:
  void update(WalkAtRelativeSpeedGenerator& generator) override;
};

/** La fase de la marcha RL (tipo calibration: ningun generador de B-Human la reinterpreta como WalkPhase). */
class NaoRLWalkPhase : public MotionPhase
{
public:
  explicit NaoRLWalkPhase(NaoRLWalkEngine& engine);

private:
  enum class Estado { interpolando, enHome, caminando, parado };

  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset,
                  MotionInfo& motionInfo) override;
  unsigned freeLimbs() const override { return bit(MotionPhase::head); }

  /** Una decision de la politica (cada 24 ms). */
  void decidir();
  /** Revisa todos los paros que no dependen de la red. */
  void revisarSeguridad();
  /** H7: "" si se puede caminar; si no, la explicacion. */
  std::string autoverificar() const;
  /** Una linea del CSV. */
  void registrar(const unsigned ahora, const float tInferenciaUs);

  NaoRLWalkEngine& engine;
  Estado estado = Estado::interpolando;
  unsigned tInicio = 0;
  unsigned tEnHome = 0;
  unsigned tMarcha = 0;
  unsigned tAnterior = 0;
  float dtUltimo = .012f;
  unsigned ciclo = 0;
  unsigned decisiones = 0;

  std::array<float, Joints::numOfJoints> anguloInicial{};
  std::array<float, Joints::numOfJoints> anguloValido{};   /**< Ultimo angulo medido finito. */
  std::array<float, Joints::numOfJoints> deseados{};       /**< Lo que pide el estado (antes de limitar). */
  std::array<float, Joints::numOfJoints> objetivos{};      /**< Lo que se manda (ya limitado). */
  static constexpr std::size_t nRetardo = 8;
  std::array<std::array<float, Joints::numOfJoints>, nRetardo> objetivosPasados{};
  std::array<int, Joints::numOfJoints> cuentaSeguimiento{};
  std::array<int, Joints::numOfJoints> cuentaCorriente{};
  bool primerCiclo = true;
  bool fallaAngulo = false;

  NaoRL::Decisor decisor;          /**< Historia, ultima accion valida y fallos seguidos (H2). */
  int excesosInferencia = 0;
  float ultimaInferenciaUs = 0.f;
  NaoRL::EntradaPaso entrada;      /**< Sensores del ciclo actual (gyro, arriba, angulos, vis). */
  NaoRL::EstadoVision vision;      /**< Ultimo percepto valido y meta cerca. */
  float sumaCorrienteHome = 0.f;   /**< H7: suma de |corriente| de las piernas durante enHome [mA]. */
  unsigned muestrasCorrienteHome = 0;
};
