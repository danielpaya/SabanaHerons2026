/**
 * @file CarrilEvaluador.h
 *
 * Módulo de evaluación del percepto del carril, SOLO para simulación (escenario Carril):
 * registra en un CSV, frame a frame, la estimación (d, psi) de la cámara inferior junto
 * con la pose real del robot que publica SimRobot (GroundTruthWorldState), para medir
 * el error medio y máximo con Util/CarrilTools/evaluar_carril.py.
 *
 * Con rutaCsv vacía no escribe nada.
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/CarrilEvaluacion.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Perception/Carril/CarrilPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include <fstream>

/** Representaciones de otros hilos, recibidas en Cognition (alias que resuelve el framework). */
STREAMABLE_WITH_BASE(MotionRobotModel, RobotModel, {,});
STREAMABLE_WITH_BASE(MotionTorsoMatrix, TorsoMatrix, {,});
STREAMABLE_WITH_BASE(MotionFootSupport, FootSupport, {,});
STREAMABLE_WITH_BASE(MotionFallDownState, FallDownState, {,});
STREAMABLE_WITH_BASE(LowerCameraMatrix, CameraMatrix, {,});

MODULE(CarrilEvaluador,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GroundTruthWorldState),
  REQUIRES(LowerCameraMatrix),
  REQUIRES(LowerCarrilPercept),
  REQUIRES(MotionFallDownState),
  REQUIRES(MotionFootSupport),
  REQUIRES(MotionRobotModel),
  REQUIRES(MotionTorsoMatrix),
  PROVIDES(CarrilEvaluacion),
  LOADS_PARAMETERS(
  {,
    (std::string) rutaCsv,   /**< Ruta del CSV de evaluación; vacía = no escribir. */
    (float) medioAnchoPie,   /**< Medio ancho del pie (mm): del centro de la suela a su borde exterior. */
  }),
});

class CarrilEvaluador : public CarrilEvaluadorBase
{
  void update(CarrilEvaluacion& theCarrilEvaluacion) override;

  std::ofstream csv;             /**< Archivo de salida; se abre (truncando) en el primer frame. */
  bool intentadoAbrir = false;   /**< Para abrir el archivo una sola vez. */
  unsigned ultimoTiempoFrame = 0; /**< Evita filas duplicadas cuando Cognition va más rápido que Lower. */
  unsigned numFrames = 0;
};
