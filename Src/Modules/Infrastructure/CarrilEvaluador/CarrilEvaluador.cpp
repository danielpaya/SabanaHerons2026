/**
 * @file CarrilEvaluador.cpp
 *
 * Implementación del registrador de evaluación del carril (ver CarrilEvaluador.h).
 */

#include "CarrilEvaluador.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(CarrilEvaluador);

void CarrilEvaluador::update(CarrilEvaluacion& theCarrilEvaluacion)
{
  // Solo en simulación: en el NAO no hay pose real y no se escribe nada.
  if(!rutaCsv.empty() && SystemCall::getMode() == SystemCall::simulatedRobot)
  {
    if(!intentadoAbrir)
    {
      intentadoAbrir = true;
      csv.open(rutaCsv, std::ios::trunc);
      if(csv.is_open())
        csv << "t,valido,lineas,d_est,psi_est_grad,ancho_est,pts_izq,pts_der,x_real,y_real,psi_real_grad,"
               "y_pie_izq,y_pie_der,soporte,meta,dist_meta,cam,estado_cuerpo\n";
    }
    // Una fila por imagen de la cámara inferior.
    if(csv.is_open() && theLowerCarrilPercept.tiempoFrame != ultimoTiempoFrame)
    {
      ultimoTiempoFrame = theLowerCarrilPercept.tiempoFrame;
      const Pose2f& poseReal = theGroundTruthWorldState.ownPose;
      csv << theLowerCarrilPercept.tiempoFrame << ','
          << (theLowerCarrilPercept.valido ? 1 : 0) << ','
          << theLowerCarrilPercept.numLineas << ','
          << theLowerCarrilPercept.d << ','
          << theLowerCarrilPercept.psi.toDegrees() << ','
          << theLowerCarrilPercept.anchoEstimado << ','
          << theLowerCarrilPercept.numPuntosIzq << ','
          << theLowerCarrilPercept.numPuntosDer << ','
          << poseReal.translation.x() << ','
          << poseReal.translation.y() << ','
          << Angle(poseReal.rotation).toDegrees() << ',';

      // Bordes exteriores de los pies en coordenadas del mundo (y del carril): suela en el
      // marco del suelo (torsoMatrix * sole, mismo patrón que el WalkingEngine), más medio
      // pie hacia fuera, transformados con la pose real del torso.
      const Vector3f suelaIzq = (theMotionTorsoMatrix * theMotionRobotModel.soleLeft).translation;
      const Vector3f suelaDer = (theMotionTorsoMatrix * theMotionRobotModel.soleRight).translation;
      const Vector2f bordeIzq = poseReal * Vector2f(suelaIzq.x(), suelaIzq.y() + medioAnchoPie);
      const Vector2f bordeDer = poseReal * Vector2f(suelaDer.x(), suelaDer.y() - medioAnchoPie);
      csv << bordeIzq.y() << ','
          << bordeDer.y() << ','
          << theMotionFootSupport.support << ','
          << (theLowerCarrilPercept.metaVisible ? 1 : 0) << ','
          << theLowerCarrilPercept.distanciaMeta << ','
          << (theLowerCameraMatrix.isValid ? 1 : 0) << ','
          << static_cast<int>(theMotionFallDownState.state) << '\n'; // FallDownState::State: 1 de pie, 2 tambaleo, 3 cayendo, 4 caído
      csv.flush();
      ++numFrames;
    }
  }
  theCarrilEvaluacion.numFrames = numFrames;
}
