/**
 * @file CarrilPerceptProvider.cpp
 *
 * Implementación del detector de líneas del carril (ver CarrilPerceptProvider.h).
 *
 * Idea: en cada fila muestreada de la imagen se buscan tramos de píxeles oscuros y
 * acromáticos; cada tramo se proyecta al suelo con la matriz de cámara y se descarta si
 * su ancho en el suelo no es compatible con una línea. Los tramos se encadenan entre
 * filas por continuidad horizontal en la imagen, de modo que cada cadena es una línea
 * candidata aunque la otra quede fuera del campo visual (robot girado). Con dos cadenas
 * se verifica paralelismo y ancho del carril; con una sola, el lado se decide por el
 * signo de su posición lateral y d se deduce del ancho nominal.
 *
 * Geometría (coordenadas del robot, eje del carril con dirección unitaria u y normal
 * izquierda n): si p0 es un punto del eje, la pose del robot respecto al carril es
 * psi = -ángulo(u) y d = -<p0, n>.
 */

#include "CarrilPerceptProvider.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <cmath>

MAKE_MODULE(CarrilPerceptProvider);

void CarrilPerceptProvider::update(CarrilPercept& theCarrilPercept)
{
  theCarrilPercept.valido = false;
  theCarrilPercept.numLineas = 0;
  theCarrilPercept.numPuntosIzq = 0;
  theCarrilPercept.numPuntosDer = 0;
  theCarrilPercept.anchoEstimado = 0.f;
  theCarrilPercept.metaVisible = false;
  theCarrilPercept.distanciaMeta = 0.f;
  theCarrilPercept.numPuntosMeta = 0;
  theCarrilPercept.tiempoFrame = theFrameInfo.time;

  if(!theCameraMatrix.isValid)
    return;

  // Encadenar los tramos de cada fila (de abajo arriba) por continuidad en la imagen.
  std::vector<Cadena> cadenas;
  std::vector<Tramo> tramos;
  std::vector<Vector2f> puntosMeta;
  for(int fila = theCameraInfo.height - 1; fila >= 0; fila -= pasoFilas)
  {
    tramos.clear();
    buscarTramosEnFila(fila, tramos, puntosMeta);
    for(const Tramo& tramo : tramos)
    {
      Cadena* mejor = nullptr;
      float mejorDistancia = 0.f;
      for(Cadena& cadena : cadenas)
      {
        // Tolerancia proporcional a las filas muestreadas sin ver la cadena (oclusiones cortas).
        const float filasSinVer = static_cast<float>(cadena.ultimaFila - fila) / static_cast<float>(pasoFilas);
        const float tolerancia = std::min(maxSaltoPx * filasSinVer, 4.f * maxSaltoPx);
        const float distancia = std::abs(tramo.xImagen - cadena.ultimaX);
        if(distancia <= tolerancia && (!mejor || distancia < mejorDistancia))
        {
          mejor = &cadena;
          mejorDistancia = distancia;
        }
      }
      if(!mejor)
      {
        cadenas.emplace_back();
        mejor = &cadenas.back();
      }
      mejor->ultimaX = tramo.xImagen;
      mejor->ultimaFila = fila;
      mejor->puntos.push_back(tramo.enSuelo);
    }
  }

  // Meta roja: con suficientes puntos, la distancia es la media de sus x (centro visible).
  if(puntosMeta.size() >= minPuntosMeta)
  {
    float suma = 0.f;
    for(const Vector2f& p : puntosMeta)
      suma += p.x();
    theCarrilPercept.metaVisible = true;
    theCarrilPercept.distanciaMeta = suma / static_cast<float>(puntosMeta.size());
    theCarrilPercept.numPuntosMeta = static_cast<unsigned>(puntosMeta.size());
  }

  // Quedarse con las dos cadenas más largas que alcancen el mínimo de puntos.
  std::sort(cadenas.begin(), cadenas.end(),
            [](const Cadena& a, const Cadena& b) {return a.puntos.size() > b.puntos.size();});
  while(cadenas.size() > 2 || (!cadenas.empty() && cadenas.back().puntos.size() < minPuntosPorLinea))
    cadenas.pop_back();
  if(cadenas.empty())
    return;

  const float anchoEntreCentros = anchoCarril + anchoLinea;
  if(cadenas.size() == 2)
  {
    Vector2f centro1, dir1, centro2, dir2;
    if(!ajustarRecta(cadenas[0].puntos, centro1, dir1) || !ajustarRecta(cadenas[1].puntos, centro2, dir2))
      return;
    // Ambas direcciones hacia delante del robot para poder compararlas y promediarlas.
    if(dir1.x() < 0.f)
      dir1 = -dir1;
    if(dir2.x() < 0.f)
      dir2 = -dir2;
    if(std::abs(Angle::normalize(dir1.angle() - dir2.angle())) > maxAnguloEntreLineas)
      return;

    const Vector2f u = (dir1 + dir2).normalized();
    const Vector2f n(-u.y(), u.x());
    const float s1 = centro1.dot(n);
    const float s2 = centro2.dot(n);
    const float ancho = std::abs(s1 - s2);
    theCarrilPercept.anchoEstimado = ancho;
    theCarrilPercept.numPuntosIzq = static_cast<unsigned>(cadenas[s1 >= s2 ? 0 : 1].puntos.size());
    theCarrilPercept.numPuntosDer = static_cast<unsigned>(cadenas[s1 >= s2 ? 1 : 0].puntos.size());
    if(std::abs(ancho - anchoEntreCentros) > toleranciaAncho)
      return;

    theCarrilPercept.psi = -std::atan2(u.y(), u.x());
    theCarrilPercept.d = -0.5f * (s1 + s2);
    theCarrilPercept.numLineas = 2;
    theCarrilPercept.valido = true;
  }
  else
  {
    Vector2f centro, dir;
    if(!ajustarRecta(cadenas[0].puntos, centro, dir))
      return;
    if(dir.x() < 0.f)
      dir = -dir;
    const Vector2f u = dir;
    const Vector2f n(-u.y(), u.x());
    const float s = centro.dot(n);
    // Lado de la línea por el signo de su posición lateral: válido mientras el robot
    // esté dentro del carril (en la carrera, salirse lo anula de todos modos).
    if(s >= 0.f) // línea izquierda
    {
      theCarrilPercept.d = 0.5f * anchoEntreCentros - s;
      theCarrilPercept.numPuntosIzq = static_cast<unsigned>(cadenas[0].puntos.size());
    }
    else // línea derecha
    {
      theCarrilPercept.d = -0.5f * anchoEntreCentros - s;
      theCarrilPercept.numPuntosDer = static_cast<unsigned>(cadenas[0].puntos.size());
    }
    theCarrilPercept.psi = -std::atan2(u.y(), u.x());
    theCarrilPercept.numLineas = 1;
    theCarrilPercept.valido = true;
  }
}

void CarrilPerceptProvider::buscarTramosEnFila(const int fila, std::vector<Tramo>& tramos,
                                               std::vector<Vector2f>& puntosMeta) const
{
  // Clases de píxel: 0 = fondo, 1 = línea negra (oscuro y acromático), 2 = meta roja (V alto).
  const int ancho = theCameraInfo.width;

  // Umbral de oscuridad relativo al fondo de la fila: la mediana de la luminancia es la
  // duela (las líneas ocupan pocos píxeles). Así el umbral sigue a la luz y al tono de la
  // madera, y acepta líneas gastadas o grises; maxYLinea queda como tope absoluto.
  int histograma[256] = {};
  for(int x = 0; x < ancho; ++x)
    ++histograma[theCameraImage.getYUV(static_cast<size_t>(x), static_cast<size_t>(fila)).y];
  int mediana = 0;
  for(int acumulado = 0; mediana < 255 && (acumulado += histograma[mediana]) < ancho / 2;)
    ++mediana;
  const int maxY = std::min(maxYLinea, static_cast<int>(fraccionYLinea * static_cast<float>(mediana)));

  int clase = 0;
  int inicio = -1;
  for(int x = 0; x <= ancho; ++x)
  {
    int claseAqui = 0;
    if(x < ancho)
    {
      const PixelTypes::YUVPixel yuv = theCameraImage.getYUV(static_cast<size_t>(x), static_cast<size_t>(fila));
      if(static_cast<int>(yuv.v) - 128 >= minVRojoMeta)
        claseAqui = 2;
      else if(yuv.y <= maxY
              && std::abs(static_cast<int>(yuv.u) - 128) <= maxCromaLinea
              && std::abs(static_cast<int>(yuv.v) - 128) <= maxCromaLinea)
        claseAqui = 1;
    }
    if(claseAqui != clase)
    {
      if(clase != 0) // termina un tramo de línea o de meta
      {
        const int fin = x - 1;
        const float xCentro = (inicio + fin) * 0.5f;
        Vector2f enSueloInicio, enSueloFin, enSueloCentro;
        if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2f(static_cast<float>(inicio), static_cast<float>(fila))),
                                        theCameraMatrix, theCameraInfo, enSueloInicio)
           && Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2f(static_cast<float>(fin), static_cast<float>(fila))),
                                           theCameraMatrix, theCameraInfo, enSueloFin)
           && Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(Vector2f(xCentro, static_cast<float>(fila))),
                                           theCameraMatrix, theCameraInfo, enSueloCentro))
        {
          const float anchoSuelo = (enSueloFin - enSueloInicio).norm();
          if(enSueloCentro.norm() <= maxDistancia)
          {
            if(clase == 1 && anchoSuelo >= minAnchoLineaSuelo && anchoSuelo <= maxAnchoLineaSuelo)
              tramos.push_back({xCentro, enSueloCentro});
            else if(clase == 2 && anchoSuelo >= minAnchoMetaSuelo && enSueloCentro.x() > 0.f)
              puntosMeta.push_back(enSueloCentro);
          }
        }
      }
      clase = claseAqui;
      inicio = x;
    }
  }
}

bool CarrilPerceptProvider::ajustarRecta(const std::vector<Vector2f>& puntos, Vector2f& centroide, Vector2f& direccion)
{
  if(puntos.size() < 2)
    return false;
  centroide = Vector2f::Zero();
  for(const Vector2f& p : puntos)
    centroide += p;
  centroide /= static_cast<float>(puntos.size());
  float sxx = 0.f, syy = 0.f, sxy = 0.f;
  for(const Vector2f& p : puntos)
  {
    const Vector2f q = p - centroide;
    sxx += q.x() * q.x();
    syy += q.y() * q.y();
    sxy += q.x() * q.y();
  }
  // Dirección principal de la nube de puntos (autovector mayor de la covarianza 2x2).
  const float angulo = 0.5f * std::atan2(2.f * sxy, sxx - syy);
  direccion = Vector2f(std::cos(angulo), std::sin(angulo));
  return true;
}
