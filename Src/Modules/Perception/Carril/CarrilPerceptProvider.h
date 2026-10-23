/**
 * @file CarrilPerceptProvider.h
 *
 * Detecta las dos líneas laterales del carril en la imagen de la cámara inferior,
 * las proyecta al suelo y estima el desplazamiento lateral (d) y la orientación (psi)
 * del robot respecto al eje del carril.
 *
 * Los umbrales de color son parámetros de configuración (carrilPerceptProvider.cfg),
 * nunca constantes en el código: en el carril real se ajustarán con fotos reales.
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/Carril/CarrilPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include <vector>

MODULE(CarrilPerceptProvider,
{,
  REQUIRES(CameraImage),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FrameInfo),
  REQUIRES(ImageCoordinateSystem),
  PROVIDES(CarrilPercept),
  LOADS_PARAMETERS(
  {,
    (int) maxYLinea,              /**< Tope absoluto de luminancia (0..255) de un píxel de línea. */
    (float) fraccionYLinea,       /**< Un píxel de línea es más oscuro que esta fracción de la luminancia
                                       mediana de su fila (umbral relativo: tolera luz y desgaste). */
    (int) maxCromaLinea,          /**< |U-128| y |V-128| máximos: la línea es gris/negra (excluye la meta roja). */
    (int) pasoFilas,              /**< Muestrear una de cada n filas de la imagen. */
    (float) minAnchoLineaSuelo,   /**< Ancho mínimo de un tramo oscuro proyectado al suelo (mm). */
    (float) maxAnchoLineaSuelo,   /**< Ancho máximo de un tramo oscuro proyectado al suelo (mm). */
    (float) maxDistancia,         /**< Distancia máxima al suelo de un punto usado (mm). */
    (unsigned) minPuntosPorLinea, /**< Puntos mínimos para ajustar cada línea. */
    (Angle) maxAnguloEntreLineas, /**< Desalineación máxima entre ambas líneas ajustadas. */
    (float) anchoCarril,          /**< Ancho libre nominal entre bordes internos de líneas (mm). */
    (float) anchoLinea,           /**< Ancho nominal de cada línea (mm). */
    (float) toleranciaAncho,      /**< Desviación máxima del ancho medido frente al nominal (mm). */
    (float) maxSaltoPx,           /**< Salto horizontal máximo (px) entre filas muestreadas para seguir una línea. */
    (int) minVRojoMeta,           /**< Mínimo de (V-128) para considerar un píxel rojo de meta. */
    (float) minAnchoMetaSuelo,    /**< Ancho mínimo en el suelo de un tramo rojo de meta (mm). */
    (unsigned) minPuntosMeta,     /**< Puntos rojos mínimos para dar la meta por vista. */
  }),
});

class CarrilPerceptProvider : public CarrilPerceptProviderBase
{
  /** Un tramo oscuro de una fila: su centro en la imagen y proyectado al suelo. */
  struct Tramo
  {
    float xImagen;
    Vector2f enSuelo;
  };

  /** Puntos de una misma línea, encadenados por continuidad entre filas de la imagen. */
  struct Cadena
  {
    float ultimaX;
    int ultimaFila;
    std::vector<Vector2f> puntos;
  };

  void update(CarrilPercept& theCarrilPercept) override;

  /** Tramos oscuros (línea) y rojos (meta) de la fila `fila` proyectados al suelo (coordenadas del robot, mm). */
  void buscarTramosEnFila(int fila, std::vector<Tramo>& tramos, std::vector<Vector2f>& puntosMeta) const;

  /** Ajusta una recta por mínimos cuadrados totales. Devuelve false con menos de dos puntos. */
  static bool ajustarRecta(const std::vector<Vector2f>& puntos, Vector2f& centroide, Vector2f& direccion);
};
