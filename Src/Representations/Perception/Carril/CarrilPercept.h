/**
 * @file CarrilPercept.h
 *
 * Estado del robot respecto al carril de la Carrera Individual (Copa NAO CCM MX 2026),
 * estimado a partir de las líneas laterales vistas por la cámara inferior.
 *
 * Convenio de coordenadas (igual que la pose del robot en el simulador):
 * el eje del carril apunta hacia la meta; d > 0 significa robot desplazado a la
 * izquierda del eje; psi > 0 significa robot girado a la izquierda (antihorario).
 */

#pragma once

#include "Math/Angle.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(CarrilPercept,
{,
  (bool)(false) valido,          /**< true si la estimación de d y psi es utilizable. */
  (unsigned)(0) numLineas,       /**< Líneas usadas: 2 = ambas (geometría verificada), 1 = solo una
                                      (el lado se decide por el signo de su posición; d menos fiable). */
  (float)(0.f) d,                /**< Desplazamiento lateral del robot respecto al eje del carril (mm). */
  (Angle)(0_deg) psi,            /**< Orientación del robot respecto al eje del carril. */
  (float)(0.f) anchoEstimado,    /**< Ancho estimado entre centros de líneas (mm); 0 con una sola línea. */
  (unsigned)(0) numPuntosIzq,    /**< Puntos proyectados al suelo usados en la línea izquierda. */
  (unsigned)(0) numPuntosDer,    /**< Puntos proyectados al suelo usados en la línea derecha. */
  (bool)(false) metaVisible,     /**< true si la franja roja de meta se ve en la imagen. */
  (float)(0.f) distanciaMeta,    /**< Distancia en el suelo al centro visible de la meta (mm, hacia delante). */
  (unsigned)(0) numPuntosMeta,   /**< Puntos proyectados al suelo usados para la meta. */
  (unsigned)(0) tiempoFrame,     /**< Marca de tiempo de la imagen de la que proviene la estimación. */
});

/** El percepto del hilo Lower visto desde otros hilos (alias que resuelve el framework). */
STREAMABLE_WITH_BASE(LowerCarrilPercept, CarrilPercept, {,});
