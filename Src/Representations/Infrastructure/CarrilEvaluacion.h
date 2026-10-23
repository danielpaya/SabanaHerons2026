/**
 * @file CarrilEvaluacion.h
 *
 * Representación de salida del módulo CarrilEvaluador (solo simulación): cuenta los
 * frames evaluados. Los datos de la evaluación van a un CSV (ver carrilEvaluador.cfg).
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(CarrilEvaluacion,
{,
  (unsigned)(0) numFrames, /**< Frames del percepto registrados en el CSV. */
});
