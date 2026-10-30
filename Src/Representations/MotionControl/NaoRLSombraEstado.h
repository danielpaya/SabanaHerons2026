/**
 * @file NaoRLSombraEstado.h
 *
 * Representacion trivial del modo sombra: existe para que B-Human instancie NaoRLSombra (un modulo tiene que proveer algo).
 * Nadie la usa para decidir nada.
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(NaoRLSombraEstado,
{,
  (bool)(false) activa,     /**< La politica cargo y el registro esta abierto. */
  (unsigned)(0) decisiones, /**< Decisiones evaluadas desde el arranque. */
});
