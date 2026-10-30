/**
 * @file NaoRLPolitica.h
 *
 * Parte "pura" (sin B-Human) de la marcha RL nao_v08 para la Carrera Individual: la red del actor, la historia de la
 * observacion y la conversion accion -> angulos objetivo. Todo sigue el contrato docs/despliegue_nao_v08.md (copia en
 * demo_carril/docs/) y se comprueba contra Python con el golden JSON (Src/Apps/Tests/NaoRL/ParidadNaoV08.cpp, etapa E0).
 *
 * Lo que NO esta aqui (va en el modulo NaoRLWalkEngine): leer los sensores del robot, los signos por canal, el reloj
 * de 24 ms y toda la seguridad.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace NaoRL
{
  constexpr double pi = 3.14159265358979323846;
  constexpr float cicloS = .012f; /**< Ciclo del hilo Motion / LoLA. */
  constexpr unsigned ciclosPorDecision = 2; /**< 24 ms (contrato, seccion 2). */

  constexpr std::size_t nMotores = 15;  /**< Tamano de la accion (contrato, seccion 1). */
  constexpr std::size_t obsPorPaso = 42; /**< Bloque de un paso (contrato, seccion 3). */
  constexpr std::size_t nHistoria = 3;   /**< Pasos en la observacion: [actual, anterior, anterior-1]. */
  constexpr std::size_t nObs = obsPorPaso * nHistoria; /**< 126. */

  using Accion = std::array<float, nMotores>;
  using Bloque = std::array<float, obsPorPaso>;
  using Observacion = std::array<float, nObs>;

  /** Orden de la accion y de "ang"/"acc" en la observacion (contrato, seccion 1). */
  constexpr std::array<const char*, nMotores> nombresMotores = {
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
    "LShoulderPitch", "RShoulderPitch", "LElbowRoll", "RElbowRoll",
  };

  /** Indices dentro de la accion (para las reglas por articulacion). */
  enum IndiceMotor : std::size_t
  {
    mLHipYawPitch, mLHipRoll, mLHipPitch, mLKneePitch, mLAnklePitch, mLAnkleRoll,
    mRHipRoll, mRHipPitch, mRKneePitch, mRAnklePitch, mRAnkleRoll,
    mLShoulderPitch, mRShoulderPitch, mLElbowRoll, mRElbowRoll,
  };

  /** Las cuatro articulaciones de roll (motores lentos, factor de velocidad y umbral de seguimiento propios). */
  constexpr bool esRoll(std::size_t i)
  {
    return i == mLHipRoll || i == mLAnkleRoll || i == mRHipRoll || i == mRAnkleRoll;
  }

  /** Una capa densa: filas = salidas, columnas = entradas, pesos por filas (W de JAX traspuesta). */
  struct Capa
  {
    unsigned filas = 0;
    unsigned columnas = 0;
    std::vector<float> pesos;
    std::vector<float> sesgo;

    bool valida() const
    {
      return filas > 0 && columnas > 0 && pesos.size() == static_cast<std::size_t>(filas) * columnas && sesgo.size() == filas;
    }
  };

  constexpr float limiteEntradaNormalizada = 10.f; /**< H2: en el entrenamiento nunca se acerca; no cambia E0. */

  /**
   * El actor determinista: x = (obs - media) / desviacion, recortada a +-limiteEntradaNormalizada (contrato 6c, H2), capas
   * ocultas con swish, capa de salida lineal, accion = tanh(primera mitad de la salida). La segunda mitad (log-std) se
   * ignora. evaluar() no reserva memoria (corre en el hilo Motion).
   */
  class Politica
  {
  public:
    /** Configura la red. Devuelve false y explica en error si las dimensiones no cuadran con el contrato. */
    bool configurar(const std::vector<float>& mediaIn, const std::vector<float>& desviacionIn, const std::vector<Capa>& ocultasIn,
                    const Capa& salidaIn, std::string& error)
    {
      valida_ = false;
      if(mediaIn.size() != nObs || desviacionIn.size() != nObs)
      {
        error = "media/desviacion deben tener " + std::to_string(nObs) + " valores";
        return false;
      }
      for(const float s : desviacionIn)
        if(!(s > 0.f) || !std::isfinite(s))
        {
          error = "desviacion con un valor no positivo o no finito";
          return false;
        }
      if(ocultasIn.empty())
      {
        error = "sin capas ocultas";
        return false;
      }
      unsigned entrada = static_cast<unsigned>(nObs);
      std::size_t ancho = nObs;
      for(const Capa& c : ocultasIn)
      {
        if(!c.valida() || c.columnas != entrada)
        {
          error = "capa oculta con dimensiones inconsistentes";
          return false;
        }
        entrada = c.filas;
        ancho = std::max<std::size_t>(ancho, c.filas);
      }
      if(!salidaIn.valida() || salidaIn.columnas != entrada || salidaIn.filas != 2 * nMotores)
      {
        error = "capa de salida debe ser " + std::to_string(2 * nMotores) + " x " + std::to_string(entrada);
        return false;
      }
      const auto todoFinito = [](const std::vector<float>& v)
      {
        return std::all_of(v.begin(), v.end(), [](float x) { return std::isfinite(x); });
      };
      bool finitos = todoFinito(mediaIn) && todoFinito(salidaIn.pesos) && todoFinito(salidaIn.sesgo);
      for(const Capa& c : ocultasIn)
        finitos = finitos && todoFinito(c.pesos) && todoFinito(c.sesgo);
      if(!finitos)
      {
        error = "pesos, sesgos o media no finitos";
        return false;
      }
      media = mediaIn;
      desviacion = desviacionIn;
      ocultas = ocultasIn;
      salida = salidaIn;
      a.assign(ancho, 0.f);
      b.assign(ancho, 0.f);
      valida_ = true;
      return true;
    }

    bool valida() const { return valida_; }

    /**
     * Que tan fuera de distribucion esta una observacion: |z| = |obs - media| / desviacion, sin recortar. Un no finito cuenta
     * como infinito.
     * @param zmax, zmaxIdx La entrada mas alejada (la primera, si empatan).
     * @param nZ4 Cuantas entradas tienen |z| > umbral.
     */
    void estadisticasZ(const Observacion& obs, const float umbral, float& zmax, int& zmaxIdx, int& nZ4) const
    {
      zmax = -1.f;
      zmaxIdx = 0;
      nZ4 = 0;
      for(std::size_t i = 0; i < nObs; ++i)
      {
        float z = std::abs((obs[i] - media[i]) / desviacion[i]);
        if(!std::isfinite(z))
          z = HUGE_VALF;
        if(z > zmax)
        {
          zmax = z;
          zmaxIdx = static_cast<int>(i);
        }
        if(z > umbral)
          ++nZ4;
      }
    }

    /** Evalua la red. Puede devolver NaN/Inf si la entrada lo es: el que llama debe filtrar. */
    Accion evaluar(const Observacion& obs)
    {
      for(std::size_t i = 0; i < nObs; ++i)
        a[i] = std::clamp((obs[i] - media[i]) / desviacion[i], -limiteEntradaNormalizada, limiteEntradaNormalizada);
      for(const Capa& c : ocultas)
      {
        densa(c, a.data(), b.data());
        for(unsigned i = 0; i < c.filas; ++i)
          b[i] = b[i] / (1.f + std::exp(-b[i])); // swish
        std::swap(a, b);
      }
      densa(salida, a.data(), b.data());
      Accion accion;
      for(std::size_t i = 0; i < nMotores; ++i)
        accion[i] = std::tanh(b[i]);
      return accion;
    }

  private:
    static void densa(const Capa& c, const float* x, float* y)
    {
      const float* w = c.pesos.data();
      for(unsigned f = 0; f < c.filas; ++f, w += c.columnas)
      {
        float s = c.sesgo[f];
        for(unsigned k = 0; k < c.columnas; ++k)
          s += w[k] * x[k];
        y[f] = s;
      }
    }

    bool valida_ = false;
    std::vector<float> media, desviacion;
    std::vector<Capa> ocultas;
    Capa salida;
    std::vector<float> a, b; /**< Buffers de trabajo (reservados en configurar()). */
  };

  /** Lo que entra a un bloque de 42, ya en el marco y con los signos del entrenamiento (ver el modulo). */
  struct EntradaPaso
  {
    std::array<float, 3> gyro{};    /**< rad/s, marco del torso (x adelante, y izquierda, z arriba). */
    std::array<float, 3> arriba{};  /**< "Arriba del mundo" en el marco del torso; derecho = (0, 0, 1). */
    Accion angulos{};               /**< Angulo medido de los 15 motores (rad), orden de la accion. */
    Accion accionAnterior{};        /**< Accion cruda de la decision previa, en [-1, 1]. */
    float vxCmd = 0.f;              /**< m/s. */
    float wzCmd = 0.f;              /**< rad/s. */
    float tMarcha = 0.f;            /**< s desde que arranco la marcha. */
    float frecuencia = 1.8f;        /**< Hz del reloj de fase. */
    float psi = 0.f;                /**< rad, ya con el signo del entrenamiento. */
    float d = 0.f;                  /**< m, ya con el signo del entrenamiento. */
  };

  constexpr float limitePsi = .5f; /**< rad (contrato, seccion 3). */
  constexpr float limiteD = .5f;   /**< m. */

  /** Arma el bloque de 42 en el orden EXACTO del contrato (seccion 3). */
  inline Bloque armarBloque(const EntradaPaso& e, const Accion& postura)
  {
    Bloque bl{};
    std::size_t k = 0;
    for(const float g : e.gyro)                                   // 0-2
      bl[k++] = g;
    for(const float u : e.arriba)                                 // 3-5
      bl[k++] = u;
    for(std::size_t i = 0; i < nMotores; ++i)                     // 6-20
      bl[k++] = e.angulos[i] - postura[i];
    for(const float a : e.accionAnterior)                         // 21-35
      bl[k++] = a;
    bl[k++] = e.vxCmd;                                            // 36
    bl[k++] = e.wzCmd;                                            // 37
    const double fase = 2. * pi * e.frecuencia * e.tMarcha;
    bl[k++] = static_cast<float>(std::cos(fase));                 // 38
    bl[k++] = static_cast<float>(std::sin(fase));                 // 39
    bl[k++] = std::clamp(e.psi, -limitePsi, limitePsi);           // 40
    bl[k++] = std::clamp(e.d, -limiteD, limiteD);                 // 41
    return bl;
  }

  /**
   * Historia de 3 pasos: el bloque actual va primero (0..41), luego los anteriores. Empieza en CEROS (no en copias
   * del primer bloque).
   */
  class Historia
  {
  public:
    void reiniciar() { obs.fill(0.f); }

    /** Desplaza [b0, b1, b2] -> [nuevo, b0, b1] y devuelve la observacion completa. */
    const Observacion& empujar(const Bloque& nuevo)
    {
      std::copy_backward(obs.begin(), obs.end() - obsPorPaso, obs.end());
      std::copy(nuevo.begin(), nuevo.end(), obs.begin());
      return obs;
    }

    const Observacion& observacion() const { return obs; }

  private:
    Observacion obs{};
  };

  /** objetivo[i] = clip(postura[i] + a[i] * escala, lo[i], hi[i]) (contrato, seccion 4). */
  inline Accion accionAObjetivos(const Accion& accion, const Accion& postura, float escala, const Accion& lo, const Accion& hi)
  {
    Accion objetivo;
    for(std::size_t i = 0; i < nMotores; ++i)
      objetivo[i] = std::clamp(postura[i] + accion[i] * escala, lo[i], hi[i]);
    return objetivo;
  }

  /** true si los 15 valores son finitos. */
  inline bool finita(const Accion& a)
  {
    return std::all_of(a.begin(), a.end(), [](float x) { return std::isfinite(x); });
  }
}
