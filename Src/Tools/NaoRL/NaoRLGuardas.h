/**
 * @file NaoRLGuardas.h
 *
 * Guardas de integridad de la orden a los motores (contrato demo_carril/docs/despliegue_nao_v08.md, seccion 6c, H1-H7).
 * Es la parte "pura" (sin B-Human): la llama NaoRLWalkEngine y la prueban los tests NaoRL.* con EL MISMO codigo. La carga
 * del .cfg (H3) esta en NaoRLPoliticaConfig.h. Lo que queda fuera de los tests es el cableado dentro de NaoRLWalkPhase.
 *
 * Todas las comparaciones estan escritas para que un NaN las haga FALLAR (!(x <= limite)), nunca pasar.
 */

#pragma once

#include "NaoRLPolitica.h" // misma carpeta: asi tambien compila en Tests, que no tiene Src/ en la ruta
#include <array>
#include <cmath>
#include <string>

namespace NaoRL
{
  constexpr float sensorApagado = 10000.f; /**< = SensorData::off (el modulo lo comprueba). */
  constexpr std::size_t nPiernas = 12;     /**< Articulaciones de pierna en Joints (incluye rHipYawPitch). */

  // -------------------------------------------------------------------------------------------------------------------
  // H1. Camara: un valor no finito o fuera de rango es "sin medicion nueva", NO un fallo de la red.

  constexpr float maxPsiCamara = static_cast<float>(pi); /**< rad. */
  constexpr float maxDCamara = 2.f;                      /**< m. */

  inline bool medicionCamaraValida(const float psi, const float dMetros)
  {
    return std::abs(psi) <= maxPsiCamara && std::abs(dMetros) <= maxDCamara;
  }

  /** Una distanciaMeta mala no debe activar "meta cerca", que desactiva el paro por camara. */
  inline bool distanciaMetaValida(const float distanciaMm)
  {
    return std::isfinite(distanciaMm) && distanciaMm >= 0.f;
  }

  /**
   * Actualiza (psi, d) de la observacion con una foto de CarrilPercept (psi en rad, d en mm). Con una medicion no valida no
   * toca nada (se mantiene el ultimo valor valido) y devuelve false: el que llama no debe contarla como foto valida.
   */
  inline bool actualizarVision(const float psiCamara, const float dCamaraMm, const float signoPsi, const float signoD, float& psi, float& d)
  {
    const float dMetros = dCamaraMm / 1000.f;
    if(!medicionCamaraValida(psiCamara, dMetros))
      return false;
    psi = signoPsi * psiCamara;
    d = signoD * dMetros;
    return true;
  }

  // -------------------------------------------------------------------------------------------------------------------
  // H2. Plausibilidad de la observacion, antes de evaluar la red.

  constexpr float maxGyroPlausible = 20.f;       /**< rad/s por eje. */
  constexpr float maxComponenteArriba = 1.05f;
  constexpr float toleranciaNormaArriba = .1f;
  constexpr float holguraAngulos = .2f;          /**< rad alrededor de jointLimits. */

  /** nullptr si los sensores son plausibles; si no, el motivo (texto estatico: no reserva memoria en el hilo Motion). */
  inline const char* motivoNoPlausible(const std::array<float, 3>& gyro, const std::array<float, 3>& arriba, const Accion& angulos,
                                       const Accion& minimos, const Accion& maximos)
  {
    for(const float g : gyro)
      if(!(std::abs(g) < maxGyroPlausible))
        return "gyro fuera de rango";
    float norma2 = 0.f;
    for(const float u : arriba)
    {
      if(!(std::abs(u) <= maxComponenteArriba))
        return "arriba fuera de rango";
      norma2 += u * u;
    }
    if(!(std::abs(std::sqrt(norma2) - 1.f) <= toleranciaNormaArriba))
      return "norma de arriba distinta de 1";
    for(std::size_t i = 0; i < nMotores; ++i)
      if(!(angulos[i] >= minimos[i] - holguraAngulos && angulos[i] <= maximos[i] + holguraAngulos))
        return "angulo fuera de jointLimits";
    return nullptr;
  }

  /**
   * Una decision de la politica con el filtro de entrada y de salida. Con entrada mala la red NO se evalua, se repite la
   * ultima accion valida y el bloque NO entra en la historia (si entrara, un solo dato malo envenenaria 3 decisiones
   * seguidas y acabaria en paro). Tras maxFallos decisiones malas seguidas pide el paro.
   */
  class Decisor
  {
  public:
    struct Resultado
    {
      Accion accion{};            /**< Lo que se debe mandar. */
      bool redEvaluada = false;   /**< false = no se evaluo o su salida no se uso: se repite la ultima accion valida. */
      bool paro = false;          /**< Se llego a maxFallos. */
      const char* motivo = nullptr; /**< Motivo del fallo de esta decision (nullptr = sin fallo). */
    };

    void reiniciar()
    {
      historia.reiniciar();
      anterior.fill(0.f);
      previa.fill(0.f);
      fallos = 0;
    }

    /** @param motivoEntrada Sensores malos (motivoNoPlausible o un angulo no finito); nullptr = sensores bien. */
    Resultado decidir(Politica& red, EntradaPaso e, const Accion& postura, const char* motivoEntrada, const int maxFallos)
    {
      Resultado r;
      previa = anterior;
      e.accionAnterior = anterior;
      r.motivo = motivoEntrada;
      if(!r.motivo)
      {
        const Bloque bloque = armarBloque(e, postura);
        if(!std::all_of(bloque.begin(), bloque.end(), [](float v) { return std::isfinite(v); }))
          r.motivo = "observacion no finita";
        else
        {
          const Accion salida = red.evaluar(historia.empujar(bloque));
          if(finita(salida))
          {
            r.accion = salida;
            r.redEvaluada = true;
          }
          else
            r.motivo = "salida de la red no finita";
        }
      }
      if(r.motivo)
      {
        r.accion = anterior;
        r.paro = ++fallos >= maxFallos;
      }
      else
        fallos = 0;
      anterior = r.accion;
      return r;
    }

    /** El watchdog descarto la ultima decision: se vuelve a la accion anterior a ella. */
    const Accion& descartarUltima()
    {
      anterior = previa;
      return anterior;
    }

    /** La observacion que decidir(e) le daria a la red, sin cambiar nada (aunque luego la rechace el filtro). */
    Observacion observacionSiguiente(EntradaPaso e, const Accion& postura) const
    {
      e.accionAnterior = anterior;
      Historia copia = historia;
      return copia.empujar(armarBloque(e, postura));
    }

    /** La ultima observacion que entro en la historia. */
    const Observacion& observacion() const { return historia.observacion(); }

    const Accion& ultimaAccion() const { return anterior; }
    int fallosSeguidos() const { return fallos; }

  private:
    Historia historia;
    Accion anterior{};  /**< Ultima accion valida (cruda, [-1, 1]). */
    Accion previa{};    /**< La de antes de la ultima decision. */
    int fallos = 0;
  };

  // -------------------------------------------------------------------------------------------------------------------
  // H4. Rangos duros de los umbrales de seguridad: fuera de rango la politica NO se arma (no basta con advertir).
  // PROPUESTAS SIN MEDIR: fijarlas con la medicion de linea base del contrato (6c), no a ojo.

  enum ParametroSeguridad : std::size_t
  {
    // Rangos del contrato (6c, H4).
    pMaxInclinacion, pMaxTemperatura, pUmbralSeguimiento, pUmbralSeguimientoRoll, pCiclosSeguimiento, pUmbralCorriente,
    pCiclosCorriente, pMargenLimites, pFactorVelRoll, pStiffnessParo, pPresupuestoInferenciaUs, pVxCmd,
    // No estan en el contrato; misma regla ("un parametro que desactive una proteccion debe impedir armar").
    pRetardoSeguimiento, pMaxFallosNaN, pMaxExcesosInferencia, pMaxEdadCamaraMs, pMaxMarchaMs, pMaxCorrienteHome,
    numParametrosSeguridad,
  };

  struct RangoSeguridad
  {
    const char* nombre;
    float minimo;
    bool minimoIncluido;
    float maximo;
    bool maximoIncluido;
  };

  constexpr std::array<RangoSeguridad, numParametrosSeguridad> rangosSeguridad = {{
    {"maxInclinacion",          .1f,   false, .6f,     true},  // (0.1, 0.6] rad
    {"maxTemperatura",          40.f,  true,  75.f,    true},  // [40, 75] C
    {"umbralSeguimiento",       .03f,  false, .3f,     true},  // (0.03, 0.30] rad
    {"umbralSeguimientoRoll",   .03f,  false, .3f,     true},  // (0.03, 0.30] rad
    {"ciclosSeguimiento",       2.f,   true,  20.f,    true},  // [2, 20]
    {"umbralCorriente",         0.f,   false, 2500.f,  true},  // (0, 2500] mA
    {"ciclosCorriente",         3.f,   true,  100.f,   true},  // [3, 100]
    {"margenLimites",           .02f,  true,  .2f,     true},  // [0.02, 0.2] rad
    {"factorVelRoll",           0.f,   false, 1.f,     true},  // (0, 1]
    {"stiffnessParo",           0.f,   true,  30.f,    true},  // [0, 30] %
    {"presupuestoInferenciaUs", 0.f,   false, 12000.f, true},  // (0, 12000] us
    {"vxCmd",                   0.f,   true,  .5f,     true},  // [0, 0.5] m/s
    {"retardoSeguimiento",      0.f,   true,  7.f,     true},  // [0, 7] ciclos: historial de objetivos del modulo
    {"maxFallosNaN",            1.f,   true,  10.f,    true},  // [1, 10] decisiones
    {"maxExcesosInferencia",    1.f,   true,  10.f,    true},  // [1, 10] decisiones
    {"maxEdadCamaraMs",         0.f,   false, 2000.f,  true},  // (0, 2000] ms: = maxEdadSinPercepto de CarrilBehavior
    {"maxMarchaMs",             0.f,   false, 120000.f, true}, // (0, 120] s
    {"maxCorrienteHome",        0.f,   false, 2500.f,  true},  // (0, 2500] mA, igual que umbralCorriente
  }};

  using ValoresSeguridad = std::array<float, numParametrosSeguridad>;

  /** H5(b): ignorarLevantado solo se acepta caminando despacio. */
  constexpr float maxVxConIgnorarLevantado = .2f;

  /** H4 + H5(b). Devuelve false y explica en error si la politica no se debe armar. */
  inline bool validarSeguridad(const ValoresSeguridad& valores, const bool ignorarLevantado, std::string& error)
  {
    for(std::size_t i = 0; i < numParametrosSeguridad; ++i)
    {
      const RangoSeguridad& r = rangosSeguridad[i];
      const float v = valores[i];
      const bool sobreMinimo = r.minimoIncluido ? v >= r.minimo : v > r.minimo;
      const bool bajoMaximo = r.maximoIncluido ? v <= r.maximo : v < r.maximo;
      if(!sobreMinimo || !bajoMaximo)
      {
        error = std::string(r.nombre) + " = " + std::to_string(v) + " fuera de " + (r.minimoIncluido ? "[" : "(") +
                std::to_string(r.minimo) + ", " + std::to_string(r.maximo) + (r.maximoIncluido ? "]" : ")");
        return false;
      }
    }
    if(ignorarLevantado && !(valores[pVxCmd] <= maxVxConIgnorarLevantado))
    {
      error = "ignorarLevantado solo se acepta con vxCmd <= " + std::to_string(maxVxConIgnorarLevantado);
      return false;
    }
    return true;
  }

  // -------------------------------------------------------------------------------------------------------------------
  // H5. Banderas de prueba.

  /** H5(a): hay que avisar en voz alta y en la cabecera del registro. */
  constexpr bool modoPrueba(const bool ignorarCamara, const bool ignorarLevantado)
  {
    return ignorarCamara || ignorarLevantado;
  }

  /** H5(c): el paro por inclinacion no depende de ninguna bandera. Un NaN (IMU rota) tambien es paro. */
  inline bool inclinacionExcedida(const float inclinacion, const float maxInclinacion)
  {
    return !(inclinacion < maxInclinacion);
  }

  // -------------------------------------------------------------------------------------------------------------------
  // H6. Guarda final, justo antes de escribir JointRequest. Es un segundo cerrojo: no se fia de limitarObjetivos.

  /**
   * Comprueba lo que se va a escribir: angulo finito, distinto de SensorData::off y dentro de [minimos, maximos], rigidez en
   * [0, 100]. Con un solo fallo TODAS las articulaciones escritas pasan a su angulo medido con rigidez stiffnessParo (es
   * un paro duro, no tiene sentido soltar solo una).
   * @return El indice de la primera articulacion mala, o -1 si todo esta bien (entonces no cambia nada).
   */
  template<std::size_t N>
  int guardaFinal(std::array<float, N>& angulos, std::array<int, N>& rigideces, const std::array<bool, N>& escribe,
                  const std::array<float, N>& medidos, const std::array<float, N>& minimos, const std::array<float, N>& maximos,
                  const int stiffnessParo)
  {
    int mala = -1;
    for(std::size_t j = 0; j < N && mala < 0; ++j)
      if(escribe[j] && !(std::isfinite(angulos[j]) && angulos[j] != sensorApagado && angulos[j] >= minimos[j] &&
                         angulos[j] <= maximos[j] && rigideces[j] >= 0 && rigideces[j] <= 100))
        mala = static_cast<int>(j);
    if(mala >= 0)
      for(std::size_t j = 0; j < N; ++j)
        if(escribe[j])
        {
          angulos[j] = medidos[j];
          rigideces[j] = stiffnessParo;
        }
    return mala;
  }

  /**
   * Objetivo del primer ciclo de una fase. MotionEngine llama a calcJoints de la fase nueva SIN update() previo, asi que lo
   * que se escribe es el angulo medido. B-Human mantiene algunas articulaciones justo en el limite (los codos en stand,
   * lElbowRoll = -2 grados), y con el ruido del sensor el medido queda fuera por milesimas: sin esto, H6 soltaria a un robot de
   * pie. Fuera por poco (holguraAngulos, como H2) se recorta al limite; mas lejos o no finito no se toca, y H6 lo para.
   */
  inline float objetivoInicial(const float medido, const float minimo, const float maximo)
  {
    return medido >= minimo - holguraAngulos && medido <= maximo + holguraAngulos ? std::clamp(medido, minimo, maximo) : medido;
  }

  // -------------------------------------------------------------------------------------------------------------------
  // H7. Autoverificacion en home, antes de caminar.

  constexpr float maxErrorHome = .1f;        /**< rad: detecta unidades, offsets y articulaciones bloqueadas. */
  constexpr float maxInclinacionHome = .1f;  /**< rad. */
  constexpr float maxGyroHome = .3f;         /**< rad/s por eje. */
  constexpr int maxTemperaturaHome = 50;     /**< C (contrato, seccion 7: temperaturas < 50 antes de cada prueba). */

  /**
   * @param corrienteMedia Media de |corriente| de las piernas en home [mA]; < 0 = sin sensor (simulacion).
   * @param exigirCorriente En el robot real no tener lectura de corriente tambien es un fallo.
   * @return "" si se puede caminar; si no, la explicacion (para el registro).
   */
  inline std::string autoverificacionHome(const std::array<float, nPiernas>& angulos, const std::array<float, nPiernas>& home,
                                          const std::array<int, nPiernas>& temperaturas, const float inclinacion,
                                          const std::array<float, 3>& gyro, const float corrienteMedia,
                                          const bool exigirCorriente, const float maxCorrienteHome)
  {
    for(std::size_t k = 0; k < nPiernas; ++k)
    {
      if(!(std::abs(angulos[k] - home[k]) < maxErrorHome))
        return "pierna " + std::to_string(k) + " a " + std::to_string(angulos[k] - home[k]) + " rad de home";
      if(!(temperaturas[k] < maxTemperaturaHome))
        return "pierna " + std::to_string(k) + " a " + std::to_string(temperaturas[k]) + " C";
    }
    if(!(inclinacion < maxInclinacionHome))
      return "torso inclinado " + std::to_string(inclinacion) + " rad";
    for(const float g : gyro)
      if(!(std::abs(g) < maxGyroHome))
        return "giroscopio no quieto: " + std::to_string(g) + " rad/s";
    if(corrienteMedia < 0.f)
    {
      if(exigirCorriente)
        return "sin lectura de corriente";
    }
    else if(!(corrienteMedia <= maxCorrienteHome))
      return "corriente media en home " + std::to_string(corrienteMedia) + " mA";
    return "";
  }
}
