/**
 * @file GuardasNaoV08.cpp
 *
 * Pruebas de las guardas H1-H7 (demo_carril/docs/despliegue_nao_v08.md, seccion 6c): cada una inyecta la entrada mala que
 * describe el contrato y comprueba que no sale orden de movimiento. Ejercitan las funciones de Tools/NaoRL que usa el modulo
 * NaoRLWalkEngine (Tests no enlaza los modulos de B-Human, asi que el cableado dentro de NaoRLWalkPhase queda para E1/E2).
 *
 * Correr desde la raiz del repo:
 *   Build/Linux/Tests/Develop/Tests --gtest_filter='NaoRL.*'
 */

#include "Platform/File.h"
#include "Streaming/InStreams.h"
#include "../../../Tools/NaoRL/NaoRLGuardas.h"
#include "../../../Tools/NaoRL/NaoRLPoliticaConfig.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

using namespace NaoRL;

namespace
{
  const float nan_ = std::numeric_limits<float>::quiet_NaN();
  const float inf_ = std::numeric_limits<float>::infinity();

  PoliticaCargada& politica()
  {
    static PoliticaCargada p;
    static bool ok = false;
    if(!ok)
    {
      std::string error;
      ok = cargarPolitica("NeuralNets/NaoRL/nao_v08.cfg", p, error);
      EXPECT_TRUE(ok) << error;
    }
    return p;
  }

  /** De pie y derecho, en home, con vision (psi, d). */
  EntradaPaso entradaNormal(const float t = 0.f, const float psi = 0.f, const float d = 0.f)
  {
    EntradaPaso e;
    e.arriba = {0.f, 0.f, 1.f};
    e.angulos = politica().postura;
    e.vxCmd = .5f;
    e.tMarcha = t;
    e.psi = psi;
    e.d = d;
    return e;
  }

  void limitesAmplios(Accion& minimos, Accion& maximos)
  {
    minimos.fill(-3.f);
    maximos.fill(3.f);
  }

  std::string leerTexto(const std::string& ruta)
  {
    std::ifstream f(ruta, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
  }

  std::string rutaCfg() { return std::string(File::getBHDir()) + "/Config/NeuralNets/NaoRL/nao_v08.cfg"; }

  /** Escribe el texto como nao_v08.cfg en un directorio temporal y lo intenta cargar. */
  bool cargarTexto(const std::string& texto, std::string& error, const std::string& nombre = "nao_v08.cfg")
  {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "naorl_h3";
    std::filesystem::create_directories(dir);
    const std::string ruta = (dir / nombre).string();
    {
      std::ofstream f(ruta, std::ios::binary | std::ios::trunc);
      f << texto;
    }
    PoliticaCargada p;
    const bool ok = cargarPolitica(ruta, p, error);
    std::filesystem::remove(ruta);
    std::filesystem::remove(dir);
    EXPECT_EQ(ok, p.politica.valida()) << "una politica rechazada no debe quedar valida";
    return ok;
  }

  /** Altera UN byte: el primer digito distinto de cero despues de la clave. */
  std::string alterarByteTras(std::string texto, const std::string& clave, const std::size_t desde = 0)
  {
    std::size_t i = texto.find(clave, desde);
    EXPECT_NE(i, std::string::npos) << clave;
    i += clave.size();
    while(i < texto.size() && !(texto[i] >= '1' && texto[i] <= '9'))
      ++i;
    EXPECT_LT(i, texto.size());
    texto[i] = texto[i] == '9' ? '1' : static_cast<char>(texto[i] + 1);
    return texto;
  }
}

/** Los umbrales de naoRLWalkEngine.cfg de CarrilRL que entran en la tabla de H4 (y las banderas). */
STREAMABLE(NaoRLSeguridadPrueba,
{,
  (bool) usarPoliticaRL,
  (bool) ignorarCamara,
  (bool) ignorarLevantado,
  (float) maxInclinacion,
  (int) maxTemperatura,
  (float) umbralSeguimiento,
  (float) umbralSeguimientoRoll,
  (int) ciclosSeguimiento,
  (int) umbralCorriente,
  (int) ciclosCorriente,
  (float) margenLimites,
  (float) factorVelRoll,
  (int) stiffnessParo,
  (int) presupuestoInferenciaUs,
  (float) vxCmd,
  (int) retardoSeguimiento,
  (int) maxFallosNaN,
  (int) maxExcesosInferencia,
  (int) maxEdadCamaraMs,
  (int) maxMarchaMs,
  (int) maxCorrienteHome,
});

namespace
{
  NaoRLSeguridadPrueba cfgEscenario()
  {
    NaoRLSeguridadPrueba c;
    InMapFile stream("Scenarios/CarrilRL/naoRLWalkEngine.cfg");
    EXPECT_TRUE(stream.exists());
    stream >> c;
    return c;
  }

  ValoresSeguridad valoresEscenario()
  {
    const NaoRLSeguridadPrueba c = cfgEscenario();
    ValoresSeguridad v{};
    v[pMaxInclinacion] = c.maxInclinacion;
    v[pMaxTemperatura] = static_cast<float>(c.maxTemperatura);
    v[pUmbralSeguimiento] = c.umbralSeguimiento;
    v[pUmbralSeguimientoRoll] = c.umbralSeguimientoRoll;
    v[pCiclosSeguimiento] = static_cast<float>(c.ciclosSeguimiento);
    v[pUmbralCorriente] = static_cast<float>(c.umbralCorriente);
    v[pCiclosCorriente] = static_cast<float>(c.ciclosCorriente);
    v[pMargenLimites] = c.margenLimites;
    v[pFactorVelRoll] = c.factorVelRoll;
    v[pStiffnessParo] = static_cast<float>(c.stiffnessParo);
    v[pPresupuestoInferenciaUs] = static_cast<float>(c.presupuestoInferenciaUs);
    v[pVxCmd] = c.vxCmd;
    v[pRetardoSeguimiento] = static_cast<float>(c.retardoSeguimiento);
    v[pMaxFallosNaN] = static_cast<float>(c.maxFallosNaN);
    v[pMaxExcesosInferencia] = static_cast<float>(c.maxExcesosInferencia);
    v[pMaxEdadCamaraMs] = static_cast<float>(c.maxEdadCamaraMs);
    v[pMaxMarchaMs] = static_cast<float>(c.maxMarchaMs);
    v[pMaxCorrienteHome] = static_cast<float>(c.maxCorrienteHome);
    return v;
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// H1: NaN en psi

GTEST_TEST(NaoRL, H1_CamaraMalaNoLlegaALaRedNiEsFalloDeRed)
{
  const float signoPsi = -1.f, signoD = 1.f;
  float psi = .1f, d = .05f;
  const struct { float psi, dMm; } malas[] = {
    {nan_, 0.f}, {0.f, nan_}, {inf_, 0.f}, {0.f, -inf_}, {4.f, 0.f}, {-4.f, 0.f}, {0.f, 2500.f}, {0.f, -2500.f}, {0.f, sensorApagado},
  };
  for(const auto& m : malas)
  {
    EXPECT_FALSE(actualizarVision(m.psi, m.dMm, signoPsi, signoD, psi, d)) << m.psi << " " << m.dMm;
    EXPECT_EQ(psi, .1f); // se mantiene el ultimo valor valido
    EXPECT_EQ(d, .05f);
  }

  // Dos carreras identicas, una con camara NaN a ratos: la red ve lo mismo, no hay fallos ni paro y la accion es identica.
  Politica& red = politica().politica;
  Decisor limpio, conNaN;
  limpio.reiniciar();
  conNaN.reiniciar();
  float psiNaN = .1f, dNaN = .05f;
  for(int k = 0; k < 20; ++k)
  {
    if(k % 2)
      EXPECT_FALSE(actualizarVision(nan_, nan_, signoPsi, signoD, psiNaN, dNaN));
    const float t = .024f * static_cast<float>(k);
    const Decisor::Resultado a = limpio.decidir(red, entradaNormal(t, .1f, .05f), politica().postura, nullptr, 3);
    const Decisor::Resultado b = conNaN.decidir(red, entradaNormal(t, psiNaN, dNaN), politica().postura, nullptr, 3);
    EXPECT_TRUE(b.redEvaluada);
    EXPECT_FALSE(b.paro);
    EXPECT_EQ(conNaN.fallosSeguidos(), 0);
    EXPECT_EQ(a.accion, b.accion);
  }

  // Una distanciaMeta rara no puede activar "meta cerca" (desactiva el paro por camara).
  EXPECT_FALSE(distanciaMetaValida(nan_));
  EXPECT_FALSE(distanciaMetaValida(-1.f));
  EXPECT_FALSE(distanciaMetaValida(-inf_));
  EXPECT_TRUE(distanciaMetaValida(1200.f));
}

GTEST_TEST(NaoRL, H1_SignosYUnidadesDeLaVisionNueva)
{
  // CarrilPercept: d en mm, > 0 = robot a la izquierda; psi > 0 = robot girado a la izquierda. Politica: d en m, mismo signo;
  // psi = rumbo deseado - rumbo real, o sea el contrario.
  float psi = 0.f, d = 0.f;
  ASSERT_TRUE(actualizarVision(.2f, 100.f, -1.f, 1.f, psi, d));
  EXPECT_FLOAT_EQ(psi, -.2f);
  EXPECT_FLOAT_EQ(d, .1f);
}

// ---------------------------------------------------------------------------------------------------------------------
// H2: gyro 1e6

GTEST_TEST(NaoRL, H2_GyroAbsurdoNoEvaluaLaRedYAcabaEnParo)
{
  Accion minimos, maximos;
  limitesAmplios(minimos, maximos);
  Politica& red = politica().politica;
  const Accion& postura = politica().postura;

  EntradaPaso mala = entradaNormal();
  mala.gyro = {1e6f, 0.f, 0.f};
  const char* motivo = motivoNoPlausible(mala.gyro, mala.arriba, mala.angulos, minimos, maximos);
  ASSERT_NE(motivo, nullptr);

  Decisor d;
  d.reiniciar();
  const Decisor::Resultado buena = d.decidir(red, entradaNormal(), postura, nullptr, 3);
  ASSERT_TRUE(buena.redEvaluada);
  ASSERT_FALSE(buena.paro);

  // Se repite la ultima accion valida (ninguna orden nueva sale del dato malo); a la tercera, paro.
  const Decisor::Resultado r1 = d.decidir(red, mala, postura, motivo, 3);
  EXPECT_FALSE(r1.redEvaluada);
  EXPECT_EQ(r1.accion, buena.accion);
  EXPECT_FALSE(r1.paro);
  const Decisor::Resultado r2 = d.decidir(red, mala, postura, motivo, 3);
  EXPECT_FALSE(r2.redEvaluada);
  EXPECT_EQ(r2.accion, buena.accion);
  EXPECT_FALSE(r2.paro);
  const Decisor::Resultado r3 = d.decidir(red, mala, postura, motivo, 3);
  EXPECT_FALSE(r3.redEvaluada);
  EXPECT_EQ(r3.accion, buena.accion);
  EXPECT_TRUE(r3.paro);
}

GTEST_TEST(NaoRL, H2_UnDatoMaloNoEnvenenaLaHistoria)
{
  Politica& red = politica().politica;
  const Accion& postura = politica().postura;
  Decisor limpio, conFallo;
  limpio.reiniciar();
  conFallo.reiniciar();
  EntradaPaso mala = entradaNormal();
  mala.gyro = {nan_, 0.f, 0.f};

  const Decisor::Resultado a0 = limpio.decidir(red, entradaNormal(0.f), postura, nullptr, 3);
  const Decisor::Resultado b0 = conFallo.decidir(red, entradaNormal(0.f), postura, nullptr, 3);
  ASSERT_EQ(a0.accion, b0.accion);
  EXPECT_FALSE(conFallo.decidir(red, mala, postura, nullptr, 3).redEvaluada); // NaN sin motivo previo: lo atrapa el bloque
  EXPECT_EQ(conFallo.fallosSeguidos(), 1);

  // Tras el dato malo, la siguiente decision buena cuenta como buena y no arrastra el NaN (antes llegaba a paro con 1 NaN).
  const Decisor::Resultado a1 = limpio.decidir(red, entradaNormal(.024f), postura, nullptr, 3);
  const Decisor::Resultado b1 = conFallo.decidir(red, entradaNormal(.024f), postura, nullptr, 3);
  EXPECT_TRUE(b1.redEvaluada);
  EXPECT_EQ(conFallo.fallosSeguidos(), 0);
  EXPECT_EQ(a1.accion, b1.accion);
}

GTEST_TEST(NaoRL, H2_PlausibilidadPorCanal)
{
  Accion minimos, maximos;
  limitesAmplios(minimos, maximos);
  const EntradaPaso ok = entradaNormal();
  EXPECT_EQ(motivoNoPlausible(ok.gyro, ok.arriba, ok.angulos, minimos, maximos), nullptr);

  const auto mal = [&](std::array<float, 3> gyro, std::array<float, 3> arriba, Accion angulos)
  {
    return motivoNoPlausible(gyro, arriba, angulos, minimos, maximos) != nullptr;
  };
  EXPECT_TRUE(mal({sensorApagado, 0.f, 0.f}, ok.arriba, ok.angulos));   // SensorData::off en un canal no articular
  EXPECT_TRUE(mal({0.f, 0.f, -20.f}, ok.arriba, ok.angulos));
  EXPECT_TRUE(mal({0.f, inf_, 0.f}, ok.arriba, ok.angulos));
  EXPECT_FALSE(mal({0.f, 0.f, 19.f}, ok.arriba, ok.angulos));
  EXPECT_TRUE(mal(ok.gyro, {0.f, 0.f, 1.2f}, ok.angulos));              // componente
  EXPECT_TRUE(mal(ok.gyro, {0.f, 0.f, .5f}, ok.angulos));               // norma
  EXPECT_TRUE(mal(ok.gyro, {nan_, 0.f, 1.f}, ok.angulos));
  Accion angulos = ok.angulos;
  angulos[mLHipRoll] = 3.21f;                                           // jointLimits + 0.2 < valor (grados, off...)
  EXPECT_TRUE(mal(ok.gyro, ok.arriba, angulos));
  angulos[mLHipRoll] = nan_;
  EXPECT_TRUE(mal(ok.gyro, ok.arriba, angulos));
  angulos[mLHipRoll] = 3.1f;                                            // dentro de la holgura
  EXPECT_FALSE(mal(ok.gyro, ok.arriba, angulos));
}

GTEST_TEST(NaoRL, H2_EntradaNormalizadaRecortada)
{
  // Una observacion absurda pero finita da una accion finita, igual a la de cualquier otra saturada al mismo lado.
  Politica& red = politica().politica;
  Observacion a, b;
  a.fill(1e6f);
  b.fill(1e30f);
  const Accion ra = red.evaluar(a), rb = red.evaluar(b);
  EXPECT_TRUE(finita(ra));
  EXPECT_EQ(ra, rb);
}

// ---------------------------------------------------------------------------------------------------------------------
// H3: archivo de pesos alterado en un byte

GTEST_TEST(NaoRL, H3_ElCfgRealPasaHashYAutotest)
{
  std::string error;
  EXPECT_TRUE(cargarTexto(leerTexto(rutaCfg()), error)) << error;
}

GTEST_TEST(NaoRL, H3_UnByteAlteradoDejaLaPoliticaNoCargada)
{
  const std::string texto = leerTexto(rutaCfg());
  ASSERT_FALSE(texto.empty());
  const std::size_t salida = texto.find("salida = ");
  const std::size_t segundaOculta = texto.find("filas", texto.find("sesgo"));
  const struct { const char* clave; std::size_t desde; } casos[] = {
    {"media = ", 0}, {"desviacion = ", 0}, {"pesos = ", 0}, {"sesgo = ", 0}, {"pesos = ", segundaOculta},
    {"pesos = ", salida}, {"sesgo = ", salida}, {"postura = ", 0}, {"actionScale = ", 0}, {"ctrlDt = ", 0},
    {"hashPesos = ", 0}, {"autotestObs = ", 0}, {"autotestAccion = ", 0},
  };
  for(const auto& c : casos)
  {
    std::string error;
    EXPECT_FALSE(cargarTexto(alterarByteTras(texto, c.clave, c.desde), error)) << c.clave;
    std::cout << "[H3] " << c.clave << "-> " << error << std::endl;
  }
}

GTEST_TEST(NaoRL, H3_TruncadoOSintaxisRotaNoAbortaYNoCarga)
{
  // Con Develop en el robot, SimpleMap hace FAIL -> abort con un error de sintaxis: tiene que atajarse antes.
  const std::string texto = leerTexto(rutaCfg());
  std::string error;
  EXPECT_FALSE(cargarTexto(texto.substr(0, 40), error)); // corte dentro del comentario: sintaxis valida, pero sin campos
  for(const double f : {.1, .5, .9, .9999})
  {
    EXPECT_FALSE(cargarTexto(texto.substr(0, static_cast<std::size_t>(static_cast<double>(texto.size()) * f)), error)) << f;
    EXPECT_NE(error.find("sintaxis"), std::string::npos) << error;
  }
  EXPECT_FALSE(cargarTexto("", error));

  // Sintaxis correcta, pero un numero que no es numero en un peso: tampoco carga.
  std::string letra = texto;
  std::size_t i = letra.find("pesos = ");
  while(!(letra[i] >= '1' && letra[i] <= '9'))
    ++i;
  letra[i] = 'x';
  EXPECT_FALSE(cargarTexto(letra, error));

  for(const char* buscado : {";", "=", "[", "]", ",", "{", "}"})
  {
    std::string roto = texto;
    const std::size_t i = roto.find(buscado, roto.find("media"));
    ASSERT_NE(i, std::string::npos);
    roto[i] = '?';
    EXPECT_FALSE(cargarTexto(roto, error)) << buscado;
    EXPECT_NE(error.find("sintaxis"), std::string::npos) << error;
  }
}

GTEST_TEST(NaoRL, H3_OtraGeneracionOSinAutotestNoCarga)
{
  const std::string texto = leerTexto(rutaCfg());
  std::string error;
  // Coherente consigo mismo (el hash pasa), pero no es la generacion pedida.
  EXPECT_FALSE(cargarTexto(texto, error, "nao_v09.cfg"));
  EXPECT_NE(error.find("generacion"), std::string::npos) << error;

  // Un .cfg viejo, sin hash ni autotest (el que habia antes de 6c).
  EXPECT_FALSE(cargarTexto(texto.substr(0, texto.find("hashPesos")), error));
  EXPECT_NE(error.find("hashPesos"), std::string::npos) << error;

  // Con el hash, pero sin casos de autotest: no se puede saltar el autotest.
  std::string sinCasos = texto;
  sinCasos.replace(sinCasos.find("autotestCasos = 2"), 17, "autotestCasos = 0");
  EXPECT_FALSE(cargarTexto(sinCasos, error));
  EXPECT_NE(error.find("autotest"), std::string::npos) << error;
}

// ---------------------------------------------------------------------------------------------------------------------
// H4: umbral 0

GTEST_TEST(NaoRL, H4_ElCfgDelEscenarioEstaEnRango)
{
  std::string error;
  EXPECT_TRUE(validarSeguridad(valoresEscenario(), false, error)) << error;
}

GTEST_TEST(NaoRL, H4_UmbralFueraDeRangoNoArma)
{
  const ValoresSeguridad buenos = valoresEscenario();
  std::string error;
  // Cada parametro: 0 si no es un valor permitido, justo fuera por los dos lados, enorme, negativo y NaN.
  for(std::size_t i = 0; i < numParametrosSeguridad; ++i)
  {
    const RangoSeguridad& r = rangosSeguridad[i];
    const float malos[] = {r.minimo - 1.f, r.maximo * 2.f + 1.f, 1e9f, -1e9f, nan_, inf_};
    for(const float v : malos)
    {
      ValoresSeguridad valores = buenos;
      valores[i] = v;
      EXPECT_FALSE(validarSeguridad(valores, false, error)) << r.nombre << " = " << v;
      EXPECT_NE(error.find(r.nombre), std::string::npos) << error;
    }
    ValoresSeguridad extremo = buenos;
    extremo[i] = r.minimo;
    EXPECT_EQ(validarSeguridad(extremo, false, error), r.minimoIncluido) << r.nombre << " en el minimo";
    extremo[i] = r.maximo;
    EXPECT_TRUE(validarSeguridad(extremo, false, error)) << r.nombre << " en el maximo: " << error;
  }
  // Los del contrato que apagan una proteccion.
  const std::size_t apagan[] = {pMaxInclinacion, pUmbralSeguimiento, pUmbralSeguimientoRoll, pCiclosSeguimiento, pUmbralCorriente, pCiclosCorriente,
                                pMargenLimites, pFactorVelRoll, pPresupuestoInferenciaUs, pMaxFallosNaN, pMaxEdadCamaraMs};
  for(const std::size_t i : apagan)
  {
    ValoresSeguridad valores = buenos;
    valores[i] = 0.f;
    EXPECT_FALSE(validarSeguridad(valores, false, error)) << rangosSeguridad[i].nombre << " = 0";
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// H5: bandera de prueba en true

GTEST_TEST(NaoRL, H5_IgnorarLevantadoAVelocidadDeCarreraNoArma)
{
  ValoresSeguridad valores = valoresEscenario();
  std::string error;
  valores[pVxCmd] = .5f;
  EXPECT_FALSE(validarSeguridad(valores, true, error));
  EXPECT_NE(error.find("ignorarLevantado"), std::string::npos) << error;
  valores[pVxCmd] = .21f;
  EXPECT_FALSE(validarSeguridad(valores, true, error));
  valores[pVxCmd] = .2f;
  EXPECT_TRUE(validarSeguridad(valores, true, error)) << error;
  EXPECT_TRUE(modoPrueba(false, true));
  EXPECT_TRUE(modoPrueba(true, false));
  EXPECT_FALSE(modoPrueba(false, false));
}

GTEST_TEST(NaoRL, H5_LaInclinacionNoSeIgnoraNunca)
{
  // inclinacionExcedida no recibe ninguna bandera a proposito.
  EXPECT_TRUE(inclinacionExcedida(.5f, .4f));
  EXPECT_TRUE(inclinacionExcedida(.4f, .4f));
  EXPECT_TRUE(inclinacionExcedida(nan_, .4f));
  EXPECT_FALSE(inclinacionExcedida(.39f, .4f));
}

GTEST_TEST(NaoRL, H5_ElEscenarioNoTraeBanderasOlvidadas)
{
  const NaoRLSeguridadPrueba c = cfgEscenario();
  EXPECT_FALSE(c.usarPoliticaRL);
  EXPECT_FALSE(c.ignorarCamara);
  EXPECT_FALSE(c.ignorarLevantado);
}

// ---------------------------------------------------------------------------------------------------------------------
// H6: objetivo fuera de rango forzado dentro de la fase

GTEST_TEST(NaoRL, H6_GuardaFinal)
{
  constexpr std::size_t n = 6;
  constexpr int stiffnessParo = 0;
  constexpr std::array<bool, n> escribe = {false, false, true, true, true, true}; // las dos primeras: la cabeza, no se toca
  constexpr std::array<float, n> medidos = {.9f, .9f, .11f, .12f, .13f, .14f};
  std::array<float, n> minimos, maximos;
  minimos.fill(-1.f);
  maximos.fill(1.f);
  const std::array<float, n> buenos = {7.f, 7.f, .5f, -.5f, 1.f, -1.f};
  const std::array<int, n> rigidecesBuenas = {50, 50, 100, 100, 0, 100};

  std::array<float, n> angulos = buenos;
  std::array<int, n> rigideces = rigidecesBuenas;
  EXPECT_EQ(guardaFinal(angulos, rigideces, escribe, medidos, minimos, maximos, stiffnessParo), -1);
  EXPECT_EQ(angulos, buenos);
  EXPECT_EQ(rigideces, rigidecesBuenas);

  const auto comprobarSoltado = [&](const char* caso, const int esperada)
  {
    EXPECT_EQ(guardaFinal(angulos, rigideces, escribe, medidos, minimos, maximos, stiffnessParo), esperada) << caso;
    for(std::size_t j = 0; j < n; ++j)
      if(escribe[j])
      {
        EXPECT_EQ(angulos[j], medidos[j]) << caso; // nada de la orden original llega a JointRequest
        EXPECT_EQ(rigideces[j], stiffnessParo) << caso;
      }
      else
      {
        EXPECT_EQ(angulos[j], buenos[j]) << caso;
        EXPECT_EQ(rigideces[j], rigidecesBuenas[j]) << caso;
      }
  };

  for(const float malo : {1.01f, -1.01f, 1e6f, nan_, inf_, -inf_, sensorApagado})
  {
    angulos = buenos;
    rigideces = rigidecesBuenas;
    angulos[3] = malo;
    comprobarSoltado("angulo", 3);
  }
  for(const int mala : {101, -1, 10000})
  {
    angulos = buenos;
    rigideces = rigidecesBuenas;
    rigideces[4] = mala;
    comprobarSoltado("rigidez", 4);
  }
  EXPECT_EQ(sensorApagado, 10000.f);
}

GTEST_TEST(NaoRL, H6_RobotDePieConElCodoEnElLimiteNoSeSuelta)
{
  // Caso real (SimRobot): stand de B-Human con lElbowRoll en su limite de -2 grados y el medido fuera por 0.00005 rad; el primer
  // calcJoints de la fase llega sin update() y H6 hacia un paro duro con el robot de pie.
  constexpr float minimo = -1.544616f, maximo = -.034907f, medido = -.034855f;
  std::array<float, 1> angulos = {objetivoInicial(medido, minimo, maximo)};
  std::array<int, 1> rigideces = {100};
  EXPECT_EQ(angulos[0], maximo);
  EXPECT_EQ(guardaFinal(angulos, rigideces, {true}, {medido}, {minimo}, {maximo}, 0), -1);
  EXPECT_EQ(rigideces[0], 100);

  // Dentro del rango no cambia nada.
  EXPECT_EQ(objetivoInicial(-.5f, minimo, maximo), -.5f);

  // Lejos del rango, o no finito, sigue siendo un dato malo: no se recorta y H6 lo para.
  for(const float malo : {maximo + .21f, minimo - .21f, 3.f, sensorApagado, nan_, inf_})
  {
    std::array<float, 1> a = {objetivoInicial(malo, minimo, maximo)};
    std::array<int, 1> r = {100};
    EXPECT_EQ(guardaFinal(a, r, {true}, {0.f}, {minimo}, {maximo}, 0), 0) << malo;
    EXPECT_EQ(r[0], 0);
  }

  // Y la fase lo usa.
  const std::string fase = leerTexto(std::string(File::getBHDir()) + "/Src/Modules/MotionControl/NaoRLWalkEngine/NaoRLWalkEngine.cpp");
  EXPECT_NE(fase.find("objetivos[j] = NaoRL::objetivoInicial(anguloInicial[j]"), std::string::npos);
}

// ---------------------------------------------------------------------------------------------------------------------
// H7: home desplazado 0.3 rad

namespace
{
  struct EstadoHome
  {
    std::array<float, nPiernas> angulos{}, home{};
    std::array<int, nPiernas> temperaturas{};
    float inclinacion = .01f;
    std::array<float, 3> gyro{.01f, -.01f, 0.f};
    float corriente = 300.f;
    bool exigirCorriente = true;

    EstadoHome()
    {
      temperaturas.fill(35);
      for(std::size_t k = 0; k < nPiernas; ++k)
        angulos[k] = home[k] = .1f * static_cast<float>(k);
    }

    std::string revisar() const
    {
      return autoverificacionHome(angulos, home, temperaturas, inclinacion, gyro, corriente, exigirCorriente, 800.f);
    }
  };
}

GTEST_TEST(NaoRL, H7_AutoverificacionPreviaImpideCaminar)
{
  EXPECT_EQ(EstadoHome().revisar(), "");

  const auto noCamina = [](EstadoHome e, const char* caso)
  {
    const std::string fallo = e.revisar();
    EXPECT_FALSE(fallo.empty()) << caso;
    std::cout << "[H7] " << caso << " -> " << fallo << std::endl;
  };

  EstadoHome e;
  e.angulos[3] += .3f; // home desplazado 0.3 rad (unidades, offset, articulacion pegada)
  noCamina(e, "home desplazado");
  e = EstadoHome();
  e.angulos[11] -= .11f;
  noCamina(e, "ultima pierna a 0.11");
  e = EstadoHome();
  e.angulos[0] = sensorApagado;
  noCamina(e, "sensor apagado");
  e = EstadoHome();
  e.angulos[0] = nan_;
  noCamina(e, "angulo NaN");
  e = EstadoHome();
  e.inclinacion = .12f;
  noCamina(e, "torso inclinado");
  e = EstadoHome();
  e.inclinacion = nan_;
  noCamina(e, "inclinacion NaN");
  e = EstadoHome();
  e.gyro[1] = .31f;
  noCamina(e, "gyro no quieto");
  e = EstadoHome();
  e.temperaturas[5] = 50;
  noCamina(e, "motor caliente");
  e = EstadoHome();
  e.corriente = 801.f;
  noCamina(e, "corriente alta");
  e = EstadoHome();
  e.corriente = nan_;
  noCamina(e, "corriente NaN");
  e = EstadoHome();
  e.corriente = -1.f;
  noCamina(e, "sin lectura de corriente en el robot");

  e = EstadoHome();
  e.corriente = -1.f;
  e.exigirCorriente = false; // simulacion: no hay sensor de corriente
  EXPECT_EQ(e.revisar(), "");
}

// ---------------------------------------------------------------------------------------------------------------------
// Fusion con la vision nueva: CarrilRL no se puede quedar atras de Carril en lo que no es la marcha.

GTEST_TEST(NaoRL, EscenarioCarrilRLUsaLaVisionDeCarril)
{
  const std::string base = std::string(File::getBHDir()) + "/Config/Scenarios/";
  for(const char* archivo : {"carrilPerceptProvider.cfg", "carrilEvaluador.cfg", "audioProvider.cfg", "whistleRecognizer.cfg"})
  {
    const std::string carril = leerTexto(base + "Carril/" + archivo);
    EXPECT_FALSE(carril.empty()) << archivo;
    EXPECT_EQ(carril, leerTexto(base + "CarrilRL/" + archivo)) << archivo;
  }

  // carrilBehavior.cfg: la unica diferencia permitida es el silbato, apagado para las pruebas E2-E4.
  std::istringstream carril(leerTexto(base + "Carril/carrilBehavior.cfg")), carrilRL(leerTexto(base + "CarrilRL/carrilBehavior.cfg"));
  int silbato = 0;
  std::string a, b;
  while(std::getline(carril, a))
  {
    ASSERT_TRUE(static_cast<bool>(std::getline(carrilRL, b)));
    if(a.rfind("arranquePorSilbato", 0) == 0)
    {
      ++silbato;
      EXPECT_EQ(a.rfind("arranquePorSilbato = true;", 0), 0u);
      EXPECT_EQ(b.rfind("arranquePorSilbato = false;", 0), 0u) << b;
      EXPECT_NE(b.find("SOLO PRUEBAS"), std::string::npos) << b;
    }
    else
      EXPECT_EQ(a, b);
  }
  EXPECT_FALSE(static_cast<bool>(std::getline(carrilRL, b))) << "CarrilRL tiene lineas de mas";
  EXPECT_EQ(silbato, 1);
}
