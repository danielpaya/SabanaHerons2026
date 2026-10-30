/**
 * @file SombraNaoV08.cpp
 *
 * Pruebas del modo sombra: que NaoRLSombra no pueda escribir a los motores, que su observacion sea la de la fase, el escenario
 * CarrilSombra y el formato del CSV. Tests no enlaza los modulos de B-Human: lo estructural se comprueba sobre el codigo
 * fuente, y lo numerico sobre las funciones compartidas de Tools/NaoRL que llaman los dos modulos.
 *
 * Correr desde la raiz del repo:
 *   Build/Linux/Tests/Develop/Tests --gtest_filter='NaoRL.*'
 */

#include "Platform/File.h"
#include "../../../Tools/NaoRL/NaoRLGuardas.h"
#include "../../../Tools/NaoRL/NaoRLObservacion.h"
#include "../../../Tools/NaoRL/NaoRLPoliticaConfig.h"
#include "../../../Tools/NaoRL/NaoRLRegistro.h"
#include "../../../Tools/NaoRL/NaoRLSombraFormato.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <random>
#include <regex>
#include <sstream>

using namespace NaoRL;

namespace
{
  std::string raiz() { return std::string(File::getBHDir()) + "/"; }

  std::string leer(const std::string& rutaRelativa)
  {
    std::ifstream f(raiz() + rutaRelativa, std::ios::binary);
    EXPECT_TRUE(f.is_open()) << rutaRelativa;
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
  }

  std::vector<std::string> lineas(const std::string& texto)
  {
    std::vector<std::string> v;
    std::istringstream is(texto);
    for(std::string l; std::getline(is, l);)
      v.push_back(l);
    return v;
  }

  std::vector<std::string> columnas(const std::string& linea)
  {
    std::vector<std::string> v;
    std::istringstream is(linea);
    for(std::string c; std::getline(is, c, ',');)
      v.push_back(c);
    return v;
  }

  /** Quita comentarios // y de bloque: un texto en un comentario no cuenta como codigo. */
  std::string sinComentarios(const std::string& s)
  {
    std::string r;
    for(std::size_t i = 0; i < s.size();)
      if(s.compare(i, 2, "/*") == 0)
      {
        const std::size_t fin = s.find("*/", i + 2);
        i = fin == std::string::npos ? s.size() : fin + 2;
      }
      else if(s.compare(i, 2, "//") == 0)
      {
        const std::size_t fin = s.find('\n', i + 2);
        i = fin == std::string::npos ? s.size() : fin;
      }
      else
        r += s[i++];
    return r;
  }

  std::vector<std::string> macros(const std::string& codigo, const char* nombre)
  {
    std::vector<std::string> v;
    const std::regex r(std::string("\\b") + nombre + R"(\s*\(\s*(\w+)\s*\))");
    for(std::sregex_iterator it(codigo.begin(), codigo.end(), r), fin; it != fin; ++it)
      v.push_back((*it)[1]);
    std::sort(v.begin(), v.end());
    return v;
  }

  const std::string dirSombra = "Src/Modules/MotionControl/NaoRLSombra/NaoRLSombra";
  const std::string dirFase = "Src/Modules/MotionControl/NaoRLWalkEngine/NaoRLWalkEngine";

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

  /** Entradas al azar pero siempre plausibles, con una semilla fija. */
  struct Azar
  {
    std::mt19937 g{1234u};
    float en(const float a, const float b) { return std::uniform_real_distribution<float>(a, b)(g); }
    bool moneda(const float p = .5f) { return en(0.f, 1.f) < p; }

    SensoresPaso sensores(const unsigned t)
    {
      SensoresPaso s;
      s.gyro = {en(-3.f, 3.f), en(-3.f, 3.f), en(-3.f, 3.f)};
      const float mx = en(-.2f, .2f), my = en(-.2f, .2f), mz = en(-.2f, .2f), mw = 1.f;
      const float n = std::sqrt(mx * mx + my * my + mz * mz + mw * mw);
      s.qw = mw / n;
      s.qx = mx / n;
      s.qy = my / n;
      s.qz = mz / n;
      for(std::size_t i = 0; i < nMotores; ++i)
        s.angulos[i] = politica().postura[i] + en(-.3f, .3f);
      s.psiCamara = moneda(.1f) ? std::numeric_limits<float>::quiet_NaN() : en(-1.f, 1.f);
      s.dCamaraMm = en(-800.f, 800.f);
      s.camaraValida = moneda(.7f);
      s.metaVisible = moneda(.2f);
      s.distanciaMetaMm = en(0.f, 3000.f);
      s.tiempoFrame = t;
      s.edadFrameMs = moneda(.1f) ? 500 : 20;
      return s;
    }
  };
}

// ---------------------------------------------------------------------------------------------------------------------
// Estructura del modulo

GTEST_TEST(NaoRL, SombraSoloProveeSuRepresentacion)
{
  const std::string h = sinComentarios(leer(dirSombra + ".h")), cpp = sinComentarios(leer(dirSombra + ".cpp"));
  const std::string todo = h + cpp;

  EXPECT_EQ(macros(todo, "PROVIDES"), std::vector<std::string>{"NaoRLSombraEstado"});
  EXPECT_EQ(macros(todo, "REQUIRES"), (std::vector<std::string>{"FallDownState", "FrameInfo", "InertialData", "JointLimits", "JointRequest",
                                                                "JointSensorData", "LowerCarrilPercept", "MotionInfo"}));

  // Ninguna otra forma de declarar o de saltarse el const.
  for(const char* prohibido : {"MODIFIES", "PROVIDES_WITHOUT_MODIFY", "PROVIDES_CONCURRENT", "USES", "const_cast", "reinterpret_cast",
                               "createPhase", "Generator", "public MotionPhase", "MotionRequest", "StandGenerator", "Blackboard"})
    EXPECT_EQ(todo.find(prohibido), std::string::npos) << prohibido;

  // Un solo update, el de su representacion; ninguno recibe JointRequest.
  const std::regex updates(R"(\bupdate\s*\(\s*(\w+)\s*&)");
  std::vector<std::string> tipos;
  for(std::sregex_iterator it(todo.begin(), todo.end(), updates), fin; it != fin; ++it)
    tipos.push_back((*it)[1]);
  EXPECT_EQ(tipos, (std::vector<std::string>{"NaoRLSombraEstado", "NaoRLSombraEstado"})); // declaracion y definicion
  EXPECT_FALSE(std::regex_search(todo, std::regex(R"(JointRequest\s*&)"))) << "nadie toma una referencia a JointRequest";
  EXPECT_FALSE(std::regex_search(todo, std::regex(R"(theJointRequest[^;]*[^=!<>]=[^=])"))) << "theJointRequest solo se lee";

  // La representacion no tiene angulos ni rigideces.
  const std::string estado = sinComentarios(leer("Src/Representations/MotionControl/NaoRLSombraEstado.h"));
  EXPECT_EQ(estado.find("Joint"), std::string::npos);

  // En el escenario, los motores siguen siendo de B-Human.
  const std::string threads = leer("Config/Scenarios/CarrilSombra/threads.cfg");
  EXPECT_NE(threads.find("{representation = JointRequest; provider = MotionEngine;}"), std::string::npos);
  EXPECT_NE(threads.find("{representation = WalkAtRelativeSpeedGenerator; provider = WalkAtSpeedEngine;}"), std::string::npos);
  EXPECT_NE(threads.find("{representation = MotionRequest; provider = CarrilBehavior;}"), std::string::npos);
  const std::regex delModulo(R"(provider\s*=\s*NaoRLSombra\s*;)");
  EXPECT_EQ(std::distance(std::sregex_iterator(threads.begin(), threads.end(), delModulo), std::sregex_iterator()), 1);
}

GTEST_TEST(NaoRL, SombraApagadaPorDefecto)
{
  const std::string h = sinComentarios(leer(dirSombra + ".h"));
  EXPECT_NE(h.find("(bool)(false) usarSombra"), std::string::npos);
  EXPECT_NE(h.find("(bool)(false) siempre"), std::string::npos);
  const std::string cpp = sinComentarios(leer(dirSombra + ".cpp"));
  const std::size_t constructor = cpp.find("NaoRLSombra::NaoRLSombra()");
  ASSERT_NE(constructor, std::string::npos);
  const std::size_t salida = cpp.find("if(!usarSombra)", constructor), carga = cpp.find("cargarPolitica", constructor);
  ASSERT_NE(salida, std::string::npos);
  EXPECT_LT(salida, carga) << "con usarSombra = false no se carga nada";
}

// ---------------------------------------------------------------------------------------------------------------------
// Misma observacion que la fase

GTEST_TEST(NaoRL, SombraYFaseUsanLaMismaCadena)
{
  const std::string sombra = sinComentarios(leer(dirSombra + ".cpp")), fase = sinComentarios(leer(dirFase + ".cpp"));
  // Las dos llaman a las mismas funciones compartidas...
  for(const char* funcion : {"NaoRL::actualizarAngulosValidos", "NaoRL::leerSensores", "NaoRL::sensoresPaso", "NaoRL::completarDecision",
                             "NaoRL::motivoEntrada", "decisor.decidir", "NaoRL::accionAObjetivos", "NaoRL::deseadosDesdeObjetivos",
                             "NaoRL::limitarObjetivos", "NaoRL::pasosMaximos", "NaoRL::inclinacionTorso", "NaoRL::cargarPolitica",
                             "NaoRL::mapearMotores", "NaoRL::poseHome"})
  {
    EXPECT_NE(sombra.find(funcion), std::string::npos) << "sombra: " << funcion;
    EXPECT_NE(fase.find(funcion), std::string::npos) << "fase: " << funcion;
  }
  // ... y ninguna tiene copia propia de la observacion.
  for(const std::string* codigo : {&sombra, &fase})
    for(const char* copia : {"armarBloque", "signoPsi", "signoD", "signoGyro", "signoArriba", "orientation3D", "foto.psi", "evaluar("})
      EXPECT_EQ(codigo->find(copia), std::string::npos) << copia;
}

GTEST_TEST(NaoRL, ObservacionIgualEnSombraYFase)
{
  // La sombra calcula sus z con observacionSiguiente() y la red recibe la de decidir(): deben ser la misma, paso a paso, y
  // las dos corridas (camino de la fase y camino de la sombra) deben dar lo mismo con las mismas entradas.
  Azar azar;
  Politica& red = politica().politica;
  const Accion& postura = politica().postura;
  Decisor fase, sombra;
  fase.reiniciar();
  sombra.reiniciar();
  EntradaPaso entradaFase, entradaSombra;
  EstadoVision visionFase, visionSombra;
  for(unsigned k = 1; k <= 400; ++k)
  {
    const SensoresPaso s = azar.sensores(k * 12u);
    entradaFase = leerSensores(s, entradaFase, false, 1300.f, visionFase);
    entradaSombra = leerSensores(s, entradaSombra, false, 1300.f, visionSombra);
    EXPECT_EQ(visionFase.ultimoPerceptoValido, visionSombra.ultimoPerceptoValido);
    if(k % ciclosPorDecision)
      continue;
    const float t = .012f * static_cast<float>(k);
    EntradaPaso eFase = entradaFase, eSombra = entradaSombra;
    completarDecision(eFase, .5f, 1.8f, t);
    completarDecision(eSombra, .5f, 1.8f, t);

    const Observacion candidata = sombra.observacionSiguiente(eSombra, postura);
    const Decisor::Resultado rFase = fase.decidir(red, eFase, postura, nullptr, 3);
    const Decisor::Resultado rSombra = sombra.decidir(red, eSombra, postura, nullptr, std::numeric_limits<int>::max());
    ASSERT_TRUE(rSombra.redEvaluada);
    EXPECT_EQ(candidata, sombra.observacion());
    EXPECT_EQ(fase.observacion(), sombra.observacion());
    EXPECT_EQ(rFase.accion, rSombra.accion);
  }
}

GTEST_TEST(NaoRL, LeerSensoresNoCambioConLaExtraccion)
{
  // Copia literal del leerSensores que tenia NaoRLWalkPhase antes de extraerlo. Si alguien cambia la funcion compartida, salta.
  Azar azar;
  EntradaPaso nueva, vieja;
  EstadoVision vision;
  unsigned ultimoValido = 0;
  bool metaCerca = false;
  const float distanciaMeta = 1300.f;
  for(unsigned k = 1; k <= 2000; ++k)
  {
    const SensoresPaso s = azar.sensores(k * 12u);
    nueva = leerSensores(s, nueva, false, distanciaMeta, vision);

    EntradaPaso e = vieja;
    e.gyro = {s.gyro[0], s.gyro[1], s.gyro[2]};
    const float w = s.qw, x = s.qx, y = s.qy, z = s.qz;
    e.arriba = {2.f * (x * z - w * y), 2.f * (y * z + w * x), 1.f - 2.f * (x * x + y * y)};
    e.angulos = s.angulos;
    const bool fresca = s.tiempoFrame && s.edadFrameMs <= 300;
    if(fresca && s.camaraValida && std::abs(s.psiCamara) <= static_cast<float>(NaoRL::pi) && std::abs(s.dCamaraMm / 1000.f) <= 2.f)
    {
      e.psi = -1.f * s.psiCamara;
      e.d = 1.f * (s.dCamaraMm / 1000.f);
      ultimoValido = s.tiempoFrame;
    }
    if(fresca && s.metaVisible && std::isfinite(s.distanciaMetaMm) && s.distanciaMetaMm >= 0.f && s.distanciaMetaMm <= distanciaMeta)
      metaCerca = true;
    vieja = e;

    ASSERT_EQ(nueva.gyro, vieja.gyro);
    ASSERT_EQ(nueva.arriba, vieja.arriba);
    ASSERT_EQ(nueva.angulos, vieja.angulos);
    ASSERT_EQ(nueva.psi, vieja.psi);
    ASSERT_EQ(nueva.d, vieja.d);
    ASSERT_EQ(vision.ultimoPerceptoValido, ultimoValido);
    ASSERT_EQ(vision.metaCerca, metaCerca);
  }

  // Sin camara: vis = (0, 0).
  EntradaPaso conCamara;
  conCamara.psi = .3f;
  conCamara.d = .2f;
  EstadoVision v;
  const EntradaPaso sinCamara = leerSensores(azar.sensores(12u), conCamara, true, distanciaMeta, v);
  EXPECT_EQ(sinCamara.psi, 0.f);
  EXPECT_EQ(sinCamara.d, 0.f);
  EXPECT_EQ(v.ultimoPerceptoValido, 0u);

  // Mando del contrato.
  EntradaPaso mando;
  mando.wzCmd = 9.f;
  completarDecision(mando, .5f, 1.8f, 2.f);
  EXPECT_EQ(mando.vxCmd, .5f);
  EXPECT_EQ(mando.wzCmd, 0.f);
  EXPECT_EQ(mando.frecuencia, 1.8f);
  EXPECT_EQ(mando.tMarcha, 2.f);
}

GTEST_TEST(NaoRL, LimitadorNoCambioConLaExtraccion)
{
  constexpr std::size_t n = 8, cabeza = 0, origen = 2, espejo = 5;
  Azar azar;
  std::array<float, n> minimos, maximos, pasos, deseados, nuevo, viejo;
  std::array<bool, n> limita;
  limita.fill(true);
  limita[cabeza] = false;
  const float margen = .05f;
  for(std::size_t j = 0; j < n; ++j)
  {
    minimos[j] = azar.en(-1.f, -.1f);
    maximos[j] = azar.en(.1f, 1.f);
    pasos[j] = azar.en(.02f, .09f);
    nuevo[j] = viejo[j] = 0.f;
  }
  minimos[7] = .3f;   // rango mas angosto que dos margenes: va al centro
  maximos[7] = .35f;
  for(int k = 0; k < 2000; ++k)
  {
    for(std::size_t j = 0; j < n; ++j)
      deseados[j] = azar.moneda(.02f) ? std::numeric_limits<float>::quiet_NaN() : azar.en(-2.f, 2.f);
    std::array<bool, n> recorteVel{}, recorteLim{};
    limitarObjetivos(deseados, nuevo, limita, minimos, maximos, pasos, margen, origen, espejo, recorteVel, recorteLim);

    for(std::size_t j = 0; j < n; ++j) // el limitarObjetivos que tenia la fase
    {
      if(j == cabeza || j == espejo)
        continue;
      float deseado = std::isfinite(deseados[j]) ? deseados[j] : viejo[j];
      const float minimo = minimos[j] + margen, maximo = maximos[j] - margen;
      deseado = minimo <= maximo ? std::clamp(deseado, minimo, maximo) : .5f * (minimos[j] + maximos[j]);
      viejo[j] += std::clamp(deseado - viejo[j], -pasos[j], pasos[j]);
    }
    viejo[espejo] = viejo[origen];
    ASSERT_EQ(nuevo, viejo);
    EXPECT_FALSE(recorteVel[cabeza]);
    EXPECT_FALSE(recorteLim[espejo]);
  }

  // Los contadores del CSV.
  std::array<float, n> objetivos{};
  std::array<bool, n> recorteVel{}, recorteLim{};
  deseados.fill(0.f);
  deseados[1] = pasos[1] / 2.f;  // ni uno ni otro
  deseados[3] = maximos[3];      // recorta el margen... y el paso, si el maximo esta lejos
  limitarObjetivos(deseados, objetivos, limita, minimos, maximos, pasos, margen, origen, espejo, recorteVel, recorteLim);
  EXPECT_FALSE(recorteVel[1]);
  EXPECT_FALSE(recorteLim[1]);
  EXPECT_TRUE(recorteLim[3]);
}

// ---------------------------------------------------------------------------------------------------------------------
// Escenario

STREAMABLE(NaoRLSombraPrueba,
{,
  (bool) usarSombra,
  (bool) siempre,
  (std::string) politica,
  (float) vxCmd,
  (float) frecuenciaMarcha,
  (std::vector<float>) lo,
  (std::vector<float>) hi,
  (std::vector<float>) velMax,
  (float) factorVelRoll,
  (float) margenLimites,
  (float) distanciaMetaSinCamara,
  (std::string) rutaRegistro,
});

STREAMABLE(NaoRLLimitadorFasePrueba,
{,
  (std::vector<float>) lo,
  (std::vector<float>) hi,
  (std::vector<float>) velMax,
  (float) factorVelRoll,
  (float) margenLimites,
  (float) distanciaMetaSinCamara,
});

GTEST_TEST(NaoRL, CarrilSombraSoloAnadeElModulo)
{
  const std::string base = "Config/Scenarios/";
  // threads.cfg: Carril + una linea, la del modulo.
  const std::vector<std::string> carril = lineas(leer(base + "Carril/threads.cfg")), sombra = lineas(leer(base + "CarrilSombra/threads.cfg"));
  ASSERT_EQ(sombra.size(), carril.size() + 1);
  std::size_t a = 0;
  int anadidas = 0;
  for(const std::string& l : sombra)
    if(a < carril.size() && l == carril[a])
      ++a;
    else
    {
      ++anadidas;
      EXPECT_NE(l.find("{representation = NaoRLSombraEstado; provider = NaoRLSombra;}"), std::string::npos) << l;
    }
  EXPECT_EQ(a, carril.size());
  EXPECT_EQ(anadidas, 1);
  // Va en el hilo Motion: entre el "executionUnit = Motion" y el siguiente executionUnit.
  const std::string texto = leer(base + "CarrilSombra/threads.cfg");
  const std::size_t motion = texto.find("executionUnit = Motion;"), modulo = texto.find("provider = NaoRLSombra;");
  ASSERT_NE(motion, std::string::npos);
  EXPECT_LT(motion, modulo);
  EXPECT_LT(modulo, texto.find("executionUnit =", motion + 10));

  // Lo demas, igual que Carril.
  for(const char* archivo : {"carrilBehavior.cfg", "carrilPerceptProvider.cfg", "carrilEvaluador.cfg", "audioProvider.cfg", "whistleRecognizer.cfg"})
    EXPECT_EQ(leer(base + "Carril/" + archivo), leer(base + "CarrilSombra/" + archivo)) << archivo;

  // Parametros de la sombra: los del encargo y los mismos del limitador que la fase.
  InMapFile stream("Scenarios/CarrilSombra/naoRLSombra.cfg");
  ASSERT_TRUE(stream.exists());
  NaoRLSombraPrueba s;
  stream >> s;
  EXPECT_TRUE(s.usarSombra);
  EXPECT_EQ(s.politica, "nao_v08");
  EXPECT_EQ(s.vxCmd, .5f);
  EXPECT_EQ(s.frecuenciaMarcha, 1.8f);
  EXPECT_EQ(s.rutaRegistro, "/home/nao/logs");
  InMapFile streamFase("Scenarios/CarrilRL/naoRLWalkEngine.cfg");
  ASSERT_TRUE(streamFase.exists());
  NaoRLLimitadorFasePrueba f;
  streamFase >> f;
  EXPECT_EQ(s.lo, f.lo);
  EXPECT_EQ(s.hi, f.hi);
  EXPECT_EQ(s.velMax, f.velMax);
  EXPECT_EQ(s.factorVelRoll, f.factorVelRoll);
  EXPECT_EQ(s.margenLimites, f.margenLimites);
  EXPECT_EQ(s.distanciaMetaSinCamara, f.distanciaMetaSinCamara);
}

// ---------------------------------------------------------------------------------------------------------------------
// CSV

GTEST_TEST(NaoRL, CabeceraDelCsvDeSombra)
{
  const std::array<const char*, nPiernasRegistro> piernas = {"lHipYawPitch", "lHipRoll", "lHipPitch", "lKneePitch", "lAnklePitch", "lAnkleRoll",
                                                             "rHipYawPitch", "rHipRoll", "rHipPitch", "rKneePitch", "rAnklePitch", "rAnkleRoll"};
  std::vector<std::string> esperadas = {"t_ms", "estado_bhuman", "t_marcha_s", "t_inferencia_us", "fallDown", "inclinacion", "gyro_x", "gyro_y", "gyro_z",
                                        "arriba_x", "arriba_y", "arriba_z", "psi_obs", "d_obs", "edad_camara_ms", "zmax", "zmax_idx", "n_z4"};
  const std::array<const char*, 15> motores = {"LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipRoll", "RHipPitch",
                                               "RKneePitch", "RAnklePitch", "RAnkleRoll", "LShoulderPitch", "RShoulderPitch", "LElbowRoll", "RElbowRoll"};
  const auto porMotor = [&](const std::string& prefijo)
  {
    for(const char* m : motores)
      esperadas.push_back(prefijo + m);
  };
  porMotor("a_");
  porMotor("obj_pol_");
  porMotor("obj_lim_");
  esperadas.push_back("n_recorte_vel");
  esperadas.push_back("n_recorte_lim");
  porMotor("bh_");
  porMotor("ang_");
  for(const char* p : piernas)
  {
    esperadas.push_back(std::string("corr_") + p);
    esperadas.push_back(std::string("temp_") + p);
  }
  const std::string cabecera = cabeceraSombra(piernas);
  EXPECT_EQ(columnas(cabecera), esperadas);

  // Una linea tiene las mismas columnas, en orden.
  FilaSombra f;
  f.tMs = 111;
  f.estadoBHuman = "walk";
  f.tMarchaS = 1.5f;
  f.fallDown = "upright";
  f.zmax = 3.25f;
  f.zmaxIdx = 42;
  f.nZ4 = 7;
  f.a.fill(.1f);
  f.objPol.fill(.2f);
  f.objLim.fill(.3f);
  f.nRecorteVel = 4;
  f.nRecorteLim = 5;
  f.bh.fill(.6f);
  f.ang.fill(.7f);
  f.corr.fill(800);
  f.temp.fill(41);
  const std::vector<std::string> v = columnas(lineaSombra(f));
  ASSERT_EQ(v.size(), esperadas.size());
  const auto valor = [&](const std::string& columna)
  {
    return std::stof(v[std::find(esperadas.begin(), esperadas.end(), columna) - esperadas.begin()]);
  };
  EXPECT_EQ(v[1], "walk");
  EXPECT_EQ(v[4], "upright");
  EXPECT_FLOAT_EQ(valor("t_marcha_s"), 1.5f);
  EXPECT_FLOAT_EQ(valor("zmax"), 3.25f);
  EXPECT_FLOAT_EQ(valor("zmax_idx"), 42.f);
  EXPECT_FLOAT_EQ(valor("n_z4"), 7.f);
  EXPECT_FLOAT_EQ(valor("a_RElbowRoll"), .1f);
  EXPECT_FLOAT_EQ(valor("obj_pol_LHipRoll"), .2f);
  EXPECT_FLOAT_EQ(valor("obj_lim_LHipRoll"), .3f);
  EXPECT_FLOAT_EQ(valor("n_recorte_vel"), 4.f);
  EXPECT_FLOAT_EQ(valor("n_recorte_lim"), 5.f);
  EXPECT_FLOAT_EQ(valor("bh_LHipRoll"), .6f);
  EXPECT_FLOAT_EQ(valor("ang_LHipRoll"), .7f);
  EXPECT_FLOAT_EQ(valor("corr_rAnkleRoll"), 800.f);
  EXPECT_FLOAT_EQ(valor("temp_rAnkleRoll"), 41.f);
}

GTEST_TEST(NaoRL, EstadisticasZContraElNormalizador)
{
  Politica& red = politica().politica;
  // El golden se genera como media + desviacion * N(0, 1): aqui se construye z exacto con una entrada conocida.
  const NaoRLPoliticaConfig c = []
  {
    NaoRLPoliticaConfig cfg;
    InMapFile stream("NeuralNets/NaoRL/nao_v08.cfg", 0u);
    stream >> cfg;
    return cfg;
  }();
  ASSERT_EQ(c.media.size(), nObs);
  Observacion obs;
  for(std::size_t i = 0; i < nObs; ++i)
    obs[i] = c.media[i];
  float zmax;
  int idx, n;
  red.estadisticasZ(obs, umbralZ, zmax, idx, n);
  EXPECT_NEAR(zmax, 0.f, 1e-3f);
  EXPECT_EQ(n, 0);

  obs[17] = c.media[17] + 6.f * c.desviacion[17];
  obs[100] = c.media[100] - 4.5f * c.desviacion[100];
  obs[5] = c.media[5] + 3.f * c.desviacion[5];
  red.estadisticasZ(obs, umbralZ, zmax, idx, n);
  EXPECT_NEAR(zmax, 6.f, 1e-2f);
  EXPECT_EQ(idx, 17);
  EXPECT_EQ(n, 2);

  obs[60] = std::numeric_limits<float>::quiet_NaN(); // un sensor roto se ve en zmax_idx, no se esconde
  red.estadisticasZ(obs, umbralZ, zmax, idx, n);
  EXPECT_TRUE(std::isinf(zmax));
  EXPECT_EQ(idx, 60);
  EXPECT_EQ(n, 3);
}

GTEST_TEST(NaoRL, RegistroDeSombraCreaArchivosNumerados)
{
  const std::filesystem::path dir = std::filesystem::temp_directory_path() / "naorl_sombra_registro";
  std::filesystem::remove_all(dir);
  {
    NaoRLRegistro r;
    ASSERT_TRUE(r.abrir(dir.string(), "a,b", "naoRLsombra_"));
    EXPECT_EQ(r.archivo(), (dir / "naoRLsombra_0.csv").string());
    r.escribir("1,2");
  }
  {
    NaoRLRegistro r;
    ASSERT_TRUE(r.abrir(dir.string(), "a,b", "naoRLsombra_"));
    EXPECT_EQ(r.archivo(), (dir / "naoRLsombra_1.csv").string());
  }
  std::ifstream f(dir / "naoRLsombra_0.csv");
  std::stringstream ss;
  ss << f.rdbuf();
  EXPECT_EQ(ss.str(), "a,b\n1,2\n");
  std::filesystem::remove_all(dir);
}
