/**
 * @file ParidadNaoV08.cpp
 *
 * Etapa E0 de la marcha RL (demo_carril/docs/despliegue_nao_v08.md, seccion 7): carga Config/NeuralNets/NaoRL/nao_v08.cfg
 * con el MISMO cargador que el modulo NaoRLWalkEngine, lee nao_v08_golden.json (64 observaciones crudas y la accion de
 * Python) y exige diferencia maxima < 1e-4. Ademas comprueba el orden del bloque de 42 contra las etiquetas del golden,
 * la historia con ceros, la accion -> objetivos, que lo/hi/velMax de naoRLWalkEngine.cfg sean los del golden y que el
 * escenario CarrilRL solo cambie el proveedor de la marcha respecto a Carril.
 *
 * Correr desde la raiz del repo (para que se encuentre Config/):
 *   Build/Linux/Tests/Develop/Tests --gtest_filter='NaoRL.*'
 */

#include "Platform/File.h"
#include "Streaming/InStreams.h"
#include "../../../Tools/NaoRL/NaoRLPoliticaConfig.h" // Tests no tiene Src/ en la ruta de inclusion
#include <gtest/gtest.h>
#include <chrono>
#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

namespace
{
  /** JSON minimo: solo lo que trae el golden (objeto de numeros, cadenas y arreglos anidados). */
  struct Json
  {
    enum Tipo { numero, cadena, arreglo, objeto } tipo = numero;
    double n = 0.;
    std::string s;
    std::vector<Json> a;
    std::map<std::string, Json> o;

    const Json& operator[](const std::string& k) const { return o.at(k); }
    std::vector<float> floats() const
    {
      std::vector<float> v;
      for(const Json& x : a)
        v.push_back(static_cast<float>(x.n));
      return v;
    }
  };

  class LectorJson
  {
  public:
    explicit LectorJson(const std::string& texto) : t(texto) {}

    Json valor()
    {
      espacios();
      Json j;
      if(t[i] == '{')
      {
        j.tipo = Json::objeto;
        ++i;
        espacios();
        while(t[i] != '}')
        {
          const std::string clave = cadena();
          espacios();
          ++i; // ':'
          j.o[clave] = valor();
          espacios();
          if(t[i] == ',')
            ++i;
          espacios();
        }
        ++i;
      }
      else if(t[i] == '[')
      {
        j.tipo = Json::arreglo;
        ++i;
        espacios();
        while(t[i] != ']')
        {
          j.a.push_back(valor());
          espacios();
          if(t[i] == ',')
            ++i;
          espacios();
        }
        ++i;
      }
      else if(t[i] == '"')
      {
        j.tipo = Json::cadena;
        j.s = cadena();
      }
      else
      {
        std::size_t usados = 0;
        j.n = std::stod(t.substr(i, 40), &usados);
        i += usados;
      }
      return j;
    }

  private:
    void espacios()
    {
      while(i < t.size() && std::isspace(static_cast<unsigned char>(t[i])))
        ++i;
    }
    std::string cadena()
    {
      std::string s;
      ++i; // '"'
      while(t[i] != '"')
        s += t[i++];
      ++i;
      return s;
    }

    const std::string& t;
    std::size_t i = 0;
  };

  std::string leerTexto(const std::string& ruta)
  {
    std::ifstream f(ruta);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
  }

  const Json& golden()
  {
    static const std::string texto = leerTexto(std::string(File::getBHDir()) + "/Src/Apps/Tests/NaoRL/nao_v08_golden.json");
    static const Json j = LectorJson(texto).valor();
    return j;
  }

  NaoRL::PoliticaCargada& cargada()
  {
    static NaoRL::PoliticaCargada p;
    static bool ok = false;
    if(!ok)
    {
      std::string error;
      ok = NaoRL::cargarPolitica("NeuralNets/NaoRL/nao_v08.cfg", p, error);
      EXPECT_TRUE(ok) << error;
    }
    return p;
  }
}

/** Solo los parametros de naoRLWalkEngine.cfg que se comparan con el golden (el resto se ignora al leer). */
STREAMABLE(NaoRLLimitesPrueba,
{,
  (std::vector<float>) lo,
  (std::vector<float>) hi,
  (std::vector<float>) velMax,
});

GTEST_TEST(NaoRL, ParidadGoldenE0)
{
  const Json& g = golden();
  ASSERT_EQ(g["generacion"].s, "nao_v08");
  ASSERT_EQ(static_cast<int>(g["n_obs"].n), static_cast<int>(NaoRL::nObs));
  ASSERT_EQ(static_cast<int>(g["n_accion"].n), static_cast<int>(NaoRL::nMotores));
  NaoRL::PoliticaCargada& p = cargada();
  ASSERT_TRUE(p.politica.valida());

  const std::vector<Json>& obs = g["obs"].a;
  const std::vector<Json>& acc = g["accion"].a;
  ASSERT_EQ(obs.size(), 64u);
  ASSERT_EQ(acc.size(), 64u);
  double maxDif = 0.;
  for(std::size_t c = 0; c < obs.size(); ++c)
  {
    const std::vector<float> o = obs[c].floats();
    const std::vector<float> esperado = acc[c].floats();
    ASSERT_EQ(o.size(), NaoRL::nObs);
    ASSERT_EQ(esperado.size(), NaoRL::nMotores);
    NaoRL::Observacion x;
    std::copy(o.begin(), o.end(), x.begin());
    const NaoRL::Accion a = p.politica.evaluar(x);
    for(std::size_t i = 0; i < NaoRL::nMotores; ++i)
      maxDif = std::max(maxDif, std::abs(static_cast<double>(a[i]) - esperado[i]));
  }
  std::cout << "[E0] 64 casos, diferencia maxima = " << maxDif << " (limite 1e-4)" << std::endl;
  EXPECT_LT(maxDif, 1e-4);

  // Tiempo de una decision en esta PC (el del Atom se mide en el robot con la columna t_inferencia_us).
  NaoRL::Observacion x;
  const std::vector<float> o = obs[0].floats();
  std::copy(o.begin(), o.end(), x.begin());
  constexpr int n = 20000;
  volatile float sumidero = 0.f; // evita que el compilador quite el bucle
  double peorUs = 0.;
  const auto t0 = std::chrono::steady_clock::now();
  for(int k = 0; k < n; ++k)
  {
    const auto a0 = std::chrono::steady_clock::now();
    x[0] = static_cast<float>(k % 7) * 1e-3f;
    sumidero = sumidero + p.politica.evaluar(x)[0];
    peorUs = std::max(peorUs, std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - a0).count());
  }
  const double mediaUs = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / n;
  std::cout << "[E0] inferencia en PC: media " << mediaUs << " us, peor " << peorUs << " us (" << n << " decisiones)"
            << std::endl;
}

GTEST_TEST(NaoRL, ContratoDelCfg)
{
  const Json& g = golden();
  NaoRL::PoliticaCargada& p = cargada();
  EXPECT_FLOAT_EQ(p.escala, static_cast<float>(g["action_scale"].n));
  EXPECT_FLOAT_EQ(p.ctrlDt, .024f);
  const std::vector<float> postura = g["postura"].floats();
  for(std::size_t i = 0; i < NaoRL::nMotores; ++i)
  {
    EXPECT_EQ(g["motores"].a[i].s, NaoRL::nombresMotores[i]);
    EXPECT_NEAR(p.postura[i], postura[i], 1e-6);
  }

  // lo/hi/velMax del .cfg de parametros del modulo == golden.
  InMapFile stream("Scenarios/CarrilRL/naoRLWalkEngine.cfg");
  ASSERT_TRUE(stream.exists());
  NaoRLLimitesPrueba lim;
  stream >> lim;
  const std::vector<float> lo = g["lo"].floats(), hi = g["hi"].floats(), vel = g["vel_max"].floats();
  ASSERT_EQ(lim.lo.size(), NaoRL::nMotores);
  ASSERT_EQ(lim.hi.size(), NaoRL::nMotores);
  ASSERT_EQ(lim.velMax.size(), NaoRL::nMotores);
  for(std::size_t i = 0; i < NaoRL::nMotores; ++i)
  {
    EXPECT_NEAR(lim.lo[i], lo[i], 1e-6) << NaoRL::nombresMotores[i];
    EXPECT_NEAR(lim.hi[i], hi[i], 1e-6) << NaoRL::nombresMotores[i];
    EXPECT_NEAR(lim.velMax[i], vel[i], 1e-5) << NaoRL::nombresMotores[i];
  }
}

GTEST_TEST(NaoRL, OrdenDelBloqueSegunEtiquetas)
{
  // Valores distintos en cada canal; luego cada posicion del bloque se compara con lo que dice su etiqueta.
  NaoRL::EntradaPaso e;
  e.gyro = {.11f, .12f, .13f};
  e.arriba = {.21f, .22f, .23f};
  NaoRL::Accion postura{};
  for(std::size_t i = 0; i < NaoRL::nMotores; ++i)
  {
    postura[i] = .01f * static_cast<float>(i);
    e.angulos[i] = 1.f + .01f * static_cast<float>(i) + postura[i];
    e.accionAnterior[i] = -.5f + .01f * static_cast<float>(i);
  }
  e.vxCmd = .5f;
  e.wzCmd = .07f;
  e.tMarcha = .1f;
  e.frecuencia = 1.8f;
  e.psi = .9f;   // se recorta a .5
  e.d = -.3f;
  const NaoRL::Bloque b = NaoRL::armarBloque(e, postura);

  const std::vector<Json>& etq = golden()["etiquetas"].a;
  ASSERT_EQ(etq.size(), NaoRL::nObs);
  const double fase = 2. * NaoRL::pi * 1.8 * .1;
  for(std::size_t k = 0; k < NaoRL::obsPorPaso; ++k)
  {
    const std::string n = etq[k].s.substr(3); // sin "h0_"
    float esperado = 1e9f;
    if(n.rfind("gyro", 0) == 0)
      esperado = e.gyro[n[4] - '0'];
    else if(n.rfind("arriba", 0) == 0)
      esperado = e.arriba[n[6] - '0'];
    else if(n.rfind("ang_", 0) == 0 || n.rfind("acc_", 0) == 0)
      for(std::size_t i = 0; i < NaoRL::nMotores; ++i)
        if(n.substr(4) == NaoRL::nombresMotores[i])
          esperado = n[1] == 'n' ? 1.f + .01f * static_cast<float>(i) : e.accionAnterior[i];
    if(n == "cmd_vx") esperado = .5f;
    if(n == "cmd_wz") esperado = .07f;
    if(n == "fase_cos") esperado = static_cast<float>(std::cos(fase));
    if(n == "fase_sin") esperado = static_cast<float>(std::sin(fase));
    if(n == "vis_psi") esperado = .5f;
    if(n == "vis_d") esperado = -.3f;
    EXPECT_NEAR(b[k], esperado, 1e-6) << "indice " << k << " (" << n << ")";
  }
  // Los bloques h1 y h2 tienen las mismas etiquetas desplazadas 42 y 84.
  for(std::size_t k = 0; k < NaoRL::obsPorPaso; ++k)
  {
    EXPECT_EQ(etq[k + 42].s, "h1_" + etq[k].s.substr(3));
    EXPECT_EQ(etq[k + 84].s, "h2_" + etq[k].s.substr(3));
  }
}

GTEST_TEST(NaoRL, HistoriaEmpiezaEnCerosYDesplaza)
{
  NaoRL::Historia h;
  h.reiniciar();
  NaoRL::Bloque b1, b2, b3;
  b1.fill(1.f);
  b2.fill(2.f);
  b3.fill(3.f);
  const NaoRL::Observacion& o1 = h.empujar(b1);
  for(std::size_t k = 0; k < NaoRL::nObs; ++k)
    EXPECT_EQ(o1[k], k < 42 ? 1.f : 0.f);
  h.empujar(b2);
  const NaoRL::Observacion& o3 = h.empujar(b3);
  for(std::size_t k = 0; k < NaoRL::nObs; ++k)
    EXPECT_EQ(o3[k], k < 42 ? 3.f : k < 84 ? 2.f : 1.f);
}

GTEST_TEST(NaoRL, AccionAObjetivos)
{
  const Json& g = golden();
  const std::vector<float> lo = g["lo"].floats(), hi = g["hi"].floats();
  NaoRL::Accion l, u, a;
  std::copy(lo.begin(), lo.end(), l.begin());
  std::copy(hi.begin(), hi.end(), u.begin());
  const NaoRL::Accion& postura = cargada().postura;
  a.fill(0.f);
  a[NaoRL::mLHipPitch] = 1.f;    // -0.4 + 0.3 = -0.1
  a[NaoRL::mLKneePitch] = -1.f;  //  0.8 - 0.3 =  0.5
  a[NaoRL::mRHipRoll] = 1.f;     //  0.0 + 0.3 =  0.3 (hi = 0.379)
  a[NaoRL::mLElbowRoll] = 1.f;   // -0.2 + 0.3 = 0.1 -> recorta a hi = -0.0349
  const NaoRL::Accion o = NaoRL::accionAObjetivos(a, postura, .3f, l, u);
  EXPECT_NEAR(o[NaoRL::mLHipPitch], -.1f, 1e-6);
  EXPECT_NEAR(o[NaoRL::mLKneePitch], .5f, 1e-6);
  EXPECT_NEAR(o[NaoRL::mRHipRoll], .3f, 1e-6);
  EXPECT_NEAR(o[NaoRL::mLElbowRoll], u[NaoRL::mLElbowRoll], 1e-6);
  EXPECT_NEAR(o[NaoRL::mLHipYawPitch], 0.f, 1e-6);
  EXPECT_TRUE(NaoRL::finita(o));
  a[3] = std::nanf("");
  EXPECT_FALSE(NaoRL::finita(a));
}

GTEST_TEST(NaoRL, EscenarioCarrilRLSoloCambiaLaMarcha)
{
  const std::string base = std::string(File::getBHDir()) + "/Config/Scenarios/";
  std::istringstream carril(leerTexto(base + "Carril/threads.cfg")), carrilRL(leerTexto(base + "CarrilRL/threads.cfg"));
  std::vector<std::string> a, b;
  for(std::string l; std::getline(carril, l);)
    a.push_back(l);
  for(std::string l; std::getline(carrilRL, l);)
    if(l.find("// CarrilRL:") == std::string::npos && l.find("// Es la UNICA diferencia") == std::string::npos)
      b.push_back(l);
  ASSERT_EQ(a.size(), b.size());
  int diferencias = 0;
  for(std::size_t k = 0; k < a.size(); ++k)
    if(a[k] != b[k])
    {
      ++diferencias;
      EXPECT_NE(a[k].find("representation = WalkAtRelativeSpeedGenerator; provider = WalkAtSpeedEngine;"), std::string::npos);
      EXPECT_NE(b[k].find("representation = WalkAtRelativeSpeedGenerator; provider = NaoRLWalkEngine;"), std::string::npos);
    }
  EXPECT_EQ(diferencias, 1);
}
