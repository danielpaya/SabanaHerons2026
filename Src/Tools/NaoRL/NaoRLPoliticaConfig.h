/**
 * @file NaoRLPoliticaConfig.h
 *
 * Lectura de Config/NeuralNets/NaoRL/<politica>.cfg (generado por rl/a_cfg.py, ver demo_carril/docs/despliegue_nao_v08.md)
 * con el streaming de B-Human. Lo usan el modulo NaoRLWalkEngine y la prueba de paridad E0, asi que la prueba
 * ejercita exactamente el mismo camino de carga que el robot.
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/InStreams.h"
#include "Platform/File.h"
#include "NaoRLPolitica.h" // misma carpeta: asi tambien compila en Tests, que no tiene Src/ en la ruta
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

/** Una capa densa del .cfg (mismo formato que CopaRaceLayerConfig de Camilo). */
STREAMABLE(NaoRLCapaConfig,
{,
  (unsigned)(0) filas,
  (unsigned)(0) columnas,
  (std::vector<float>) pesos,
  (std::vector<float>) sesgo,
});

/** El .cfg completo tal como lo escribe rl/a_cfg.py. */
STREAMABLE(NaoRLPoliticaConfig,
{,
  (std::string) nombre,
  (std::string) activacion,
  (std::vector<float>) media,
  (std::vector<float>) desviacion,
  (std::vector<NaoRLCapaConfig>) ocultas,
  (NaoRLCapaConfig) salida,
  (unsigned)(0) historia,
  (unsigned)(0) obsPorPaso,
  (float)(0.f) actionScale,
  (float)(0.f) ctrlDt,
  (float)(0.f) vxMax,
  (std::vector<float>) postura,
  (std::vector<std::string>) motores,
  (std::string) hashPesos,            /**< FNV-1a 64 de media, desviacion y capas (contrato 6c, H3; hash_pesos() de a_cfg.py). */
  (std::vector<float>) autotestObs,   /**< autotestCasos observaciones crudas seguidas. */
  (std::vector<float>) autotestAccion, /**< La accion esperada de cada una. */
  (unsigned)(0) autotestCasos,
});

namespace NaoRL
{
  constexpr unsigned minCasosAutotest = 2;     /**< H3: los que escribe a_cfg.py. */
  constexpr float toleranciaAutotest = 1e-4f;  /**< H3: la misma de la paridad E0. */

  /**
   * Lo que el hash NO cubre y un byte alterado cambiaria sin que nada lo note (H7 compara contra esta misma postura):
   * se fija al contrato (seccion 3). Otra generacion con otra postura debe actualizar esta tabla a proposito.
   */
  constexpr Accion posturaContrato = {0.f, 0.f, -.4f, .8f, -.4f, 0.f, 0.f, -.4f, .8f, -.4f, 0.f, 1.57f, 1.57f, -.2f, .2f};
  constexpr float escalaContrato = .3f;
  constexpr float ctrlDtContrato = .024f;

  /** Lo que el modulo necesita del .cfg, ya validado. */
  struct PoliticaCargada
  {
    std::string nombre;
    Politica politica;
    Accion postura{};
    float escala = 0.f;
    float ctrlDt = 0.f;
    float vxMax = 0.f;
  };

  /** FNV-1a de 64 bits sobre floats, byte a byte en little-endian (hash_pesos() de rl/a_cfg.py). */
  class HashPesos
  {
  public:
    void avanzar(const std::vector<float>& valores)
    {
      for(const float v : valores)
      {
        std::uint32_t bits;
        std::memcpy(&bits, &v, sizeof(bits));
        for(int desplazamiento = 0; desplazamiento < 32; desplazamiento += 8)
          h = (h ^ ((bits >> desplazamiento) & 0xffu)) * 0x100000001b3ull;
      }
    }

    std::string texto() const
    {
      char buf[17];
      std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
      return buf;
    }

  private:
    std::uint64_t h = 0xcbf29ce484222325ull;
  };

  /** El hash de un .cfg ya leido: media, desviacion y, por capa (la de salida al final), pesos por filas y sesgo. */
  inline std::string calcularHashPesos(const NaoRLPoliticaConfig& c)
  {
    HashPesos hash;
    hash.avanzar(c.media);
    hash.avanzar(c.desviacion);
    for(const NaoRLCapaConfig& cc : c.ocultas)
    {
      hash.avanzar(cc.pesos);
      hash.avanzar(cc.sesgo);
    }
    hash.avanzar(c.salida.pesos);
    hash.avanzar(c.salida.sesgo);
    return hash.texto();
  }

  /**
   * Comprueba la gramatica del .cfg ANTES de darselo a InMapFile: con Develop en el robot, un error de sintaxis en SimpleMap
   * es FAIL -> abort, o sea que un archivo truncado tumbaria bhuman en vez de dejar la politica "no cargada". Acepta justo lo
   * que escribe a_cfg.py: registro = (clave '=' valor ';')*, valor = literal | cadena | '[' valores ']' | '{' registro '}'.
   */
  class SintaxisCfg
  {
  public:
    explicit SintaxisCfg(const std::string& texto) : t(texto) {}

    bool valida()
    {
      return registro(0) && (espacios(), i == t.size());
    }

  private:
    static constexpr int maxProfundidad = 8; /**< El .cfg tiene 3 niveles: una entrada mala no debe agotar la pila. */

    void espacios()
    {
      while(i < t.size())
      {
        if(t[i] == ' ' || t[i] == '\n' || t[i] == '\r' || t[i] == '\t')
          ++i;
        else if(t[i] == '/' && i + 1 < t.size() && t[i + 1] == '/')
          while(i < t.size() && t[i] != '\n')
            ++i;
        else
          break;
      }
    }

    bool literal()
    {
      const std::size_t inicio = i;
      while(i < t.size() && (std::isalnum(static_cast<unsigned char>(t[i])) || t[i] == '_' || t[i] == '.' || t[i] == '+' || t[i] == '-'))
        ++i;
      return i > inicio;
    }

    bool cadena()
    {
      ++i; // '"'
      while(i < t.size() && t[i] != '"' && t[i] != '\n')
        ++i;
      if(i >= t.size() || t[i] != '"')
        return false;
      ++i;
      return true;
    }

    bool simbolo(const char c)
    {
      espacios();
      if(i >= t.size() || t[i] != c)
        return false;
      ++i;
      return true;
    }

    /** Hasta fin de archivo (nivel 0) o hasta la '}' que lo cierra (la consume el que llama). */
    bool registro(const int nivel)
    {
      while(true)
      {
        espacios();
        if(i >= t.size() || t[i] == '}')
          return nivel == 0 ? i >= t.size() : i < t.size();
        if(!literal() || !simbolo('=') || !valor(nivel) || !simbolo(';'))
          return false;
      }
    }

    bool valor(const int nivel)
    {
      if(nivel >= maxProfundidad)
        return false;
      espacios();
      if(i >= t.size())
        return false;
      if(t[i] == '"')
        return cadena();
      if(t[i] == '{')
      {
        ++i;
        return registro(nivel + 1) && simbolo('}');
      }
      if(t[i] == '[')
      {
        ++i;
        espacios();
        if(i < t.size() && t[i] == ']')
        {
          ++i;
          return true;
        }
        do
        {
          if(!valor(nivel + 1))
            return false;
        }
        while(simbolo(','));
        return simbolo(']');
      }
      return literal();
    }

    const std::string& t;
    std::size_t i = 0;
  };

  /** Lee el archivo tal cual (ruta relativa a Config/ o absoluta). */
  inline bool leerArchivo(const std::string& ruta, std::string& texto)
  {
    File archivo(ruta, "rb");
    if(!archivo.exists())
      return false;
    texto.resize(archivo.getSize());
    if(!texto.empty())
      archivo.read(texto.data(), texto.size());
    return true;
  }

  /**
   * Lee y valida el .cfg (ruta relativa a Config/, p. ej. "NeuralNets/NaoRL/nao_v08.cfg"). Rechaza todo lo que no
   * coincida con el contrato: sintaxis, activacion distinta de swish, otra generacion, otra historia u orden de motores,
   * dimensiones, y (H3) el hash de los pesos y el autotest de 2 casos. Si devuelve false, salida.politica queda sin cargar.
   */
  inline bool cargarPolitica(const std::string& ruta, PoliticaCargada& salida, std::string& error)
  {
    std::string texto;
    if(!leerArchivo(ruta, texto))
    {
      error = "no existe " + ruta;
      return false;
    }
    if(!SintaxisCfg(texto).valida())
    {
      error = "archivo truncado o con errores de sintaxis";
      return false;
    }
    // Mascara 0: en el robot, InMap hace FAIL -> abort con un campo ausente, de otro tipo o sin usar. Aqui eso no debe tumbar
    // bhuman: los campos se quedan vacios y lo rechaza la validacion de abajo (hash, autotest, dimensiones).
    InMapFile stream(ruta, 0u);
    if(!stream.exists())
    {
      error = "no existe " + ruta;
      return false;
    }
    NaoRLPoliticaConfig c;
    stream >> c;
    if(c.activacion != "swish")
    {
      error = "activacion '" + c.activacion + "' (se espera swish)";
      return false;
    }
    // De otra generacion: el hash no lo detecta, porque ese archivo es coherente consigo mismo.
    const std::size_t barra = ruta.find_last_of("/\\");
    const std::string archivo = barra == std::string::npos ? ruta : ruta.substr(barra + 1);
    if(c.nombre + ".cfg" != archivo)
    {
      error = "el archivo " + archivo + " dice ser la generacion '" + c.nombre + "'";
      return false;
    }
    if(c.historia != nHistoria || c.obsPorPaso != obsPorPaso)
    {
      error = "historia/obsPorPaso distintos de 3 x 42";
      return false;
    }
    if(c.motores.size() != nMotores || c.postura.size() != nMotores)
    {
      error = "motores/postura no tienen 15 entradas";
      return false;
    }
    for(std::size_t i = 0; i < nMotores; ++i)
    {
      if(c.motores[i] != nombresMotores[i])
      {
        error = "orden de motores distinto al del contrato en la posicion " + std::to_string(i) + " (" + c.motores[i] + ")";
        return false;
      }
      if(!(std::abs(c.postura[i] - posturaContrato[i]) <= 1e-5f))
      {
        error = "postura de " + c.motores[i] + " distinta de la del contrato";
        return false;
      }
    }
    if(!(std::abs(c.actionScale - escalaContrato) <= 1e-6f))
    {
      error = "actionScale distinto de " + std::to_string(escalaContrato);
      return false;
    }
    if(!(std::abs(c.ctrlDt - ctrlDtContrato) <= 1e-6f))
    {
      error = "ctrlDt distinto de " + std::to_string(ctrlDtContrato);
      return false;
    }

    // H3(a): el hash de los pesos, recalculado con los floats leidos.
    const std::string hash = calcularHashPesos(c);
    if(c.hashPesos != hash)
    {
      error = "hashPesos '" + c.hashPesos + "' no coincide con el calculado '" + hash + "'";
      return false;
    }

    const auto aCapa = [](const NaoRLCapaConfig& cc)
    {
      Capa capa;
      capa.filas = cc.filas;
      capa.columnas = cc.columnas;
      capa.pesos = cc.pesos;
      capa.sesgo = cc.sesgo;
      return capa;
    };
    std::vector<Capa> ocultas;
    for(const NaoRLCapaConfig& cc : c.ocultas)
      ocultas.push_back(aCapa(cc));
    Politica politica;
    if(!politica.configurar(c.media, c.desviacion, ocultas, aCapa(c.salida), error))
      return false;

    // H3(b): autotest al cargar.
    if(c.autotestCasos < minCasosAutotest || c.autotestObs.size() != c.autotestCasos * nObs ||
       c.autotestAccion.size() != c.autotestCasos * nMotores)
    {
      error = "autotest incompleto (" + std::to_string(c.autotestCasos) + " casos)";
      return false;
    }
    for(std::size_t caso = 0; caso < c.autotestCasos; ++caso)
    {
      Observacion obs;
      std::copy_n(c.autotestObs.begin() + caso * nObs, nObs, obs.begin());
      const Accion accion = politica.evaluar(obs);
      for(std::size_t i = 0; i < nMotores; ++i)
        if(!(std::abs(accion[i] - c.autotestAccion[caso * nMotores + i]) <= toleranciaAutotest))
        {
          error = "autotest: caso " + std::to_string(caso) + ", motor " + nombresMotores[i] + " difiere mas de 1e-4";
          return false;
        }
    }

    salida.nombre = c.nombre;
    salida.politica = std::move(politica);
    std::copy(c.postura.begin(), c.postura.end(), salida.postura.begin());
    salida.escala = c.actionScale;
    salida.ctrlDt = c.ctrlDt;
    salida.vxMax = c.vxMax;
    return true;
  }
}
