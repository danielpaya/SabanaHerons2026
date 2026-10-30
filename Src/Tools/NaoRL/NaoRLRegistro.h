/**
 * @file NaoRLRegistro.h
 *
 * Escritor de CSV en un hilo aparte: el hilo Motion solo encola lineas (no toca el disco). Lo usan la fase de marcha y el
 * modo sombra.
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

class NaoRLRegistro
{
public:
  ~NaoRLRegistro()
  {
    if(hilo.joinable())
    {
      {
        std::lock_guard<std::mutex> l(mutex);
        terminar = true;
      }
      cv.notify_one();
      hilo.join();
    }
  }

  /** Abre <directorio>/<prefijo><n>.csv (n = primer numero libre) y escribe la cabecera. */
  bool abrir(const std::string& directorio, const std::string& cabecera, const std::string& prefijo = "naoRL_")
  {
    std::error_code ec;
    std::filesystem::create_directories(directorio, ec);
    for(int n = 0; n < 10000 && ruta.empty(); ++n)
    {
      const std::string candidato = directorio + "/" + prefijo + std::to_string(n) + ".csv";
      if(!std::filesystem::exists(candidato, ec))
        ruta = candidato;
    }
    if(ruta.empty())
      return false;
    salida.open(ruta, std::ios::trunc);
    if(!salida.is_open())
      return false;
    salida << cabecera << '\n';
    salida.flush();
    estaAbierto = true;
    hilo = std::thread([this] { bucle(); });
    return true;
  }

  bool abierto() const { return estaAbierto; }
  const std::string& archivo() const { return ruta; }

  void escribir(std::string linea)
  {
    if(!estaAbierto)
      return;
    {
      std::lock_guard<std::mutex> l(mutex);
      if(cola.size() > 4000) // ~100 s de atraso: el disco no da abasto; mejor perder lineas que memoria
      {
        ++descartadas;
        return;
      }
      cola.emplace_back(std::move(linea));
    }
    cv.notify_one();
  }

private:
  void bucle()
  {
#ifdef __linux__
    // El hilo hereda la prioridad de tiempo real del hilo Motion: el disco no debe competir con el.
    sched_param p{};
    p.sched_priority = 0;
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &p);
#endif
    std::deque<std::string> lote;
    while(true)
    {
      {
        std::unique_lock<std::mutex> l(mutex);
        cv.wait(l, [this] { return terminar || !cola.empty(); });
        lote.swap(cola);
        if(lote.empty() && terminar)
          break;
      }
      for(const std::string& linea : lote)
        salida << linea << '\n';
      if(descartadas)
        salida << "# lineas descartadas por cola llena: " << descartadas.exchange(0) << '\n';
      salida.flush(); // por lote: si el robot se apaga al caer, lo escrito queda en el archivo
      lote.clear();
    }
    salida.close();
  }

  std::ofstream salida;
  std::string ruta;
  bool estaAbierto = false;
  std::thread hilo;
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<std::string> cola;
  bool terminar = false;
  std::atomic<unsigned> descartadas{0};
};
