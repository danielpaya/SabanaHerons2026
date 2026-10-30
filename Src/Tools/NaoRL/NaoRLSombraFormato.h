/**
 * @file NaoRLSombraFormato.h
 *
 * Formato de naoRLsombra_<n>.csv (lo lee demo_carril/rl/analizar_sombra.py). Sin B-Human, para poder probarlo en PC.
 * Una linea por decision. Las lineas que empiezan por '#' son comentarios (el analizador las ignora).
 */

#pragma once

#include "NaoRLPolitica.h" // misma carpeta: asi tambien compila en Tests, que no tiene Src/ en la ruta
#include <array>
#include <cstdio>
#include <string>

namespace NaoRL
{
  constexpr std::size_t nPiernasRegistro = 12;
  constexpr float umbralZ = 4.f; /**< Columna n_z4. */

  struct FilaSombra
  {
    unsigned tMs = 0;
    const char* estadoBHuman = "";
    float tMarchaS = 0.f;
    float tInferenciaUs = 0.f;
    const char* fallDown = "";
    float inclinacion = 0.f;
    std::array<float, 3> gyro{}, arriba{};
    float psiObs = 0.f, dObs = 0.f;
    unsigned edadCamaraMs = 0;
    float zmax = 0.f;
    int zmaxIdx = 0;
    int nZ4 = 0;
    Accion a{};       /**< Accion cruda. */
    Accion objPol{};  /**< Objetivo antes del limitador (ya decodificado: postura + a * escala, recortado a lo/hi). */
    Accion objLim{};  /**< Objetivo tras los 2 ciclos de 12 ms del limitador simulado. */
    int nRecorteVel = 0; /**< Motores que recorto el limite de velocidad en esta decision. */
    int nRecorteLim = 0; /**< Motores recortados por lo/hi o por el margen a jointLimits en esta decision. */
    Accion bh{};      /**< Lo que manda B-Human en JointRequest. */
    Accion ang{};     /**< Medido, en crudo. */
    std::array<int, nPiernasRegistro> corr{}, temp{};
  };

  /** @param piernas Nombres de las 12 articulaciones de pierna (estilo de B-Human, como en naoRL_<n>.csv). */
  inline std::string cabeceraSombra(const std::array<const char*, nPiernasRegistro>& piernas)
  {
    std::string c = "t_ms,estado_bhuman,t_marcha_s,t_inferencia_us,fallDown,inclinacion,gyro_x,gyro_y,gyro_z,arriba_x,arriba_y,"
                    "arriba_z,psi_obs,d_obs,edad_camara_ms,zmax,zmax_idx,n_z4";
    const auto porMotor = [&c](const char* prefijo)
    {
      for(const char* motor : nombresMotores)
        c += std::string(",") + prefijo + motor;
    };
    porMotor("a_");
    porMotor("obj_pol_");
    porMotor("obj_lim_");
    c += ",n_recorte_vel,n_recorte_lim";
    porMotor("bh_");
    porMotor("ang_");
    for(const char* pierna : piernas)
      c += std::string(",corr_") + pierna + ",temp_" + pierna;
    return c;
  }

  inline std::string lineaSombra(const FilaSombra& f)
  {
    char buf[256];
    std::string linea;
    linea.reserve(2200);
    std::snprintf(buf, sizeof(buf), "%u,%s,%.3f,%.0f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%.3f,%d,%d", f.tMs, f.estadoBHuman,
                  f.tMarchaS, f.tInferenciaUs, f.fallDown, f.inclinacion, f.gyro[0], f.gyro[1], f.gyro[2], f.arriba[0], f.arriba[1],
                  f.arriba[2], f.psiObs, f.dObs, f.edadCamaraMs, f.zmax, f.zmaxIdx, f.nZ4);
    linea += buf;
    const auto motores = [&](const Accion& valores)
    {
      for(const float v : valores)
      {
        std::snprintf(buf, sizeof(buf), ",%.4f", v);
        linea += buf;
      }
    };
    motores(f.a);
    motores(f.objPol);
    motores(f.objLim);
    std::snprintf(buf, sizeof(buf), ",%d,%d", f.nRecorteVel, f.nRecorteLim);
    linea += buf;
    motores(f.bh);
    motores(f.ang);
    for(std::size_t k = 0; k < nPiernasRegistro; ++k)
    {
      std::snprintf(buf, sizeof(buf), ",%d,%d", f.corr[k], f.temp[k]);
      linea += buf;
    }
    return linea;
  }
}
