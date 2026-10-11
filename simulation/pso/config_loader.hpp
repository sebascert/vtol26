#ifndef CONFIG_LOADER_HPP
#define CONFIG_LOADER_HPP

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class CargadorConfiguracion {
public:
    struct Configuracion {
        double rho = 1.225;
        double g = 9.81;
        double masa = 12.0;
        std::vector<double> inercia = {0.25, 0.28, 0.35};

        double area_ala = 0.28;
        double cl0_ala = 0.28;
        double cl_alpha_ala = 4.6;
        double cd0_ala = 0.028;
        double k_ala = 0.06;

        double area_cola = 0.05;
        double cl_gamma_cola = 0.52;
        double cd_gamma_cola = 0.016;
        double angulo_diedro_cola = 0.785;

        double area_ruddervator = 0.02;
        double cl_ruddervator = 0.28;
        double cd_ruddervator = 0.012;
        double angulo_diedro_ruddervator = 0.785;

        double area_aleron = 0.03;
        double cl_beta_aleron = 0.32;
        double cd_beta_aleron = 0.015;

        double coeficiente_empuje = 2.4e-4;
        double velocidad_aire_referencia = 20.0;

        double escala_omega = 1000.0;
        double escala_angulo = 1.0;
        double k_w = 1.0;
        double k_epsilon = 1.0;
        double k_gamma = 1.0;
        double k_beta = 1.0;
        double k_dinamica = 1.0;
        double k_pos = 1.0;
        double k_orientacion = 1.0;
        double dt = 0.1;

        double w_max = 0.9;
        double w_min = 0.4;
        double c1 = 2.0;
        double c2 = 2.0;
        int num_particulas = 40;
        int max_iteraciones = 120;
        double k_am = 5.0;
        int ciclos_externos = 20;
        double contraccion_k_am = 0.6;
        double k_am_minimo = 0.02;
        double umbral_convergencia = 1e-4;

        std::vector<double> posicion_objetivo = {0.0, 0.0, 0.0};
        std::vector<double> orientacion_objetivo = {0.0, 0.0, 0.0};
        bool tiene_pose_objetivo = false;
    };

    static Configuracion cargarDesdeJson(const std::string& rutaArchivo) {
        std::ifstream archivo(rutaArchivo);
        if (!archivo.is_open()) {
            throw std::runtime_error("No se pudo abrir " + rutaArchivo);
        }

        std::stringstream flujo;
        flujo << archivo.rdbuf();
        std::string textoJson = flujo.str();

        Configuracion configuracion;
        analizarJson(textoJson, configuracion);
        return configuracion;
    }

private:
    static void analizarJson(const std::string& json, Configuracion& configuracion) {
        configuracion.rho = extraerDouble(json, "\"rho\"", "physics", configuracion.rho);
        configuracion.g = extraerDouble(json, "\"g\"", "physics", configuracion.g);
        configuracion.masa = extraerDouble(json, "\"mass\"", "physics", configuracion.masa);
        configuracion.inercia = extraerArregloDouble(json, "\"inertia\"", "physics", configuracion.inercia);

        configuracion.area_ala = extraerDouble(json, "\"area\"", "wing", configuracion.area_ala);
        configuracion.cl0_ala = extraerDouble(json, "\"cl0\"", "wing", configuracion.cl0_ala);
        configuracion.cl_alpha_ala = extraerDouble(json, "\"cl_alpha\"", "wing", configuracion.cl_alpha_ala);
        configuracion.cd0_ala = extraerDouble(json, "\"cd0\"", "wing", configuracion.cd0_ala);
        configuracion.k_ala = extraerDouble(json, "\"k\"", "wing", configuracion.k_ala);

        configuracion.area_cola = extraerDouble(json, "\"area\"", "tail", configuracion.area_cola);
        configuracion.cl_gamma_cola = extraerDouble(json, "\"cl_gamma\"", "tail", configuracion.cl_gamma_cola);
        configuracion.cd_gamma_cola = extraerDouble(json, "\"cd_gamma\"", "tail", configuracion.cd_gamma_cola);
        configuracion.angulo_diedro_cola = extraerDouble(json, "\"cant_angle\"", "tail", configuracion.angulo_diedro_cola);

        configuracion.area_ruddervator = extraerDouble(json, "\"area\"", "ruddervator", configuracion.area_ruddervator);
        configuracion.cl_ruddervator = extraerDouble(json, "\"cl\"", "ruddervator", configuracion.cl_ruddervator);
        configuracion.cd_ruddervator = extraerDouble(json, "\"cd\"", "ruddervator", configuracion.cd_ruddervator);
        configuracion.angulo_diedro_ruddervator = extraerDouble(json, "\"cant_angle\"", "ruddervator", configuracion.angulo_diedro_ruddervator);

        configuracion.area_aleron = extraerDouble(json, "\"area\"", "aileron", configuracion.area_aleron);
        configuracion.cl_beta_aleron = extraerDouble(json, "\"cl_beta\"", "aileron", configuracion.cl_beta_aleron);
        configuracion.cd_beta_aleron = extraerDouble(json, "\"cd_beta\"", "aileron", configuracion.cd_beta_aleron);

        configuracion.coeficiente_empuje = extraerDouble(json, "\"thrust_coefficient\"", "propulsion", configuracion.coeficiente_empuje);
        configuracion.velocidad_aire_referencia = extraerDouble(json, "\"reference_airspeed\"", "propulsion", configuracion.velocidad_aire_referencia);

        configuracion.escala_omega = extraerDouble(json, "\"omega_scale\"", "cost_function", configuracion.escala_omega);
        configuracion.escala_angulo = extraerDouble(json, "\"angle_scale\"", "cost_function", configuracion.escala_angulo);
        configuracion.k_w = extraerDouble(json, "\"k_w\"", "cost_function", configuracion.k_w);
        configuracion.k_epsilon = extraerDouble(json, "\"k_epsilon\"", "cost_function", configuracion.k_epsilon);
        configuracion.k_gamma = extraerDouble(json, "\"k_gamma\"", "cost_function", configuracion.k_gamma);
        configuracion.k_beta = extraerDouble(json, "\"k_beta\"", "cost_function", configuracion.k_beta);
        configuracion.k_dinamica = extraerDouble(json, "\"k_dynamics\"", "cost_function", configuracion.k_dinamica);
        configuracion.k_pos = extraerDouble(json, "\"k_pos\"", "cost_function", configuracion.k_pos);
        configuracion.k_orientacion = extraerDouble(json, "\"k_att\"", "cost_function", configuracion.k_orientacion);
        configuracion.dt = extraerDouble(json, "\"dt\"", "cost_function", configuracion.dt);

        configuracion.w_max = extraerDouble(json, "\"w_max\"", "pso", configuracion.w_max);
        configuracion.w_min = extraerDouble(json, "\"w_min\"", "pso", configuracion.w_min);
        configuracion.c1 = extraerDouble(json, "\"c1\"", "pso", configuracion.c1);
        configuracion.c2 = extraerDouble(json, "\"c2\"", "pso", configuracion.c2);
        configuracion.num_particulas = static_cast<int>(extraerDouble(json, "\"num_particles\"", "pso", static_cast<double>(configuracion.num_particulas)));
        configuracion.max_iteraciones = static_cast<int>(extraerDouble(json, "\"max_iterations\"", "pso", static_cast<double>(configuracion.max_iteraciones)));
        configuracion.k_am = extraerDouble(json, "\"k_am\"", "pso", configuracion.k_am);
        configuracion.ciclos_externos = static_cast<int>(extraerDouble(json, "\"external_cycles\"", "pso", static_cast<double>(configuracion.ciclos_externos)));
        configuracion.contraccion_k_am = extraerDouble(json, "\"k_am_contraction\"", "pso", configuracion.contraccion_k_am);
        configuracion.k_am_minimo = extraerDouble(json, "\"k_am_min\"", "pso", configuracion.k_am_minimo);
        configuracion.umbral_convergencia = extraerDouble(json, "\"convergence_threshold\"", "pso", configuracion.umbral_convergencia);

        configuracion.tiene_pose_objetivo = tieneObjeto(json, "target_pose");
        configuracion.posicion_objetivo = extraerArregloDouble(json, "\"position\"", "target_pose", configuracion.posicion_objetivo);
        configuracion.orientacion_objetivo = extraerArregloDouble(json, "\"orientation\"", "target_pose", configuracion.orientacion_objetivo);
        if (configuracion.posicion_objetivo.size() != 3 || configuracion.orientacion_objetivo.size() != 3) {
            configuracion.posicion_objetivo = {0.0, 0.0, 0.0};
            configuracion.orientacion_objetivo = {0.0, 0.0, 0.0};
            configuracion.tiene_pose_objetivo = false;
        }
    }

    static bool tieneObjeto(const std::string& json, const std::string& nombreObjeto) {
        const std::string clave = "\"" + nombreObjeto + "\"";
        return json.find(clave) != std::string::npos;
    }

    static std::string extraerObjeto(const std::string& json, const std::string& contexto) {
        if (contexto.empty()) {
            return json;
        }

        const std::string claveContexto = "\"" + contexto + "\"";
        size_t posicionContexto = json.find(claveContexto);
        if (posicionContexto == std::string::npos) {
            return json;
        }

        size_t llaveAbierta = json.find('{', posicionContexto);
        if (llaveAbierta == std::string::npos) {
            return json;
        }

        int profundidad = 0;
        for (size_t i = llaveAbierta; i < json.size(); ++i) {
            if (json[i] == '{') {
                ++profundidad;
            } else if (json[i] == '}') {
                --profundidad;
                if (profundidad == 0) {
                    return json.substr(llaveAbierta, i - llaveAbierta + 1);
                }
            }
        }
        return json;
    }

    static double extraerDouble(const std::string& json, const std::string& clave, const std::string& contexto, double valorDefecto) {
        const std::string ambito = extraerObjeto(json, contexto);
        size_t pos = ambito.find(clave);
        if (pos == std::string::npos) {
            return valorDefecto;
        }

        pos = ambito.find(':', pos);
        if (pos == std::string::npos) {
            return valorDefecto;
        }

        size_t inicio = pos + 1;
        while (inicio < ambito.size() && std::isspace(static_cast<unsigned char>(ambito[inicio]))) {
            ++inicio;
        }

        size_t fin = inicio;
        while (fin < ambito.size()) {
            const char c = ambito[fin];
            if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') {
                ++fin;
            } else {
                break;
            }
        }

        if (fin == inicio) {
            return valorDefecto;
        }

        try {
            return std::stod(ambito.substr(inicio, fin - inicio));
        } catch (...) {
            return valorDefecto;
        }
    }

    static std::vector<double> extraerArregloDouble(const std::string& json,
                                                     const std::string& clave,
                                                     const std::string& contexto,
                                                     const std::vector<double>& valorDefecto) {
        const std::string ambito = extraerObjeto(json, contexto);
        size_t pos = ambito.find(clave);
        if (pos == std::string::npos) {
            return valorDefecto;
        }

        size_t corcheteAbierto = ambito.find('[', pos);
        if (corcheteAbierto == std::string::npos) {
            return valorDefecto;
        }

        int profundidad = 0;
        size_t corcheteCerrado = std::string::npos;
        for (size_t i = corcheteAbierto; i < ambito.size(); ++i) {
            if (ambito[i] == '[') {
                ++profundidad;
            } else if (ambito[i] == ']') {
                --profundidad;
                if (profundidad == 0) {
                    corcheteCerrado = i;
                    break;
                }
            }
        }

        if (corcheteCerrado == std::string::npos || corcheteCerrado <= corcheteAbierto + 1) {
            return valorDefecto;
        }

        std::vector<double> resultado;
        std::stringstream flujo(ambito.substr(corcheteAbierto + 1, corcheteCerrado - corcheteAbierto - 1));
        std::string numero;
        while (std::getline(flujo, numero, ',')) {
            size_t inicio = numero.find_first_not_of(" \t\n\r");
            size_t fin = numero.find_last_not_of(" \t\n\r");
            if (inicio == std::string::npos) {
                continue;
            }
            try {
                resultado.push_back(std::stod(numero.substr(inicio, fin - inicio + 1)));
            } catch (...) {
                return valorDefecto;
            }
        }

        return resultado.empty() ? valorDefecto : resultado;
    }
};

#endif
