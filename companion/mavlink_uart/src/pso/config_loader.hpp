#ifndef CONFIG_LOADER_HPP
#define CONFIG_LOADER_HPP

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * Parser JSON minimalista para config.json.
 * Está pensado para estructuras simples (números, arrays y objetos anidados).
 */
class ConfigLoader {
public:
    struct Config {
        // Physics
        double rho = 1.225;
        double g = 9.81;
        double mass = 12.0;
        std::vector<double> inertia = {0.25, 0.28, 0.35};

        // Wing
        double wing_area = 0.28;
        double wing_cl0 = 0.28;
        double wing_cl_alpha = 4.6;
        double wing_cd0 = 0.028;
        double wing_k = 0.06;

        // Tail
        double tail_area = 0.05;
        double tail_cl_gamma = 0.52;
        double tail_cd_gamma = 0.016;
        double tail_cant_angle = 0.785;

        // Ruddervator
        double rud_area = 0.02;
        double rud_cl = 0.28;
        double rud_cd = 0.012;
        double rud_cant_angle = 0.785;

        // Aileron
        double ail_area = 0.03;
        double ail_cl_beta = 0.32;
        double ail_cd_beta = 0.015;

        // Propulsion
        double thrust_coeff = 2.4e-4;
        double ref_airspeed = 20.0;

        // Cost function
        double omega_scale = 1000.0;
        double angle_scale = 1.0;
        double k_w = 1.0;
        double k_epsilon = 1.0;
        double k_gamma = 1.0;
        double k_beta = 1.0;
        double k_dynamics = 1.0;
        double k_pos = 1.0;
        double k_att = 1.0;
        double dt = 0.1;

        // PSO
        double w_max = 0.9;
        double w_min = 0.4;
        double c1 = 2.0;
        double c2 = 2.0;
        int num_particles = 40;
        int max_iterations = 120;
        double k_am = 5.0;
        int external_cycles = 20;
        double k_am_contraction = 0.6;
        double k_am_min = 0.02;
        double convergence_threshold = 1e-4;

        // Target
        std::vector<double> target_pose_position = {0.0, 0.0, 0.0};
        std::vector<double> target_pose_orientation = {0.0, 0.0, 0.0};
        bool has_target_pose = false;
    };

    static Config loadFromJson(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("No se pudo abrir " + filepath);
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string json_str = buffer.str();

        Config cfg;
        parseJson(json_str, cfg);
        return cfg;
    }

private:
    static void parseJson(const std::string& json, Config& cfg) {
        // Physics
        cfg.rho = extractDouble(json, "\"rho\"", "physics", cfg.rho);
        cfg.g = extractDouble(json, "\"g\"", "physics", cfg.g);
        cfg.mass = extractDouble(json, "\"mass\"", "physics", cfg.mass);
        cfg.inertia = extractDoubleArray(json, "\"inertia\"", "physics", cfg.inertia);

        // Wing
        cfg.wing_area = extractDouble(json, "\"area\"", "wing", cfg.wing_area);
        cfg.wing_cl0 = extractDouble(json, "\"cl0\"", "wing", cfg.wing_cl0);
        cfg.wing_cl_alpha = extractDouble(json, "\"cl_alpha\"", "wing", cfg.wing_cl_alpha);
        cfg.wing_cd0 = extractDouble(json, "\"cd0\"", "wing", cfg.wing_cd0);
        cfg.wing_k = extractDouble(json, "\"k\"", "wing", cfg.wing_k);

        // Tail
        cfg.tail_area = extractDouble(json, "\"area\"", "tail", cfg.tail_area);
        cfg.tail_cl_gamma = extractDouble(json, "\"cl_gamma\"", "tail", cfg.tail_cl_gamma);
        cfg.tail_cd_gamma = extractDouble(json, "\"cd_gamma\"", "tail", cfg.tail_cd_gamma);
        cfg.tail_cant_angle = extractDouble(json, "\"cant_angle\"", "tail", cfg.tail_cant_angle);

        // Ruddervator
        cfg.rud_area = extractDouble(json, "\"area\"", "ruddervator", cfg.rud_area);
        cfg.rud_cl = extractDouble(json, "\"cl\"", "ruddervator", cfg.rud_cl);
        cfg.rud_cd = extractDouble(json, "\"cd\"", "ruddervator", cfg.rud_cd);
        cfg.rud_cant_angle = extractDouble(json, "\"cant_angle\"", "ruddervator", cfg.rud_cant_angle);

        // Aileron
        cfg.ail_area = extractDouble(json, "\"area\"", "aileron", cfg.ail_area);
        cfg.ail_cl_beta = extractDouble(json, "\"cl_beta\"", "aileron", cfg.ail_cl_beta);
        cfg.ail_cd_beta = extractDouble(json, "\"cd_beta\"", "aileron", cfg.ail_cd_beta);

        // Propulsion
        cfg.thrust_coeff = extractDouble(json, "\"thrust_coefficient\"", "propulsion", cfg.thrust_coeff);
        cfg.ref_airspeed = extractDouble(json, "\"reference_airspeed\"", "propulsion", cfg.ref_airspeed);

        // Cost function
        cfg.omega_scale = extractDouble(json, "\"omega_scale\"", "cost_function", cfg.omega_scale);
        cfg.angle_scale = extractDouble(json, "\"angle_scale\"", "cost_function", cfg.angle_scale);
        cfg.k_w = extractDouble(json, "\"k_w\"", "cost_function", cfg.k_w);
        cfg.k_epsilon = extractDouble(json, "\"k_epsilon\"", "cost_function", cfg.k_epsilon);
        cfg.k_gamma = extractDouble(json, "\"k_gamma\"", "cost_function", cfg.k_gamma);
        cfg.k_beta = extractDouble(json, "\"k_beta\"", "cost_function", cfg.k_beta);
        cfg.k_dynamics = extractDouble(json, "\"k_dynamics\"", "cost_function", cfg.k_dynamics);
        cfg.k_pos = extractDouble(json, "\"k_pos\"", "cost_function", cfg.k_pos);
        cfg.k_att = extractDouble(json, "\"k_att\"", "cost_function", cfg.k_att);
        cfg.dt = extractDouble(json, "\"dt\"", "cost_function", cfg.dt);

        // PSO
        cfg.w_max = extractDouble(json, "\"w_max\"", "pso", cfg.w_max);
        cfg.w_min = extractDouble(json, "\"w_min\"", "pso", cfg.w_min);
        cfg.c1 = extractDouble(json, "\"c1\"", "pso", cfg.c1);
        cfg.c2 = extractDouble(json, "\"c2\"", "pso", cfg.c2);
        cfg.num_particles = static_cast<int>(extractDouble(json, "\"num_particles\"", "pso", static_cast<double>(cfg.num_particles)));
        cfg.max_iterations = static_cast<int>(extractDouble(json, "\"max_iterations\"", "pso", static_cast<double>(cfg.max_iterations)));
        cfg.k_am = extractDouble(json, "\"k_am\"", "pso", cfg.k_am);
        cfg.external_cycles = static_cast<int>(extractDouble(json, "\"external_cycles\"", "pso", static_cast<double>(cfg.external_cycles)));
        cfg.k_am_contraction = extractDouble(json, "\"k_am_contraction\"", "pso", cfg.k_am_contraction);
        cfg.k_am_min = extractDouble(json, "\"k_am_min\"", "pso", cfg.k_am_min);
        cfg.convergence_threshold = extractDouble(json, "\"convergence_threshold\"", "pso", cfg.convergence_threshold);

        cfg.has_target_pose = hasObject(json, "target_pose");
        cfg.target_pose_position = extractDoubleArray(json, "\"position\"", "target_pose", cfg.target_pose_position);
        cfg.target_pose_orientation = extractDoubleArray(json, "\"orientation\"", "target_pose", cfg.target_pose_orientation);
        if (cfg.target_pose_position.size() != 3 || cfg.target_pose_orientation.size() != 3) {
            cfg.target_pose_position = {0.0, 0.0, 0.0};
            cfg.target_pose_orientation = {0.0, 0.0, 0.0};
            cfg.has_target_pose = false;
        }
    }

    static bool hasObject(const std::string& json, const std::string& object_name) {
        const std::string key = "\"" + object_name + "\"";
        return json.find(key) != std::string::npos;
    }

    static std::string extractObject(const std::string& json, const std::string& context) {
        if (context.empty()) {
            return json;
        }

        const std::string context_key = "\"" + context + "\"";
        size_t context_pos = json.find(context_key);
        if (context_pos == std::string::npos) {
            return json;
        }

        size_t open_brace = json.find('{', context_pos);
        if (open_brace == std::string::npos) {
            return json;
        }

        int depth = 0;
        for (size_t i = open_brace; i < json.size(); ++i) {
            if (json[i] == '{') {
                ++depth;
            } else if (json[i] == '}') {
                --depth;
                if (depth == 0) {
                    return json.substr(open_brace, i - open_brace + 1);
                }
            }
        }
        return json;
    }

    static double extractDouble(const std::string& json, const std::string& key, const std::string& context, double fallback) {
        const std::string scope = extractObject(json, context);
        size_t pos = scope.find(key);
        if (pos == std::string::npos) {
            return fallback;
        }

        pos = scope.find(':', pos);
        if (pos == std::string::npos) {
            return fallback;
        }

        size_t start = pos + 1;
        while (start < scope.size() && std::isspace(static_cast<unsigned char>(scope[start]))) {
            ++start;
        }

        size_t end = start;
        while (end < scope.size()) {
            const char c = scope[end];
            if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') {
                ++end;
            } else {
                break;
            }
        }

        if (end == start) {
            return fallback;
        }

        try {
            return std::stod(scope.substr(start, end - start));
        } catch (...) {
            return fallback;
        }
    }

    static std::vector<double> extractDoubleArray(const std::string& json,
                                                  const std::string& key,
                                                  const std::string& context,
                                                  const std::vector<double>& fallback) {
        const std::string scope = extractObject(json, context);
        size_t pos = scope.find(key);
        if (pos == std::string::npos) {
            return fallback;
        }

        size_t open_bracket = scope.find('[', pos);
        if (open_bracket == std::string::npos) {
            return fallback;
        }

        int depth = 0;
        size_t close_bracket = std::string::npos;
        for (size_t i = open_bracket; i < scope.size(); ++i) {
            if (scope[i] == '[') {
                ++depth;
            } else if (scope[i] == ']') {
                --depth;
                if (depth == 0) {
                    close_bracket = i;
                    break;
                }
            }
        }

        if (close_bracket == std::string::npos || close_bracket <= open_bracket + 1) {
            return fallback;
        }

        std::vector<double> result;
        std::stringstream ss(scope.substr(open_bracket + 1, close_bracket - open_bracket - 1));
        std::string num;
        while (std::getline(ss, num, ',')) {
            size_t start = num.find_first_not_of(" \t\n\r");
            size_t end = num.find_last_not_of(" \t\n\r");
            if (start == std::string::npos) {
                continue;
            }
            try {
                result.push_back(std::stod(num.substr(start, end - start + 1)));
            } catch (...) {
                return fallback;
            }
        }

        return result.empty() ? fallback : result;
    }
};

#endif // CONFIG_LOADER_HPP