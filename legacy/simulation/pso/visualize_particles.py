#!/usr/bin/env python3

import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

plt.style.use("dark_background")


class VisualizadorTiempoReal:
    def __init__(self, ruta_config="config.json", ruta_json="pso_data.json"):
        self.ruta_config = Path(ruta_config)
        self.ruta_json = Path(ruta_json)
        self.configuracion = self._cargar_configuracion()
        if ruta_json == "pso_data.json":
            self.ruta_json = Path(self.configuracion["visualization"].get("json_path", ruta_json))

        self.dimensiones = self.configuracion["visualization"].get("dims", [0, 1, 2])
        self.etiquetas = self.configuracion["visualization"].get("labels", ["ω_I", "ω_D", "ω_T"])
        self.etiquetas_seleccionadas = [self.etiquetas[i] for i in self.dimensiones]
        self.limites_ejes = self.configuracion["visualization"].get("axis_limits", [[-10, 2000], [-10, 2000], [-10, 2000]])
        self.intervalo_ms = self.configuracion["visualization"].get("interval_ms", 120)
        self.mostrar_desde_uno = bool(self.configuracion["visualization"].get("display_iteration_one_based", True))

        self.zoom_dinamico = self.configuracion["visualization"].get("dynamic_zoom", True)
        self.suavizado_zoom = float(self.configuracion["visualization"].get("zoom_smooth", 0.12))
        self.margen_zoom = float(self.configuracion["visualization"].get("zoom_padding", 0.12))
        self.rango_minimo = float(self.configuracion["visualization"].get("min_span", 1.0))

        colores = self.configuracion["visualization"].get("colors", {})
        tamanos = self.configuracion["visualization"].get("sizes", {})
        self.color_particulas = colores.get("particles", "lime")
        self.color_mejor_global = colores.get("gbest", "gold")
        self.color_objetivo = colores.get("target", "white")
        self.color_centro = colores.get("center", "magenta")
        self.color_pose_objetivo = colores.get("target_pose", "cyan")
        self.color_pose_predicha = colores.get("predicted_gbest_pose", "red")
        self.tamano_particulas = tamanos.get("particles", 34)
        self.tamano_mejor_global = tamanos.get("gbest", 220)
        self.tamano_objetivo = tamanos.get("target", 300)
        self.tamano_centro = tamanos.get("center", 90)
        self.tamano_pose_objetivo = tamanos.get("target_pose", 350)
        self.tamano_pose_predicha = tamanos.get("predicted_gbest_pose", 280)

        self.objetivo_actuacion = None
        estado_inicial_config = self.configuracion["visualization"].get("initial_state", None)
        self.estado_inicial = np.array(estado_inicial_config, dtype=float) if estado_inicial_config is not None else None

        self._ultima_data = None

        self.fig = plt.figure(figsize=(13, 6))
        self.ax3d = self.fig.add_subplot(1, 2, 1, projection="3d")
        self.ax2d = self.fig.add_subplot(1, 2, 2)

        self.ax_pose = self.fig.add_axes([0.03, 0.58, 0.24, 0.32], projection="3d")
        self.ax_pose.set_title("Pose (mundo)", color="0.9", fontsize=9)
        self.ax_pose.set_facecolor("black")
        self.ax_pose.tick_params(colors="0.82", labelsize=7)

        self._configurar_3d()
        self._configurar_2d()

        self.xlim_actual = self.ax3d.get_xlim()
        self.ylim_actual = self.ax3d.get_ylim()
        self.zlim_actual = self.ax3d.get_zlim()

        self.scatter_particulas = self.ax3d.scatter([], [], [], c=self.color_particulas, alpha=0.6, s=self.tamano_particulas, linewidths=0, label="Partículas")
        self.scatter_mejor_global = self.ax3d.scatter([], [], [], c=self.color_mejor_global, alpha=1.0, s=self.tamano_mejor_global, marker="*", linewidths=0, label="Mejor global")
        self.scatter_objetivo = self.ax3d.scatter([], [], [], c=self.color_objetivo, alpha=0.9, s=self.tamano_objetivo, marker="x", linewidths=2, label="Objetivo")
        self.scatter_centro = self.ax3d.scatter([], [], [], c=self.color_centro, alpha=0.95, s=self.tamano_centro, marker="D", linewidths=0, label="Centro búsqueda")
        self.scatter_pose_objetivo = self.ax_pose.scatter([], [], [], c=self.color_pose_objetivo, alpha=1.0, s=self.tamano_pose_objetivo, marker="s", linewidths=1.5, label="Pose objetivo (mundo)")
        self.scatter_pose_predicha = self.ax_pose.scatter([], [], [], c=self.color_pose_predicha, alpha=1.0, s=self.tamano_pose_predicha, marker="^", linewidths=0, label="Pose mejor global (mundo)")
        (self.linea_trayectoria_centro,) = self.ax3d.plot([], [], [], color=self.color_centro, linewidth=1.2, alpha=0.65)

        if self.estado_inicial is not None and self.estado_inicial.size > max(self.dimensiones):
            x_inicial = float(self.estado_inicial[self.dimensiones[0]])
            y_inicial = float(self.estado_inicial[self.dimensiones[1]])
            z_inicial = float(self.estado_inicial[self.dimensiones[2]])
            color_inicial = self.configuracion["visualization"].get("initial_color", "white")
            tamano_inicial = self.configuracion["visualization"].get("initial_size", 120)
            self.scatter_inicial = self.ax3d.scatter([x_inicial], [y_inicial], [z_inicial], c=color_inicial, marker="o", s=tamano_inicial, alpha=0.95, linewidths=0, label="Inicio")
        else:
            self.scatter_inicial = self.ax3d.scatter([], [], [], c="white", marker="o", s=0, alpha=0)

        (self.linea_aptitud,) = self.ax2d.plot([], [], color="cyan", linewidth=1.7, alpha=0.9)
        self.texto_distancia = self.ax2d.text(
            0.02, 0.98, "", transform=self.ax2d.transAxes,
            ha="left", va="top", color="0.9", fontsize=8
        )

        self.ax3d.legend(facecolor="black", edgecolor="white", labelcolor="white", fontsize=8)
        try:
            self.ax_pose.legend(facecolor="black", edgecolor="white", labelcolor="white", fontsize=7)
        except Exception:
            pass

        self.anim = FuncAnimation(
            self.fig, self._actualizar_frame,
            interval=self.intervalo_ms, blit=False, cache_frame_data=False,
        )

    def _cargar_configuracion(self):
        try:
            with self.ruta_config.open("r") as f:
                return json.load(f)
        except FileNotFoundError:
            print("⚠ config.json no encontrado, usando valores por defecto")
            return self._configuracion_predeterminada()
        except json.JSONDecodeError:
            print("⚠ Error al parsear config.json")
            return self._configuracion_predeterminada()

    @staticmethod
    def _configuracion_predeterminada():
        return {
            "visualization": {
                "dims": [0, 1, 2],
                "axis_limits": [[-10, 2000], [-10, 2000], [-10, 2000]],
                "interval_ms": 120,
                "colors": {
                    "particles": "lime",
                    "gbest": "gold",
                    "target": "white",
                    "center": "magenta",
                    "target_pose": "cyan",
                    "predicted_gbest_pose": "red",
                },
                "sizes": {
                    "particles": 34,
                    "gbest": 220,
                    "target": 300,
                    "center": 90,
                    "target_pose": 350,
                    "predicted_gbest_pose": 280,
                },
                "labels": ["ω_I", "ω_D", "ω_T", "ε_I", "ε_D", "γ_I", "γ_D", "β_I", "β_D"],
            }
        }

    def _configurar_3d(self):
        self.ax3d.set_facecolor("black")
        self.ax3d.xaxis.set_pane_color((0, 0, 0, 0))
        self.ax3d.yaxis.set_pane_color((0, 0, 0, 0))
        self.ax3d.zaxis.set_pane_color((0, 0, 0, 0))
        self.ax3d.grid(True, alpha=0.08)
        self.ax3d.tick_params(colors="0.82", labelsize=8)

        xlim, ylim, zlim = self.limites_ejes
        self.ax3d.set_xlim(xlim[0], xlim[1])
        self.ax3d.set_ylim(ylim[0], ylim[1])
        self.ax3d.set_zlim(zlim[0], zlim[1])

        self.ax3d.set_xlabel(self.etiquetas_seleccionadas[0], color="0.9", fontsize=9)
        self.ax3d.set_ylabel(self.etiquetas_seleccionadas[1], color="0.9", fontsize=9)
        self.ax3d.set_zlabel(self.etiquetas_seleccionadas[2], color="0.9", fontsize=9)
        self.ax3d.set_title("Enjambre PSO (3D)", color="0.9", fontsize=10)

    def _configurar_2d(self):
        self.ax2d.set_facecolor("black")
        self.ax2d.grid(True, alpha=0.08)
        self.ax2d.tick_params(colors="0.82", labelsize=8)
        self.ax2d.set_title("Convergencia de la aptitud", color="0.9", fontsize=10)
        self.ax2d.set_xlabel("Iteración", color="0.9", fontsize=9)
        self.ax2d.set_ylabel("Aptitud del mejor global (log)", color="0.9", fontsize=9)
        self.ax2d.set_yscale("log")

    def _cargar_json_seguro(self):
        try:
            with self.ruta_json.open("r") as f:
                datos = json.load(f)
            self._ultima_data = datos
            return datos
        except FileNotFoundError:
            return self._ultima_data
        except json.JSONDecodeError:
            return self._ultima_data

    def _actualizar_frame(self, _):
        datos = self._cargar_json_seguro()
        if not datos:
            return (
                self.scatter_particulas, self.scatter_mejor_global, self.scatter_objetivo, self.scatter_centro,
                self.scatter_pose_objetivo, self.scatter_pose_predicha,
                self.linea_trayectoria_centro, self.linea_aptitud, self.texto_distancia,
            )

        lista_particulas = datos.get("particles", [])
        if not lista_particulas:
            return (
                self.scatter_particulas, self.scatter_mejor_global, self.scatter_objetivo, self.scatter_centro,
                self.scatter_pose_objetivo, self.scatter_pose_predicha,
                self.linea_trayectoria_centro, self.linea_aptitud, self.texto_distancia,
            )

        particulas = np.asarray(lista_particulas, dtype=float)
        if particulas.ndim != 2 or particulas.shape[1] <= max(self.dimensiones):
            return (
                self.scatter_particulas, self.scatter_mejor_global, self.scatter_objetivo, self.scatter_centro,
                self.scatter_pose_objetivo, self.scatter_pose_predicha,
                self.linea_trayectoria_centro, self.linea_aptitud, self.texto_distancia,
            )

        xs = particulas[:, self.dimensiones[0]]
        ys = particulas[:, self.dimensiones[1]]
        zs = particulas[:, self.dimensiones[2]]
        self.scatter_particulas.set_offsets(np.c_[xs, ys])
        self.scatter_particulas.set_3d_properties(zs, zdir="z")

        tiene_objetivo_actuacion = False
        xt = yt = zt = 0.0

        pose_objetivo = datos.get("target_pose", {})
        posicion_pose_objetivo = np.asarray(pose_objetivo.get("position", []), dtype=float)
        tiene_pose_objetivo = posicion_pose_objetivo.ndim == 1 and posicion_pose_objetivo.size == 3 and np.all(np.isfinite(posicion_pose_objetivo))

        pose_predicha_mejor_global = datos.get("predicted_gbest_pose", {})
        posicion_pose_predicha = np.asarray(pose_predicha_mejor_global.get("position", []), dtype=float)
        tiene_pose_predicha = posicion_pose_predicha.ndim == 1 and posicion_pose_predicha.size == 3 and np.all(np.isfinite(posicion_pose_predicha))

        historial_centros = np.asarray(datos.get("center_history", []), dtype=float)
        tiene_historial_centros = historial_centros.ndim == 2 and historial_centros.shape[1] > max(self.dimensiones) and historial_centros.shape[0] > 0
        if tiene_historial_centros:
            hist_x = historial_centros[:, self.dimensiones[0]]
            hist_y = historial_centros[:, self.dimensiones[1]]
            hist_z = historial_centros[:, self.dimensiones[2]]
            self.linea_trayectoria_centro.set_data(hist_x, hist_y)
            self.linea_trayectoria_centro.set_3d_properties(hist_z, zdir="z")
        else:
            self.linea_trayectoria_centro.set_data([], [])
            self.linea_trayectoria_centro.set_3d_properties([], zdir="z")

        centro_busqueda = datos.get("search_center", None)
        punto_centro = None
        if centro_busqueda is not None and len(centro_busqueda) > max(self.dimensiones):
            sc = np.asarray(centro_busqueda, dtype=float)
            punto_centro = (
                float(sc[self.dimensiones[0]]),
                float(sc[self.dimensiones[1]]),
                float(sc[self.dimensiones[2]]),
            )
            self.scatter_centro.set_offsets(np.c_[[punto_centro[0]], [punto_centro[1]]])
            self.scatter_centro.set_3d_properties([punto_centro[2]], zdir="z")
        else:
            self.scatter_centro.set_offsets(np.empty((0, 2)))
            self.scatter_centro.set_3d_properties([], zdir="z")

        if self.zoom_dinamico:
            minx, maxx = float(np.min(xs)), float(np.max(xs))
            miny, maxy = float(np.min(ys)), float(np.max(ys))
            minz, maxz = float(np.min(zs)), float(np.max(zs))

            mejor_global = datos.get("gbest", None)
            g_x = g_y = g_z = None
            if mejor_global and len(mejor_global) > max(self.dimensiones):
                mejor_global = np.asarray(mejor_global, dtype=float)
                g_x = float(mejor_global[self.dimensiones[0]])
                g_y = float(mejor_global[self.dimensiones[1]])
                g_z = float(mejor_global[self.dimensiones[2]])

            if g_x is not None:
                minx = min(minx, g_x)
                maxx = max(maxx, g_x)
                miny = min(miny, g_y)
                maxy = max(maxy, g_y)
                minz = min(minz, g_z)
                maxz = max(maxz, g_z)

            if tiene_objetivo_actuacion:
                minx = min(minx, xt)
                maxx = max(maxx, xt)
                miny = min(miny, yt)
                maxy = max(maxy, yt)
                minz = min(minz, zt)
                maxz = max(maxz, zt)

            if tiene_historial_centros:
                minx = min(minx, float(np.min(hist_x)))
                maxx = max(maxx, float(np.max(hist_x)))
                miny = min(miny, float(np.min(hist_y)))
                maxy = max(maxy, float(np.max(hist_y)))
                minz = min(minz, float(np.min(hist_z)))
                maxz = max(maxz, float(np.max(hist_z)))

            if punto_centro is not None:
                minx = min(minx, punto_centro[0])
                maxx = max(maxx, punto_centro[0])
                miny = min(miny, punto_centro[1])
                maxy = max(maxy, punto_centro[1])
                minz = min(minz, punto_centro[2])
                maxz = max(maxz, punto_centro[2])

            rango_x = max(maxx - minx, self.rango_minimo)
            rango_y = max(maxy - miny, self.rango_minimo)
            rango_z = max(maxz - minz, self.rango_minimo)

            margen_x = rango_x * self.margen_zoom
            margen_y = rango_y * self.margen_zoom
            margen_z = rango_z * self.margen_zoom

            xlim_objetivo = (minx - margen_x, maxx + margen_x)
            ylim_objetivo = (miny - margen_y, maxy + margen_y)
            zlim_objetivo = (minz - margen_z, maxz + margen_z)

            xlim_previo = self.ax3d.get_xlim()
            ylim_previo = self.ax3d.get_ylim()
            zlim_previo = self.ax3d.get_zlim()

            alpha = self.suavizado_zoom
            xlim_nuevo = (xlim_previo[0] + alpha * (xlim_objetivo[0] - xlim_previo[0]),
                        xlim_previo[1] + alpha * (xlim_objetivo[1] - xlim_previo[1]))
            ylim_nuevo = (ylim_previo[0] + alpha * (ylim_objetivo[0] - ylim_previo[0]),
                        ylim_previo[1] + alpha * (ylim_objetivo[1] - ylim_previo[1]))
            zlim_nuevo = (zlim_previo[0] + alpha * (zlim_objetivo[0] - zlim_previo[0]),
                        zlim_previo[1] + alpha * (zlim_objetivo[1] - zlim_previo[1]))

            self.ax3d.set_xlim(xlim_nuevo)
            self.ax3d.set_ylim(ylim_nuevo)
            self.ax3d.set_zlim(zlim_nuevo)

        mejor_global = datos.get("gbest", None)
        if mejor_global and len(mejor_global) > max(self.dimensiones):
            mejor_global = np.asarray(mejor_global, dtype=float)
            xg = mejor_global[self.dimensiones[0]]
            yg = mejor_global[self.dimensiones[1]]
            zg = mejor_global[self.dimensiones[2]]
            self.scatter_mejor_global.set_offsets(np.c_[[xg], [yg]])
            self.scatter_mejor_global.set_3d_properties([zg], zdir="z")

        self.scatter_objetivo.set_offsets(np.empty((0, 2)))
        self.scatter_objetivo.set_3d_properties([], zdir="z")

        if tiene_pose_objetivo:
            self.scatter_pose_objetivo.set_offsets(np.c_[[float(posicion_pose_objetivo[0])], [float(posicion_pose_objetivo[1])]])
            self.scatter_pose_objetivo.set_3d_properties([float(posicion_pose_objetivo[2])], zdir="z")
        else:
            self.scatter_pose_objetivo.set_offsets(np.empty((0, 2)))
            self.scatter_pose_objetivo.set_3d_properties([], zdir="z")

        if tiene_pose_predicha:
            self.scatter_pose_predicha.set_offsets(np.c_[[float(posicion_pose_predicha[0])], [float(posicion_pose_predicha[1])]])
            self.scatter_pose_predicha.set_3d_properties([float(posicion_pose_predicha[2])], zdir="z")
        else:
            self.scatter_pose_predicha.set_offsets(np.empty((0, 2)))
            self.scatter_pose_predicha.set_3d_properties([], zdir="z")

        puntos_pose = []
        if tiene_pose_objetivo:
            puntos_pose.append(np.asarray([float(posicion_pose_objetivo[0]), float(posicion_pose_objetivo[1]), float(posicion_pose_objetivo[2])]))
        if tiene_pose_predicha:
            puntos_pose.append(np.asarray([float(posicion_pose_predicha[0]), float(posicion_pose_predicha[1]), float(posicion_pose_predicha[2])]))
        if len(puntos_pose) > 0:
            arreglo = np.vstack(puntos_pose)
            pminx, pminy, pminz = float(np.min(arreglo[:,0])), float(np.min(arreglo[:,1])), float(np.min(arreglo[:,2]))
            pmaxx, pmaxy, pmaxz = float(np.max(arreglo[:,0])), float(np.max(arreglo[:,1])), float(np.max(arreglo[:,2]))
            rango_pose_x = max(pmaxx - pminx, self.rango_minimo)
            rango_pose_y = max(pmaxy - pminy, self.rango_minimo)
            rango_pose_z = max(pmaxz - pminz, self.rango_minimo)
            margen_pose_x = rango_pose_x * self.margen_zoom
            margen_pose_y = rango_pose_y * self.margen_zoom
            margen_pose_z = rango_pose_z * self.margen_zoom
            self.ax_pose.set_xlim(pminx - margen_pose_x, pmaxx + margen_pose_x)
            self.ax_pose.set_ylim(pminy - margen_pose_y, pmaxy + margen_pose_y)
            self.ax_pose.set_zlim(pminz - margen_pose_z, pmaxz + margen_pose_z)

        if self.estado_inicial is not None and self.estado_inicial.size > max(self.dimensiones):
            ix = float(self.estado_inicial[self.dimensiones[0]])
            iy = float(self.estado_inicial[self.dimensiones[1]])
            iz = float(self.estado_inicial[self.dimensiones[2]])
            self.scatter_inicial.set_offsets(np.c_[[ix], [iy]])
            self.scatter_inicial.set_3d_properties([iz], zdir="z")

        historial_aptitud = np.asarray(datos.get("fitness_history", []), dtype=float)
        if historial_aptitud.size > 0:
            x_historial = np.arange(historial_aptitud.size)
            y_historial = np.clip(historial_aptitud, 1e-12, None)
            self.linea_aptitud.set_data(x_historial, y_historial)
            self.ax2d.set_xlim(0, max(1, historial_aptitud.size - 1))
            ymin = float(np.min(y_historial))
            ymax = float(np.max(y_historial))
            if ymax <= ymin:
                ymax = ymin * 1.05
            self.ax2d.set_ylim(ymin * 0.9, ymax * 1.1)

        distancia_a_objetivo = datos.get("distance_to_target", None)
        if distancia_a_objetivo is not None:
            self.texto_distancia.set_text(f"distancia al objetivo: {float(distancia_a_objetivo):.6f}")
        else:
            self.texto_distancia.set_text("")

        iteracion = datos.get("iteration", None)
        ciclo = datos.get("outer_iteration", None)
        k_am_actual = datos.get("k_am_current", None)
        if iteracion is not None:
            iteracion_mostrada = int(iteracion + 1) if self.mostrar_desde_uno else int(iteracion)
            partes_titulo = [f"PSO | Iteración {iteracion_mostrada}"]
            if ciclo is not None:
                partes_titulo.append(f"Ciclo {int(ciclo)}")
            if k_am_actual is not None:
                partes_titulo.append(f"K_AM {float(k_am_actual):.4f}")
            self.fig.suptitle(" | ".join(partes_titulo), color="0.9", fontsize=10)

        return (
            self.scatter_particulas, self.scatter_mejor_global, self.scatter_objetivo, self.scatter_centro,
            self.scatter_pose_objetivo, self.scatter_pose_predicha,
            self.linea_trayectoria_centro, self.linea_aptitud, self.texto_distancia,
        )

    def iniciar(self):
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    visualizador = VisualizadorTiempoReal(ruta_config="config.json", ruta_json="pso_data.json")
    visualizador.iniciar()
