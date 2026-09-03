#!/usr/bin/env python3
"""
Analiza el CSV generado por nav2_eval.py y genera graficas + un resumen
de texto listo para usar en la presentacion.

Uso:
    python3 analyze_nav2_results.py nav2_eval_XXXXXXXX_XXXXXX.csv
"""

import sys
import csv
import statistics
import matplotlib.pyplot as plt


def load_csv(path):
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for r in reader:
            r['distancia_objetivo_m'] = float(r['distancia_objetivo_m'])
            r['angulo_objetivo_deg'] = float(r['angulo_objetivo_deg'])
            r['error_posicion_m'] = float(r['error_posicion_m'])
            r['error_orientacion_deg'] = float(r['error_orientacion_deg'])
            r['exito'] = r['exito'] in ('True', 'true', '1')
            rows.append(r)
    return rows


def success_rate(rows):
    return 100.0 * sum(1 for r in rows if r['exito']) / len(rows) if rows else 0.0


def main():
    if len(sys.argv) < 2:
        print("Uso: python3 analyze_nav2_results.py <archivo.csv>")
        sys.exit(1)

    rows = load_csv(sys.argv[1])
    angulos = sorted(set(r['angulo_objetivo_deg'] for r in rows))
    distancias = sorted(set(r['distancia_objetivo_m'] for r in rows))

    print(f"Total de pruebas: {len(rows)}")
    print(f"Tasa de exito global: {success_rate(rows):.1f}%\n")

    print("Por angulo de giro final:")
    for a in angulos:
        subset = [r for r in rows if r['angulo_objetivo_deg'] == a]
        errores = [r['error_orientacion_deg'] for r in subset]
        print(f"  {a:>5.0f} grados: exito={success_rate(subset):5.1f}%  "
              f"error_prom={statistics.mean(errores):.2f} deg  "
              f"error_max={max(errores):.2f} deg")

    print("\nPor distancia:")
    for d in distancias:
        subset = [r for r in rows if r['distancia_objetivo_m'] == d]
        print(f"  {d} m: exito={success_rate(subset):.1f}%")

    # --- Grafica 1: tasa de exito por angulo ---
    tasas_ang = [success_rate([r for r in rows if r['angulo_objetivo_deg'] == a]) for a in angulos]
    plt.figure(figsize=(6, 4))
    plt.bar([f"{int(a)}°" for a in angulos], tasas_ang, color='#4C72B0')
    plt.ylabel("Tasa de exito (%)")
    plt.title("Tasa de exito por angulo de giro final")
    plt.ylim(0, 100)
    plt.tight_layout()
    plt.savefig("exito_por_angulo.png", dpi=150)
    plt.close()

    # --- Grafica 2: tasa de exito por distancia ---
    tasas_dist = [success_rate([r for r in rows if r['distancia_objetivo_m'] == d]) for d in distancias]
    plt.figure(figsize=(6, 4))
    plt.bar([f"{d} m" for d in distancias], tasas_dist, color='#55A868')
    plt.ylabel("Tasa de exito (%)")
    plt.title("Tasa de exito por distancia recorrida")
    plt.ylim(0, 100)
    plt.tight_layout()
    plt.savefig("exito_por_distancia.png", dpi=150)
    plt.close()

    # --- Grafica 3: error de orientacion por angulo (boxplot) ---
    datos_box = [[r['error_orientacion_deg'] for r in rows if r['angulo_objetivo_deg'] == a] for a in angulos]
    plt.figure(figsize=(6, 4))
    plt.boxplot(datos_box, labels=[f"{int(a)}°" for a in angulos])
    plt.ylabel("Error de orientacion final (grados)")
    plt.title("Distribucion del error de orientacion por angulo pedido")
    plt.tight_layout()
    plt.savefig("error_orientacion_boxplot.png", dpi=150)
    plt.close()

    print("\nGraficas guardadas: exito_por_angulo.png, exito_por_distancia.png, "
          "error_orientacion_boxplot.png")


if __name__ == "__main__":
    main()