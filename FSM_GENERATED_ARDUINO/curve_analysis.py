#!/usr/bin/env python3
"""
curve_analysis.py
==================
Analiza rasta broja ciklusa (i memorije) u funkciji broja stanja N (ili dubine
hijerarhije D za hsm_nested), na osnovu CSV-a koji generise sweep_benchmark.py.

Za svaku (pattern, platform) kombinaciju:
1. Fituje O(1) (konstantan) i O(n) (linearan) model preko scipy.stats.linregress
2. Koristi p-vrednost NAGIBA (ne R^2!) da odluci da li je rast staticki znacajan.
   R^2 sam za sebe je nepouzdana metrika kad je pattern skoro konstantan (mali
   sum moze dati nizak R^2 linearnog fita i za O(1) i za O(n) podatke), pa se
   klasifikacija oslanja na: "je li nagib razlicit od nule uz p<0.05?"
   - p > 0.05 -> nagib nije statisticki znacajan -> ponasanje je O(1)
   - p <= 0.05 -> nagib je znacajan -> ponasanje raste sa N -> O(n) (ili gore)
3. Dodatno racuna R^2 oba modela radi vizuelne/tabelarne provere
4. Generise grafikone (cikli vs N, za sve pattern-e na istom grafu, po platformi)
   i finalnu tabelu (CSV + odštampanu) sa klasifikacijom po pattern-u

Ulaz: CSV sa kolonama (minimalno):
    platform, pattern, param, param_value, metric, value
gde je 'param' jedno od {"n_states","depth"}, a 'metric' npr. "avg_cycles",
"flash_bytes", "ram_bytes". Ovo odgovara izlazu sweep_benchmark.py.

Upotreba:
    python curve_analysis.py --csv sweep_results.csv --metric avg_cycles --outdir output
"""

import argparse
import csv
import os
from collections import defaultdict

import numpy as np
from scipy import stats

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def classify_growth(x_vals, y_vals, alpha=0.05):
    """Klasifikuje rast kao O(1) ili O(n) na osnovu p-vrednosti nagiba linearne
    regresije (ne R^2 - vidi napomenu u header-u modula)."""
    x = np.array(x_vals, dtype=float)
    y = np.array(y_vals, dtype=float)
    if len(x) < 3 or np.all(x == x[0]):
        return {"slope": float("nan"), "intercept": float(np.mean(y)),
                "r2_linear": float("nan"), "slope_pvalue": float("nan"),
                "classification": "N/A (nedovoljno tacaka)"}

    slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
    r2_linear = r_value ** 2
    is_constant = p_value > alpha
    return {
        "slope": slope,
        "intercept": intercept,
        "r2_linear": r2_linear,
        "slope_pvalue": p_value,
        "std_err": std_err,
        "classification": "O(1)" if is_constant else "O(n) (rastuci)"
    }


def analyze(rows, metric, outdir):
    os.makedirs(outdir, exist_ok=True)

    # grupisi po (platform, pattern), sortirano po param_value
    groups = defaultdict(list)
    for r in rows:
        if r["metric"] != metric:
            continue
        key = (r["platform"], r["pattern"])
        groups[key].append((float(r["param_value"]), float(r["value"])))

    summary_rows = []
    plot_data = defaultdict(dict)  # platform -> pattern -> (xs, ys)

    for (platform, pattern), points in sorted(groups.items()):
        points.sort()
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        result = classify_growth(xs, ys)
        summary_rows.append({
            "platform": platform, "pattern": pattern, "metric": metric,
            "n_points": len(points),
            "slope": result["slope"], "intercept": result["intercept"],
            "r2_linear": result["r2_linear"], "slope_pvalue": result["slope_pvalue"],
            "classification": result["classification"],
        })
        plot_data[platform][pattern] = (xs, ys)

    summary_csv = os.path.join(outdir, f"growth_classification_{metric}.csv")
    with open(summary_csv, "w", newline="") as f:
        fieldnames = ["platform", "pattern", "metric", "n_points", "slope",
                      "intercept", "r2_linear", "slope_pvalue", "classification"]
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(summary_rows)

    print(f"\n=== Klasifikacija rasta za metriku '{metric}' ===")
    print(f"{'Platforma':<8} {'Pattern':<22} {'Nagib':>10} {'p-vrednost':>12} {'R2(lin)':>8}  Klasifikacija")
    for r in summary_rows:
        slope_str = f"{r['slope']:.4f}" if not np.isnan(r['slope']) else "N/A"
        p_str = f"{r['slope_pvalue']:.4g}" if not np.isnan(r['slope_pvalue']) else "N/A"
        r2_str = f"{r['r2_linear']:.3f}" if not np.isnan(r['r2_linear']) else "N/A"
        print(f"{r['platform']:<8} {r['pattern']:<22} {slope_str:>10} {p_str:>12} {r2_str:>8}  {r['classification']}")

    if HAVE_MPL:
        for platform, patterns in plot_data.items():
            plt.figure(figsize=(8, 5))
            for pattern, (xs, ys) in patterns.items():
                plt.plot(xs, ys, marker="o", label=pattern)
            plt.xlabel("N (broj stanja / dubina hijerarhije)")
            plt.ylabel(metric)
            plt.title(f"{metric} vs N — platforma: {platform}")
            plt.legend()
            plt.grid(True, alpha=0.3)
            fig_path = os.path.join(outdir, f"growth_{platform}_{metric}.png")
            plt.savefig(fig_path, dpi=150, bbox_inches="tight")
            plt.close()
            print(f"Grafikon sacuvan: {fig_path}")

    return summary_csv


def main():
    parser = argparse.ArgumentParser(description="Fitovanje O(1)/O(n) krive na sweep rezultate")
    parser.add_argument("--csv", required=True, help="ulazni CSV (long format: platform,pattern,param,param_value,metric,value)")
    parser.add_argument("--metric", default="avg_cycles", help="koja metrika se analizira (avg_cycles, flash_bytes, ram_bytes...)")
    parser.add_argument("--outdir", default="output", help="direktorijum za CSV/PNG izlaze")
    args = parser.parse_args()

    rows = load_csv(args.csv)
    analyze(rows, args.metric, args.outdir)


if __name__ == "__main__":
    main()
