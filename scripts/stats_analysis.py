#!/usr/bin/env python3
"""Statistical analysis of the 10 trials: scatter with regression, correlation matrix, Bland-Altman, tests.

    python3 scripts/stats_analysis.py      # -> results/stats/{stats_analysis.json, *.pdf, *.png, stats_table.tex}

Why these (and not a confusion matrix): the outcome is a continuous distance, not a class, so agreement between two
measurement methods (robot estimate vs tape) is judged with a scatter + regression, a Bland-Altman plot (bias and
limits of agreement) and correlations; a confusion matrix only applies to classification.
"""
import csv
import glob
import json
from pathlib import Path

import numpy as np
from scipy import stats

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results' / 'stats'
BLUE, ORANGE, GREEN, GREY, RED = '#2A78D6', '#EB6834', '#0B7A3E', '#69707A', '#C0392B'


def load():
    rows = list(csv.DictReader(open(ROOT / 'results/summary_table.csv')))
    glitch, sig = [], []
    for r in rows:
        f = sorted(glob.glob(str(ROOT / f"results/trials/trial_{int(r['trial']):02d}_*/summary.json")))[-1]
        s = json.load(open(f))
        glitch.append(s.get('n_odom_glitches', 0))
        sig.append(s['sigma_d_final'])
    g = lambda k: np.array([float(r[k]) for r in rows])   # noqa: E731
    d = dict(trial=g('trial'), d_est=g('d_est_m'), d_meas=g('d_meas_m'), e=g('error_m'), e_cam=g('error_cam_m'),
             e_odom=g('error_odom_m'), glitches=np.array(glitch, float), sigma=np.array(sig))
    d['d_cam'] = d['d_meas'] + d['e_cam']
    d['d_odom'] = d['d_meas'] + d['e_odom']
    return d


def analyse(d):
    n = len(d['e'])
    res = {'n': n}
    for key, e in [('ekf', d['e']), ('camera', d['e_cam']), ('wheels', d['e_odom'])]:
        m, s = e.mean(), e.std(ddof=1)
        t, p = stats.ttest_1samp(e, 0.0)
        ci = stats.t.interval(0.95, n - 1, loc=m, scale=s / np.sqrt(n))
        res[key] = dict(mean_cm=100 * m, std_cm=100 * s, mae_cm=100 * np.abs(e).mean(),
                        rmse_cm=100 * np.sqrt((e ** 2).mean()), min_cm=100 * e.min(), max_cm=100 * e.max(),
                        range_cm=100 * (e.max() - e.min()), median_cm=100 * np.median(e),
                        t_bias=float(t), p_bias=float(p), ci95_mean_cm=[100 * ci[0], 100 * ci[1]],
                        loa_cm=[100 * (m - 1.96 * s), 100 * (m + 1.96 * s)],
                        shapiro_p=float(stats.shapiro(e).pvalue))
    lr = stats.linregress(d['d_meas'], d['d_est'])
    r, p = stats.pearsonr(d['d_meas'], d['d_est'])
    res['regression'] = dict(slope=lr.slope, intercept_m=lr.intercept, r2=lr.rvalue ** 2, p=lr.pvalue,
                             slope_se=lr.stderr, pearson_r=r, pearson_p=p)
    res['covariance_cm2'] = (np.cov(d['d_est'], d['d_meas']) * 1e4).tolist()
    w = stats.wilcoxon(np.abs(d['e_odom']), np.abs(d['e']))
    res['ekf_vs_wheels_abs_error'] = dict(wilcoxon_stat=float(w.statistic), p=float(w.pvalue))
    F = d['e_odom'].var(ddof=1) / d['e'].var(ddof=1)
    res['variance_ratio_wheels_over_ekf'] = dict(F=float(F), p=float(stats.f.sf(F, n - 1, n - 1)),
                                                 levene_p=float(stats.levene(d['e_odom'], d['e']).pvalue))
    w2 = stats.wilcoxon(np.abs(d['e_cam']), np.abs(d['e']))
    res['ekf_vs_camera_abs_error'] = dict(wilcoxon_stat=float(w2.statistic), p=float(w2.pvalue))
    names = ['d_meas', 'd_est', 'd_cam', 'd_odom', 'e', 'e_cam', 'e_odom', 'glitches', 'sigma']
    M = np.corrcoef(np.vstack([d[k] for k in names]))
    res['corr_names'], res['corr'] = names, M.tolist()
    return res


def figures(d, res):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    OUT.mkdir(parents=True, exist_ok=True)

    # 1) scatter: estimate vs tape, with y = x, regression and 2-sigma bars
    fig, ax = plt.subplots(figsize=(5.6, 5.0))
    lo, hi = 0.474, 0.502
    ax.plot([lo, hi], [lo, hi], color=GREY, ls='--', lw=1.2, label='perfect estimate  ($y=x$)')
    rg = res['regression']
    xx = np.array([lo, hi])
    ax.plot(xx, rg['intercept_m'] + rg['slope'] * xx, color=BLUE, lw=1.4, alpha=0.7,
            label=f"least-squares fit  (slope {rg['slope']:.2f}, $R^2$ = {rg['r2']:.2f})")
    ax.errorbar(d['d_meas'], d['d_est'], yerr=2 * d['sigma'], fmt='o', ms=7, color=BLUE, ecolor=BLUE, alpha=0.9,
                elinewidth=0.9, capsize=3, mec='white', label=r'trial, $\pm2\sigma$ from $P$')
    for k in range(len(d['trial'])):
        ax.annotate(f"{int(d['trial'][k])}", (d['d_meas'][k], d['d_est'][k]), xytext=(5, 4),
                    textcoords='offset points', fontsize=8.5, color=GREY)
    e = res['ekf']
    txt = (f"MAE  {e['mae_cm']:.2f} cm\nrange  {e['range_cm']:.2f} cm\nbias  {e['mean_cm']:+.2f} cm (p = "
           f"{e['p_bias']:.2f})\n$\\sigma$  {e['std_cm']:.2f} cm\nPearson r  {rg['pearson_r']:.2f}")
    ax.text(0.97, 0.04, txt, transform=ax.transAxes, ha='right', va='bottom', fontsize=9,
            bbox=dict(boxstyle='round,pad=0.4', facecolor='white', edgecolor='#cccccc'))
    ax.set_xlim(lo, hi), ax.set_ylim(lo, hi), ax.set_aspect('equal')
    ax.set_xlabel('measured final distance, tape (m)')
    ax.set_ylabel('estimated final distance, EKF (m)')
    ax.legend(loc='upper left', fontsize=8.2, frameon=False)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    for ext in ('pdf', 'png'):
        fig.savefig(OUT / f'fig_scatter_regression.{ext}', dpi=220)
    plt.close(fig)

    # 2) correlation matrix heatmap
    labels = {'d_meas': '$d_{meas}$ tape', 'd_est': '$d_{est}$ EKF', 'd_cam': '$d_{cam}$ camera',
              'd_odom': '$d_{wheels}$', 'e': '$e$ EKF', 'e_cam': '$e$ camera', 'e_odom': '$e$ wheels',
              'glitches': 'odom glitches', 'sigma': r'$\sigma_{EKF}$'}
    M = np.array(res['corr'])
    names = res['corr_names']
    fig, ax = plt.subplots(figsize=(6.6, 5.6))
    im = ax.imshow(M, cmap='RdBu_r', vmin=-1, vmax=1)
    ax.set_xticks(range(len(names)), [labels[k] for k in names], rotation=40, ha='right')
    ax.set_yticks(range(len(names)), [labels[k] for k in names])
    for i in range(len(names)):
        for j in range(len(names)):
            ax.text(j, i, f'{M[i, j]:.2f}', ha='center', va='center', fontsize=8,
                    color='white' if abs(M[i, j]) > 0.6 else 'black')
    ax.spines[:].set_visible(False)
    cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03)
    cb.set_label('Pearson correlation  r')
    fig.tight_layout()
    for ext in ('pdf', 'png'):
        fig.savefig(OUT / f'fig_correlation_matrix.{ext}', dpi=220)
    plt.close(fig)

    # 3) Bland-Altman: EKF and wheels against the tape
    fig, axs = plt.subplots(1, 2, figsize=(9.4, 3.7), sharey=False)
    for ax, key, col, title in [(axs[0], 'e', BLUE, 'EKF vs tape'), (axs[1], 'e_odom', GREEN, 'wheels alone vs tape')]:
        est = d['d_meas'] + d[key]
        mean = 100 * (est + d['d_meas']) / 2
        diff = 100 * d[key]
        m, s = diff.mean(), diff.std(ddof=1)
        ax.axhline(0, color=GREY, lw=0.8)
        ax.axhline(m, color=col, lw=1.6, label=f'bias {m:+.2f} cm')
        ax.axhline(m + 1.96 * s, color=col, lw=1.1, ls='--', label=f'limits of agreement  {m - 1.96 * s:+.2f} to '
                   f'{m + 1.96 * s:+.2f} cm')
        ax.axhline(m - 1.96 * s, color=col, lw=1.1, ls='--')
        ax.scatter(mean, diff, color=col, s=40, edgecolor='white', zorder=3)
        for k in range(len(mean)):
            ax.annotate(f"{int(d['trial'][k])}", (mean[k], diff[k]), xytext=(4, 3), textcoords='offset points',
                        fontsize=8, color=GREY)
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('mean of the two methods (cm)')
        ax.set_ylabel('difference, method $-$ tape (cm)')
        ax.legend(fontsize=7.8, frameon=False, loc='upper left')
        ax.grid(alpha=0.3)
    axs[0].set_ylim(-1.5, 1.5)
    axs[1].set_ylim(-6, 10)
    fig.tight_layout()
    for ext in ('pdf', 'png'):
        fig.savefig(OUT / f'fig_bland_altman.{ext}', dpi=220)
    plt.close(fig)


def main():
    d = load()
    res = analyse(d)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'stats_analysis.json').write_text(json.dumps(res, indent=1, default=float))
    figures(d, res)
    for k in ('ekf', 'camera', 'wheels'):
        r = res[k]
        print(f"{k:7s} mean {r['mean_cm']:+.3f}  sd {r['std_cm']:.3f}  MAE {r['mae_cm']:.3f}  RMSE {r['rmse_cm']:.3f}  "
              f"range {r['range_cm']:.3f}  t {r['t_bias']:+.2f} p {r['p_bias']:.3f}  CI95 "
              f"[{r['ci95_mean_cm'][0]:+.3f}, {r['ci95_mean_cm'][1]:+.3f}]  LoA [{r['loa_cm'][0]:+.2f}, "
              f"{r['loa_cm'][1]:+.2f}]  Shapiro p {r['shapiro_p']:.2f}")
    print('regression', {k: round(v, 4) for k, v in res['regression'].items()})
    print('wilcoxon EKF vs wheels', res['ekf_vs_wheels_abs_error'], ' EKF vs camera', res['ekf_vs_camera_abs_error'])
    print('variance ratio wheels/EKF', res['variance_ratio_wheels_over_ekf'])
    names = res['corr_names']
    M = np.array(res['corr'])
    for i, a in enumerate(names):
        print(f'{a:9s}', ' '.join(f'{M[i, j]:+.2f}' for j in range(len(names))))


if __name__ == '__main__':
    main()
