#!/usr/bin/env python3
"""Turn the 8-10 trial logs into the presentation figures, table and statistics. No ROS needed.

    python3 scripts/analyze_trials.py                      # real trials in results/trials
    python3 scripts/analyze_trials.py --trials results/_demo/trials --out results/_demo   # simulated

Reads   <trials>/trial_XX_*/summary.json (+ steps.csv); only trials with d_meas and valid = true.
        If a trial number was run more than once, the newest folder is used.
Writes  <out>/summary_table.csv | .md | .tex     one row per trial + the statistics
        <out>/statistics.json                     every number in one place
        <out>/figures/fig1..fig5 (.png 300 dpi and .pdf)

Error convention (spec, required work 3):  e_i = d_est_i - d_meas_i   (+ = robot thought it was farther away)
"""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]

# validated categorical slots 1-3 (colour-blind safe for scatter use), ink and grid
C_EKF, C_CAM, C_ODOM = '#2a78d6', '#eb6834', '#1baf7a'
INK, INK2, MUTED, GRID = '#0b0b0b', '#52514e', '#898781', '#e4e6e3'
EST_NAMES = {'ekf': 'EKF (prediction + correction)', 'cam': 'camera only', 'odom': 'odometry only'}
EST_COLORS = {'ekf': C_EKF, 'cam': C_CAM, 'odom': C_ODOM}


def style():
    plt.rcParams.update({
        'figure.dpi': 110, 'savefig.dpi': 300, 'font.size': 10, 'axes.titlesize': 11, 'axes.titleweight': 'bold',
        'axes.titlelocation': 'left', 'axes.edgecolor': MUTED, 'axes.labelcolor': INK2, 'xtick.color': INK2,
        'ytick.color': INK2, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
        'axes.spines.top': False, 'axes.spines.right': False, 'legend.frameon': False, 'lines.linewidth': 2,
    })


# ------------------------------------------------------------------ loading

def load_trials(trials_dir, include_invalid=False):
    newest = {}
    for path in sorted(Path(trials_dir).glob('trial_*/summary.json')):
        s = json.loads(path.read_text())
        s['_dir'] = path.parent
        newest[s['trial']] = s                     # sorted by name -> later timestamp wins
    trials, skipped = [], []
    for n in sorted(newest):
        s = newest[n]
        if s.get('d_meas') is None:
            skipped.append(f'trial {n}: no tape measurement yet (scripts/add_measurement.py)')
        elif not s.get('valid', True) and not include_invalid:
            skipped.append(f'trial {n}: invalid ({s.get("stop_reason")})')
        else:
            trials.append(s)
    return trials, skipped


def load_steps(folder):
    with open(Path(folder) / 'steps.csv', newline='') as f:
        rows = list(csv.DictReader(f))
    cols = {}
    for key in rows[0]:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[key]))
            except ValueError:
                vals.append(np.nan if key != 'state' else r[key])
        cols[key] = np.array(vals, dtype=object if key == 'state' else float)
    return cols


# ------------------------------------------------------------------ statistics

def stats_of(e):
    e = np.asarray([v for v in e if v is not None], dtype=float)
    if len(e) == 0:
        return None
    n = len(e)
    return {
        'n': n,
        'mean_error_bias': float(e.mean()),
        'mae': float(np.abs(e).mean()),
        'rmse': float(np.sqrt(np.mean(e ** 2))),
        'min_error': float(e.min()),
        'max_error': float(e.max()),
        'range_of_errors': float(e.max() - e.min()),
        'variance_sample': float(e.var(ddof=1)) if n > 1 else 0.0,     # divide by n-1
        'std_sample': float(e.std(ddof=1)) if n > 1 else 0.0,
        'variance_population': float(e.var(ddof=0)),                    # divide by n
        'std_population': float(e.std(ddof=0)),
    }


def compute(trials):
    d_est = np.array([s['d_est_final'] for s in trials])
    d_meas = np.array([s['d_meas'] for s in trials])
    sigma = np.array([s.get('sigma_d_final') or np.nan for s in trials])
    e = d_est - d_meas
    out = {'trials': [s['trial'] for s in trials], 'ekf': stats_of(e),
           'cam': stats_of([s.get('error_cam') for s in trials]),
           'odom': stats_of([s.get('error_odom') for s in trials])}
    if len(trials) > 1:
        cov = np.cov(d_est, d_meas, ddof=1)
        out['covariance_est_meas'] = float(cov[0, 1])
        out['variance_est'] = float(cov[0, 0])
        out['variance_meas'] = float(cov[1, 1])
        out['correlation_est_meas'] = float(np.corrcoef(d_est, d_meas)[0, 1]) if cov[0, 0] * cov[1, 1] > 0 else None
    # does the EKF know how wrong it is? compare its own sigma with the real spread
    out['ekf_mean_sigma_d'] = float(np.nanmean(sigma))
    out['ekf_consistency_ratio'] = (out['ekf']['std_sample'] / out['ekf_mean_sigma_d']
                                    if out['ekf_mean_sigma_d'] > 0 and len(trials) > 1 else None)
    out['ekf_mean_nees'] = float(np.nanmean((e / sigma) ** 2))            # ~1 if P is honest
    out['ekf_within_2sigma'] = int(np.sum(np.abs(e) <= 2 * sigma))
    return out


# ------------------------------------------------------------------ outputs

def write_tables(trials, st, out):
    out.mkdir(parents=True, exist_ok=True)
    head = ['trial', 'd_est_m', 'sigma_ekf_m', 'd_meas_m', 'error_m', 'abs_error_m', 'error_cam_m', 'error_odom_m',
            'stop_reason']
    rows = []
    for s in trials:
        e = s['d_est_final'] - s['d_meas']
        rows.append([s['trial'], s['d_est_final'], s.get('sigma_d_final'), s['d_meas'], e, abs(e),
                     s.get('error_cam'), s.get('error_odom'), s.get('stop_reason', '')])
    fmt = lambda v: '' if v is None else (f'{v:.4f}' if isinstance(v, float) else str(v))  # noqa: E731
    with open(out / 'summary_table.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(head)
        for r in rows:
            w.writerow([fmt(v) for v in r])

    k = st['ekf']
    cm = lambda v: '-' if v is None else f'{100 * v:+.2f}'  # noqa: E731
    md = ['| Trial | Estimated d (m) | EKF 1σ (cm) | Measured d (m) | Error e = est − meas (cm) | |e| (cm) '
          '| Camera-only e (cm) | Odometry-only e (cm) |',
          '|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        md.append(f'| {r[0]} | {r[1]:.3f} | {100 * (r[2] or 0):.2f} | {r[3]:.3f} | {cm(r[4])} | {100 * r[5]:.2f} '
                  f'| {cm(r[6])} | {cm(r[7])} |')
    md += ['', '| Statistic (EKF final distance error) | Value |', '|---|---:|',
           f'| Number of trials | {k["n"]} |',
           f'| Mean absolute error (MAE) | {100 * k["mae"]:.2f} cm |',
           f'| Range of errors (max − min) | {100 * k["range_of_errors"]:.2f} cm '
           f'({cm(k["min_error"])} to {cm(k["max_error"])} cm) |',
           f'| Mean error (bias) | {cm(k["mean_error_bias"])} cm |',
           f'| Variance σ² (sample, n−1) | {1e4 * k["variance_sample"]:.3f} cm² |',
           f'| Standard deviation σ (sample) | {100 * k["std_sample"]:.2f} cm |',
           f'| RMSE | {100 * k["rmse"]:.2f} cm |']
    if 'covariance_est_meas' in st:
        md += [f'| Covariance cov(d_est, d_meas) | {1e4 * st["covariance_est_meas"]:.3f} cm² |',
               f'| Correlation (d_est, d_meas) | {fmt(st["correlation_est_meas"])} |']
    md += [f'| Mean EKF 1σ from P | {100 * st["ekf_mean_sigma_d"]:.2f} cm |',
           f'| Real σ / EKF σ (≈1 = honest P) | {fmt(st["ekf_consistency_ratio"])} |',
           f'| Trials with |e| ≤ 2σ_EKF | {st["ekf_within_2sigma"]} of {k["n"]} |']
    for key in ('cam', 'odom'):
        if st[key]:
            md.append(f'| MAE, {EST_NAMES[key]} | {100 * st[key]["mae"]:.2f} cm |')
    (out / 'summary_table.md').write_text('\n'.join(md) + '\n', encoding='utf-8')

    tex = ['\\begin{tabular}{rrrrrr}', '\\hline',
           'Trial & $d_{est}$ (m) & $\\sigma_{EKF}$ (cm) & $d_{meas}$ (m) & $e$ (cm) & $|e|$ (cm) \\\\', '\\hline']
    for r in rows:
        tex.append(f'{r[0]} & {r[1]:.3f} & {100 * (r[2] or 0):.2f} & {r[3]:.3f} & {cm(r[4])} & {100 * r[5]:.2f} \\\\')
    tex += ['\\hline', f'\\multicolumn{{4}}{{r}}{{MAE}} & \\multicolumn{{2}}{{r}}{{{100 * k["mae"]:.2f} cm}} \\\\',
            f'\\multicolumn{{4}}{{r}}{{Range of errors}} & \\multicolumn{{2}}{{r}}{{{100 * k["range_of_errors"]:.2f} cm}} \\\\',
            f'\\multicolumn{{4}}{{r}}{{Mean error (bias)}} & \\multicolumn{{2}}{{r}}{{{cm(k["mean_error_bias"])} cm}} \\\\',
            f'\\multicolumn{{4}}{{r}}{{Std.\\ deviation $\\sigma$}} & \\multicolumn{{2}}{{r}}{{{100 * k["std_sample"]:.2f} cm}} \\\\',
            '\\hline', '\\end{tabular}']
    (out / 'summary_table.tex').write_text('\n'.join(tex) + '\n', encoding='utf-8')
    (out / 'statistics.json').write_text(json.dumps(st, indent=2))


def save(fig, out, name):
    d = out / 'figures'
    d.mkdir(parents=True, exist_ok=True)
    fig.savefig(d / f'{name}.png', bbox_inches='tight')
    fig.savefig(d / f'{name}.pdf', bbox_inches='tight')
    plt.close(fig)


def title_note(sim):
    return '  [SIMULATED DATA]' if sim else ''


def fig_scatter(trials, st, out, sim):
    """Deliverable 6a: estimated vs measured final distance, one dot per trial, ±2σ from P."""
    d_est = np.array([s['d_est_final'] for s in trials])
    d_meas = np.array([s['d_meas'] for s in trials])
    sig = np.array([s.get('sigma_d_final') or 0 for s in trials])
    fig, ax = plt.subplots(figsize=(6.2, 5.4))
    lo = min(d_est.min() - 2 * sig.max(), d_meas.min()) - 0.01
    hi = max(d_est.max() + 2 * sig.max(), d_meas.max()) + 0.01
    ax.plot([lo, hi], [lo, hi], color=MUTED, lw=1, ls='--', label='perfect estimate (e = 0)')
    ax.errorbar(d_meas, d_est, yerr=2 * sig, fmt='none', ecolor=C_EKF, elinewidth=1, alpha=0.45, capsize=3,
                label='±2σ from the EKF covariance P')
    ax.scatter(d_meas, d_est, s=64, color=C_EKF, edgecolor='white', linewidth=2, zorder=3,
               label=EST_NAMES['ekf'])
    for s, x, y in zip(trials, d_meas, d_est):
        ax.annotate(str(s['trial']), (x, y), xytext=(6, -3), textcoords='offset points', fontsize=8, color=INK2)
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_aspect('equal')
    ax.set_xlabel('measured final distance, tape (m)')
    ax.set_ylabel('estimated final distance, EKF (m)')
    k = st['ekf']
    ax.set_title('Estimated vs measured final distance' + title_note(sim))
    ax.text(0.03, 0.97, f'MAE {100 * k["mae"]:.2f} cm   bias {100 * k["mean_error_bias"]:+.2f} cm\n'
            f'σ {100 * k["std_sample"]:.2f} cm   range {100 * k["range_of_errors"]:.2f} cm',
            transform=ax.transAxes, va='top', fontsize=9, color=INK)
    ax.legend(loc='lower right', fontsize=8)
    save(fig, out, 'fig1_scatter_estimated_vs_measured')


def fig_errors(trials, st, out, sim):
    """Deliverable 6b: final distance error per trial, with mean, ±σ and the two single-sensor baselines."""
    n = np.array([s['trial'] for s in trials])
    e = 100 * np.array([s['d_est_final'] - s['d_meas'] for s in trials])
    sig = 100 * np.array([s.get('sigma_d_final') or 0 for s in trials])
    k = st['ekf']
    fig, ax = plt.subplots(figsize=(8, 4.4))
    ax.axhline(0, color=INK2, lw=1)
    m, sd = 100 * k['mean_error_bias'], 100 * k['std_sample']
    ax.axhspan(m - sd, m + sd, color=C_EKF, alpha=0.08, lw=0, label=f'mean ± σ ({m:+.2f} ± {sd:.2f} cm)')
    ax.axhline(m, color=C_EKF, lw=1, ls='--')
    for key, mk in (('odom', 's'), ('cam', '^')):
        vals = [s.get(f'error_{key}') for s in trials]
        if any(v is not None for v in vals):
            ax.scatter(n, [np.nan if v is None else 100 * v for v in vals], marker=mk, s=40, facecolor='white',
                       edgecolor=EST_COLORS[key], linewidth=1.6, zorder=3, label=EST_NAMES[key])
    ax.errorbar(n, e, yerr=2 * sig, fmt='o', ms=8, color=C_EKF, mec='white', mew=1.5, elinewidth=1, capsize=3,
                zorder=4, label=EST_NAMES['ekf'] + ', ±2σ')
    for x, y in zip(n, e):
        ax.annotate(f'{y:+.1f}', (x, y), xytext=(8, -3), textcoords='offset points', fontsize=8, color=INK2)
    ax.set_xticks(n)
    ax.set_xlabel('trial')
    ax.set_ylabel('final distance error  e = d_est − d_meas (cm)')
    ax.set_title('Final distance estimation error per trial' + title_note(sim))
    ax.legend(loc='upper left', bbox_to_anchor=(1.0, 1.0), fontsize=8)
    save(fig, out, 'fig2_error_per_trial')


def fig_time(trials, out, sim):
    """Distance to the tag during each trial: EKF vs camera vs odometry alone (role of prediction/correction)."""
    fig, ax = plt.subplots(figsize=(8, 4.6))
    for i, s in enumerate(trials):
        c = load_steps(s['_dir'])
        drive = np.where(c['state'] != 'WAITING')[0]
        if len(drive) == 0:
            continue
        t = c['t'] - c['t'][drive[0]]
        first = i == 0
        ax.plot(t, c['d_odom_only'], color=C_ODOM, lw=1, alpha=0.6, label=EST_NAMES['odom'] if first else None)
        ok = c['tag_new'] == 1
        ax.scatter(t[ok], c['tag_d'][ok], s=5, color=C_CAM, alpha=0.35, lw=0, label=EST_NAMES['cam'] if first else None)
        ax.plot(t, c['d_est'], color=C_EKF, lw=1.5, alpha=0.9, label=EST_NAMES['ekf'] if first else None)
    stop = trials[0].get('stop_distance')
    if stop is not None:
        ax.axhline(stop, color=INK2, lw=1, ls=':', label=f'stop distance {stop:.2f} m')
    ax.set_xlim(left=-2)
    ax.set_xlabel('time from drive start (s)')
    ax.set_ylabel('distance chassis front → tag (m)')
    ax.set_title(f'Distance to the tag, all {len(trials)} trials' + title_note(sim))
    ax.legend(loc='upper right', fontsize=8, markerscale=3)
    save(fig, out, 'fig3_distance_vs_time')


def fig_uncertainty(trials, out, sim):
    """P at work: the EKF's 1σ distance uncertainty grows while predicting and shrinks at each camera update."""
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for i, s in enumerate(trials):
        c = load_steps(s['_dir'])
        drive = np.where(c['state'] != 'WAITING')[0]
        if len(drive) == 0:
            continue
        t = c['t'] - c['t'][drive[0]]
        ax.plot(t, 100 * c['sigma_d'], color=C_EKF, lw=1, alpha=0.55, label='EKF 1σ of d (from P)' if i == 0 else None)
    ax.set_xlim(left=-2)
    ax.set_ylim(bottom=0)
    ax.set_xlabel('time from drive start (s)')
    ax.set_ylabel('1σ distance uncertainty (cm)')
    ax.set_title('How sure the EKF is: σ_d = √(J P J^T)' + title_note(sim))
    ax.legend(loc='upper right', fontsize=8)
    save(fig, out, 'fig4_ekf_uncertainty')


def fig_breakdown(st, out, sim):
    """Deliverable 7 visual: MAE, |bias|, σ, range and RMSE for EKF vs the single-sensor baselines."""
    keys = [k for k in ('ekf', 'cam', 'odom') if st.get(k)]
    metrics = [('mae', 'MAE'), ('mean_error_bias', '|bias|'), ('std_sample', 'σ'), ('range_of_errors', 'range'),
               ('rmse', 'RMSE')]
    x = np.arange(len(metrics))
    w = 0.8 / len(keys)
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for i, key in enumerate(keys):
        vals = [100 * abs(st[key][m]) for m, _ in metrics]
        bars = ax.bar(x + (i - (len(keys) - 1) / 2) * w, vals, width=w - 0.03, color=EST_COLORS[key],
                      label=EST_NAMES[key], edgecolor='white', linewidth=2)
        if key == 'ekf':
            ax.bar_label(bars, fmt='%.1f', fontsize=8, color=INK2, padding=2)
    ax.set_xticks(x, [label for _, label in metrics])
    ax.set_ylabel('cm')
    ax.grid(axis='x', visible=False)
    ax.set_title('Error statistics: prediction + correction vs one sensor alone' + title_note(sim))
    ax.legend(fontsize=8)
    save(fig, out, 'fig5_statistics_breakdown')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trials', default=str(ROOT / 'results' / 'trials'))
    ap.add_argument('--out', default=str(ROOT / 'results'))
    ap.add_argument('--include-invalid', action='store_true', help='also use trials stopped by a safety brake')
    args = ap.parse_args()
    trials, skipped = load_trials(args.trials, args.include_invalid)
    for msg in skipped:
        print('skipped', msg)
    if not trials:
        raise SystemExit(f'No measured trials in {args.trials}')
    sim = any(s.get('simulated') for s in trials)
    out = Path(args.out)
    style()
    st = compute(trials)
    write_tables(trials, st, out)
    fig_scatter(trials, st, out, sim)
    fig_errors(trials, st, out, sim)
    fig_time(trials, out, sim)
    fig_uncertainty(trials, out, sim)
    fig_breakdown(st, out, sim)
    print((out / 'summary_table.md').read_text(encoding='utf-8'))
    print(f'Figures in {out / "figures"}' + ('   (SIMULATED DATA)' if sim else ''))


if __name__ == '__main__':
    main()
