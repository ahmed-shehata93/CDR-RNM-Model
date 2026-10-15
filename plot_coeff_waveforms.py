"""Rebuild PLL analog waveforms from coeff_snapshots.csv.

The testbench writes this file on every charge-pump / VCO update and does not
sample the waveforms in the simulator. Each data row is one XREAL sequence:

    evt,t_s,phi_off,pfd,seq,n,b,a,m,t0,b,a,m,t0,...

seq is icp, lf_v, lf_vc, fout, or df. A snapshot replaces that sequence from
t_s until the next snapshot. vco_rf uses phi_off and the df terms:

    phi(t) = phi_off + integrate(df, t)
    vco_rf = V_RF_PEAK * cos(2*pi*(F_VCO*t + phi(t)))

Set DUMP_WAVES=1 in pll_full_tb.sv to also record the 50 ps samplers in
waves_new.shm and pll_full_tb.vcd.

Example:
    python plot_coeff_waveforms.py
    python plot_coeff_waveforms.py coeff_snapshots.csv --zoom 2e-6 --out waves.png
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

TWO_PI = 2.0 * np.pi
A_FLOOR = 1.0e-30


def load_snapshots(path):
    meta = {}
    events = {}
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                parts = line[1:].split()
                if len(parts) >= 2:
                    meta[parts[0]] = float(parts[1])
                continue
            if line.startswith("evt"):
                continue
            cols = line.split(",")
            evt = int(cols[0])
            n = int(cols[5])
            terms = []
            idx = 6
            for _ in range(n):
                terms.append(
                    (
                        float(cols[idx]),
                        float(cols[idx + 1]),
                        int(float(cols[idx + 2])),
                        float(cols[idx + 3]),
                    )
                )
                idx += 4
            ev = events.setdefault(
                evt,
                {
                    "t": float(cols[1]),
                    "phi": float(cols[2]),
                    "pfd": int(float(cols[3])),
                    "seq": {},
                },
            )
            ev["seq"][cols[4].strip()] = terms
    ordered = [events[k] for k in sorted(events, key=lambda i: (events[i]["t"], i))]
    return meta, ordered


def eval_xreal(terms, t):
    """Match eval_xreal_at_t: b * exp(a*dt) * dt^(m-1) / (m-1)! for t >= t0."""
    y = np.zeros(np.shape(t), dtype=float)
    tt = np.asarray(t, dtype=float)
    for b, a, m, t0 in terms:
        dt = tt - t0
        active = dt >= 0.0
        if not np.any(active):
            continue
        dta = dt[active]
        tv = b * np.exp(a * dta)
        if m > 1:
            fact = 1
            for pwr in range(1, m):
                fact *= pwr
                tv *= dta
            tv /= fact
        y[active] += tv
    return y


def integrate_m1(terms, t):
    """Match integrate_xreal_m1_at_t (m = 1 only)."""
    y = np.zeros(np.shape(t), dtype=float)
    tt = np.asarray(t, dtype=float)
    for b, a, m, t0 in terms:
        if m != 1:
            raise ValueError(f"df term m={m} is not 1")
        dt = tt - t0
        active = dt >= 0.0
        if not np.any(active):
            continue
        dta = dt[active]
        if abs(a) < A_FLOOR:
            y[active] += b * dta
        else:
            y[active] += (b / a) * (np.exp(a * dta) - 1.0)
    return y


def settled_dc(terms):
    """Sum of a=0, m=1 coefficients: the value left after exponentials die."""
    acc = 0.0
    for b, a, m, _t0 in terms:
        if m == 1 and abs(a) < A_FLOOR:
            acc += b
    return acc


def _segment_bounds(event_t, sample_t):
    """sample_t[start:end] lies in [event_t[i], event_t[i+1])."""
    starts = np.searchsorted(sample_t, event_t, side="left")
    bounds = []
    n = len(sample_t)
    for i, i0 in enumerate(starts):
        i1 = int(starts[i + 1]) if i + 1 < len(starts) else n
        bounds.append((int(i0), i1))
    return bounds


def sample_seq(events, name, sample_t):
    y = np.full(sample_t.shape, np.nan, dtype=float)
    if not events:
        return y
    event_t = np.array([ev["t"] for ev in events], dtype=float)
    for ev, (i0, i1) in zip(events, _segment_bounds(event_t, sample_t)):
        if i0 >= i1:
            continue
        terms = ev["seq"].get(name, [])
        y[i0:i1] = eval_xreal(terms, sample_t[i0:i1]) if terms else 0.0
    return y


def sample_rf(events, sample_t, f_vco, v_peak):
    y = np.full(sample_t.shape, np.nan, dtype=float)
    if not events:
        return y
    event_t = np.array([ev["t"] for ev in events], dtype=float)
    for ev, (i0, i1) in zip(events, _segment_bounds(event_t, sample_t)):
        if i0 >= i1:
            continue
        ts = sample_t[i0:i1]
        phi = ev["phi"] + integrate_m1(ev["seq"].get("df", []), ts)
        y[i0:i1] = v_peak * np.cos(TWO_PI * (f_vco * ts + phi))
    return y


def hold_average(event_t, held, bin_edges):
    """Time average of a stepwise hold over each [bin_edges[i], bin_edges[i+1])."""
    out = np.zeros(len(bin_edges) - 1, dtype=float)
    if len(event_t) == 0:
        return out
    t_end = np.empty(len(event_t), dtype=float)
    t_end[:-1] = event_t[1:]
    t_end[-1] = bin_edges[-1]
    for i, (a, b, val) in enumerate(zip(event_t, t_end, held)):
        if b <= a:
            continue
        i0 = int(np.searchsorted(bin_edges, a, side="right") - 1)
        i1 = int(np.searchsorted(bin_edges, b, side="left"))
        i0 = max(i0, 0)
        i1 = min(i1, len(out))
        for k in range(i0, i1):
            lo = max(a, bin_edges[k])
            hi = min(b, bin_edges[k + 1])
            if hi > lo:
                out[k] += val * (hi - lo)
    width = np.diff(bin_edges)
    np.divide(out, width, out=out, where=width > 0.0)
    return out


def _check_formulas():
    t = np.array([0.0, 1e-9, 2e-9])
    step = eval_xreal([(0.01, 0.0, 1, 0.0)], t)
    if not np.allclose(step, 0.01):
        raise AssertionError(step)
    # m=3 -> b * exp(0) * dt^2 / 2
    ramp = eval_xreal([(2.0, 0.0, 3, 0.0)], t)
    expect = 2.0 * t**2 / 2.0
    if not np.allclose(ramp, expect):
        raise AssertionError((ramp, expect))
    integ = integrate_m1([(4.0, 0.0, 1, 0.0)], t)
    if not np.allclose(integ, 4.0 * t):
        raise AssertionError(integ)
    # (b/a)*(exp(a*dt)-1) with a=-1e9, b=1 at dt=0 is 0
    z = integrate_m1([(1.0, -1.0e9, 1, 0.0)], np.array([0.0]))
    if abs(z[0]) > 1e-12:
        raise AssertionError(z)


def _argv_without_kernel_flag(argv):
    """Drop ipykernel's ``-f <connection.json>`` so argparse does not see it."""
    out = []
    i = 0
    while i < len(argv):
        if argv[i] == "-f":
            i += 2
            continue
        out.append(argv[i])
        i += 1
    return out


def _resolve_csv(csv_name):
    path = Path(csv_name)
    if path.is_file():
        return path
    beside_script = Path(__file__).resolve().parent / csv_name
    if beside_script.is_file():
        return beside_script
    raise SystemExit(
        f"cannot find {csv_name}\n"
        f"  looked in {Path.cwd()}\n"
        f"  and in {beside_script.parent}"
    )


def main():
    _check_formulas()

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", default="coeff_snapshots.csv")
    ap.add_argument("--t0", type=float, default=0.0, help="plot start [s]")
    ap.add_argument("--t1", type=float, default=None, help="plot end [s] (default: T_SIM or last event)")
    ap.add_argument("--dt", type=float, default=10e-9, help="baseband sample step [s]")
    ap.add_argument("--zoom", type=float, default=1e-6, help="center of the CP / RF zoom [s]")
    ap.add_argument("--zoom-span", type=float, default=2e-9, help="width of the CP / RF zoom [s]")
    ap.add_argument("--out", default="coeff_waveforms.png")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args(_argv_without_kernel_flag(sys.argv[1:]))
    csv_path = _resolve_csv(args.csv)

    meta, events = load_snapshots(csv_path)
    if not events:
        raise SystemExit(f"no snapshots in {args.csv}")

    f_vco = meta.get("F_VCO", 4.9995e9)
    v_peak = meta.get("V_RF_PEAK", 0.5)
    t1 = args.t1
    if t1 is None:
        t1 = meta.get("T_SIM", events[-1]["t"])
    if t1 <= args.t0:
        t1 = events[-1]["t"]
    sample_t = np.arange(args.t0, t1 + args.dt * 0.5, args.dt)
    if len(sample_t) < 2:
        raise SystemExit("time window is empty; check --t0/--t1/--dt")

    lf_v = sample_seq(events, "lf_v", sample_t)
    lf_vc = sample_seq(events, "lf_vc", sample_t)
    fout = sample_seq(events, "fout", sample_t)

    ev_t = np.array([ev["t"] for ev in events], dtype=float)
    icp_hold = np.array([settled_dc(ev["seq"].get("icp", [])) for ev in events], dtype=float)
    bin_edges = np.arange(args.t0, t1 + args.dt, args.dt)
    if bin_edges[-1] < t1:
        bin_edges = np.append(bin_edges, t1)
    icp_avg = hold_average(ev_t, icp_hold, bin_edges)
    t_avg = 0.5 * (bin_edges[:-1] + bin_edges[1:])

    z0 = args.zoom - 0.5 * args.zoom_span
    z1 = args.zoom + 0.5 * args.zoom_span
    z_dt = min(args.zoom_span / 400.0, 5e-12)
    if z_dt <= 0.0:
        z_dt = 5e-12
    zoom_t = np.arange(z0, z1, z_dt)
    icp_z = sample_seq(events, "icp", zoom_t)
    rf_z = sample_rf(events, zoom_t, f_vco, v_peak)

    tus = sample_t * 1e6
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), constrained_layout=True)

    axes[0].plot(tus, lf_v * 1e3, label="lf_v")
    axes[0].plot(tus, lf_vc * 1e3, label="lf_vc", alpha=0.85)
    axes[0].set_ylabel("Voltage (mV)")
    axes[0].set_title("Loop filter")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(tus, fout / 1e9)
    axes[1].set_ylim(4.9, 5.2)
    axes[1].set_ylabel("Frequency (GHz)")
    axes[1].set_title("VCO fout")
    axes[1].grid(True)

    axes[2].plot(t_avg * 1e6, icp_avg * 1e6)
    axes[2].set_ylabel("Current (uA)")
    axes[2].set_title("Charge-pump current, settled value averaged over each sample step")
    axes[2].grid(True)

    for ax in axes:
        ax.set_xlabel("Time (us)")

    out = Path(args.out)
    icp_out = out.with_name(out.stem + "_zoom_icp" + out.suffix)
    rf_out = out.with_name(out.stem + "_zoom_vco_rf" + out.suffix)
    zoom_title = f"Zoom at {args.zoom * 1e6:.3f} us"

    fig_icp, ax_icp = plt.subplots(figsize=(10, 4), constrained_layout=True)
    ax_icp.plot(zoom_t * 1e9, icp_z * 1e6, color="C0")
    ax_icp.set_ylabel("icp (uA)")
    ax_icp.set_xlabel("Time (ns)")
    ax_icp.set_title(zoom_title)
    ax_icp.grid(True)

    fig_rf, ax_rf = plt.subplots(figsize=(10, 4), constrained_layout=True)
    ax_rf.plot(zoom_t * 1e9, rf_z, color="C1")
    ax_rf.set_ylabel("vco_rf (V)")
    ax_rf.set_xlabel("Time (ns)")
    ax_rf.set_title(zoom_title)
    ax_rf.grid(True)

    fig.savefig(out, dpi=120)
    fig_icp.savefig(icp_out, dpi=120)
    fig_rf.savefig(rf_out, dpi=120)
    print(f"Wrote {out}, {icp_out.name}, {rf_out.name}  ({len(events)} snapshots, {len(sample_t)} baseband points)")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
