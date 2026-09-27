// Full PLL TB: ref/fb -> PFD -> CP -> LF -> VCO -> xreal_to_xbit -> xbit_to_logic
//
// Feedback path (closed loop):
//   VCO fout (XREAL) -> xreal_to_xbit (INTEGRATED, optional white edge jitter)
//                    -> xbit_to_logic -> fb_rise (logic)
//   FB_JITTER_EN / FB_JITTER_SIGMA switch noise on the FB zero-crossings only.
// Reference:
//   ref_xbit_gen -> ref_clk / ref_rise (logic)
//
// Viva plot helpers (real-valued):
//   ref_rise, fb_rise            — logic levels into PFD
//   ref_xbit_at_t, fb_xbit_at_t  — plot copies of ref/fb levels
//   fb_fall_t / fb_ideal_t / fb_jitter_t -> fb_xbit_fall_times.csv
//     noisy hi->lo times, ideal grid at T_avg, TIE = noisy - ideal
//   icp_at_t                     — charge-pump current [A] via cp_eval_current_at_t()
//   lf_v_at_t, vco_fout_at_t     — eval_xreal_at_t()
//   pfd_state_at_t               — -1 DOWN, 0 ZERO, +1 UP
//   vco_rf_at_t                  — bounded cos (plot only; not in PFD path)
//
// Compile: irun -f compile.f   top: pll_full_tb
//
// Debug: VCO_DEBUG_MODE selects the VCO vin source (bypassing loop-filter output):
//   0 = closed loop (LF output drives VCO)
//   1 = unit step  (VCO_STEP_AMP)
//   2 = ramp       (slope VCO_RAMP_SLOPE, built as A*u(t) - A*exp(-t/tau), m=1 terms only)

`timescale 1ns / 10fs

import xreal_pkg::*;

module pll_full_tb;

    localparam int  VCO_DEBUG_MODE = 0;       // 0=closed loop, 1=step, 2=ramp
    localparam real VCO_STEP_AMP   = 1.0;
    // Ramp ~ VCO_RAMP_SLOPE * t (linear while t << VCO_RAMP_TAU)
    localparam real VCO_RAMP_SLOPE = 2.0e5;   // [V/s] -> 1 V at 5 us
    localparam real VCO_RAMP_TAU   = 50.0e-6; // [s] curvature constant (>> T_SIM)
    localparam real VCO_RAMP_AMP   = VCO_RAMP_SLOPE * VCO_RAMP_TAU;

    localparam real F_REF          = 5.0e9;   // Reference XBIT frequency [Hz]
    // PFD is armed on the FB edge after the first REF, so the first compared
    // pair is one period later. A delay of D from the enable edge then leaves
    // an UP pulse of (T_ref - D), not D. D=0.05 ns was a 150 ps / 270° step:
    // near the ±1-cycle PFD rail, so the loop cycle-slips (short UP run, then
    // a long DN run) and Vc is not a damped sine. 0.25 cycle = 90° stays linear.
    localparam real PHASE_STEP_CYC = 0.25;
    localparam real T_REF_NS       = 1.0e9 / F_REF;
    localparam real T_REF_START_NS = T_REF_NS * (1.0 - PHASE_STEP_CYC);
    // 500 kHz below F_REF. Lock Vctrl = df/Kvco = 10 mV. Whether this is a
    // 2nd-order ring or a pull-in ramp depends on lock-in range ~ Icp*Kvco*R.
    localparam real F_VCO          = 4.9995e9;
    localparam real K_VCO          = 50.0e6;
    localparam real I_CP           = 100.0e-6;
    // Series-R + C (+ optional C2 ripple).  ζ = (R/2)*sqrt(Icp*Kvco*C).
    // C=2nF, R=189.7 => ζ≈0.3, wn≈1.58e6, Td≈4.2 µs (several rings in T_SIM).
    // lock-in ≈ Icp*Kvco*R ≈ 0.95 MHz > 500 kHz offset (linear, not pull-in).
    // C2 = C/20.  PFD is armed on an FB edge after the first REF so the first
    // charge-pump pulse is UP (F_VCO is low) — releasing on REF made FB-first
    // DOWN and a −10 mV dip that hid the ring.
    // Lock Vctrl~(F_REF-F_VCO)/Kvco. 500 kHz / 50 MHz/V = 10 mV.
    // Seed Vc and VCO at this voltage so acquisition is a phase step (PFD
    // proportional) instead of a frequency pull (long UP then long DN ramps).
    localparam real VC_LOCK        = (F_REF - F_VCO) / K_VCO;
    localparam real CAP_LF         = 2.0e-9;         // integrating cap [F]
    localparam real CAP_C2         = 100.0e-12;      // ripple cap [F]
    // Leak must be a parasitic, not a loop element: it caps Vctrl at Iavg*TAU_LF/CAP_LF.
    localparam real TAU_LF         = 10.0e-3;        // leak time constant [s]
    localparam real POLE_LF        = -1.0 / TAU_LF;
    localparam real FILTER_C       = 1.0 / CAP_LF;   // gain 1/C [V/(A*s)]
    localparam real R_LF           = 189.7;          // [ohm] ζ≈0.3
    // eq. (5) tau: CP current rise/fall time. Rounds the edges of the rectangular
    // pulse, so it must stay well under a PFD pulse (T_AND_NS up to a ref period).
    localparam real TAU_CP         = 10.0e-12;       // [s]
    localparam real T_SIM          = 30.0e-6;
    // At ~5 GHz, T/2~100ps — 0.05 ns keeps ~4 plot points per RF period. Finer
    // steps cost linearly in runtime (each one re-evaluates the XREAL sums) and
    // no longer set the fb fall-time resolution, which is now edge-driven.
    localparam real SAMPLE_SLOW    = 0.05;
    localparam real SAMPLE_STEP_RF = 0.05;
    localparam real V_RF_PEAK      = 0.5;
    // Future XBIT window: 2 edges/cycle * F * T_HORIZON must fit MAX_XBIT_EDGES
    // (512). At 5 GHz, T_HORIZON=40ns => ~400 edges. Keep-alive at T_HORIZON/2.
    localparam real FB_T_HORIZON   = 40.0e-9;
    // White VCO edge jitter (zero-crossing uncertainty) on FB XBIT path only.
    // FB_JITTER_EN=0 => ideal edges; =1 => t_edge += N(0, FB_JITTER_SIGMA^2).
    localparam bit  FB_JITTER_EN    = 1'b0;
    localparam real FB_JITTER_SIGMA = 5.0e-12; // RMS [s] (5 ps default when on)
    localparam int  FB_JITTER_SEED  = 1;
    // Hi-to-low crossings of fb_rise -> CSV. One fall per FB cycle over T_SIM,
    // plus headroom for lock overshoot above F_REF / F_VCO.
    localparam int  MAX_FB_FALL     = int'((F_REF > F_VCO ? F_REF : F_VCO) * T_SIM) + 10000;

    logic clk;
    logic rst_n;
    logic pfd_rst_n;
    logic ref_enable = 1'b0;
    bit   fb_live = 1'b0;
    bit   ref_seen_after_fb = 1'b0;
    bit   pfd_arm = 1'b0;

    logic fb_in_valid;
    logic fb_xbit_valid;
    logic ref_clk;
    logic ref_rise;
    logic fb_rise;
    logic fb_logic;
    logic pfd_valid;
    logic cp_cmd_valid;
    logic icp_valid;
    logic lf_in_valid;
    logic lf_out_valid;
    logic vco_in_valid;
    logic vco_freq_valid;

    logic cp_up_cmd;
    logic cp_down_cmd;

    int   pfd_state;
    real  t_event_s;
    real  fb_t_event_s;
    real  fb_phase_off;

    xbit_edge_t  fb_edges [0:MAX_XBIT_EDGES-1];
    int          fb_edge_count;
    logic [1:0]  fb_level_at_zero;

    xreal_term_t icp_terms  [0:MAX_XREAL_TERMS-1];
    int          icp_count;
    xreal_term_t lf_in_terms  [0:MAX_XREAL_TERMS-1];
    int          lf_in_count;
    xreal_term_t lf_out_terms [0:MAX_XREAL_TERMS-1];
    int          lf_out_count;
    xreal_term_t vco_vin_terms  [0:MAX_XREAL_TERMS-1];
    int          vco_vin_count;
    xreal_term_t vco_freq_terms [0:MAX_XREAL_TERMS-1];
    int          vco_freq_count;
    xreal_term_t vco_df_terms [0:MAX_XREAL_TERMS-1];
    int          vco_df_count;

    xbit_seq_t  fb_xbit_plot;
    xreal_seq_t icp_seq;
    xreal_seq_t lf_v_seq;
    xreal_seq_t vco_fout_seq;
    xreal_seq_t vco_delta_f_cached;

    real ref_xbit_at_t;
    real fb_xbit_at_t;
    real icp_at_t;
    real lf_v_at_t;
    real lf_vc_at_t;
    real vco_fout_at_t;
    real vco_rf_at_t;
    real pfd_state_at_t;
    real fb_phase_at_event;
    real t_now_s;

    // Vector of fb_xbit_at_t hi->lo mid-scale crossing times [s] (noisy)
    // Ideal grid uses average period; jitter = noisy - ideal (TIE)
    real fb_fall_t   [0:MAX_FB_FALL-1];
    real fb_ideal_t  [0:MAX_FB_FALL-1];
    real fb_jitter_t [0:MAX_FB_FALL-1];
    int  fb_fall_count;
    int  fb_fall_csv_fd;
    int  fb_fall_i;
    int  fb_n_cycles;
    real fb_period_sum;
    real fb_T_avg;
    real fb_jitter_sumsq;
    real fb_jitter_rms;

    bit fb_xbit_ready;
    bit icp_ready;
    bit lf_ready;
    bit vco_ready;

    bit fb_refresh_pending;
    bit vco_stim_pending;
    bit lf_stim_pending;
    bit fb_keepalive_req;
    bit fb_keepalive_armed;
    bit fb_ka_async;

    logic fb_xbit_valid_d1;

    real cp_t_event;
    int  li;
    int  ev_i;

    localparam real TWO_PI = 2.0 * 3.14159265358979323846;

    // RF phase bookkeeping: delta_f cache is rebased at every VCO event, so keep
    // the already-accumulated phase in an offset to stay continuous across swaps.
    real        rf_phi_off_cyc = 0.0;
    real        rf_phi_cyc;
    real        fix_t_ev;
    real        fix_phi_old;
    real        fix_phi_new;

    // Identical to pfd_valid (no NBA one-cycle delay).
    assign cp_cmd_valid = pfd_valid;

    // Keep ref_rise identical to free-running ref_clk (do not sample on system clk).
    assign ref_rise = ref_clk;

    // Closed-loop FB: XBIT edges scheduled onto continuous logic for the PFD.
    assign fb_rise = fb_logic;

    ref_xbit_gen #(
        .F_HZ       (F_REF),
        .T_START_NS (T_REF_START_NS)
    ) ref_gen (
        .enable  (ref_enable),
        .ref_clk (ref_clk)
    );

    // Hold PFD in reset until FB is live, REF has ticked once, then one more
    // FB edge. Releasing on the first REF made the next edge FB → DOWN.
    // Arming on FB means the next edge is REF → UP (F_VCO is below F_REF).
    assign pfd_rst_n = rst_n & pfd_arm;

    // XREAL fout -> XBIT (INTEGRATED = phase-accurate VCO feedback).
    // SQUARE+PHASE_SRC=0/1 would map the old xbit_from_xreal_freq*_ helpers.
    xreal_to_xbit #(
        .EDGE_MODE        (1),
        .PHASE_SRC        (1),
        .T_HORIZON        (FB_T_HORIZON),
        .PHASE_OFFSET_RAD (0.0),
        .ENABLE_JITTER    (FB_JITTER_EN),
        .JITTER_SIGMA_S   (FB_JITTER_SIGMA),
        .JITTER_SEED      (FB_JITTER_SEED)
    ) fb_xreal_to_xbit (
        .clk                 (clk),
        .rst_n               (rst_n),
        .vin_valid           (fb_in_valid),
        .freq_terms          (vco_freq_terms),
        .freq_term_count     (vco_freq_count),
        .t_event             (fb_t_event_s),
        .phase_rad           (fb_phase_off),
        .xbit_valid          (fb_xbit_valid),
        .xbit_edges          (fb_edges),
        .xbit_edge_count     (fb_edge_count),
        .level_at_zero       (fb_level_at_zero),
        .phase_rad_at_event  (fb_phase_at_event)
    );

    xbit_to_logic fb_xbit_to_logic (
        .clk             (clk),
        .rst_n           (rst_n),
        .xbit_valid      (fb_xbit_valid),
        .xbit_edges      (fb_edges),
        .xbit_edge_count (fb_edge_count),
        .level_at_zero   (fb_level_at_zero),
        .bit_out         (fb_logic)
    );

    // Reset delay must stay a small fraction of the compare period, else UP/DN have
    // no room left to widen and the PFD degenerates into a pure phase detector.
    // At F_REF=5 GHz (200 ps) 0.05 ns was 25% of the period; 0.01 ns is 5%.
    pfd #(
        .T_AND_NS (0.01)
    ) pfd_dut (
        .clk         (clk),
        .rst_n       (pfd_rst_n),
        .ref_rise    (ref_rise),
        .fb_rise     (fb_rise),
        .pfd_valid   (pfd_valid),
        .pfd_state   (pfd_state),
        .cp_up_cmd   (cp_up_cmd),
        .cp_down_cmd (cp_down_cmd)
    );

    charge_pump #(
        .I_UP   (I_CP),
        .I_DOWN (-I_CP),
        .TAU_CP (TAU_CP)
    ) cp_dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .cmd_valid     (cp_cmd_valid),
        .cp_up_cmd     (cp_up_cmd),
        .cp_down_cmd   (cp_down_cmd),
        .t_event       (cp_t_event),
        .icp_in_terms  (icp_terms),
        .icp_in_count  (icp_count),
        .icp_valid     (icp_valid),
        .icp_out_terms (icp_terms),
        .icp_out_count (icp_count)
    );

    loop_filter #(
        .FILTER_C  (FILTER_C),
        .FILTER_P  (POLE_LF),
        .FILTER_N  (1),
        .FILTER_R  (R_LF),
        .FILTER_C2 (CAP_C2),
        .VC_INIT   (VC_LOCK)
    ) lf_dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .in_valid       (lf_in_valid),
        .in_terms       (lf_in_terms),
        .in_term_count  (lf_in_count),
        .out_valid      (lf_out_valid),
        .out_terms      (lf_out_terms),
        .out_term_count (lf_out_count)
    );

    vco #(
        .K_VCO (K_VCO),
        .F0    (F_VCO)
    ) vco_dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .vin_valid       (vco_in_valid),
        .vin_terms       (vco_vin_terms),
        .vin_term_count  (vco_vin_count),
        .t_event         (t_event_s),
        .freq_valid      (vco_freq_valid),
        .freq_terms      (vco_freq_terms),
        .freq_term_count (vco_freq_count),
        .delta_terms     (vco_df_terms),
        .delta_term_count(vco_df_count),
        .spectral_valid  (),
        .spec_out        (),
        .spec_out_count  ()
    );

    initial clk = 1'b0;
    always #1ns clk = ~clk;

    function automatic void pack_xbit_from_ports(
        output xbit_seq_t seq,
        input  xbit_edge_t edges [0:MAX_XBIT_EDGES-1],
        input  int         edge_count,
        input  logic [1:0] lvl_at_zero
    );
        begin
            xbit_clear(seq);
            seq.level_at_zero = xbit_val_e'(lvl_at_zero);
            seq.count = edge_count;
            for (int i = 0; i < edge_count; i++) begin
                seq.ev[i].t_edge = edges[i].t_edge;
                seq.ev[i].level  = xbit_val_e'(edges[i].level);
            end
        end
    endfunction

    function automatic void refresh_icp_seq_from_ports();
        begin
            icp_seq.count = icp_count;
            for (li = 0; li < icp_count; li = li + 1)
                icp_seq.terms[li] = icp_terms[li];
        end
    endfunction

    // fout terms rebase f0 to the last VCO event, so integral(fout) misses F0*t
    // already accumulated.  Keepalive must continue the true VCO cycle count or
    // FB gains ~1/T_HORIZON extra Hertz and Vctrl locks at -that/Kvco (~-0.5 V).
    function automatic real fb_phase_align_rad(input real t);
        real true_cyc, model_cyc, diff;
        begin
            true_cyc  = F_VCO * t + rf_phi_off_cyc
                      + integrate_xreal_m1_at_t(vco_delta_f_cached, t);
            model_cyc = integrate_xreal_m1_at_t(vco_fout_seq, t);
            diff = true_cyc - model_cyc;
            diff = diff - $floor(diff);
            if (diff < 0.0)
                diff = diff + 1.0;
            fb_phase_align_rad = TWO_PI * diff;
        end
    endfunction

    // Drive LF+VCO from a PFD/CP edge (true event time).  The 2 ns system clock
    // aliases 5 GHz UP/DN pulses into a DC I_CP ramp and kills the R zero.
    task automatic commit_analog_from_cp();
        begin
            lf_in_count = icp_count;
            for (ev_i = 0; ev_i < icp_count; ev_i = ev_i + 1)
                lf_in_terms[ev_i] = icp_terms[ev_i];
            lf_dut.apply_event();

            lf_v_seq.count = lf_out_count;
            for (ev_i = 0; ev_i < lf_out_count; ev_i = ev_i + 1)
                lf_v_seq.terms[ev_i] = lf_out_terms[ev_i];
            lf_ready = 1'b1;

            if (VCO_DEBUG_MODE == 0) begin
                fix_t_ev    = cp_t_event;
                fix_phi_old = vco_ready ? integrate_xreal_m1_at_t(vco_delta_f_cached, fix_t_ev) : 0.0;

                vco_vin_count = lf_out_count;
                for (ev_i = 0; ev_i < lf_out_count; ev_i = ev_i + 1)
                    vco_vin_terms[ev_i] = lf_out_terms[ev_i];
                t_event_s = cp_t_event;
                vco_dut.apply_vin();

                vco_fout_seq.count = vco_freq_count;
                for (ev_i = 0; ev_i < vco_freq_count; ev_i = ev_i + 1)
                    vco_fout_seq.terms[ev_i] = vco_freq_terms[ev_i];

                vco_delta_f_cached.count = vco_df_count;
                for (ev_i = 0; ev_i < vco_df_count; ev_i = ev_i + 1)
                    vco_delta_f_cached.terms[ev_i] = vco_df_terms[ev_i];

                fix_phi_new     = integrate_xreal_m1_at_t(vco_delta_f_cached, fix_t_ev);
                rf_phi_off_cyc += fix_phi_old - fix_phi_new;

                if (!vco_ready) begin
                    fb_t_event_s       = fix_t_ev;
                    fb_phase_off       = fb_phase_align_rad(fix_t_ev);
                    fb_refresh_pending = 1'b1;
                end
                vco_ready = 1'b1;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fb_keepalive_armed <= 1'b0;
            fb_ka_async        = 1'b0;
            lf_in_valid        <= 1'b0;
            vco_in_valid       <= 1'b0;
            fb_in_valid        <= 1'b0;
            fb_refresh_pending <= 1'b0;
            vco_stim_pending   <= 1'b0;
            lf_stim_pending    <= 1'b0;
        end else begin
            fb_in_valid  <= 1'b0;

            // One-cycle LF stimulus — count/terms must be NBA-settled first
            if (lf_stim_pending) begin
                lf_in_valid     <= 1'b1;
                lf_stim_pending <= 1'b0;
            end else begin
                lf_in_valid <= 1'b0;
            end

            // One-cycle VCO stimulus (single NBA driver — avoids racing initial blocking assign)
            if (vco_stim_pending) begin
                vco_in_valid      <= 1'b1;
                vco_stim_pending  <= 1'b0;
            end else begin
                vco_in_valid <= 1'b0;
            end

            // Timer only arms. Reload on a real async fb_logic edge (not a clock
            // sample of it): at ~5 GHz the 2 ns clk aliases fb, so
            // (fb_logic != fb_logic_d1) can stay false for longer than T_HORIZON
            // and the scheduled edges expire.  Timeout below force-refreshes if
            // still armed after a full keepalive period.
            if (fb_keepalive_req && vco_ready) begin
                fb_keepalive_armed <= 1'b1;
                fb_keepalive_req   = 1'b0;
            end

            if (fb_refresh_pending) begin
                fb_phase_off       <= fb_phase_align_rad(fb_t_event_s);
                fb_in_valid        <= 1'b1;
                fb_refresh_pending <= 1'b0;
            end else if (fb_keepalive_armed && fb_ka_async) begin
                fb_t_event_s       <= $realtime * 1.0e-9;
                fb_phase_off       <= fb_phase_align_rad($realtime * 1.0e-9);
                fb_in_valid        <= 1'b1;
                fb_keepalive_armed <= 1'b0;
                fb_ka_async        = 1'b0;
            end

            // Drain CP result into loop filter (CP applied async on UP/DN edges)
            if (icp_valid) begin
                refresh_icp_seq_from_ports();
                icp_ready = 1'b1;
                // Analog path is event-driven from apply_cmd; do not re-feed LF
                // on the 2 ns clock (that stretched I_CP into a DC ramp).
                cp_dut.clear_valid();
            end

            if (lf_out_valid) begin
                // Closed-loop LF/VCO already committed on the CP edge.
                if (VCO_DEBUG_MODE != 0) begin
                    lf_v_seq.count = lf_out_count;
                    for (li = 0; li < lf_out_count; li = li + 1)
                        lf_v_seq.terms[li] = lf_out_terms[li];
                    lf_ready = 1'b1;
                end
            end

            if (vco_freq_valid && !vco_ready) begin
                fix_t_ev    = $realtime * 1.0e-9;
                fix_phi_old = vco_ready ? integrate_xreal_m1_at_t(vco_delta_f_cached, fix_t_ev) : 0.0;

                vco_fout_seq.count = vco_freq_count;
                for (li = 0; li < vco_freq_count; li = li + 1)
                    vco_fout_seq.terms[li] = vco_freq_terms[li];

                // Arm fb XBIT once on first fout; later refreshes are keepalive-only.
                // Reloading xbit_to_logic on every VCO update warps duty cycle.
                if (!vco_ready) begin
                    fb_t_event_s       <= fix_t_ev;
                    fb_phase_off       <= fb_phase_align_rad(fix_t_ev);
                    fb_refresh_pending <= 1'b1;
                end
                vco_ready = 1'b1;

                // delta_f from the VCO's own synchronous port (terms + count are NBA
                // outputs of the same event — no mixing with a newer vin commit).
                vco_delta_f_cached.count = vco_df_count;
                for (li = 0; li < vco_df_count; li = li + 1)
                    vco_delta_f_cached.terms[li] = vco_df_terms[li];

                // Phase continuity: absorb the old-vs-new integral mismatch at t_ev.
                fix_phi_new     = integrate_xreal_m1_at_t(vco_delta_f_cached, fix_t_ev);
                rf_phi_off_cyc += fix_phi_old - fix_phi_new;
            end
        end
    end

    // Real fb edges (5 GHz) — clock-sampled comparison aliases and misses them.
    always @(fb_logic) begin
        if (rst_n && fb_keepalive_armed)
            fb_ka_async = 1'b1;
    end

    // Refresh XBIT before the scheduled edge window expires (latest fout terms).
    initial begin
        fb_keepalive_req = 1'b0;
        forever begin
            #(FB_T_HORIZON * 0.5 * 1s);
            if (rst_n && vco_ready)
                fb_keepalive_req = 1'b1;
            // Still armed after a full keepalive period: force a reload so a
            // frozen/aliased fb cannot expire the XBIT window.
            if (rst_n && fb_keepalive_armed)
                fb_ka_async = 1'b1;
        end
    end

    // fb hi->lo times, taken from the scheduled edge itself: exact to the 1 ps
    // timescale instead of being quantised by the plot sampling period.
    always @(negedge fb_rise) begin
        if (rst_n && fb_fall_count < MAX_FB_FALL) begin
            fb_fall_t[fb_fall_count] = $realtime * 1.0e-9;
            fb_fall_count            = fb_fall_count + 1;
        end
    end

    // Apply CP on every PFD command change (captures UP-only, DN-only, both-high, and reset-to-0)
    always @(cp_up_cmd or cp_down_cmd) begin
        if (pfd_rst_n) begin
            pfd_state_at_t = real'(pfd_state);
            cp_t_event     = $realtime * 1.0e-9;
            cp_dut.apply_cmd(cp_up_cmd, cp_down_cmd, cp_t_event);
            commit_analog_from_cp();
            cp_dut.clear_valid();
        end
    end

    always @(posedge ref_clk) begin
        if (rst_n && fb_live)
            ref_seen_after_fb <= 1'b1;
    end

    always @(posedge fb_rise or negedge rst_n) begin
        if (!rst_n)
            pfd_arm <= 1'b0;
        else if (ref_seen_after_fb && !pfd_arm) begin
            pfd_arm <= 1'b1;
        end
    end

    always @(posedge fb_rise) begin
        if (rst_n && !fb_live) begin
            fb_live    = 1'b1;
            ref_enable = 1'b1;
        end
    end

    // Latch fb XBIT one cycle after xbit_valid so NBA edge ports are settled.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fb_xbit_valid_d1 <= 1'b0;
        end else begin
            fb_xbit_valid_d1 <= fb_xbit_valid;
            if (fb_xbit_valid_d1) begin
                pack_xbit_from_ports(
                    fb_xbit_plot, fb_edges, fb_edge_count, fb_level_at_zero
                );
            end
        end
    end

    initial begin
        ref_xbit_at_t = 0; fb_xbit_at_t = 0;
        icp_at_t = 0;
        lf_v_at_t = 0; lf_vc_at_t = 0; vco_fout_at_t = 0;
        pfd_state_at_t = 0;
        fb_fall_count = 0;
        forever begin
            #(SAMPLE_SLOW * 1ns);
            t_now_s = $realtime * 1.0e-9;
            ref_xbit_at_t = ref_clk ? 1.0 : 0.0;
            fb_xbit_at_t  = fb_rise ? 1.0 : 0.0;
            if (fb_xbit_plot.count > 0)
                fb_xbit_ready = 1'b1;
            if (icp_count > 0) begin
                refresh_icp_seq_from_ports();
                icp_at_t = cp_eval_current_at_t(icp_seq, t_now_s);
            end
            if (lf_ready) begin
                lf_v_at_t = eval_xreal_at_t(lf_v_seq, t_now_s);
                lf_vc_at_t = eval_xreal_at_t(lf_dut.vc_state, t_now_s);
            end
            if (vco_ready)
                vco_fout_at_t = eval_xreal_at_t(vco_fout_seq, t_now_s);
        end
    end

    initial begin
        t_now_s     = 0.0;
        rf_phi_cyc  = 0.0;
        vco_rf_at_t = V_RF_PEAK;
        forever begin
            #(SAMPLE_STEP_RF * 1ns);
            t_now_s = $realtime * 1.0e-9;
            rf_phi_cyc = 0.0;
            if (vco_ready)
                rf_phi_cyc = rf_phi_off_cyc
                           + integrate_xreal_m1_at_t(vco_delta_f_cached, t_now_s);
            vco_rf_at_t = V_RF_PEAK * $cos(TWO_PI * (F_VCO * t_now_s + rf_phi_cyc));
        end
    end

    // Waveforms go to the SHM database only (see the $shm_open block below).
    // The VCD used to hold the same signals a second time (~528 MB per run);
    // re-add $dumpfile/$dumpvars here if a VCD-only viewer is needed.

    initial begin
        xreal_seq_t vco_step_in;

        rst_n = 0;
        ref_enable = 0;
        fb_live = 0;
        ref_seen_after_fb = 0;
        pfd_arm = 0;
        fb_in_valid = 0;
        fb_xbit_ready = 0;
        icp_ready = 0; lf_ready = 0; vco_ready = 0;
        fb_refresh_pending = 0;
        vco_stim_pending = 0;
        lf_stim_pending = 0;
        fb_keepalive_req = 0;
        fb_keepalive_armed = 0;
        fb_ka_async = 0;
        fb_phase_off    = 0.0;

        repeat (5) @(posedge clk);
        rst_n = 1;

        // Seed LF capacitor + VCO at lock voltage so fout starts at F_REF.
        // A frequency pull saturates the PFD (long UP, then long DN) and makes
        // vctrl a triangle; a phase step keeps UP/DN proportional (more sine).
        if (VCO_DEBUG_MODE == 0) begin
            lf_dut.seed_init();
            lf_v_seq.count = lf_out_count;
            for (li = 0; li < lf_out_count; li = li + 1)
                lf_v_seq.terms[li] = lf_out_terms[li];
            lf_ready = 1'b1;

            xreal_clear(vco_step_in);
            xreal_add_unit_step(vco_step_in, VC_LOCK, 0.0);
            vco_vin_count = vco_step_in.count;
            for (li = 0; li < vco_step_in.count; li = li + 1)
                vco_vin_terms[li] = vco_step_in.terms[li];
            t_event_s = 0.0;
            @(posedge clk);
            vco_stim_pending = 1'b1;
            @(posedge clk);
            @(posedge clk);
        end

        if (VCO_DEBUG_MODE != 0) begin
            xreal_clear(vco_step_in);
            if (VCO_DEBUG_MODE == 1) begin
                xreal_add_unit_step(vco_step_in, VCO_STEP_AMP, 0.0);
            end else begin
                // ramp: A*u(t) - A*exp(-t/tau) ~= (A/tau)*t = VCO_RAMP_SLOPE*t
                xreal_add_unit_step(vco_step_in, VCO_RAMP_AMP, 0.0);
                xreal_add_exponential(vco_step_in, -VCO_RAMP_AMP, -1.0 / VCO_RAMP_TAU, 0.0);
            end
            vco_vin_count = vco_step_in.count;
            for (li = 0; li < vco_step_in.count; li = li + 1)
                vco_vin_terms[li] = vco_step_in.terms[li];
            t_event_s = 0.0;
            @(posedge clk);
            vco_stim_pending = 1'b1;
            @(posedge clk);
            @(posedge clk);
            @(posedge clk);
            xreal_print("DEBUG VCO vin (bypass LF)", vco_step_in);
        end

        $display("============================================================");
        if (VCO_DEBUG_MODE == 1)
            $display(" PLL TB — DEBUG: VCO vin = unit step (%0.3f V), LF disconnected", VCO_STEP_AMP);
        else if (VCO_DEBUG_MODE == 2)
            $display(" PLL TB — DEBUG: VCO vin = ramp (%0.3e V/s), LF disconnected", VCO_RAMP_SLOPE);
        else
            $display(" PLL full chain — PFD / CP / LF / VCO / xreal_to_xbit / xbit_to_logic");
        $display(" F_REF=%0.3e Hz  F_VCO=%0.3e Hz  I_CP=%0.3e A  R_LF=%0.3e ohm  C=%0.3e F  C2=%0.3e F",
                 F_REF, F_VCO, I_CP, R_LF, CAP_LF, CAP_C2);
        $display(" 2nd-order: wn=%0.3e rad/s  zeta=%0.3f  lock-in df~%0.3e Hz  init df=%0.3e Hz",
                 (I_CP * K_VCO / CAP_LF) ** 0.5,
                 0.5 * R_LF * ((I_CP * K_VCO * CAP_LF) ** 0.5),
                 I_CP * K_VCO * R_LF, F_REF - F_VCO);
        $display(" lock Vctrl~(F_REF-F_VCO)/Kvco = %0.4f V", (F_REF - F_VCO) / K_VCO);
        $display(" ramp I/C = %0.3e V/s   CP hold kick Kvco*Icp*R = %0.3e Hz   I*R = %0.3e V",
                 I_CP / CAP_LF, K_VCO * I_CP * R_LF, I_CP * R_LF);
        $display(" fb path: VCO fout -> xreal_to_xbit(INTEGRATED) -> xbit_to_logic -> PFD");
        if (FB_JITTER_EN)
            $display(" FB white edge jitter ON  sigma=%0.3e s  seed=%0d",
                     FB_JITTER_SIGMA, FB_JITTER_SEED);
        else
            $display(" FB white edge jitter OFF");
        $display(" ref_clk starts when fb_rise is live, plus T_REF_START_NS phase offset");
        $display(" icp_at_t = charge-pump current [A]");
        $display(" Viva: ref_xbit_at_t fb_xbit_at_t icp_at_t lf_v_at_t lf_vc_at_t vco_fout_at_t vco_rf_at_t");
        $display("------------------------------------------------------------");
        $display(" SIM_START  target_T_SIM = %0.6e s  (%0.3f us)", T_SIM, T_SIM * 1.0e6);
        $display(" SIM_START  sim_time     = %0.9f s  ($realtime=%0t)", $realtime / 1s, $realtime);
        $display(" SIM_START  wall_clock:");
        $system("date");
        $display("============================================================");

        #(T_SIM * 1s);

        // Build ideal period grid + TIE jitter; dump noisy / ideal / jitter CSV
        if (fb_fall_count >= 2) begin
            fb_n_cycles   = fb_fall_count - 1;
            // Average period = sum of consecutive periods / number of cycles
            // (equals (t_last - t_first) / (N-1))
            fb_period_sum = 0.0;
            for (fb_fall_i = 0; fb_fall_i < fb_n_cycles; fb_fall_i = fb_fall_i + 1)
                fb_period_sum = fb_period_sum
                              + (fb_fall_t[fb_fall_i + 1] - fb_fall_t[fb_fall_i]);
            fb_T_avg = fb_period_sum / real'(fb_n_cycles);

            fb_jitter_sumsq = 0.0;
            for (fb_fall_i = 0; fb_fall_i < fb_fall_count; fb_fall_i = fb_fall_i + 1) begin
                fb_ideal_t[fb_fall_i]  = fb_fall_t[0] + real'(fb_fall_i) * fb_T_avg;
                fb_jitter_t[fb_fall_i] = fb_fall_t[fb_fall_i] - fb_ideal_t[fb_fall_i];
                fb_jitter_sumsq = fb_jitter_sumsq
                                + fb_jitter_t[fb_fall_i] * fb_jitter_t[fb_fall_i];
            end
            fb_jitter_rms = $sqrt(fb_jitter_sumsq / real'(fb_fall_count));

            fb_fall_csv_fd = $fopen("fb_xbit_fall_times.csv", "w");
            if (fb_fall_csv_fd == 0) begin
                $display(" ERROR: could not open fb_xbit_fall_times.csv for write");
            end else begin
                $fwrite(fb_fall_csv_fd, "idx,t_noisy_s,t_ideal_s,jitter_s\n");
                for (fb_fall_i = 0; fb_fall_i < fb_fall_count; fb_fall_i = fb_fall_i + 1)
                    $fwrite(fb_fall_csv_fd, "%0d,%0.16e,%0.16e,%0.16e\n",
                            fb_fall_i,
                            fb_fall_t[fb_fall_i],
                            fb_ideal_t[fb_fall_i],
                            fb_jitter_t[fb_fall_i]);
                $fclose(fb_fall_csv_fd);
                $display(" Wrote %0d rows to fb_xbit_fall_times.csv", fb_fall_count);
                $display(" T_avg = %0.16e s  (%0.6f GHz)  from %0d cycles",
                         fb_T_avg, 1.0e-9 / fb_T_avg, fb_n_cycles);
                $display(" TIE RMS (noisy - ideal) = %0.6e s  (%0.3f ps)",
                         fb_jitter_rms, fb_jitter_rms * 1.0e12);
                if (fb_fall_count >= MAX_FB_FALL)
                    $display(" WARNING: fb_fall vector full (MAX_FB_FALL=%0d); increase limit",
                             MAX_FB_FALL);
            end
        end else begin
            $display(" WARNING: need >= 2 fall edges for T_avg / jitter (got %0d)",
                     fb_fall_count);
        end

        $display("============================================================");
        $display(" SIM_END    sim_time     = %0.9f s  (%0.3f us)", $realtime / 1s, ($realtime / 1s) * 1.0e6);
        $display(" SIM_END    $realtime    = %0t", $realtime);
        $display(" SIM_END    wall_clock:");
        $system("date");
        $display(" (For machine runtime see irun.log 'total:' or shell 'time irun ...')");
        $display("============================================================");
        $finish;
    end

    // Probe the plot signals explicitly. $shm_probe("AS") recorded every signal in
    // the design (including the fb_fall_* vectors and the 512-entry edge arrays).
    initial begin
        $shm_open("waves_new.shm");
        $shm_probe(ref_xbit_at_t, ref_clk, fb_xbit_at_t, ref_rise, fb_rise,
                   icp_at_t, lf_v_at_t, lf_vc_at_t, vco_fout_at_t, vco_rf_at_t,
                   pfd_state_at_t, fb_phase_at_event);
        $shm_probe(pfd_dut.ref_seen, pfd_dut.fb_seen);
    end
endmodule
