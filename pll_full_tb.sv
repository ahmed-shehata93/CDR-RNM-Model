// Full PLL TB: ref/fb -> PFD -> CP -> LF -> VCO -> xreal_to_xbit -> xbit_to_logic
//
// Feedback path (closed loop):
//   VCO fout (XREAL) -> xreal_to_xbit (INTEGRATED) -> xbit_to_logic -> fb_rise (logic)
// Reference:
//   ref_xbit_gen -> ref_clk / ref_rise (logic)
//
// Viva plot helpers (real-valued):
//   ref_rise, fb_rise            — logic levels into PFD
//   ref_xbit_at_t, fb_xbit_at_t  — plot copies of ref/fb levels
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

`timescale 1ns / 1ps

import xreal_pkg::*;

module pll_full_tb;

    localparam int  VCO_DEBUG_MODE = 0;       // 0=closed loop, 1=step, 2=ramp
    localparam real VCO_STEP_AMP   = 1.0;
    // Ramp ~ VCO_RAMP_SLOPE * t (linear while t << VCO_RAMP_TAU)
    localparam real VCO_RAMP_SLOPE = 2.0e5;   // [V/s] -> 1 V at 5 us
    localparam real VCO_RAMP_TAU   = 50.0e-6; // [s] curvature constant (>> T_SIM)
    localparam real VCO_RAMP_AMP   = VCO_RAMP_SLOPE * VCO_RAMP_TAU;

    localparam real F_REF          = 60.0e6;  // Reference XBIT frequency [Hz]
    localparam real T_REF_START_NS = 0.0;     // ref first-edge delay vs fb [ns]
    localparam real F_VCO          = 59.0e6;  // VCO free-running center [Hz]
    localparam real K_VCO          = 50.0e6;
    localparam real I_CP           = 100.0e-6;
    // Series-R + C (+ optional C2 ripple).  ζ = (R/2)*sqrt(Icp*Kvco*C).
    // Tradeoff: large C => slow lock; large R => Kvco*I*R frequency kick / I*R pulses.
    // C=10nF, R=100: I/C=10mV/us (~20mV in 2us), kick=0.5MHz, I*R=10mV spikes.
    // C2 filters short ref-rate I*R spikes into VCO / lf_v.
    localparam real CAP_LF         = 10.0e-9;        // integrating cap [F]
    localparam real CAP_C2         = 2.0e-9;         // ripple cap [F]
    localparam real TAU_LF         = 100.0e-6;       // leak time constant [s]
    localparam real POLE_LF        = -1.0 / TAU_LF;
    localparam real FILTER_C       = 1.0 / CAP_LF;   // gain 1/C [V/(A*s)]
    localparam real R_LF           = 100.0;          // [ohm]
    localparam real TAU_CP         = 1.0e-9;
    localparam real T_SIM          = 50.2e-6;
    localparam real SAMPLE_SLOW    = 0.1;
    localparam real SAMPLE_STEP_RF = 0.1;
    localparam real V_RF_PEAK      = 0.5;
    // Future XBIT window per converter update (~2*f edges). Keep-alive below
    // re-arms before the window expires between sparse VCO events.
    localparam real FB_T_HORIZON   = 2.0e-6;

    logic clk;
    logic rst_n;

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
    real vco_fout_at_t;
    real vco_rf_at_t;
    real pfd_state_at_t;
    real fb_phase_at_event;
    real t_now_s;

    bit fb_xbit_ready;
    bit icp_ready;
    bit lf_ready;
    bit vco_ready;

    bit fb_refresh_pending;
    bit vco_stim_pending;
    bit lf_stim_pending;
    bit fb_keepalive_req;
    bit fb_keepalive_armed;
    logic fb_logic_d1;

    logic fb_xbit_valid_d1;

    real cp_t_event;
    int  li;

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
        .ref_clk (ref_clk)
    );

    // XREAL fout -> XBIT (INTEGRATED = phase-accurate VCO feedback).
    // SQUARE+PHASE_SRC=0/1 would map the old xbit_from_xreal_freq*_ helpers.
    xreal_to_xbit #(
        .EDGE_MODE        (1),
        .PHASE_SRC        (1),
        .T_HORIZON        (FB_T_HORIZON),
        .PHASE_OFFSET_RAD (0.0)
    ) fb_xreal_to_xbit (
        .clk                 (clk),
        .rst_n               (rst_n),
        .vin_valid           (fb_in_valid),
        .freq_terms          (vco_freq_terms),
        .freq_term_count     (vco_freq_count),
        .t_event             (fb_t_event_s),
        .phase_rad           (0.0),
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

    pfd #(
        .T_AND_NS (0.05)
    ) pfd_dut (
        .clk         (clk),
        .rst_n       (rst_n),
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
        .FILTER_C2 (CAP_C2)
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

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fb_keepalive_armed <= 1'b0;
            fb_logic_d1        <= 1'b0;
            lf_in_valid        <= 1'b0;
            vco_in_valid       <= 1'b0;
            fb_in_valid        <= 1'b0;
            fb_refresh_pending <= 1'b0;
            vco_stim_pending   <= 1'b0;
            lf_stim_pending    <= 1'b0;
        end else begin
            fb_in_valid  <= 1'b0;
            fb_logic_d1  <= fb_logic;

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

            // Timer only arms; actual XBIT reload waits for an fb_logic edge so we
            // never cancel a mid half-cycle (that caused ~55% duty every 1 us).
            if (fb_keepalive_req && vco_ready) begin
                fb_keepalive_armed <= 1'b1;
                fb_keepalive_req   = 1'b0;
            end

            if (fb_refresh_pending) begin
                fb_in_valid        <= 1'b1;
                fb_refresh_pending <= 1'b0;
            end else if (fb_keepalive_armed && (fb_logic != fb_logic_d1)) begin
                fb_t_event_s       <= $realtime * 1.0e-9;
                fb_in_valid        <= 1'b1;
                fb_keepalive_armed <= 1'b0;
            end

            // Drain CP result into loop filter (CP applied async on UP/DN edges)
            if (icp_valid) begin
                refresh_icp_seq_from_ports();
                icp_ready = 1'b1;

                // All NBA + pending: same race as VCO vin (blocking count + NBA terms
                // let LF see new count with old terms → unpaired ±10 V PF residues).
                lf_in_count      <= icp_count;
                for (li = 0; li < icp_count; li = li + 1)
                    lf_in_terms[li] <= icp_terms[li];
                lf_stim_pending  <= 1'b1;
                cp_dut.clear_valid();
            end

            if (lf_out_valid) begin
                lf_v_seq.count = lf_out_count;
                for (li = 0; li < lf_out_count; li = li + 1)
                    lf_v_seq.terms[li] = lf_out_terms[li];
                lf_ready = 1'b1;

                if (VCO_DEBUG_MODE == 0) begin
                    // All NBA: count/terms/t_event settle together before
                    // vco_stim_pending pulses vin_valid on the NEXT clock.
                    vco_vin_count     <= lf_out_count;
                    for (li = 0; li < lf_out_count; li = li + 1)
                        vco_vin_terms[li] <= lf_out_terms[li];
                    t_event_s         <= $realtime * 1.0e-9;
                    vco_stim_pending  <= 1'b1;
                end
            end

            if (vco_freq_valid) begin
                fix_t_ev    = $realtime * 1.0e-9;
                fix_phi_old = vco_ready ? integrate_xreal_m1_at_t(vco_delta_f_cached, fix_t_ev) : 0.0;

                vco_fout_seq.count = vco_freq_count;
                for (li = 0; li < vco_freq_count; li = li + 1)
                    vco_fout_seq.terms[li] = vco_freq_terms[li];

                // Arm fb XBIT once on first fout; later refreshes are keepalive-only.
                // Reloading xbit_to_logic on every VCO update warps duty cycle.
                if (!vco_ready) begin
                    fb_t_event_s       <= fix_t_ev;
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

    // Refresh XBIT before the scheduled edge window expires (latest fout terms).
    initial begin
        fb_keepalive_req = 1'b0;
        forever begin
            #(FB_T_HORIZON * 0.5 * 1s);
            if (rst_n && vco_ready)
                fb_keepalive_req = 1'b1;
        end
    end

    // Apply CP on every PFD command change (captures UP-only, DN-only, both-high, and reset-to-0)
    always @(cp_up_cmd or cp_down_cmd) begin
        if (rst_n) begin
            pfd_state_at_t = real'(pfd_state);
            cp_t_event     = $realtime * 1.0e-9;
            cp_dut.apply_cmd(cp_up_cmd, cp_down_cmd, cp_t_event);
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
        ref_xbit_at_t = 0; fb_xbit_at_t = 0; icp_at_t = 0;
        lf_v_at_t = 0; vco_fout_at_t = 0; vco_rf_at_t = 0;
        pfd_state_at_t = 0;
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
            if (lf_ready)
                lf_v_at_t = eval_xreal_at_t(lf_v_seq, t_now_s);
            if (vco_ready)
                vco_fout_at_t = eval_xreal_at_t(vco_fout_seq, t_now_s);
        end
    end

    initial begin
        forever begin
            #(SAMPLE_STEP_RF * 1ns);
            t_now_s = $realtime * 1.0e-9;
            if (vco_ready) begin
                // Same as eval_vco_rf_at_t, plus the cross-event phase offset.
                rf_phi_cyc  = rf_phi_off_cyc
                            + integrate_xreal_m1_at_t(vco_delta_f_cached, t_now_s);
                vco_rf_at_t = V_RF_PEAK * $cos(TWO_PI * (F_VCO * t_now_s + rf_phi_cyc));
            end
        end
    end

    initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, pll_full_tb.ref_xbit_at_t);
        $dumpvars(0, pll_full_tb.ref_clk);
        $dumpvars(0, pll_full_tb.fb_xbit_at_t);
        $dumpvars(0, pll_full_tb.ref_rise);
        $dumpvars(0, pll_full_tb.fb_rise);
        $dumpvars(0, pll_full_tb.icp_at_t);
        $dumpvars(0, pll_full_tb.lf_v_at_t);
        $dumpvars(0, pll_full_tb.vco_fout_at_t);
        $dumpvars(0, pll_full_tb.vco_rf_at_t);
        $dumpvars(0, pll_full_tb.pfd_state_at_t);
        $dumpvars(0, pll_full_tb.pfd_dut.ref_seen);
        $dumpvars(0, pll_full_tb.pfd_dut.fb_seen);
        $dumpvars(0, pll_full_tb.fb_phase_at_event);
    end

    initial begin
        xreal_seq_t vco_step_in;

        rst_n = 0;
        fb_in_valid = 0;
        fb_xbit_ready = 0;
        icp_ready = 0; lf_ready = 0; vco_ready = 0;
        fb_refresh_pending = 0;
        vco_stim_pending = 0;
        lf_stim_pending = 0;
        fb_keepalive_req = 0;
        fb_keepalive_armed = 0;
        fb_logic_d1 = 0;

        repeat (5) @(posedge clk);
        rst_n = 1;

        // Seed free-running VCO (vin=0 => fout=F0) so xreal_to_xbit / xbit_to_logic
        // can drive fb_rise before the first CP/LF event.
        if (VCO_DEBUG_MODE == 0) begin
            xreal_clear(vco_step_in);
            vco_vin_count = 0;
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
        $display(" lock Vctrl~(F_REF-F_VCO)/Kvco = %0.4f V", (F_REF - F_VCO) / K_VCO);
        $display(" ramp I/C = %0.3e V/s   CP hold kick Kvco*Icp*R = %0.3e Hz   I*R = %0.3e V",
                 I_CP / CAP_LF, K_VCO * I_CP * R_LF, I_CP * R_LF);
        $display(" fb path: VCO fout -> xreal_to_xbit(INTEGRATED) -> xbit_to_logic -> PFD");
        $display(" ref_clk = free-running square wave from ref_xbit_gen");
        $display(" icp_at_t = charge-pump current [A]");
        $display(" Viva: ref_xbit_at_t fb_xbit_at_t icp_at_t lf_v_at_t vco_fout_at_t vco_rf_at_t");
        $display("------------------------------------------------------------");
        $display(" SIM_START  target_T_SIM = %0.6e s  (%0.3f us)", T_SIM, T_SIM * 1.0e6);
        $display(" SIM_START  sim_time     = %0.9f s  ($realtime=%0t)", $realtime / 1s, $realtime);
        $display(" SIM_START  wall_clock:");
        $system("date");
        $display("============================================================");

        #(T_SIM * 1s);

        $display("============================================================");
        $display(" SIM_END    sim_time     = %0.9f s  (%0.3f us)", $realtime / 1s, ($realtime / 1s) * 1.0e6);
        $display(" SIM_END    $realtime    = %0t", $realtime);
        $display(" SIM_END    wall_clock:");
        $system("date");
        $display(" (For machine runtime see irun.log 'total:' or shell 'time irun ...')");
        $display("============================================================");
        $finish;
    end

    initial begin
        $dumpfile("out.vcd");
        $shm_open("waves_new.shm"); $shm_probe("AS");
    end
endmodule
