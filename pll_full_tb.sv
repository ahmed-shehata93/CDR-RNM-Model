// Full PLL TB: ref/fb XBIT -> PFD -> CP -> LF -> VCO -> fb buffer
//
// Viva plot helpers (real-valued):
//   ref_rise, fb_rise           — XBIT levels sampled on posedge clk (blocking, aligned with PFD)
//   ref_xbit_at_t, fb_xbit_at_t  — eval_xbit_at_t() plot copies of ref_rise/fb_rise
//   icp_at_t                     — charge-pump current [A] via cp_eval_current_at_t()
//   lf_v_at_t, vco_fout_at_t     — eval_xreal_at_t()
//   pfd_state_at_t               — -1 DOWN, 0 ZERO, +1 UP
//   vco_rf_at_t                  — bounded cos(omega_0*t + phi(t))
//
// Compile: irun -f compile.f   top: pll_full_tb
//
// Debug: set VCO_STEP_DEBUG=1 to drive VCO from a unit step (like pll_lf_vco_tb x_in),
//        bypassing loop-filter output. Set to 0 for closed-loop PLL.

`timescale 1ns / 1ps

import xreal_pkg::*;

module pll_full_tb;

    localparam bit  VCO_STEP_DEBUG = 1'b1;
    localparam real VCO_STEP_AMP   = 1.0;

    localparam real F_REF          = 60.0e6;  // Reference XBIT frequency [Hz]
    localparam real F_VCO          = 10.0e6;  // VCO center (free-running) frequency [Hz]
    localparam real K_VCO          = 50.0e6;
    localparam real TAU_LF         = 1.0e-6;
    localparam real POLE_LF        = -1.0 / TAU_LF;
    localparam real FILTER_C       = -POLE_LF;
    localparam real I_CP           = 100.0e-6;
    localparam real TAU_CP         = 100.0e-9;
    localparam real T_SIM          = 5.0e-6;
    localparam real SAMPLE_SLOW    = 0.1;
    localparam real SAMPLE_STEP_RF = 0.5;
    localparam real V_RF_PEAK      = 0.5;

    logic clk;
    logic rst_n;

    logic ref_in_valid;
    logic ref_xbit_valid;
    logic fb_in_valid;
    logic fb_xbit_valid;
    logic ref_rise;
    logic fb_rise;
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

    xreal_term_t freq_init_terms [0:MAX_XREAL_TERMS-1];
    int          freq_init_count;

    xbit_edge_t  ref_edges [0:MAX_XBIT_EDGES-1];
    int          ref_edge_count;
    logic [1:0]  ref_level_at_zero;

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

    xbit_seq_t  ref_xbit_plot;
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

    bit xbit_ready;
    bit fb_xbit_ready;
    bit icp_ready;
    bit lf_ready;
    bit vco_ready;

    bit fb_refresh_pending;

    logic ref_xbit_valid_d1;
    logic fb_xbit_valid_d1;

    real cp_t_event;
    int  li;

    ref_xbit_gen #(
        .T_STOP       (T_SIM),
        .PHASE_OFFSET (0.7)
    ) ref_gen (
        .clk             (clk),
        .rst_n           (rst_n),
        .fin_valid       (ref_in_valid),
        .freq_terms      (freq_init_terms),
        .freq_term_count (freq_init_count),
        .t_event         (t_event_s),
        .xbit_valid      (ref_xbit_valid),
        .xbit_edges      (ref_edges),
        .xbit_edge_count (ref_edge_count),
        .level_at_zero   (ref_level_at_zero)
    );

    freq_xbit_buffer #(
        .T_STOP (T_SIM)
    ) fb_buf (
        .clk                 (clk),
        .rst_n               (rst_n),
        .vin_valid           (fb_in_valid),
        .freq_terms          (vco_freq_terms),
        .freq_term_count     (vco_freq_count),
        .t_event             (fb_t_event_s),
        .xbit_valid          (fb_xbit_valid),
        .xbit_edges          (fb_edges),
        .xbit_edge_count     (fb_edge_count),
        .level_at_zero       (fb_level_at_zero),
        .phase_rad_at_event  (fb_phase_at_event)
    );

    pfd pfd_dut (
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
        .FILTER_C (FILTER_C),
        .FILTER_P (POLE_LF),
        .FILTER_N (1)
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
            ref_rise           = 1'b0;
            fb_rise            = 1'b0;
            cp_cmd_valid       <= 1'b0;
            lf_in_valid        <= 1'b0;
            vco_in_valid       <= 1'b0;
            fb_in_valid        <= 1'b0;
            fb_refresh_pending <= 1'b0;
        end else begin
            if (ref_xbit_plot.count > 0)
                ref_rise = (eval_xbit_at_t(ref_xbit_plot, $realtime * 1.0e-9) >= 0.5);
            else
                ref_rise = 1'b0;
            if (fb_xbit_plot.count > 0)
                fb_rise = (eval_xbit_at_t(fb_xbit_plot, $realtime * 1.0e-9) >= 0.5);
            else
                fb_rise = 1'b0;
            pfd_dut.clock_tick(ref_rise, fb_rise);

            lf_in_valid  <= 1'b0;
            vco_in_valid <= 1'b0;
            fb_in_valid  <= 1'b0;
            cp_cmd_valid <= 1'b0;

            if (fb_refresh_pending) begin
                fb_in_valid        <= 1'b1;
                fb_refresh_pending <= 1'b0;
            end

            if (pfd_valid) begin
                pfd_state_at_t <= real'(pfd_state);
                cp_t_event     <= $realtime * 1.0e-9;
                cp_cmd_valid   <= 1'b1;
            end

            if (icp_valid) begin
                refresh_icp_seq_from_ports();
                icp_ready = 1'b1;

                lf_in_count = icp_count;
                for (li = 0; li < icp_count; li = li + 1)
                    lf_in_terms[li] <= icp_terms[li];
                lf_in_valid <= 1'b1;
            end

            if (lf_out_valid) begin
                lf_v_seq.count = lf_out_count;
                for (li = 0; li < lf_out_count; li = li + 1)
                    lf_v_seq.terms[li] = lf_out_terms[li];
                lf_ready = 1'b1;

                if (!VCO_STEP_DEBUG) begin
                    vco_vin_count = lf_out_count;
                    for (li = 0; li < lf_out_count; li = li + 1)
                        vco_vin_terms[li] <= lf_out_terms[li];
                    t_event_s    <= $realtime * 1.0e-9;
                    vco_in_valid <= 1'b1;
                end
            end

            if (vco_freq_valid) begin
                vco_fout_seq.count = vco_freq_count;
                for (li = 0; li < vco_freq_count; li = li + 1)
                    vco_fout_seq.terms[li] = vco_freq_terms[li];
                vco_ready          = 1'b1;
                vco_delta_f_cached = xreal_delta_f(vco_fout_seq, F_VCO);

                fb_t_event_s       <= $realtime * 1.0e-9;
                fb_refresh_pending <= 1'b1;
            end
        end
    end

    // Latch ref/fb XBIT one cycle after xbit_valid so NBA edge ports are settled.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ref_xbit_valid_d1 <= 1'b0;
            fb_xbit_valid_d1  <= 1'b0;
        end else begin
            ref_xbit_valid_d1 <= ref_xbit_valid;
            fb_xbit_valid_d1  <= fb_xbit_valid;
            if (ref_xbit_valid_d1) begin
                pack_xbit_from_ports(
                    ref_xbit_plot, ref_edges, ref_edge_count, ref_level_at_zero
                );
            end
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
            if (ref_xbit_plot.count > 0)
                ref_xbit_at_t = eval_xbit_at_t(ref_xbit_plot, t_now_s);
            if (fb_xbit_plot.count > 0) begin
                fb_xbit_ready = 1'b1;
                fb_xbit_at_t  = eval_xbit_at_t(fb_xbit_plot, t_now_s);
            end
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
            if (vco_ready)
                vco_rf_at_t = eval_vco_rf_at_t(
                    F_VCO, vco_delta_f_cached, t_now_s, V_RF_PEAK
                );
        end
    end

    initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, pll_full_tb.ref_xbit_at_t);
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
        xreal_seq_t ref_freq_seq;
        xreal_seq_t vco_step_in;

        rst_n = 0;
        ref_rise = 0; fb_rise = 0;
        ref_in_valid = 0; fb_in_valid = 0;
        freq_init_count = 0;
        xbit_ready = 0; fb_xbit_ready = 0;
        icp_ready = 0; lf_ready = 0; vco_ready = 0;
        fb_refresh_pending = 0;

        repeat (5) @(posedge clk);
        rst_n = 1;

        xreal_clear(ref_freq_seq);
        xreal_add_term(ref_freq_seq, F_REF, 0.0, 1, 0.0);
        freq_init_count = ref_freq_seq.count;
        for (li = 0; li < ref_freq_seq.count; li = li + 1)
            freq_init_terms[li] = ref_freq_seq.terms[li];

        t_event_s = 0.0;
        @(posedge clk);
        ref_in_valid = 1'b1;
        @(posedge clk);
        ref_in_valid = 1'b0;
        @(posedge clk);

        wait (ref_xbit_plot.count > 0);
        xbit_ready = 1'b1;

        if (VCO_STEP_DEBUG) begin
            xreal_clear(vco_step_in);
            xreal_add_unit_step(vco_step_in, VCO_STEP_AMP, 0.0);
            vco_vin_count = vco_step_in.count;
            for (li = 0; li < vco_step_in.count; li = li + 1)
                vco_vin_terms[li] = vco_step_in.terms[li];
            t_event_s = 0.0;
            @(posedge clk);
            vco_in_valid = 1'b1;
            @(posedge clk);
            vco_in_valid = 1'b0;
            xreal_print("DEBUG VCO step input (bypass LF)", vco_step_in);
        end

        xreal_print("Reference F_REF XREAL", ref_freq_seq);
        xbit_print("Reference XBIT", ref_xbit_plot);

        $display("============================================================");
        if (VCO_STEP_DEBUG)
            $display(" PLL TB — DEBUG: VCO vin = unit step (%0.3f V), LF disconnected", VCO_STEP_AMP);
        else
            $display(" PLL full chain — PFD / CP / LF / VCO (closed-loop fb)");
        $display(" F_REF=%0.3e Hz  F_VCO=%0.3e Hz  I_CP=%0.3e A", F_REF, F_VCO, I_CP);
        $display(" icp_at_t = charge-pump current [A]");
        $display(" Viva: ref_xbit_at_t fb_xbit_at_t icp_at_t lf_v_at_t vco_fout_at_t vco_rf_at_t");
        $display("============================================================");

        #(T_SIM * 1s);
        $finish;
    end

    initial begin
        $dumpfile("out.vcd");
        $shm_open("waves_new.shm"); $shm_probe("AS");
    end
endmodule
