// PLL LF + VCO testbench — Sections IV-C and IV-D.
//
// Signal chain: I_CP (XREAL) -> loop_filter -> VCO
//
// Viva / SimVision scope signals:
//   x_at_t           — charge-pump input x(t)
//   lf_v_at_t        — loop-filter voltage y(t)
//   vco_fout_at_t    — VCO frequency fout(t)  [XREAL output, eq. 8]
//   vco_rf_at_t      — VCO RF approximation s(t) [spectral eq. 9]
//   vco_spec_at_t    — spectral sum from I/Q bins [eq. III-B]
//
// Compile: irun -f compile.f
// Import:  import xreal_pkg::*;   (package name, NOT file name 00_xreal_pkg)

`timescale 1ns / 1ps

import xreal_pkg::*;

module pll_lf_vco_tb;

    localparam real TAU         = 1.0e-6;
    localparam real POLE        = -1.0 / TAU;
    localparam real FILTER_C    = -POLE;
    localparam real EXP_POLE    = -2.0e6;
    localparam bit  USE_STEP    = 1'b1;
    localparam bit  USE_EXP     = 1'b1;
    localparam real K_VCO          = 50.0e6;    // 50 MHz/V
    localparam real F0             = 10.0e6;    // 10 MHz center (Viva-friendly)
    localparam real V_RF_PEAK      = 0.5;       // 0.5 V peak -> 1.0 V peak-to-peak
    localparam real SAMPLE_STEP    = 10.0;      // ns — slow signals (lf_v, fout)
    localparam real SAMPLE_STEP_RF = 0.5;       // ns — RF sine (~200 samples/period @10MHz)

    logic clk;
    logic rst_n;
    logic lf_in_valid;
    logic lf_out_valid;
    logic vco_in_valid;
    logic vco_freq_valid;
    logic vco_spec_valid;

    xreal_term_t lf_in_terms   [0:MAX_XREAL_TERMS-1];
    int          lf_in_count;
    xreal_term_t lf_out_terms  [0:MAX_XREAL_TERMS-1];
    int          lf_out_count;
    xreal_term_t vco_freq_terms [0:MAX_XREAL_TERMS-1];
    int          vco_freq_count;
    spectral_bin_t vco_spec_out  [0:MAX_SPECTRAL_BINS-1];
    int            vco_spec_count;

    real t_event_s;

    // Viva-plottable waveforms
    real x_at_t;
    real lf_v_at_t;
    real vco_fout_at_t;
    real vco_rf_at_t;
    real vco_spec_at_t;
    real t_now_s;

    bit x_in_ready;
    bit lf_out_ready;
    bit vco_out_ready;

    xreal_seq_t    x_in;
    xreal_seq_t    lf_v;
    xreal_seq_t    vco_fout;
    xreal_seq_t    vco_delta_f_cached;
    spectral_seq_t vco_spec;

    loop_filter #(
        .FILTER_C (FILTER_C),
        .FILTER_P (POLE),
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
        .F0    (F0)
    ) vco_dut (
        .clk                (clk),
        .rst_n              (rst_n),
        .vin_valid          (vco_in_valid),
        .vin_terms          (lf_out_terms),
        .vin_term_count     (lf_out_count),
        .t_event            (t_event_s),
        .freq_valid         (vco_freq_valid),
        .freq_terms         (vco_freq_terms),
        .freq_term_count    (vco_freq_count),
        .spectral_valid     (vco_spec_valid),
        .spec_out           (vco_spec_out),
        .spec_out_count     (vco_spec_count)
    );

    initial clk = 1'b0;
    always #1ns clk = ~clk;

    // Latch loop-filter output for sampling and checks
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lf_v.count      <= 0;
            lf_out_ready    <= 1'b0;
        end else if (lf_out_valid) begin
            lf_v.count <= lf_out_count;
            for (int i = 0; i < lf_out_count; i++) begin
                lf_v.terms[i].b  <= lf_out_terms[i].b;
                lf_v.terms[i].a  <= lf_out_terms[i].a;
                lf_v.terms[i].m  <= lf_out_terms[i].m;
                lf_v.terms[i].t0 <= lf_out_terms[i].t0;
            end
            lf_out_ready <= 1'b1;
        end
    end

    // Latch VCO outputs
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vco_fout.count  <= 0;
            vco_spec.count  <= 0;
            vco_out_ready   <= 1'b0;
        end else if (vco_freq_valid) begin
            vco_fout.count <= vco_freq_count;
            for (int i = 0; i < vco_freq_count; i++) begin
                vco_fout.terms[i].b  <= vco_freq_terms[i].b;
                vco_fout.terms[i].a  <= vco_freq_terms[i].a;
                vco_fout.terms[i].m  <= vco_freq_terms[i].m;
                vco_fout.terms[i].t0 <= vco_freq_terms[i].t0;
            end
            vco_spec.count <= vco_spec_count;
            for (int k = 0; k < vco_spec_count; k++) begin
                vco_spec.entry[k].omega <= vco_spec_out[k].omega;
                vco_spec.entry[k].I_val <= vco_spec_out[k].I_val;
                vco_spec.entry[k].Q_val <= vco_spec_out[k].Q_val;
            end
            vco_out_ready <= 1'b1;
        end
    end

    // Slow sampler: LF voltage, VCO frequency
    initial begin
        x_at_t = 0.0; lf_v_at_t = 0.0; vco_fout_at_t = 0.0;
        x_in_ready = 1'b0;
        forever begin
            #(SAMPLE_STEP * 1ns);
            t_now_s = $realtime * 1.0e-9;
            if (x_in_ready)
                x_at_t = eval_xreal_at_t(x_in, t_now_s);
            if (lf_out_ready)
                lf_v_at_t = eval_xreal_at_t(lf_v, t_now_s);
            if (vco_out_ready)
                vco_fout_at_t = eval_xreal_at_t(vco_fout, t_now_s);
        end
    end

    // Fast sampler: RF waveforms (smooth sinusoid in Viva)
    initial begin
        vco_rf_at_t = 0.0;
        vco_spec_at_t = 0.0;
        forever begin
            #(SAMPLE_STEP_RF * 1ns);
            t_now_s = $realtime * 1.0e-9;
            if (vco_out_ready) begin
                vco_rf_at_t = eval_vco_rf_at_t(
                    F0,
                    vco_delta_f_cached,
                    t_now_s,
                    V_RF_PEAK
                );
                vco_spec_at_t = eval_spectral_scaled(
                    vco_spec,
                    t_now_s,
                    V_RF_PEAK
                );
            end
        end
    end

    initial begin
        $dumpfile("wave.vcd");
        $dumpvars(0, pll_lf_vco_tb.x_at_t);
        $dumpvars(0, pll_lf_vco_tb.lf_v_at_t);
        $dumpvars(0, pll_lf_vco_tb.vco_fout_at_t);
        $dumpvars(0, pll_lf_vco_tb.vco_rf_at_t);
        $dumpvars(0, pll_lf_vco_tb.vco_spec_at_t);
    end

    initial begin
        int pass_count;
        int fail_count;
        xreal_seq_t y_expected;
        xreal_seq_t fout_expected;
        xreal_seq_t delta_f;
        real t_check, err;

        pass_count = 0;
        fail_count = 0;

        rst_n         = 1'b0;
        lf_in_valid   = 1'b0;
        vco_in_valid  = 1'b0;
        lf_in_count   = 0;
        t_event_s     = 0.0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        xreal_clear(x_in);
        if (USE_STEP)
            xreal_add_unit_step(x_in, 1.0, 0.0);
        if (USE_EXP)
            xreal_add_exponential(x_in, 1.0, EXP_POLE, 0.0);

        lf_in_count = x_in.count;
        for (int i = 0; i < x_in.count; i++)
            lf_in_terms[i] = x_in.terms[i];
        x_in_ready = 1'b1;

        $display("============================================================");
        $display(" PLL LF + VCO TB — Sections IV-C / IV-D");
        $display(" K_VCO=%0.3e Hz/V   F0=%0.3e Hz", K_VCO, F0);
        $display(" V_RF_PEAK=%0.3f V  (%0.1f Vpp)", V_RF_PEAK, 2.0 * V_RF_PEAK);
        $display(" RF sample: %0.3f ns (%0.0f pts/period @F0)", SAMPLE_STEP_RF,
                 (1.0e9 / F0) / SAMPLE_STEP_RF);
        $display("============================================================");
        xreal_print("CP input X(s)", x_in);

        y_expected = convolve_with_filter(x_in, FILTER_C, POLE, 1);
        xreal_print("Expected LF Y(s)", y_expected);

        // Stimulus: loop filter
        @(posedge clk);
        lf_in_valid = 1'b1;
        @(posedge clk);
        lf_in_valid = 1'b0;

        wait (lf_out_ready);
        @(posedge clk);

        xreal_print("LF output Y(s) / VCO Vin", lf_v);

        // Stimulus: VCO on LF output event
        t_event_s    = $realtime * 1.0e-9;
        vco_in_valid = 1'b1;
        @(posedge clk);
        vco_in_valid = 1'b0;

        wait (vco_out_ready);
        @(posedge clk);
        vco_delta_f_cached = xreal_delta_f(vco_fout, F0);

        xreal_print("VCO fout XREAL", vco_fout);
        spectral_print("VCO spectral bins", vco_spec);

        // Check eq. (8): fout = Ki*Vin + f0
        fout_expected = vco_v_to_f(lf_v, K_VCO, F0, t_event_s);
        if (vco_fout.count != fout_expected.count)
            fail_count++;
        else begin
            for (int i = 0; i < vco_fout.count; i++) begin
                if (real_abs(vco_fout.terms[i].b - fout_expected.terms[i].b) > 1.0e-3 ||
                    real_abs(vco_fout.terms[i].a - fout_expected.terms[i].a) > 1.0e-3)
                    fail_count++;
            end
            if (fail_count == 0) begin
                $display("PASS: VCO fout XREAL matches Ki*Vin + f0");
                pass_count++;
            end
        end

        delta_f = xreal_delta_f(vco_fout, F0);

        $display("------------------------------------------------------------");
        $display("  t[ns]   lf_v(t)   fout(t)[MHz]   vco_rf(t)   vco_spec(t)");
        $display("------------------------------------------------------------");

        for (int k = 0; k <= 10; k++) begin
            t_check = k * 0.5 * TAU;
            $display(" %7.1f  %0.5f   %0.6f        %0.5f      %0.5f",
                     t_check * 1.0e9,
                     eval_xreal_at_t(lf_v, t_check),
                     eval_xreal_at_t(vco_fout, t_check) / 1.0e6,
                     eval_vco_rf_at_t(F0, delta_f, t_check, V_RF_PEAK),
                     eval_spectral_scaled(vco_spec, t_check, V_RF_PEAK));
            pass_count++;
        end

        $display("============================================================");
        if (fail_count == 0)
            $display(" ALL CHECKS PASSED (%0d checks)", pass_count);
        else
            $display(" FAILED: %0d errors", fail_count);
        $display("============================================================");

        #10us;
        $finish;
    end

initial begin
$dumpfile("out.vcd");
//$strobe ("FSM : data_buf[0] = %2d ",ctle_dir[0]);
$shm_open("waves_new.shm"); $shm_probe("AS");
end

endmodule
