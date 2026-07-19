// Testbench: multi-term XREAL input through 1st-order loop filter.
//
// Default input (superposition):
//   x(t) = u(t) + exp(-2e6*t)     <=>   X(s) = 1/s + 1/(s + 2e6)
//
// Plottable waveforms (EDA Playground EPWave scope):
//   x_at_t  — inverse Laplace of X(s), evaluated each sample period
//   y_at_t  — inverse Laplace of Y(s), evaluated each sample period
// Do NOT scope x_in.count; that is only the number of XREAL terms.
//
// Simulators: compile 00_xreal_pkg.sv FIRST (see compile.f).
//   Cadence:  irun -f compile.f
//   VCS:      vcs -sverilog 00_xreal_pkg.sv loop_filter.sv loop_filter_tb.sv

`timescale 1ns / 1ps

import xreal_pkg::*;

module loop_filter_tb;

    localparam real TAU          = 1.0e-6;   // filter time constant [s]
    localparam real POLE         = -1.0 / TAU;
    localparam real FILTER_C     = -POLE;    // DC gain = 1
    localparam real EXP_POLE     = -2.0e6;   // input exponential decay rate [1/s]
    localparam bit  USE_STEP     = 1'b1;
    localparam bit  USE_EXP      = 1'b0;
    localparam real SAMPLE_STEP  = 10.0;     // waveform sample period [ns]

    logic clk;
    logic rst_n;
    logic in_valid;
    logic out_valid;

    xreal_term_t in_terms  [0:MAX_XREAL_TERMS-1];
    int          in_term_count;

    xreal_term_t out_terms [0:MAX_XREAL_TERMS-1];
    int          out_term_count;

    // Scope these signals in EPWave (real analog waveforms)
    real x_at_t;
    real y_at_t;
    real t_now_s;

    bit  x_in_ready;
    bit  y_out_ready;

    loop_filter #(
        .FILTER_C (FILTER_C),
        .FILTER_P (POLE),
        .FILTER_N (1)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .in_valid       (in_valid),
        .in_terms       (in_terms),
        .in_term_count  (in_term_count),
        .out_valid      (out_valid),
        .out_terms      (out_terms),
        .out_term_count (out_term_count)
    );

    initial clk = 1'b0;
    always #1ns clk = ~clk;

    xreal_seq_t x_in;
    xreal_seq_t y_expected;
    xreal_seq_t y_out;
    real        t_check;
    real        x_check;
    real        y_model;
    real        y_analytic;
    real        err;
    int         pass_count;
    int         fail_count;

    // Latch DUT Y(s) when the filter responds
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_out.count <= 0;
            y_out_ready <= 1'b0;
        end else if (out_valid) begin
            y_out.count <= out_term_count;
            for (int i = 0; i < out_term_count; i++) begin
                y_out.terms[i].b  <= out_terms[i].b;
                y_out.terms[i].a  <= out_terms[i].a;
                y_out.terms[i].m  <= out_terms[i].m;
                y_out.terms[i].t0 <= out_terms[i].t0;
            end
            y_out_ready <= 1'b1;
        end
    end

    // Continuous sampler: evaluates XREAL sequences in the time domain
    initial begin
        x_at_t      = 0.0;
        y_at_t      = 0.0;
        x_in_ready  = 1'b0;
        forever begin
            #(SAMPLE_STEP * 1ns);
            t_now_s = $realtime * 1.0e-9;
            if (x_in_ready)
                x_at_t = eval_xreal_at_t(x_in, t_now_s);
            if (y_out_ready)
                y_at_t = eval_xreal_at_t(y_out, t_now_s);
        end
    end

    // VCD dump for EPWave (Synopsys VCS)
    initial begin
     //   $dumpfile("wave.shm");
      //  $dumpvars(0, loop_filter_tb.x_at_t, loop_filter_tb.y_at_t);
    end

initial begin
$dumpfile("out.vcd");
//$strobe ("FSM : data_buf[0] = %2d ",ctle_dir[0]);
$shm_open("waves_new.shm"); $shm_probe("AS");
end


    function automatic bit terms_match(input xreal_term_t a, input xreal_term_t b);
        begin
            terms_match = (real_abs(a.b - b.b) < 1.0e-9) &&
                          (real_abs(a.a - b.a) < 1.0e-9) &&
                          (a.m == b.m) &&
                          (real_abs(a.t0 - b.t0) < 1.0e-15);
        end
    endfunction

    initial begin
        pass_count = 0;
        fail_count = 0;

        rst_n         = 1'b0;
        in_valid      = 1'b0;
        in_term_count = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        xreal_clear(x_in);
        if (USE_STEP)
            xreal_add_unit_step(x_in, 1.0, 0.0);
          //  xreal_add_unit_step(x_in, 2.0, 5e-6);

        if (USE_EXP)
            xreal_add_exponential(x_in, 1.0, EXP_POLE, 0.0);

        in_term_count = x_in.count;
        for (int i = 0; i < x_in.count; i++)
            in_terms[i] = x_in.terms[i];
        x_in_ready = 1'b1;

        $display("============================================================");
        $display(" XREAL Loop Filter TB — multi-term input");
        $display(" H(s) = %0.3e / (s %0.3e)", FILTER_C, POLE);
        $display(" Scope x_at_t and y_at_t in EPWave (not x_in.count)");
        $display("============================================================");
        xreal_print("Input X(s)", x_in);

        y_expected = convolve_with_filter(x_in, FILTER_C, POLE, 1);
        xreal_print("Expected Y(s) from s-domain multiply", y_expected);

        @(posedge clk);
        in_valid = 1'b1;
        @(posedge clk);
        in_valid = 1'b0;

        wait (y_out_ready);
        @(posedge clk);

        xreal_print("DUT output Y(s)", y_out);

        if (y_out.count != y_expected.count) begin
            $error("Term count mismatch: DUT=%0d expected=%0d",
                   y_out.count, y_expected.count);
            fail_count++;
        end else begin
            for (int i = 0; i < y_out.count; i++) begin
                if (!terms_match(y_out.terms[i], y_expected.terms[i])) begin
                    $error("Y(s) term[%0d] mismatch: got b=%0.6e a=%0.6e m=%0d",
                           i, y_out.terms[i].b, y_out.terms[i].a, y_out.terms[i].m);
                    fail_count++;
                end
            end
            if (fail_count == 0) begin
                $display("PASS: DUT Y(s) matches analytical partial-fraction decomposition");
                pass_count++;
            end
        end

        $display("------------------------------------------------------------");
        $display(" Time-domain verification (eval_xreal_at_t)");
        $display("   t [ns]    x(t)         y(t)         y_expected     error");
        $display("------------------------------------------------------------");

        for (int k = 0; k <= 10; k++) begin
            t_check    = k * 0.5 * TAU;
            x_check    = eval_xreal_at_t(x_in, t_check);
            y_model    = eval_xreal_at_t(y_out, t_check);
            y_analytic = eval_xreal_at_t(y_expected, t_check);
            err        = y_model - y_analytic;

            $display(" %8.1f   %0.6f     %0.6f     %0.6f      %0.3e",
                     t_check * 1.0e9, x_check, y_model, y_analytic, err);

            if (real_abs(err) > 1.0e-6) begin
                $error("Time-domain error too large at t=%0.3e s", t_check);
                fail_count++;
            end else begin
                pass_count++;
            end
        end

        $display("============================================================");
        if (fail_count == 0)
            $display(" ALL CHECKS PASSED (%0d assertions)", pass_count);
        else
            $display(" FAILED: %0d errors, %0d passes", fail_count, pass_count);
        $display("============================================================");

        #10us;
        $finish;
    end

endmodule
