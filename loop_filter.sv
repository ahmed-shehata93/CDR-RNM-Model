// Event-driven loop filter model — Section IV-C.
// Compile 00_xreal_pkg.sv first.  Import: xreal_pkg::*  (NOT the file name)

import xreal_pkg::*;

module loop_filter #(
    parameter real FILTER_C = 1.0e6,
    parameter real FILTER_P = -1.0e6,
    parameter int  FILTER_N = 1
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        in_valid,
    input  xreal_term_t in_terms  [0:MAX_XREAL_TERMS-1],
    input  int          in_term_count,
    output logic        out_valid,
    output xreal_term_t out_terms [0:MAX_XREAL_TERMS-1],
    output int          out_term_count
);

    xreal_seq_t in_seq;
    xreal_seq_t out_seq;
    xreal_seq_t out_state;      // accumulated output (filter memory across events)
    bit         out_state_ready;
    real        lf_tk;
    real        lf_y0;
    int         lf_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid       <= 1'b0;
            out_term_count  <= 0;
            out_state_ready  = 1'b0;
        end else if (in_valid) begin
            in_seq.count = in_term_count;
            for (lf_i = 0; lf_i < in_term_count; lf_i = lf_i + 1)
                in_seq.terms[lf_i] = in_terms[lf_i];

            // Event time = input terms' origin (CP rebases all terms to the event)
            lf_tk = (in_seq.count > 0) ? in_seq.terms[0].t0 : ($realtime * 1.0e-9);

            // Filter memory: previous output evaluated at the new event time
            lf_y0 = out_state_ready ? eval_xreal_at_t(out_state, lf_tk) : 0.0;

            // Zero-state response to the input from tk onward
            out_seq = convolve_with_filter(in_seq, FILTER_C, FILTER_P, FILTER_N);

            // Zero-input response: prior state decays with the filter pole
            if (real_abs(lf_y0) > 1.0e-15)
                xreal_add_term(out_seq, lf_y0, FILTER_P, 1, lf_tk);
            xreal_compress(out_seq);
            xreal_sort(out_seq);

            out_state       = out_seq;
            out_state_ready = 1'b1;

            out_term_count <= out_seq.count;
            for (lf_i = 0; lf_i < out_seq.count; lf_i = lf_i + 1) begin
                out_terms[lf_i].b  <= out_seq.terms[lf_i].b;
                out_terms[lf_i].a  <= out_seq.terms[lf_i].a;
                out_terms[lf_i].m  <= out_seq.terms[lf_i].m;
                out_terms[lf_i].t0 <= out_seq.terms[lf_i].t0;
            end
            out_valid <= 1'b1;
        end else begin
            out_valid <= 1'b0;
        end
    end

endmodule
