// Event-driven loop filter model — Section IV-C.
// Compile 00_xreal_pkg.sv first.  Import: xreal_pkg::*  (NOT the file name)
//
// Impedance: Z(s) = FILTER_R + FILTER_C / (s - FILTER_P)^FILTER_N
//   FILTER_R = 0  -> leaky integrator only (legacy)
//   FILTER_R > 0  -> series R with C; continuous state is capacitor voltage Vc only.
//   Vraw = R*I + Vc
// Optional FILTER_C2 > 0: ripple cap — low-pass Vraw with tau = R*C2 before Vout
//   (attenuates short CP I*R spikes; DC / slow Vc still passes).

import xreal_pkg::*;

module loop_filter #(
    parameter real FILTER_C  = 1.0e6,
    parameter real FILTER_P  = -1.0e6,
    parameter int  FILTER_N  = 1,
    parameter real FILTER_R  = 0.0,
    parameter real FILTER_C2 = 0.0
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
    xreal_seq_t vc_seq;
    xreal_seq_t r_seq;
    xreal_seq_t out_seq;
    xreal_seq_t raw_seq;
    xreal_seq_t c2_seq;
    xreal_seq_t vc_state;       // capacitor voltage memory across events
    xreal_seq_t c2_state;       // ripple-filter state across events
    bit         vc_state_ready;
    bit         c2_state_ready;
    real        lf_tk;
    real        lf_vc0;
    real        lf_c20;
    real        tau_c2;
    real        pole_c2;
    real        gain_c2;
    int         lf_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid        <= 1'b0;
            out_term_count   <= 0;
            vc_state_ready    = 1'b0;
            c2_state_ready    = 1'b0;
        end else if (in_valid) begin
            in_seq.count = in_term_count;
            for (lf_i = 0; lf_i < in_term_count; lf_i = lf_i + 1)
                in_seq.terms[lf_i] = in_terms[lf_i];

            // Event time = input terms' origin (CP rebases all terms to the event)
            lf_tk = (in_seq.count > 0) ? in_seq.terms[0].t0 : ($realtime * 1.0e-9);

            // Capacitor memory: previous Vc at the new event time.
            // CP input is typically only 1–2 rebased terms (not full history), so
            // reconvolve alone is ~0 at tk — lf_vc0 is the real integrating state.
            lf_vc0 = vc_state_ready ? eval_xreal_at_t(vc_state, lf_tk) : 0.0;

            // Zero-state response to the (compressed) input terms at this event
            vc_seq = convolve_with_filter(in_seq, FILTER_C, FILTER_P, FILTER_N);

            // Zero-input response: prior Vc decays with the filter pole
            if (real_abs(lf_vc0) > 1.0e-15)
                xreal_add_term(vc_seq, lf_vc0, FILTER_P, 1, lf_tk);
            xreal_compress(vc_seq);
            xreal_sort(vc_seq);

            vc_state       = vc_seq;
            vc_state_ready = 1'b1;

            // Vraw = R*I + Vc
            raw_seq = vc_seq;
            if (real_abs(FILTER_R) > 1.0e-30) begin
                r_seq = xreal_scale(FILTER_R, in_seq);
                for (lf_i = 0; lf_i < r_seq.count; lf_i = lf_i + 1)
                    xreal_add_term(
                        raw_seq,
                        r_seq.terms[lf_i].b,
                        r_seq.terms[lf_i].a,
                        r_seq.terms[lf_i].m,
                        r_seq.terms[lf_i].t0
                    );
                xreal_compress(raw_seq);
                xreal_sort(raw_seq);
            end

            // Optional C2 ripple LPF: H = (1/tau)/(s+1/tau), tau = R*C2
            if (real_abs(FILTER_C2) > 1.0e-30 && real_abs(FILTER_R) > 1.0e-30) begin
                tau_c2  = FILTER_R * FILTER_C2;
                pole_c2 = -1.0 / tau_c2;
                gain_c2 =  1.0 / tau_c2;
                lf_c20  = c2_state_ready ? eval_xreal_at_t(c2_state, lf_tk) : 0.0;
                c2_seq  = convolve_with_filter(raw_seq, gain_c2, pole_c2, 1);
                if (real_abs(lf_c20) > 1.0e-15)
                    xreal_add_term(c2_seq, lf_c20, pole_c2, 1, lf_tk);
                xreal_compress(c2_seq);
                xreal_sort(c2_seq);
                c2_state       = c2_seq;
                c2_state_ready = 1'b1;
                out_seq        = c2_seq;
            end else begin
                out_seq = raw_seq;
            end

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
