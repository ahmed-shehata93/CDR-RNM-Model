// VCO model — Section IV-D.
// Compile 00_xreal_pkg.sv first.  Import: xreal_pkg::*  (NOT the file name)

import xreal_pkg::*;

module vco #(
    parameter real K_VCO = 50.0e6,
    parameter real F0    = 10.0e6
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        vin_valid,
    input  xreal_term_t vin_terms  [0:MAX_XREAL_TERMS-1],
    input  int          vin_term_count,
    input  real         t_event,
    output logic        freq_valid,
    output xreal_term_t freq_terms [0:MAX_XREAL_TERMS-1],
    output int          freq_term_count,
    // delta_f = K_VCO * vin (fout minus the f0 term), synchronous with freq_valid
    output xreal_term_t delta_terms [0:MAX_XREAL_TERMS-1],
    output int          delta_term_count,
    output logic        spectral_valid,
    output spectral_bin_t spec_out [0:MAX_SPECTRAL_BINS-1],
    output int          spec_out_count
);

    xreal_seq_t    vin_seq;
    xreal_seq_t    fout_seq;
    xreal_seq_t    delta_f_seq;
    spectral_seq_t spec_seq;
    int            vco_i;
    int            vco_k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            freq_valid       <= 1'b0;
            freq_term_count  <= 0;
            delta_term_count <= 0;
            spectral_valid   <= 1'b0;
            spec_out_count   <= 0;
        end else if (vin_valid) begin
            vin_seq.count = vin_term_count;
            for (vco_i = 0; vco_i < vin_term_count; vco_i = vco_i + 1)
                vin_seq.terms[vco_i] = vin_terms[vco_i];

            fout_seq    = vco_v_to_f(vin_seq, K_VCO, F0, t_event);
            // Build delta_f directly as K*vin: xreal_delta_f(fout, f0) mis-subtracts
            // f0 when fout carries several DC terms at different t0.
            delta_f_seq = xreal_scale(K_VCO, vin_seq);
            spec_seq    = vco_delta_f_to_spectral(delta_f_seq, F0, t_event);

            freq_term_count <= fout_seq.count;
            for (vco_i = 0; vco_i < fout_seq.count; vco_i = vco_i + 1) begin
                freq_terms[vco_i].b  <= fout_seq.terms[vco_i].b;
                freq_terms[vco_i].a  <= fout_seq.terms[vco_i].a;
                freq_terms[vco_i].m  <= fout_seq.terms[vco_i].m;
                freq_terms[vco_i].t0 <= fout_seq.terms[vco_i].t0;
            end
            freq_valid <= 1'b1;

            delta_term_count <= delta_f_seq.count;
            for (vco_i = 0; vco_i < delta_f_seq.count; vco_i = vco_i + 1) begin
                delta_terms[vco_i].b  <= delta_f_seq.terms[vco_i].b;
                delta_terms[vco_i].a  <= delta_f_seq.terms[vco_i].a;
                delta_terms[vco_i].m  <= delta_f_seq.terms[vco_i].m;
                delta_terms[vco_i].t0 <= delta_f_seq.terms[vco_i].t0;
            end

            spec_out_count <= spec_seq.count;
            for (vco_k = 0; vco_k < spec_seq.count; vco_k = vco_k + 1) begin
                spec_out[vco_k].omega <= spec_seq.entry[vco_k].omega;
                spec_out[vco_k].I_val <= spec_seq.entry[vco_k].I_val;
                spec_out[vco_k].Q_val <= spec_seq.entry[vco_k].Q_val;
            end
            spectral_valid <= 1'b1;
        end else begin
            freq_valid     <= 1'b0;
            spectral_valid <= 1'b0;
        end
    end

endmodule
