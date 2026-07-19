// Reference clock XBIT generator — same frequency as VCO XREAL output.
// Uses xbit_from_xreal_freq with configurable phase offset vs feedback.

import xreal_pkg::*;

module ref_xbit_gen #(
    parameter real T_STOP       = 5.0e-6,
    parameter real PHASE_OFFSET = 0.0
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        fin_valid,
    input  xreal_term_t freq_terms [0:MAX_XREAL_TERMS-1],
    input  int          freq_term_count,
    input  real         t_event,
    output logic        xbit_valid,
    output xbit_edge_t  xbit_edges  [0:MAX_XBIT_EDGES-1],
    output int          xbit_edge_count,
    output logic [1:0]  level_at_zero
);

    xreal_seq_t freq_seq;
    xbit_seq_t  xbit_seq;
    int         i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xbit_valid      <= 1'b0;
            xbit_edge_count <= 0;
            level_at_zero   <= XBIT_VAL_0;
        end else if (fin_valid) begin
            freq_seq.count = freq_term_count;
            for (i = 0; i < freq_term_count; i = i + 1)
                freq_seq.terms[i] = freq_terms[i];

            xbit_seq = xbit_from_xreal_freq(
                freq_seq, t_event, PHASE_OFFSET, T_STOP
            );

            xbit_edge_count <= xbit_seq.count;
            level_at_zero   <= 2'(xbit_seq.level_at_zero);
            for (i = 0; i < xbit_seq.count; i = i + 1) begin
                xbit_edges[i].t_edge <= xbit_seq.ev[i].t_edge;
                xbit_edges[i].level  <= 2'(xbit_seq.ev[i].level);
            end
            xbit_valid <= 1'b1;
        end else begin
            xbit_valid <= 1'b0;
        end
    end

endmodule
