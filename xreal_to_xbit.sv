// Generic XREAL frequency -> XBIT edge converter.
//
// EDGE_MODE:
//   0 SQUARE     — frozen-f square wave (xbit_gen_square_freq). Unifies:
//                    xbit_from_xreal_freq / xbit_from_xreal_freq_vco_phase
//   1 INTEGRATED — edges at half-cycle crossings of integral fout
//                    (xbit_gen_from_fout_integrated)
//
// PHASE_SRC (SQUARE only; INTEGRATED always applies phase_offset_rad):
//   0 EXTERNAL   — use phase_rad port (xbit_from_xreal_freq)
//   1 FROM_FOUT  — phase = wrap(integral_phase(fout,t) + phase_offset_rad)
//                  (xbit_from_xreal_freq_vco_phase)
//
// Edges are generated on [t_event, t_event+T_HORIZON] so a finite MAX_XBIT_EDGES
// budget can drive a live PFD via xbit_to_logic.
//
// Optional white edge jitter (zero-crossing uncertainty):
//   ENABLE_JITTER=1 adds independent N(0, JITTER_SIGMA_S^2) to each edge time.
//   ENABLE_JITTER=0 leaves ideal edges unchanged.

import xreal_pkg::*;

module xreal_to_xbit #(
    parameter int  EDGE_MODE        = 1,      // 0=SQUARE, 1=INTEGRATED
    parameter int  PHASE_SRC        = 1,      // 0=EXTERNAL, 1=FROM_FOUT (SQUARE)
    parameter real T_HORIZON        = 1.0e-6, // [s] future window from t_event
    parameter real PHASE_OFFSET_RAD = 0.0,    // static trim [rad]
    parameter bit  ENABLE_JITTER    = 1'b0,   // 1 = white timing noise on edges
    parameter real JITTER_SIGMA_S   = 1.0e-12,// RMS edge jitter [s]
    parameter int  JITTER_SEED      = 1       // RNG seed (reproducible)
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        vin_valid,
    input  xreal_term_t freq_terms [0:MAX_XREAL_TERMS-1],
    input  int          freq_term_count,
    input  real         t_event,
    input  real         phase_rad,            // used when EDGE_MODE=SQUARE & PHASE_SRC=EXTERNAL
    output logic        xbit_valid,
    output xbit_edge_t  xbit_edges  [0:MAX_XBIT_EDGES-1],
    output int          xbit_edge_count,
    output logic [1:0]  level_at_zero,
    output real         phase_rad_at_event
);

    xreal_seq_t freq_seq;
    xbit_seq_t  xbit_seq;
    real        t_start;
    real        t_stop;
    real        phase_use;
    real        f_hz;
    int         i;
    int         jitter_seed;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xbit_valid         <= 1'b0;
            xbit_edge_count    <= 0;
            level_at_zero      <= XBIT_VAL_0;
            phase_rad_at_event <= 0.0;
            jitter_seed         = JITTER_SEED;
        end else if (vin_valid) begin
            freq_seq.count = freq_term_count;
            for (i = 0; i < freq_term_count; i = i + 1)
                freq_seq.terms[i] = freq_terms[i];

            t_start = t_event;
            t_stop  = t_event + T_HORIZON;
            if (t_stop < t_start)
                t_stop = t_start;

            if (EDGE_MODE == 0) begin
                // ---- SQUARE: freeze f(t_event), place periodic edges ----
                if (PHASE_SRC == 0)
                    phase_use = phase_rad + PHASE_OFFSET_RAD;
                else
                    phase_use = xreal_inst_phase_rad_at_t(freq_seq, t_start)
                              + PHASE_OFFSET_RAD;
                phase_use = phase_wrap_0_2pi(phase_use);
                phase_rad_at_event <= phase_use;

                f_hz = eval_xreal_at_t(freq_seq, t_start);
                if (f_hz <= 0.0)
                    f_hz = 1.0;

                xbit_seq = xbit_gen_square_freq(
                    f_hz, t_start, phase_use, t_stop, XBIT_VAL_0
                );
            end else begin
                // ---- INTEGRATED: edges track continuous VCO phase ----
                phase_use = xreal_inst_phase_rad_at_t(freq_seq, t_start)
                          + PHASE_OFFSET_RAD;
                phase_use = phase_wrap_0_2pi(phase_use);
                phase_rad_at_event <= phase_use;

                xbit_seq = xbit_gen_from_fout_integrated(
                    freq_seq, t_start, t_stop, PHASE_OFFSET_RAD
                );
            end

            if (ENABLE_JITTER)
                xbit_apply_white_jitter(
                    xbit_seq, t_start, JITTER_SIGMA_S, jitter_seed
                );

            xbit_edge_count <= xbit_seq.count;
            level_at_zero   <= 2'(xbit_seq.level_at_zero);
            for (i = 0; i < xbit_seq.count; i = i + 1) begin
                xbit_edges[i].t_edge <= xbit_seq.ev[i].t_edge;
                xbit_edges[i].level  <= 2'(xbit_seq.ev[i].level);
            end
            // Clear unused slots so stale future edges cannot leak into xbit_to_logic
            for (i = xbit_seq.count; i < MAX_XBIT_EDGES; i = i + 1) begin
                xbit_edges[i].t_edge <= 0.0;
                xbit_edges[i].level  <= XBIT_VAL_0;
            end
            xbit_valid <= 1'b1;
        end else begin
            xbit_valid <= 1'b0;
        end
    end

endmodule
