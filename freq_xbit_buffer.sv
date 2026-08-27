// Backward-compatible wrapper: XREAL frequency -> XBIT via xreal_to_xbit
// (INTEGRATED edge mode). Prefer instantiating xreal_to_xbit directly.

import xreal_pkg::*;

module freq_xbit_buffer #(
    parameter real T_STOP       = 5.0e-6,
    parameter real PHASE_OFFSET = 0.0
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        vin_valid,
    input  xreal_term_t freq_terms [0:MAX_XREAL_TERMS-1],
    input  int          freq_term_count,
    input  real         t_event,
    output logic        xbit_valid,
    output xbit_edge_t  xbit_edges  [0:MAX_XBIT_EDGES-1],
    output int          xbit_edge_count,
    output logic [1:0]  level_at_zero,
    output real         phase_rad_at_event
);

    // Horizon from each event (not full [0,T_STOP]) so MAX_XBIT_EDGES can
    // cover a live feedback window for xbit_to_logic.
    localparam real T_HORIZON = (T_STOP > 1.0e-6) ? 1.0e-6 : T_STOP;

    xreal_to_xbit #(
        .EDGE_MODE        (1),
        .PHASE_SRC        (1),
        .T_HORIZON        (T_HORIZON),
        .PHASE_OFFSET_RAD (PHASE_OFFSET)
    ) u_xreal_to_xbit (
        .clk                 (clk),
        .rst_n               (rst_n),
        .vin_valid           (vin_valid),
        .freq_terms          (freq_terms),
        .freq_term_count     (freq_term_count),
        .t_event             (t_event),
        .phase_rad           (0.0),
        .xbit_valid          (xbit_valid),
        .xbit_edges          (xbit_edges),
        .xbit_edge_count     (xbit_edge_count),
        .level_at_zero       (level_at_zero),
        .phase_rad_at_event  (phase_rad_at_event)
    );

endmodule
