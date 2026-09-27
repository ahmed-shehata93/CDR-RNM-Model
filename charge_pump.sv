// Charge pump — Section IV-B eq. (5).
// Pulse-driven: UP/DOWN commands set target current; release (both low) -> 0 A.
// TB calls apply_cmd() in the same cycle as pfd_valid (cmd_valid == pfd_valid).

import xreal_pkg::*;

module charge_pump #(
    parameter real I_UP   = 100.0e-6,
    parameter real I_DOWN = -100.0e-6,
    parameter real TAU_CP = 100.0e-9
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        cmd_valid,
    input  logic        cp_up_cmd,
    input  logic        cp_down_cmd,
    input  real         t_event,
    input  xreal_term_t icp_in_terms  [0:MAX_XREAL_TERMS-1],
    input  int          icp_in_count,
    output logic        icp_valid,
    output xreal_term_t icp_out_terms [0:MAX_XREAL_TERMS-1],
    output int          icp_out_count
);

    xreal_seq_t icp_seq;
    real        I0;
    real        Iend;
    int         i;

    task automatic apply_cmd(
        input logic up_cmd,
        input logic down_cmd,
        input real  t_ev
    );
        begin
            icp_seq.count = icp_in_count;
            for (i = 0; i < icp_in_count; i = i + 1)
                icp_seq.terms[i] = icp_in_terms[i];

            I0 = cp_eval_current_at_t(icp_seq, t_ev);

            // Section IV-A: PFD is UP / DOWN / ZERO, so both-high is ZERO (no current).
            if (up_cmd && down_cmd)
                Iend = 0.0;
            else if (up_cmd)
                Iend = I_UP;
            else if (down_cmd)
                Iend = I_DOWN;
            else
                Iend = 0.0;

            cp_append_transition(icp_seq, Iend, I0, TAU_CP, t_ev);

            icp_out_count = icp_seq.count;
            for (i = 0; i < icp_seq.count; i = i + 1) begin
                icp_out_terms[i].b  = icp_seq.terms[i].b;
                icp_out_terms[i].a  = icp_seq.terms[i].a;
                icp_out_terms[i].m  = icp_seq.terms[i].m;
                icp_out_terms[i].t0 = icp_seq.terms[i].t0;
            end
            icp_valid = 1'b1;
        end
    endtask

    task automatic clear_valid();
        begin
            icp_valid = 1'b0;
        end
    endtask

    always @(negedge rst_n) begin
        icp_valid     = 1'b0;
        icp_out_count = 0;
    end

endmodule
