// Phase-frequency detector — Section IV-A (Cadence-safe int state storage)
// ref_rise / fb_rise sampled on posedge clk in TB, then clock_tick(ref_lvl, fb_lvl).
// CP levels: cp_up_cmd held from ref edge until fb edge; on match cp_down_cmd pulses
// one cycle then both commands return low. ref_seen / fb_seen mirror input levels.

import xreal_pkg::*;

module pfd (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       ref_rise,
    input  logic       fb_rise,
    output logic       pfd_valid,
    output int         pfd_state,
    output logic       cp_up_cmd,
    output logic       cp_down_cmd
);

    int   state_q;
    logic ref_seen;
    logic fb_seen;
    logic ref_lvl_d;
    logic fb_lvl_d;
    logic ref_edge;
    logic fb_edge;

    task clock_tick(
        input logic ref_lvl,
        input logic fb_lvl
    );
        begin
            cp_up_cmd   = 1'b0;
            cp_down_cmd = 1'b0;
            pfd_valid   = 1'b0;

            ref_edge = ref_lvl && !ref_lvl_d;
            fb_edge  = fb_lvl && !fb_lvl_d;

            // Hold UP or DOWN until the partner edge completes the cycle.
            if (state_q == PFD_UP)
                cp_up_cmd = 1'b1;
            else if (state_q == PFD_DOWN)
                cp_down_cmd = 1'b1;

            // Match: fb arrives while UP is active — DOWN pulse, then idle.
            if (fb_edge && (state_q == PFD_UP)) begin
                cp_up_cmd   = 1'b0;
                cp_down_cmd = 1'b1;
                state_q     = PFD_ZERO;
                pfd_state   = PFD_ZERO;
                pfd_valid   = 1'b1;
            end
            // Match: ref arrives while DOWN is active — UP pulse, then idle.
            else if (ref_edge && (state_q == PFD_DOWN)) begin
                cp_up_cmd   = 1'b1;
                cp_down_cmd = 1'b0;
                state_q     = PFD_ZERO;
                pfd_state   = PFD_ZERO;
                pfd_valid   = 1'b1;
            end
            // Simultaneous edges while idle — DOWN pulse only, then idle.
            else if (ref_edge && fb_edge && (state_q == PFD_ZERO)) begin
                cp_up_cmd   = 1'b0;
                cp_down_cmd = 1'b1;
                state_q     = PFD_ZERO;
                pfd_state   = PFD_ZERO;
                pfd_valid   = 1'b1;
            end
            // Ref leads: enter UP (cp_up stays high until fb match).
            else if (ref_edge && (state_q == PFD_ZERO)) begin
                state_q   = PFD_UP;
                pfd_state = PFD_UP;
                cp_up_cmd = 1'b1;
                pfd_valid = 1'b1;
            end
            // Fb leads: enter DOWN (cp_down stays high until ref match).
            else if (fb_edge && (state_q == PFD_ZERO)) begin
                state_q     = PFD_DOWN;
                pfd_state   = PFD_DOWN;
                cp_down_cmd = 1'b1;
                pfd_valid   = 1'b1;
            end

            ref_seen  = ref_lvl;
            fb_seen   = fb_lvl;
            ref_lvl_d = ref_lvl;
            fb_lvl_d  = fb_lvl;
        end
    endtask

    always @(negedge rst_n) begin
        state_q     = PFD_ZERO;
        ref_seen    = 1'b0;
        fb_seen     = 1'b0;
        ref_lvl_d   = 1'b0;
        fb_lvl_d    = 1'b0;
        pfd_valid   = 1'b0;
        pfd_state   = PFD_ZERO;
        cp_up_cmd   = 1'b0;
        cp_down_cmd = 1'b0;
    end

endmodule
