// Classic PFD: two D-flip-flops + delayed AND reset pulse.
//   posedge ref_rise -> UP = 1 (holds until FB completes the cycle)
//   posedge fb_rise  -> DN = 1 (both stay high ~T_AND_NS, then AND reset clears both)

`timescale 1ns / 10fs

import xreal_pkg::*;

module pfd #(
    parameter real T_AND_NS = 0.05   // AND + reset path delay [ns]
)(
    input  logic       clk,
    input  logic       rst_n,
    input  logic       ref_rise,
    input  logic       fb_rise,
    output logic       pfd_valid,
    output int         pfd_state,
    output logic       cp_up_cmd,
    output logic       cp_down_cmd
);

    logic up_ff;
    logic dn_ff;
    logic ref_seen;
    logic fb_seen;
    logic both_ff;
    logic inputs_both;
    logic clr_pulse;

    assign ref_seen    = ref_rise;
    assign fb_seen     = fb_rise;
    assign both_ff     = up_ff & dn_ff;
    assign inputs_both = ref_rise & fb_rise;

    // Cmds mirror the FF states; the delayed AND reset bounds the both-high overlap
    assign cp_up_cmd   = up_ff;
    assign cp_down_cmd = dn_ff;

    // UP: set on rising REF, cleared only by the AND reset pulse.  A REF edge that
    // arrives while UP is already armed holds it high — that is the frequency-error
    // (cycle-slip) signal, so it must not be masked by the FB level.
    always @(posedge ref_rise or posedge clr_pulse or negedge rst_n) begin
        if (!rst_n || clr_pulse)
            up_ff <= 1'b0;
        else
            up_ff <= 1'b1;
    end

    // DN: set on rising FB, cleared only by the AND reset pulse.
    always @(posedge fb_rise or posedge clr_pulse or negedge rst_n) begin
        if (!rst_n || clr_pulse)
            dn_ff <= 1'b0;
        else
            dn_ff <= 1'b1;
    end

    // Clear pulse: both FFs high -> wait AND-gate delay -> one-shot clear
    always @(posedge both_ff or negedge rst_n) begin
        if (!rst_n) begin
            clr_pulse <= 1'b0;
        end else begin
            clr_pulse <= 1'b0;
            #(T_AND_NS);
            if (up_ff || dn_ff) begin
                clr_pulse <= 1'b1;
                #0.001;
                clr_pulse <= 1'b0;
            end
        end
    end

    always @(*) begin
        if (cp_up_cmd && !cp_down_cmd)
            pfd_state = PFD_UP;
        else if (cp_down_cmd && !cp_up_cmd)
            pfd_state = PFD_DOWN;
        else
            pfd_state = PFD_ZERO;
    end

    always @(cp_up_cmd or cp_down_cmd or negedge rst_n) begin
        if (!rst_n)
            pfd_valid = 1'b0;
        else
            pfd_valid = 1'b1;
    end

    task clock_tick(
        input logic ref_lvl,
        input logic fb_lvl
    );
        begin
        end
    endtask

endmodule
