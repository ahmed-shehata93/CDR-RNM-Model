// Classic PFD: two D-flip-flops + delayed AND reset pulse.
//   posedge ref_rise -> UP = 1 (holds until FB completes the cycle)
//   posedge fb_rise  -> DN = 1 (both stay high ~T_AND_NS, then AND reset clears both)
//   Illegal lone-set guards: an edge with the partner level already high and its FF
//   idle is ignored, so a cmd can never latch high with no partner edge coming.

`timescale 1ns / 1ps

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

    // UP: set on rising REF (unless FB already high with DN not armed — illegal)
    always @(posedge ref_rise or posedge clr_pulse or negedge rst_n) begin
        if (!rst_n || clr_pulse)
            up_ff <= 1'b0;
        else if (fb_rise && !dn_ff)
            up_ff <= 1'b0;
        else
            up_ff <= 1'b1;
    end

    // DN: set on rising FB only if REF is low (FB leads) or UP already armed (REF led)
    //     Block DN-only while REF is already high and UP is 0.
    always @(posedge fb_rise or posedge clr_pulse or negedge rst_n) begin
        if (!rst_n || clr_pulse)
            dn_ff <= 1'b0;
        else if (ref_rise && !up_ff)
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
