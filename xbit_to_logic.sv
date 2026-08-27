// XBIT edge sequence -> continuous logic level for PFD / digital blocks.
//
// Loads a new edge list one clk after xbit_valid (NBA settle), sets bit_out to
// the level at "now", then schedules future edges. gen_id cancels stale forks.
//
// Avoid reloading mid half-cycle when possible — regenerating INTEGRATED edges
// while a level is already driven shifts the next boundary and warps duty.

`timescale 1ns / 1ps

import xreal_pkg::*;

module xbit_to_logic (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        xbit_valid,
    input  xbit_edge_t  xbit_edges  [0:MAX_XBIT_EDGES-1],
    input  int          xbit_edge_count,
    input  logic [1:0]  level_at_zero,
    output logic        bit_out
);

    xbit_seq_t seq;
    logic      xbit_valid_d1;
    int        gen_id;
    int        i;
    real       t_now;
    xbit_val_e cur_lvl;

    function automatic xbit_val_e level_at_time(
        input xbit_seq_t s,
        input real       t
    );
        xbit_val_e lvl;
        begin
            lvl = s.level_at_zero;
            for (int k = 0; k < s.count; k++) begin
                if (t >= s.ev[k].t_edge)
                    lvl = s.ev[k].level;
            end
            level_at_time = lvl;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xbit_valid_d1 <= 1'b0;
            bit_out       <= 1'b0;
            gen_id        <= 0;
        end else begin
            xbit_valid_d1 <= xbit_valid;

            if (xbit_valid_d1) begin
                xbit_clear(seq);
                seq.level_at_zero = xbit_val_e'(level_at_zero);
                seq.count = xbit_edge_count;
                for (i = 0; i < xbit_edge_count; i = i + 1) begin
                    seq.ev[i].t_edge = xbit_edges[i].t_edge;
                    seq.ev[i].level  = xbit_val_e'(xbit_edges[i].level);
                end

                t_now   = $realtime * 1.0e-9;
                cur_lvl = level_at_time(seq, t_now);
                bit_out <= (cur_lvl == XBIT_VAL_1);

                gen_id = gen_id + 1;
                begin : schedule_edges
                    automatic int my_id = gen_id;
                    automatic int ei;
                    for (ei = 0; ei < seq.count; ei = ei + 1) begin
                        automatic real       te  = seq.ev[ei].t_edge;
                        automatic real       dly;
                        automatic xbit_val_e lv  = seq.ev[ei].level;
                        if (te > t_now) begin
                            dly = (te - t_now) * 1.0e9;
                            fork
                                begin
                                    #(dly);
                                    if (my_id == gen_id)
                                        bit_out <= (lv == XBIT_VAL_1);
                                end
                            join_none
                        end
                    end
                end
            end
        end
    end

endmodule
