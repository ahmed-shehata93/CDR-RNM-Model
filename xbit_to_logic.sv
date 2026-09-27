// XBIT edge sequence -> continuous logic level for PFD / digital blocks.
//
// Loads a new edge list one clk after xbit_valid (NBA settle), sets bit_out to
// the level at "now", then schedules future edges. gen_id cancels stale forks.
//
// Avoid reloading mid half-cycle when possible — regenerating INTEGRATED edges
// while a level is already driven shifts the next boundary and warps duty.

`timescale 1ns / 10fs

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
    real       t_last_toggle;
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
            t_last_toggle  = 0.0;
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
                // First load may snap to the sequence level. Later reloads must
                // NOT: snapping every keepalive injects an extra FB edge
                // (~1/T_HORIZON Hz) and offsets lock Vctrl by that / Kvco.
                if (gen_id == 0) begin
                    cur_lvl = level_at_time(seq, t_now);
                    bit_out <= (cur_lvl == XBIT_VAL_1);
                end

                gen_id = gen_id + 1;
                // A reload used to drop the pending edge and take the new list's
                // absolute times. If that lands just after a toggle, the half-cycle
                // in progress is cut to a few picoseconds (runt between normal
                // pulses). Continue from the last pin toggle using the new
                // half-period so duty stays one VCO half-cycle.
                begin : schedule_edges
                    automatic int   my_id = gen_id;
                    automatic int   ei;
                    automatic int   n_sp;
                    automatic int   n_l;
                    automatic real  acc_sp;
                    automatic real  spacing;
                    automatic real  t_stop_l;
                    automatic real  t_last_l;
                    automatic real  t0;
                    automatic real  edge_dt;
                    automatic logic lvl0;
                    automatic bit   use_anchor;
                    automatic real  te_l [0:MAX_XBIT_EDGES-1];
                    automatic bit   lv_l [0:MAX_XBIT_EDGES-1];

                    n_sp        = 0;
                    n_l         = 0;
                    acc_sp      = 0.0;
                    t_stop_l    = t_now;
                    t0          = t_now;
                    t_last_l    = t_last_toggle;
                    lvl0        = bit_out;
                    for (ei = 0; ei < seq.count; ei = ei + 1) begin
                        if (seq.ev[ei].t_edge > t_now && n_l < MAX_XBIT_EDGES) begin
                            if (n_l > 0) begin
                                edge_dt = seq.ev[ei].t_edge - te_l[n_l - 1];
                                if (edge_dt > 40.0e-12 && edge_dt < 400.0e-12) begin
                                    acc_sp = acc_sp + edge_dt;
                                    n_sp   = n_sp + 1;
                                end
                            end
                            te_l[n_l] = seq.ev[ei].t_edge;
                            lv_l[n_l] = (seq.ev[ei].level == XBIT_VAL_1);
                            if (te_l[n_l] > t_stop_l)
                                t_stop_l = te_l[n_l];
                            n_l = n_l + 1;
                        end
                    end
                    spacing    = (n_sp > 0) ? (acc_sp / real'(n_sp)) : 100.0e-12;
                    use_anchor = (t_last_l > 0.0) && ((t0 - t_last_l) < (20.0 * spacing));
                    fork
                        begin
                            if (use_anchor) begin
                                automatic real  t_sched;
                                automatic real  t_prev;
                                automatic logic lvl;
                                automatic int   n_emit;
                                t_sched = t_last_l + spacing;
                                if (t_sched < t0)
                                    t_sched = t0;
                                lvl    = !lvl0;
                                t_prev = t0;
                                n_emit = 0;
                                while (t_sched <= t_stop_l && n_emit < MAX_XBIT_EDGES) begin
                                    #((t_sched - t_prev) * 1.0e9);
                                    if (my_id != gen_id)
                                        break;
                                    bit_out       <= lvl;
                                    t_last_toggle  = t_sched;
                                    lvl            = !lvl;
                                    t_prev         = t_sched;
                                    t_sched        = t_sched + spacing;
                                    n_emit         = n_emit + 1;
                                end
                            end else begin
                                automatic real t_prev;
                                automatic int  ei2;
                                t_prev = t0;
                                for (ei2 = 0; ei2 < n_l; ei2 = ei2 + 1) begin
                                    #((te_l[ei2] - t_prev) * 1.0e9);
                                    if (my_id != gen_id)
                                        break;
                                    bit_out       <= lv_l[ei2];
                                    t_last_toggle  = te_l[ei2];
                                    t_prev         = te_l[ei2];
                                end
                            end
                        end
                    join_none
                end
            end
        end
    end

endmodule
