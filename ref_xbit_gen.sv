// Reference clock generator — free-running square wave at F_HZ.
// T_START_NS delays the first edge, setting the ref/fb phase offset at t=0.
// HALF_CYCLE is half-period in ns (timescale 1ns/1ps).

`timescale 1ns / 1ps

module ref_xbit_gen #(
    parameter real F_HZ       = 60.0e6,
    parameter real T_START_NS = 0.0     // initial phase delay [ns]
)(
    output logic ref_clk
);

    localparam real HALF_CYCLE = 0.5e9 / F_HZ;  // [ns]

    initial begin
        ref_clk = 1'b0;
        #(T_START_NS);
        forever #(HALF_CYCLE) ref_clk = ~ref_clk;
    end

endmodule
