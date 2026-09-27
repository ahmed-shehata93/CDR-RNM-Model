// Reference clock generator — square wave at F_HZ.
// Stays low until enable; T_START_NS is then the ref/fb phase offset [ns].
// HALF_CYCLE is half-period in ns (timescale 1ns/10fs).

`timescale 1ns / 10fs

module ref_xbit_gen #(
    parameter real F_HZ       = 60.0e6,
    parameter real T_START_NS = 0.0     // phase delay after enable [ns]
)(
    input  logic enable,
    output logic ref_clk
);

    localparam real HALF_CYCLE = 0.5e9 / F_HZ;  // [ns]

    initial begin
        ref_clk = 1'b0;
        wait (enable === 1'b1);
        #(T_START_NS);
        ref_clk = 1'b1;                 // first rise at enable + T_START_NS
        forever #(HALF_CYCLE) ref_clk = ~ref_clk;
    end

endmodule
