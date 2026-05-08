// snap_lut.v — BRAM-based snap lookup table for iCE40UP5K
// Maps 10-bit angle index → snapped angle + triple info
// Behavioral RAM that yosys/nextpnr maps to SB_RAM40_4K
//
// Q4.12 fixed point: 4 integer bits + 12 fractional bits
// 1024 entries × 32 bits = 4KB → fits in 2 BRAM blocks (iCE40 has 30)

`timescale 1ns / 1ps

module snap_lut #(
    parameter ADDR_W = 10,
    parameter DATA_W = 16
)(
    input  wire                clk,
    input  wire [ADDR_W-1:0]   addr,
    output reg  [DATA_W-1:0]   snapped,
    output reg  [15:0]         triple_c,
    output reg                  in_band
);

    // Combined snap+triple RAM: 1024×32 = 2 BRAM blocks
    (* ram_style = "block" *)
    reg [31:0] snap_ram [0:1023];

    initial begin
        $readmemh("snap_lut.hex", snap_ram);
    end

    // Margin RAM: 1024×16 = 1 BRAM block  
    (* ram_style = "block" *)
    reg [15:0] margin_ram [0:1023];

    initial begin
        $readmemh("snap_margin.hex", margin_ram);
    end

    // Registered read
    reg [31:0]  ram_dout;
    reg [15:0]  margin_dout;

    always @(posedge clk) begin
        ram_dout    <= snap_ram[addr];
        margin_dout <= margin_ram[addr];
    end

    // Deadband threshold: ~0.05 radians in Q4.12 ≈ 205
    localparam [15:0] DEADBAND = 16'd205;

    // Combinational output unpack
    always @(*) begin
        snapped  = ram_dout[31:16];
        triple_c = ram_dout[15:0];
        in_band  = (margin_dout < DEADBAND);
    end

endmodule
