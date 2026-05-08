// constraint_snap_top.v — Top-level constraint snap module
// Takes raw angle → snaps to nearest Pythagorean triple angle
// Pipelined: 2 clock cycles latency, 1 cycle per lookup throughput
//
// Input:  16-bit angle (Q4.12 fixed point, 0..2π ≈ 0..25736)
// Output: snapped angle, is_safe flag, margin value
//
// iCE40UP5K target — uses ~2 BRAM blocks

`timescale 1ns / 1ps

module constraint_snap_top #(
    parameter ADDR_W      = 10,
    parameter DATA_W      = 16,
    parameter DEADBAND    = 16'd205       // ~0.05 rad in Q4.12
)(
    input  wire                clk,
    input  wire                rst_n,

    // Angle input (Q4.12 fixed point, 0..2π)
    input  wire [DATA_W-1:0]   angle_in,
    input  wire                angle_valid,

    // Snapped output
    output wire [DATA_W-1:0]   snapped_angle,
    output wire [15:0]         triple_c,
    output wire                 is_safe,      // within deadband
    output wire [15:0]         margin,
    output wire                 result_valid
);

    // ── Stage 1: Convert angle to BRAM address ──
    // angle_in is Q4.12, range 0..25736 (2π)
    // BRAM address: top 10 bits of angle_in
    // Mapping: addr = angle_in[15:6] for 1024 entries
    reg [ADDR_W-1:0] addr_reg;
    reg              valid_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr_reg  <= {ADDR_W{1'b0}};
            valid_reg <= 1'b0;
        end else if (angle_valid) begin
            // Scale: angle range 0..25736 → address 0..1023
            // Simple: take top 10 bits
            addr_reg  <= angle_in[15:6];
            valid_reg <= 1'b1;
        end else begin
            valid_reg <= 1'b0;
        end
    end

    // ── Stage 2: BRAM lookup ──
    wire [DATA_W-1:0] bram_snapped;
    wire [15:0]       bram_triple_c;
    wire               bram_in_band;

    snap_lut #(
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W)
    ) u_snap_lut (
        .clk(clk),
        .addr(addr_reg),
        .snapped(bram_snapped),
        .triple_c(bram_triple_c),
        .in_band(bram_in_band)
    );

    // ── Stage 3: Output registration ──
    reg [DATA_W-1:0]   snapped_r;
    reg [15:0]         triple_c_r;
    reg                safe_r;
    reg                valid_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snapped_r  <= {DATA_W{1'b0}};
            triple_c_r <= 16'd0;
            safe_r     <= 1'b0;
            valid_r    <= 1'b0;
        end else begin
            snapped_r  <= bram_snapped;
            triple_c_r <= bram_triple_c;
            safe_r     <= bram_in_band;
            valid_r    <= valid_reg;
        end
    end

    // ── Outputs ──
    assign snapped_angle = snapped_r;
    assign triple_c      = triple_c_r;
    assign is_safe       = safe_r;
    assign margin        = 16'd0;  // Margin available via margin_ram in snap_lut
    assign result_valid  = valid_r;

endmodule
