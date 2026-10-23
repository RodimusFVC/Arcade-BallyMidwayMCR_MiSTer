//============================================================================
//
//  diag_overlay - rows of bit cells drawn over the core's picture (no SignalTap: read values off the screen)
//  Copyright (C) 2026 Rodimus
//
//  Spec: vault Tools/Reusable diagnostic swatch-overlay module.md. Conventions:
//    each row is one byte, col 0 = LEFTMOST = bit 0; lit col = 1 (white), dim col = 0 (dark grey);
//    rows stack downward from (X0, Y0) in visible-pixel coordinates; enable = 0 is a pure pass-through.
//  Position comes from ce_pix / hblank / vblank, so it works on any core without its counters.
//
//============================================================================

module diag_overlay #(
    parameter N_ROWS  = 4,
    parameter CELL_W  = 6,
    parameter CELL_H  = 8,
    parameter ROW_GAP = 4,
    parameter X0      = 16,
    parameter Y0      = 24
)(
    input  logic                clk,
    input  logic                ce_pix,
    input  logic                hblank,
    input  logic                vblank,
    input  logic                enable,
    input  logic [N_ROWS*8-1:0] rowdata,       // row 0 in [7:0]
    input  logic          [7:0] core_r, core_g, core_b,
    output logic          [7:0] out_r,  out_g,  out_b
);

logic [9:0] x, y;
logic       hb_d;
always_ff @(posedge clk) if (ce_pix) begin
    hb_d <= hblank;
    if (hblank) x <= 10'd0; else x <= x + 10'd1;
    if (vblank) y <= 10'd0; else if (hblank && !hb_d) y <= y + 10'd1;
end

localparam ROW_H = CELL_H + ROW_GAP;
wire [9:0] dx = x - X0[9:0];
wire [9:0] dy = y - Y0[9:0];
wire [9:0] row  = dy / ROW_H;
wire [9:0] ry   = dy % ROW_H;
wire [9:0] col = dx / CELL_W;
wire [9:0] cx   = dx % CELL_W;
wire in_band = enable && x >= X0 && y >= Y0 && row < N_ROWS && ry < CELL_H && col < 8 && cx != CELL_W - 1;
wire lit     = rowdata[row * 8 + col];

assign {out_r, out_g, out_b} = !in_band ? {core_r, core_g, core_b} : lit ? 24'hFFFFFF : 24'h303030;

endmodule
