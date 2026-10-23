//============================================================================
//
//  MCR raster timing: 512 x 480 interlaced at 15.72 kHz (Claude/mcr_schematic_facts.md, cross-check table)
//  Copyright (C) 2026 Rodimus
//
//  H: 635 pixel clocks per line (the board's /634 reload), active 0-511.
//  V: two fields of 240 active lines, 16 blank lines each; field 0 = 256 lines, field 1 = 255 lines, so the
//     frame is 511 lines and field 1's VSYNC starts half a line late (H 317, the board's /317) = interlace.
//  The real blank / sync decode sits in unprinted PALs and the monitor took /HBLNK /VBLNK only, so sync is
//  placed here mid-blanking with porches on both sides:
//
//    axis      active    front porch       sync               back porch        total
//    H         0-511     512-527 (16)      528-574 (47)       575-634 (60)      635
//    V fld 0   0-239     240-243 (4)       244-247 (4)        248-255 (8)       256
//    V fld 1   0-239     240-243.5 (3.5)   243.5-247.5 (4)    247.5-254 (7.5)   255
//
//  h_adj / v_adj (CRT position, signed) move the sync start only and are clamped so no porch reaches 0.
//
//============================================================================

module mcr_timing
(
    input            clk,
    input            reset,
    input            ce_pix,
    input      [3:0] h_adj,
    input      [3:0] v_adj,

    output reg [9:0] h,
    output reg [7:0] line,        // line within the field
    output reg       field,       // 0 = even video lines (2 * line), 1 = odd (2 * line + 1)
    output           hblank,
    output           vblank,
    output reg       hs,
    output reg       vs,
    output           v493         // the board's "493" decode: one line, once per frame (CTC TRG3, watchdog clock)
);

    localparam [9:0] H_LAST  = 10'd634;
    localparam [9:0] H_HALF  = 10'd317;

    wire signed [4:0] hadj_s = {h_adj[3], h_adj};
    wire signed [4:0] vadj_s = {v_adj[3], v_adj};
    wire signed [4:0] hadj_c = (hadj_s < -5'sd7) ? -5'sd7 : hadj_s;
    wire signed [4:0] vadj_c = (vadj_s < -5'sd3) ? -5'sd3 : (vadj_s > 5'sd4) ? 5'sd4 : vadj_s;
    wire [9:0] hs_start = 10'd528 + {{4{hadj_c[4]}}, hadj_c, 1'b0};
    wire [9:0] hs_end   = hs_start + 10'd47;
    wire [7:0] vs_line  = 8'd244 + {{3{vadj_c[4]}}, vadj_c};

    wire [7:0] last_line = field ? 8'd254 : 8'd255;

    always @(posedge clk) begin
        if (reset) begin
            h <= 10'd0; line <= 8'd0; field <= 1'b0;
        end else if (ce_pix) begin
            if (h == H_LAST) begin
                h <= 10'd0;
                if (line == last_line) begin
                    line  <= 8'd0;
                    field <= ~field;
                end else
                    line <= line + 8'd1;
            end else
                h <= h + 10'd1;
        end
    end

    // sync as positions (line, h): field 0 starts at the line start, field 1 half a line earlier on the line before
    wire [17:0] pos      = {line, h};
    wire [17:0] vs_start = field ? {vs_line - 8'd1, H_HALF} : {vs_line, 10'd0};
    wire [17:0] vs_stop  = field ? {vs_line + 8'd3, H_HALF} : {vs_line + 8'd4, 10'd0};

    always @(posedge clk) begin
        if (ce_pix) begin
            hs <= (h >= hs_start) && (h < hs_end);
            vs <= (pos >= vs_start) && (pos < vs_stop);
        end
    end

    assign hblank = h[9];                       // 512-634
    assign vblank = (line >= 8'd240);
    assign v493   = field && (line == 8'd237);

endmodule
