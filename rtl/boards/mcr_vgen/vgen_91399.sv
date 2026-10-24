//============================================================================
//
//  Midway 91399 MCR/II Video Gen: 32 x 32 sprites into a double line buffer
//  Copyright (C) 2026 Rodimus
//
//  Facts: Claude/mcr_schematic_facts.md Part B (91399 schematic) and MAME bally/mcr_v.cpp render_sprites_91399.
//  Sprite RAM (on the CPU board, read through port B): 128 x 4 bytes = Y, picture (b5-0 code, b6 H flip,
//  b7 V flip), X, unused. Screen position (512 x 480 space): x = (X - 4) * 2 (+16 per set, byte 152),
//  y = (240 - Y) * 2; cocktail: flips inverted, x = 466 - x (+16 per set), y = 450 - y. Both wrap at 512.
//  Sprite ROM (rom3, shared): four 8K quarters, a sprite row = 4 bytes from each, 2 pixels per byte (high nibble
//  first): row bytes go quarter 0, 1, 2, 3, each giving the next 2 pixels, then the next byte of each quarter.
//  Line buffer (schematic): pixels are ORed into the buffer, which is cleared as it is shown. The CPU board shows
//  a sprite pixel where its low 3 bits are not 0, coloured by the background tile's sprite bank.
//
//  While display line n is shown, the other buffer is drawn for the next display line of the same field
//  (2 video lines down, or line 0 / 1 of the next field after the last line).
//
//============================================================================

module vgen_91399
(
    input             clk,
    input             reset,
    input             ce_pix,
    input       [2:0] pix_phase,

    input       [9:0] h,
    input       [7:0] line,
    input             field,
    input             cocktail,       // output port 0 D6
    input             flip_x,         // picture mirrored left-right (core flip option ^ set mirror X)
    input             flip_y,         // picture mirrored top-bottom (core flip option ^ set mirror Y)
    input       [1:0] xoffs,          // byte 152: [0] +16 normal, [1] +16 cocktail

    output reg  [8:0] spr_a,          // sprite RAM read (CPU board port B)
    input       [7:0] spr_d,
    output reg [16:0] rom_a,          // rom3 (sprites)
    input       [7:0] rom_d,

    output reg  [3:0] spix,           // sprite pixel for slot h, valid from phase 7 (same timing as the background)

    // line buffers 0 / 1 (shared with the other sprite board, rtl/mcr_board.sv): port A reads, port B writes
    output      [8:0] lb_aa [2],
    input       [7:0] lb_qa [2],
    output      [8:0] lb_ab [2],
    output      [7:0] lb_db [2],
    output            lb_web [2]
);

    // ------------------------------------------------------------ line buffers (2 x 256 words of 2 pixels, low half)
    reg        sel;                   // buffer being drawn; ~sel is shown
    reg  [7:0] dr_ra, dr_wa, ds_a;
    reg  [7:0] dr_wd;
    reg        dr_we, ds_clr;
    wire [7:0] b0_qa = lb_qa[0], b1_qa = lb_qa[1];
    assign lb_aa[0]  = {1'b0, sel ? ds_a : dr_ra};
    assign lb_ab[0]  = {1'b0, sel ? ds_a : dr_wa};
    assign lb_db[0]  = sel ? 8'h00 : dr_wd;
    assign lb_web[0] = sel ? ds_clr : dr_we;
    assign lb_aa[1]  = {1'b0, sel ? dr_ra : ds_a};
    assign lb_ab[1]  = {1'b0, sel ? dr_wa : ds_a};
    assign lb_db[1]  = sel ? dr_wd : 8'h00;
    assign lb_web[1] = sel ? dr_we : ds_clr;
    wire [7:0] dr_q = sel ? b1_qa : b0_qa;
    wire [7:0] ds_q = sel ? b0_qa : b1_qa;

    // ------------------------------------------------------------ display side: read, then clear the word
    wire [8:0] xs   = flip_x ? ~h[8:0] : h[8:0];          // buffer position of this screen pixel
    wire       last = xs[0] ^ flip_x;                     // second pixel of the word on this line
    always @(posedge clk) begin
        ds_clr <= 1'b0;
        case (pix_phase)
            3'd2: ds_a <= xs[8:1];
            3'd4: begin
                spix   <= xs[0] ? ds_q[3:0] : ds_q[7:4];
                ds_clr <= last;
            end
            default: ;
        endcase
    end

    // ------------------------------------------------------------ draw side
    // target: the next display line; its source line when the core flip is on
    wire [7:0] last_line = field ? 8'd254 : 8'd255;
    wire       wrap      = (line == last_line);
    wire [8:0] y_next    = wrap ? {8'd0, ~field} : {line + 8'd1, field};
    wire [8:0] ty_w      = flip_y ? (9'd479 - y_next) : y_next;

    localparam S_IDLE = 3'd0, S_Y = 3'd1, S_CODE = 3'd2, S_X = 3'd3, S_ROM = 3'd4, S_WAIT = 3'd5, S_RMW = 3'd6;
    reg  [2:0] st;
    reg  [6:0] n;                     // sprite index
    reg  [8:0] ty;
    reg  [4:0] row;
    reg  [5:0] code;
    reg        hfl;
    reg  [7:0] sxw;                   // word position of the sprite's left edge
    reg  [3:0] k;                     // byte within the row: k[3:2] = column byte, k[1:0] = quarter
    reg        wait2;

    wire       line_start = ce_pix && (h == 10'd634);        // the next slot is h = 0 of a new line
    wire [8:0] sy_n  = 9'd480 - {spr_d, 1'b0};               // (240 - Y) * 2, mod 512
    wire [8:0] sy_c  = 9'd450 - sy_n;
    wire [8:0] sy    = cocktail ? sy_c : sy_n;
    wire [8:0] dy    = ty - sy;

    always @(posedge clk) begin
        dr_we <= 1'b0;
        if (reset) begin
            st <= S_IDLE; sel <= 1'b0;
        end else if (line_start) begin
            sel <= ~sel;              // what was drawn is shown next; draw the other buffer
            ty  <= ty_w;
            n   <= 7'd0;
            spr_a <= 9'd0;
            st  <= S_Y; wait2 <= 1'b1;
        end else case (st)
            S_Y: begin
                if (wait2) wait2 <= 1'b0;
                else if (dy < 9'd32) begin
                    row   <= dy[4:0];
                    spr_a <= {n, 2'd1};
                    st    <= S_CODE; wait2 <= 1'b1;
                end else if (n == 7'd127)
                    st <= S_IDLE;
                else begin
                    n     <= n + 7'd1;
                    spr_a <= {n + 7'd1, 2'd0};
                    wait2 <= 1'b1;
                end
            end
            S_CODE: begin
                if (wait2) wait2 <= 1'b0;
                else begin
                    code  <= spr_d[5:0];
                    hfl   <= spr_d[6] ^ cocktail;
                    row   <= row ^ {5{spr_d[7] ^ cocktail}};
                    spr_a <= {n, 2'd2};
                    st    <= S_X; wait2 <= 1'b1;
                end
            end
            S_X: begin
                if (wait2) wait2 <= 1'b0;
                else begin : xpos
                    reg [8:0] sx;
                    sx = {spr_d, 1'b0} - 9'd8;
                    if (cocktail) sx = 9'd466 - sx + (xoffs[1] ? 9'd16 : 9'd0);
                    else          sx = sx + (xoffs[0] ? 9'd16 : 9'd0);
                    sxw <= sx[8:1];
                    k   <= 4'd0;
                    st  <= S_ROM;
                end
            end
            S_ROM: begin : fetch
                // byte k: quarter k[1:0], row byte k[3:2] -> source pixels 8*k[3:2] + 2*k[1:0] + {0,1}
                reg [3:0] w;
                w = {k[3:2], k[1:0]};                          // source word 0-15 within the row
                rom_a <= {2'b00, k[1:0], code, row, k[3:2]};   // quarter * 8K + code * 128 + row * 4 + column
                dr_ra <= sxw + {4'd0, hfl ? ~w : w};
                st    <= S_WAIT;
            end
            S_WAIT: st <= S_RMW;
            S_RMW: begin
                dr_wa <= dr_ra;
                dr_wd <= dr_q | (hfl ? {rom_d[3:0], rom_d[7:4]} : rom_d);
                dr_we <= 1'b1;
                if (k == 4'd15) begin
                    if (n == 7'd127) st <= S_IDLE;
                    else begin
                        n     <= n + 7'd1;
                        spr_a <= {n + 7'd1, 2'd0};
                        st    <= S_Y; wait2 <= 1'b1;
                    end
                end else begin
                    k  <= k + 4'd1;
                    st <= S_ROM;
                end
            end
            default: ;
        endcase
    end

endmodule
