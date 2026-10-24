//============================================================================
//
//  Midway 91464 MCR/III Super Video Gen: 32 x 32 sprites into a double line buffer, one pixel + colour per word
//  Copyright (C) 2026 Rodimus
//
//  Facts: Claude/mcr_schematic_facts.md Part B board 2 (91464 schematic) and MAME bally/mcr_v.cpp render_sprites_91464.
//  Sprite RAM (on the CPU board, read through port B): 128 x 4 bytes = Y, flags (b3-0 colour group, b4 H flip, b5 V flip),
//  picture, X. Screen position (512 x 480 space): x = (X - 3) * 2, y = (241 - Y) * 2; cocktail: flips inverted,
//  x = 480 - x, y = 452 - y. Both wrap at 512; video lines 0 and 1 never get sprites (MAME).
//  Picture = byte 2, masked to the ROM size (byte 152 [5:4]: 64K = 128 pictures, 128K = 256), as MAME's % elements.
//  Sprite ROM (rom3, shared): four quarters, same row layout as the 91399 (4 bytes from each quarter per row).
//  Line buffer (schematic): a nonzero pixel overwrites what is there (no OR-merge, the last object wins), stored with
//  the colour group COL = ~flags[3:0]; the buffer is cleared as it is shown. Pixel 0 is never written.
//
//  While display line n is shown, the other buffer is drawn for the next display line of the same field
//  (2 video lines down, or line 0 / 1 of the next field after the last line).
//
//============================================================================

module vgen_91464
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
    input             rom128,         // sprite ROM 128K (else 64K)

    output reg  [8:0] spr_a,          // sprite RAM read (CPU board port B)
    input       [7:0] spr_d,
    output reg [16:0] rom_a,          // rom3 (sprites)
    input       [7:0] rom_d,

    output reg  [7:0] spix,           // {COL3-0, VID3-0} for slot h, valid from phase 5 (same timing as the 91399)

    // line buffers 0 / 1 (shared with the other sprite board, rtl/mcr_board.sv): port A reads, port B writes
    output      [8:0] lb_aa [2],
    input       [7:0] lb_qa [2],
    output      [8:0] lb_ab [2],
    output      [7:0] lb_db [2],
    output            lb_web [2]
);

    // ------------------------------------------------------------ line buffers (2 x 512 pixels)
    reg        sel;                   // buffer being drawn; ~sel is shown
    reg  [8:0] dr_wa, ds_a;
    reg  [7:0] dr_wd;
    reg        dr_we, ds_clr;
    wire [7:0] b0_qa = lb_qa[0], b1_qa = lb_qa[1];
    assign lb_aa[0]  = ds_a;
    assign lb_ab[0]  = sel ? ds_a : dr_wa;
    assign lb_db[0]  = sel ? 8'h00 : dr_wd;
    assign lb_web[0] = sel ? ds_clr : dr_we;
    assign lb_aa[1]  = ds_a;
    assign lb_ab[1]  = sel ? dr_wa : ds_a;
    assign lb_db[1]  = sel ? dr_wd : 8'h00;
    assign lb_web[1] = sel ? dr_we : ds_clr;
    wire [7:0] ds_q = sel ? b0_qa : b1_qa;

    // ------------------------------------------------------------ display side: read, then clear the pixel
    always @(posedge clk) begin
        ds_clr <= 1'b0;
        case (pix_phase)
            3'd2: ds_a <= flip_x ? ~h[8:0] : h[8:0];
            3'd4: begin
                spix   <= ds_q;
                ds_clr <= 1'b1;
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

    localparam S_IDLE = 3'd0, S_Y = 3'd1, S_FLAGS = 3'd2, S_CODE = 3'd3, S_X = 3'd4, S_ROM = 3'd5, S_WAIT = 3'd6, S_PIX = 3'd7;
    reg  [2:0] st;
    reg  [6:0] n;                     // sprite index
    reg  [8:0] ty;
    reg  [4:0] row;
    reg  [7:0] code;
    reg  [3:0] col;
    reg        hfl;
    reg  [8:0] sx;                    // pixel position of the sprite's left edge
    reg  [3:0] k;                     // byte within the row: k[3:2] = column byte, k[1:0] = quarter
    reg        px1;                   // second pixel of the byte
    reg        wait2;

    wire       line_start = ce_pix && (h == 10'd634);        // the next slot is h = 0 of a new line
    wire [8:0] sy_n  = 9'd482 - {spr_d, 1'b0};               // (241 - Y) * 2, mod 512
    wire [8:0] sy_c  = 9'd452 - sy_n;
    wire [8:0] sy    = cocktail ? sy_c : sy_n;
    wire [8:0] dy    = ty - sy;

    // source pixel of this write: 8 * column byte + 2 * quarter + px1 (high nibble first), mirrored by the H flip
    wire [4:0] spx   = {k[3:2], k[1:0], px1};
    wire [3:0] pix   = px1 ? rom_d[3:0] : rom_d[7:4];

    always @(posedge clk) begin
        dr_we <= 1'b0;
        if (reset) begin
            st <= S_IDLE; sel <= 1'b0;
        end else if (line_start) begin
            sel <= ~sel;              // what was drawn is shown next; draw the other buffer
            ty  <= ty_w;
            n   <= 7'd0;
            spr_a <= 9'd0;
            st  <= (ty_w < 9'd2 || ty_w >= 9'd480) ? S_IDLE : S_Y;
            wait2 <= 1'b1;
        end else case (st)
            S_Y: begin
                if (wait2) wait2 <= 1'b0;
                else if (dy < 9'd32) begin
                    row   <= dy[4:0];
                    spr_a <= {n, 2'd1};
                    st    <= S_FLAGS; wait2 <= 1'b1;
                end else if (n == 7'd127)
                    st <= S_IDLE;
                else begin
                    n     <= n + 7'd1;
                    spr_a <= {n + 7'd1, 2'd0};
                    wait2 <= 1'b1;
                end
            end
            S_FLAGS: begin
                if (wait2) wait2 <= 1'b0;
                else begin
                    col   <= ~spr_d[3:0];
                    hfl   <= spr_d[4] ^ cocktail;
                    row   <= row ^ {5{spr_d[5] ^ cocktail}};
                    spr_a <= {n, 2'd2};
                    st    <= S_CODE; wait2 <= 1'b1;
                end
            end
            S_CODE: begin
                if (wait2) wait2 <= 1'b0;
                else begin
                    code  <= rom128 ? spr_d : {1'b0, spr_d[6:0]};
                    spr_a <= {n, 2'd3};
                    st    <= S_X; wait2 <= 1'b1;
                end
            end
            S_X: begin
                if (wait2) wait2 <= 1'b0;
                else begin : xpos
                    reg [8:0] x;
                    x = {spr_d, 1'b0} - 9'd6;
                    sx <= cocktail ? (9'd480 - x) : x;
                    k  <= 4'd0;
                    st <= S_ROM;
                end
            end
            S_ROM: begin
                // quarter k[1:0], row byte k[3:2]: quarter * (32K or 16K) + code * 128 + row * 4 + column
                rom_a <= rom128 ? {k[1:0], code, row, k[3:2]} : {1'b0, k[1:0], code[6:0], row, k[3:2]};
                px1   <= 1'b0;
                st    <= S_WAIT;
            end
            S_WAIT: st <= S_PIX;
            S_PIX: begin
                dr_wa <= sx + {4'd0, hfl ? ~spx : spx};
                dr_wd <= {col, pix};
                dr_we <= (pix != 4'd0);
                if (!px1)
                    px1 <= 1'b1;
                else if (k == 4'd15) begin
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
