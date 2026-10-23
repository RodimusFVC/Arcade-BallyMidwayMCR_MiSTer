//============================================================================
//
//  MCR CPU board: Z80 bus decode, RAMs, Z80 CTC, watchdog, raster timing, background tiles, palette
//  Copyright (C) 2026 Rodimus
//
//  Boards: 90010 Super CPU (MCR II) and 91490 (MCR III, no schematic: MAME). 90009 / 91475 come later.
//  Facts: Claude/mcr_schematic_facts.md (Part A = 90010 schematic) and MAME bally/mcr.cpp, mcr_v.cpp.
//
//  90010 memory (MAME cpu_90010_map, schematic SEL0-5):
//    0000-BFFF ROM (rom0, shared)          C000-C7FF battery RAM, mirrored to DFFF
//    E000-E1FF sprite RAM, mirrored over E000-E7FF / F000-F7FF (A11 = 0)
//    E800-EFFF video RAM, mirrored at F800 (A11 = 1); writes to its top 128 bytes also set the 64 x 9 palette
//      (entry = offset[6:1], bit 8 = A0)
//  91490 memory (MAME cpu_91490_map): 0000-DFFF ROM, E000-E7FF battery RAM, E800-E9FF sprite RAM (mirror EA00),
//    F000-F7FF video RAM, F800-FFFF palette (write only, entry = A[6:1], bit 8 = A0). Unmapped reads FF.
//  I/O (low 8 bits, both boards): 00-04 read SSIO input ports IP0-IP4 (mirror 18), 07 read SSIO status, 00-03 write output
//    port 0 (D6 = cocktail flip, D0-D2 coin meters), 04-07 output port 4, 1C-1F SSIO command latches,
//    E0 watchdog clear, F0-F3 CTC. Unmapped reads FF.
//  Interrupts: CTC only (IM2 daisy, one device). TRG1 = ZC/TO0, TRG2 = VBLANK (each field),
//    TRG3 = "493" (once per frame). Watchdog: 16 x "493" without an E0 write resets the Z80 and CTC.
//
//============================================================================

module mcr_cpu
(
    input             clk,
    input             reset,
    input             pause,
    input             ce_cpu,         // 2.5 MHz (90010), 5 MHz (91490)
    input       [1:0] board,          // cfg byte 148: 1 = 90010, 3 = 91490
    input             spr_91464,      // cfg byte 149: sprite colour from the sprite board, not the tile
    input             bg32k,          // cfg byte 152 [3]: background ROM 32K (91490: 10-bit tile codes)
    input             ce_pix,         // 10 MHz, the last clock of each pixel slot
    input       [2:0] pix_phase,      // 0-7 within the pixel slot (ce_pix = 7)

    // the shared main Z80 (dispatcher pool slot z80_0)
    output chip_pool_pkg::z80r_i_t z80_0_i,
    input  chip_pool_pkg::z80r_o_t z80_0_o,

    // shared ROM blocks: rom0 = main program, rom1 = background tiles (port B + second read port A)
    output     [15:0] rom0_a,
    input       [7:0] rom0_d,
    output reg [14:0] rom1_a,
    input       [7:0] rom1_d,
    output reg [14:0] rom1b_a,
    input       [7:0] rom1b_d,

    // SSIO side of the bus (inputs, status, latches, output ports)
    input       [7:0] ip0, ip1, ip2, ip3, ip4,
    input       [7:0] ssio_status,
    output reg [31:0] ssio_cmd,       // command latches 1C-1F, latch n = [8n+7:8n]
    output reg  [7:0] op0,
    output reg  [7:0] op4,

    // battery RAM <-> MRA NVRAM (index 4)
    input      [10:0] nv_addr,
    input       [7:0] nv_din,
    input             nv_we,
    output      [7:0] nv_dout,

    input             crt_flip,
    input       [3:0] h_adj,
    input       [3:0] v_adj,

    // sprite board: raster position, sprite RAM read port, sprite pixel back
    output      [9:0] ras_h,
    output      [7:0] ras_line,
    output            ras_field,
    input       [8:0] spr_ra,
    output      [7:0] spr_rd,
    input       [7:0] spix,           // 91399: {0, VID3-0}; 91464: {COL3-0, VID3-0}

    output reg  [2:0] video_r,
    output reg  [2:0] video_g,
    output reg  [2:0] video_b,
    output            video_hs,
    output            video_vs,
    output reg        video_hblank,
    output reg        video_vblank,
    output            video_field,

    output            wd_reset
);

    // ------------------------------------------------------------ raster
    wire [9:0] h;
    wire [7:0] line;
    wire       field, hblank, vblank, v493;
    mcr_timing timing
    (
        .clk(clk), .reset(reset), .ce_pix(ce_pix), .h_adj(h_adj), .v_adj(v_adj),
        .h(h), .line(line), .field(field), .hblank(hblank), .vblank(vblank),
        .hs(video_hs), .vs(video_vs), .v493(v493)
    );
    assign video_field = field;
    assign ras_h = h; assign ras_line = line; assign ras_field = field;

    // ------------------------------------------------------------ watchdog
    reg  [3:0] wd_cnt;
    reg  [5:0] wd_pulse;
    reg        v493_d;
    wire       wd_clear;
    always @(posedge clk) begin
        if (reset) begin
            wd_cnt <= 4'd0; wd_pulse <= 6'd0; v493_d <= 1'b0;
        end else begin
            v493_d <= v493;
            if (wd_pulse != 6'd0) wd_pulse <= wd_pulse - 6'd1;
            if (wd_clear)
                wd_cnt <= 4'd0;
            else if (v493 && !v493_d && !pause) begin       // frozen while paused
                if (wd_cnt == 4'd15) begin
                    wd_cnt <= 4'd0; wd_pulse <= 6'd63;
                end else
                    wd_cnt <= wd_cnt + 4'd1;
            end
        end
    end
    assign wd_reset = (wd_pulse != 6'd0);
    wire cpu_reset = reset | wd_reset;                     // the watchdog resets the Z80 and CTC, never itself

    // ------------------------------------------------------------ Z80 bus
    wire [15:0] A    = z80_0_o.a;
    wire  [7:0] dout = z80_0_o.dout;
    wire        m1   = ~z80_0_o.m1_n;
    wire        mreq = ~z80_0_o.mreq_n;
    wire        iorq = ~z80_0_o.iorq_n;
    wire        rd   = ~z80_0_o.rd_n;
    wire        wr   = ~z80_0_o.wr_n;
    wire        rfsh = ~z80_0_o.rfsh_n;

    wire mem_rd = mreq && rd && !rfsh;
    wire mem_wr = mreq && wr && !cpu_reset;
    wire io_rd  = iorq && rd && !m1;
    wire io_wr  = iorq && wr && !m1 && !cpu_reset;
    wire inta_l = iorq && m1;
    wire fetch  = m1 && mreq && rd && !rfsh;

    wire mcr3    = (board == 2'd3);
    wire cs_rom  = mcr3 ? (A[15:13] != 3'b111)  : (A[15:14] != 2'b11);          // 0000-DFFF / 0000-BFFF
    wire cs_nv   = mcr3 ? (A[15:11] == 5'b11100) : (A[15:13] == 3'b110);        // E000-E7FF / C000-DFFF
    wire cs_spr  = mcr3 ? (A[15:10] == 6'b111010) : ((A[15:13] == 3'b111) && !A[11]);
    wire cs_vram = mcr3 ? (A[15:11] == 5'b11110) : ((A[15:13] == 3'b111) &&  A[11]);
    wire cs_pal  = mcr3 ? (A[15:11] == 5'b11111) : (cs_vram && (A[10:7] == 4'hF));

    // end-of-cycle strobes (from the LaserdiscGames LD-V1000 firmware board)
    reg  [7:0] ba_q, bd_q;
    reg        iw_q, ia_q, f_q, last_ed;
    reg  [7:0] fop_q;
    wire [7:0] cpu_di;
    always @(posedge clk) begin
        if (io_wr) begin ba_q <= A[7:0]; bd_q <= dout; end
        if (fetch) fop_q <= cpu_di;
        iw_q <= io_wr; ia_q <= inta_l; f_q <= fetch;
    end
    wire io_wr_end = iw_q && !io_wr;
    wire inta_stb  = inta_l && !ia_q;
    wire fetch_end = f_q && !fetch;
    wire reti_stb  = fetch_end && last_ed && (fop_q == 8'h4D);
    always @(posedge clk) begin
        if (cpu_reset)      last_ed <= 1'b0;
        else if (fetch_end) last_ed <= (fop_q == 8'hED);
    end

    wire io_ssio = (A[7:5] == 3'b000);
    assign wd_clear = io_wr && (A[7:0] == 8'hE0);
    wire ctc_we  = io_wr_end && (ba_q[7:2] == 6'b111100);

    always @(posedge clk) begin
        if (reset) begin
            op0 <= 8'h00; op4 <= 8'h00;
            ssio_cmd <= 32'd0;
        end else if (io_wr) begin
            if (A[7:0] <= 8'h03) op0 <= dout;
            if (A[7:2] == 6'b000001) op4 <= dout;
            if (A[7:2] == 6'b000111) ssio_cmd[{A[1:0], 3'b000} +: 8] <= dout;
        end
    end

    // ------------------------------------------------------------ RAMs
    wire [7:0] nv_q, spr_q, vram_q;
    dpram_dc #(.widthad_a(11)) u_nvram
    (
        .clock_a(clk), .address_a(A[10:0]), .data_a(dout), .wren_a(mem_wr && cs_nv), .q_a(nv_q),
        .clock_b(clk), .address_b(nv_addr), .data_b(nv_din), .wren_b(nv_we), .q_b(nv_dout)
    );
    dpram_dc #(.widthad_a(9)) u_spriteram       // port B: the sprite board
    (
        .clock_a(clk), .address_a(A[8:0]), .data_a(dout), .wren_a(mem_wr && cs_spr), .q_a(spr_q),
        .clock_b(clk), .address_b(spr_ra), .data_b(8'h00), .wren_b(1'b0), .q_b(spr_rd)
    );
    reg  [10:0] vaddr;
    wire  [7:0] vq;
    dpram_dc #(.widthad_a(11)) u_videoram
    (
        .clock_a(clk), .address_a(A[10:0]), .data_a(dout), .wren_a(mem_wr && cs_vram), .q_a(vram_q),
        .clock_b(clk), .address_b(vaddr), .data_b(8'h00), .wren_b(1'b0), .q_b(vq)
    );

    // palette: 64 x 9 (93419), bit 8 = A0; 90010: written through the top 128 bytes of video RAM, 91490: F800-FFFF
    reg [8:0] pal [64];
    always @(posedge clk)
        if (mem_wr && cs_pal) pal[A[6:1]] <= {A[0], dout};

    assign rom0_a = A;

    // ------------------------------------------------------------ CTC
    wire [7:0] ctc_do, ctc_vec;
    wire [3:0] ctc_zc;
    wire       ctc_int_n;
    ldv_z80ctc u_ctc
    (
        .clk(clk), .reset_n(~cpu_reset), .ce(ce_cpu & ~pause),
        .wr_stb(ctc_we), .wsel(ba_q[1:0]), .wdata(bd_q), .rsel(A[1:0]), .dout(ctc_do),
        .trg({v493, vblank, ctc_zc[0], 1'b0}), .trg_en(~pause),
        .inta(inta_stb), .reti(reti_stb), .int_n(ctc_int_n), .ack_vector(ctc_vec), .zc_to(ctc_zc)
    );

    // ------------------------------------------------------------ power-on register state
    // A Z80 powers up with IX = IY = FFFF (MAME z80.cpp); the T80 register file starts at 0. Tapper / Timber push
    // IX or IY as a cold-start check and take the warm path on 0000 (halt with no CTC set up -> watchdog). After a
    // core reset (not a watchdog reset, which keeps the registers) the first fetches are fed
    // LD IX,FFFF / LD IY,FFFF / JP 0000 (addresses 0000-000A), then the ROM runs from 0000.
    reg        boot, rd_a_q;
    reg  [7:0] boot_d;
    wire       rd_a = mem_rd && (A == 16'h000A);
    always @(posedge clk) begin
        rd_a_q <= rd_a;
        if (reset)               boot <= 1'b1;
        else if (rd_a_q && !rd_a) boot <= 1'b0;            // the JP's last operand read has ended
    end
    always @(*) begin
        case (A[3:0])
            4'h0: boot_d = 8'hDD;  4'h1: boot_d = 8'h21;  4'h2: boot_d = 8'hFF;  4'h3: boot_d = 8'hFF;
            4'h4: boot_d = 8'hFD;  4'h5: boot_d = 8'h21;  4'h6: boot_d = 8'hFF;  4'h7: boot_d = 8'hFF;
            4'h8: boot_d = 8'hC3;  4'h9: boot_d = 8'h00;  default: boot_d = 8'h00;
        endcase
    end

    // ------------------------------------------------------------ CPU data in
    reg [7:0] di;
    always @(*) begin
        di = 8'hFF;
        if (inta_l)
            di = ctc_vec;
        else if (mem_rd) begin
            if (boot)         di = boot_d;
            else if (cs_rom)  di = rom0_d;
            else if (cs_nv)   di = nv_q;
            else if (cs_spr)  di = spr_q;
            else if (cs_vram) di = vram_q;
        end else if (io_rd) begin
            if (io_ssio) begin
                case (A[2:0])
                    3'd0: di = ip0;
                    3'd1: di = ip1;
                    3'd2: di = ip2;
                    3'd3: di = ip3;
                    3'd4: di = ip4;
                    3'd7: di = ssio_status;
                    default: di = 8'hFF;
                endcase
            end else if (A[7:2] == 6'b111100)
                di = ctc_do;
        end
    end
    assign cpu_di = di;

    assign z80_0_i.reset_n = ~cpu_reset;
    assign z80_0_i.ce      = ce_cpu & ~pause;
    assign z80_0_i.int_n   = ctc_int_n;
    assign z80_0_i.nmi_n   = 1'b1;
    assign z80_0_i.di      = di;

    // ------------------------------------------------------------ background (90010, 91490)
    // 32 x 30 tiles, 2 bytes each (MAME mcr_90010_get_tile_info): code = [8:0], H flip [9], V flip [10],
    // colour [12:11], sprite colour bank [15:14]. 91490 (mcr_91490_get_tile_info): code = [9:0], H flip [10],
    // V flip [11], colour [13:12], [15:14] unused (the 91464 colours its sprites). Tiles are 8 x 8 drawn 2 x 2
    // (16 x 16 on screen); 4 bpp from two ROM halves, 2 bits each (MAME mcr_bg_layout: planes 3-2 = high half,
    // 1-0 = low half, MSB first). Halves of 8K (code mod 512), or 16K with a 32K ROM (code mod 1024).
    // One pixel per 8-clock slot: video RAM low / high byte, then both ROM halves at once (rom1 ports B and A).
    wire       flip = op0[6] ^ crt_flip;
    wire [8:0] sx = flip ? ~h[8:0] : h[8:0];
    wire [8:0] vy = {line, field};
    wire [8:0] sy = flip ? (9'd479 - vy) : vy;

    reg  [7:0] lo;
    reg  [1:0] col_grp, spr_bank;
    reg  [1:0] pxk;
    always @(posedge clk) begin
        case (pix_phase)
            3'd0: vaddr <= {sy[8:4], sx[8:4], 1'b0};
            3'd2: begin lo <= vq; vaddr[0] <= 1'b1; end
            3'd4: begin : tile
                reg [2:0] px, py;
                reg [9:0] code;
                px   = sx[3:1] ^ {3{mcr3 ? vq[2] : vq[1]}};
                py   = sy[3:1] ^ {3{mcr3 ? vq[3] : vq[2]}};
                code = mcr3 ? {vq[1:0], lo} : {1'b0, vq[0], lo};
                if (mcr3 && bg32k) begin
                    rom1_a  <= {1'b0, code, py, px[2]};     // low half (gfx1 0000-3FFF)
                    rom1b_a <= {1'b1, code, py, px[2]};     // high half (gfx1 4000-7FFF)
                end else begin
                    rom1_a  <= {2'b00, code[8:0], py, px[2]};   // low half (gfx1 0000-1FFF)
                    rom1b_a <= {2'b01, code[8:0], py, px[2]};   // high half (gfx1 2000-3FFF)
                end
                col_grp <= mcr3 ? vq[5:4] : vq[4:3];
                spr_bank <= vq[7:6];
                pxk     <= px[1:0];
            end
            default: ;
        endcase
    end

    // pixel value at phase 6, out with the slot's end (one slot behind the counters, as hs / vs).
    // A sprite pixel wins where its low 3 bits are not 0, in the sprite bank of the tile under it (MAME, Part A),
    // or in the 91464's colour group (MAME render_sprites_91464 with the 91490: colour = COL1-0).
    wire [2:0] bit_hi = 3'd7 - {pxk, 1'b0};
    wire [3:0] pix = {rom1b_d[bit_hi], rom1b_d[bit_hi - 3'd1], rom1_d[bit_hi], rom1_d[bit_hi - 3'd1]};
    wire [1:0] sbank = spr_91464 ? spix[5:4] : spr_bank;
    wire [8:0] pcol = (spix[2:0] != 3'd0) ? pal[{sbank, spix[3:0]}] : pal[{col_grp, pix}];
    reg  [8:0] pcol_q;
    always @(posedge clk) if (pix_phase == 3'd6) pcol_q <= pcol;

    always @(posedge clk) begin
        if (ce_pix) begin
            video_hblank <= hblank;
            video_vblank <= vblank;
            if (hblank || vblank) begin
                video_r <= 3'd0; video_g <= 3'd0; video_b <= 3'd0;
            end else begin
                video_r <= pcol_q[8:6]; video_g <= pcol_q[2:0]; video_b <= pcol_q[5:3];
            end
        end
    end

endmodule
