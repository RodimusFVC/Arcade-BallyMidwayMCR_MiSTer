//============================================================================
//
//  Bally Midway MCR I / II / III board dispatcher
//  Copyright (C) 2026 Rodimus
//
//  One rbf for every bally/mcr.cpp set; the board set is chosen at run time by MRA index-1 bytes 148-157
//  (tools/mra_cfg.py). Shared CPUs / sound chips and ROM storage live here (chip pool convention,
//  rtl/chip_pool_pkg.sv); boards in rtl/boards/ drive them through bus ports.
//
//  Boards so far: CPU board (90009 / 90010 / 91490 decode, timing, background, palette), 91399 and 91464 sprite boards, SSIO.
//
//  Clock: clk = 80 MHz. ce_pix = clk / 8 = 10 MHz (board 9.984 MHz), ce_cpu = clk / 32 = 2.5 MHz (2.496), 91490:
//  clk / 16 = 5 MHz (MAME 5 MHz);
//  SSIO: ce_8m = clk / 10 = 8 MHz, ce_2m = clk / 40 = 2 MHz (16 MHz crystal / 2, / 8).
//
//  ioctl index 0 (tools/make_mra.py): 00000 main CPU (64K) | 10000 SSIO (16K) | 14000 lamp PROM | 14040 SSIO PROM |
//                                     18000 background (32K) | 20000 sprites (128K) | 40000 SNT / TCS (32K)
//
//============================================================================

module mcr_board
(
    input             clk,
    input             reset,
    input             pause,

    input       [7:0] cfg_cpu,        // byte 148: 0 = 90009, 1 = 90010, 2 = 91475, 3 = 91490
    input       [7:0] cfg_spr,        // byte 149
    input       [7:0] cfg_snd,        // byte 150
    input       [7:0] cfg_io,         // byte 151
    input       [7:0] cfg_xoffs,      // byte 152: sprite X offsets, gfx ROM sizes

    input       [7:0] ip0, ip1, ip2, ip3, ip4,
    input       [7:0] ssio_dip,       // SSIO DIP switch block (sound CPU F000)
    input       [7:0] p24, p2c,       // Draw Poker extra ports

    // analog controls (rtl/mcr_inputs.sv): cfg bytes 153-163, OSD options, MiSTer devices per player
    input       [7:0] an_desc [3],
    input       [7:0] an_min, an_max,
    input       [7:0] an_sens [3],
    input       [7:0] an_keyd [3],
    input       [2:0] osd_sens,
    input             osd_rev,
    input             osd_wheel,
    input       [8:0] spinner [4],
    input       [7:0] paddle [4],
    input      [15:0] lstick [4],
    input      [15:0] rstick [4],
    input      [24:0] mouse,
    input       [3:0] dpad [4],       // {U, D, L, R}

    input      [24:0] ioctl_addr,
    input       [7:0] ioctl_dout,
    input             ioctl_wr0,

    input      [10:0] nv_addr,
    input       [7:0] nv_din,
    input             nv_we,
    output      [7:0] nv_dout,

    input             crt_flip,
    input             mirror_x,       // cfg byte 1 [1]
    input             mirror_y,       // cfg byte 1 [2]
    input       [3:0] h_adj,
    input       [3:0] v_adj,

    output            ce_pix,
    output      [7:0] video_r,
    output      [7:0] video_g,
    output      [7:0] video_b,
    output            video_hs,
    output            video_vs,
    output            video_hblank,
    output            video_vblank,
    output            video_field,

    output     [15:0] audio_l,
    output     [15:0] audio_r,
    output      [7:0] op0,
    output            wd_reset
);

    // ------------------------------------------------------------ clock enables
    reg [4:0] phase = 5'd0;
    always @(posedge clk) phase <= phase + 5'd1;
    assign ce_pix     = (phase[2:0] == 3'd7);
    wire   ce_cpu     = (cfg_cpu[1:0] == 2'd3) ? (phase[3:0] == 4'd15) : (phase == 5'd31);

    reg [5:0] sphase = 6'd0;                       // SSIO: 80 MHz / 40
    always @(posedge clk) sphase <= (sphase == 6'd39) ? 6'd0 : sphase + 6'd1;
    wire   ce_8m      = (sphase == 6'd9) || (sphase == 6'd19) || (sphase == 6'd29) || (sphase == 6'd39);
    wire   ce_2m      = (sphase == 6'd39);

    /////////////////////////////  Shared CPU / sound chip pool  /////////////////////////////

    // Main Z80: every CPU board (90009, 90010, 91475, 91490). RESET_n on the bus: the board watchdog resets it.
    chip_pool_pkg::z80r_i_t cpu_z80_0_i, z80_0_i;
    chip_pool_pkg::z80r_o_t z80_0_o;
    assign z80_0_i = cpu_z80_0_i;

    wire [15:0] z80_0_a;
    wire  [7:0] z80_0_do;
    wire        z80_0_m1_n, z80_0_mreq_n, z80_0_iorq_n, z80_0_rd_n, z80_0_wr_n, z80_0_rfsh_n;

    T80s #(.Mode(0), .T2Write(1), .IOWait(1)) z80_0
    (
        .RESET_n(~reset & z80_0_i.reset_n), .CLK(clk), .CEN(z80_0_i.ce), .WAIT_n(1'b1), .INT_n(z80_0_i.int_n), .NMI_n(z80_0_i.nmi_n), .BUSRQ_n(1'b1),
        .M1_n(z80_0_m1_n), .MREQ_n(z80_0_mreq_n), .IORQ_n(z80_0_iorq_n), .RD_n(z80_0_rd_n), .WR_n(z80_0_wr_n), .RFSH_n(z80_0_rfsh_n),
        .HALT_n(), .BUSAK_n(), .A(z80_0_a), .DI(z80_0_i.di), .DO(z80_0_do)
    );

    assign z80_0_o.a = z80_0_a;           assign z80_0_o.dout = z80_0_do;
    assign z80_0_o.m1_n = z80_0_m1_n;     assign z80_0_o.mreq_n = z80_0_mreq_n; assign z80_0_o.iorq_n = z80_0_iorq_n;
    assign z80_0_o.rd_n = z80_0_rd_n;     assign z80_0_o.wr_n = z80_0_wr_n;     assign z80_0_o.rfsh_n = z80_0_rfsh_n;

    // Sound Z80: SSIO (all sets; held in reset where the SSIO CPU is not fitted)
    chip_pool_pkg::z80r_i_t ssio_z80_1_i, z80_1_i;
    chip_pool_pkg::z80r_o_t z80_1_o;
    assign z80_1_i = ssio_z80_1_i;

    wire [15:0] z80_1_a;
    wire  [7:0] z80_1_do;
    wire        z80_1_m1_n, z80_1_mreq_n, z80_1_iorq_n, z80_1_rd_n, z80_1_wr_n, z80_1_rfsh_n;

    T80s #(.Mode(0), .T2Write(1), .IOWait(1)) z80_1
    (
        .RESET_n(~reset & z80_1_i.reset_n), .CLK(clk), .CEN(z80_1_i.ce), .WAIT_n(1'b1), .INT_n(z80_1_i.int_n), .NMI_n(z80_1_i.nmi_n), .BUSRQ_n(1'b1),
        .M1_n(z80_1_m1_n), .MREQ_n(z80_1_mreq_n), .IORQ_n(z80_1_iorq_n), .RD_n(z80_1_rd_n), .WR_n(z80_1_wr_n), .RFSH_n(z80_1_rfsh_n),
        .HALT_n(), .BUSAK_n(), .A(z80_1_a), .DI(z80_1_i.di), .DO(z80_1_do)
    );

    assign z80_1_o.a = z80_1_a;           assign z80_1_o.dout = z80_1_do;
    assign z80_1_o.m1_n = z80_1_m1_n;     assign z80_1_o.mreq_n = z80_1_mreq_n; assign z80_1_o.iorq_n = z80_1_iorq_n;
    assign z80_1_o.rd_n = z80_1_rd_n;     assign z80_1_o.wr_n = z80_1_wr_n;     assign z80_1_o.rfsh_n = z80_1_rfsh_n;

    // AY-3-8910 x2: SSIO (AY0 left, AY1 right)
    chip_pool_pkg::ay_i_t   [1:0] ssio_ay_i, ay_i;
    chip_pool_pkg::ayio_o_t [1:0] ay_o;
    assign ay_i = ssio_ay_i;

    genvar ai;
    generate
        for (ai = 0; ai < 2; ai = ai + 1) begin : g_ay
            wire [7:0] dout, a, b, c, ioa, iob;
            wire [9:0] snd;
            jt49_bus ay
            (
                .rst_n(~reset), .clk(clk), .clk_en(ay_i[ai].ce), .bdir(ay_i[ai].bdir), .bc1(ay_i[ai].bc1), .din(ay_i[ai].din), .sel(1'b1),
                .dout(dout), .sound(snd), .A(a), .B(b), .C(c), .sample(),
                .IOA_in(ay_i[ai].ioa_in), .IOA_out(ioa), .IOB_in(ay_i[ai].iob_in), .IOB_out(iob)
            );
            assign ay_o[ai].dout = dout; assign ay_o[ai].a = a; assign ay_o[ai].b = b; assign ay_o[ai].c = c;
            assign ay_o[ai].ioa_out = ioa; assign ay_o[ai].iob_out = iob; assign ay_o[ai].sound = snd;
        end
    endgenerate

    /////////////////////////////  Shared ROM blocks  /////////////////////////////

    // rom0: main CPU program, ioctl 00000-0FFFF (port A loads, port B = CPU)
    wire        load_rom0 = ioctl_wr0 && (ioctl_addr[24:16] == 9'h000);
    wire [15:0] rom0_a;
    wire  [7:0] rom0_d;
    dpram_dc #(.widthad_a(16)) rom0
    (
        .clock_a(clk), .address_a(ioctl_addr[15:0]), .data_a(ioctl_dout), .wren_a(load_rom0), .q_a(),
        .clock_b(clk), .address_b(rom0_a), .data_b(8'h00), .wren_b(1'b0), .q_b(rom0_d)
    );

    // rom1: background tiles, ioctl 18000-1FFFF; port A is the second read port when not loading
    wire        load_rom1 = ioctl_wr0 && (ioctl_addr[24:15] == 10'b0000000011);
    wire [14:0] rom1_a, rom1b_a;
    wire  [7:0] rom1_d, rom1b_d;
    dpram_dc #(.widthad_a(15)) rom1
    (
        .clock_a(clk), .address_a(load_rom1 ? ioctl_addr[14:0] : rom1b_a), .data_a(ioctl_dout), .wren_a(load_rom1), .q_a(rom1b_d),
        .clock_b(clk), .address_b(rom1_a), .data_b(8'h00), .wren_b(1'b0), .q_b(rom1_d)
    );

    // rom2: SSIO program, ioctl 10000-13FFF
    wire        load_rom2 = ioctl_wr0 && (ioctl_addr[24:14] == 11'd4);
    wire [13:0] rom2_a;
    wire  [7:0] rom2_d;
    dpram_dc #(.widthad_a(14)) rom2
    (
        .clock_a(clk), .address_a(ioctl_addr[13:0]), .data_a(ioctl_dout), .wren_a(load_rom2), .q_a(),
        .clock_b(clk), .address_b(rom2_a), .data_b(8'h00), .wren_b(1'b0), .q_b(rom2_d)
    );

    // rom3: sprites, ioctl 20000-3FFFF
    wire        load_rom3 = ioctl_wr0 && (ioctl_addr[24:17] == 8'd1);
    wire [16:0] rom3_a;
    wire  [7:0] rom3_d;
    dpram_dc #(.widthad_a(17)) rom3
    (
        .clock_a(clk), .address_a(ioctl_addr[16:0]), .data_a(ioctl_dout), .wren_a(load_rom3), .q_a(),
        .clock_b(clk), .address_b(rom3_a), .data_b(8'h00), .wren_b(1'b0), .q_b(rom3_d)
    );

    /////////////////////////////  Boards  /////////////////////////////

    wire [31:0] ssio_cmd;
    wire  [7:0] op4, ssio_status;
    wire        cpu_wd;
    wire  [9:0] ras_h;
    wire  [7:0] ras_line;
    wire        ras_field;
    wire  [8:0] spr_ra, spr_ra_99, spr_ra_64;
    wire  [7:0] spr_rd;
    wire [16:0] rom3_a_99, rom3_a_64;
    wire  [3:0] spix_99;
    wire  [7:0] spix_64;
    wire        spr_64 = cfg_spr[0];
    wire  [7:0] spix   = spr_64 ? spix_64 : {4'h0, spix_99};
    assign spr_ra = spr_64 ? spr_ra_64 : spr_ra_99;
    assign rom3_a = spr_64 ? rom3_a_64 : rom3_a_99;

    // ------------------------------------------------------------ inputs: analog fields + per-game handlers
    wire [7:0] dp_out;
    wire [7:0] gip0, gip1, gip2, gip4;
    reg        vbl_d = 1'b0;
    always @(posedge clk) vbl_d <= video_vblank;
    wire       fld_tick = video_vblank && !vbl_d;
    mcr_inputs inputs
    (
        .clk(clk), .reset(reset), .frame(fld_tick && video_field), .fld_tick(fld_tick),
        .io_id(cfg_io), .an_desc(an_desc), .an_min(an_min), .an_max(an_max), .an_sens(an_sens), .an_keyd(an_keyd),
        .osd_sens(osd_sens), .osd_rev(osd_rev), .osd_wheel(osd_wheel),
        .spinner(spinner), .paddle(paddle), .lstick(lstick), .rstick(rstick), .mouse(mouse), .dpad(dpad),
        .op0(op0), .op4(op4), .dp_out(dp_out),
        .ip0_i(ip0), .ip1_i(ip1), .ip2_i(ip2), .ip4_i(ip4),
        .ip0(gip0), .ip1(gip1), .ip2(gip2), .ip4(gip4)
    );

    mcr_cpu cpu
    (
        .clk(clk), .reset(reset), .pause(pause), .ce_cpu(ce_cpu), .ce_pix(ce_pix), .pix_phase(phase[2:0]),
        .board(cfg_cpu[1:0]), .dpoker(cfg_io == 8'd3), .twotiger(cfg_io == 8'd5), .spr_91464(spr_64), .bg32k(cfg_xoffs[3]),
        .z80_0_i(cpu_z80_0_i), .z80_0_o(z80_0_o),
        .rom0_a(rom0_a), .rom0_d(rom0_d), .rom1_a(rom1_a), .rom1_d(rom1_d), .rom1b_a(rom1b_a), .rom1b_d(rom1b_d),
        .ip0(gip0), .ip1(gip1), .ip2(gip2), .ip3(ip3), .ip4(gip4),
        .ssio_status(ssio_status), .p24(p24), .p2c(p2c), .dp_out(dp_out),
        .ssio_cmd(ssio_cmd), .op0(op0), .op4(op4),
        .nv_addr(nv_addr), .nv_din(nv_din), .nv_we(nv_we), .nv_dout(nv_dout),
        .crt_flip(crt_flip), .mirror_x(mirror_x), .mirror_y(mirror_y), .h_adj(h_adj), .v_adj(v_adj),
        .ras_h(ras_h), .ras_line(ras_line), .ras_field(ras_field), .spr_ra(spr_ra), .spr_rd(spr_rd), .spix(spix),
        .video_r(video_r), .video_g(video_g), .video_b(video_b),
        .video_hs(video_hs), .video_vs(video_vs), .video_hblank(video_hblank), .video_vblank(video_vblank),
        .video_field(video_field),
        .wd_reset(cpu_wd)
    );
    assign wd_reset = cpu_wd;

    // sprite line buffers: one pair, used by whichever sprite board the set has (the other's writes are dropped)
    wire [8:0] lb_aa_99 [2], lb_ab_99 [2], lb_aa_64 [2], lb_ab_64 [2];
    wire [7:0] lb_db_99 [2], lb_db_64 [2], lb_qa [2];
    wire       lb_we_99 [2], lb_we_64 [2];
    genvar li;
    generate
        for (li = 0; li < 2; li = li + 1) begin : g_lb
            dpram_dc #(.widthad_a(9)) lb
            (
                .clock_a(clk), .address_a(spr_64 ? lb_aa_64[li] : lb_aa_99[li]), .data_a(8'h00), .wren_a(1'b0),
                .q_a(lb_qa[li]),
                .clock_b(clk), .address_b(spr_64 ? lb_ab_64[li] : lb_ab_99[li]),
                .data_b(spr_64 ? lb_db_64[li] : lb_db_99[li]), .wren_b(spr_64 ? lb_we_64[li] : lb_we_99[li]), .q_b()
            );
        end
    endgenerate

    vgen_91399 vgen
    (
        .clk(clk), .reset(reset), .ce_pix(ce_pix), .pix_phase(phase[2:0]),
        .h(ras_h), .line(ras_line), .field(ras_field), .cocktail(op0[6]), .flip_x(crt_flip ^ mirror_x), .flip_y(crt_flip ^ mirror_y), .xoffs(cfg_xoffs[1:0]),
        .spr_a(spr_ra_99), .spr_d(spr_rd), .rom_a(rom3_a_99), .rom_d(rom3_d), .spix(spix_99),
        .lb_aa(lb_aa_99), .lb_qa(lb_qa), .lb_ab(lb_ab_99), .lb_db(lb_db_99), .lb_web(lb_we_99)
    );

    vgen_91464 vgen3
    (
        .clk(clk), .reset(reset), .ce_pix(ce_pix), .pix_phase(phase[2:0]),
        .h(ras_h), .line(ras_line), .field(ras_field), .cocktail(op0[6]), .flip_x(crt_flip ^ mirror_x), .flip_y(crt_flip ^ mirror_y), .rom128(cfg_xoffs[5]),
        .spr_a(spr_ra_64), .spr_d(spr_rd), .rom_a(rom3_a_64), .rom_d(rom3_d), .spix(spix_64),
        .lb_aa(lb_aa_64), .lb_qa(lb_qa), .lb_ab(lb_ab_64), .lb_db(lb_db_64), .lb_web(lb_we_64)
    );

    ssio ssio
    (
        .clk(clk), .reset(reset | cpu_wd), .pause(pause), .run(cfg_snd[0]), .ce_cpu(ce_2m), .ce_8m(ce_8m),
        .z80_1_i(ssio_z80_1_i), .z80_1_o(z80_1_o), .ay_i(ssio_ay_i), .ay_o(ay_o),
        .rom2_a(rom2_a), .rom2_d(rom2_d),
        .prom_we(ioctl_wr0 && (ioctl_addr[24:5] == 20'h00A02)), .prom_a(ioctl_addr[4:0]), .prom_d(ioctl_dout),
        .cmd(ssio_cmd), .status(ssio_status), .dip(ssio_dip),
        .audio_l(audio_l), .audio_r(audio_r)
    );



endmodule
