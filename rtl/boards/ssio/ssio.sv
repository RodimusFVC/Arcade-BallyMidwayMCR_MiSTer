//============================================================================
//
//  Midway Super Sound I/O (SSIO, 90908 / 90913 / 91483 / 91657): sound Z80, two AY-3-8910s, volume circuit
//  Copyright (C) 2026 Rodimus
//
//  Facts: Claude/mcr_schematic_facts.md (Part C = SSIO schematic, Part 0 = MCR II guide) and MAME
//  bally/midway_sound.cpp. The main-CPU side (input ports, latches 1C-1F, status at 07) is decoded on the CPU board.
//
//  Sound Z80 (2 MHz = 16 MHz / 8), memory only (no I/O decode):
//    0000-3FFF ROM (rom2, shared)    8000-83FF RAM, mirrored to 8FFF    9000-9FFF read command latch A[1:0]
//    A000 / A001 / A002 AY0 address write / data read / data write (mirrored to AFFF), B000-BFFF the same for AY1
//    C000-CFFF write status (read by the main CPU at 07)    D000 LED (ignored)    E000-EFFF read: clear /SINT
//    F000-FFFF read DIP switches (6 fitted)
//  /SINT: a 7-bit counter clocked at 50 kHz (8 MHz / 160); bit 6 = interrupt (1.28 ms), cleared by an E000 read.
//
//  Volume circuit (schematic, Part C 1.4): an 8 MHz count of 0-159 (50 kHz period) walks the 82S123 PROM MSB
//  first; every 0 bit clocks six 4-bit down counters, reloaded each period from AY0 port A low / high, port B low
//  and AY1 port A low / high, port B low (one per channel). A counter at 0 closes a 4016 pair that switches both
//  resistors of that channel's 2-pole low-pass (Sallen-Key, 5.6K / 2.2 nF, Q = 1, ~12.9 kHz): while open the
//  filter holds. So the value sets how long per period the channel's filter tracks: 0 = always.
//  Mute: AY1 port B bit 7 = 1 (0.33 s ramp). AY0 = left, AY1 = right.
//  #unverified: the schematic read clocks the counters on every 0 bit; MAME counts only 1 -> 0 transitions.
//
//============================================================================

module ssio
(
    input             clk,
    input             reset,
    input             pause,
    input             run,            // 0 = SSIO CPU not fitted / suspended (Demolition Derby): held in reset
    input             ce_cpu,         // 2 MHz (sound Z80 and AY master clock)
    input             ce_8m,          // 8 MHz

    // the shared sound Z80 (pool slot z80_1) and the two shared AYs
    output chip_pool_pkg::z80r_i_t z80_1_i,
    input  chip_pool_pkg::z80r_o_t z80_1_o,
    output chip_pool_pkg::ay_i_t   [1:0] ay_i,
    input  chip_pool_pkg::ayio_o_t [1:0] ay_o,

    output     [13:0] rom2_a,
    input       [7:0] rom2_d,

    // volume PROM (82S123, MAME device midssio), loaded from ioctl
    input             prom_we,
    input       [4:0] prom_a,
    input       [7:0] prom_d,

    input      [31:0] cmd,            // command latches from the main CPU, latch n = [8n+7:8n]
    output reg  [7:0] status,
    input       [7:0] dip,

    output     [15:0] audio_l,
    output     [15:0] audio_r
);

    wire cpu_reset = reset | ~run;

    // ------------------------------------------------------------ 50 kHz period, /SINT
    reg  [7:0] per;                 // 0-159 at 8 MHz
    wire       per_wrap = ce_8m && (per == 8'd159);
    always @(posedge clk) begin
        if (reset)     per <= 8'd0;
        else if (ce_8m) per <= per_wrap ? 8'd0 : per + 8'd1;
    end

    reg  [6:0] sint_cnt;
    wire       irq_clr;
    always @(posedge clk) begin
        if (cpu_reset || irq_clr) sint_cnt <= 7'd0;
        else if (per_wrap && !pause) sint_cnt <= sint_cnt + 7'd1;
    end

    // ------------------------------------------------------------ Z80 bus
    wire [15:0] A    = z80_1_o.a;
    wire  [7:0] dout = z80_1_o.dout;
    wire        mreq = ~z80_1_o.mreq_n;
    wire        rd   = ~z80_1_o.rd_n;
    wire        wr   = ~z80_1_o.wr_n;
    wire        rfsh = ~z80_1_o.rfsh_n;
    wire        mem_rd = mreq && rd && !rfsh;
    wire        mem_wr = mreq && wr && !cpu_reset;

    wire [3:0] page = A[15:12];
    assign irq_clr = mem_rd && (page == 4'hE);

    always @(posedge clk) begin
        if (cpu_reset)                       status <= 8'h00;
        else if (mem_wr && page == 4'hC)     status <= dout;
    end

    wire [7:0] ram_q;
    dpram_dc #(.widthad_a(10)) u_ram
    (
        .clock_a(clk), .address_a(A[9:0]), .data_a(dout), .wren_a(mem_wr && page == 4'h8), .q_a(ram_q),
        .clock_b(clk), .address_b(10'd0), .data_b(8'h00), .wren_b(1'b0), .q_b()
    );
    assign rom2_a = A[13:0];

    // AYs: A[1:0] = 0 address write, 1 data read, 2 data write
    wire ay0_cs = mreq && !rfsh && (page == 4'hA);
    wire ay1_cs = mreq && !rfsh && (page == 4'hB);
    wire ay_bdir = wr && (A[1:0] == 2'd0 || A[1:0] == 2'd2);
    wire ay_bc1  = (wr && A[1:0] == 2'd0) || (rd && A[1:0] == 2'd1);
    assign ay_i[0].ce = ce_cpu;  assign ay_i[0].bdir = ay0_cs && ay_bdir; assign ay_i[0].bc1 = ay0_cs && ay_bc1;
    assign ay_i[1].ce = ce_cpu;  assign ay_i[1].bdir = ay1_cs && ay_bdir; assign ay_i[1].bc1 = ay1_cs && ay_bc1;
    assign ay_i[0].din = dout;   assign ay_i[0].ioa_in = 8'hFF; assign ay_i[0].iob_in = 8'hFF;
    assign ay_i[1].din = dout;   assign ay_i[1].ioa_in = 8'hFF; assign ay_i[1].iob_in = 8'hFF;

    reg [7:0] di;
    always @(*) begin
        case (page)
            4'h0, 4'h1, 4'h2, 4'h3: di = rom2_d;
            4'h8:                   di = ram_q;
            4'h9:                   di = cmd[{A[1:0], 3'b000} +: 8];
            4'hA:                   di = ay_o[0].dout;
            4'hB:                   di = ay_o[1].dout;
            4'hF:                   di = dip;
            default:                di = 8'hFF;
        endcase
    end

    assign z80_1_i.reset_n = ~cpu_reset;
    assign z80_1_i.ce      = ce_cpu & ~pause;
    assign z80_1_i.int_n   = ~sint_cnt[6];
    assign z80_1_i.nmi_n   = 1'b1;
    assign z80_1_i.di      = di;

    // ------------------------------------------------------------ volume circuit
    reg [7:0] prom [32];
    always @(posedge clk) if (prom_we) prom[prom_a] <= prom_d;
    wire [7:0] pbyte = prom[per[7:3]];
    wire       pbit  = pbyte[3'd7 - per[2:0]];

    wire [3:0] vol_in [6];
    assign vol_in[0] = ay_o[0].ioa_out[3:0]; assign vol_in[1] = ay_o[0].ioa_out[7:4]; assign vol_in[2] = ay_o[0].iob_out[3:0];
    assign vol_in[3] = ay_o[1].ioa_out[3:0]; assign vol_in[4] = ay_o[1].ioa_out[7:4]; assign vol_in[5] = ay_o[1].iob_out[3:0];
    wire [7:0] ch_in [6];
    assign ch_in[0] = ay_o[0].a; assign ch_in[1] = ay_o[0].b; assign ch_in[2] = ay_o[0].c;
    assign ch_in[3] = ay_o[1].a; assign ch_in[4] = ay_o[1].b; assign ch_in[5] = ay_o[1].c;

    // gated 2-pole low-pass per channel, state-variable form at 8 MHz: k = 1/128 (~10 kHz), Q = 1
    reg  [3:0]         vcnt [6];
    reg  signed [19:0] lp   [6];
    reg  signed [19:0] bp   [6];
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 6; i = i + 1) begin vcnt[i] <= 4'd0; lp[i] <= 20'sd0; bp[i] <= 20'sd0; end
        end else if (ce_8m) begin
            for (i = 0; i < 6; i = i + 1) begin
                if (per == 8'd0)
                    vcnt[i] <= vol_in[i];
                else if (!pbit && vcnt[i] != 4'd0)
                    vcnt[i] <= vcnt[i] - 4'd1;
                if (vcnt[i] == 4'd0) begin : track         // 4016 closed: the filter follows its input
                    reg signed [19:0] x;
                    x = {4'd0, ch_in[i], 8'd0};
                    lp[i] <= lp[i] + (bp[i] >>> 7);
                    bp[i] <= bp[i] + ((x - lp[i] - bp[i]) >>> 7);
                end
            end
        end
    end

    // ------------------------------------------------------------ mix, DC block, mute
    // mute ramp: 0.33 s from full to 0 in 256 steps (one step per ~65 x 20 us periods)
    reg [7:0] mgain;
    reg [6:0] mdiv;
    wire      mute = ay_o[1].iob_out[7];
    always @(posedge clk) begin
        if (reset) begin
            mgain <= 8'd255; mdiv <= 7'd0;
        end else if (per_wrap) begin
            mdiv <= (mdiv == 7'd64) ? 7'd0 : mdiv + 7'd1;
            if (mdiv == 7'd64) begin
                if (mute && mgain != 8'd0)    mgain <= mgain - 8'd1;
                if (!mute && mgain != 8'd255) mgain <= mgain + 8'd1;
            end
        end
    end

    function automatic signed [21:0] sx22(input signed [19:0] v); sx22 = {{2{v[19]}}, v}; endfunction
    wire signed [21:0] sum_l = sx22(lp[0]) + sx22(lp[1]) + sx22(lp[2]);
    wire signed [21:0] sum_r = sx22(lp[3]) + sx22(lp[4]) + sx22(lp[5]);

    // DC blocker at 50 kHz: y = x - avg, avg += (x - avg) / 1024 (~8 Hz)
    reg signed [21:0] avg_l, avg_r;
    reg signed [15:0] out_l, out_r;
    always @(posedge clk) begin
        if (reset) begin
            avg_l <= 22'sd0; avg_r <= 22'sd0; out_l <= 16'sd0; out_r <= 16'sd0;
        end else if (per_wrap) begin : mix
            reg signed [21:0] dl, dr;
            reg signed [30:0] gl, gr;
            avg_l <= avg_l + ((sum_l - avg_l) >>> 10);
            avg_r <= avg_r + ((sum_r - avg_r) >>> 10);
            dl = sum_l - avg_l;
            dr = sum_r - avg_r;
            gl = dl * $signed({1'b0, mgain});
            gr = dr * $signed({1'b0, mgain});
            out_l <= gl[25:10];                         // |dl| <= ~98K (three full-scale channels) x 255 < 2^25
            out_r <= gr[25:10];
        end
    end
    assign audio_l = out_l;
    assign audio_r = out_r;

endmodule
