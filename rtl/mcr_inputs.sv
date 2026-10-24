//============================================================================
//
//  MCR analog controls and per-game input handlers (MAME bally/mcr.cpp INPUT_PORTS + init_xxx custom handlers)
//  Copyright (C) 2026 Rodimus
//
//  The board ports arrive as bytes built from the MRA input map (digital controls, DIPs, analog rest values).
//  Here the analog fields are filled in (cfg bytes 153-163, tools/mra_cfg.py) and MAME's custom handlers applied
//  per game I/O id (cfg byte 151):
//    1 Solar Fox: IP0 / IP1 player mux on output port 0 D6        2 Kick: 4-bit dial, cocktail dial in IP1 high nibble
//    3 Draw Poker: coin-in release / hopper / coin-drop switches   4 Wacko: trackball mux on output port 4 D0
//    6 Kozmik Kroozr: dial bits 7-4 in IP1 bits 6, 2-0            9 Demolition Derby: 4 wheels, mux on output port 4 D6 / D7
//  Dials and trackballs are MAME IPT_DIAL counters (mcr_dial); analog sticks (IPT_AD_STICK) are absolute.
//
//============================================================================

// MAME IPT_DIAL / IPT_TRACKBALL axis: a free-running 8-bit counter (from Pole Position's pp_steering.sv).
// Units (MAME ioport.cpp analog_field, sensitivity s = sens * 100 / 128 %):
//   mouse: s / 100 counts per count; spinner: twice the mouse; held key: keydelta * s / 100 counts per frame;
//   analog stick, rate: 16 * s / 100 counts per frame at full deflection (MAME's absolute-on-relative);
//   analog stick / paddle, wheel: the counter tracks the device, 1 * s / 100 counts per unit (+-128 units).
// OSD sensitivity / direction as Pole Position (1.0 .75 .5 .25 .125 1.25 1.5 2.0 x).
module mcr_dial
(
    input             clk,
    input             reset,
    input             frame,          // once per MAME frame (30 Hz)
    input       [7:0] sens,           // MAME sensitivity * 128 / 100
    input       [7:0] keyd,           // MAME key delta
    input             reverse,        // MAME PORT_REVERSE ^ OSD direction
    input       [2:0] osd_sens,
    input             wheel,          // OSD: analog stick is a wheel (absolute) instead of a rate

    input       [8:0] spinner,        // hps_io spinner: [8] toggles per event, [7:0] signed delta
    input             mouse_ev,
    input       [8:0] mouse_d,        // signed
    input       [7:0] stick,          // signed
    input       [7:0] paddle,         // 0-255, absolute (an idle paddle adds nothing)
    input             use_paddle,
    input             dec,
    input             inc,

    output      [7:0] pos
);

    function automatic signed [27:0] osd_scale(input signed [27:0] v, input [2:0] s);   // * {8,6,4,2,1,10,12,16} / 8
        case (s)
            3'd0:    osd_scale = v;
            3'd1:    osd_scale = (v >>> 1) + (v >>> 2);
            3'd2:    osd_scale = v >>> 1;
            3'd3:    osd_scale = v >>> 2;
            3'd4:    osd_scale = v >>> 3;
            3'd5:    osd_scale = v + (v >>> 2);
            3'd6:    osd_scale = v + (v >>> 1);
            default: osd_scale = v <<< 1;
        endcase
    endfunction

    reg               spin_seen = 1'b0;
    reg signed [8:0]  stk_last = 9'sd0, pad_last = 9'sd0;
    reg signed [23:0] posq = 24'sd0;                       // position in 1/256 counts

    wire signed [9:0]  spin_d = (spinner[8] != spin_seen) ? $signed({{2{spinner[7]}}, spinner[7:0]}) : 10'sd0;
    wire signed [9:0]  mous_d = mouse_ev ? $signed({mouse_d[8], mouse_d}) : 10'sd0;
    wire signed [8:0]  sx     = $signed({stick[7], stick});
    wire               live   = (sx > 9'sd12) || (sx < -9'sd12);
    wire signed [9:0]  key_v  = (inc && !dec) ? $signed({2'b00, keyd}) : (dec && !inc) ? -$signed({2'b00, keyd}) : 10'sd0;

    // wheel / paddle: absolute positions (one unit = one mouse count), tracked as deltas
    wire signed [8:0]  pad_t  = use_paddle ? ($signed({1'b0, paddle}) - 9'sd128) : 9'sd0;
    wire signed [8:0]  stk_t  = (wheel && live) ? sx : 9'sd0;

    // every source in 1/8 mouse counts, then ONE multiply by the sensitivity: 1/256 counts = raw8 * sens / 4
    // (mouse: sens / 128 count per count; spinner 2x; stick rate 16 * s/100 counts per frame = stick / 8 per frame;
    //  key: keydelta * s/100 counts per frame)
    wire signed [17:0] ev8    = 18'sd8 * ($signed({{8{spin_d[9]}}, spin_d}) * 18'sd2 + {{8{mous_d[9]}}, mous_d} +
                                          {{9{stk_t[8]}}, stk_t} - {{9{stk_last[8]}}, stk_last} +
                                          {{9{pad_t[8]}}, pad_t} - {{9{pad_last[8]}}, pad_last});
    wire signed [17:0] fr8    = (!wheel && live) ? {{9{sx[8]}}, sx} : live ? 18'sd0 : 18'sd8 * {{8{key_v[9]}}, key_v};
    wire signed [17:0] raw8   = ev8 + (frame ? fr8 : 18'sd0);
    wire signed [27:0] d_q    = osd_scale(28'(raw8 * $signed({2'b00, sens})) >>> 2, osd_sens);

    always @(posedge clk) begin
        spin_seen  <= spinner[8];
        stk_last   <= stk_t;
        pad_last   <= pad_t;
        if (reset) posq <= 24'sd0;
        else       posq <= posq + (reverse ? -d_q[23:0] : d_q[23:0]);
    end

    assign pos = posq[15:8];

endmodule


module mcr_inputs
(
    input             clk,
    input             reset,
    input             frame,          // once per MAME frame
    input             fld_tick,       // once per field (60 Hz): Draw Poker timers

    input       [7:0] io_id,          // cfg byte 151
    input       [7:0] an_desc [3],    // cfg bytes 153-155: slots IP1, IP2, IP4
    input       [7:0] an_min,         // cfg byte 156
    input       [7:0] an_max,         // cfg byte 157
    input       [7:0] an_sens [3],    // cfg bytes 158-160
    input       [7:0] an_keyd [3],    // cfg bytes 161-163

    input       [2:0] osd_sens,
    input             osd_rev,
    input             osd_wheel,

    input       [8:0] spinner [4],
    input       [7:0] paddle [4],
    input      [15:0] lstick [4],     // {Y, X} signed
    input      [15:0] rstick [4],
    input      [24:0] mouse,          // hps_io ps2_mouse
    input       [3:0] dpad [4],       // {U, D, L, R}

    input       [7:0] op0,
    input       [7:0] op4,
    input       [7:0] dp_out,         // Draw Poker output port 34

    input       [7:0] ip0_i, ip1_i, ip2_i, ip4_i,
    output reg  [7:0] ip0, ip1, ip2, ip4
);

    // ------------------------------------------------------------ axis parameters: the slot that names the axis
    wire [2:0] t0 = an_desc[0][2:0], t1 = an_desc[1][2:0], t2 = an_desc[2][2:0];
    wire [1:0] xs = (t0 == 3'd1 || t0 == 3'd2) ? 2'd0 : (t1 == 3'd1 || t1 == 3'd2) ? 2'd1 : 2'd2;
    wire [1:0] ys = (t0 == 3'd3) ? 2'd0 : (t1 == 3'd3) ? 2'd1 : 2'd2;
    wire       steer = an_desc[xs][7];                       // dial / trackball on the right stick

    // ------------------------------------------------------------ mouse
    reg  mouse_seen = 1'b0;
    always @(posedge clk) mouse_seen <= mouse[24];
    wire       mouse_ev = (mouse[24] != mouse_seen);
    wire [8:0] mouse_x  = {mouse[4], mouse[15:8]};
    wire [8:0] mouse_y  = -{mouse[5], mouse[23:16]};          // PS/2 Y is up-positive; MAME mouse Y is down-positive

    // ------------------------------------------------------------ dial / trackball X (P1-P4), trackball Y (P1-P2)
    wire [7:0] dx [4];
    wire [7:0] dy [2];
    genvar p;
    generate
        for (p = 0; p < 4; p = p + 1) begin : g_x
            wire [15:0] st = steer ? rstick[p] : lstick[p];
            mcr_dial ax
            (
                .clk(clk), .reset(reset), .frame(frame), .sens(an_sens[xs]), .keyd(an_keyd[xs]),
                .reverse(an_desc[xs][4] ^ osd_rev), .osd_sens(osd_sens), .wheel(osd_wheel),
                .spinner(spinner[p]), .mouse_ev(mouse_ev && p == 0), .mouse_d(mouse_x), .stick(st[7:0]),
                .paddle(paddle[p]), .use_paddle(!steer),
                .dec(!steer && dpad[p][1]), .inc(!steer && dpad[p][0]), .pos(dx[p])
            );
        end
        for (p = 0; p < 2; p = p + 1) begin : g_y
            wire [15:0] st = steer ? rstick[p] : lstick[p];
            mcr_dial ay
            (
                .clk(clk), .reset(reset), .frame(frame), .sens(an_sens[ys]), .keyd(an_keyd[ys]),
                .reverse(an_desc[ys][4] ^ osd_rev), .osd_sens(osd_sens), .wheel(osd_wheel),
                .spinner(9'd0), .mouse_ev(mouse_ev && p == 0), .mouse_d(mouse_y), .stick(st[15:8]),
                .paddle(8'h80), .use_paddle(1'b0),
                .dec(!steer && dpad[p][3]), .inc(!steer && dpad[p][2]), .pos(dy[p])
            );
        end
    endgenerate

    // ------------------------------------------------------------ analog stick (IPT_AD_STICK, absolute), P1-P2
    // value = rest + stick * (max - rest) / 128 or stick * (rest - min) / 128; a held D-pad moves key delta per frame
    // towards the end, and back to the rest value when released (MAME autocentre)
    function automatic [7:0] ad_val(input [7:0] rest, input signed [8:0] off, input [7:0] lo, input [7:0] hi);
        reg signed [17:0] v;
        begin
            v = (off * $signed({10'd0, (off >= 0) ? hi - rest : rest - lo})) >>> 7;   // one multiply per axis
            v = $signed({10'd0, rest}) + v;
            ad_val = (v < $signed({10'd0, lo})) ? lo : (v > $signed({10'd0, hi})) ? hi : v[7:0];
        end
    endfunction

    reg signed [8:0] key_off [2][2];                         // [player][axis] D-pad offset, +-128 = full
    wire [7:0] rest_x [2], rest_y [2];
    wire [7:0] adx [2], ady [2];
    wire [1:0] sxs = (t0 == 3'd4) ? 2'd0 : (t1 == 3'd4) ? 2'd1 : 2'd2;
    wire [1:0] sys = (t0 == 3'd5) ? 2'd0 : (t1 == 3'd5) ? 2'd1 : 2'd2;
    wire [7:0] kx = an_keyd[sxs], ky = an_keyd[sys];
    wire       live_x [2], live_y [2];
    generate
        for (p = 0; p < 2; p = p + 1) begin : g_ad
            wire signed [8:0] sx = $signed({lstick[p][7], lstick[p][7:0]});
            wire signed [8:0] sy = $signed({lstick[p][15], lstick[p][15:8]});
            wire lx = (sx > 9'sd4) || (sx < -9'sd4), ly = (sy > 9'sd4) || (sy < -9'sd4);
            assign live_x[p] = lx; assign live_y[p] = ly;
            assign adx[p] = ad_val(rest_x[p], lx ? sx : key_off[p][0], an_min, an_max);
            assign ady[p] = ad_val(rest_y[p], ly ? sy : key_off[p][1], an_min, an_max);
        end
    endgenerate

    function automatic signed [8:0] ramp(input signed [8:0] v, input up, input dn, input [7:0] k);
        reg signed [10:0] n, w, kk;
        begin
            w  = {{2{v[8]}}, v};
            kk = $signed({3'd0, k});
            n  = (up && !dn) ? w + kk : (dn && !up) ? w - kk : (w > kk) ? w - kk : (w < -kk) ? w + kk : 11'sd0;
            ramp = (n > 11'sd128) ? 9'sd128 : (n < -11'sd128) ? -9'sd128 : n[8:0];
        end
    endfunction

    always @(posedge clk) begin
        if (reset) begin
            key_off[0][0] <= 9'sd0; key_off[0][1] <= 9'sd0; key_off[1][0] <= 9'sd0; key_off[1][1] <= 9'sd0;
        end else if (frame) begin
            // MiSTer also turns the left stick into D-pad bits: the stick wins while it is deflected
            key_off[0][0] <= live_x[0] ? 9'sd0 : ramp(key_off[0][0], dpad[0][0], dpad[0][1], kx);
            key_off[0][1] <= live_y[0] ? 9'sd0 : ramp(key_off[0][1], dpad[0][2], dpad[0][3], ky);
            key_off[1][0] <= live_x[1] ? 9'sd0 : ramp(key_off[1][0], dpad[1][0], dpad[1][1], kx);
            key_off[1][1] <= live_y[1] ? 9'sd0 : ramp(key_off[1][1], dpad[1][2], dpad[1][3], ky);
        end
    end

    // ------------------------------------------------------------ slot values
    // rest value of each stick axis = the slot that carries it
    function automatic [7:0] rest_of(input [2:0] typ, input pl);
        rest_of = (t0 == typ && an_desc[0][3] == pl) ? ip1_i : (t1 == typ && an_desc[1][3] == pl) ? ip2_i :
                  (t2 == typ && an_desc[2][3] == pl) ? ip4_i : 8'h80;
    endfunction
    assign rest_x[0] = rest_of(3'd4, 1'b0); assign rest_x[1] = rest_of(3'd4, 1'b1);
    assign rest_y[0] = rest_of(3'd5, 1'b0); assign rest_y[1] = rest_of(3'd5, 1'b1);

    wire wacko_mux = (io_id == 8'd4) && op4[0];               // Wacko: cocktail trackball

    function automatic [7:0] axis(input [7:0] d);
        reg pl;
        begin
            pl = d[3] | wacko_mux;
            case (d[2:0])
                3'd1, 3'd2: axis = dx[{1'b0, pl}];
                3'd3:       axis = dy[pl];
                3'd4:       axis = adx[pl];
                3'd5:       axis = ady[pl];
                default:    axis = 8'h00;
            endcase
        end
    endfunction

    function automatic [7:0] merge(input [7:0] port, input [7:0] d);
        reg [7:0] m;
        begin
            m = 8'hFF >> d[6:5];
            merge = (d[2:0] == 3'd0) ? port : ((port & ~m) | (axis(d) & m));
        end
    endfunction

    // ------------------------------------------------------------ Draw Poker coin mechanism (MAME mcr_dpoker_state)
    // coin-in: the release switch reads closed for 100 ms once the coin leaves the hit switch (MAME coin_in_hit runs
    // on newval = 1, the active-low bit going high); hopper: 500 ms after output 34 D6 rises,
    // the coin-out switch toggles (100 ms closed, 250 ms open) while D6 stays set
    reg       coin_d = 1'b1, cs1 = 1'b0, cs3 = 1'b0, hop_d = 1'b0;
    reg [4:0] coin_t = 5'd0, hop_t = 5'd0;
    always @(posedge clk) begin
        if (reset) begin
            cs1 <= 1'b0; cs3 <= 1'b0; coin_t <= 5'd0; hop_t <= 5'd0; coin_d <= 1'b1; hop_d <= 1'b0;
        end else begin
            coin_d <= ip0_i[0];
            hop_d  <= dp_out[6];
            if (!coin_d && ip0_i[0]) begin cs1 <= 1'b1; coin_t <= 5'd6; end
            else if (fld_tick && coin_t != 5'd0) begin
                coin_t <= coin_t - 5'd1;
                if (coin_t == 5'd1) cs1 <= 1'b0;
            end
            if (dp_out[6] && !hop_d) hop_t <= 5'd30;
            else if (!dp_out[6]) begin cs3 <= 1'b0; hop_t <= 5'd0; end
            else if (fld_tick && hop_t != 5'd0) begin
                hop_t <= hop_t - 5'd1;
                if (hop_t == 5'd1) begin cs3 <= ~cs3; hop_t <= cs3 ? 5'd15 : 5'd6; end
            end
        end
    end

    // ------------------------------------------------------------ Demolition Derby wheel mux
    reg dd_mux = 1'b0;
    always @(posedge clk) begin
        if (reset)        dd_mux <= 1'b0;
        else if (op4[7])  dd_mux <= 1'b0;
        else if (op4[6])  dd_mux <= 1'b1;
    end

    // ------------------------------------------------------------ ports
    wire [7:0] kd = axis(an_desc[0]);                         // Kroozr / Kick slot 0 dial
    always @(*) begin
        ip0 = ip0_i;
        ip1 = merge(ip1_i, an_desc[0]);
        ip2 = merge(ip2_i, an_desc[1]);
        ip4 = merge(ip4_i, an_desc[2]);
        case (io_id)
            8'd1: begin                                       // Solar Fox (MAME solarfox_ip0_r / ip1_r)
                if (op0[6]) begin
                    ip0 = ip0_i | 8'h08;
                    ip1 = ip1_i | 8'hF0;
                end else begin
                    ip0 = (ip0_i & ~8'h1C) | 8'h08 | ((ip0_i & 8'h08) >> 1) | ((ip2_i & 8'h01) << 4);
                    ip1 = (ip1_i >> 4) | 8'hF0;
                end
            end
            8'd2: begin                                       // Kick: dial low nibble, cocktail dial high nibble
                ip1 = {an_desc[1][2:0] != 3'd0 ? dx[1][3:0] : 4'hF, kd[3:0]};
                ip2 = ip2_i;
            end
            8'd3: ip0 = {~ip0_i[6], ip0_i[6:4], ~cs3, 1'b1, ~cs1, ip0_i[0]};   // Draw Poker
            8'd6: ip1 = (ip1_i & ~8'h47) | {1'b0, kd[7], 3'b000, kd[6:4]};     // Kozmik Kroozr
            8'd9: begin                                       // Demolition Derby: wheel (inverted) in bits 7-2
                ip1 = {~(dd_mux ? dx[2][5:0] : dx[0][5:0]), ip1_i[1:0]};
                ip2 = {~(dd_mux ? dx[3][5:0] : dx[1][5:0]), ip2_i[1:0]};
            end
            default: ;
        endcase
    end

endmodule
