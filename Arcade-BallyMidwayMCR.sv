//============================================================================
//
//  Bally Midway MCR I / II / III arcade hardware for MiSTer
//  Copyright (C) 2026 Rodimus
//
//  Top level carried over from the Rodimus Gottlieb / Williams / Exidy cores (Pac-Man core lineage, MikeJ / Sorgelig)
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//
//============================================================================

module emu
(
    `include "sys/emu_ports.vh"
);

wire        CLK_SYS;
wire        locked;
wire [127:0] status;
wire  [1:0] buttons;
wire        forced_scandoubler;
wire [10:0] ps2_key;
wire        ioctl_download;
wire        ioctl_upload;
wire        ioctl_upload_req;
wire  [7:0] ioctl_din;
wire        ioctl_wr;
wire  [7:0] ioctl_index;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] joystick_0, joystick_1, joystick_2, joystick_3;
wire [21:0] gamma_bus;
wire        direct_video;
wire        video_rotated;
wire        pause_cpu;
wire        hblank, vblank;
wire        video_field;

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

assign VGA_F1 = video_field;   // native 480i: the field tells the scaler / analog path which lines this is
assign VGA_SCALER = 0;
assign VGA_DISABLE = 0;
assign FB_FORCE_BLANK = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

wire [15:0] audio_l, audio_r;
assign AUDIO_L = pause_cpu ? 16'd0 : audio_l;
assign AUDIO_R = pause_cpu ? 16'd0 : audio_r;
assign AUDIO_S = 1;   // signed
assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign LED_USER  = ioctl_download;
assign BUTTONS = 0;

///////////////////////////////////////////////////

// MRA index 1 (layout in tools/mra_cfg.py; first 148 bytes as the Gottlieb / Exidy cores):
//   byte 0      game id (informational)
//   byte 1      flags: [0] 4-way joystick, [1] mirror X, [2] mirror Y, [4] vertical, [7] vertical is ROT90 (cw)
//   bytes 4-11  idle level of the 8 ports (IP0, IP1, IP2, IP3, IP4, SSIO DIP, P24, P2C)
//   bytes 12-19 per port: DIP switch byte that supplies it (bit 7 set = none, use the idle byte)
//   bytes 20-83 input map, one byte per port bit (bit 0 first): control id, 0 = none; a pressed control inverts its bit
//   bytes 84-147 per port bit: which bit of its DIP switch byte supplies it
//   bytes 148-163 board config: CPU board, sprite board, sound, game I/O, sprite offsets, analog x3, stick range,
//               analog sensitivity x3, key delta x3
reg [7:0] game_var    = 8'd0;
reg [7:0] game_flags  = 8'h00;
reg [7:0] cfg[16]     = '{8'd1,8'd0,8'd0,8'd0,8'd0,8'd0,8'd0,8'd0,8'd0,8'd0,8'd0,8'd0,8'd0,8'd0,8'd0,8'd0};
reg [5:0] in_map[64];
reg [7:0] port_idle[8] = '{8'hFF,8'hFF,8'hFF,8'hFF,8'hFF,8'hFF,8'hFF,8'hFF};
reg [7:0] port_dip[8]  = '{8'h80,8'h80,8'h80,8'h80,8'h80,8'h80,8'h80,8'h80};
wire [7:0] map_idx8  = ioctl_addr[7:0] - 8'd20;    // bytes 20-83  -> 0-63
wire [7:0] perm_idx8 = ioctl_addr[7:0] - 8'd84;    // bytes 84-147 -> 0-63
wire [7:0] cfg_idx8  = ioctl_addr[7:0] - 8'd148;   // bytes 148-163 -> 0-15
wire [5:0] map_idx   = map_idx8[5:0];
wire [5:0] perm_idx  = perm_idx8[5:0];
wire [2:0] idle_idx  = ioctl_addr[2:0] - 3'd4;     // bytes 4-11  -> 0-7
wire [2:0] dip_idx   = ioctl_addr[2:0] - 3'd4;     // bytes 12-19 -> 0-7
reg  [2:0] dip_bit[64];
initial for (int i = 0; i < 64; i++) dip_bit[i] = i % 8;
initial for (int i = 0; i < 64; i++) in_map[i] = 6'd0;

always @(posedge CLK_SYS) begin
    if (ioctl_wr && ioctl_index == 8'd1) begin
        if (ioctl_addr >= 25'd84 && ioctl_addr < 25'd148) dip_bit[perm_idx] <= ioctl_dout[2:0];
        if (ioctl_addr == 25'd0) game_var   <= ioctl_dout;
        if (ioctl_addr == 25'd1) game_flags <= ioctl_dout;
        if (ioctl_addr >= 25'd4  && ioctl_addr < 25'd12) port_idle[idle_idx] <= ioctl_dout;
        if (ioctl_addr >= 25'd12 && ioctl_addr < 25'd20) port_dip[dip_idx]   <= ioctl_dout;
        if (ioctl_addr >= 25'd20 && ioctl_addr < 25'd84) in_map[map_idx] <= ioctl_dout[5:0];
        if (ioctl_addr >= 25'd148 && ioctl_addr < 25'd164) cfg[cfg_idx8[3:0]] <= ioctl_dout;
    end
end

wire game_vert = game_flags[4];
wire vert_view = game_vert & ~status[12];

wire [1:0] ar = status[9:8];

assign VIDEO_ARX = (!ar) ? (vert_view ? 12'd3 : 12'd4) : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? (vert_view ? 12'd4 : 12'd3) : 12'd0;

// Status bit layout kept the same as the Gottlieb core
`include "build_id.v"
localparam CONF_STR = {
	"MCR;;",
	"P1,Video Options;",
	"P1O89,Aspect Ratio,Original,Full screen,[ARC1],[ARC2];",
	"P1OB,HDMI Flip,Off,On;",
	"P1OM,CRT Flip,Off,On;",
	"P1O[40:37],H Position (CRT),0,+1,+2,+3,+4,+5,+6,+7,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[44:41],V Position (CRT),0,+1,+2,+3,+4,+5,+6,+7,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1OGI,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"-;",
	"H1P2,Game Options;",
	"H1P2O[47:45],Dial Sensitivity,1.0x,0.75x,0.5x,0.25x,0.125x,1.25x,1.5x,2.0x;",
	"H1P2O[48],Dial Direction,Normal,Reversed;",
	"H1P2O[49],Analog Stick as Dial,Rate,Wheel;",
	"-;",
	"P3,Pause Options;",
	"P3OJ,Pause when OSD is open,On,Off;",
	"P3OK,Dim video after 10s,On,Off;",
	"-;",
	"OR,Autosave NVRAM,On,Off;",
	"-;",
	"DIP;",
	"-;",
	"R0,Reset;",
	"J1,Btn 1,Btn 2,Btn 3,Btn 4,Coin,Start 1P,Start 2P,Pause,Btn 5,Btn 6,Btn 7,Btn 8;",
	"jn,A,B,X,Y,Select,Start,R,L;",
	"V,v",`BUILD_DATE
};

// analog devices per player (MCR dials, trackballs, sticks, wheels: rtl/mcr_inputs.sv)
wire [15:0] lstick [4], rstick [4];
wire  [7:0] paddle [4];
wire  [8:0] spinner [4];
wire [24:0] ps2_mouse;
wire        no_analog = (cfg[5] == 8'd0) && (cfg[6] == 8'd0) && (cfg[7] == 8'd0);   // hides Game Options

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(CLK_SYS),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),
	.direct_video(direct_video),
	.video_rotated(video_rotated),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({no_analog, direct_video}),

	.ioctl_download(ioctl_download),
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_din(ioctl_din),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(1'b0),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_2(joystick_2),
	.joystick_3(joystick_3),
	.joystick_l_analog_0(lstick[0]), .joystick_l_analog_1(lstick[1]),
	.joystick_l_analog_2(lstick[2]), .joystick_l_analog_3(lstick[3]),
	.joystick_r_analog_0(rstick[0]), .joystick_r_analog_1(rstick[1]),
	.joystick_r_analog_2(rstick[2]), .joystick_r_analog_3(rstick[3]),
	.paddle_0(paddle[0]), .paddle_1(paddle[1]), .paddle_2(paddle[2]), .paddle_3(paddle[3]),
	.spinner_0(spinner[0]), .spinner_1(spinner[1]), .spinner_2(spinner[2]), .spinner_3(spinner[3]),
	.ps2_mouse(ps2_mouse),
	.ps2_key(ps2_key)
);

////////////////////   CLOCKS   ///////////////////

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(CLK_SYS),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll),
	.locked(locked)
);

wire [63:0] reconfig_to_pll;
wire [63:0] reconfig_from_pll;
wire        cfg_waitrequest;

pll_cfg pll_cfg
(
	.mgmt_clk(CLK_50M),
	.mgmt_reset(0),
	.mgmt_waitrequest(cfg_waitrequest),
	.mgmt_read(0),
	.mgmt_readdata(),
	.mgmt_write(0),
	.mgmt_address(0),
	.mgmt_writedata(0),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

// Hold the board in reset until the PLL is locked and the ROM download has finished
wire reset = RESET | status[0] | buttons[1] | ioctl_download | ~locked;

///////////////////         Keyboard           //////////////////

// CTL-2026-10-24: original block commented out below (replaced by rtl/mister_custom/controllers.sv modules after it)
// reg kb_up = 0, kb_down = 0, kb_left = 0, kb_right = 0;
// reg kb_b1 = 0, kb_b2 = 0;
// reg kb_coin1 = 0, kb_coin2 = 0, kb_start1 = 0, kb_start2 = 0, kb_pause = 0, kb_service = 0, kb_tilt = 0, kb_service1 = 0;
// reg kb_keyin = 0, kb_keyout = 0, kb_payout = 0, kb_door1o = 0, kb_door1l = 0, kb_door2o = 0, kb_door2l = 0;   // Draw Poker
//
// wire       pressed = ~ps2_key[9];
// wire [7:0] code    = ps2_key[7:0];
//
// always @(posedge CLK_SYS) begin
// 	reg old_state;
// 	old_state <= ps2_key[10];
// 	if (old_state != ps2_key[10]) begin
// 		case (code)
// 			'h16: kb_start1   <= pressed; // 1
// 			'h1E: kb_start2   <= pressed; // 2
// 			'h2E: kb_coin1    <= pressed; // 5
// 			'h36: kb_coin2    <= pressed; // 6
// 			'h46: kb_service  <= pressed; // 9 (test switch)
// 			'h06: kb_service  <= pressed; // F2
// 			'h3E: kb_service1 <= pressed; // 8 (service credit)
// 			'h2C: kb_tilt     <= pressed; // T
// 			'h4D: kb_pause    <= pressed; // P
// 			'h15: kb_keyin    <= pressed; // Q  (MAME defaults for Draw Poker's keys and doors)
// 			'h1D: kb_keyout   <= pressed; // W
// 			'h25: kb_payout   <= pressed; // 4
// 			'h1C: kb_door1o   <= pressed; // A
// 			'h1B: kb_door1l   <= pressed; // S
// 			'h23: kb_door2o   <= pressed; // D
// 			'h2B: kb_door2l   <= pressed; // F
//
// 			'h75: kb_up       <= pressed; // up
// 			'h72: kb_down     <= pressed; // down
// 			'h6B: kb_left     <= pressed; // left
// 			'h74: kb_right    <= pressed; // right
// 			'h14: kb_b1       <= pressed; // ctrl
// 			'h11: kb_b2       <= pressed; // alt
// 		endcase
// 	end
// end
// CTL2-2026-10-25: v1 keyboard + local extra-key block commented out below (now ctl_keyboard's extra-key table)
// // CTL-2026-10-24: keyboard via ctl_keyboard (keys act on press; the old ~ps2_key[9] acted on release).
// // kb: [0] up [1] down [2] left [3] right [4] ctrl [5] alt [6] 1 [7] 2 [8] 5 [9] 6 [10] 9 [11] T [12] P
// wire [12:0] kb;
// ctl_keyboard u_kb(.clk(CLK_SYS), .ps2_key(ps2_key), .kb(kb));
// wire kb_up    = kb[0];
// wire kb_down  = kb[1];
// wire kb_left  = kb[2];
// wire kb_right = kb[3];
// wire kb_pause = kb[12];
// // MCR's extra keys stay local (same on-press polarity): 9 and F2 share the test switch (the last key event wins,
// // as before), 8 service credit, Draw Poker's keys and doors (MAME defaults)
// reg kb_service = 0, kb_service1 = 0;
// reg kb_keyin = 0, kb_keyout = 0, kb_payout = 0, kb_door1o = 0, kb_door1l = 0, kb_door2o = 0, kb_door2l = 0;
// reg kbx_old_state = 1'b0;
// always @(posedge CLK_SYS) begin
// 	kbx_old_state <= ps2_key[10];
// 	if (kbx_old_state != ps2_key[10]) begin
// 		case (ps2_key[7:0])
// 			8'h46: kb_service  <= ps2_key[9]; // 9 (test switch)
// 			8'h06: kb_service  <= ps2_key[9]; // F2
// 			8'h3E: kb_service1 <= ps2_key[9]; // 8 (service credit)
// 			8'h15: kb_keyin    <= ps2_key[9]; // Q
// 			8'h1D: kb_keyout   <= ps2_key[9]; // W
// 			8'h25: kb_payout   <= ps2_key[9]; // 4
// 			8'h1C: kb_door1o   <= ps2_key[9]; // A
// 			8'h1B: kb_door1l   <= ps2_key[9]; // S
// 			8'h23: kb_door2o   <= ps2_key[9]; // D
// 			8'h2B: kb_door2l   <= ps2_key[9]; // F
// 			default: ;
// 		endcase
// 	end
// end
// controllers.sv ctl_keyboard (keys act on press): kb [0] up [1] down [2] left [3] right [4] ctrl [5] alt [6] 1 [7] 2
// [8] 5 [9] 6 [10] 9 [11] T [12] P; kbx [0] 9 / F2 test switch (last key event wins), [1] 8 service credit, Draw Poker
// (MAME defaults) [2] Q key in, [3] W key out, [4] 4 payout, [5] A door 1 open, [6] S door 1 lock, [7] D door 2 open,
// [8] F door 2 lock
wire [12:0] kb;
wire  [8:0] kbx;
ctl_keyboard #(.NX(10), .NXO(9), .XKEYS({24'h10_2B_08, 24'h10_23_07, 24'h10_1B_06, 24'h10_1C_05, 24'h10_25_04,
                                         24'h10_1D_03, 24'h10_15_02, 24'h10_3E_01, 24'h10_06_00, 24'h10_46_00})) u_kb
(
	.clk(CLK_SYS), .ps2_key(ps2_key), .pause(pause_cpu), .kb(kb), .kbx(kbx)
);
wire kb_up    = kb[0];
wire kb_down  = kb[1];
wire kb_left  = kb[2];
wire kb_right = kb[3];
wire kb_pause = kb[12];
wire kb_service = kbx[0], kb_service1 = kbx[1];
wire kb_keyin  = kbx[2], kb_keyout = kbx[3], kb_payout = kbx[4];
wire kb_door1o = kbx[5], kb_door1l = kbx[6], kb_door2o = kbx[7], kb_door2l = kbx[8];

//////////////////  Arcade Buttons/Interfaces   ///////////////////////////

// Joystick bits: 0 R, 1 L, 2 D, 3 U, 4-7 Btn 1-4, 8 Coin, 9 Start 1P, 10 Start 2P, 11 Pause, 12-15 Btn 5-8
wire [3:0] dir1_raw = {joystick_0[3] | kb_up, joystick_0[2] | kb_down, joystick_0[1] | kb_left, joystick_0[0] | kb_right};
wire [3:0] dir2_raw = joystick_1[3:0];
wire [3:0] dir1, dir2;   // {U, D, L, R}

// CTL-2026-10-24: original block commented out below (replaced by rtl/mister_custom/controllers.sv modules after it)
// joy4way joy4way_1(.clk(CLK_SYS), .en(game_flags[0]), .in(dir1_raw), .out(dir1));
// joy4way joy4way_2(.clk(CLK_SYS), .en(game_flags[0]), .in(dir2_raw), .out(dir2));
// CTL-2026-10-24: 4-way restriction via ctl_joy4way (MRA byte 1 bit 0)
ctl_joy4way u_joy4way_1(.clk(CLK_SYS), .en(game_flags[0]), .in(dir1_raw), .out(dir1));
ctl_joy4way u_joy4way_2(.clk(CLK_SYS), .en(game_flags[0]), .in(dir2_raw), .out(dir2));

wire m_pause = joystick_0[11] | kb_pause;

// CTL-2026-10-24: original block commented out below (replaced by rtl/mister_custom/controllers.sv modules after it)
// // right analog stick as 4 directions {U, D, L, R}: the dominant axis past half deflection (a 4-way stick)
// wire [3:0] rs_dir [2];
// genvar rsi;
// generate
// 	for (rsi = 0; rsi < 2; rsi = rsi + 1) begin : g_rs
// 		wire signed [7:0] x = rstick[rsi][7:0], y = rstick[rsi][15:8];
// 		wire [7:0] ax = x[7] ? -x : x, ay = y[7] ? -y : y;
// 		wire       h  = (ax >= ay), on = (h ? ax : ay) >= 8'd64;
// 		assign rs_dir[rsi] = {on && !h && y[7], on && !h && !y[7], on && h && x[7], on && h && !x[7]};
// 	end
// endgenerate
// CTL-2026-10-24: right analog stick as 4 directions {U, D, L, R} via ctl_stick_dirs MODE 1 (dominant axis >= 64)
wire [3:0] rs_dir [2];
ctl_stick_dirs #(.MODE(1), .THRESH(7'd64)) u_rs_dir0(.stick(rstick[0]), .dir(rs_dir[0]));
ctl_stick_dirs #(.MODE(1), .THRESH(7'd64)) u_rs_dir1(.stick(rstick[1]), .dir(rs_dir[1]));

// CTL-2026-10-24: original block commented out below (replaced by rtl/mister_custom/controllers.sv modules after it)
// // control ids used by the MRA input map (tools/mra_cfg.py; Gottlieb numbering, extended for MCR)
// wire [63:0] ctl =
// {
// 	4'd0,                                           // 63-60 reserved
// 	kb_door2l, kb_door2o, kb_door1l, kb_door1o,     // 59-56 Draw Poker doors (lock / open)
// 	kb_payout, kb_keyout, kb_keyin,                 // 55-53 Draw Poker payout, key out, key in
// 	joystick_0[15:14],                              // 52-51 P1 Btn 8-7
// 	joystick_3[5:4],                                // 50-49 P4 Btn 2-1
// 	joystick_2[5],                                  // 48 P3 Btn 2
// 	kb_service1,                                    // 47 service credit
// 	1'b0,                                           // 46 reserved
// 	joystick_3[9],                                  // 45 start 4
// 	joystick_3[8],                                  // 44 coin 4
// 	joystick_2[9],                                  // 43 start 3
// 	joystick_2[4],                                  // 42 P3 Btn 1
// 	joystick_2[0], joystick_2[1], joystick_2[2], joystick_2[3],   // 41 P3 R, 40 L, 39 D, 38 U
// 	2'd0,                                           // 37-36 reserved
// 	joystick_1[13:12],                              // 35-34 P2 Btn 6-5
// 	joystick_0[13:12],                              // 33-32 P1 Btn 6-5
// 	rs_dir[1][0], rs_dir[1][1], rs_dir[1][2], rs_dir[1][3],   // 31 P2 right stick R, 30 L, 29 D, 28 U
// 	rs_dir[0][0], rs_dir[0][1], rs_dir[0][2], rs_dir[0][3],   // 27 P1 right stick R, 26 L, 25 D, 24 U (Wacko fire)
// 	joystick_2[8],                                  // 23 coin 3
// 	kb_service,                                     // 22 service (test switch)
// 	kb_tilt,                                        // 21 tilt
// 	joystick_0[10] | joystick_1[10] | joystick_1[9] | kb_start2,   // 20 start 2 (pad 2's Start is Start 2P)
// 	joystick_0[9]  | kb_start1,                     // 19 start 1
// 	joystick_1[8]  | kb_coin2,                      // 18 coin 2
// 	joystick_0[8]  | kb_coin1,                      // 17 coin 1
// 	joystick_1[7:4],                                // 16-13 P2 Btn 4-1
// 	dir2[0], dir2[1], dir2[2], dir2[3],             // 12 R, 11 L, 10 D, 9 U
// 	joystick_0[7:6], joystick_0[5] | kb_b2, joystick_0[4] | kb_b1,   // 8-5 P1 Btn 4-1
// 	dir1[0], dir1[1], dir1[2], dir1[3],             // 4 R, 3 L, 2 D, 1 U
// 	1'b0                                            // 0 none
// };
// CTL-2026-10-24: control ids 0-23 and 32-35 via ctl_ids; id 22 keeps MCR's 9 / F2 test switch; ids 24-31 and 36+
// stay core-specific (tools/mra_cfg.py numbering)
/* verilator lint_off UNUSED */
wire [35:0] ids;                                    // ids[22] (9 only) replaced by kb_service (9 or F2)
/* verilator lint_on UNUSED */
ctl_ids u_ids(.joy0(joystick_0), .joy1(joystick_1), .kb(kb), .dir1(dir1), .dir2(dir2),
	.coin1(joystick_0[8] | kb[8]), .coin2(joystick_1[8] | kb[9]), .coin3(joystick_2[8]), .ctl(ids));
wire [63:0] ctl =
{
	4'd0,                                           // 63-60 reserved
	kb_door2l, kb_door2o, kb_door1l, kb_door1o,     // 59-56 Draw Poker doors (lock / open)
	kb_payout, kb_keyout, kb_keyin,                 // 55-53 Draw Poker payout, key out, key in
	joystick_0[15:14],                              // 52-51 P1 Btn 8-7
	joystick_3[5:4],                                // 50-49 P4 Btn 2-1
	joystick_2[5],                                  // 48 P3 Btn 2
	kb_service1,                                    // 47 service credit
	1'b0,                                           // 46 reserved
	joystick_3[9],                                  // 45 start 4
	joystick_3[8],                                  // 44 coin 4
	joystick_2[9],                                  // 43 start 3
	joystick_2[4],                                  // 42 P3 Btn 1
	joystick_2[0], joystick_2[1], joystick_2[2], joystick_2[3],   // 41 P3 R, 40 L, 39 D, 38 U
	2'd0,                                           // 37-36 reserved
	ids[35:32],                                     // 35-34 P2 Btn 6-5, 33-32 P1 Btn 6-5
	rs_dir[1][0], rs_dir[1][1], rs_dir[1][2], rs_dir[1][3],   // 31 P2 right stick R, 30 L, 29 D, 28 U
	rs_dir[0][0], rs_dir[0][1], rs_dir[0][2], rs_dir[0][3],   // 27 P1 right stick R, 26 L, 25 D, 24 U (Wacko fire)
	ids[23],                                        // 23 coin 3
	kb_service,                                     // 22 service (test switch: 9 or F2)
	ids[21:0]                                       // 21 tilt, 20-19 starts, 18-17 coins, 16-1 pads, 0 none
};

// DIP switches arrive from the OSD via ioctl index 254
reg [7:0] dip_sw[8] = '{8'hFF,8'hFF,8'hFF,8'hFF,8'h00,8'h00,8'h00,8'h00};
always @(posedge CLK_SYS) begin
	if (ioctl_wr && (ioctl_index == 8'd254) && !ioctl_addr[24:3])
		dip_sw[ioctl_addr[2:0]] <= ioctl_dout;
end

// CTL-2026-10-24: original block commented out below (replaced by rtl/mister_custom/controllers.sv modules after it)
// reg [7:0] in_port[8];
// always @(posedge CLK_SYS) begin
// 	for (int p = 0; p < 8; p++)
// 		for (int b = 0; b < 8; b++)
// 			in_port[p][b] <= (port_dip[p][7] ? port_idle[p][b] : dip_sw[port_dip[p][2:0]][dip_bit[p*8 + b]]) ^ ctl[in_map[p*8 + b]];
// end
// CTL2-2026-10-25: v1 port_base generate commented out below (now ctl_port_map BASE_MODE 2)
// // CTL-2026-10-24: input ports via ctl_port_map; each bit's base level is its idle bit, or the DIP switch bit the MRA
// // names (port_dip / dip_bit), and a pressed control inverts it
// wire [63:0]  port_base;
// wire [511:0] in_map_p;
// wire [63:0]  in_ports_p;
// genvar gp;
// generate
// 	for (gp = 0; gp < 64; gp = gp + 1) begin : g_port_bit
// 		assign port_base[gp] = port_dip[gp/8][7] ? port_idle[gp/8][gp%8] : dip_sw[port_dip[gp/8][2:0]][dip_bit[gp]];
// 		assign in_map_p[gp*8 +: 8] = {2'b00, in_map[gp]};
// 	end
// endgenerate
// ctl_port_map #(.NP(8), .IDW(6), .DIP_MODE(0)) u_ports(.clk(CLK_SYS), .idle(port_base), .in_map(in_map_p), .dips(32'd0),
// 	.ctl(ctl), .ovr_mask(64'd0), .ovr_val(64'd0), .ports(in_ports_p));
// wire [7:0] in_port[8];
// generate
// 	for (gp = 0; gp < 8; gp = gp + 1) begin : g_in_port
// 		assign in_port[gp] = in_ports_p[gp*8 +: 8];
// 	end
// endgenerate
// MRA input map -> board input ports (controllers.sv ctl_port_map, registered): port_dip[p][7] = 1 takes the idle byte,
// else DIP byte port_dip[p][2:0] bit dip_bit[]; a pressed control (id in_map[]) inverts the bit
wire  [7:0] in_port[8];
wire [63:0] pm_idle, pm_sel, pm_dips, pm_ports;
wire [511:0] pm_map;
wire [191:0] pm_dbit;
// CTL2-2026-10-25: Quartus 17.0 rejects `for (genvar ...)` in the loop header (error 10170) and genvar b clashed
// with wire b; genvars now declared at module scope inside generate. Original block:
// for (genvar p = 0; p < 8; p++) begin : g_pm
// 	assign pm_idle[p*8 +: 8] = port_idle[p];
// 	assign pm_sel[p*8 +: 8]  = port_dip[p];
// 	assign pm_dips[p*8 +: 8] = dip_sw[p];
// 	assign in_port[p]        = pm_ports[p*8 +: 8];
// 	for (genvar b = 0; b < 8; b++) begin : g_b
// 		assign pm_map[(p*8 + b)*8 +: 8]  = {2'b00, in_map[p*8 + b]};
// 		assign pm_dbit[(p*8 + b)*3 +: 3] = dip_bit[p*8 + b];
// 	end
// end
genvar pm_p, pm_b;
generate
	for (pm_p = 0; pm_p < 8; pm_p = pm_p + 1) begin : g_pm
		assign pm_idle[pm_p*8 +: 8] = port_idle[pm_p];
		assign pm_sel[pm_p*8 +: 8]  = port_dip[pm_p];
		assign pm_dips[pm_p*8 +: 8] = dip_sw[pm_p];
		assign in_port[pm_p]        = pm_ports[pm_p*8 +: 8];
		for (pm_b = 0; pm_b < 8; pm_b = pm_b + 1) begin : g_b
			assign pm_map[(pm_p*8 + pm_b)*8 +: 8]  = {2'b00, in_map[pm_p*8 + pm_b]};
			assign pm_dbit[(pm_p*8 + pm_b)*3 +: 3] = dip_bit[pm_p*8 + pm_b];
		end
	end
endgenerate
ctl_port_map #(.NP(8), .IDW(6), .BASE_MODE(2)) u_ports
(
	.clk(CLK_SYS), .idle(pm_idle), .in_map(pm_map), .dips(32'd0), .port_sel(pm_sel), .dip_bytes(pm_dips),
	.dip_bit(pm_dbit), .ctl(ctl), .ovr_mask(64'd0), .ovr_val(64'd0), .ports(pm_ports)
);

// PAUSE SYSTEM
wire [23:0] rgb_out;
pause #(8,8,8,80) pause
(
	.*,
	.clk_sys(CLK_SYS),
	.user_button(m_pause),
	.pause_request(1'b0),
	.options(~status[20:19])
);

///////////////                 Video                  ////////////////

wire hs, vs;
wire [7:0] r, g, b;
wire ce_pix;

wire rotate_ccw = ~game_flags[7];  // ROT270 sets rotate CCW, ROT90 sets CW
wire no_rotate  = ~game_vert | status[12] | direct_video;
wire flip       = status[11];
screen_rotate screen_rotate(.*);

arcade_video #(512,24) arcade_video
(
	.*,

	.clk_video(CLK_SYS),

	.RGB_in(rgb_out),
	.HBlank(hblank),
	.VBlank(vblank),
	.HSync(hs),
	.VSync(vs),

	.fx(status[18:16])
);

///////////////                 Board                  ////////////////

wire [7:0] op0;

mcr_board board
(
	.clk(CLK_SYS),
	.reset(reset),
	.pause(pause_cpu),

	.cfg_cpu(cfg[0]),
	.cfg_spr(cfg[1]),
	.cfg_snd(cfg[2]),
	.cfg_io(cfg[3]),
	.cfg_xoffs(cfg[4]),

	.ip0(in_port[0]),
	.ip1(in_port[1]),
	.ip2(in_port[2]),
	.ip3(in_port[3]),
	.ip4(in_port[4]),
	.ssio_dip(in_port[5]),
	.p24(in_port[6]),
	.p2c(in_port[7]),

	.an_desc('{cfg[5], cfg[6], cfg[7]}),
	.an_min(cfg[8]),
	.an_max(cfg[9]),
	.an_sens('{cfg[10], cfg[11], cfg[12]}),
	.an_keyd('{cfg[13], cfg[14], cfg[15]}),
	.osd_sens(status[47:45]),
	.osd_rev(status[48]),
	.osd_wheel(status[49]),
	.spinner(spinner),
	.paddle(paddle),
	.lstick(lstick),
	.rstick(rstick),
	.mouse(ps2_mouse),
	.dpad('{{dir1[3], dir1[2], dir1[1], dir1[0]}, {dir2[3], dir2[2], dir2[1], dir2[0]},
	        joystick_2[3:0], joystick_3[3:0]}),

	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wr0(ioctl_wr & (ioctl_index == 8'd0)),

	// NVRAM (MRA <nvram index="4" size="2048">): loaded with the MRA, saved when the OSD opens / closes
	.nv_addr(ioctl_addr[10:0]),
	.nv_din(ioctl_dout),
	.nv_we(ioctl_download & ioctl_wr & (ioctl_index == 8'd4)),
	.nv_dout(ioctl_din),

	.crt_flip(status[22]),
	// byte 1 mirrors are on the screen after MAME's axis swap; the board mirrors the raw picture, which
	// screen_rotate then turns, so a vertical set's screen X mirror is the board's Y mirror and vice versa
	.mirror_x(game_vert ? game_flags[2] : game_flags[1]),
	.mirror_y(game_vert ? game_flags[1] : game_flags[2]),
	.h_adj(status[40:37]),
	.v_adj(status[44:41]),

	.ce_pix(ce_pix),
	.video_r(r),
	.video_g(g),
	.video_b(b),
	.video_hs(hs),
	.video_vs(vs),
	.video_hblank(hblank),
	.video_vblank(vblank),
	.video_field(video_field),

	.audio_l(audio_l),
	.audio_r(audio_r),
	.op0(op0),
	.wd_reset()
);

assign ioctl_upload_req = ~status[27] & ~ioctl_download;

endmodule

// CTL-2026-10-24: no longer instantiated (ctl_joy4way in rtl/mister_custom/controllers.sv replaces it)
// // 4-way joystick: the most recently pressed direction wins (from the MiSTer Pac-Man port)
// module joy4way
// (
// 	input        clk,
// 	input        en,
// 	input  [3:0] in,
// 	output [3:0] out
// );
//
// reg  [3:0] mask = 4'hF;
// reg  [3:0] in1 = 0, in2 = 0;
// wire [3:0] innew = in1 & ~in2;
//
// assign out = in1 & mask;
//
// always @(posedge clk) begin
// 	in1 <= in;
// 	in2 <= in1;
//
// 	if (innew[0]) mask <= 4'b0001;
// 	if (innew[1]) mask <= 4'b0010;
// 	if (innew[2]) mask <= 4'b0100;
// 	if (innew[3]) mask <= 4'b1000;
//
// 	if (!(in & mask) || !en) mask <= 4'hF;
// end
//
// endmodule

