`timescale 1ps/1ps
module tb;
  parameter real FREQ = 1000.0;
  parameter integer NEW = 1;
  parameter real DUR_S = 0.12;
  reg clk_sys = 0, clk_74a = 0;
  always #23280 clk_sys = ~clk_sys;   // ~21.477 MHz
  always #6734  clk_74a = ~clk_74a;   // ~74.25 MHz
  // DSP CE model (CEGen-like fractional divider) and sample strobe every 128 CEs
  integer acc = 0, cecnt = 0, n = 0;
  reg rdy = 0; reg signed [15:0] l = 0, r = 0;
  real fs_in; initial fs_in = 4104960.0/128.0;
  always @(posedge clk_sys) begin
    rdy <= 0;
    acc = acc + 4104960;
    if (acc >= 21477270) begin
      acc = acc - 21477270;
      cecnt = cecnt + 1;
      if (cecnt == 128) begin
        cecnt = 0;
        l <= $rtoi(20000.0*$sin(2.0*3.14159265358979*FREQ*n/fs_in));
        r <= $rtoi(20000.0*$cos(2.0*3.14159265358979*FREQ*n/fs_in));
        n = n + 1; rdy <= 1;
      end
    end
  end
  wire mclk, lrck, dac;
  wire req, vld; wire [15:0] ol, or_;
  integer f;
  generate if (NEW) begin : g
    audio_resampler rs(.clk(clk_sys), .reset(1'b0), .in_valid(rdy), .in_l(l), .in_r(r),
      .out_req(req), .out_valid(vld), .out_l(ol), .out_r(or_));
    sound_i2s #(.CHANNEL_WIDTH(16), .SIGNED_INPUT(1)) i2s(.clk_74a(clk_74a), .clk_audio(clk_sys),
      .audio_l(ol), .audio_r(or_), .audio_sample_valid(vld), .audio_sample_req(req),
      .audio_mclk(mclk), .audio_lrck(lrck), .audio_dac(dac));
  end else begin : g
    sound_i2s_old #(.CHANNEL_WIDTH(16), .SIGNED_INPUT(1)) i2s(.clk_74a(clk_74a), .clk_audio(clk_sys),
      .audio_l(l), .audio_r(r), .audio_mclk(mclk), .audio_lrck(lrck), .audio_dac(dac));
  end endgenerate
  // I2S receiver: 64 SCLK per frame, MSB first, sampled on SCLK rising edge
  wire sclk = g.i2s.audgen_sclk;
  reg [31:0] sh = 0; reg plr = 0; integer bitn = 0; reg [15:0] left;
  initial begin
    g.i2s.audio_mclk = 0; g.i2s.aud_mclk_divider = 0; g.i2s.audio_lrck = 0; g.i2s.audio_lrck_cnt = 0;
    f = $fopen("out.txt", "w");
    #(DUR_S*1e12); $fclose(f); $finish;
  end
  always @(posedge sclk) begin
    if (lrck != plr) begin
      if (bitn >= 17) begin
        if (plr == 0) left = sh[15:0]; else $fdisplay(f, "%0d %0d", $signed(left), $signed(sh[15:0]));
      end
      bitn = 0; sh = 0;
    end
    if (bitn >= 1 && bitn <= 16) sh = {sh[30:0], dac};
    bitn = bitn + 1; plr = lrck;
  end
endmodule
