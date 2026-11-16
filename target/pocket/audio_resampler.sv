// MIT License
//
// Copyright (c) 2026
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
//
// Asynchronous sample rate converter: SNES S-DSP (~32 kHz) -> I2S (48 kHz)
//
// The S-DSP produces one stereo sample every 128 DSP clocks (~32.07 kHz, or
// exactly 32 kHz with DSP_FREQ=1, and slightly different again on PAL). The
// Pocket's I2S DAC consumes exactly 48 kHz derived from the 74.25 MHz clock.
//
// The original core sampled-and-held the DSP output once per I2S frame. Because
// 48/32 is not an integer, DSP samples were irregularly held for 1 or 2 output
// frames, which produces strong aliasing / imaging ("gritty", "crunchy" sound).
//
// This module instead:
//   * Captures every DSP sample using the DSP's real SND_RDY strobe
//   * Measures the actual DSP sample period (averaged over 32 samples) in
//     clk_sys cycles, so it adapts to NTSC/PAL and the DSP_FREQ setting
//   * On every I2S frame request, computes the exact fractional position of
//     "now" between DSP samples, and evaluates a 32 tap polyphase Kaiser
//     windowed-sinc interpolation filter (64 phases, linearly interpolated
//     between phases) at that position
//
// Every input sample is consumed exactly once and every output sample is
// produced exactly when the DAC asks for it, so no samples are dropped or
// repeated and the pitch / tempo of the audio is identical to the original
// 32 kHz stream. Added latency is TAPS/2 DSP samples (~0.5 ms).
//
// Cost: 1 small coefficient ROM (1056 x 18), 1 64 x 32 history RAM, 3
// multipliers. One output takes ~200 clk_sys cycles out of the ~447 available.
//
module audio_resampler #(
    // Nominal DSP sample period * 32, in clk_sys cycles. Used until the first
    // measurement completes. 21477270 / 32070 * 32 ~= 21430
    parameter [15:0] NOMINAL_PERIOD_X32 = 16'd21430,
    // Valid range for a measured period. Measurements outside of this range
    // (e.g. when the DSP is paused for save states) are discarded.
    parameter [15:0] MIN_PERIOD_X32 = 16'd19000,
    parameter [15:0] MAX_PERIOD_X32 = 16'd24000
) (
    input wire clk,
    input wire reset,

    // Input samples from the S-DSP (clk domain)
    input wire               in_valid,
    input wire signed [15:0] in_l,
    input wire signed [15:0] in_r,

    // Output sample request (one clk-wide pulse per output frame)
    input wire out_req,

    // Interpolated output samples, valid for one cycle
    output reg               out_valid = 0,
    output reg signed [15:0] out_l = 0,
    output reg signed [15:0] out_r = 0
);
  // Must match support/audio/gen_resampler_coefs.py
  localparam TAPS = 32;
  localparam PHASES = 64;

  ////////////////////////////////////////////////////////////////////////////
  // Input history (ring buffer) and timing

  reg [31:0] hist[64];
  reg [5:0] hist_wp = 0;  // Next write location. Newest sample is hist_wp - 1
  reg [5:0] hist_rp = 0;
  reg [31:0] hist_q = 0;

  integer i;
  initial begin
    for (i = 0; i < 64; i = i + 1) hist[i] = 0;
  end

  always @(posedge clk) begin
    if (in_valid) hist[hist_wp] <= {in_l, in_r};
    hist_q <= hist[hist_rp];
  end

  // Cycles since the most recent input sample (saturating)
  reg [15:0] since_in = 0;

  // Period measurement over 32 input samples
  reg [15:0] period_cnt = 0;
  reg [4:0] period_samples = 0;
  reg [15:0] period_x32 = NOMINAL_PERIOD_X32;

  always @(posedge clk) begin
    if (reset) begin
      since_in <= 0;
      period_cnt <= 0;
      period_samples <= 0;
      period_x32 <= NOMINAL_PERIOD_X32;
    end else begin
      if (since_in != 16'hFFFF) since_in <= since_in + 16'd1;
      if (period_cnt != 16'hFFFF) period_cnt <= period_cnt + 16'd1;

      if (in_valid) begin
        hist_wp  <= hist_wp + 6'd1;
        since_in <= 16'd1;

        period_samples <= period_samples + 5'd1;
        if (period_samples == 5'd31) begin
          // Counter covers exactly 32 sample periods here
          period_cnt <= 16'd1;

          if (period_cnt >= MIN_PERIOD_X32 && period_cnt <= MAX_PERIOD_X32) begin
            period_x32 <= period_cnt;
          end
        end
      end
    end
  end

  ////////////////////////////////////////////////////////////////////////////
  // Coefficient ROM. Only phases 0..32 are stored; the kernel is symmetric so
  // C[p][k] == C[64 - p][31 - k]

  reg [10:0] rom_addr = 0;
  wire signed [17:0] rom_q;

  audio_resampler_coefs coefs (
      .clk (clk),
      .addr(rom_addr),
      .q   (rom_q)
  );

  function automatic [10:0] coef_index(input [6:0] phase, input [4:0] tap);
    reg [6:0] mirrored;
    begin
      mirrored = 7'd64 - phase;
      if (phase <= 7'd32) coef_index = {phase[5:0], tap};
      else coef_index = {mirrored[5:0], ~tap};  // ~tap == 31 - tap
    end
  endfunction

  ////////////////////////////////////////////////////////////////////////////
  // Output computation

  localparam ST_IDLE = 0;
  localparam ST_DIV = 1;
  localparam ST_SETUP = 2;
  localparam ST_TAP0 = 3;
  localparam ST_TAP1 = 4;
  localparam ST_TAP2 = 5;
  localparam ST_TAP3 = 6;
  localparam ST_TAP4 = 7;
  localparam ST_OUT = 8;

  reg [3:0] state = ST_IDLE;

  // Divider: mu = (elapsed * 32 * 65536) / period_x32, clamped to 16 bits
  reg [31:0] div_rem;
  reg [31:0] div_num;
  reg [15:0] div_den;
  reg [16:0] div_quot;
  reg [5:0] div_count;
  reg mu_saturated;

  wire [32:0] div_trial = {div_rem[31:0], div_num[31]} - {17'b0, div_den};

  reg [15:0] mu;
  wire [6:0] phase = {1'b0, mu[15:10]};
  wire [9:0] phase_frac = mu[9:0];

  reg [5:0] base;  // Snapshot of the newest sample's location
  reg [4:0] tap;

  reg signed [17:0] coef0;
  reg signed [19:0] coef;
  reg signed [15:0] x_l, x_r;
  reg signed [39:0] acc_l, acc_r;

  wire signed [18:0] coef_delta = rom_q - coef0;
  wire signed [30:0] coef_delta_scaled = coef_delta * $signed({1'b0, phase_frac});

  function automatic signed [15:0] round_clamp(input signed [39:0] x);
    reg signed [39:0] r;
    begin
      r = (x + 40'sd32768) >>> 16;
      if (r > 40'sd32767) round_clamp = 16'sh7FFF;
      else if (r < -40'sd32768) round_clamp = 16'sh8000;
      else round_clamp = r[15:0];
    end
  endfunction

  always @(posedge clk) begin
    out_valid <= 0;

    if (reset) begin
      state <= ST_IDLE;
      out_l <= 0;
      out_r <= 0;
    end else begin
      case (state)
        ST_IDLE: begin
          if (out_req) begin
            base <= hist_wp - 6'd1;

            // since_in counts from 1 at the strobe; convert to elapsed cycles
            div_num <= {since_in - 16'd1, 16'b0} << 5;
            div_den <= period_x32;
            div_rem <= 0;
            div_quot <= 0;
            div_count <= 0;

            // Guard against since_in overflowing the numerator
            mu_saturated <= since_in >= 16'd2048;
            state <= since_in >= 16'd2048 ? ST_SETUP : ST_DIV;
          end
        end

        ST_DIV: begin
          // Restoring division, one bit per clock
          div_num <= {div_num[30:0], 1'b0};

          if (!div_trial[32]) begin
            div_rem  <= div_trial[31:0];
            div_quot <= {div_quot[15:0], 1'b1};
          end else begin
            div_rem  <= {div_rem[30:0], div_num[31]};
            div_quot <= {div_quot[15:0], 1'b0};
          end

          // Saturate: once the quotient would exceed 16 bits, mu = ~1.0
          if (div_quot[16]) div_quot <= 17'h1FFFF;

          div_count <= div_count + 6'd1;
          if (div_count == 6'd31) state <= ST_SETUP;
        end

        ST_SETUP: begin
          mu <= (mu_saturated || div_quot[16]) ? 16'hFFFF : div_quot[15:0];
          tap <= 0;
          acc_l <= 0;
          acc_r <= 0;
          state <= ST_TAP0;
        end

        // Output time is TAPS/2 samples behind "now":
        //   y = sum_k x[newest - k] * h(k + mu - TAPS/2)
        ST_TAP0: begin
          rom_addr <= coef_index(phase, tap);
          hist_rp <= base - {1'b0, tap};
          state <= ST_TAP1;
        end

        ST_TAP1: begin
          rom_addr <= coef_index(phase + 7'd1, tap);
          state <= ST_TAP2;
        end

        ST_TAP2: begin
          // ROM and history RAM data for TAP0 addresses are now valid
          coef0 <= rom_q;
          x_l <= hist_q[31:16];
          x_r <= hist_q[15:0];
          state <= ST_TAP3;
        end

        ST_TAP3: begin
          // Linearly interpolate between adjacent phases
          coef  <= coef0 + (coef_delta_scaled >>> 10);
          state <= ST_TAP4;
        end

        ST_TAP4: begin
          acc_l <= acc_l + coef * x_l;
          acc_r <= acc_r + coef * x_r;
          tap   <= tap + 5'd1;
          state <= tap == TAPS - 1 ? ST_OUT : ST_TAP0;
        end

        ST_OUT: begin
          out_l <= round_clamp(acc_l);
          out_r <= round_clamp(acc_r);
          out_valid <= 1;
          state <= ST_IDLE;
        end

        default: state <= ST_IDLE;
      endcase
    end
  end
endmodule
