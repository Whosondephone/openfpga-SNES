// MIT License

// Copyright (c) 2022 Adam Gastineau

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

// A very simple audio i2s bridge to APF, based on their example code
//
// Modified: samples are no longer sampled-and-held from a free running source.
// Once per 48kHz I2S frame, `audio_sample_req` is pulsed (in the clk_audio
// domain). The core must answer with exactly one new sample by pulsing
// `audio_sample_valid` (also clk_audio domain). That sample is played in the
// following I2S frame, so the source runs natively at 48kHz, locked to the DAC.
module sound_i2s #(
    parameter CHANNEL_WIDTH = 15,
    parameter SIGNED_INPUT  = 0
) (
    input wire clk_74a,
    input wire clk_audio,

    // Left and right audio channels. Can be in an arbitrary clock domain
    input wire [CHANNEL_WIDTH - 1:0] audio_l,
    input wire [CHANNEL_WIDTH - 1:0] audio_r,
    // Pulse for one clk_audio cycle when audio_l/audio_r hold a new 48kHz sample
    input wire audio_sample_valid,
    // Pulses for one clk_audio cycle at the start of every 48kHz I2S frame
    output reg audio_sample_req = 0,

    output reg audio_mclk,
    output reg audio_lrck,
    output reg audio_dac
);
  //
  // audio i2s generator
  //

  reg audgen_nextsamp;

  // generate MCLK = 12.288mhz with fractional accumulator
  reg [21:0] audgen_accum = 0;
  localparam [20:0] CYCLE_48KHZ = 21'd122880 * 2;
  always @(posedge clk_74a) begin
    audgen_accum <= audgen_accum + CYCLE_48KHZ;
    if (audgen_accum >= 21'd742500) begin
      audio_mclk   <= ~audio_mclk;
      audgen_accum <= audgen_accum - 21'd742500 + CYCLE_48KHZ;
    end
  end

  // generate SCLK = 3.072mhz by dividing MCLK by 4
  reg [1:0] aud_mclk_divider;
  reg prev_audio_mclk;
  wire audgen_sclk = aud_mclk_divider[1]  /* synthesis keep*/;

  always @(posedge clk_74a) begin
    if (audio_mclk && ~prev_audio_mclk) begin
      aud_mclk_divider <= aud_mclk_divider + 1'b1;
    end

    prev_audio_mclk <= audio_mclk;
  end

  // shift out audio data as I2S
  // 32 total bits per channel, but only 16 active bits at the start and then 16 dummy bits
  //
  // synchronize audio samples coming from the core

  localparam CHANNEL_LEFT_HIGH = SIGNED_INPUT ? 16 : 15;
  localparam CHANNEL_RIGHT_HIGH = 16 + CHANNEL_LEFT_HIGH;

  // Width of channel with signed component
  localparam SIGNED_CHANNEL_WIDTH = SIGNED_INPUT ? CHANNEL_WIDTH : CHANNEL_WIDTH + 1;

  wire [31:0] audgen_sampdata;

  assign audgen_sampdata[CHANNEL_LEFT_HIGH-1:CHANNEL_LEFT_HIGH-CHANNEL_WIDTH]   = audio_l;
  assign audgen_sampdata[CHANNEL_RIGHT_HIGH-1:CHANNEL_RIGHT_HIGH-CHANNEL_WIDTH] = audio_r;

  generate
    if (!SIGNED_INPUT) begin
      // If not signed, make sure high bit is 0
      assign audgen_sampdata[31] = 0;
      assign audgen_sampdata[15] = 0;
    end
  endgenerate

  generate
    if (15 - SIGNED_CHANNEL_WIDTH > 0) begin
      assign audgen_sampdata[31-SIGNED_CHANNEL_WIDTH:16] = 0;
      assign audgen_sampdata[15-SIGNED_CHANNEL_WIDTH:0]  = 0;
    end
  endgenerate

  sync_fifo #(
      .WIDTH(32)
  ) sync_fifo (
      .clk_write(clk_audio),
      .clk_read (clk_74a),

      .write_en(audio_sample_valid),  // one write per 48kHz sample
      .data(audgen_sampdata),
      .data_s(audgen_sampdata_s)
  );

  // Request the next sample from the core once per I2S frame. A toggle is
  // generated in the clk_74a domain and synchronized into clk_audio
  reg req_toggle_74a = 0;
  reg [2:0] req_toggle_s = 0;

  always @(posedge clk_audio) begin
    req_toggle_s <= {req_toggle_s[1:0], req_toggle_74a};
    audio_sample_req <= req_toggle_s[2] ^ req_toggle_s[1];
  end

  wire [31:0] audgen_sampdata_s;

  reg [31:0] audgen_sampshift;
  reg [4:0] audio_lrck_cnt;
  reg prev_audgen_sclk;
  always @(posedge clk_74a) begin
    if (prev_audgen_sclk && ~audgen_sclk) begin
      // output the next bit
      audio_dac <= audgen_sampshift[31];

      // 48khz * 64
      audio_lrck_cnt <= audio_lrck_cnt + 1'b1;
      if (audio_lrck_cnt == 31) begin
        // switch channels
        audio_lrck <= ~audio_lrck;

        // Reload sample shifter
        if (~audio_lrck) begin
          audgen_sampshift <= audgen_sampdata_s;
          // Start of a new stereo frame: ask the core for the next sample
          req_toggle_74a   <= ~req_toggle_74a;
        end
      end else if (audio_lrck_cnt < 16) begin
        // only shift for 16 clocks per channel
        audgen_sampshift <= {audgen_sampshift[30:0], 1'b0};
      end
    end

    prev_audgen_sclk <= audgen_sclk;
  end

  initial begin
    // Verify parameters
    if (CHANNEL_WIDTH > 16) begin
      $error("CHANNEL_WIDTH must be <= 16. Received %d", CHANNEL_WIDTH);
    end

    if (SIGNED_INPUT != 0 && SIGNED_INPUT != 1) begin
      $error("SIGNED_INPUT must be 0 or 1. Received %d", SIGNED_INPUT);
    end

    if (CHANNEL_WIDTH == 16 && SIGNED_INPUT == 0) begin
      $error("Cannot have CHANNEL_WIDTH of 16 and an unsigned input");
    end
  end
endmodule
