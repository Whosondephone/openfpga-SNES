# Native 48 kHz audio

The S-DSP generates audio at ~32.07 kHz (32 kHz with `DSP_FREQ`). The Pocket's
I2S DAC runs at 48 kHz. Previously the core sampled-and-held the DSP output once
per 48 kHz frame, which irregularly repeated samples (1 or 2 frames each) and
caused audible aliasing/distortion.

`target/pocket/audio_resampler.sv` now converts the DSP stream to 48 kHz:

- Every DSP sample is captured using the DSP's `SND_RDY` strobe (plumbed out as
  `AUDIO_RDY` / `audio_rdy`).
- The I2S block (`sound_i2s.sv`) requests exactly one sample per 48 kHz frame.
- The resampler measures the real DSP sample period and computes the exact
  fractional position of each output sample, then evaluates a 32 tap, 64 phase
  Kaiser windowed-sinc polyphase filter.

Each input sample is used exactly once and each output sample is produced on
demand by the DAC clock, so pitch and tempo are unchanged.

## Regenerating coefficients

```
python3 support/audio/gen_resampler_coefs.py
```

## Simulation (Icarus Verilog)

```
cd support/audio/sim
P=../../../target/pocket
iverilog -g2012 -o new.vvp -P tb.NEW=1 -P tb.FREQ=3000.0 -P tb.DUR_S=0.3 \
  tb.sv sync_fifo_sim.sv $P/audio_resampler.sv $P/audio_resampler_coefs.sv $P/sound_i2s.sv
vvp -n new.vvp   # writes out.txt (decoded I2S samples at 48 kHz)
```

`an.py` reports the measured tone frequency and THD+N. Results (20000 amplitude tone):

| Tone | Old THD+N | New THD+N | Pitch |
|---|---|---|---|
| 440 Hz | -32.1 dB | -80.9 dB | identical |
| 1 kHz | -24.9 dB | -76.7 dB | identical |
| 3 kHz | -15.3 dB | -68.7 dB | identical |
| 7 kHz | -7.6 dB | -61.4 dB | identical |
| 12 kHz | -2.1 dB | -56.7 dB | identical |
