# Joyful Noise Tiny Utility Board Firmware

## Firmware To-Do

### ENV
- [x] Initial implementation
- [x] Refactor to use `FastClock`
- [ ] Make retrig optional
- [ ] Use 10-bit PWM?

### LFO
- [x] Initial implementation
- [x] 10-bit PWM
- [ ] Automatic clock detection
- [ ] Allow control of amplitude

### VCO
- [x] Initial implementation
- [ ] Adjust SUB/DTN knob response to reflect panel graphics
- [ ] Investigate waveform glitches
- [ ] Ensure audio-rate sync works
- [ ] Experiment with 10-bit audio?
- [ ] Experiment with band-limited wavetables?

### RAND
- [x] Initial implementation
- [ ] Allow control of randomness and/or slew
- [ ] Make it faster?

### Beat Tool
- [x] Implement clock generation
- [x] Implement clock division
- [x] Implement clock multiplication
- [x] Implement burst generation
- [ ] Refactor to use `FastClock`
- [ ] Program the "internal clock" signal chain logic
- [ ] Implement swing
- [ ] Implement gate delay
- [ ] Implement gate lengthen
- [ ] Implement random sequencing
- [ ] Implement euclidean sequencing

### QUANT
- [ ] Initial implementation

### CV Tool
- [ ] Initial implementation

### µTM
- [ ] Initial implementation
- [ ] Implement random gates
- [ ] Implement chaining

### RAT
- [x] Port and test Muskrat firmware
- [ ] Improve user interface

### DSL
- [ ] Port and test Dialup firmware
- [ ] Improve user interface

### STRANG
- [x] Initial Karplus-Strong implementation
- [ ] Much more experimenting to get a decent instrument out of it
- [ ] Experiment with 10-bit audio?

### LFG
- [ ] Initial implementation

### CHRD
- [ ] Initial implementation

### GRNS
- [ ] Port and test Grains firmware
- [ ] Port and test alternative Grains firmwares?
- [ ] Port and test TOOL firmwares?
