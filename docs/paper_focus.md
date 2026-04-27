# Paper focus (this branch): passive acoustic backup

This branch leads the paper with a single sharp empirical claim:

> Passive ego-noise reflectometry, using onboard microphones and the
> drone's own propeller noise as the illumination source, is a viable
> SLAM-degradation backup for GPS-denied micro-drones.

Everything else in the repository -- ORB-SLAM3, the 1D rangefinder,
the perception-aware MPC, the on-device SLM advisor, the Pi 5 + AI
HAT target -- is *the host system in which the claim is evaluated*,
not the contribution.

## Central comparison

`(acoustic_backup_on)` vs `(acoustic_backup_off)` under scripted
SLAM-dropout scenarios (`experiments/configs/paper_acoustic.yaml`):

- corridor white wall
- 2 s synthetic texture mask burst
- sudden illumination drop
- fast yaw that breaks ORB tracking

25 trials per cell across 5 seeds. Primary metric: collision count.
Secondaries: minimum clearance, acoustic TP/FP rate, mission
completion, rotor-SNR-at-mic, extra power consumption.

## Most important supporting evidence

- **SNR-floor bench**: `experiments/snr_analysis.py`. Sweep reflector
  distance and rotor RPM; show the SNR boundary above which proximity
  classification is reliable. This number is the central honest
  caveat the paper must own.
- **BatDeck contrast**: simulate the active-ultrasound proximity
  signal from the same scenario geometry to show what we *give up* by
  staying passive. Argue the wins (no emitter, multi-role mic, lower
  acoustic signature) explicitly, with numbers.

## Out of scope for this paper

- The SLM-advisor design pattern. We use the SLM as published in the
  shared codebase but make no claim about whether it beats a
  rule-tree supervisor. That claim lives in the sibling branch
  `claude/paper-slm-advisor-VgEQR`.

## Realistic venue

ICUAS or RA-L if execution is solid. ICRA / IROS main track if we
also land a real-flight demo (tethered indoor is enough) and the
SNR-floor result is strong.
