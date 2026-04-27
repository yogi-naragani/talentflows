# Hardware target: Raspberry Pi 5 + AI HAT (Hailo)

This document captures the deployment assumptions for the paper's
hardware experiments. Numbers are vendor-published or measured on
common builds; treat them as design estimates and re-measure on the
actual airframe before final results.

## Compute platform

| Component | Spec |
|---|---|
| SoC | Broadcom BCM2712, quad-core Cortex-A76 @ 2.4 GHz |
| RAM | 8 GB LPDDR4X-4267 (16 GB option preferred for SLM headroom) |
| Storage | NVMe via M.2 HAT, or A2 microSD (NVMe strongly recommended) |
| AI accelerator | Raspberry Pi AI HAT+ |
| NPU options | Hailo-8L (13 TOPS) or Hailo-8 (26 TOPS) |
| Connectivity | PCIe 2.0 x1 (FW 2.0 unlocks PCIe 3.0 x1) |
| OS | Raspberry Pi OS Bookworm 64-bit, or Ubuntu 24.04 |
| ROS 2 | Jazzy (Ubuntu 24.04) — Humble works on 22.04 |

## Power consumption

Pi 5 alone:

- Idle, headless: ~3 W
- Typical load (one core busy, video decode, USB): ~5–7 W
- Sustained 4-core 100% + NVMe I/O: ~9–12 W
- Recommended PSU: 27 W (5 V / 5 A USB-C PD); below that the firmware
  caps USB current and may throttle

Hailo on the AI HAT+:

- Hailo-8L: ~2.5 W typical
- Hailo-8: ~5 W typical, ~7–8 W peak under sustained inference

Combined Pi 5 + AI HAT+ (Hailo-8) under our workload:

- Steady state (SLAM + MPC + acoustic + reasoning idle): ~10–12 W
- Burst (SLM advisory tick + perception together): ~14–16 W
- Headroom for camera, IMU, range sensor, mic preamp: +1–2 W
- **Budget for compute payload: ~15–18 W**

For a 500 g class quadrotor in hover (~150 W to motors), compute is
~10% of total power. Useful flight-time impact is small; thermal
management on the Pi (active cooler required) matters more than the
energy cost.

## Compute budget allocation

Estimated CPU shares on the four Cortex-A76 cores at 2.4 GHz, with
ORB-SLAM3 mono-inertial as the SLAM backend (see
`docs/slam_tradeoffs.md`):

| Workload | Cores | Notes |
|---|---|---|
| ORB-SLAM3 (mono-inertial, VGA, 30 Hz) | ~1.5 | Tracking on one core, local mapping on another |
| Acoustic processing (4-ch, 48 kHz, 20 Hz) | ~0.5 | Band-pass + GCC-PHAT on CPU; movable to NPU later |
| MPC (CasADi + IPOPT, N=20, 100 Hz) | ~0.3 | HPIPM swap reduces this further |
| ROS 2 middleware + bridges | ~0.2 | DDS overhead on a single Pi |
| SLM advisor (0.5–2 Hz, see below) | ~1.0 burst | Bursts during a tick; idle otherwise |
| Headroom | ~0.5 | Logging, watchdog, recovery |

Total ~4 cores with bursty contention during SLM ticks. If contention
during SLM bursts hurts MPC determinism, route the SLM through Hailo
or move it to an external companion (Pixel phone tethered).

## NPU offload candidates (Hailo-8/8L)

Things that map cleanly to Hailo:

- ORB feature extraction or SuperPoint + LightGlue for the SLAM
  front-end (frees ~0.5–1 CPU core; many community models exist)
- Per-frame scene classifier feeding the reasoner ("corridor",
  "open", "low-texture wall") to keep the prompt compact
- Acoustic 1D-CNN for proximity classification, replacing the
  hand-rolled GCC-PHAT path

Things that do **not** fit Hailo well right now:

- Decoder-only LLMs (Gemma / Phi / Llama) — Hailo's compiler targets
  CNN/ViT graphs; SLM inference stays on CPU via llama.cpp.
- Bundle adjustment / IPOPT — solver work, not tensor work.

## On-device SLM substitution: "Gemini Nano-class," not Gemini Nano

Honest framing: Gemini Nano runs on Pixel / Tensor G3+ via Android
AICore. **It does not run on Pi 5.** For deployment on Pi 5 + AI HAT
the realistic substitutes for the *advisory* role are:

| Model | Params | Quant | RAM | Pi 5 CPU tok/s | Notes |
|---|---|---|---|---|---|
| Gemma 3 1B Instruct | 1.0 B | Q4_K_M | ~1.2 GB | ~5–10 | Best quality/speed for advisor role |
| Llama 3.2 1B Instruct | 1.2 B | Q4_K_M | ~1.4 GB | ~4–8 | Strong instruction following |
| Phi-3.5 mini Instruct | 3.8 B | Q4_K_M | ~3.0 GB | ~2–4 | Stronger reasoning, slower |
| Qwen 2.5 1.5B | 1.5 B | Q4_K_M | ~1.5 GB | ~4–7 | Solid tool/JSON output |

Recommended default: **Gemma 3 1B Q4_K_M via llama.cpp on CPU.**
Output is the same JSON schema; one tick (≤128 output tokens) lands
in well under one second, matching the 0.5–2 Hz advisory rate.

The paper should say: "We deploy a Gemini Nano-class on-device small
language model. On the Pi 5 + AI HAT target we use Gemma 3 1B
(Q4\_K\_M, llama.cpp) as a stand-in; on Tensor-class hardware (Pixel
8/9) the equivalent role is filled by Gemini Nano via AICore. The
contribution is the *role*, not the specific weights."

## Sim vs hardware mode

Gazebo Harmonic does not run usefully in real time on the Pi 5. We
use three deployment modes:

1. **Pure simulation on workstation.** Everything runs locally;
   `ros2 launch gps_denied_drone sim.launch.py`. Used for ablations
   and trajectory-error metrics.
2. **Hardware-in-the-loop.** gz-sim on a workstation, the five drone
   nodes on the Pi 5, ROS 2 over wired Ethernet (RMW Cyclone DDS,
   `ROS_LOCALHOST_ONLY=0`). Validates real compute/power on the Pi.
3. **Pure hardware flight.** No sim; nodes consume real camera /
   IMU / range / mic streams, MPC outputs go to the flight
   controller (PX4 or Betaflight).

A `launch/deploy_pi5.launch.py` variant brings up only the nodes
without gz-sim, configured for hardware topics.

## Action items before publication

- Measure end-to-end latency: image-in to motor-cmd-out, on the Pi 5,
  warm CPU.
- Measure SLM tick latency under contention with SLAM + MPC running.
- Measure thermal: case-temperature soak at 25 °C ambient with the
  Active Cooler; report whether throttling occurs.
- Verify Hailo driver versions: HailoRT >= 4.18 with Pi 5 firmware
  that exposes PCIe Gen 3.
- Confirm power draw with a USB-C PD meter on a representative
  flight loadout.
