# contracts/learned_component.md

**Pre-state:** `worlds/warehouse.sdf` runs end-to-end with VIO. Brev
account configured with a spot-instance GPU template (RTX 4090 / A10
class is fine).

**Post-state (Option A, depth):** A ROS 2 node consumes
`/camera/image_raw` and publishes `/depth/predicted` (sensor_msgs/Image).
Side-by-side RViz screenshot of RGB / predicted depth / ground-truth
depth lives in `results/depth_qualitative.md`.

**Post-state (Option B, RL policy):** ONNX-exported PPO policy runs
in a ROS 2 node consuming depth + IMU and publishing velocity
commands. Drone avoids obstacles in a separate test world.

**Dependencies:**
- `contracts/custom_world.md`
- `contracts/vio_pipeline.md`
- Brev (cloud GPU)

**Breakage risk:** MEDIUM. Training is unpredictable; ONNX export +
runtime version mismatch is a classic failure mode.

**Acceptance test (Option A):**
```bash
ros2 run gps_denied_drone depth_node
ros2 topic hz /depth/predicted   # ~10-30 Hz
```
A screenshot in `results/depth_qualitative.md` shows that the
prediction follows the obstacle silhouette.

**Current status:** todo. Day 8-9 of the sprint.

**Decision: pick Option A (depth).** Faster, fewer moving parts, the
qualitative result is enough for the paper. RL is a Day-11 stretch.

**Watch out for:**
- Use `onnxruntime` not full PyTorch for inference -- 10x faster on
  CPU and avoids the CUDA-version sin.
- Depth Anything v2 small (~25 MB) is plenty; do not chase huge
  models.
- If fine-tuning runs >30 minutes per epoch on Brev, **abort and use
  zero-shot**. The paper still benefits from a real depth signal in
  the loop; the fine-tune is icing.

**Cost ceiling:** $80 of Brev credit. Use spot instances; checkpoint
to local disk after every epoch.
