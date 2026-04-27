"""Python-only simulator harness.

Runs the same Monitor + advisor + SafetySupervisor + controller loop
as the ROS 2 / Gazebo stack, but with synthetic sensors and dynamics
so paper experiments can be reproduced on a laptop in seconds. No
ROS 2, no Gazebo, no GUI dependencies.

Layout:
    quadrotor.py     point-mass + attitude dynamics
    world.py         scenario geometry and ground truth
    degradation.py   programmable SLAM / sensor degradation
    sensors_sim.py   synthetic camera / range / acoustic / IMU / battery
    runner.py        glues everything together for one trial
    metrics.py       per-trial KPIs
    cli.py           runs an ablation grid from a YAML config
"""
