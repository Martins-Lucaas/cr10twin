# Real-time control, filtering, latency and safety for IMU-based teleoperation of a 6-DOF industrial arm (Dobot CR10 focus)

## Q1. Filtering the reference signal, its latency cost, and acceptable end-to-end latency

### Takeaway
Plain low-pass filtering of the human reference removes physiological tremor (6–15 Hz) but adds phase lag. Speed-adaptive filters avoid much of that lag: the 1€ filter, and adaptive Kalman filters or Fourier linear combiners. In the human-factors studies found, performance drops measurably at about 200–300 ms of delay, and some authors put the threshold as low as 30 ms. A practical target is to keep IMU-to-motion latency well under 100–150 ms. No IMU-teleop paper found reports a clean end-to-end figure for an industrial arm.

### Cited Findings
- Physiological tremor lies at "6 to 15 Hz in each principal axis", with amplitudes of 50–100 µm. Low-pass filters attenuate it but have "inherent phase delays that were not conducive to real-time applications" — [IMU Motion Capture with Adaptive Tremor Attenuation (Sensors 2022, PMC9100553)](https://pmc.ncbi.nlm.nih.gov/articles/PMC9100553/)
- The same paper surveys the alternatives to low-pass filtering. FLC "cannot handle high frequency signals". WFLC degrades when the signal has several dominant frequencies. BMFLC needs prior knowledge of the subject's tremor. A fixed-gain EKF trades tremor rejection against delay. Its solution is a variable-gain EKF whose gain is driven by sEMG muscle activation — [PMC9100553](https://pmc.ncbi.nlm.nih.gov/articles/PMC9100553/)
- That system used 50 Hz IMUs (gForcePro+) to drive a ROKAE xMate3 Pro running a 1 kHz control loop. Against VICON, the IMU joint angles had RMSE 4.837° ± 1.433°. Tremor attenuation cut Z-tracking RMSE from 0.0015 m to 0.0006 m. The paper cites delays "30 ms and above" as able to degrade human-machine control, and an electromechanical delay of about 57 ms. It does not measure its own filter latency — [PMC9100553](https://pmc.ncbi.nlm.nih.gov/articles/PMC9100553/)
- The 1€ filter (Casiez, Roussel & Vogel, CHI 2012) is a first-order low-pass whose cutoff rises with speed. A low cutoff at low speed reduces jitter, and a higher cutoff at high speed reduces lag. Tuning has two steps. First set beta = 0 and mincutoff ≈ 1 Hz, then lower mincutoff while holding still until jitter is gone. Second, raise beta while moving fast until lag is acceptable. dcutoff is normally fixed at 1 Hz — [Casiez 1€ filter page](https://cristal.univ-lille.fr/~casiez/1euro); [oneeurofilter on PyPI](https://pypi.org/project/oneeurofilter)
- "There is likely not a single set of optimal values" for the 1€ filter. The best values depend on the devices, user preference and task — [GDR MACS/CNRS](https://gdr-macs.cnrs.fr/node/4547)
- In an IMU whole-body humanoid teleop system, the EMA filter introduced "a small, motion-speed-dependent phase lag". The authors suggest an adaptive Kalman filter could reduce it — [arXiv 2605.12347](https://arxiv.org/html/2605.12347)
- A paper on VR teleoperation of an industrial arm notes that position-controlled industrial arms need the incoming tracking signals filtered. The aim is to reach low latency while keeping the commanded motion feasible — [arXiv 2305.10960](https://arxiv.org/pdf/2305.10960)
- In a human-in-the-loop study, delays of 0–500 ms were tested in 100 ms steps (n = 10). Performance degraded significantly at 200–300 ms, and people began to perceive the delay in the 100–200 ms window — [arXiv 2508.18074](https://arxiv.org/html/2508.18074v1)
- In multi-robot search-and-rescue teleoperation (n = 80), 500 ms of latency significantly increased task time and perceived physical workload — [Automation in Construction / ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0926580517310890)
- A review of vehicle teleoperation reports that about 300 ms has a noticeable effect on operator behaviour. It gives a cognitive threshold of about 0.4 s for real-time performance, and notes that people can detect latency as small as 10 ms — [PMC11207977](https://pmc.ncbi.nlm.nih.gov/articles/PMC11207977/)
- An fNIRS study (n = 41) found that teleoperation latency increases prefrontal and motor-cortex functional connectivity, a sign of higher cognitive load — [PMC11599268](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11599268/)
- Wearable IMU-only teleoperation of a collaborative industrial robot has been demonstrated (Škulj, Vrabič, Podržaj). Upper-body segment orientations are mapped to robot motion commands after a calibration that aligns the IMU, operator and robot frames. I could not retrieve the full text, so its latency numbers are unknown — [Sensors 2021, PMC8434127](https://pmc.ncbi.nlm.nih.gov/articles/PMC8434127/)

### Inferences
- Latency budget for CR10 ServoJ streaming at 33 Hz:
  - IMU sample period: about 10–20 ms at 50–100 Hz.
  - Filter group delay: tens of ms with a 1€ filter during fast motion, more with a fixed 2–5 Hz low-pass.
  - Command period: 30 ms.
  - Controller tracking delay: governed by ServoJ `t`/`aheadtime`; see Q3.
  - The total plausibly lands at 100–200 ms. That is on the edge of the 200–300 ms degradation zone, so every stage should be minimised and measured.
- Recommended filter order:
  1. Do sensor fusion (orientation) on the IMU itself.
  2. Apply a 1€ filter, or a tremor notch/FLC, to the task-space reference.
  3. Apply rate/acceleration limiting in joint space.
  4. Let the robot's own servo gain/lookahead do the final smoothing.
  - Do not stack several fixed low-pass filters, because their lags add up.
- Measure end-to-end latency directly. One option is to timestamp IMU packets, then cross-correlate the operator's wrist trajectory with joint feedback from the robot's 8 ms feedback port (30004).

### Gaps
- No peer-reviewed paper found reports measured end-to-end latency (IMU to robot motion) for an IMU-wearable system driving an industrial 6-DOF arm. The Škulj et al. full text was blocked by a browser check.
- No source quantified the group delay of the 1€ filter under specific parameters. This needs to be measured in the project.

## Q2. Online IK for streaming targets, singularities, joint limits, MoveIt Servo, and rate limiting

### Takeaway
For streaming targets, use either a Jacobian-based velocity servo with singularity and joint-limit scaling (MoveIt Servo), or a fast numerical or analytic IK seeded with the previous solution, followed by joint-space velocity/acceleration (ideally jerk) limiting. TRAC-IK solves about 99.9% of targets in roughly 0.2 ms on 6-DOF chains. KDL's plain inverse-Jacobian solver fails often.

### Cited Findings
- MoveIt Servo accepts three input types:
  - `JointJog` on `joint_command_in_topic`.
  - `TwistStamped` on `cartesian_command_in_topic`, which must be in the planning frame.
  - `PoseStamped` on `pose_command_in_topic`.
  - Output is a `JointTrajectory` or `Float64MultiArray`. A `pause_servo` service and a status topic are available.
  - Source: [MoveIt 2 Realtime Servo tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- MoveIt Servo scales velocities down when it approaches singularities and, optionally, collisions. It enforces joint position and velocity limits. IK is done via the inverse Jacobian or an IK plugin, with `bio_ik` given as the common choice. With an RT kernel it runs SCHED_FIFO at priority 40 — [MoveIt Servo tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- MoveIt Servo smoothing plugins — [MoveIt Servo tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html):
  - Butterworth: efficient and never overshoots in joint space, but does not limit acceleration or jerk.
  - AccelerationLimited: obeys acceleration limits, but not jerk, and may overshoot.
  - Ruckig: the smoothest, respects acceleration and jerk limits, and is "required for some industrial robots". It can swirl at sharp corners.
- MoveIt Servo defaults — [servo_parameters.yaml](https://raw.githubusercontent.com/moveit/moveit2/main/moveit_ros/moveit_servo/config/servo_parameters.yaml):

  | Parameter | Default | Meaning |
  |---|---|---|
  | `publish_period` | 0.01 s | 100 Hz output |
  | `incoming_command_timeout` | 0.1 s | Older commands are discarded |
  | `lower_singularity_threshold` | 17 | Jacobian condition number at which deceleration starts |
  | `hard_stop_singularity_threshold` | 30 | Condition number at which motion stops |
  | `leaving_singularity_threshold_multiplier` | 2.0 | |
  | `joint_limit_margins` | 0.1 (rad) | |
  | `check_collisions` | true | |
  | `collision_check_rate` | 10 Hz | |
  | `self_collision_proximity_threshold` | 0.01 m | Start decelerating |
  | `scene_collision_proximity_threshold` | 0.02 m | Start decelerating |
  | `max_expected_latency` | 0.1 s | |
  | `use_smoothing` | true | |
  | `smoothing_filter_plugin_name` | ButterworthFilterPlugin | |

- TRAC-IK README benchmarks (6-DOF ABB IRB120) — [TRAC-IK GitHub](https://github.com/traclabs/trac_ik); [TRAC-IK paper, Humanoids 2015 (IEEE)](https://ieeexplore.ieee.org/document/7363472/):

  | Solver | Solve rate | Time |
  |---|---|---|
  | KDL | 39.41% | 3.08 ms |
  | KDL-RR | 98.51% | 0.33 ms |
  | TRAC-IK | 99.96% | 0.24 ms |

  When numerical solvers fail, they run for the full timeout.
- In an independent test, TRAC-IK solved more than 99.8% of inputs and KDL 96% — [Open Arms, arXiv 2205.12992](https://arxiv.org/pdf/2205.12992)

### Inferences
- The CR10 has six revolute joints with a UR-like geometry (offset wrist). An analytic or closed-form IK, or TRAC-IK seeded with the current joint state, is fast enough for 33 Hz ServoJ or even 500 Hz.
  - Pick the branch nearest the previous solution, and reject any solution whose joint jump exceeds `v_max·dt`. This avoids elbow/wrist flips.
- Alternative: send ServoP (Cartesian) and let the Dobot controller do IK. This is simpler, but you lose control over singularity and branch handling. That makes ServoJ with your own IK plus limits the safer choice near wrist singularities.
- A damped least squares velocity IK (MoveIt Servo-style) with condition-number thresholds degrades gracefully near singularities. Using MoveIt Servo with pose commands and a Ruckig smoother, and forwarding its joint output to ServoJ, gives singularity, joint-limit and collision scaling "for free".
- Target interpolation: IMU at about 50–100 Hz versus 33 Hz ServoJ needs resampling. Hold the latest filtered target and pass it through a Ruckig/jerk-limited online trajectory generator at the command rate. This matches Dobot's advice to "carry out speed planning" (Q3).

### Gaps
- I did not find peer-reviewed, Dobot-specific IK guidance (DH parameters or analytic IK) for the CR10. Use the URDF in the project's description package.
- I did not retrieve the Ruckig paper (Berscheid & Kröger, RSS 2021) itself; it is mentioned only through the MoveIt docs.

## Q3. Dobot CR10 ServoJ/ServoP specifics compared with UR servoj

### Takeaway
On the V4 controller, the CR series exposes `ServoJ(J1..J6, t, aheadtime, gain)` and `ServoP(X,Y,Z,Rx,Ry,Rz, t, aheadtime, gain)`:
- Recommended rate: 33 Hz (30 ms).
- `t`: 0.004–3600 s, default 0.1.
- `aheadtime`: 20–100, default 50.
- `gain`: 200–1000, default 500.
- Each new Servo command interrupts the previous one.

UR's servoj expects about 500 Hz (t = 0.002 s), with lookahead 0.03–0.2 s and gain 100–2000. Dobot is a much lower-rate interface, so the host must provide smooth, speed-planned set-points.

### Cited Findings
- Dobot TCP/IP Remote Control Interface Guide (6-axis controller V4, document v4.6.6, 2026-05-27) — [Dobot TCP/IP guide v4.6.6 PDF](https://a.storyblok.com/f/298593/x/ce7fbaa06a/dobot-tcp_ip-remote-control-interface-guide-v4-6-6_20260527_en.pdf):
  - **ServoJ** is "the dynamic following command based on joint space… generally used for the stepping function of online control to realize dynamic following by cyclic calling. The calling frequency is recommended to be set to 33Hz, that is, the interval of cyclic calling is 30ms."
  - Joints are in degrees.
  - Optional parameters:

    | Parameter | Format | Range | Default | Notes |
    |---|---|---|---|---|
    | `t` | `t=value` | [0.004, 3600.0] s | 0.1 | |
    | `aheadtime` | `aheadtime=value` | [20.0, 100.0] | 50 | "Advanced time, similar to the D term in PID control", unitless |
    | `gain` | `gain=value` | [200.0, 1000.0] | 500 | "Proportional gain for the target position, similar to the P term" |

  - "Smaller aheadtime or larger gain values enable quicker response, but may cause instability or jitter."
  - NOTICE: "This command is not affected by the global rate, but is constrained by the speed limit. If the t value is set too small, the robot will not be able to meet the specified t due to the speed limit… it is recommended to carry out speed planning for the running point, and issue the speed-planned points at a fixed interval t to ensure that the robot can smoothly track the target point."
  - The return is `ErrorID,{ResultID},ServoJ(...)`, where ResultID is the algorithm-queue ID.
  - The doc's example is `ServoJ(0,0,-90,0,90,0,t=0.1,aheadtime=50,gain=500)`, called every 30 ms with J3 incremented 1° each time.
  - **ServoP** has the same parameters with a Cartesian pose: X/Y/Z in mm, Rx/Ry/Rz in degrees, relative to the global User/Tool frames. It has the same 33 Hz recommendation.
  - ServoJ and ServoP were added in V4.5.0 (2023-11). V4.6.0 (2025-01) "Corrected the execution time range for ServoP and ServoJ commands" and added RequestControl.
  - Ports:
    - 29999 is the Dashboard (control and motion commands in V4).
    - 30004 sends real-time feedback every 8 ms (1440-byte packets).
    - 30005 sends every 200 ms by default, and 30006 every 1000 ms by default.
  - `RequestControl()` switches the controller to TCP mode, which can only be entered while the robot is disabled or unpowered.
  - `EmergencyStop(1)` disables the robot and raises an alarm.
  - `Stop`/`Pause` act on the motion queue.
  - `SetCollisionLevel(0–5)`: 0 disables collision detection and 5 is the most sensitive. If it is not set, the value configured in the software before entering TCP mode applies.
  - Queue commands run after the preceding motions. Immediate commands do not wait.
- Older CR firmware (V3 protocol, as documented by Trossen) — [Trossen Robotics Dobot CR TCP/IP docs](https://docs.trossenrobotics.com/dobot_cr_cobots_docs/tcpip_protocol/functions.html):
  - `ServoJ(J1..J6)` and `ServoP(X,Y,Z,A,B,C)` take no t/aheadtime/gain.
  - They are sent on motion port 30003.
  - "This command can be interrupted by another ServoJ command".
  - The cycle interval should be "at least 30ms".
  - Ports 30004–30006 require controller 3.5.2 or later.
- A Dobot staff reply (relayed via search snippet, MG400 forum thread) said ServoJ interrupts the current motion and replans toward the new target, so sending too densely causes constant replanning and stalled motion. ServoJ is not supported on the MG400 — [Dobot forum thread 6259](https://forum.dobot.cc/t/does-servoj-command-work-frequency/6259). The page itself could not be fetched (certificate error), so this is unverified.
- UR `servoj(q, a, v, t=0.002, lookahead_time=0.1, gain=300)` — [UR Script Manual SW5.24 servoj](https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-scriptmanual/all_scripts/servoj_qavt0-008lookahead_time.htm):
  - It is for "online realtime control of joint positions".
  - `a` and `v` are not used.
  - `t` blocks for that duration, and sending a new setpoint every time step is preferred.
  - `lookahead_time` (0.03–0.2 s) "projects the current position forward in time with the current velocity".
  - `gain` ranges 100–2000.
  - High gain or short lookahead can cause instability and vibration, "especially with noisy targets or infrequent updates".
  - Cartesian servoing is done with `get_inverse_kin(x)` followed by `servoj(q, lookahead_time=0.05, gain=500)`.
- The ur_rtde servoJ example uses dt = 1/500 s, lookahead = 0.1 and gain = 800 in a `waitPeriod` loop, after a regular moveJ to the start pose. The default RTDE frequency is 500 Hz on e-Series and 125 Hz on CB-series. In servoL, speed and acceleration are "NOT used in current version" — [ur_rtde servoJ example](https://sdurobotics.gitlab.io/ur_rtde/pages/examples/high_frequency_servoing/servoj_example.html); [ur_rtde API](https://sdurobotics.gitlab.io/ur_rtde/api/api.html)
- RTDE output can run at the 500 Hz control rate. If the controller lacks resources, it skips packets and sends only the latest data — [UR RTDE guide](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html)

### Inferences
- Comparison:

  | Aspect | Dobot CR | UR |
  |---|---|---|
  | Command rate | 33 Hz | 500 Hz |
  | Effect | Each ServoJ is a short replan, so commands must already be smooth | Fine-grained set-points |
  | Lookahead | Unitless `aheadtime` (20–100) | Seconds (0.03–0.2) |
  | Gain | 200–1000 | 100–2000 |

  The two parameter scales are not directly comparable.
- CR10 recipe (inference; verify on hardware):
  1. Fixed 30 ms timer.
  2. Pass the filtered target through a jerk-limited generator at 30 ms steps (Ruckig).
  3. Send ServoJ with `t` ≈ 0.03 to match the send period. The doc's own example pairs `t=0.1` with a 30 ms call interval; `t` is the time to reach that point, so a larger t adds lag and smoothing.
  4. Start with aheadtime 50 and gain 500. Lower aheadtime or raise gain only if the lag is unacceptable and no jitter appears.
- Pitfalls:
  - Sending faster than the controller expects can cause stalling or replanning, according to the forum report.
  - A large per-step jump can be infeasible because of the speed limit. `t` will not be met and the robot lags.
  - Units are degrees and mm, not radians.
  - The `SpeedFactor`/global rate does not scale ServoJ, so speed limits must be enforced on the host.
  - Get firmware right: the V3 form has no parameters and uses port 30003, while V4 has parameters and uses port 29999.
- Read joint feedback from port 30004 (8 ms) to close an outer monitoring loop. Compare commanded and actual position, and abort if the error grows beyond a threshold.

### Gaps
- Dobot does not document the exact semantics of `aheadtime` units, the internal servo rate, or the transport delay from command to motion. These must be measured.
- I found no official Dobot guidance on behaviour when ServoJ commands stop arriving: whether the robot holds or decelerates, and whether there is a watchdog. Assume it simply holds at the last target and implement a host-side watchdog.
- No public GitHub issue with ServoJ jitter measurements for the CR10 was found.

## Q4. Safety: workspace limits, speed and separation, collision checking, enable switch, standards, and stream dropout

### Takeaway
ISO 10218-1/-2:2025 now absorb ISO/TS 15066 (mostly into Part 2). Reduced speed, monitored at 250 mm/s or less, remains the norm when a person shares the space. Teleoperated manipulators can fall outside the industrial-robot scope, so the classification needs a documented risk assessment. In practice, layer several protections:
- A hardware E-stop and a three-position enabling device held by the operator.
- Host-side workspace boxes, plus speed, acceleration and jerk caps.
- The robot's own collision detection (`SetCollisionLevel`).
- MoveIt planning-scene collision scaling.
- A watchdog that stops motion when the IMU stream is late (for example, more than 100 ms, matching MoveIt Servo's default).

### Cited Findings
- The ISO 10218-1:2025 foreword says ISO/TS 15066:2016 content was added to the 10218 series. Its introduction says most collaborative-application requirements went into ISO 10218-2:2025 — [ISO OBP ISO 10218-1:2025](https://www.iso.org/obp/ui/en/#!iso:std:73933:en); [A3 Updated ISO 10218 FAQ](https://www.automate.org/robotics/blogs/updated-iso-10218-faq); [ANSI blog](https://blog.ansi.org/ansi/iso-10218-1-2025-robots-and-robotic-devices-safety/)
- Reduced speed is a formal term in the 2025 editions and shall be no greater than 250 mm/s (PLd/SIL 2) when an operator shares the space in manual mode. These are secondary summaries, not the standard text — [evsint summary](https://www.evsint.com/collaborative-robot-safety-standards-2026-iso-10218-2025-ts-15066/); [Safetics ISO 10218-2:2025 guide](https://en.doc.safetics.io/insight-10218-2/)
- The ISO 10218-1:2025 scope lists tele-operated manipulators among non-industrial robot applications, which affects whether the standard applies directly. This is per search-result reading of the ISO OBP page and needs checking against the scope clause — [ISO OBP](https://www.iso.org/obp/ui/en/#!iso:std:73933:en)
- The three-position enabling device requirement is in ISO 10218-1:2011 Annex C, as cited in a patent. Vendors market three-position switches for ISO 10218 compliance. I found no quote of the 2025 enabling-device clause — [USPTO patent 11667037](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11667037); [IDEC technical article](https://mktg.idec.com/en-us/iso-10218-idec-technical-article)
- Dobot controller safety primitives — [Dobot TCP/IP guide v4.6.6](https://a.storyblok.com/f/298593/x/ce7fbaa06a/dobot-tcp_ip-remote-control-interface-guide-v4-6-6_20260527_en.pdf):
  - `EmergencyStop(mode)`: the robot is disabled and alarmed, and the E-stop must be released and the alarm cleared before re-enabling.
  - `SetCollisionLevel(level)`: 0 is off, 1–5 increases sensitivity.
  - `SetBackDistance`: collision back-off distance.
  - `SetSafeSkin`.
  - `Stop`/`Pause`.
  - ServoJ/ServoP are "constrained by the speed limit".
- MoveIt Servo safety-relevant defaults — [servo_parameters.yaml](https://raw.githubusercontent.com/moveit/moveit2/main/moveit_ros/moveit_servo/config/servo_parameters.yaml):
  - `incoming_command_timeout` 0.1 s ("Commands will be discarded if they are older than the timeout").
  - Self- and scene-collision deceleration at 0.01 m and 0.02 m.
  - Collision checks at 10 Hz.
  - Joint limit margins of 0.1 rad.
  - Singularity hard stop at condition number 30.

### Inferences
- Recommended layered architecture (inference):
  1. **Hardware layer:** the CR10's physical E-stop, plus a three-position enabling switch held by the operator.
     - Wire it to safety I/O if possible; otherwise into the host loop as a minimum.
     - Releasing it or squeezing to the panic position triggers `Stop`/`Pause`, and `EmergencyStop` where appropriate.
  2. **Host gating:** motion only while enable is pressed and the IMU is calibrated. Use a clutch (re-index) so that re-engaging does not cause a jump: the offset between the human and robot pose is re-captured at each engage.
  3. **Workspace:** an axis-aligned Cartesian box or set of planes (table, walls, operator side), plus a tool-orientation cone.
     - Clamp the target, don't reject it, so the robot slides along the boundary.
     - Also add joint-limit margins.
  4. **Kinematic limits:** per-joint velocity, acceleration and jerk caps, plus a TCP speed cap. Use 250 mm/s when people are nearby, in line with the reduced-speed convention.
  5. **Collision:** keep the robot's `SetCollisionLevel` at 3 or more. Add MoveIt planning-scene checks in the twin (see Q5).
  6. **Watchdog on IMU stream loss:**
     - If no fresh sample arrives within about 100 ms (about 3 ServoJ periods), stop updating the target, hold the last pose, and ramp down. Then `Stop`/`Pause`.
     - Require an explicit re-enable rather than resuming automatically.
     - Also monitor IMU quality: magnetometer disturbance, gyro saturation, and quaternion jumps above a threshold count as faults.
  7. **Tracking-error monitor:** compare port 30004 feedback with the commanded position, and stop if the error exceeds a threshold.
- An ISO/TS 15066 speed-and-separation calculation is only meaningful with separation sensing such as a laser scanner or vision. With IMU-only input, rely on power and force limiting (the cobot's collision detection) plus reduced speed.

### Gaps
- I did not access the normative text of ISO 10218-1/-2:2025, so clause numbers and exact enabling-device wording are unverified.
- I found no standard or paper that specifically defines safe-stop behaviour for wearable-IMU dropout. The watchdog values are engineering inferences anchored on the MoveIt Servo default.
- The CR10's safety-rated functions (safety I/O, safety-rated speed monitoring) were not researched in detail.

## Q5. Shared control and assistance: virtual fixtures, feedback, digital-twin preview

### Takeaway
Virtual fixtures, meaning geometric constraints to curves, planes or orientations, and digital-twin visualisation are established ways to assist teleoperation. Digital-twin and MR systems that couple a simulated robot with haptic or visual feedback are in recent IEEE RA-L work. For this project, a straightforward pattern is to run the IMU-driven command through a MoveIt/RViz or Gazebo twin first (a "ghost" preview, plus collision and limit checks), and only forward it to the real CR10 once validated and enabled.

### Cited Findings
- Digital Twin-Driven Mixed Reality teleoperation with haptic rendering (Fan et al., IEEE RA-L 2023) combines a DT-driven MR manipulation interface with kinaesthetic and wearable tactile feedback — [IEEE 10287552](https://ieeexplore.ieee.org/document/10287552/)
- CAD-guided virtual fixtures constrain the user to a curve, surface or orientation and regulate motion anisotropically to assist multi-step manipulation — [VF Designer (ResearchGate)](https://www.researchgate.net/publication/401341208_VF_Designer_CAD-Guided_Virtual_Fixtures_for_Enhanced_Robot_Teleoperation_in_Multi-Step_Manipulation_Tasks)
- A Unity digital twin that mirrors the physical workspace, with a forward command path and a visual/position feedback path, has been used for adaptive shared control. One related system limits increment sizes to suppress operator tremor — [arXiv 2603.14388](https://arxiv.org/pdf/2603.14388); [arXiv 2503.06359](https://arxiv.org/pdf/2503.06359)
- A phone's IMU and camera have been used to estimate a Cartesian pose that commands a manipulator's tool pose over 5G — [arXiv 2405.07128](https://arxiv.org/html/2405.07128v1)
- MoveIt Servo's collision checking uses the planning scene to decelerate near obstacles. It is a ready-made twin-based guard — [MoveIt Servo tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)

### Inferences
- Suggested modes for the cr10twin project:
  - **(a) Preview-only:** the IMU drives a ghost robot in RViz/Gazebo, and the real robot is idle.
  - **(b) Shadow:** the twin runs the full pipeline (IK, limits, collision) and the real robot follows only while enable is held.
  - **(c) Assisted:** virtual fixtures project the target onto a task plane, line or orientation lock (for example, keeping the tool vertical). The robot mirrors only selected DOFs or uses scaled motion.
- Visual feedback in the GUI helps the operator anticipate clamping and slowdowns:
  - Distance to workspace limits.
  - Singularity condition number.
  - Tracking error.
  - Watchdog state.

### Gaps
- I found no peer-reviewed study quantifying how much digital-twin preview improves safety or performance specifically for wearable-IMU arm teleoperation.
- Rosenberg's original virtual-fixtures work (1993) predates the 2015–2026 window and was not retrieved.
