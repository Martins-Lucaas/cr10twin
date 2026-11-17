# Mapping strategies from IMU-tracked human arm motion to a 6-DOF industrial/collaborative arm

Scope: (1) Cartesian TCP-only mapping with robot IK, (2) joint-to-joint retargeting, (3) hybrid TCP + elbow/arm-posture approaches, with evidence on metrics, relative vs absolute mapping, clutching, frame alignment, handling different kinematics, and gripper control.

Note on evidence quality: full-text access failed for some key sources (MDPI and PMC pages returned 403 or CAPTCHA, e.g. Škulj et al. 2021). Numbers below come only from texts I could actually read (arXiv HTML/PDF, Frontiers, the Rakita HRI'17 preprint PDF). Where I only saw an abstract or a secondary summary, I say so.

## 1. Which mapping do IMU teleoperation papers use, what metrics do they report, and are there direct comparisons?

### Takeaway
Almost all IMU or wearable teleoperation of industrial and collaborative arms uses Cartesian (task-space) TCP mapping. The usual pipeline is: IMU orientations go through a human-arm forward-kinematics model to a wrist/hand pose, that pose becomes a TCP target, and the robot solves IK. Joint-to-joint mapping shows up mainly for anthropomorphic arms or humanoids. The only controlled head-to-head comparison I found (Wang et al., ICRA 2021, N=6, a 4-DOF anthropomorphic arm, not IMU-based) found similar final performance for both mappings, with faster learning under task-space mapping. Reported quantitative outcomes are sparse and come from small samples. NASA-TLX and SUS appear in only a few studies.

### Cited Findings
**Direct joint-space vs task-space comparison**
- Wang, Murphy, Kenney, Ramos, "A Comparison Between Joint Space and Task Space Mappings for Dynamic Teleoperation of an Anthropomorphic Robotic Arm in Reaction Tests" (ICRA 2021). Setup: 6 subjects, a 4-DOF anthropomorphic arm, a motion-capture linkage (not IMU), and BATAK-style reaction tests. Results: all subjects became proficient with both mappings, even though human and robot differ in size and link-length ratio, and stable mean reaction times were "close" between the mappings in the reaction tests. "Most subjects developed their teleoperation proficiencies more quickly with the task space mapping than with the joint space mapping after similar amounts of practice." — [arXiv 2011.02508](https://arxiv.org/abs/2011.02508)
- In the same paper, the "task space" mapping is itself a hybrid. Shoulder joints θ1–θ2 are mapped joint-to-joint (θcmd1,2 = θH1,2). The elbow/forearm DOFs are found by scaling the hand position in the arm's workplane by the arm-length ratio L_R/L_H and solving planar 2-DoF IK. The mappings ran at 1 kHz. — [arXiv 2011.02508](https://arxiv.org/pdf/2011.02508)
- The prior work that Wang et al. cite found task-space mapping faster and more accurate than joint-space mapping for a robot-steered surgical needle. — [arXiv 2011.02508](https://arxiv.org/abs/2011.02508)
- Subjects reported that fear of damaging the robot was the main thing limiting speed, and 3 of 6 also named perception difficulty. Mapping preference depended on order (subjects tended to prefer the second mapping they used), so subjective preference did not separate the mappings. — [arXiv 2011.02508](https://arxiv.org/pdf/2011.02508)

**Cartesian mimicry with optimization IK on a 6-DOF cobot (UR5): the strongest user-study evidence**
- Rakita, Mutlu, Gleicher, "A Motion Retargeting Method for Effective Mimicry-based Teleoperation of Robot Arms" (HRI 2017). Setup: UR5 with a Robotiq 85 gripper, hand pose from an HTC Vive controller (not an IMU), N=24 novices (11 M/13 F, aged 18–46), within-subjects comparison of relaxed-mimicry against a Geomagic Touch 6-DOF stylus and the UR5 teach-pendant touchscreen. Aggregate task time (mean ± SD): touchscreen 433.79 ± 102.5 s, stylus 296.38 ± 145.68 s, mimicry 124.75 ± 47.98 s. Per task, mimicry vs stylus vs touchscreen: bottles 53.33/110.20/160.75 s, plates 27.67/68.41/133.46 s, legos 43.75/117.75/139.58 s. On 7-point ratings, ease of use was 5.67 (mimicry), 4.51 (stylus), 5.63 (touch); satisfaction 6.52/5.97/5.96; enjoyment 5.99/5.13/4.81. — [Rakita et al. HRI'17 preprint](https://graphics.cs.wisc.edu/Papers/2017/RMG17/hri17-preprint.pdf)
- Measured end-to-end latency, from hand motion to visible robot motion, was 130–140 ms. — [Rakita et al. HRI'17](https://graphics.cs.wisc.edu/Papers/2017/RMG17/hri17-preprint.pdf)

**Wearable IMU (+sEMG) Cartesian incremental interface vs a haptic device**
- Jia, Zhou, Zhang, "An Agile Large-Workspace Teleoperation Interface Based on Human Arm Motion and Force Estimation" (arXiv 2410.15414, 2024; listed as 2025 by Portsmouth). Setup: two MYO armbands (IMU + 8-ch sEMG) compared with an Omega 7, 7 subjects × 3 tasks × 10 trials. Completion time, Omega 7 vs armband: free motion 10.4 vs 11.3 s, handover 17.9 vs 16.6 s, pick-and-place 19.4 vs 20.8 s. Success rate: free motion 84% vs 93%, handover 85% vs 89%, pick-and-place 83% vs 82%. The text says the wearable had lower NASA-TLX workload overall, but the numbers appear only in a figure. — [arXiv 2410.15414](https://arxiv.org/html/2410.15414v1)

**IMU wearable vs a surgical master (da Vinci MTM)**
- Fu, Azimi, Kazanzides (JHU, arXiv 2103.08119). Setup: two LPMS-B2 IMUs (upper arm and forearm), a ball-joint arm FK model giving 5 DOF (wrist rotation not captured), absolute pose relative to a fixed shoulder, no clutch, a virtual ring-on-wire task, only 2 users. Straight-wire completion time, MTM vs IMU: User1 6.4 vs 7.6 s, User2 9.7 vs 9.2 s. S-wire: 18.0 vs 24.7 s and 22.5 vs 25.8 s. Mean position error ranged 7.2–20.8 mm with IMU and 9.6–13.9 mm with MTM. Mean orientation error reached 29.1° with IMU. Link-length calibration error was 9.7% ± 9.4% (upper arm) and 7.8% ± 5.7% (forearm). — [arXiv 2103.08119](https://arxiv.org/html/2103.08119)

**IMU-only teleoperation of an industrial cobot**
- Škulj, Vrabič, Podržaj, "A Wearable IMU System for Flexible Teleoperation of a Collaborative Industrial Robot" (Sensors 21(17):5871, 2021). IMUs only, measuring torso, upper-arm and forearm orientation as quaternions. A calibration step aligns the sensor, operator and robot frames and accounts for where the operator stands relative to the robot. Demonstrated on a workpiece-insertion task and compared with a standard robot controller. The abstract claims the system is intuitive and robust but gives no numbers, and I could not open the full text. — [Sensors 2021](https://doi.org/10.3390/s21175871); [PMC8434127](https://pmc.ncbi.nlm.nih.gov/articles/PMC8434127/)
- Stan & Napoletano (Sensors 26(15):5002, 2026): three BNO055 IMUs (upper arm, forearm, hand) at 50 Hz over MQTT, driving a simulated UR10e + Robotiq 2F-85 in Gazebo/ROS 2. Segment orientations give a target end-effector pose, and a Cartesian motion controller interpolates. A button on the hand unit controls the gripper. The paper predicts NASA-TLX-style and usability ratings from end-effector kinematics: best LOSO R² was 0.787 for usability, 0.758 for frustration, 0.716 for effort. Median usability rating was 4 (IQR 3.75–4). It reports no comparison between mappings. — [PMC13469392](https://pmc.ncbi.nlm.nih.gov/articles/PMC13469392/)
- An AR manipulator study comparing a marker-based interface with a CV+IMU mapping found almost identical NASA-TLX (28.4 vs 28.6) and high SUS for both. This comes from a search summary only, not verified in the full text. — [arXiv 2602.06273](https://arxiv.org/pdf/2602.06273)
- In a 2024 Frontiers study of wearable IMU mocap vs a gamepad for a legged manipulator, the gamepad was better on simple locomotion tasks, the mocap had an edge on complex operations, and mocap performance improved with practice. This is a search summary only. — [Frontiers 2024](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1430842/full)

### Inferences
- The evidence favors TCP mapping with IK for task performance and learnability on non-anthropomorphic cobots. Rakita et al. cut task time by about 3.5× against the teach pendant and about 2.4× against a 6-DOF stylus on a UR5, but that input was a Vive tracker, not IMUs. With IMUs, drift and calibration errors (centimetres, tens of degrees) will erode precision.
- Wearable IMU interfaces reach roughly the same completion times as desktop masters (within about ±10–30%). Success rates are similar or better on gross tasks and similar or worse on fine tasks.
- No study I found compares joint-to-joint, TCP-only and hybrid mappings head-to-head with IMU input on a 6-DOF industrial arm. That is a real gap, and a possible contribution for the CR10 project.

### Gaps
- No NASA-TLX or SUS numbers comparing mapping types. Jia et al. give NASA-TLX only in a figure.
- Full quantitative results of Škulj et al. 2021 (the closest analog to an IMU-driven CR10) could not be retrieved because MDPI and PMC blocked access. The full text should be checked manually.
- The 2017 ROMAN paper on IMU-based human FK pose mapping (DOI 10.1109/ROMAN.2017.8172295) was not read.

## 2. Relative vs absolute mapping, workspace scaling, clutching, orientation conventions and frame alignment

### Takeaway
Most working systems use relative (incremental) position mapping with a clutch that re-centres the mapping, plus a scale factor. Orientation is usually mapped as a displacement from a captured initial frame, or absolutely when the frames are well aligned. Absolute position from IMU-based FK is fragile, because link-length calibration errors of about 8–10% and yaw drift on the order of tens of degrees translate directly into TCP error.

### Cited Findings
- **Rakita et al. (relative + clutch + scaling).** Hand position is measured relative to the starting configuration, so the initial hand pose corresponds to the initial robot configuration. Orientation displacements from the initial frames are copied directly, using a quaternion displacement applied in the end-effector frame (r_q = h_q * r_q0). A clutch button "disconnects positional motions… effectively re-centering the mapping". Positional displacement is scaled when the workspaces differ. The Vive trigger controls the gripper. — [HRI'17 preprint](https://graphics.cs.wisc.edu/Papers/2017/RMG17/hri17-preprint.pdf)
- **Jia et al. (purely incremental, wearable).** Position: Δp = p(t) − p(t−1), smoothed by a 10-sample moving average, then p_R(t+1) = p_R(t) + Δp_avg. Orientation: ΔQ_F = Q_F(t) ⊗ Q_F⁻¹(t−1), with SVD-based quaternion averaging over 10 samples, then Q_R(t+1) = ΔQ_avg ⊗ Q_R(t). The inertial and shoulder frames are set to match the manipulator base orientation. The paper does not mention a clutch. IMU sampling is 50 Hz while the arm runs at 1000 Hz. — [arXiv 2410.15414](https://arxiv.org/html/2410.15414v1)
- **Zhou et al. (hybrid: incremental position + absolute orientation).** Wrist/hand position is mapped incrementally with a mapping scale, and attitude is mapped absolutely. On a 7-DOF arm the elbow also gets absolute attitude mapping for obstacle avoidance. IMU yaw drift reached about 70° after 20 min because the magnetometer failed. Uncorrected, the robot arm angle deviated about 30° from the operator's and end-effector position about 0.25 m. With an LSTM-based correction these fell to at most about 10° and about 0.13 m. — [Frontiers Robot. AI 2025, 10.3389/frobt.2025.1638853](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1638853/full)
- **Fu et al. (absolute, no clutch).** Absolute wrist pose from a fixed-shoulder FK model, with no clutch (listed as future work). Accuracy was user-dependent: one novice had a 47.4% non-collision rate with IMU vs 85.8% with the MTM. — [arXiv 2103.08119](https://arxiv.org/html/2103.08119)
- **Škulj et al.** An explicit calibration procedure aligns the sensor, operator and robot coordinate frames, including operator position relative to the robot, using a base posture recorded at start-up. This was needed because IMU frames are misaligned after initialization. Abstract-level evidence only. — [Sensors 2021](https://doi.org/10.3390/s21175871)
- **Clutch-anchored relative mapping in VR surgical teleoperation.** The controller and robot poses are recorded at clutch engagement, and only the displacement afterwards is applied. The authors argue this removes the need for absolute controller-to-robot workspace alignment and reduces sensitivity to jitter. This is a controller-based system, not IMU, and comes from a search summary. — [arXiv 2609.25527](https://arxiv.org/pdf/2609.25527)
- **Workspace scaling by arm-length ratio.** Wang et al. scale the human hand position in the arm plane by L_R/L_H before IK. — [arXiv 2011.02508](https://arxiv.org/pdf/2011.02508). Jia et al. used 100× amplification for the Omega 7 baseline and no stated scaling for the wearable. — [arXiv 2410.15414](https://arxiv.org/html/2410.15414v1)
- **Drift and recalibration.** Head-worn IMU teleoperation of mobile manipulators needed frequent recalibration because of drift. — [arXiv 2603.07672](https://arxiv.org/html/2603.07672v1). Jia et al. list long-task IMU drift as their main limitation. — [arXiv 2410.15414](https://arxiv.org/html/2410.15414v1)

### Inferences
- For an IMU-driven CR10, the most defensible configuration from this evidence has five parts:
  - relative (delta) TCP position anchored at clutch engagement;
  - a deadman or clutch button, or a gesture, to engage, which also lets the operator re-index when out of reach;
  - a tunable position scale (around L_robot/L_human, or less than 1 for precision);
  - orientation applied as a displacement relative to the engagement frame, to tolerate yaw drift and imperfect heading alignment;
  - a one-shot frame-alignment calibration (a known posture facing the robot) to fix the heading between the human and robot base frames.
- Absolute position mapping needs accurate link lengths and drift-free yaw. IMU-only setups rarely achieve this; the evidence shows about 10% link error and tens of degrees of yaw drift.

### Gaps
- I found no controlled study that isolates relative vs absolute or clutch vs no-clutch with IMU input and reports time or accuracy.
- No specific evidence on gesture-based engagement compared with a button-based deadman for IMU teleoperation.

## 3. Handling kinematic dissimilarity: optimization-based retargeting, IK with posture objectives, and hybrid elbow/swivel matching (and what suits a non-redundant 6-DOF arm)

### Takeaway
Optimization-based IK, which treats the hand pose as a weighted soft goal alongside smoothness, joint-limit, singularity and velocity terms, is the established way to handle the human–robot mismatch on a 6-DOF cobot (RelaxedIK on the UR5). Elbow swivel or arm-angle matching through nullspace needs redundancy, so it applies directly to 7-DOF arms (Franka, KUKA iiwa). On a non-redundant 6-DOF arm (CR10, UR) there is no nullspace once the full 6-DOF TCP pose is tracked. Elbow posture can then only influence the choice among discrete IK branches (elbow up/down, shoulder left/right, wrist flip), or be traded off against TCP accuracy in a weighted, relaxed objective.

### Cited Findings
- **RelaxedIK.** IK is framed as a weighted optimization. End-effector position and orientation goals are objectives rather than hard constraints. Minimal joint velocity, smoothness and singularity avoidance are additional objectives. The weights adapt at each step ("importance-based IK"), e.g. favouring smooth motion over exact orientation during fast large motions. Each solve starts from the previous solution, which avoids configuration jumps. — [Rakita et al. HRI'17](https://graphics.cs.wisc.edu/Papers/2017/RMG17/hri17-preprint.pdf). The open-source solver includes UR5 and other robot models and reportedly runs at more than 3000 Hz for single arms. — [GitHub uwgraphics/relaxed_ik](https://github.com/uwgraphics/relaxed_ik)
- **Rakita et al. on the mismatch problem.** The UR5 has roughly the workspace of a human arm but very different kinematics. Small hand motions can require large joint motions, and joint and velocity limits make smooth tracking difficult. — [HRI'17 preprint](https://graphics.cs.wisc.edu/Papers/2017/RMG17/hri17-preprint.pdf)
- **Joint-to-joint when the morphology differs.** One-to-one joint-angle mapping was judged infeasible for the Unitree G1 humanoid because link proportions, DOF and joint ranges differ. Joints were instead mapped to functional counterparts through geometric projection and clipped to hardware limits. — [arXiv 2605.12347](https://arxiv.org/pdf/2605.12347). Open Arms notes that joint replication is feasible only when the robot is anthropomorphic, and chose hand tracking plus IK. — [arXiv 2205.12992](https://arxiv.org/pdf/2205.12992)
- **Hybrid joint/task mapping on a 4-DOF arm.** Shoulder DOFs are mapped joint-to-joint, and the remaining DOFs come from scaled task-space IK in the arm plane. Users learned this faster than pure joint mapping. — [arXiv 2011.02508](https://arxiv.org/pdf/2011.02508)
- **Arm-angle (swivel) mapping on a 7-DOF arm from IMUs.** Zhou et al. use the arm angle (the angle between the shoulder–elbow–wrist plane and a reference plane) and the global configuration parameters GC2/GC4/GC6 to resolve redundancy. The elbow attitude is mapped absolutely so the robot can avoid obstacles. Arm-angle deviation was about 30° uncorrected and at most about 10° corrected. — [Frontiers 2025](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1638853/full)
- **Swivel angle from IMUs.** It must be inferred from the upper-arm and forearm segment orientations, through geometry or forward kinematics, because no sensor measures the elbow directly. — [WITRA, Hackster](https://www.hackster.io/vbazan/wearable-interface-for-teleoperation-of-robot-arms-witra-1405ab). HL-IK (humanoid arm) aligns the robot elbow with the inferred human elbow, but its swivel-based IK was evaluated only in simulation. — [arXiv 2509.20263](https://arxiv.org/pdf/2509.20263)
- **Elbow + wrist keypoint tracking for a 6-DOF arm.** A low-cost bimanual 6-DOF arm tracks human elbow and wrist positions with differential IK (PINK), minimizing distance at both points rather than copying joint angles. — [arXiv 2502.16908](https://arxiv.org/pdf/2502.16908) (search summary). Riemannian retargeting expresses hand poses in the shoulder frame scaled by total arm length to absorb length differences. — [arXiv 2208.01372](https://arxiv.org/pdf/2208.01372)
- **Review on IMU upper-limb teleoperation.** The common practice maps Cartesian limb motion and solves IK by minimizing a cost under robot constraints. Some works argue that lower-priority elbow motion should be included, for operation in confined spaces. — search summary of [PMC8434127 / related review](https://pmc.ncbi.nlm.nih.gov/articles/PMC8434127) (could not open to verify which paper is meant).

### Inferences
- **For a non-redundant 6-DOF CR10, recommended baseline:** a TCP-only relative mapping, solved by a damped least-squares or relaxed optimization IK seeded from the current configuration, to avoid branch jumps. Add joint-limit and singularity penalties and velocity/acceleration limits.
- **"Hybrid" posture on a 6-DOF arm is possible in two limited forms:**
  - Choose the IK branch (elbow up/down, etc.) that best matches the human elbow, then hold it.
  - Relax TCP orientation (for example, track position plus tool z-axis with yaw free, giving a 5-DOF task). That leaves 1 DOF of redundancy, which can then serve an elbow/swivel posture objective.
- **Pure joint-to-joint mapping suits the CR10 poorly.** Its UR-like joint layout (offset shoulder/elbow, three-axis non-spherical wrist) is not anthropomorphic, so mapping human shoulder and elbow angles to J1–J6 gives a TCP that does not follow the hand. The literature consistently reserves joint mapping for anthropomorphic or humanoid arms.

### Gaps
- No user study I found evaluates elbow-posture objectives on a non-redundant 6-DOF arm with IMU input.
- I did not retrieve a quantitative comparison of RelaxedIK-style vs analytical IK in IMU teleoperation.

## 4. Gripper/hand control combined with arm IMUs (IMU, EMG, data glove)

### Takeaway
The simplest reliable options are a button or trigger on a hand-held unit (Rakita, Stan & Napoletano) or binary sEMG grasp detection from a forearm armband (Jia et al.: logistic regression, open/close). Gesture sets from the Myo (fist, pinch, wave) are common in low-cost demos but lack rigorous user metrics.

### Cited Findings
- **Jia et al.** 8-ch forearm sEMG, with MAV, WL and RMS features per channel (24-D vector), fed to logistic regression (threshold 0.5), giving a binary contraction/relaxation output that closes or opens the gripper. Classification accuracy is not reported. Pick-and-place success was 82% vs 83% for the Omega 7. — [arXiv 2410.15414](https://arxiv.org/html/2410.15414v1)
- **Stan & Napoletano 2026.** A push-button on the hand IMU unit drives a Robotiq 2F-85 (simulated UR10e). — [PMC13469392](https://pmc.ncbi.nlm.nih.gov/articles/PMC13469392/)
- **Rakita et al. 2017.** The Vive trigger controls a Robotiq 85, and a separate button is the clutch. — [HRI'17 preprint](https://graphics.cs.wisc.edu/Papers/2017/RMG17/hri17-preprint.pdf)
- **Myo default gestures.** Pinch, fist, open, wave in and wave out were used to command a 5-DoF arm and gripper servo (IOP Conf. Ser. 2021). — [IOP 1115/012010](https://iopscience.iop.org/article/10.1088/1757-899X/1115/1/012010/pdf). Hassan et al. classified 7 sEMG gestures with k-NN (6 subjects) to navigate a 5-DoF arm. — [arXiv 1810.09929](https://arxiv.org/abs/1810.09929)
- **Myo armband vs UR10e teach pendant.** Compared on pick-and-place and timed landmark-touch tasks (Actuators 2025/2026). Numbers were not retrieved. — [doi 10.3390/act15040210](https://doi.org/10.3390/act15040210)
- **Wearable multimodal fusion (Fang et al. 2017, IJARS).** Combines IMUs with hand sensing for arm–hand teleoperation. Not read in detail. — [SAGE 10.1177/1729881417717057](https://journals.sagepub.com/doi/full/10.1177/1729881417717057)

### Inferences
- For a parallel-jaw gripper on the CR10, a binary command is enough. A button, or an sEMG grip threshold, on the same hand unit that hosts the deadman/clutch keeps the mapping simple and deterministic. Data gloves only pay off for multi-fingered hands.
- Avoid using hand IMU gestures for the gripper when the same hand IMU drives TCP orientation, because the gesture would perturb the pose command.

### Gaps
- No accuracy or latency numbers for sEMG grasp classification in Jia et al. No data-glove plus arm-IMU study with an industrial arm and user metrics was found within this budget.
