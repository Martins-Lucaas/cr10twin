# Estimating Human Arm Pose (Shoulder, Upper Arm, Forearm, Hand/Wrist) from Wearable IMUs for Robot Teleoperation

Scope note: these notes come from 17 search and fetch calls. Several primary full texts (PMC, MDPI) were behind bot checks or returned 403, so some numbers come from abstracts or search snippets. Each is marked where that applies. All numbers are as reported by the cited source.

## 1. Sensor fusion choices and reported orientation / wrist-position accuracy

### Takeaway
When the filters are tuned well, the standard fusion filters (Madgwick, Mahony, KF/EKF/UKF, complementary) reach similar accuracy on human motion, about 4–7° absolute orientation error. Tuning, rotation speed, sensor hardware and the magnetic environment affect accuracy more than which filter is chosen. Commercial engines (Xsens, Noitom) reach about 2–5° in good conditions. They degrade near ferromagnetic material unless a disturbance model is applied.

### Cited Findings
- Caruso et al. (Sensors 2021) compared 10 sensor-fusion algorithms (SFAs) across 9 scenarios: 3 rotation rates × 3 commercial MIMUs. With optimal tuning there were "no statistically significant differences" among the SFAs, and absolute errors ranged from 3.8° to 7.1°. Parameters had to be tuned per algorithm and per scenario. With non-specific parameters, errors were comparable to those obtained with default values. Higher rotation rates significantly worsened performance, and the MIMU model also affected error. Source: abstract/snippet. — [Caruso et al. 2021 (repository PDF)](https://iris.polito.it/bitstream/11583/2898632/1/Caruso%20et%20al_2021_a.pdf)
- The original Madgwick gradient-descent filter reported <0.8° static RMS error and <1.7° dynamic RMS error. This is "matching that of the Kalman based algorithm", but the test was on a controlled rig, not in human-limb conditions. — [Madgwick (pure.ul.ie record)](https://pure.ul.ie/en/publications/estimation-of-imu-and-marg-orientation-using-a-gradient-descent-a/)
- Rankings between Madgwick, Mahony and EKF are inconsistent across studies. A foot-mounted study found Madgwick gave better heading RMSE than Mahony, while Mahony had a shorter execution time. A quadcopter study found Mahony beat both Madgwick and EKF. — [Ludwig et al. SPIE 2018](https://web.cs.ndsu.nodak.edu/~siludwig/Publish/papers/SPIE20181.pdf); [Ludwig et al. ICUAS 2018](https://web.cs.ndsu.nodak.edu/~siludwig/Publish/papers/ICUAS2018.pdf)
- El-Gohary & McNames used a UKF with an arm kinematic model, drift models, range-of-motion (ROM) constraints and zero-velocity updates, with 2 IMUs (upper arm and forearm). Average RMS error was about 3° across all 6 shoulder/elbow angles, and the UKF was slightly better than an EKF. This was validated against a robot arm. An earlier ISB 2009 abstract reported elbow flexion/extension RMSE of 2.1° and pronation/supination RMSE of 3.1°. The 2011 version was validated against optical motion capture with correlation >0.9. — [OHSU record](https://ohsu.elsevierpure.com/en/publications/upper-limb-joint-angle-tracking-with-inertial-sensors-4); [ISB 2009 abstract](https://media.isbweb.org/images/conf/2009/data/pdf/147.pdf)
- The PNP paper (SIGGRAPH 2024) compared per-sensor orientation error in its own recording. Mean error was Xsens DOT 5.3°, Noitom PN Lab 2.0°, and PNP's custom fusion 2.4°. — [PNP, arXiv 2404.19619 (Fig. 9, supplementary)](https://arxiv.org/pdf/2404.19619)
- IRSST (manual handling) found Xsens joint angles were within an acceptable 5° threshold compared with optoelectronic cameras. The same researchers also concluded that magnetic disturbances made Xsens results less reliable. — [IRSST](https://www.irsst.qc.ca/en/headlines/id/713/manual-handling-towards-a-more-precise-understanding-of-physical-demands)
- WearMoCap (Frontiers Robot. AI 2024) used Android's built-in EKF fusion of accelerometer, gyroscope and magnetometer on a smartwatch and phone, followed by an LSTM. Wrist position RMSE was 6.79 cm in "Upper Arm" mode (watch on the wrist plus phone strapped to the upper arm). — [WearMoCap](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1478016/full)
- Wrist angle from 2 IMUs (hand relative to forearm) against 8-camera optical capture, single subject: RMS deviation of 11.4° (flexion/extension) and 6.3° (ulnar/radial deviation) in a cluster task, and 10.1° / 4.8° in material handling. The authors call these pilot data. — [CDC/NIOSH pilot](https://stacks.cdc.gov/view/cdc/215807)
- Six-sensor system validation: elbow flexion was accurate to within 8.2 ± 2.8°, and the IMU underestimated large angles. Source: search snippet. — [PMC5459676](https://pmc.ncbi.nlm.nih.gov/articles/PMC5459676)

### Inferences
- For a teleoperation build, the choice between Madgwick, Mahony and EKF matters less than tuning the gain/β for the expected motion speed and handling heading (see section 4). A realistic orientation budget per segment is about 3–7°.
- At the end of a roughly 60 cm arm chain, 5° of segment orientation error corresponds to roughly 3–5 cm of hand position error. This matches the 3.5–7 cm wrist errors reported in section 2. The figure is my geometric estimate and not taken from a source.

### Gaps
- I did not retrieve the per-algorithm RMSE table from Caruso et al. 2021.
- I found no head-to-head peer-reviewed comparison of Madgwick, Mahony and EKF specifically for upper-limb teleoperation with wrist position as the output.

## 2. Kinematic arm models (chained segments, fixed bone lengths, 7-DOF, constraints) for wrist/hand position; error in cm

### Takeaway
The dominant non-learning approach is forward kinematics. It chains segment orientations, either from a torso or shoulder anchor or from a hip origin, through measured or default bone lengths, often with joint constraints applied in a UKF/EKF. Wrist position errors are typically about 3.5 cm on average (Filippeschi et al. comparison of 5 methods against Vicon) and about 6.8 cm RMSE for consumer-device or learned variants. The main error sources are segment-length errors and calibration/heading errors.

### Cited Findings
- Filippeschi et al. (Sensors 2017) is a survey plus an experimental comparison of 5 IMU arm-tracking techniques against Vicon. "All but one of the selected models perform similarly (about 35 mm average position estimation error)". They list knowledge of human parameters (arm length) as an error source, alongside drift, magnetic disturbance, calibration and fast motion. Source: abstract; I did not get the per-joint table. — [Filippeschi et al. 2017, PMC5492902](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5492902/)
- A Filippeschi/Sant'Anna group method uses 3 UKFs to estimate joint angles, IMU poses and limb lengths. Validated with optical motion capture, it "improves the wrist position estimation". I did not find the numeric value. — [IRIS SSSA 11382/521549](https://www.iris.santannapisa.it/handle/11382/521549)
- WearMoCap computes forward kinematics from the hip origin with default upper-arm length 26 cm, forearm length 22 cm, and a fixed shoulder offset from the hip (X −17.01, Y 43.1, Z −0.67 cm). The authors list atypical limb lengths as a limitation. Reported wrist RMSE was 6.79 cm (Upper Arm mode). In robot tasks, the deviation from OptiTrack-driven control was handover +1.9 ± 3.7 cm (Upper Arm), +2.2 ± 3.6 cm (Pocket) and +4.5 ± 9.7 cm (Watch Only), and teleoperation (Pocket) +1.8 ± 6.7 cm. In the Watch Only ablation, removing global orientation raised hand error from 10.82 to 19.26 cm, the largest effect of any input. — [WearMoCap](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1478016/full)
- Prior smartwatch-only wrist errors quoted in WearMoCap: Liu et al. 2023, 10.93 cm MAE; Wei et al. 2021, 8.5 cm (two-IMU variant). — [WearMoCap](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1478016/full)
- El-Gohary & McNames modelled the arm as a robot-like kinematic chain in a UKF, with ROM limits and zero-velocity updates, and got about 3° RMS joint error (see section 1). — [OHSU record](https://ohsu.elsevierpure.com/en/publications/upper-limb-joint-angle-tracking-with-inertial-sensors-4)
- Teleoperation mapping used by several IMU systems: forward kinematics of the human arm gives an (x, y, z) hand position, which then feeds the robot's inverse kinematics. The 7-DOF redundancy of the human arm (elbow swivel) is noted as a complication. — [arXiv 2103.08119](https://arxiv.org/pdf/2103.08119v2)
- A PSO–ML-LSTM paper (Frontiers Robot. AI 2025) uses 2 IMUs (upper arm and forearm) plus a handheld handle, a DH virtual human arm, and a 7-DOF manipulator. Uncorrected IMU drift gave about 30° arm-angle deviation and about 0.25 m end-effector deviation. With KF plus a PSO-tuned ML-LSTM yaw estimator, these dropped to 10° and 0.13 m. Caveat: the paper has internal inconsistencies (RMSE reported as both 7.27 and 8.31). — [Frontiers Robot. AI 2025](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1638853/full)
- Škulj et al. (Sensors 2021) built an industrial cobot teleoperation system using IMUs only on the upper body. Calibration aligns the IMU, operator and robot frames, and relative hand motions are mapped to end-effector motion while accounting for the operator's orientation to the robot. They tested a workpiece insertion task and judged it "intuitive" and "robust enough". I could not retrieve quantitative position error because PMC was blocked. — [Škulj et al. 2021, PMC8434127](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8434127/); [DOAJ](https://doaj.org/article/f909dc7b855e4545beff29a8a6bc18bf)
- Another IMU teleoperation study argues that the operator's visual feedback loop tolerates drift, because the operator corrects the remote end-effector by sight. It includes an arm link-length calibration method. — [par.nsf.gov 10617259](https://par.nsf.gov/servlets/purl/10617259)

### Inferences
- For absolute wrist position from orientation-only IMUs, expect about 3.5–7 cm error. Relative (incremental) hand-motion mapping, as in Škulj, plus operator visual feedback hides much of this in closed-loop teleoperation. The WearMoCap robot-task deviations of about 2 cm against mocap-driven control support this.
- Measuring the subject's actual segment lengths is a cheap improvement over default lengths.

### Gaps
- I could not retrieve per-method cm tables (Filippeschi 2017) or Škulj 2021 accuracy numbers.
- I could not confirm the Joukov et al. 2017 upper-arm IMU numbers.

## 3. Sensor-to-segment calibration (N-pose/T-pose, functional, heading reset) and its effect on error

### Takeaway
Static-pose (N/T-pose) and functional calibrations give similar joint-angle accuracy, about 5–10°. Operator rigor matters more than which method is used. Calibration error is a first-order contributor even for learned methods: sparse-IMU networks lose about 0.5 cm of joint position error when calibration quality drops.

### Cited Findings
- Bouvier et al. (Sensors 2015) compared 3 sensor-to-segment calibrations for wrist, elbow and shoulder angles: aligning with the sensor's technical axes, a static pose, and functional movements. The design was 10 subjects × 5 sessions × 3 operators. Trueness and precision were good, at 5–10°, with only small discrepancies between calibrations. Accuracy "seems to be more dependent on the level of rigor of the experimental procedure" than on the calibration choice. — [Bouvier et al. 2015, PMC4570347](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4570347/)
- The PNP paper explains that sparse-IMU methods calibrate with a T-pose to estimate the IMU-to-world and bone-to-sensor matrices. Because subjects "cannot perform a perfect T-pose", calibration error results. On TotalCapture, the "Official" calibration (higher error) compared with "DIP" calibration (lower error) changed joint position error as follows: TransPose 7.10 → 6.55 cm, PIP 6.22 → 5.61 cm, PNP 4.89 → 4.74 cm. — [PNP, arXiv 2404.19619](https://arxiv.org/pdf/2404.19619)
- WearMoCap calibration: hold the arm parallel to the hip/chest for about 1 s (Watch/Pocket modes), or 2 poses including arm stretched forward (Upper Arm mode, 2–3 s). About 10% of sessions longer than 15 min needed recalibration because of yaw drift. Sensor slippage after calibration degrades accuracy. — [WearMoCap](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1478016/full)
- Reviews: Fang et al. 2023 (Sensors, 52 studies) reviewed IMU-to-joint-angle methods for the scapulothoracic, humerothoracic, glenohumeral and elbow joints against optoelectronic systems. Another review says upper-limb IMU joint-angle accuracy "remains poorly understood" and depends on sensor fusion and sensor-to-segment alignment. A 2025 review lists "non-standard calibration procedures" and magnetometer vulnerability as persistent issues. Source: snippets only (MDPI returned 403). — [Fang et al. 2023](https://www.mdpi.com/1424-8220/23/14/6535); [PMC10386307](https://pmc.ncbi.nlm.nih.gov/articles/PMC10386307); [MDPI JSAN 2025](https://www.mdpi.com/2224-2708/15/1/8)
- Škulj et al.'s calibration explicitly aligns the coordinate frames of all IMUs, the operator and the robot. This is the teleoperation-specific extra step. — [Škulj et al. 2021](https://doaj.org/article/f909dc7b855e4545beff29a8a6bc18bf)

### Inferences
- For teleoperation, a short static N-pose or T-pose plus a "point at the robot" or "arm forward" pose to fix heading relative to the robot frame is sufficient and standard. Re-running it (a heading reset) periodically is the practical drift fix.

### Gaps
- I did not obtain quantitative pooled numbers from the Fang 2023 review.
- I found no source that directly quantifies the effect on cm-level wrist error of functional versus static calibration.

## 4. Yaw/heading drift and magnetic disturbance near robots/steel; mitigation

### Takeaway
Yaw is the weak axis. Accelerometers correct only roll and pitch, and magnetometers are corrupted by steel and motors. Reported drift reaches about 70° over 20 min with a malfunctioning magnetometer, and up to about 50° instantaneous error near large metal objects without compensation. Mitigations include magnetometer-free fusion with periodic zero-point or heading resets, kinematic/joint constraints between adjacent segments, magnetic disturbance models, relative orientation to a torso reference, learned drift correction, and relying on the operator's visual feedback.

### Cited Findings
- Without a reliable magnetic field, filters fall back to gyroscope and accelerometer data, and heading error grows steadily. Schiefer et al. introduced "zero points" (ZPs) that give heading and gyroscope-bias information, with bidirectional computation. In a 40-min box-handling study (7 IMUs, 8 subjects, no magnetometer, ZPs on average every 1.1 min), mean RMSE was 1.7–7.6° for roll/pitch and 3.5–15.0° for heading. The worst case was forearm heading, with 95% limits of agreement of −19.3° to 18.9°. — [Univ. Lübeck record](https://research.uni-luebeck.de/en/publications/optimization-of-inertial-sensor-based-motion-capturing-for-magnet/); [CDC stacks PDF](https://stacks.cdc.gov/view/cdc/231139/cdc_231139_DS1.pdf)
- A ferromagnetic study from Xsens/Movella authors reported instantaneous errors up to 50° near a large metal object without compensation, reduced to 3.6° RMS with a magnetic disturbance model. — [Movella/Xsens paper](https://www.movella.com/hubfs/Downloads/Research%20papers/Estimating-Body-Segment-Orientation-by-Applying-Inertial-and-Magnetic-Sensing-Near-ferromagnetic-Materials.pdf)
- Xsens technical material says heading is corrupted by steel structures, reinforced concrete and electronic equipment. Later marketing claims of "immunity" to magnetic distortion are not independently verified in what I found. — [Xsens blog](https://www.xsens.com/resources/blog/consistent-tracking-of-human-motion-using-inertial-sensing); [CG Channel 2017](https://www.cgchannel.com/?p=89357)
- A PSO–ML-LSTM teleoperation study reported about 70° cumulative yaw error after 20 min of random motion with a malfunctioning magnetometer. KF plus learned yaw correction reduced the average cumulative error to about 2.8°. — [Frontiers Robot. AI 2025](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1638853/full)
- Kinematic constraints, such as approximate 2-DoF joint models (e.g., elbow) and ROM limits, can reduce long-term heading drift by exploiting coupling between adjacent segments. Source: 2026 arXiv preprint, not peer reviewed. — [arXiv 2607.26027](https://arxiv.org/pdf/2607.26027)
- An ISMCR 2018 industrial arm-tracking study used gyroscope plus accelerometer only (no magnetometer) because of electromagnetic disturbance. It limited drift during slow or no-motion phases. — [IMEKO TC17 2018](https://imeko.org/publications/tc17-2018/IMEKO-TC17-2018-013.pdf)
- The El-Gohary & McNames UKF reduced drift with gyroscope and accelerometer bias estimation, ROM constraints and zero-velocity updates. — [OHSU record](https://ohsu.elsevierpure.com/en/publications/upper-limb-joint-angle-tracking-with-inertial-sensors-4)
- A review argues that how far speed-related and magnetic errors can be mitigated "is ultimately bound by the quality of the gyroscope". — [PMC9605636](https://pmc.ncbi.nlm.nih.gov/articles/PMC9605636)
- WearMoCap attributes drift mostly to yaw, from shifts in measured magnetic north, and handles it by recalibration (about 10% of sessions longer than 15 min). — [WearMoCap](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1478016/full)
- An editorial on wearables for HRI also cites up to 70° gyroscopic drift over 20 min under magnetic interference as a key limitation of IMU teleoperation. — [Frontiers editorial, PMC12722782](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12722782/)

### Inferences
- Near a CR10/cobot, 6-axis fusion (no magnetometer, e.g., Madgwick/Mahony IMU mode) is the safer default. Segment orientations should be expressed relative to a chest or torso IMU, so shared yaw drift cancels. Heading resets triggered by a gesture or button, plus elbow hinge constraints, keep relative drift bounded. This is a synthesis of the cited mitigations, not a single validated recipe.

### Gaps
- I found no study that quantifies magnetometer error specifically next to an operating industrial robot (motors energised and moving).
- The Applied Ergonomics 2017 Xsens magnetic-disturbance study was found only as an abstract mention, with no numbers.

## 5. Learning-based sparse-IMU pose estimation (DIP, TransPose, PIP, etc.) and use for robot control

### Takeaway
With 6 IMUs (forearms, lower legs, head, pelvis), sparse-IMU networks reach about 5–9.5 cm mean joint position error and 9–18° angular error in real time (60–90+ fps). These are full-body, so per-arm accuracy is not isolated. Use for robot control is still rare. The examples are WearMoCap (LSTM, 1–2 devices, UR5/drone), a learned yaw corrector for a 7-DOF arm, and humanoid teleoperation preprints that use commercial IMU suits rather than DIP/PIP-style networks.

### Cited Findings
- PNP paper Table 1, joint position error in cm and angular error in degrees. TotalCapture (official calibration): DIP 9.47 cm / 17.57°, TransPose 7.10 / 14.91°, TIP 6.76 / 14.45°, PIP 6.22 / 13.85°, PNP 4.89 / 11.10°. DIP-IMU dataset: DIP 7.59, TransPose 6.03, TIP 5.63, PIP 5.12, PNP 4.97 cm. Jitter (10³ m/s³) was TransPose 1.95 versus PIP 0.21. PNP runs at 60 fps on a laptop CPU (i7-12700H, no GPU). In all these methods, the IMUs sit on the leaf joints (forearms, lower legs, head) plus the root (pelvis). — [PNP, arXiv 2404.19619](https://arxiv.org/pdf/2404.19619)
- TransPose (SIGGRAPH 2021) uses 6 IMUs, runs at over 90 fps, and estimates global translation via foot-ground contact. — [TransPose, arXiv 2105.04605](https://arxiv.org/abs/2105.04605)
- PIP (CVPR 2022) uses a neural kinematics estimator plus a physics-aware optimizer with 6 IMUs. — [PIP, CVPR 2022](https://openaccess.thecvf.com/content/CVPR2022/papers/Yi_Physical_Inertial_Poser_PIP_Physics-Aware_Real-Time_Human_Motion_Tracking_From_CVPR_2022_paper.pdf)
- DIP (SIGGRAPH Asia 2018) uses a recurrent network that regresses SMPL parameters from 6 IMUs in real time. SIP (Eurographics 2017) was the offline optimization predecessor. — [DIP, arXiv 1810.04703](https://arxiv.org/pdf/1810.04703); [SIP, arXiv 1703.08014](https://arxiv.org/pdf/1703.08014)
- MobilePoser (1–3 IMUs in consumer devices) reports mean per-joint rotation error of about 17.2° (DIP), 12.8° (TransPose) and 12.1° (PIP) with 6 IMUs. Its own errors with fewer IMUs are worse, as expected. — [MobilePoser, arXiv 2504.12492](https://arxiv.org/pdf/2504.12492)
- SparsePoser notes that TransPose and PIP struggle with ambiguous poses such as crossed hands and crouching. — [SparsePoser, arXiv 2311.02191](https://arxiv.org/pdf/2311.02191)
- Robot control uses:
  - WearMoCap uses LSTM and differentiable ensemble Kalman filter models to drive a UR5 (handover, teleoperation) and a drone via ROS, at 62 Hz inference for Pocket mode. — [WearMoCap](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1478016/full)
  - Durrani & Khan (arXiv May 2026, preprint) teleoperate a whole-body Unitree G1 from a Virdyn IMU suit "without any … learning-based components". Challenges include human–robot link-length mismatch, IMU high-frequency noise and joint-limit safety. — [arXiv 2605.12347](https://arxiv.org/pdf/2605.12347)
  - TelePreview takes raw arm IMU data and outputs a 6-DoF wrist pose as the end-effector command. I found no accuracy figure. — [TelePreview, arXiv 2412.13548](https://arxiv.org/pdf/2412.13548)

### Inferences
- About 5–7 cm full-body joint error is in the same range as analytic forward-kinematics arm tracking (section 2). For arm-only teleoperation, a dedicated 3–4 IMU kinematic chain is simpler and more transparent than a full-body sparse poser. Learned components seem most useful for correcting drift or yaw, or for reducing the sensor count (1–2 devices).

### Gaps
- I did not find a peer-reviewed paper that uses DIP, TransPose or PIP directly as the input for robot-arm teleoperation with quantified end-effector error.
- I did not find arm-only (wrist) errors broken out from the full-body metrics.

## 6. How many IMUs and where

### Takeaway
The common configurations are as follows:
- 2 IMUs (upper arm and forearm) give shoulder and elbow angles and an approximate wrist position, assuming a fixed shoulder.
- 3 IMUs (adding the hand) give wrist orientation.
- A 4th IMU on the torso or chest gives a body-fixed reference, which is important for heading and for operator turning.

A single wrist device is possible with learning, but at about 8–11 cm wrist error.

### Cited Findings
- Two sensors (upper arm and forearm) with a UKF gave about 3° RMS for 6 shoulder/elbow angles. — [El-Gohary & McNames, OHSU](https://ohsu.elsevierpure.com/en/publications/upper-limb-joint-angle-tracking-with-inertial-sensors-4)
- Two IMUs (upper arm and forearm) plus a hand-held handle drove a 7-DOF robot arm. — [Frontiers Robot. AI 2025](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1638853/full)
- WearMoCap results by configuration: watch on the wrist plus phone on the upper arm was the most accurate (6.79 cm wrist RMSE). Watch plus phone in the pocket gave body orientation. Watch only was the least accurate (hand error about 10.8 cm in the ablation). Global orientation was the most important input. — [WearMoCap](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1478016/full)
- Wrist flexion and deviation need a hand IMU relative to a forearm IMU (RMS 5–11° against optical). — [CDC/NIOSH pilot](https://stacks.cdc.gov/view/cdc/215807)
- For wrist posture during wheelchair propulsion, a wrist-placed IMU estimated elbow flexion better than a mid-forearm IMU. Peak elbow flexion was still overestimated by 27.8°, showing that placement matters. Source: snippet. — [PMC6929166](https://pmc.ncbi.nlm.nih.gov/articles/PMC6929166)
- Škulj et al. used IMUs only on the upper body, accounting for the operator's orientation relative to the robot, which implies a torso reference. — [Škulj et al. 2021](https://doaj.org/article/f909dc7b855e4545beff29a8a6bc18bf)
- Sparse full-body posers use 6 IMUs (2 forearms, 2 lower legs, head, pelvis). The arms are inferred from forearm IMUs only, with no upper-arm sensor. — [PNP](https://arxiv.org/pdf/2404.19619)
- Fine hand teleoperation: an 18-IMMU data glove for arm-hand teleoperation. — [Industrial Robot 2016 (unpaywall)](https://unpaywall.org/10.1108%2FIR-07-2016-0179)

### Inferences
- For a single-arm teleoperation rig near a robot, my recommended minimum is 4 IMUs: chest/torso, upper arm, forearm, and hand. The torso IMU makes the setup robust to operator rotation and shared yaw drift, and the hand IMU gives wrist orientation for the tool. This recommendation is synthesized from the cited works, not taken from a single validated benchmark.

### Gaps
- I found no controlled study that isolates the cm-level wrist-position benefit of adding a torso IMU versus assuming a fixed shoulder.
- Latency (ms) figures for IMU-to-robot pipelines were rarely reported. Only frame rates were found: TransPose >90 fps, PNP 60 fps, WearMoCap 62 Hz inference.
