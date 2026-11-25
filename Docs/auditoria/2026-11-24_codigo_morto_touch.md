# C07 touch_pack: decisão por candidato a código morto (24/11/2026)

Entrada: `relatorios/dead_touch.json` (213 candidatos; linhas são as do relatório). Tabela gerada por script: procura o nome em todo `src/` e `sensors/` (py, yaml, xml, cfg, xacro, urdf, msg), separa definição de uso e classifica pelo primeiro uso de produção no mesmo pacote. Revisada item a item; onde a primeira ocorrência era docstring ou comentário, a evidência foi trocada à mão. As linhas de evidência são as do código em 24/11 e podem deslocar.

Conclusão: 199 mantidos, 14 removidos. O detector não enxerga callbacks ROS/Tk, métodos de mixin, alias de import, threads e tabelas de despacho.

Nomes fora da lista também removidos por ficarem órfãos: `prepare_servoj`, `halt`, `FEEDBACK_TCP_FORCE_OFFSET`, `arm_base_z`, `retract_mm`, `approach_v_min_mms`. Mantidos por só aparecerem em teste (remover exigiria apagar teste): `savgol_filter`, `wrap180`, `twist_deg`. Mantido `config/tactile_hand_controllers.yaml`: o launch não o carrega, mas `grasp_ml_pack/doc/covvi_control.md` o descreve como o conjunto em malha fechada; escolher qual vale exige Gazebo.

| Categoria | Quantidade |
|---|---|
| callback Tk (command) | 31 |
| chamada por mixin | 28 |
| callback ROS | 27 |
| thread | 27 |
| referência direta | 20 |
| callback Tk (after) | 20 |
| callback Tk (bind) | 16 |
| REMOVIDO | 13 |
| callback de biblioteca | 5 |
| timer ROS | 5 |
| callback injetado | 5 |
| injeção (tap) | 4 |
| API pública declarada | 3 |
| callback Tk (trace) | 2 |
| entry point | 1 |
| tabela de despacho | 1 |
| monkeypatch de biblioteca | 1 |
| sinal Qt | 1 |
| REMOVIDO (C07) | 1 |
| handler de sinal | 1 |
| chamada por alias | 1 |

| # | Símbolo | Origem | Categoria | Evidência | Decisão |
|---|---|---|---|---|---|
| 1 | `ModbusCrcError` | touch_pack/ft_modbus.py:77 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; achado do bloco de origem | removido |
| 2 | `_patch` | launch/tactile_cell.launch.py:150 | callback de biblioteca | `tactile_cell.launch.py:134` `re.sub(..., _patch, ...)` | mantido |
| 3 | `generate_launch_description` | launch/tactile_cell.launch.py:810 | entry point | convenção do `ros2 launch` | mantido |
| 4 | `zmin` | scripts/gen_tcp_meshes_from_step.py:95 | callback de biblioteca | `sorted(out, key=zmin)` | mantido |
| 5 | `_manual_attack` | touch_pack/explorer_attack.py:103 | chamada por mixin | tactile_explorer.py:2818 `return self._manual_attack(np.asarray(pts[0], dtype=float), cfg)` | mantido |
| 6 | `_apply_saved_plane` | touch_pack/explorer_attack.py:242 | chamada por mixin | tactile_explorer.py:2760 `return self._apply_saved_plane()` | mantido |
| 7 | `_on_load_cell` | touch_pack/force_sync_node.py:91 | callback ROS | force_sync_node.py:73 `self._on_load_cell, QOS_SENSOR)` | mantido |
| 8 | `_on_touch` | touch_pack/force_sync_node.py:98 | callback ROS | force_sync_node.py:75 `self._on_touch, QOS_SENSOR)` | mantido |
| 9 | `_tap_init` | touch_pack/ft_cmd_channel.py:32 | chamada por mixin | ft_serial.py:169 `self._tap_init()` | mantido |
| 10 | `_line_write` | touch_pack/ft_cmd_channel.py:51 | injeção (tap) | `ft_cmd_channel.py:83` `yield self._line_write, self._tap_read` | mantido |
| 11 | `_tap_read` | touch_pack/ft_cmd_channel.py:55 | injeção (tap) | `ft_cmd_channel.py:83` | mantido |
| 12 | `read_output_rate_hz` | touch_pack/ft_modbus.py:430 | tabela de despacho | `ft_modbus.py:440` | mantido |
| 13 | `_worker` | touch_pack/ft_polled.py:133 | thread | ft_polled.py:99 `target=self._worker, daemon=True, name='ft-polled')` | mantido |
| 14 | `_on_rezero` | touch_pack/ft_receiver_node.py:402 | callback ROS | ft_receiver_node.py:221 `self._on_rezero, 10)` | mantido |
| 15 | `_publish_status` | touch_pack/ft_receiver_node.py:538 | timer ROS | ft_receiver_node.py:335 `self.create_timer(1.0, self._publish_status)` | mantido |
| 16 | `_check_link` | touch_pack/ft_receiver_node.py:555 | timer ROS | ft_receiver_node.py:336 `self.create_timer(5.0, self._check_link)` | mantido |
| 17 | `_report_link_health` | touch_pack/ft_receiver_node.py:580 | timer ROS | ft_receiver_node.py:337 `self.create_timer(10.0, self._report_link_health)` | mantido |
| 18 | `_on_command` | touch_pack/ft_receiver_node.py:650 | callback ROS | ft_receiver_node.py:223 `self._on_command, 10)` | mantido |
| 19 | `_run_command` | touch_pack/ft_receiver_node.py:695 | thread | ft_receiver_node.py:684 `threading.Thread(target=self._run_command, args=(cmd, args),` | mantido |
| 20 | `_line_write` | touch_pack/ft_serial.py:186 | injeção (tap) | `ft_cmd_channel.py:83` `yield self._line_write, self._tap_read` | mantido |
| 21 | `_line_write` | touch_pack/ft_tcp.py:232 | injeção (tap) | `ft_cmd_channel.py:83` `yield self._line_write, self._tap_read` | mantido |
| 22 | `_toggle_camera_control` | touch_pack/gui_camera.py:39 | referência direta | palpation_gui.py:1107 `conn, '📷', 'CAM OFF', self._toggle_camera_control,` | mantido |
| 23 | `_poll_camera_teleop` | touch_pack/gui_camera.py:73 | callback Tk (after) | gui_camera.py:65 `self._cam_teleop_after = self.root.after(500, self._poll_camera_teleop)` | mantido |
| 24 | `_on_camera_curl` | touch_pack/gui_camera.py:84 | callback injetado | gui_camera.py:59 `on_curl=self._on_camera_curl, camera_index=cam_idx,` | mantido |
| 25 | `_apply_camera_curl` | touch_pack/gui_camera.py:87 | callback Tk (after) | gui_camera.py:80 `self.root.after(0, self._apply_camera_curl, curl)` | mantido |
| 26 | `_pump_camera_preview` | touch_pack/gui_camera.py:156 | callback Tk (after) | gui_camera.py:168 `self._cam_preview_after = self.root.after(60, self._pump_camera_preview)` | mantido |
| 27 | `_apply_covvi_grip` | touch_pack/gui_hand.py:66 | callback Tk (command) | palpation_gui.py:2161 `grow, text='✓  Apply', command=self._apply_covvi_grip,` | mantido |
| 28 | `_on_real_hand_posn` | touch_pack/gui_hand.py:139 | callback ROS | gui_hand.py:197 `self._on_real_hand_posn, 10)` | mantido |
| 29 | `_pipe_hand_log` | touch_pack/gui_hand.py:317 | thread | gui_hand.py:324 `threading.Thread(target=_pipe_hand_log, daemon=True,` | mantido |
| 30 | `_disconnect_hand_worker` | touch_pack/gui_hand.py:380 | thread | gui_hand.py:378 `target=self._disconnect_hand_worker,` | mantido |
| 31 | `_hand_watchdog_loop` | touch_pack/gui_hand.py:417 | thread | gui_hand.py:409 `target=self._hand_watchdog_loop, daemon=True)` | mantido |
| 32 | `_on_hand_died` | touch_pack/gui_hand.py:435 | callback Tk (after) | gui_hand.py:433 `self.root.after(0, self._on_hand_died)` | mantido |
| 33 | `_on_hand_respawn` | touch_pack/gui_hand.py:454 | callback Tk (after) | gui_hand.py:453 `self.root.after(15000, self._on_hand_respawn)` | mantido |
| 34 | `_toggle_hand_power` | touch_pack/gui_hand.py:596 | referência direta | palpation_gui.py:1101 `conn, '⊙', 'PWR OFF', self._toggle_hand_power,` | mantido |
| 35 | `_imu_watchdog` | touch_pack/gui_imu.py:367 | thread | gui_imu.py:365 `threading.Thread(target=self._imu_watchdog, daemon=True,` | mantido |
| 36 | `_imu_on_calib` | touch_pack/gui_imu.py:648 | callback ROS | gui_imu.py:261 `self.create_subscription(String, CALIB_STATUS_TOPIC, self._imu_on_calib, 10)` | mantido |
| 37 | `_imu_zero` | touch_pack/gui_imu.py:776 | callback Tk (command) | gui_imu.py:151 `btns, text='⟲  Zero', command=self._imu_zero,` | mantido |
| 38 | `_imu_on_mirror_change` | touch_pack/gui_imu.py:781 | callback Tk (command) | gui_imu.py:168 `command=self._imu_on_mirror_change, bg=PANEL, fg=TEXT,` | mantido |
| 39 | `_imu_shutdown` | touch_pack/gui_imu.py:796 | chamada por mixin | palpation_gui.py:4746 `self._imu_shutdown()` | mantido |
| 40 | `_imu_tick` | touch_pack/gui_imu.py:810 | callback Tk (after) | gui_imu.py:363 `self._imu_after = self.root.after(IMU_TICK_MS, self._imu_tick)` | mantido |
| 41 | `_page_choose` | touch_pack/gui_imu_calib.py:182 | referência direta | gui_imu_calib.py:126 `(self._page_choose, self._page_prepare, self._page_run, self._page_result)[step]` | mantido |
| 42 | `_start_queue` | touch_pack/gui_imu_calib.py:234 | chamada por mixin | gui_imu_calib.py:207 `self._next = self._btn('Next  →', self._start_queue)` | mantido |
| 43 | `_page_prepare` | touch_pack/gui_imu_calib.py:239 | referência direta | gui_imu_calib.py:126 `(self._page_choose, self._page_prepare, self._page_run, self._page_result)[step]` | mantido |
| 44 | `_begin` | touch_pack/gui_imu_calib.py:271 | chamada por mixin | gui_imu_calib.py:259 `self._start = self._btn('▶  Start recording', self._begin, OK)` | mantido |
| 45 | `_page_run` | touch_pack/gui_imu_calib.py:282 | referência direta | gui_imu_calib.py:126 `(self._page_choose, self._page_prepare, self._page_run, self._page_result)[step]` | mantido |
| 46 | `_page_result` | touch_pack/gui_imu_calib.py:351 | referência direta | gui_imu_calib.py:126 `(self._page_choose, self._page_prepare, self._page_run, self._page_result)[step]` | mantido |
| 47 | `_repeat` | touch_pack/gui_imu_calib.py:399 | chamada por mixin | gui_imu_calib.py:386 `self._btn('Repeat', self._repeat, BTN_NEUTRAL, side='left')` | mantido |
| 48 | `_next_in_queue` | touch_pack/gui_imu_calib.py:403 | chamada por mixin | gui_imu_calib.py:380 `self._btn(f'Next sensor: ID {remaining[0]}  →', self._next_in_queue, PRIMARY)` | mantido |
| 49 | `_mag_next` | touch_pack/gui_imu_calib.py:408 | chamada por mixin | gui_imu_calib.py:384 `self._btn('Calibrate magnetometer next  →', self._mag_next, BTN_NEUTRAL,` | mantido |
| 50 | `_lc_do_tare` | touch_pack/gui_loadcell.py:105 | callback Tk (command) | gui_loadcell.py:417 `command=self._lc_do_tare,` | mantido |
| 51 | `_cb_lc_tared` | touch_pack/gui_loadcell.py:134 | callback ROS | palpation_gui.py:654 `Bool, '/load_cell/tared', self._cb_lc_tared, 10)` | mantido |
| 52 | `_cb_lc_force_net_gui` | touch_pack/gui_loadcell.py:138 | callback ROS | palpation_gui.py:649 `Float32, '/load_cell/force_net', self._cb_lc_force_net_gui, QOS_SENSOR)` | mantido |
| 53 | `_cb_lc_force_raw_gui` | touch_pack/gui_loadcell.py:144 | callback ROS | palpation_gui.py:651 `Float32, '/load_cell/force', self._cb_lc_force_raw_gui, QOS_SENSOR)` | mantido |
| 54 | `_ft_feed_processing` | touch_pack/gui_loadcell.py:171 | chamada por mixin | palpation_gui.py:2677 `self._ft_feed_processing(now, vals)` | mantido |
| 55 | `_cb_ft_cmd_result` | touch_pack/gui_loadcell.py:225 | callback Tk (after) | palpation_gui.py:660 `lambda m: self.root.after(0, self._cb_ft_cmd_result, m), 10)` | mantido |
| 56 | `_build_lc_axes_tab` | touch_pack/gui_loadcell.py:277 | chamada por mixin | palpation_gui.py:2971 `self._build_lc_axes_tab(self._scrollable(tab_axes))` | mantido |
| 57 | `_ft_do_scan` | touch_pack/gui_loadcell.py:565 | callback Tk (command) | gui_loadcell.py:544 `tk.Button(r5, text='⌕  Scan registers', command=self._ft_do_scan,` | mantido |
| 58 | `_ft_apply_sg` | touch_pack/gui_loadcell.py:635 | callback Tk (command) | gui_loadcell.py:612 `command=self._ft_apply_sg, bg=PANEL, fg=TEXT,` | mantido |
| 59 | `_ft_apply_stats_window` | touch_pack/gui_loadcell.py:697 | callback Tk (command) | gui_loadcell.py:666 `command=self._ft_apply_stats_window).pack(side='left')` | mantido |
| 60 | `_ft_reset_stats` | touch_pack/gui_loadcell.py:706 | callback Tk (command) | gui_loadcell.py:667 `tk.Button(row, text='Reset', command=self._ft_reset_stats,` | mantido |
| 61 | `_ft_toggle_recording` | touch_pack/gui_loadcell.py:734 | callback Tk (command) | gui_loadcell.py:718 `row, text='●  Start recording', command=self._ft_toggle_recording,` | mantido |
| 62 | `_refresh_ft_axes` | touch_pack/gui_loadcell.py:838 | callback Tk (after) | `gui_loadcell.py:853` e `palpation_gui.py:2972` `self.root.after(..., self._refresh_ft_axes)` | mantido |
| 63 | `_build_manip3d_tab` | touch_pack/gui_manip3d_tab.py:102 | chamada por mixin | palpation_gui.py:1236 `self._build_manip3d_tab(tab_manip)` | mantido |
| 64 | `_manip_on_q` | touch_pack/gui_manip3d_tab.py:352 | callback injetado | gui_manip3d_tab.py:114 `on_q=self._manip_on_q,` | mantido |
| 65 | `_manip_on_state` | touch_pack/gui_manip3d_tab.py:374 | callback injetado | gui_manip3d_tab.py:116 `on_state=self._manip_on_state,` | mantido |
| 66 | `_manip_on_drag_change` | touch_pack/gui_manip3d_tab.py:410 | callback injetado | gui_manip3d_tab.py:117 `on_drag_change=self._manip_on_drag_change,` | mantido |
| 67 | `_manip_clear_active` | touch_pack/gui_manip3d_tab.py:424 | referência direta | gui_manip3d_tab.py:424 `200, self._manip_clear_active)` | mantido |
| 68 | `_manip_capture_pose` | touch_pack/gui_manip3d_tab.py:442 | callback Tk (command) | gui_manip3d_tab.py:222 `command=self._manip_capture_pose,` | mantido |
| 69 | `_manip_go_home` | touch_pack/gui_manip3d_tab.py:449 | callback Tk (command) | gui_manip3d_tab.py:230 `command=self._manip_go_home,` | mantido |
| 70 | `_manip_scene_worker` | touch_pack/gui_manip3d_tab.py:468 | thread | gui_manip3d_tab.py:467 `threading.Thread(target=self._manip_scene_worker,` | mantido |
| 71 | `_manip_scene_done` | touch_pack/gui_manip3d_tab.py:499 | callback Tk (after) | gui_manip3d_tab.py:505 `self.root.after(0, self._manip_scene_done, scene, error)` | mantido |
| 72 | `_build_manual_extras` | touch_pack/gui_manual_extras.py:68 | chamada por mixin | palpation_gui.py:2092 `self._build_manual_extras(extras)` | mantido |
| 73 | `_build_matrix_group` | touch_pack/gui_matrix.py:30 | chamada por mixin | palpation_gui.py:1565 `self._build_matrix_group(self._matrix_group)` | mantido |
| 74 | `_on_matrix_param_change` | touch_pack/gui_matrix.py:339 | callback Tk (trace) | gui_matrix.py:183 `var.trace_add('write', self._on_matrix_param_change)` | mantido |
| 75 | `_build_poses_tab` | touch_pack/gui_poses.py:69 | chamada por mixin | palpation_gui.py:1222 `self._build_poses_tab(tab_poses)   # layout próprio — sem _scrollable externo` | mantido |
| 76 | `_load_poses_data` | touch_pack/gui_poses.py:187 | chamada por mixin | palpation_gui.py:867 `self._load_poses_data()` | mantido |
| 77 | `_on_movement_select` | touch_pack/gui_poses.py:247 | callback Tk (bind) | gui_poses.py:179 `self._movs_lbx.bind('<<ListboxSelect>>', self._on_movement_select)` | mantido |
| 78 | `_add_pose_to_seq` | touch_pack/gui_poses.py:321 | referência direta | gui_poses.py:381 `for txt, cmd in [('+ Adicionar', _add_pose_to_seq),` | mantido |
| 79 | `_remove_pose_from_seq` | touch_pack/gui_poses.py:333 | referência direta | gui_poses.py:382 `('−', _remove_pose_from_seq),` | mantido |
| 80 | `_move_up` | touch_pack/gui_poses.py:343 | referência direta | gui_poses.py:383 `('↑', _move_up),` | mantido |
| 81 | `_move_down` | touch_pack/gui_poses.py:355 | referência direta | gui_poses.py:384 `('↓', _move_down)]:` | mantido |
| 82 | `_on_spd` | touch_pack/gui_poses.py:389 | callback Tk (command) | gui_poses.py:410 `command=_on_spd).pack(anchor='w', pady=(0, 10))` | mantido |
| 83 | `_on_dur` | touch_pack/gui_poses.py:406 | callback Tk (command) | gui_poses.py:428 `command=_on_dur).pack(anchor='w', pady=(0, 16))` | mantido |
| 84 | `_capture_pose_robot` | touch_pack/gui_poses.py:444 | callback Tk (command) | gui_poses.py:95 `command=self._capture_pose_robot,` | mantido |
| 85 | `_capture_pose_sim` | touch_pack/gui_poses.py:456 | callback Tk (command) | gui_poses.py:103 `command=self._capture_pose_sim,` | mantido |
| 86 | `_rename_selected_pose` | touch_pack/gui_poses.py:482 | callback Tk (command) | gui_poses.py:133 `command=self._rename_selected_pose,` | mantido |
| 87 | `_delete_selected_pose` | touch_pack/gui_poses.py:496 | callback Tk (command) | gui_poses.py:141 `command=self._delete_selected_pose,` | mantido |
| 88 | `_new_movement` | touch_pack/gui_poses.py:513 | callback Tk (command) | gui_poses.py:156 `command=self._new_movement,` | mantido |
| 89 | `_stop_execution` | touch_pack/gui_poses.py:568 | callback Tk (command) | gui_poses.py:444 `command=self._stop_execution,` | mantido |
| 90 | `_execute_movement_worker` | touch_pack/gui_poses.py:584 | thread | gui_poses.py:582 `target=self._execute_movement_worker,` | mantido |
| 91 | `_ok` | touch_pack/gui_poses.py:696 | callback Tk (command) | gui_poses.py:716 `tk.Button(row, text='OK', command=_ok,` | mantido |
| 92 | `_cancel` | touch_pack/gui_poses.py:702 | callback Tk (command) | gui_poses.py:720 `tk.Button(row, text='Cancel', command=_cancel,` | mantido |
| 93 | `_mirror_movj_debounced` | touch_pack/gui_robot.py:90 | chamada por mixin | palpation_gui.py:2439 `self._mirror_movj_debounced(positions_rad[:6])` | mantido |
| 94 | `_mirror_poll_loop` | touch_pack/gui_robot.py:144 | thread | palpation_gui.py:896 `target=self._mirror_poll_loop, daemon=True, name='mirror-poll')` | mantido |
| 95 | `_load_robot_config` | touch_pack/gui_robot.py:325 | chamada por mixin | palpation_gui.py:678 `self._load_robot_config()` | mantido |
| 96 | `_connect_real_robot` | touch_pack/gui_robot.py:366 | referência direta | palpation_gui.py:1142 `conn_rob, '⚡', 'Connect', self._connect_real_robot,` | mantido |
| 97 | `_connect_robot_worker` | touch_pack/gui_robot.py:410 | thread | gui_robot.py:437 `target=self._connect_robot_worker, args=(ip,), daemon=True).start()` | mantido |
| 98 | `_sync_gazebo_to_real` | touch_pack/gui_robot.py:545 | thread | gui_robot.py:596 `threading.Thread(target=self._sync_gazebo_to_real, args=(drv,),` | mantido |
| 99 | `_update_sliders` | touch_pack/gui_robot.py:572 | callback Tk (after) | gui_robot.py:635 `self.root.after(0, _update_sliders)` | mantido |
| 100 | `_robot_heartbeat_loop` | touch_pack/gui_robot.py:633 | thread | gui_robot.py:678 `target=self._robot_heartbeat_loop, daemon=True)` | mantido |
| 101 | `_on_robot_connection_lost` | touch_pack/gui_robot.py:728 | callback Tk (after) | gui_robot.py:727 `self.root.after(0, self._on_robot_connection_lost)` | mantido |
| 102 | `_robot_reconnect_worker` | touch_pack/gui_robot.py:768 | thread | gui_robot.py:832 `target=self._robot_reconnect_worker, args=(ip,), daemon=True)` | mantido |
| 103 | `_set_robot_mode` | touch_pack/gui_robot.py:823 | callback Tk (command) | palpation_gui.py:1154 `command=self._set_robot_mode)  # type: ignore[arg-type]` | mantido |
| 104 | `_estop` | touch_pack/gui_robot.py:857 | referência direta | palpation_gui.py:1059 `estop = _hdr_btn(top, '■', 'E-STOP', self._estop,` | mantido |
| 105 | `_start_touch_source` | touch_pack/gui_sensors.py:114 | chamada por mixin | palpation_gui.py:903 `self._start_touch_source()` | mantido |
| 106 | `_retry_touch_source` | touch_pack/gui_sensors.py:168 | callback Tk (after) | gui_sensors.py:188 `self.root.after(2000, self._retry_touch_source)` | mantido |
| 107 | `_on_touch_sample` | touch_pack/gui_sensors.py:189 | callback injetado | `palpation_gui.py:877` `on_sample=self._on_touch_sample` | mantido |
| 108 | `_build_sensors_tab` | touch_pack/gui_sensors.py:211 | chamada por mixin | `palpation_gui.py:1225` `self._build_sensors_tab(tab_sensors)` | mantido |
| 109 | `_toggle_sensors_detach` | touch_pack/gui_sensors.py:322 | referência direta | gui_sensors.py:228 `hdr, '⧉', 'Detach', self._toggle_sensors_detach)` | mantido |
| 110 | `_refresh_sensors_tab` | touch_pack/gui_sensors.py:350 | referência direta | gui_sensors.py:375 `80, self._refresh_sensors_tab)` | mantido |
| 111 | `timed_blit` | touch_pack/gui_sensors.py:399 | monkeypatch de biblioteca | `gui_sensors.py:405` `canvas.blit = timed_blit` | mantido |
| 112 | `_touch_anim_frame` | touch_pack/gui_sensors.py:406 | callback de biblioteca | `FuncAnimation(...)` em gui_sensors.py | mantido |
| 113 | `_toggle_recording` | touch_pack/gui_sensors.py:521 | callback Tk (command) | palpation_gui.py:1891 `command=self._toggle_recording, bg=BTN_NEUTRAL, fg=TEXT,` | mantido |
| 114 | `_discard_learning_recording` | touch_pack/gui_sensors.py:571 | callback Tk (after) | gui_sensors.py:569 `self.root.after(0, self._discard_learning_recording, str(msg.data))` | mantido |
| 115 | `_recording_status_tick` | touch_pack/gui_sensors.py:783 | referência direta | gui_sensors.py:565 `self._REC_STATUS_MS, self._recording_status_tick)` | mantido |
| 116 | `_cb_touch_value` | touch_pack/gui_sensors.py:839 | referência direta | gui_sensors.py:166 `Float32, '/touch_sensor/value', self._cb_touch_value,` | mantido |
| 117 | `_spawn_touch_receiver` | touch_pack/gui_sensors.py:850 | chamada por mixin | palpation_gui.py:4501 `self._spawn_touch_receiver()` | mantido |
| 118 | `_pipe_log` | touch_pack/gui_sensors.py:871 | thread | gui_sensors.py:870 `threading.Thread(target=_pipe_log, daemon=True,` | mantido |
| 119 | `_kill_touch_receiver` | touch_pack/gui_sensors.py:878 | chamada por mixin | palpation_gui.py:4799 `self._kill_touch_receiver()` | mantido |
| 120 | `_draw_touch_spark` | touch_pack/gui_sensors.py:892 | chamada por mixin | palpation_gui.py:3877 `self._draw_touch_spark()` | mantido |
| 121 | `_build_traj_card` | touch_pack/gui_trajectory.py:55 | chamada por mixin | gui_poses.py:111 `self._build_traj_card(left)` | mantido |
| 122 | `_traj_toggle` | touch_pack/gui_trajectory.py:81 | callback Tk (command) | gui_trajectory.py:71 `btns, text='⏺  Record', command=self._traj_toggle,` | mantido |
| 123 | `_loop` | touch_pack/hand_camera_teleop.py:310 | thread | hand_camera_teleop.py:267 `target=self._loop, name='hand-camera-teleop', daemon=True)` | mantido |
| 124 | `_cb_real_posn` | touch_pack/hand_latency_probe.py:177 | callback ROS | hand_latency_probe.py:172 `self._cb_real_posn, 10)` | mantido |
| 125 | `_cb_calib` | touch_pack/imu_node.py:76 | callback ROS | imu_node.py:74 `self.create_subscription(String, CALIB_REQ_TOPIC, self._cb_calib, 10)` | mantido |
| 126 | `create_sensor_client` | touch_pack/imu_node.py:209 | sinal Qt | imu_node.py:337 `central_ctrl.list_sensor.connect(create_sensor_client)` | mantido |
| 127 | `_real_loop` | touch_pack/imu_recorder.py:116 | thread | imu_recorder.py:85 `self._reader = threading.Thread(target=self._real_loop, args=(real_reader,),` | mantido |
| 128 | `_write_loop` | touch_pack/imu_recorder.py:127 | thread | imu_recorder.py:80 `self._writer = threading.Thread(target=self._write_loop, daemon=True,` | mantido |
| 129 | `cost` | touch_pack/kinematics.py:413 | callback de biblioteca | `solutions.sort(key=cost)` | mantido |
| 130 | `reach_margin` | touch_pack/kinematics.py:552 | API pública declarada | docstring de `kinematics.py:26`; gêmea usada em grasp_ml_pack (`grasp_quality_net.py:93`) | mantido |
| 131 | `singularity_distances` | touch_pack/kinematics.py:561 | API pública declarada | docstring de `kinematics.py:27`; gêmea usada em grasp_ml_pack | mantido |
| 132 | `_finger_ik` | touch_pack/kinematics.py:594 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; item C04 | removido |
| 133 | `grasp_center_in_hand` | touch_pack/kinematics.py:744 | REMOVIDO (C07) | zero referências em touch_pack, rehab_pack e hand_pack, fora da API declarada; a versão viva é a de grasp_ml_pack. Saíram juntos os privados que só ela alcançava: `hand_fk`, `_finger_tip_in_hand_long`, `_thumb_tip_in_hand`, `_HAND_MCP`, `_THUMB_*`, `_K_P_THUMB`. `_rotate_axis_angle` fica (usada na correção do XC1) | removido |
| 134 | `hand_ik` | touch_pack/kinematics.py:759 | API pública declarada | docstring de `kinematics.py:29`; gêmea usada em grasp_ml_pack | mantido |
| 135 | `_real_poll_loop` | touch_pack/latency_probe.py:157 | thread | latency_probe.py:154 `threading.Thread(target=self._real_poll_loop, daemon=True,` | mantido |
| 136 | `_lifecycle_init` | touch_pack/line_source.py:61 | chamada por mixin | ft_serial.py:161 `self._lifecycle_init()` | mantido |
| 137 | `_worker` | touch_pack/line_source.py:117 | thread | line_source.py:101 `target=self._worker, daemon=True, name=self._THREAD_NAME)` | mantido |
| 138 | `set_end_effector` | touch_pack/manip3d.py:528 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; achado do bloco de origem | removido |
| 139 | `_on_press` | touch_pack/manip3d.py:685 | callback Tk (bind) | manip3d.py:487 `self.bind('<ButtonPress-1>', self._on_press)` | mantido |
| 140 | `_on_motion` | touch_pack/manip3d.py:705 | callback Tk (bind) | manip3d.py:488 `self.bind('<B1-Motion>', self._on_motion)` | mantido |
| 141 | `_on_release` | touch_pack/manip3d.py:723 | callback Tk (bind) | manip3d.py:489 `self.bind('<ButtonRelease-1>', self._on_release)` | mantido |
| 142 | `_on_hover` | touch_pack/manip3d.py:738 | callback Tk (bind) | manip3d.py:490 `self.bind('<Motion>', self._on_hover)` | mantido |
| 143 | `_on_cam_press` | touch_pack/manip3d.py:747 | callback Tk (bind) | manip3d.py:491 `self.bind('<ButtonPress-2>', self._on_cam_press)` | mantido |
| 144 | `_on_cam_motion` | touch_pack/manip3d.py:751 | callback Tk (bind) | manip3d.py:492 `self.bind('<B2-Motion>', self._on_cam_motion)` | mantido |
| 145 | `_on_cam_release` | touch_pack/manip3d.py:760 | callback Tk (bind) | manip3d.py:493 `self.bind('<ButtonRelease-2>', self._on_cam_release)` | mantido |
| 146 | `_on_wheel` | touch_pack/manip3d.py:763 | callback Tk (bind) | manip3d.py:497 `self.bind('<MouseWheel>', self._on_wheel)` | mantido |
| 147 | `_on_configure` | touch_pack/manip3d.py:768 | callback Tk (bind) | manip3d.py:500 `self.bind('<Configure>', self._on_configure)` | mantido |
| 148 | `_connect_loop` | touch_pack/mirror_node.py:156 | thread | mirror_node.py:156 `threading.Thread(target=self._connect_loop, daemon=True,` | mantido |
| 149 | `_cb_status` | touch_pack/mirror_node.py:214 | callback ROS | mirror_node.py:147 `PalpationStatus, '/palpation/status', self._cb_status, 10)` | mantido |
| 150 | `_cb_trajectory` | touch_pack/mirror_node.py:227 | callback ROS | mirror_node.py:150 `self._cb_trajectory, 1)` | mantido |
| 151 | `_movj_send` | touch_pack/mirror_node.py:247 | referência direta | mirror_node.py:341 `_MOVJ_DEBOUNCE_S, self._movj_send, args=[target[:6]])` | mantido |
| 152 | `_servoj_loop` | touch_pack/mirror_node.py:261 | thread | mirror_node.py:158 `threading.Thread(target=self._servoj_loop, daemon=True,` | mantido |
| 153 | `_experiment_active` | touch_pack/palpation_gui.py:930 | chamada por mixin | gui_sensors.py:663 `if self._experiment_active():` | mantido |
| 154 | `_draw_force_spark` | touch_pack/palpation_gui.py:958 | chamada por mixin | gui_sensors.py:520 `self._draw_force_spark(self._sens_force_spark)` | mantido |
| 155 | `_on_inner` | touch_pack/palpation_gui.py:1275 | callback Tk (bind) | palpation_gui.py:1282 `inner.bind('<Configure>', _on_inner)` | mantido |
| 156 | `_on_canvas` | touch_pack/palpation_gui.py:1279 | callback Tk (bind) | palpation_gui.py:1286 `canvas.bind('<Configure>', _on_canvas)` | mantido |
| 157 | `_wheel` | touch_pack/palpation_gui.py:1285 | callback Tk (bind) | palpation_gui.py:1294 `lambda _e: (canvas.bind_all('<MouseWheel>', _wheel),` | mantido |
| 158 | `_publish_arm_q` | touch_pack/palpation_gui.py:2368 | chamada por mixin | gui_camera.py:144 `self._publish_arm_q(q_rad, MANIP_TRAJ_DURATION_S)` | mantido |
| 159 | `_cb_arm_trajectory` | touch_pack/palpation_gui.py:2388 | callback ROS | palpation_gui.py:640 `self._cb_arm_trajectory, 1)  # depth=1: só o setpoint mais recente` | mantido |
| 160 | `_cb_joint_states` | touch_pack/palpation_gui.py:2434 | callback ROS | palpation_gui.py:644 `JointState, '/joint_states', self._cb_joint_states, 5)` | mantido |
| 161 | `_prehome_worker` | touch_pack/palpation_gui.py:2525 | thread | palpation_gui.py:2529 `threading.Thread(target=self._prehome_worker,` | mantido |
| 162 | `_prehome_failed` | touch_pack/palpation_gui.py:2593 | callback Tk (after) | palpation_gui.py:2602 `self.root.after(0, self._prehome_failed, erro, sync_only)` | mantido |
| 163 | `_cb_ft_rate` | touch_pack/palpation_gui.py:2607 | callback ROS | palpation_gui.py:455 `self._cb_ft_rate, 10)` | mantido |
| 164 | `_solve_tcp_perpendicular` | touch_pack/palpation_gui.py:2683 | callback Tk (command) | palpation_gui.py:2080 `command=self._solve_tcp_perpendicular,` | mantido |
| 165 | `_capture_arm_from_robot` | touch_pack/palpation_gui.py:2801 | callback Tk (command) | palpation_gui.py:2068 `command=self._capture_arm_from_robot,` | mantido |
| 166 | `_on_home_known_toggle` | touch_pack/palpation_gui.py:2864 | callback Tk (command) | palpation_gui.py:1636 `variable=self.home_known_var, command=self._on_home_known_toggle,` | mantido |
| 167 | `_sync_home_known_checkbox` | touch_pack/palpation_gui.py:2889 | callback Tk (after) | palpation_gui.py:3722 `self.root.after(0, self._sync_home_known_checkbox, hk)` | mantido |
| 168 | `_on_tab_changed` | touch_pack/palpation_gui.py:2932 | callback Tk (bind) | palpation_gui.py:1244 `nb.bind('<<NotebookTabChanged>>', self._on_tab_changed, add='+')` | mantido |
| 169 | `_pipe_log` | touch_pack/palpation_gui.py:2987 | thread | palpation_gui.py:3004 `threading.Thread(target=_pipe_log, daemon=True,` | mantido |
| 170 | `_toggle_drag` | touch_pack/palpation_gui.py:3022 | callback Tk (command) | gui_poses.py:86 `command=self._toggle_drag,` | mantido |
| 171 | `_update_sliders_from_q` | touch_pack/palpation_gui.py:3047 | callback Tk (after) | gui_robot.py:210 `self.root.after(0, self._update_sliders_from_q,` | mantido |
| 172 | `_on_force_live_change` | touch_pack/palpation_gui.py:3529 | callback Tk (trace) | palpation_gui.py:1381 `self.force_sp_var.trace_add('write', self._on_force_live_change)` | mantido |
| 173 | `_publish_set_force` | touch_pack/palpation_gui.py:3547 | callback Tk (after) | palpation_gui.py:3557 `self._set_force_after_id = self.root.after(150, self._publish_set_force)` | mantido |
| 174 | `_snap` | touch_pack/palpation_gui.py:3572 | callback Tk (after) | palpation_gui.py:3596 `lambda *_a: self.root.after_idle(_snap))` | mantido |
| 175 | `_update_drag_btn_auto` | touch_pack/palpation_gui.py:3642 | callback Tk (after) | gui_robot.py:140 `self.root.after(0, self._update_drag_btn_auto, False)` | mantido |
| 176 | `_cb_status` | touch_pack/palpation_gui.py:3669 | callback ROS | palpation_gui.py:440 `PalpationStatus, '/palpation/status', self._cb_status, 10)` | mantido |
| 177 | `_refresh_status_panel` | touch_pack/palpation_gui.py:3712 | callback Tk (after) | palpation_gui.py:899 `self.root.after(100, self._refresh_status_panel)` | mantido |
| 178 | `_toggle_pause` | touch_pack/palpation_gui.py:3873 | callback Tk (command) | palpation_gui.py:1867 `command=self._toggle_pause, bg=BTN_NEUTRAL, fg=TEXT,` | mantido |
| 179 | `_on_stop_palpation` | touch_pack/palpation_gui.py:4009 | callback Tk (command) | palpation_gui.py:1858 `command=self._on_stop_palpation, bg=WARN, fg='white',` | mantido |
| 180 | `_on_start` | touch_pack/palpation_gui.py:4028 | callback Tk (command) | palpation_gui.py:1878 `command=self._on_start, bg=PRIMARY, fg='white',` | mantido |
| 181 | `_spin_ros` | touch_pack/palpation_gui.py:4634 | thread | palpation_gui.py:892 `target=self._spin_ros, daemon=True)` | mantido |
| 182 | `_sighandler` | touch_pack/palpation_gui.py:4807 | handler de sinal | `signal.signal(signal.SIGTERM, _sighandler)` | mantido |
| 183 | `_cb_attack_ref` | touch_pack/palpation_logger.py:367 | callback ROS | palpation_logger.py:325 `String, '/palpation/attack_ref', self._cb_attack_ref, 10)` | mantido |
| 184 | `_cb_set_force` | touch_pack/palpation_logger.py:500 | callback ROS | palpation_logger.py:335 `Float32, '/palpation/set_force', self._cb_set_force, 10)` | mantido |
| 185 | `_cb_status` | touch_pack/palpation_logger.py:513 | callback ROS | palpation_logger.py:331 `PalpationStatus, '/palpation/status', self._cb_status, 10)` | mantido |
| 186 | `_cb_matrix_point` | touch_pack/palpation_logger.py:549 | callback ROS | palpation_logger.py:354 `MatrixPoint, '/palpation/matrix_point', self._cb_matrix_point, 50)` | mantido |
| 187 | `_cb_event` | touch_pack/palpation_logger.py:672 | callback ROS | palpation_logger.py:347 `String, TOUCH_EVENT_TOPIC, self._cb_event, TOUCH_EVENT_QOS_DEPTH)` | mantido |
| 188 | `_cb_lc_sample` | touch_pack/palpation_logger.py:702 | referência direta | palpation_logger.py:342 `LoadCellSample, '/load_cell/sample_net', self._cb_lc_sample,` | mantido |
| 189 | `_watchdog` | touch_pack/palpation_logger.py:778 | timer ROS | palpation_logger.py:362 `self.create_timer(1.0, self._watchdog)` | mantido |
| 190 | `_generate_report` | touch_pack/palpation_logger.py:857 | thread | palpation_logger.py:867 `target=self._generate_report, args=(run_path,),` | mantido |
| 191 | `_keepalive_loop` | touch_pack/real_driver.py:355 | thread | real_driver.py:385 `target=self._keepalive_loop, daemon=True, name='cr10-keepalive')` | mantido |
| 192 | `mov_j_cartesian` | touch_pack/real_driver.py:926 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; item C04 | removido |
| 193 | `mov_l_cartesian` | touch_pack/real_driver.py:937 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; item C04 | removido |
| 194 | `rel_movl_user` | touch_pack/real_driver.py:945 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; item C04 | removido |
| 195 | `sync` | touch_pack/real_driver.py:1048 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; achado do bloco de origem | removido |
| 196 | `read_tcp_force` | touch_pack/real_driver.py:1181 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; achado do bloco de origem | removido |
| 197 | `get_angle_deg` | touch_pack/real_driver.py:1203 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; item C04 | removido |
| 198 | `tool_do` | touch_pack/real_driver.py:1218 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; item C04 | removido |
| 199 | `resample_trajectory` | touch_pack/real_driver.py:1235 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; item C04 | removido |
| 200 | `_on_joints` | touch_pack/real_pose_sync.py:116 | callback ROS | real_pose_sync.py:122 `self.create_subscription(JointState, '/joint_states', _on_joints, 10)` | mantido |
| 201 | `_on_tare` | touch_pack/sim_force_bridge.py:76 | callback ROS | sim_force_bridge.py:64 `self.create_subscription(Empty, '/load_cell/tare', self._on_tare, 10)` | mantido |
| 202 | `_cb_lc_sample_net` | touch_pack/tactile_explorer.py:633 | referência direta | tactile_explorer.py:568 `self._cb_lc_sample_net, qos_force,` | mantido |
| 203 | `_cb_set_force` | touch_pack/tactile_explorer.py:722 | callback ROS | tactile_explorer.py:557 `self._cb_set_force, 10, callback_group=cb)` | mantido |
| 204 | `_cb_stop` | touch_pack/tactile_explorer.py:730 | callback ROS | tactile_explorer.py:551 `self._cb_stop, 10, callback_group=cb)` | mantido |
| 205 | `_cb_forget_contact` | touch_pack/tactile_explorer.py:749 | callback ROS | tactile_explorer.py:559 `self._cb_forget_contact, 10, callback_group=cb)` | mantido |
| 206 | `_flush_learned` | touch_pack/tactile_explorer.py:944 | timer ROS | tactile_explorer.py:606 `self.create_timer(1.0, self._flush_learned, callback_group=cb)` | mantido |
| 207 | `_cartesian_stream` | touch_pack/tactile_explorer.py:2060 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; item C04 | removido |
| 208 | `_worker` | touch_pack/touch_source.py:310 | thread | touch_source.py:259 `target=self._worker, daemon=True, name="touch-serial")` | mantido |
| 209 | `latest_voltages` | touch_pack/touch_source.py:514 | REMOVIDO | zero chamadores em src/, scripts, launch e setup.py; achado do bloco de origem | removido |
| 210 | `init_blit` | touch_pack/touch_source.py:764 | callback de biblioteca | `FuncAnimation(init_func=...)` em gui_sensors.py | mantido |
| 211 | `_show` | touch_pack/ui_helpers.py:72 | callback Tk (bind) | ui_helpers.py:68 `widget.bind('<Enter>', self._show, add='+')` | mantido |
| 212 | `_hide` | touch_pack/ui_helpers.py:88 | callback Tk (bind) | ui_helpers.py:69 `widget.bind('<Leave>', self._hide, add='+')` | mantido |
| 213 | `coarse_scene` | touch_pack/urdf_scene.py:537 | chamada por alias | `gui_manip3d_tab.py:495` `_coarse_scene(scene)` | mantido |
