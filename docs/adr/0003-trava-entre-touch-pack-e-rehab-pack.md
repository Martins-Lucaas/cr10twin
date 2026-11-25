# 0003 — touch_pack e rehab_pack nunca controlam ao mesmo tempo

Os dois packs usam o mesmo CR10, a mesma FA7155 e o mesmo
`/cr10_group_controller/joint_trajectory`. Com um aberto, o outro não inicia
controle algum: cada pack tem o seu `pack_lock.py`, sem importar o outro; só a pasta
das travas é comum (flock, máquina toda, inclusive entre
`ROS_DOMAIN_ID`s; o kernel solta a trava quando o processo morre).

- O launch e o `main()` dos nós de controle tomam a trava. `ft_receiver` não
  (cada pack tem o seu). A porta serial é aberta com `exclusive=True`.
- Os produtores do mesmo comando em `hand_pack` e `grasp_ml_pack` tomam a trava do
  `touch_pack`: convivem com ele, não com o rehab.
- Fora da trava (outra máquina, `ros2 topic pub`): o rehab conta os publicadores do
  comando do braço ao habilitar, no jog e a cada tick; um de fora recusa ou aborta.
- Trava na máquina, não no domínio: o hardware é um só. Para uma simulação
  isolada, sem hardware, use outro `CR10_PACK_LOCK_DIR`.
- Fica em `/run/lock/cr10twin`, que o `docker-compose.yml` monta. Outro computador e
  contêiner iniciado sem esse volume não são enxergados.
