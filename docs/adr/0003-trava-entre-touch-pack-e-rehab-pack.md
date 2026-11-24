# 0003 — touch_pack e rehab_pack nunca controlam ao mesmo tempo

Os dois packs usam o mesmo CR10, a mesma FA7155 e o mesmo
`/cr10_group_controller/joint_trajectory`. Com um aberto, o outro não inicia
controle algum: `touch_pack/pack_lock.py` (flock, máquina toda, inclusive entre
`ROS_DOMAIN_ID`s; o kernel solta a trava quando o processo morre).

- O launch e o `main()` dos nós de controle tomam a trava. `ft_receiver` não:
  é usado pelos dois packs. A porta serial é aberta com `exclusive=True`.
- Trava na máquina, não no domínio: o hardware é um só. Para uma simulação
  isolada, sem hardware, use outro `CR10_PACK_LOCK_DIR`.
- Fica em `/run/lock/cr10twin`, que o `docker-compose.yml` monta. Outro computador e
  contêiner iniciado sem esse volume não são enxergados.
