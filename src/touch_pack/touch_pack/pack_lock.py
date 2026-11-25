"""Trava entre touch_pack e rehab_pack: com um aberto, o outro não controla nada.

hand_pack e grasp_ml_pack, que também comandam o braço, tomam a do touch_pack:
convivem com ele e entre si, e nunca com o rehab_pack.

Cada processo de controle segura um flock COMPARTILHADO no arquivo do seu pack
(vários nós do mesmo pack convivem) e recusa se alguém segura o do outro pack.
O kernel solta o flock quando o processo morre: não há trava órfã.
Vale para a máquina toda, inclusive entre ROS_DOMAIN_IDs.
"""
from __future__ import annotations

import fcntl
import os

PACKS = ('touch_pack', 'rehab_pack')
_ABERTO = {'touch_pack': 'touch_pack (ou hand_pack/grasp_ml_pack)', 'rehab_pack': 'rehab_pack'}
_held: dict = {}      # (pasta, pack) -> arquivo aberto; vive até o processo sair


class PackLockError(RuntimeError):
    """O outro pack está aberto."""


def _dir() -> str:
    # /run/lock e não a home: o contêiner o monta e roda como outro usuário
    d = os.environ.get('CR10_PACK_LOCK_DIR') or '/run/lock/cr10twin'
    os.makedirs(d, exist_ok=True)
    _open_mode(d, 0o777)
    return d


def _open_mode(path: str, mode: int) -> None:
    try:
        os.chmod(path, mode)      # qualquer usuário (host, contêiner) participa
    except PermissionError:
        pass                      # não é nosso: quem criou já abriu


def _open(d: str, name: str):
    path = os.path.join(d, name)
    f = open(path, 'a')
    _open_mode(path, 0o666)
    return f


def acquire(pack: str) -> None:
    """Toma a trava do `pack` para este processo; PackLockError se o outro estiver aberto."""
    if pack not in PACKS:
        raise ValueError(pack)
    d = _dir()
    if (d, pack) in _held:
        return
    other = PACKS[1 - PACKS.index(pack)]
    # guard: serializa testar + tomar, senão dois packs poderiam entrar juntos
    with _open(d, 'guard.lock') as guard:
        fcntl.flock(guard, fcntl.LOCK_EX)
        with _open(d, f'{other}.lock') as probe:
            try:
                fcntl.flock(probe, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise PackLockError(
                    f'{_ABERTO[other]} está aberto: {pack} não pode controlar nada. '
                    f'Feche e tente de novo.') from None
        mine = _open(d, f'{pack}.lock')
        fcntl.flock(mine, fcntl.LOCK_SH)
        _held[(d, pack)] = mine


def acquire_or_exit(pack: str) -> None:
    """Para o `main()` dos nós: sai com a mensagem, antes de criar qualquer publisher."""
    try:
        acquire(pack)
    except PackLockError as exc:
        raise SystemExit(f'[TRAVA] {exc}') from None
