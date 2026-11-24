"""Escolha do dispositivo de computação (GPU ROCm/CUDA se utilizável, senão CPU)."""
from __future__ import annotations

import torch


def pick_device(prefer: str = 'auto') -> torch.device:
    if prefer != 'auto':
        return torch.device(prefer)
    try:
        if torch.cuda.is_available():
            torch.zeros(1, device='cuda')        # falha cedo se a arch não for suportada
            return torch.device('cuda')
    except Exception:
        pass
    return torch.device('cpu')


def describe(dev: torch.device) -> str:
    if dev.type == 'cuda':
        return f'GPU {torch.cuda.get_device_name(0)} (torch {torch.__version__})'
    return f'CPU {torch.get_num_threads()} threads (torch {torch.__version__})'
