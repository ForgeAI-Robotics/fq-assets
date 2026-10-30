"""kami: a fast simulator of paper (stiff in bending, nearly unstretchable, holds creases) for batches of sheets on the GPU.

    import kami
    sheet = kami.Sheet(kami.Paper.printer_80gsm(), kami.Sim(N=33), kami.CreaseLaw.k1c(), batch=16)
    sheet.step(60)
"""
from .config import CreaseLaw, Paper, Sim
from .sheet import Sheet
from . import metrics

__all__ = ['CreaseLaw', 'Paper', 'Sim', 'Sheet', 'metrics']
__version__ = '0.1.0.dev0'
