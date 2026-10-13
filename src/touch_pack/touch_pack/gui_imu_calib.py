"""gui_imu_calib.py — assistente de calibração dos IMUs (janela própria).

Quatro passos, cada um só avança com confirmação do usuário:

  1. ESCOLHA     tipo (Acc + Gyro | Magnetômetro) e quais IDs — só os
                 conectados podem ser marcados;
  2. PREPARO     instruções do tipo + checklist que precisa ser todo marcado
                 para liberar o Start (e os sensores escolhidos ainda ligados);
  3. COLETA      barra de progresso por sensor, dica do que fazer agora e
                 Cancelar (o imu_node aborta sem gravar). No MAG, um sensor
                 3D animado (gui_imu_anim) mostra o movimento, em sincronia
                 com a dica;
  4. RESULTADO   ✓/✗ por sensor (MAG com a qualidade do ajuste) e a
                 confirmação de deixar parado ~2 s para a orientação recomeçar.

Acc + Gyro calibra todos os escolhidos de uma vez (ficam parados na mesa).
O magnetômetro vai UM sensor por vez — é preciso girá-lo na mão, longe dos
outros — e os escolhidos entram numa fila: preparo → coleta → resultado para
cada um.

A coleta e o cálculo são do imu_node (rotina do WBAN): esta janela só manda
"AG 4" / "MAG 5" / "CANCEL 4" e lê o andamento que o host recebe em
/imu/calibration (`host._imu_calib`). Nada de ttk.Progressbar (segfault com
Canvas embutido na GUI, ver palpation_gui) — as barras são Canvas.
"""
from __future__ import annotations

import time
import tkinter as tk

from .gui_imu_anim import MAG_PHASE_S, MAG_PHASES, Sensor3DAnim
from .ui_helpers import (
    BG, BORDER, BTN_NEUTRAL, DANGER, FONT_HEAD, FONT_LBL, FONT_SMALL, OK, PANEL,
    PRIMARY, TEXT, TEXT_DIM, TEXT_MUTED, WARN, _shade,
)

KINDS = {
    'AG': {
        'name': 'Accelerometer + Gyroscope',
        'what': 'Measures the offsets of the accelerometer and the gyroscope '
                '(both from the same recording) at 500 Hz.',
        'duration_s': 10.0,
        'checklist': (
            '{who} OFF the body, lying on a flat, stable table.',
            'Level (not tilted) and not touching each other.',
            'Nobody will touch or bump the table during the recording (~15 s).',
        ),
        'during': ('Do NOT touch the sensors or the table.',),
    },
    'MAG': {
        'name': 'Magnetometer',
        'what': 'Fits the hard/soft-iron correction of the magnetometer. Needed '
                'for a stable heading (yaw).',
        'duration_s': 30.0,
        'checklist': (
            'I am holding {who} in my hand; the other sensors are put away.',
            'I am at least 1 m away from the robot, metal tables, magnets and '
            'the laptop.',
            'I watched the animation: I will do each movement SLOWLY, following '
            'it, for the whole 30 s.',
        ),
        'during': tuple(tip for _p, tip in MAG_PHASES),
    },
}
STEPS = ('1  Choose', '2  Prepare', '3  Record', '4  Result')
ANSWER_TIMEOUT_S = 5.0
SETTLE_S = 2.0


def mag_quality(cov_pct: float) -> tuple[str, str]:
    """Texto e cor da qualidade do ajuste (coef. de variação do raio)."""
    if cov_pct < 3.0:
        return 'good', OK
    if cov_pct < 6.0:
        return 'acceptable', WARN
    return 'poor — repeat it, rotating through more directions', DANGER


class ImuCalibWizard(tk.Toplevel):
    def __init__(self, host, kind: str = 'AG', targets=()):
        super().__init__(host.root)
        self.host = host
        self.title('IMU calibration')
        self.configure(bg=BG)
        self.geometry('600x660')
        self.minsize(560, 600)
        self.transient(host.root.winfo_toplevel())
        self.protocol('WM_DELETE_WINDOW', self._on_close)

        self.kind = tk.StringVar(self, value=kind)
        self.sel = {sid: tk.BooleanVar(self, value=sid in targets) for sid in range(1, 7)}
        self.step = 0
        self.running: list[int] = []
        self.queue: list[int] = []   # escolhidos; o MAG anda nela um por vez
        self.qi = 0
        self.t_start = 0.0
        self._after = None
        self._settled = tk.BooleanVar(self, value=False)

        head = tk.Frame(self, bg=PANEL, highlightthickness=1, highlightbackground=BORDER)
        head.pack(fill='x', padx=12, pady=(12, 0))
        self._step_lbls = []
        for i, txt in enumerate(STEPS):
            lb = tk.Label(head, text=txt, font=FONT_LBL, bg=PANEL, padx=10, pady=8)
            lb.pack(side='left', expand=True, fill='x')
            self._step_lbls.append(lb)
        self.body = tk.Frame(self, bg=PANEL, highlightthickness=1,
                             highlightbackground=BORDER)
        self.body.pack(fill='both', expand=True, padx=12, pady=12)
        self.foot = tk.Frame(self, bg=BG)
        self.foot.pack(fill='x', padx=12, pady=(0, 12))
        self._show(0)
        self._poll()

    # ── Infra ────────────────────────────────────────────────────────
    def _clear(self):
        for w in (*self.body.winfo_children(), *self.foot.winfo_children()):
            w.destroy()

    def _show(self, step: int) -> None:
        self.step = step
        for i, lb in enumerate(self._step_lbls):
            lb.config(fg=PRIMARY if i == step else (TEXT_MUTED if i < step else TEXT_DIM),
                      font=FONT_HEAD if i == step else FONT_LBL)
        self._clear()
        (self._page_choose, self._page_prepare, self._page_run, self._page_result)[step]()

    def _btn(self, text, cmd, color=PRIMARY, side='right'):
        fg = 'white' if color not in (BTN_NEUTRAL,) else TEXT
        b = tk.Button(self.foot, text=text, command=cmd, bg=color, fg=fg,
                      activebackground=_shade(color, -0.08), activeforeground=fg,
                      disabledforeground=TEXT_DIM,
                      font=FONT_LBL, relief='flat', bd=0, padx=16, pady=7,
                      cursor='hand2')
        b.pack(side=side, padx=(6, 0) if side == 'right' else (0, 6))
        b.color = color
        return b

    @staticmethod
    def _enable(btn, on: bool) -> None:
        """Desabilitado fica cinza de verdade — só o texto apagado parecia ativo."""
        c = btn.color if on else BTN_NEUTRAL
        btn.config(state='normal' if on else 'disabled', bg=c,
                   cursor='hand2' if on else 'arrow')

    def _text(self, text, fg=TEXT, font=FONT_LBL, pady=(0, 6)):
        lb = tk.Label(self.body, text=text, font=font, bg=PANEL, fg=fg,
                      anchor='w', justify='left', wraplength=500)
        lb.pack(fill='x', padx=16, pady=pady)
        return lb

    def _check(self, parent, text, var, cmd=None):
        cb = tk.Checkbutton(parent, text=text, variable=var, command=cmd,
                            bg=PANEL, fg=TEXT, activebackground=PANEL,
                            selectcolor=PANEL, font=FONT_LBL, anchor='w',
                            justify='left', wraplength=470, highlightthickness=0,
                            bd=0, cursor='hand2')
        cb.pack(fill='x', padx=16, pady=3)
        return cb

    def _live(self) -> set:
        return set(getattr(self.host, '_imu_live', set()))

    def _chosen(self) -> list[int]:
        return [sid for sid, v in self.sel.items() if v.get()]

    def _targets(self) -> list[int]:
        """Quem entra na próxima coleta: todos (AG) ou o da vez (MAG)."""
        if self.kind.get() == 'MAG':
            return self.queue[self.qi:self.qi + 1]
        return list(self.queue)

    def _who(self) -> str:
        t = self._targets()
        return f'Sensor ID {t[0]}' if len(t) == 1 else f'Sensors {", ".join(map(str, t))}'

    def _status(self, sid: int):
        st = self.host._imu_calib.get(sid)
        return st if st is not None and st['t'] >= self.t_start else None

    # ── 1. Escolha ───────────────────────────────────────────────────
    def _page_choose(self):
        self._text('What do you want to calibrate?', font=FONT_HEAD, pady=(14, 6))
        for code, spec in KINDS.items():
            tk.Radiobutton(self.body, text=f'{spec["name"]}  (~{spec["duration_s"]:.0f} s)',
                           value=code, variable=self.kind, command=self._refresh_choose,
                           bg=PANEL, fg=TEXT, activebackground=PANEL, selectcolor=PANEL,
                           font=FONT_LBL, anchor='w', highlightthickness=0, bd=0,
                           cursor='hand2').pack(fill='x', padx=16, pady=2)
        self._what = self._text('', fg=TEXT_MUTED, font=FONT_SMALL, pady=(2, 10))
        self._text('Which sensors? (only connected ones can be chosen)',
                   font=FONT_HEAD, pady=(8, 6))
        row = tk.Frame(self.body, bg=PANEL)
        row.pack(fill='x', padx=16)
        self._sel_cbs = {}
        for sid, var in self.sel.items():
            cb = tk.Checkbutton(row, text=f'ID {sid}', variable=var,
                                command=self._refresh_choose, bg=PANEL, fg=TEXT,
                                activebackground=PANEL, selectcolor=PANEL,
                                font=FONT_LBL, highlightthickness=0, bd=0,
                                cursor='hand2')
            cb.pack(side='left', padx=(0, 12))
            self._sel_cbs[sid] = cb
        self._one_msg = self._text('', fg=TEXT_MUTED, font=FONT_SMALL, pady=(8, 0))
        self._choose_msg = self._text('', fg=WARN, font=FONT_SMALL, pady=(6, 0))
        self._btn('Cancel', self._on_close, BTN_NEUTRAL, side='left')
        self._next = self._btn('Next  →', self._start_queue)
        self._refresh_choose()

    def _refresh_choose(self):
        if self.step != 0:
            return
        live = self._live()
        self._what.config(text=KINDS[self.kind.get()]['what'])
        for sid, cb in self._sel_cbs.items():
            cb.config(state='normal' if sid in live else 'disabled',
                      fg=TEXT if sid in live else TEXT_DIM)
            if sid not in live:
                self.sel[sid].set(False)
        chosen = self._chosen()
        if not live:
            msg = 'No IMU is connected. Turn the sensors on and wait for their LED to turn green.'
        elif not chosen:
            msg = 'Pick at least one sensor.'
        else:
            msg = ''
        self._choose_msg.config(text=msg)
        self._one_msg.config(
            text='The magnetometer is calibrated ONE sensor at a time: the chosen '
                 'ones are done in sequence, each with its own steps.'
            if self.kind.get() == 'MAG' and len(chosen) > 1 else '')
        self._enable(self._next, bool(chosen))

    def _start_queue(self):
        self.queue, self.qi = self._chosen(), 0
        self._show(1)

    # ── 2. Preparo ───────────────────────────────────────────────────
    def _page_prepare(self):
        spec = KINDS[self.kind.get()]
        mag = self.kind.get() == 'MAG'
        of = f'  ({self.qi + 1} of {len(self.queue)})' if mag and len(self.queue) > 1 else ''
        self._text(f'{spec["name"]} — {self._who()}{of}', font=FONT_HEAD, pady=(14, 4))
        if mag:
            self._text('How to move it (watch one full cycle, ~30 s). The orange '
                       'end is the nose, the arrow on top points to it:',
                       fg=TEXT_MUTED, font=FONT_SMALL, pady=(0, 2))
            Sensor3DAnim(self.body, width=320, height=210, demo=True).pack(pady=(0, 4))
        self._text('Confirm each item. Start unlocks when all are checked.',
                   fg=TEXT_MUTED, font=FONT_SMALL, pady=(0, 6))
        self._prep_vars = []
        for item in spec['checklist']:
            v = tk.BooleanVar(self, value=False)
            self._check(self.body, item.format(who=self._who()), v, self._refresh_prepare)
            self._prep_vars.append(v)
        self._prep_msg = self._text('', fg=WARN, font=FONT_SMALL, pady=(12, 0))
        if not (mag and self.qi > 0):   # no meio da fila não volta à escolha
            self._btn('←  Back', lambda: self._show(0), BTN_NEUTRAL, side='left')
        self._start = self._btn('▶  Start recording', self._begin, OK)
        self._refresh_prepare()

    def _refresh_prepare(self):
        if self.step != 1:
            return
        gone = [sid for sid in self._targets() if sid not in self._live()]
        ready = all(v.get() for v in self._prep_vars) and not gone
        self._prep_msg.config(text=f'Sensor(s) {gone} disconnected — check them.'
                              if gone else '')
        self._enable(self._start, ready)

    def _begin(self):
        if self.host._imu_tracking:
            self._prep_msg.config(text='Stop IMU tracking first.')
            return
        self.running = self._targets()
        self.t_start = time.monotonic()
        for sid in self.running:
            self.host._imu_calib_send(self.kind.get(), str(sid))
        self._show(2)

    # ── 3. Coleta ────────────────────────────────────────────────────
    def _page_run(self):
        spec = KINDS[self.kind.get()]
        self._text(f'Recording — {spec["name"]} — {self._who()}', font=FONT_HEAD,
                   pady=(14, 4))
        self._hint = tk.Label(self.body, text='', font=FONT_HEAD, bg=_shade(WARN, 0.8),
                              fg=TEXT, pady=12, wraplength=480)
        self._hint.pack(fill='x', padx=16, pady=(4, 8))
        if self.kind.get() == 'MAG':
            anim = Sensor3DAnim(self.body, width=320, height=210, demo=False)
            anim.set_clock(self.t_start)   # a animação segue a dica, fase a fase
            anim.pack(pady=(0, 6))
        self._bars = {}
        for sid in self.running:
            row = tk.Frame(self.body, bg=PANEL)
            row.pack(fill='x', padx=16, pady=4)
            tk.Label(row, text=f'ID {sid}', font=FONT_LBL, bg=PANEL, fg=TEXT,
                     width=5, anchor='w').pack(side='left')
            cv = tk.Canvas(row, height=18, bg=BTN_NEUTRAL, highlightthickness=0)
            cv.pack(side='left', fill='x', expand=True)
            bar = cv.create_rectangle(0, 0, 0, 18, fill=PRIMARY, outline='')
            lbl = tk.Label(row, text='waiting…', font=FONT_SMALL, bg=PANEL,
                           fg=TEXT_MUTED, width=12, anchor='e')
            lbl.pack(side='left', padx=(8, 0))
            self._bars[sid] = (cv, bar, lbl)
        self._run_msg = self._text('', fg=TEXT_MUTED, font=FONT_SMALL, pady=(12, 0))
        self._btn('✕  Cancel calibration', self._cancel, DANGER, side='left')

    def _refresh_run(self):
        spec = KINDS[self.kind.get()]
        el = time.monotonic() - self.t_start
        tips = spec['during']
        self._hint.config(text=tips[int(el // MAG_PHASE_S) % len(tips)])
        finished, answered = 0, False
        for sid, (cv, bar, lbl) in self._bars.items():
            st = self._status(sid)
            w = max(cv.winfo_width(), 1)
            if st is None:
                continue
            answered = True
            state = st['state']
            if state in ('started', 'progress'):
                p = float(st.get('progress', 0.0))
                cv.coords(bar, 0, 0, w * p / 100.0, 18)
                cv.itemconfig(bar, fill=PRIMARY)
                lbl.config(text=f'{p:.0f}%', fg=TEXT)
            else:
                finished += 1
                ok = state == 'done'
                cv.coords(bar, 0, 0, w, 18)
                cv.itemconfig(bar, fill=OK if ok else DANGER)
                lbl.config(text={'done': '✓ done', 'cancelled': 'cancelled'}.get(
                    state, '✗ failed'), fg=OK if ok else DANGER)
        if not answered and el > ANSWER_TIMEOUT_S:
            self._run_msg.config(text='imu_node is not answering — is it running? '
                                      '(launch with end_effector:=hand imu:=true)',
                                 fg=DANGER)
        else:
            self._run_msg.config(text=f'{el:.0f} s elapsed — expected ~'
                                      f'{spec["duration_s"]:.0f} s (longer if the '
                                      'Wi-Fi drops samples).', fg=TEXT_MUTED)
        if finished == len(self._bars):
            self._show(3)

    def _cancel(self):
        for sid in self.running:
            self.host._imu_calib_send('CANCEL', str(sid))
        self._run_msg.config(text='Cancelling…', fg=WARN)

    # ── 4. Resultado ─────────────────────────────────────────────────
    def _page_result(self):
        spec = KINDS[self.kind.get()]
        self._text(f'Result — {spec["name"]}', font=FONT_HEAD, pady=(14, 8))
        any_ok = False
        for sid in self.running:
            st = self._status(sid) or {'state': 'error', 'msg': 'no answer'}
            if st['state'] == 'done':
                any_ok = True
                txt, fg = f'✓  ID {sid}: saved.', OK
                if 'cov_pct' in st:
                    q, fg = mag_quality(st['cov_pct'])
                    txt += f'  Fit quality: {q} (radius variation {st["cov_pct"]:.1f}%).'
            elif st['state'] == 'cancelled':
                txt, fg = f'–  ID {sid}: cancelled, nothing saved.', TEXT_MUTED
            else:
                txt, fg = f'✗  ID {sid}: failed — {st.get("msg") or "unknown error"}.', DANGER
            self._text(txt, fg=fg)
        if any_ok:
            self._text('The new values are already in use. The orientation of '
                       'the calibrated sensor(s) restarts now:', fg=TEXT_MUTED,
                       font=FONT_SMALL, pady=(12, 4))
            self._check(self.body, f'I put {self._who()[0].lower()}{self._who()[1:]} down and held it '
                        f'STILL for ~{SETTLE_S:.0f} s (LED green again).',
                        self._settled, self._refresh_result)
        remaining = (self.queue[self.qi + 1:] if self.kind.get() == 'MAG' else [])
        if remaining:
            self._text(f'Next in line: sensor ID {remaining[0]}'
                       f'{f" (+{len(remaining) - 1} more)" if len(remaining) > 1 else ""}.',
                       fg=TEXT_MUTED, font=FONT_SMALL, pady=(8, 0))
            self._btn(f'Next sensor: ID {remaining[0]}  →', self._next_in_queue, PRIMARY)
        else:
            self._btn('Finish', self._on_close, PRIMARY)
        if any_ok and self.kind.get() == 'AG':
            self._btn('Calibrate magnetometer next  →', self._mag_next, BTN_NEUTRAL,
                      side='left')
        self._btn('Repeat', self._repeat, BTN_NEUTRAL, side='left')
        self._finish_needs_settle = any_ok
        self._refresh_result()

    def _refresh_result(self):
        if self.step != 3:
            return
        ok = not self._finish_needs_settle or self._settled.get()
        for w in self.foot.winfo_children():
            t = w.cget('text')
            if t == 'Finish' or t.startswith('Next sensor'):
                self._enable(w, ok)

    def _repeat(self):
        self._settled.set(False)
        self._show(1)

    def _next_in_queue(self):
        self.qi += 1
        self._settled.set(False)
        self._show(1)

    def _mag_next(self):
        self.kind.set('MAG')
        self.qi = 0
        self._settled.set(False)
        self._show(1)

    # ── Laço / fechamento ────────────────────────────────────────────
    def _poll(self):
        self._after = None
        try:
            if self.step == 0:
                self._refresh_choose()
            elif self.step == 1:
                self._refresh_prepare()
            elif self.step == 2:
                self._refresh_run()
        except tk.TclError:
            return   # janela fechando
        self._after = self.after(150, self._poll)

    def _on_close(self):
        if self.step == 2:
            self._cancel()
        if self._after is not None:
            self.after_cancel(self._after)
            self._after = None
        self.host._imu_wizard = None
        self.destroy()
