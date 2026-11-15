"""ROS-free bookkeeping for the latency monitor: rate, gaps, ages and the twist_mux winner.

Times are seconds (float). Everything is reset at each report so a report covers one window.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class WindowStats:
    """Arrival times and optional ages of one topic over a report window."""
    arrivals: List[float] = field(default_factory=list)
    ages: List[float] = field(default_factory=list)
    last_arrival: Optional[float] = None   # kept across windows so the first gap is measured
    gaps: List[float] = field(default_factory=list)

    def add(self, now: float, age: Optional[float] = None) -> None:
        if self.last_arrival is not None:
            self.gaps.append(now - self.last_arrival)
        self.last_arrival = now
        self.arrivals.append(now)
        if age is not None:
            self.ages.append(age)

    def report(self, window: float, now: Optional[float] = None) -> Dict[str, float]:
        """Rate over `window`, longest gap (including silence until `now`), age percentiles."""
        gaps = list(self.gaps)
        if now is not None and self.last_arrival is not None:
            gaps.append(now - self.last_arrival)   # a topic that went silent shows up here
        out = {
            'count': float(len(self.arrivals)),
            'hz': len(self.arrivals) / window if window > 0 else float('nan'),
            'max_gap': max(gaps) if gaps else float('nan'),
            'age_p50': float('nan'), 'age_p95': float('nan'), 'age_max': float('nan'),
        }
        if self.ages:
            a = np.asarray(self.ages)
            out['age_p50'] = float(np.percentile(a, 50))
            out['age_p95'] = float(np.percentile(a, 95))
            out['age_max'] = float(a.max())
        return out

    def reset(self) -> None:
        self.arrivals.clear()
        self.ages.clear()
        self.gaps.clear()


def parse_pairs(entries: Sequence[str]) -> List[Tuple[str, str]]:
    """'cmd_topic=source_topic' strings -> pairs. A missing or empty source gives ''."""
    pairs = []
    for entry in entries:
        entry = entry.strip()
        if not entry:
            continue
        topic, _, source = entry.partition('=')
        pairs.append((topic.strip(), source.strip()))
    return pairs


def twist_equal(a: Tuple[float, ...], b: Tuple[float, ...], tol: float = 1e-6) -> bool:
    return len(a) == len(b) and all(abs(x - y) <= tol for x, y in zip(a, b))


def mux_winner(output: Tuple[float, ...], t_out: float,
               history: Dict[str, Sequence[Tuple[Tuple[float, ...], float]]],
               max_age: float, grace: float) -> str:
    """Name of the input that produced a mux output seen at `t_out`.

    `history` maps input name -> recent (twist values, arrival time). An input matches when one of
    its messages has the same values and arrived within [t_out - max_age, t_out + grace]: the mux
    output can reach us before the input message itself, so call this `grace` seconds later.
    The match closest in time wins. Returns '?' when nothing matches.
    """
    best, best_dt = '?', float('inf')
    for name, entries in history.items():
        for values, arrival in entries:
            if not (t_out - max_age <= arrival <= t_out + grace) or not twist_equal(values, output):
                continue
            dt = abs(arrival - t_out)
            if dt < best_dt:
                best, best_dt = name, dt
    return best
