"""Token-clip selection and gait-cycle planning for SONIC remote control."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable

import numpy as np


TokenMotionIntent = tuple[str, str]


def velocity_to_token_intent(
    *,
    forward_mps: float,
    left_mps: float,
    yaw_rate_rad_s: float,
    turn_error_rad: float | None = None,
    linear_epsilon: float = 0.03,
    yaw_epsilon: float = 0.03,
    normal_forward_threshold_mps: float = 0.55,
) -> TokenMotionIntent | None:
    """Map a body Twist to the same one-key-at-a-time contract as the GUI.

    G1Pilot already turns in place before translating, but yaw is deliberately
    given priority here as a second safety boundary.  A combined velocity must
    never blend two recorded pose-token primitives.
    """

    values = (forward_mps, left_mps, yaw_rate_rad_s)
    if not all(np.isfinite(float(value)) for value in values):
        raise ValueError("navigation velocity contains a non-finite value")
    if linear_epsilon < 0.0 or yaw_epsilon < 0.0:
        raise ValueError("velocity deadbands must be non-negative")
    if normal_forward_threshold_mps <= 0.0:
        raise ValueError("normal forward threshold must be positive")

    if turn_error_rad is not None and not np.isfinite(float(turn_error_rad)):
        raise ValueError("navigation turn error contains a non-finite value")
    if abs(yaw_rate_rad_s) >= yaw_epsilon:
        error_deg = abs(float(np.degrees(
            yaw_rate_rad_s if turn_error_rad is None else turn_error_rad
        )))
        if error_deg < 45.0:
            turn_variant = "30"
        elif error_deg < 75.0:
            turn_variant = "60"
        else:
            turn_variant = "90"
        return ("q" if yaw_rate_rad_s > 0.0 else "e", turn_variant)
    if abs(left_mps) >= linear_epsilon:
        return ("a" if left_mps > 0.0 else "d", "normal")
    if abs(forward_mps) >= linear_epsilon:
        if forward_mps < 0.0:
            return ("s", "normal")
        speed = (
            "normal"
            if forward_mps >= normal_forward_threshold_mps
            else "slow"
        )
        return ("w", speed)
    return None


def velocity_to_continuous_token_intent(
    *,
    forward_mps: float,
    left_mps: float,
    yaw_rate_rad_s: float,
    turn_error_rad: float | None = None,
    forward_mode: str = "normal",
    normal_forward_threshold_mps: float = 0.20,
    micro_turn_threshold_deg: float = 15.0,
    slow_turn_threshold_deg: float = 30.0,
    linear_epsilon: float = 0.03,
    yaw_epsilon: float = 0.03,
) -> TokenMotionIntent | None:
    """Map navigation velocity to the continuous v4 keyboard primitives."""
    values = (forward_mps, left_mps, yaw_rate_rad_s)
    if not all(np.isfinite(float(value)) for value in values):
        raise ValueError("navigation velocity contains a non-finite value")
    if forward_mode not in ("slow", "normal"):
        raise ValueError("forward_mode must be slow or normal")
    if not 0.0 < micro_turn_threshold_deg < slow_turn_threshold_deg:
        raise ValueError(
            "turn thresholds must satisfy 0 < micro < slow"
        )
    if normal_forward_threshold_mps <= linear_epsilon:
        raise ValueError(
            "normal_forward_threshold_mps must exceed the linear deadband"
        )
    if turn_error_rad is not None and not np.isfinite(float(turn_error_rad)):
        raise ValueError("navigation turn error contains a non-finite value")

    if abs(yaw_rate_rad_s) >= yaw_epsilon:
        error_deg = abs(
            float(
                np.degrees(
                    yaw_rate_rad_s if turn_error_rad is None else turn_error_rad
                )
            )
        )
        if error_deg <= micro_turn_threshold_deg:
            variant = "micro"
        elif error_deg <= slow_turn_threshold_deg:
            variant = "slow"
        else:
            variant = "normal"
        return ("q" if yaw_rate_rad_s > 0.0 else "e", variant)
    if abs(left_mps) >= linear_epsilon:
        return ("a" if left_mps > 0.0 else "d", "continuous")
    if abs(forward_mps) >= linear_epsilon:
        if forward_mps < 0.0:
            return ("s", "continuous")
        selected_forward_mode = forward_mode
        if (
            forward_mode == "normal"
            and forward_mps < normal_forward_threshold_mps
        ):
            selected_forward_mode = "slow"
        return ("w", selected_forward_mode)
    return None


class ContinuousVelocityIntentSelector:
    """Stateful velocity mapper that prevents turn-token interruptions.

    Navigation yaw is closed outside this class by the IMU-fused odometry
    controller.  This selector only chooses which validated continuous token
    primitive should realize that command. By default the turn speed selected
    at turn entry is latched until yaw rate returns to zero or direction
    changes. This prevents a normal-to-slow variant change from making the
    sequencer blend through stand in the middle of one IMU-closed turn.
    """

    def __init__(
        self,
        *,
        forward_mode: str = "normal",
        micro_turn_threshold_deg: float = 15.0,
        slow_turn_threshold_deg: float = 30.0,
        turn_speed_hysteresis_deg: float = 5.0,
        latch_turn_variant: bool = True,
        normal_forward_threshold_mps: float = 0.20,
        linear_epsilon: float = 0.03,
        yaw_epsilon: float = 0.03,
    ) -> None:
        if forward_mode not in ("slow", "normal"):
            raise ValueError("forward_mode must be slow or normal")
        if not 0.0 < micro_turn_threshold_deg < slow_turn_threshold_deg:
            raise ValueError(
                "turn thresholds must satisfy 0 < micro < slow"
            )
        if not 0.0 <= turn_speed_hysteresis_deg < slow_turn_threshold_deg:
            raise ValueError(
                "turn_speed_hysteresis_deg must be non-negative and smaller "
                "than slow_turn_threshold_deg"
            )
        if linear_epsilon < 0.0 or yaw_epsilon < 0.0:
            raise ValueError("velocity deadbands must be non-negative")
        if normal_forward_threshold_mps <= linear_epsilon:
            raise ValueError(
                "normal_forward_threshold_mps must exceed the linear deadband"
            )
        self.forward_mode = forward_mode
        self.micro_turn_threshold_deg = float(micro_turn_threshold_deg)
        self.slow_turn_threshold_deg = float(slow_turn_threshold_deg)
        self.turn_speed_hysteresis_deg = float(turn_speed_hysteresis_deg)
        self.latch_turn_variant = bool(latch_turn_variant)
        self.linear_epsilon = float(linear_epsilon)
        self.yaw_epsilon = float(yaw_epsilon)
        self.normal_forward_threshold_mps = float(
            normal_forward_threshold_mps
        )
        self._turn_key: str | None = None
        self._turn_variant: str | None = None

    def reset(self) -> None:
        self._turn_key = None
        self._turn_variant = None

    def select(
        self,
        *,
        forward_mps: float,
        left_mps: float,
        yaw_rate_rad_s: float,
        turn_error_rad: float | None = None,
    ) -> TokenMotionIntent | None:
        values = (forward_mps, left_mps, yaw_rate_rad_s)
        if not all(np.isfinite(float(value)) for value in values):
            raise ValueError("navigation velocity contains a non-finite value")
        if turn_error_rad is not None and not np.isfinite(float(turn_error_rad)):
            raise ValueError("navigation turn error contains a non-finite value")

        if abs(yaw_rate_rad_s) < self.yaw_epsilon:
            self.reset()
            return velocity_to_continuous_token_intent(
                forward_mps=forward_mps,
                left_mps=left_mps,
                yaw_rate_rad_s=yaw_rate_rad_s,
                turn_error_rad=turn_error_rad,
                forward_mode=self.forward_mode,
                micro_turn_threshold_deg=self.micro_turn_threshold_deg,
                slow_turn_threshold_deg=self.slow_turn_threshold_deg,
                normal_forward_threshold_mps=(
                    self.normal_forward_threshold_mps
                ),
                linear_epsilon=self.linear_epsilon,
                yaw_epsilon=self.yaw_epsilon,
            )

        key = "q" if yaw_rate_rad_s > 0.0 else "e"
        error_deg = abs(
            float(
                np.degrees(
                    yaw_rate_rad_s if turn_error_rad is None else turn_error_rad
                )
            )
        )
        micro_lower = max(
            0.0,
            self.micro_turn_threshold_deg - self.turn_speed_hysteresis_deg,
        )
        micro_upper = self.micro_turn_threshold_deg + self.turn_speed_hysteresis_deg
        slow_lower = self.slow_turn_threshold_deg - self.turn_speed_hysteresis_deg
        slow_upper = self.slow_turn_threshold_deg + self.turn_speed_hysteresis_deg
        if key != self._turn_key or self._turn_variant is None:
            if error_deg <= self.micro_turn_threshold_deg:
                variant = "micro"
            elif error_deg <= self.slow_turn_threshold_deg:
                variant = "slow"
            else:
                variant = "normal"
        elif self.latch_turn_variant:
            variant = self._turn_variant
        else:
            variant = self._turn_variant
            if variant == "normal" and error_deg <= slow_lower:
                variant = "micro" if error_deg <= micro_lower else "slow"
            elif variant == "slow":
                if error_deg >= slow_upper:
                    variant = "normal"
                elif error_deg <= micro_lower:
                    variant = "micro"
            elif variant == "micro" and error_deg >= micro_upper:
                variant = "normal" if error_deg >= slow_upper else "slow"
        self._turn_key = key
        self._turn_variant = variant
        return key, variant


@dataclass(frozen=True)
class ClipQuality:
    max_gap_s: float
    end_leg_velocity_rms: float
    end_base_ang_velocity_rms: float

    @property
    def usable_for_loop(self) -> bool:
        return (
            self.max_gap_s <= 0.10
            and self.end_leg_velocity_rms <= 0.30
            and self.end_base_ang_velocity_rms <= 0.20
        )

    @property
    def score(self) -> float:
        return (
            8.0 * self.max_gap_s
            + self.end_leg_velocity_rms
            + self.end_base_ang_velocity_rms
        )


@dataclass(frozen=True)
class LoopPlan:
    enter_start: int
    loop_start: int
    loop_end: int
    exit_start: int
    phase_error: float
    exit_phase_error: float

    def validate(self, frame_count: int) -> None:
        if not (
            0 <= self.enter_start < self.loop_start < self.loop_end
            <= self.exit_start < frame_count
        ):
            raise ValueError(f"invalid loop plan for {frame_count} frames: {self}")


@dataclass
class TokenClip:
    path: Path
    primitive: str
    target_yaw_deg: float | None
    timestamps: np.ndarray
    tokens: np.ndarray
    body_q: np.ndarray
    body_dq: np.ndarray
    base_ang_vel: np.ndarray
    left_hand_joints: np.ndarray
    right_hand_joints: np.ndarray
    phase_boundaries: dict[str, int] | None
    source_state: str | None
    target_state: str | None
    events: tuple[dict[str, object], ...]
    measured_yaw_change_deg: float | None
    duration_s: float | None

    @classmethod
    def load(cls, path: Path) -> "TokenClip":
        metadata = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
        with np.load(path / "frames.npz") as arrays:
            values = {key: arrays[key].copy() for key in arrays.files}
        tokens = np.asarray(values["motion_token"], dtype=np.float32)
        if tokens.ndim != 2 or tokens.shape[1] != 64 or not np.isfinite(tokens).all():
            raise ValueError(f"{path.name}: expected finite motion_token[N,64]")
        frame_count = len(tokens)
        required = {
            "timestamp_unix": 1,
            "body_q": 29,
            "body_dq": 29,
            "base_ang_vel": 3,
            "left_hand_joints": 7,
            "right_hand_joints": 7,
        }
        for key, width in required.items():
            array = np.asarray(values[key])
            if key == "timestamp_unix":
                if array.shape != (frame_count,):
                    raise ValueError(f"{path.name}: invalid {key} shape {array.shape}")
            elif array.shape != (frame_count, width):
                raise ValueError(f"{path.name}: invalid {key} shape {array.shape}")
            if not np.isfinite(array).all():
                raise ValueError(f"{path.name}: {key} contains non-finite values")
        return cls(
            path=path,
            primitive=str(metadata["primitive"]),
            target_yaw_deg=metadata.get("target_yaw_deg"),
            timestamps=np.asarray(values["timestamp_unix"], dtype=np.float64),
            tokens=tokens,
            body_q=np.asarray(values["body_q"], dtype=np.float32),
            body_dq=np.asarray(values["body_dq"], dtype=np.float32),
            base_ang_vel=np.asarray(values["base_ang_vel"], dtype=np.float32),
            left_hand_joints=np.asarray(values["left_hand_joints"], dtype=np.float32),
            right_hand_joints=np.asarray(values["right_hand_joints"], dtype=np.float32),
            phase_boundaries=(
                {key: int(value) for key, value in metadata["phase_boundaries"].items()}
                if isinstance(metadata.get("phase_boundaries"), dict)
                else None
            ),
            source_state=metadata.get("source_state"),
            target_state=metadata.get("target_state"),
            events=tuple(metadata.get("events", ())),
            measured_yaw_change_deg=(
                float(metadata["measured_yaw_change_deg"])
                if metadata.get("measured_yaw_change_deg") is not None
                else None
            ),
            duration_s=(
                float(metadata["duration_s"])
                if metadata.get("duration_s") is not None
                else None
            ),
        )

    @property
    def frame_count(self) -> int:
        return len(self.tokens)

    def quality(self, rate_hz: float = 50.0) -> ClipQuality:
        gaps = np.diff(self.timestamps)
        window = max(1, min(self.frame_count, int(round(0.75 * rate_hz))))
        return ClipQuality(
            max_gap_s=float(gaps.max()) if gaps.size else 0.0,
            end_leg_velocity_rms=float(np.sqrt(np.mean(self.body_dq[-window:, :12] ** 2))),
            end_base_ang_velocity_rms=float(
                np.sqrt(np.mean(self.base_ang_vel[-window:] ** 2))
            ),
        )

    def fixed_hands(self) -> tuple[np.ndarray, np.ndarray]:
        """Return robust constant hands, excluding marker edges."""
        margin = min(25, max(1, self.frame_count // 5))
        region = slice(margin, max(margin + 1, self.frame_count - margin))
        return (
            np.median(self.left_hand_joints[region], axis=0).astype(np.float32),
            np.median(self.right_hand_joints[region], axis=0).astype(np.float32),
        )


def load_clips(root: Path) -> list[TokenClip]:
    if not root.is_dir():
        raise FileNotFoundError(f"token dataset does not exist: {root}")
    clips = []
    for path in sorted(item for item in root.iterdir() if item.is_dir()):
        if (path / "metadata.json").is_file() and (path / "frames.npz").is_file():
            clips.append(TokenClip.load(path))
    if not clips:
        raise ValueError(f"no token clips found under {root}")
    return clips


def select_best_clip(
    clips: Iterable[TokenClip],
    primitive: str,
    *,
    target_yaw_deg: float | None = None,
    require_loop_quality: bool = False,
) -> TokenClip:
    candidates = [clip for clip in clips if clip.primitive == primitive]
    if target_yaw_deg is not None:
        candidates = [
            clip
            for clip in candidates
            if clip.target_yaw_deg is not None
            and abs(float(clip.target_yaw_deg) - target_yaw_deg) < 0.5
        ]
    if not candidates:
        raise ValueError(f"no clip for primitive={primitive}, target_yaw_deg={target_yaw_deg}")
    usable = [clip for clip in candidates if clip.quality().usable_for_loop]
    if require_loop_quality and not usable:
        details = ", ".join(
            f"{clip.path.name}:{clip.quality()}" for clip in candidates
        )
        raise ValueError(f"no loop-quality clip for {primitive}: {details}")
    pool = usable if usable else candidates
    return min(pool, key=lambda clip: clip.quality().score)


def select_best_loop_clip(
    clips: Iterable[TokenClip], primitive: str
) -> tuple[TokenClip, LoopPlan]:
    """Select a clean clip using both recording quality and loop continuity."""
    candidates = [
        clip
        for clip in clips
        if clip.primitive == primitive and clip.quality().usable_for_loop
    ]
    if not candidates:
        raise ValueError(f"no loop-quality clip for {primitive}")
    planned: list[tuple[float, TokenClip, LoopPlan]] = []
    for clip in candidates:
        plan = build_loop_plan(clip)
        # A discontinuity in the latent gait phase is more dangerous than a
        # small difference in the already-acceptable recording tail metrics.
        score = plan.phase_error + plan.exit_phase_error + 0.10 * clip.quality().score
        planned.append((score, clip, plan))
    _, clip, plan = min(planned, key=lambda item: item[0])
    return clip, plan


def select_best_phase_matched_loop_clip(
    clips: Iterable[TokenClip], primitive: str
) -> tuple[TokenClip, LoopPlan]:
    """Select a short phase-matched cycle for visibly periodic long clips.

    Most recorded v2 locomotion benefits from retaining its full steady
    section. Running is different: replaying the complete several-second
    section makes every hold look like another one-shot clip. Its arm/gait
    entry is played once, then only a phase-matched gait cycle is repeated.
    """
    candidates = [
        clip
        for clip in clips
        if clip.primitive == primitive and clip.quality().usable_for_loop
    ]
    if not candidates:
        raise ValueError(f"no loop-quality clip for {primitive}")
    planned: list[tuple[float, TokenClip, LoopPlan]] = []
    for clip in candidates:
        plan = build_loop_plan(clip, preserve_recorded_section=False)
        if primitive == "forward_run":
            arm_activity = _smooth(
                np.sqrt(np.mean(clip.body_dq[:, 15:29] ** 2, axis=1)),
                window=5,
            )
            sustained_arm_onset = next(
                (
                    frame
                    for frame in range(0, max(0, plan.loop_start - 10))
                    if frame + 10 <= clip.frame_count
                    and bool(np.all(arm_activity[frame : frame + 10] >= 0.05))
                ),
                plan.enter_start,
            )
            # Retain 0.1 s of quiet pose before the first deliberate arm
            # motion, but remove the otherwise visible idle wait on key-down.
            run_enter_start = max(
                plan.enter_start, sustained_arm_onset - int(round(0.10 * 50.0))
            )
            plan = LoopPlan(
                enter_start=run_enter_start,
                loop_start=plan.loop_start,
                loop_end=plan.loop_end,
                exit_start=plan.exit_start,
                phase_error=plan.phase_error,
                exit_phase_error=plan.exit_phase_error,
            )
            plan.validate(clip.frame_count)
        heading_drift = (
            abs(float(clip.measured_yaw_change_deg)) / float(clip.duration_s)
            if clip.measured_yaw_change_deg is not None
            and clip.duration_s is not None
            and clip.duration_s > 0.0
            else 0.0
        )
        score = (
            plan.phase_error
            + plan.exit_phase_error
            + 0.20 * heading_drift
            + 0.05 * clip.quality().score
        )
        planned.append((score, clip, plan))
    _, clip, plan = min(planned, key=lambda item: item[0])
    return clip, plan


def select_best_real_loop_clip(
    clips: Iterable[TokenClip], primitive: str
) -> tuple[TokenClip, LoopPlan]:
    """Rank continuous clips using measurements recorded on the real G1.

    Non-turning commands prioritize heading preservation after the usual data
    quality gates.  Continuous turns prioritize direction, a complete turn,
    and loop continuity.  This profile deliberately does not use MuJoCo drift
    to choose a real-robot command.
    """
    candidates = [
        clip
        for clip in clips
        if clip.primitive == primitive
        and clip.quality().usable_for_loop
        and clip.measured_yaw_change_deg is not None
        and clip.duration_s is not None
        and clip.duration_s > 0.0
    ]
    if not candidates:
        return select_best_loop_clip(clips, primitive)

    planned: list[tuple[float, TokenClip, LoopPlan]] = []
    turning = primitive in ("turn_left_continuous", "turn_right_continuous")
    expected_sign = 1.0 if primitive == "turn_left_continuous" else -1.0
    for clip in candidates:
        plan = build_loop_plan(clip)
        yaw = float(clip.measured_yaw_change_deg)
        if turning:
            if yaw * expected_sign <= 0.0:
                continue
            # Recordings were collected as roughly one full continuous turn.
            turn_completion_error = abs(abs(yaw) - 360.0) / 360.0
            score = (
                2.0 * turn_completion_error
                + plan.phase_error
                + 0.05 * clip.quality().score
            )
        else:
            heading_drift_deg_s = abs(yaw) / float(clip.duration_s)
            score = (
                heading_drift_deg_s
                + 0.25 * plan.phase_error
                + 0.05 * clip.quality().score
            )
        planned.append((score, clip, plan))
    if not planned:
        raise ValueError(f"no real-robot-qualified loop clip for {primitive}")
    _, clip, plan = min(planned, key=lambda item: item[0])
    return clip, plan


def select_turn_loop_clip(
    clips: Iterable[TokenClip], primitive: str
) -> tuple[TokenClip, LoopPlan]:
    """Choose a multi-angle turn whose middle is sustained, one-way rotation."""
    if primitive not in ("turn_left", "turn_right"):
        raise ValueError(f"not a turn primitive: {primitive}")
    planned: list[tuple[float, TokenClip, LoopPlan]] = []
    for clip in clips:
        if clip.primitive != primitive or clip.target_yaw_deg is None:
            continue
        plan = build_loop_plan(
            clip, min_cycle_s=0.50, max_cycle_s=1.75
        )
        angular_velocity = clip.base_ang_vel[plan.loop_start : plan.loop_end, 2]
        timestamps = clip.timestamps[plan.loop_start : plan.loop_end]
        if len(angular_velocity) < 2:
            continue
        yaw_change_deg = float(
            np.degrees(
                np.sum(angular_velocity[:-1] * np.diff(timestamps))
            )
        )
        expected_sign = float(np.sign(clip.target_yaw_deg))
        minimum_rotation = max(20.0, 0.55 * abs(float(clip.target_yaw_deg)))
        if yaw_change_deg * expected_sign < minimum_rotation:
            continue
        direction_consistency = float(
            np.mean(angular_velocity * expected_sign > 0.20)
        )
        # Prefer a phase-continuous loop, consistently rotating in the
        # requested direction. Larger-angle recordings usually contain the
        # longest stable middle, but angle alone is not used as the selector.
        score = (
            plan.phase_error
            + plan.exit_phase_error
            + 2.0 * (1.0 - direction_consistency)
            + 0.05 * clip.quality().score
        )
        planned.append((score, clip, plan))
    if not planned:
        raise ValueError(f"no sustained rotating clip for {primitive}")
    _, clip, plan = min(planned, key=lambda item: item[0])
    return clip, plan


def _smooth(values: np.ndarray, window: int = 11) -> np.ndarray:
    window = min(window, len(values))
    if window <= 1:
        return values.copy()
    return np.convolve(values, np.ones(window) / window, mode="same")


def build_loop_plan(
    clip: TokenClip,
    *,
    rate_hz: float = 50.0,
    min_cycle_s: float = 0.50,
    max_cycle_s: float = 1.75,
    preserve_recorded_section: bool = True,
) -> LoopPlan:
    """Find phase-matched enter/loop/exit boundaries from robot state and token."""
    if clip.frame_count < int(rate_hz * 3.0):
        raise ValueError(f"{clip.path.name}: clip is too short for cycle extraction")
    n = clip.frame_count
    q = clip.body_q[:, :12].astype(np.float64)
    dq = clip.body_dq[:, :12].astype(np.float64)
    token = clip.tokens.astype(np.float64)
    activity = _smooth(np.sqrt(np.mean(dq * dq, axis=1)))
    boundaries = clip.phase_boundaries
    if boundaries is not None:
        motion_start = int(boundaries["motion_start"])
        steady_start = int(boundaries["loop_start"])
        steady_end = int(boundaries["loop_end"])
        motion_end = int(boundaries["motion_end"])
        if not 0 <= motion_start < steady_start < steady_end < motion_end <= n:
            raise ValueError(f"{clip.path.name}: invalid recorded phase boundaries")
        search_lo = steady_start
        search_hi = steady_end
    else:
        motion_start = -1
        steady_end = -1
        motion_end = -1
        search_lo = int(0.18 * n)
        search_hi = int(0.70 * n)
    if search_hi - search_lo < int(max_cycle_s * rate_hz):
        raise ValueError(f"{clip.path.name}: insufficient steady-motion search range")
    active_threshold = float(np.quantile(activity[search_lo:search_hi], 0.30))
    q_scale = np.std(q[search_lo:search_hi], axis=0) + 0.05
    dq_scale = np.std(dq[search_lo:search_hi], axis=0) + 0.20
    token_scale = np.std(token[search_lo:search_hi], axis=0) + 0.05

    def phase_distance(first: int, second: int) -> float:
        return float(
            np.mean(((q[first] - q[second]) / q_scale) ** 2)
            + 0.25 * np.mean(((dq[first] - dq[second]) / dq_scale) ** 2)
            + 0.50 * np.mean(((token[first] - token[second]) / token_scale) ** 2)
        )

    min_frames = max(2, int(round(min_cycle_s * rate_hz)))
    max_frames = max(min_frames, int(round(max_cycle_s * rate_hz)))
    best: tuple[float, int, int] | None = None
    if boundaries is not None and preserve_recorded_section:
        # A v2 stable section is already a continuous operator command.  Keep
        # almost all of it, choosing matching phases only near its two ends.
        # A short 0.5--1.7 s gait loop made held control visibly periodic and
        # repeatedly amplified small heading errors in the recording.
        edge_window = max(min_frames, int(round(rate_hz)))
        first_stop = min(search_hi - min_frames, search_lo + edge_window)
        second_start = max(search_lo + min_frames, search_hi - edge_window)
        for first in range(search_lo, first_stop):
            for second in range(max(second_start, first + min_frames), search_hi):
                score = phase_distance(first, second)
                if best is None or score < best[0]:
                    best = (score, first, second)
    else:
        first_stop = search_hi - min_frames
        if boundaries is not None:
            # Enter a recorded steady run promptly instead of waiting until a
            # globally optimal cycle several seconds into the clip. The cycle
            # may end later, but its first phase must lie in the first 2.5 s
            # of the operator-annotated steady section.
            first_stop = min(first_stop, search_lo + int(round(2.5 * rate_hz)))
        for first in range(search_lo, first_stop):
            if activity[first] < active_threshold:
                continue
            for cycle_frames in range(min_frames, max_frames + 1):
                second = first + cycle_frames
                if second >= search_hi or activity[second] < active_threshold:
                    continue
                score = phase_distance(first, second)
                if best is None or score < best[0]:
                    best = (score, first, second)
    if best is None:
        raise ValueError(f"{clip.path.name}: no phase-matched loop found")
    phase_error, loop_start, loop_end = best

    if boundaries is not None:
        exit_lo = max(loop_end + 5, steady_end - max_frames)
        exit_hi = max(exit_lo + 1, motion_end)
    else:
        exit_lo = max(loop_end + 5, int(0.68 * n))
        exit_hi = max(exit_lo + 1, int(0.93 * n))
    exit_matches = [
        (phase_distance(loop_start, frame), frame)
        for frame in range(exit_lo, min(exit_hi, n - 1))
    ]
    if not exit_matches:
        raise ValueError(f"{clip.path.name}: no exit phase candidates")
    exit_phase_error, exit_start = min(exit_matches)

    if boundaries is not None:
        enter_start = max(0, min(motion_start - 10, loop_start - 1))
    else:
        above = activity > 0.20
        onset = next(
            (
                frame
                for frame in range(0, max(1, loop_start - 10))
                if frame + 10 <= n and bool(np.all(above[frame : frame + 10]))
            ),
            max(0, loop_start - int(rate_hz)),
        )
        enter_start = max(0, min(onset - int(round(0.30 * rate_hz)), loop_start - 1))
    plan = LoopPlan(
        enter_start=enter_start,
        loop_start=loop_start,
        loop_end=loop_end,
        exit_start=exit_start,
        phase_error=phase_error,
        exit_phase_error=exit_phase_error,
    )
    plan.validate(n)
    return plan


class ContinuousTokenNavigationSequencer:
    """Asynchronous navigation player for the validated continuous v3 set.

    A held ROS command repeats only the selected steady range. Command removal,
    stale odometry, goal arrival, and direction changes begin a bounded blend
    from the current token to stand instead of finishing a multi-second clip.
    """

    _VALID_VARIANTS = {
        "w": frozenset(("slow", "normal")),
        "s": frozenset(("continuous",)),
        "a": frozenset(("continuous",)),
        "d": frozenset(("continuous",)),
        "q": frozenset(("micro", "slow", "normal")),
        "e": frozenset(("micro", "slow", "normal")),
    }

    def __init__(
        self,
        clips: Iterable[TokenClip],
        *,
        rate_hz: float = 50.0,
        stop_blend_s: float = 0.30,
    ) -> None:
        clips = list(clips)
        if rate_hz <= 0.0 or stop_blend_s <= 0.0:
            raise ValueError("rate_hz and stop_blend_s must be positive")
        self.stand = select_best_clip(clips, "stand")
        self.motion = {
            ("w", "slow"): select_best_loop_clip(clips, "forward_slow"),
            ("w", "normal"): select_best_loop_clip(clips, "forward_normal"),
            ("s", "continuous"): select_best_loop_clip(
                clips, "backward_continuous"
            ),
            ("a", "continuous"): select_best_loop_clip(
                clips, "strafe_left_continuous"
            ),
            ("d", "continuous"): select_best_loop_clip(
                clips, "strafe_right_continuous"
            ),
            ("q", "slow"): select_best_loop_clip(
                clips, "turn_left_continuous"
            ),
            ("e", "slow"): select_best_loop_clip(
                clips, "turn_right_continuous"
            ),
            ("q", "micro"): select_best_loop_clip(
                clips, "turn_left_micro_continuous"
            ),
            ("e", "micro"): select_best_loop_clip(
                clips, "turn_right_micro_continuous"
            ),
            ("q", "normal"): select_best_loop_clip(clips, "turn_left_normal"),
            ("e", "normal"): select_best_loop_clip(clips, "turn_right_normal"),
        }
        self.left_hand, self.right_hand = self.stand.fixed_hands()
        self.hold = self.stand.tokens[-1].copy()
        self.rate_hz = float(rate_hz)
        self.blend_frames = max(1, int(round(stop_blend_s * rate_hz)))
        self.requested: TokenMotionIntent | None = None
        self.active: TokenMotionIntent | None = None
        self.phase = "idle"
        self._clip: TokenClip | None = None
        self._plan: LoopPlan | None = None
        self._index = 0
        self._end = 0
        self._blend_start: np.ndarray | None = None
        self._blend_index = 0

    @property
    def state(self) -> str:
        if self.phase == "blend":
            suffix = "idle" if self.active is None else "-".join(self.active)
            return f"blend-{suffix}"
        if self.active is None:
            return "idle"
        return f"{self.active[0]}-{self.active[1]}:{self.phase}"

    @property
    def is_idle(self) -> bool:
        return self.active is None and self.requested is None and self.phase == "idle"

    def request(self, intent: TokenMotionIntent | None) -> None:
        if intent is None:
            self.requested = None
            return
        normalized = (str(intent[0]).lower(), str(intent[1]).lower())
        key, variant = normalized
        if key not in self._VALID_VARIANTS or variant not in self._VALID_VARIANTS[key]:
            raise ValueError(f"unsupported continuous token intent: {normalized}")
        self.requested = normalized

    def _start_requested(self) -> None:
        if self.requested is None:
            self.active = None
            self.phase = "idle"
            return
        self.active = self.requested
        self._clip, self._plan = self.motion[self.active]
        self.phase = "enter"
        self._index = self._plan.enter_start
        self._end = self._plan.loop_start

    def _start_blend(self) -> None:
        self.phase = "blend"
        self._blend_start = self.hold.copy()
        self._blend_index = 0

    def _blend_step(self) -> np.ndarray:
        assert self._blend_start is not None
        self._blend_index += 1
        alpha = min(1.0, self._blend_index / self.blend_frames)
        target = self.stand.tokens[-1]
        token = ((1.0 - alpha) * self._blend_start + alpha * target).astype(
            np.float32
        )
        self.hold = token
        if self._blend_index >= self.blend_frames:
            self.active = None
            self._clip = None
            self._plan = None
            self.phase = "idle"
            self._blend_start = None
            self._start_requested()
        return token

    def step(self) -> np.ndarray:
        if self.phase == "blend":
            return self._blend_step()
        if self.active is None:
            self._start_requested()
            if self.active is None:
                return self.hold
        if self.requested != self.active:
            self._start_blend()
            return self._blend_step()

        assert self._clip is not None and self._plan is not None
        if self._index >= self._end:
            self.phase = "loop"
            self._index = self._plan.loop_start
            self._end = self._plan.loop_end
        token = self._clip.tokens[self._index]
        self._index += 1
        self.hold = token
        return token


class TokenPrimitiveSequencer:
    """Emit one recorded token per tick for an asynchronous navigation intent.

    The keyboard player is intentionally blocking because it owns the input
    loop.  Navigation callbacks are asynchronous, so this sequencer preserves
    the exact clip boundaries while advancing only one 50 Hz frame at a time.
    Direction changes are queued until the current gait segment reaches a safe
    boundary. Forward uses a phase-matched exit; angle-qualified turns execute
    complete captured clips before the next odometry-based decision.
    """

    _VALID_KEYS = frozenset(("w", "s", "a", "d", "q", "e"))

    def __init__(self, clips: Iterable[TokenClip]) -> None:
        clips = list(clips)
        self.stand = select_best_clip(clips, "stand")
        self.forward = {
            "slow": select_best_loop_clip(clips, "forward_slow"),
            "normal": select_best_loop_clip(clips, "forward_normal"),
        }
        self.turn = {
            "q": {
                variant: select_best_clip(
                    clips, "turn_left", target_yaw_deg=float(variant)
                )
                for variant in ("30", "60", "90")
            },
            "e": {
                variant: select_best_clip(
                    clips, "turn_right", target_yaw_deg=-float(variant)
                )
                for variant in ("30", "60", "90")
            },
        }
        self.discrete = {
            "s": select_best_clip(clips, "backward"),
            "a": select_best_clip(clips, "strafe_left"),
            "d": select_best_clip(clips, "strafe_right"),
        }
        self.left_hand, self.right_hand = self.stand.fixed_hands()
        self.hold = self.stand.tokens[-1].copy()
        self.requested: TokenMotionIntent | None = None
        self.active: TokenMotionIntent | None = None
        self.phase = "idle"
        self._clip: TokenClip | None = None
        self._plan: LoopPlan | None = None
        self._index = 0
        self._end = 0

    @property
    def state(self) -> str:
        if self.active is None:
            return "idle"
        key, speed = self.active
        suffix = f"-{speed}" if key in ("w", "q", "e") else ""
        return f"{key}{suffix}:{self.phase}"

    @property
    def is_idle(self) -> bool:
        return self.active is None and self.requested is None

    def request(self, intent: TokenMotionIntent | None) -> None:
        if intent is None:
            self.requested = None
            return
        key, speed = str(intent[0]).lower(), str(intent[1]).lower()
        if key not in self._VALID_KEYS:
            raise ValueError(f"unsupported token motion key: {key}")
        valid_variants = (
            ("30", "60", "90") if key in self.turn else ("slow", "normal")
        )
        if speed not in valid_variants:
            raise ValueError(f"unsupported token motion speed: {speed}")
        if key not in ("w", "q", "e"):
            speed = "normal"
        self.requested = (key, speed)

    def _segment(self, phase: str, start: int, end: int) -> None:
        if self._clip is None or not 0 <= start < end <= self._clip.frame_count:
            raise ValueError(
                f"invalid {phase} token range {start}:{end} for {self._clip}"
            )
        self.phase = phase
        self._index = start
        self._end = end

    def _start_requested(self) -> None:
        if self.requested is None:
            return
        self.active = self.requested
        key, speed = self.active
        if key == "w":
            self._clip, self._plan = self.forward[speed]
            self._segment(
                "enter", self._plan.enter_start, self._plan.loop_start
            )
        elif key in self.turn:
            self._clip, self._plan = self.turn[key][speed], None
            # Navigation turns are angle-qualified one-shot keyboard clips.
            # Repeating the stable middle is reserved for manual key holding.
            self._segment("cycle", 0, self._clip.frame_count)
        else:
            self._clip, self._plan = self.discrete[key], None
            self._segment("cycle", 0, self._clip.frame_count)

    def _finish_active(self) -> None:
        self.active = None
        self.phase = "idle"
        self._clip = None
        self._plan = None
        self._index = 0
        self._end = 0

    def _advance_boundary(self) -> None:
        if self.active is None:
            self._start_requested()
            return
        same_request = self.requested == self.active
        if self.phase == "enter":
            if same_request:
                assert self._plan is not None
                self._segment("loop", self._plan.loop_start, self._plan.loop_end)
            else:
                assert self._plan is not None and self._clip is not None
                self._segment("exit", self._plan.exit_start, self._clip.frame_count)
        elif self.phase == "loop":
            assert self._plan is not None and self._clip is not None
            if same_request:
                self._segment("loop", self._plan.loop_start, self._plan.loop_end)
            else:
                self._segment("exit", self._plan.exit_start, self._clip.frame_count)
        elif self.phase == "cycle":
            assert self._clip is not None
            if same_request:
                self._segment("cycle", 0, self._clip.frame_count)
            else:
                self._finish_active()
                self._start_requested()
        elif self.phase == "exit":
            self._finish_active()
            self._start_requested()
        else:
            raise RuntimeError(f"unknown token sequencer phase: {self.phase}")

    def step(self) -> np.ndarray:
        while self.active is None or self._index >= self._end:
            previous = (self.active, self.phase, self._index, self._end)
            self._advance_boundary()
            if self.active is None:
                return self.hold
            current = (self.active, self.phase, self._index, self._end)
            if current == previous:
                raise RuntimeError("token sequencer did not advance")
        assert self._clip is not None
        token = self._clip.tokens[self._index]
        self._index += 1
        self.hold = token
        return token
