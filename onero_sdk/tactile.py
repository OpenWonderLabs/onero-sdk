"""Optional tactile-sensor support for the Onero gripper."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .grippers import OneroGripper


@dataclass(frozen=True)
class TactileOnlineStatus:
    """Online state reported by the two optional tactile ports."""

    sensor_1_online: bool
    sensor_2_online: bool
    raw_mask: int
    timestamp: float


@dataclass(frozen=True)
class TactileCalibrationResult:
    """Raw recalibration result for each optional tactile sensor."""

    sensor_1: int
    sensor_2: int
    timestamp: float

    @property
    def sensor_1_success(self) -> bool:
        return self.sensor_1 == 0x01

    @property
    def sensor_2_success(self) -> bool:
        return self.sensor_2 == 0x01


@dataclass(frozen=True)
class TactilePoint:
    """One tactile point, retaining raw values until scaling is verified."""

    point_id: int
    fx_raw: int
    fy_raw: int
    fz_raw: int
    fx: float | None = None
    fy: float | None = None
    fz: float | None = None


@dataclass(frozen=True)
class TactileSample:
    """A synchronous sample returned by one tactile sensor."""

    sensor_id: int
    points: tuple[TactilePoint, ...]
    timestamp: float


class GripperTactile:
    """Optional tactile component owned by :class:`OneroGripper`.

    The component shares its parent gripper's CAN transport and I/O lock. It
    does not own or close communication resources and does not require the
    gripper motor to be enabled.
    """

    _ONLINE = 0xD8
    _RECALIBRATE = 0xDB

    def __init__(self, gripper: OneroGripper) -> None:
        self._gripper = gripper
        self._last_online_status: TactileOnlineStatus | None = None
        self._last_sample: TactileSample | None = None

    @property
    def online_status(self) -> TactileOnlineStatus | None:
        return self._last_online_status

    @property
    def state(self) -> TactileSample | None:
        return self._last_sample

    def _management_request(self, command: int, response_length: int) -> bytes:
        gripper = self._gripper
        bus = gripper._require_bus()
        esc_id = gripper.command_id
        payload = bytes([esc_id & 0xFF, (esc_id >> 8) & 0xFF, command])
        with gripper._io_lock:
            bus.send(0x7FF, payload)
            frame = bus.recv_until(
                gripper.feedback_id, timeout=gripper._feedback_timeout
            )
        if frame is None:
            raise RuntimeError(
                f"Onero tactile command 0x{command:02X} timed out"
            )
        data = frame.data
        if len(data) < response_length:
            raise ValueError(
                f"Onero tactile response 0x{command:02X} must contain at "
                f"least {response_length} bytes"
            )
        if data[:3] != payload:
            raise ValueError(
                f"Onero tactile response does not match command 0x{command:02X}"
            )
        return data

    def get_online_status(self) -> TactileOnlineStatus:
        data = self._management_request(self._ONLINE, 4)
        mask = int(data[3])
        status = TactileOnlineStatus(
            sensor_1_online=bool(mask & 0x01),
            sensor_2_online=bool(mask & 0x02),
            raw_mask=mask,
            timestamp=time.monotonic(),
        )
        self._last_online_status = status
        return status

    def recalibrate(self) -> TactileCalibrationResult:
        data = self._management_request(self._RECALIBRATE, 5)
        return TactileCalibrationResult(
            sensor_1=int(data[3]),
            sensor_2=int(data[4]),
            timestamp=time.monotonic(),
        )

    @staticmethod
    def _validate_read(sensor_id: int, start: int, count: int) -> None:
        if isinstance(sensor_id, bool) or not isinstance(sensor_id, int):
            raise ValueError("sensor_id must be 1 or 2")
        if sensor_id not in (1, 2):
            raise ValueError("sensor_id must be 1 or 2")
        for name, value in (("start", start), ("count", count)):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{name} must be an integer")
        if not 0 <= start <= 9:
            raise ValueError("start must be in [0, 9]")
        if count < 1 or start + count > 10:
            raise ValueError("count must select points within [0, 9]")

    @staticmethod
    def _point(point_id: int, raw: bytes) -> TactilePoint:
        if len(raw) != 3:
            raise ValueError("a tactile point must contain Fx, Fy and Fz")
        fx, fy, fz = (int(value) for value in raw)
        return TactilePoint(
            point_id=point_id,
            fx_raw=fx,
            fy_raw=fy,
            fz_raw=fz,
            # Only the Fz scale is defined by the available protocol.
            fz=fz * 0.1,
        )

    def read_resultant(self, sensor_id: int) -> TactilePoint:
        return self.read_points(sensor_id=sensor_id, start=0, count=1).points[0]

    def read_points(self, sensor_id: int, start: int = 0,
                    count: int = 10) -> TactileSample:
        self._validate_read(sensor_id, start, count)
        gripper = self._gripper
        bus = gripper._require_bus()
        upload_id = gripper.feedback_id + 0x400
        expected_frames = math.ceil(count / 2)
        payload = bytes([sensor_id, start, count])
        frames = []
        deadline = time.monotonic() + gripper._feedback_timeout

        with gripper._io_lock:
            bus.send(gripper.command_id, payload)
            while len(frames) < expected_frames:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                frame = bus.recv(timeout=remaining)
                if frame is None:
                    break
                if frame.arbitration_id == upload_id:
                    frames.append(frame)

        if len(frames) != expected_frames:
            raise RuntimeError(
                f"Onero tactile sample timed out after {len(frames)} of "
                f"{expected_frames} frames"
            )

        points: list[TactilePoint] = []
        next_point = start
        for frame in frames:
            data = frame.data
            if len(data) not in (5, 8):
                raise ValueError("Onero tactile upload must contain 5 or 8 bytes")
            if data[0] != sensor_id:
                raise ValueError("Onero tactile upload sensor ID does not match")
            if data[1] != next_point:
                raise ValueError("Onero tactile upload point sequence is invalid")
            points.append(self._point(next_point, data[2:5]))
            next_point += 1
            if len(data) == 8 and len(points) < count:
                points.append(self._point(next_point, data[5:8]))
                next_point += 1

        if len(points) != count:
            raise ValueError(
                f"Onero tactile upload returned {len(points)} of {count} points"
            )

        sample = TactileSample(
            sensor_id=sensor_id,
            points=tuple(points),
            timestamp=time.monotonic(),
        )
        self._last_sample = sample
        return sample
