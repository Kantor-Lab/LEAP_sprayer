from typing import cast, overload

import numpy as np

# math stuff
SQUARE_METERS_PER_ACRE = (
    # an acre is 1 chain by 1 furlong
    (
        660  # 1 furlong in feet
        * 66  # 1 chain in feet
    )
    * (
        12  # 12 inches per foot
        * 2.54  # centimers per inch
        / 100  # meters per centimer
    )
    ** 2
)


class NozzleInfo:
    """
    Hold information for a specific nozzle.

    Attributes:
        nozzle_angle (float): Angle of the nozzle in degrees.
        duty_cycle_min (float): Minimum duty cycle (effective rate) of the nozzle. In [0.0, 1.0]
        ideal_duty_cycle (float): Ideal duty cycle (most effective rate) of the nozzle.
                          In [duty_cycle_min, 1.0].
        max_flow_rate (float): Maximum flow rate of the nozzle in gallons per minute.
                               Should correspond to max (1.0) duty cycle.
        nozzle_box_depth (float): Depth of the box which the sprayer can hit.
                                  Not computed because this tends to be fuzzy on spec sheets
    """

    def __init__(
        self,
        nozzle_angle: float,
        min_duty_cycle: float,
        ideal_duty_cycle: float,
        max_flow_rate: float,
        nozzle_box_depth: float,
    ):
        self.nozzle_angle = nozzle_angle
        assert isinstance(nozzle_angle, float), (
            f'Nozzle angle {nozzle_angle} is not a float (this is annoyingly important)'
        )
        assert 0.0 < self.nozzle_angle <= 180.0, (
            f'Nozzle angle {self.nozzle_angle}° is probably incorrect'
        )

        self.min_duty_cycle = min_duty_cycle
        assert 0 <= self.min_duty_cycle <= 1, (
            f'Duty cycle min {self.min_duty_cycle} is out of range [0.0, 1.0]'
        )

        self.ideal_duty_cycle = ideal_duty_cycle
        assert self.min_duty_cycle <= self.ideal_duty_cycle <= 1, (
            f'Ideal duty cycle {self.ideal_duty_cycle} is out of range [{min_duty_cycle = }, 1.0]'
        )

        self.max_flow_rate = max_flow_rate
        assert self.max_flow_rate >= 0, f'Max flow rate {self.max_flow_rate} gpm less than 0'

        self.nozzle_box_depth = nozzle_box_depth
        assert self.nozzle_box_depth > 0, (
            f'Nozzle box depth {self.nozzle_box_depth} is less than 0'
        )


SPOT_SPRAYER_INFO = NozzleInfo(
    nozzle_angle=40.0,
    min_duty_cycle=0.2,
    ideal_duty_cycle=0.7,
    max_flow_rate=0.25,
    nozzle_box_depth=0.01,  # assume pretty close to a line
)

# gallons per acre target deposition rate
TARGET_GPA_RANGE = (10.0, 15.0)
assert 0 < TARGET_GPA_RANGE[0] <= TARGET_GPA_RANGE[1]

# length of time which constitutes "one" signal
# PWM works by sending a high voltage for only a part of this time,
# cutting the actual output of the sprayer
PWM_PERIOD_MS = 50

NDArray64 = np.ndarray[float, np.dtype[np.float64]]
ScalarOrArray = float | NDArray64


# declare overloads for the types so type checkers understand how args map to outputs
@overload
def compute_sprayer_footprint(
    nozzle_angle: float,
    nozzle_height: float,
) -> float: ...


@overload
def compute_sprayer_footprint(
    nozzle_angle: NDArray64,
    nozzle_height: ScalarOrArray,
) -> NDArray64: ...


@overload
def compute_sprayer_footprint(
    nozzle_angle: ScalarOrArray,
    nozzle_height: NDArray64,
) -> NDArray64: ...


def compute_sprayer_footprint(
    nozzle_angle: ScalarOrArray,
    nozzle_height: ScalarOrArray,
) -> ScalarOrArray:
    """
    Compute the width of the spray footprint based on the nozzle angle and height.

    Args:
        nozzle_angle: The angle of the nozzle in degrees, or the angles of each nozzle.
        nozzle_height: The height of the sprayer in meters, or the height of each nozzle.

    Returns:
        The width of the spray footprint in meters, potentially for each nozzle.
    """

    if isinstance(nozzle_angle, np.ndarray) and isinstance(nozzle_height, np.ndarray):
        assert nozzle_angle.shape == nozzle_height.shape, (
            'Nozzle angles and heights must have the same shape to broadcast'
        )
    else:  # mostly to satisfy the type checker, something about numpy's C hooks breaks otherwise
        assert (
            (isinstance(nozzle_angle, float) and isinstance(nozzle_height, np.ndarray))
            or (isinstance(nozzle_angle, np.ndarray) and isinstance(nozzle_height, float))
            or (isinstance(nozzle_angle, float) and isinstance(nozzle_height, float))
        ), (
            f'Nozzle angle and height must be floats or arrays of floats, '
            f'got {type(nozzle_angle) = } and {type(nozzle_height) = }'
        )

    # divide by two to get half angle and get half base width,
    # then multiply by two to get the full width
    return nozzle_height * np.tan(np.deg2rad(nozzle_angle / 2)) * 2


def _compute_exact_duty(
    target_gpa: float,
    max_gpm: float,
    footprint_m: ScalarOrArray,
    speed_mps: ScalarOrArray,
) -> ScalarOrArray:
    """
    Compute the exact duty cycle required to achieve the target GPA.

    May be greater than 1, indicating this isn't achievable.
    """
    # this math thoroughly confused us multiple times, so here is the derivation in full
    # given target_gpa = (duty * max_gpm) / (footprint * speed) * c,
    # where is c is some constant to handle the unit conversion
    # we can solve for duty
    # duty = (target_gpa * footprint * speed) / (max_gpm * c)
    # and we can solve for what c has to cancel from that (duty must be unitless)
    # 1 = ((gallon/acre) * m * (m/s)) / ((gallon/min) * c)
    #   = (m²/acre) * (min/s) / c
    #   = SQUARE_METERS_PER_ACRE * 1/60 / c
    # ⇒ c = SQUARE_METERS_PER_ACRE / 60

    return (target_gpa * footprint_m * speed_mps) / (max_gpm * SQUARE_METERS_PER_ACRE / 60)


@overload
def get_best_duty(
    nozzle_info: NozzleInfo,
    height_m: float,
    speed_mps: float,
) -> float: ...


@overload
def get_best_duty(
    nozzle_info: NozzleInfo,
    height_m: NDArray64,
    speed_mps: NDArray64,
) -> NDArray64: ...


@overload
def get_best_duty(
    nozzle_info: NozzleInfo,
    height_m: float,
    speed_mps: NDArray64,
) -> NDArray64: ...


@overload
def get_best_duty(
    nozzle_info: NozzleInfo,
    height_m: NDArray64,
    speed_mps: float,
) -> NDArray64: ...


def get_best_duty(
    nozzle_info: NozzleInfo,
    height_m: ScalarOrArray,
    speed_mps: ScalarOrArray,
) -> ScalarOrArray:

    is_scalar = isinstance(height_m, float) and isinstance(speed_mps, float)
    if not is_scalar:
        assert isinstance(height_m, float) or np.issubdtype(
            cast(np.ndarray, height_m).dtype, np.floating
        ), f'height_m must be a float or numpy float array, was {type(height_m)}'
        assert isinstance(speed_mps, float) or np.issubdtype(
            cast(np.ndarray, speed_mps).dtype, np.floating
        ), f'speed_mps must be a float or numpy float array, was {type(speed_mps)}'

    footprint_m = compute_sprayer_footprint(nozzle_info.nozzle_angle, height_m)

    min_duty = _compute_exact_duty(
        TARGET_GPA_RANGE[0],
        nozzle_info.max_flow_rate,
        footprint_m,
        speed_mps,
    )

    max_duty = _compute_exact_duty(
        TARGET_GPA_RANGE[1],
        nozzle_info.max_flow_rate,
        footprint_m,
        speed_mps,
    )

    ideal_is_viable = (min_duty <= nozzle_info.ideal_duty_cycle) & (
        nozzle_info.ideal_duty_cycle <= max_duty
    )

    if isinstance(ideal_is_viable, np.ndarray) and isinstance(speed_mps, np.ndarray):
        assert ideal_is_viable.shape == speed_mps.shape, (
            f'ideal_is_viable shape {ideal_is_viable.shape} does not match'
            f'speed_mps shape {speed_mps.shape}'
        )

    if np.all(ideal_is_viable):
        return (
            np.full_like(speed_mps, nozzle_info.ideal_duty_cycle)
            if not is_scalar
            else nozzle_info.ideal_duty_cycle
        )

    # normalize to 1D. otherwise not_viable = np.True_, which adds an axis and messes things up
    min_duty = np.atleast_1d(min_duty)
    max_duty = np.atleast_1d(max_duty)
    ideal_is_viable = np.atleast_1d(ideal_is_viable)

    bounds = np.hstack((np.reshape(min_duty, (-1, 1)), np.reshape(max_duty, (-1, 1))))
    best_bound = np.argmin(
        np.abs(bounds - nozzle_info.ideal_duty_cycle),
        axis=-1,  # equivalent to 1 for actual arrays, but -1 handles non-array floats
    )

    duty_result = np.empty_like(min_duty, dtype=np.float64)
    duty_result[ideal_is_viable] = nozzle_info.ideal_duty_cycle

    not_viable = np.logical_not(ideal_is_viable)  # need logical not to avoid deprecation warnings
    duty_result[not_viable] = bounds[not_viable][
        np.arange(len(bounds[not_viable])), best_bound[not_viable]
    ]

    duty_result = np.clip(duty_result, nozzle_info.min_duty_cycle, 1.0)

    return duty_result if not is_scalar else float(duty_result.item())
