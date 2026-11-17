"""IntegratedServoDatasheet: the typed datasheet fields of a motor, a reducer and a driver sold as one SKU.

Every BldcMotor field, plus the reducer's. Torques and speeds are at the OUTPUT, after the reducer; Kv, Kt and rotor
inertia stay on the rotor (registry definitions), so a source that states them after the reducer says at='output'
and the datasheet carries them through gear_ratio, marked derived.

The servo's models are the ModelFamily `robot_models.IntegratedServo` (PEP 0004 amendment 2); its M1 rung reads this
class's fields. The type that joins the two is robotparts' follow-up.
"""

from __future__ import annotations

from dataclasses import dataclass

from ...datasheet import MECHANICAL, PERFORMANCE, Field, Stated
from ...datasheet.units import FORCE, FRACTION, INERTIA, RATIO, SMALL_ANGLE
from .bldc.bldc_motor import BldcMotor


@dataclass(frozen=True)
class IntegratedServoDatasheet(BldcMotor):
    """An integrated servo's datasheet: the motor's fields and the reducer's."""

    gear_ratio: Stated | None = Field(RATIO, "Rotor turns per output turn, > 0: 10 for a 10:1 reducer.",
                                      group=PERFORMANCE, positive=True)
    backlash: Stated | None = Field(SMALL_ANGLE, "Backlash at the output.", group=PERFORMANCE)
    efficiency: Stated | None = Field(FRACTION, "Reducer efficiency, power out over power in, at the source's "
                                      "operating point.", group=PERFORMANCE, positive=True)
    gearbox_input_inertia: Stated | None = Field(INERTIA, "The reducer's inertia referred to its input shaft.",
                                                 group=MECHANICAL, positive=True)
    output_inertia: Stated | None = Field(INERTIA, "The part's own inertia about its axis, referred to the output "
                                          "(MuJoCo's armature).", group=MECHANICAL, positive=True)
    max_axial_load: Stated | None = Field(FORCE, "Largest axial load on the output (catalogue name).",
                                          group=MECHANICAL, positive=True)
    max_radial_load: Stated | None = Field(FORCE, "Largest radial load on the output (catalogue name).",
                                           group=MECHANICAL, positive=True)


# WHY a back-compat alias (PEP 0004 amendment 2): robotparts' part file, its six sources/*.py and test_datasheets.py,
# this repo's frozen rulers (test_ladders_acceptance.py pins `ladder.datasheet is IntegratedServo`) and the round-2
# fixture all import `IntegratedServo` from here as the datasheet. It stays the datasheet at this path until they move
# to `IntegratedServoDatasheet`; the bare `robot_models.IntegratedServo` is the servo's ModelFamily.
IntegratedServo = IntegratedServoDatasheet
