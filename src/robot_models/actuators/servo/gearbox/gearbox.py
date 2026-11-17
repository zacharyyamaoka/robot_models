"""Gearbox: the typed datasheet fields of a reducer sold on its own (StepperOnline AD60-G10-D14 on the rack).

Names are robotparts registry keys (key_specs["gearbox"] first); `max_axial_load` / `max_radial_load` are the
catalogue's names. No part is migrated onto this type yet (PEP 0003 §9); it is here so the servo tree is whole.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from ....datasheet import MECHANICAL, PERFORMANCE, Datasheet, Field, Stated
from ....datasheet.units import FORCE, FRACTION, INERTIA, LENGTH, MASS, RATIO, SMALL_ANGLE, TORQUE


@dataclass(frozen=True)
class Gearbox(Datasheet):
    """A reducer's datasheet. Torques at its output."""

    KEY_SPECS: ClassVar[tuple[str, ...]] = ("gear_ratio", "rated_torque", "peak_torque")

    gear_ratio: Stated | None = Field(RATIO, "Input turns per output turn, > 0.", group=PERFORMANCE, positive=True)
    rated_torque: Stated | None = Field(TORQUE, "Continuous torque at the output.", group=PERFORMANCE, positive=True)
    peak_torque: Stated | None = Field(TORQUE, "Peak torque at the output.", group=PERFORMANCE, positive=True)
    backlash: Stated | None = Field(SMALL_ANGLE, "Backlash at the output.", group=PERFORMANCE)
    efficiency: Stated | None = Field(FRACTION, "Power out over power in, at the source's operating point.",
                                      group=PERFORMANCE, positive=True)
    gearbox_input_inertia: Stated | None = Field(INERTIA, "Inertia referred to the input shaft.", group=MECHANICAL,
                                                 positive=True)
    mass: Stated | None = Field(MASS, "Mass as sold.", group=MECHANICAL, positive=True)
    outer_diameter: Stated | None = Field(LENGTH, "Largest diameter of a round housing.", group=MECHANICAL, positive=True)
    axial_length: Stated | None = Field(LENGTH, "Overall length along the axis.", group=MECHANICAL, positive=True)
    max_axial_load: Stated | None = Field(FORCE, "Largest axial load on the output (catalogue name).", group=MECHANICAL,
                                          positive=True)
    max_radial_load: Stated | None = Field(FORCE, "Largest radial load on the output (catalogue name).",
                                           group=MECHANICAL, positive=True)
