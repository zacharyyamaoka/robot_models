"""BldcMotor: the typed datasheet fields of a bare brushless motor (a PMSM / BLDC with its own encoder, no reducer).

The fields are rung 1 of the type's model ladder, "the datasheet: the stated values, nothing fitted" (robotparts
registry `model_rungs.actuator`); the fitted rungs above it move here from sim2real's type cards (the layering
decision's step 7), not in this file.

Names are robotparts registry keys; the ones marked `catalogue name` are not registry keys yet but are the names the
catalogue already uses (PEP 0003 §3). Torques and speeds are at the part's output, which on a bare motor is its shaft.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from ....datasheet import ELECTRICAL, MECHANICAL, PERFORMANCE, Datasheet, Field, Stated
from ....datasheet.units import (
    BACK_EMF_CONSTANT,
    COUNT,
    CURRENT,
    DAMPING,
    INDUCTANCE,
    INERTIA,
    LENGTH,
    MASS,
    POWER,
    RESISTANCE,
    SPEED,
    TORQUE,
    TORQUE_CONSTANT,
    VELOCITY_CONSTANT,
    VOLTAGE,
)


@dataclass(frozen=True)
class BldcMotor(Datasheet):
    """A bare brushless motor's datasheet. One `Stated` per field, or None when no source states it."""

    # robotparts registry key_specs["actuator"]
    KEY_SPECS: ClassVar[tuple[str, ...]] = ("peak_torque", "rated_torque", "max_output_speed", "mass")

    # --- performance ---
    peak_torque: Stated | None = Field(TORQUE, "Peak (stall, max) torque at the output.", group=PERFORMANCE, positive=True)
    rated_torque: Stated | None = Field(TORQUE, "Continuous (rated) torque at the output.", group=PERFORMANCE, positive=True)
    max_output_speed: Stated | None = Field(SPEED, "No-load (maximum) speed at the output.", group=PERFORMANCE, positive=True)
    rated_speed: Stated | None = Field(SPEED, "Speed at rated torque, at the output.", group=PERFORMANCE, positive=True)
    rated_power: Stated | None = Field(POWER, "Continuous MECHANICAL output power, rated torque × rated speed; an "
                                       "electrical input power is not this.", group=PERFORMANCE, positive=True)

    # --- electrical ---
    nominal_voltage: Stated | None = Field(VOLTAGE, "Nominal supply voltage.", group=ELECTRICAL, positive=True)
    voltage_min: Stated | None = Field(VOLTAGE, "Lowest supply voltage the part is rated for.", group=ELECTRICAL)
    voltage_max: Stated | None = Field(VOLTAGE, "Highest supply voltage the part is rated for.", group=ELECTRICAL)
    rated_phase_current: Stated | None = Field(CURRENT, "Continuous phase current at rated torque (catalogue name).",
                                               group=ELECTRICAL, positive=True)
    max_phase_current: Stated | None = Field(CURRENT, "Peak phase current, at peak torque (catalogue name).",
                                             group=ELECTRICAL, positive=True)
    kv: Stated | None = Field(VELOCITY_CONSTANT, "Velocity constant: no-load ROTOR speed per volt (line-to-line peak "
                              "back-EMF). A geared part's source may state it at the output: say at='output'.",
                              group=ELECTRICAL, positive=True, side="rotor", reflect=1)
    kt: Stated | None = Field(TORQUE_CONSTANT, "Torque constant: ROTOR torque per ampere of phase-current amplitude. "
                              "A geared part's source may state it at the output: say at='output'.",
                              group=ELECTRICAL, positive=True, side="rotor", reflect=-1)
    back_emf_constant: Stated | None = Field(BACK_EMF_CONSTANT, "Line-to-line back-EMF per speed (catalogue name).",
                                             group=ELECTRICAL, positive=True)
    resistance_line_to_line: Stated | None = Field(RESISTANCE, "Winding resistance between two leads (line to line); "
                                                   "a page's 'phase to phase' is this.", group=ELECTRICAL, positive=True)
    inductance_line_to_line: Stated | None = Field(INDUCTANCE, "Winding inductance between two leads (line to line).",
                                                   group=ELECTRICAL, positive=True)
    pole_pairs: Stated | None = Field(COUNT, "Magnetic pole pairs.", group=ELECTRICAL, kind="integer", positive=True)
    encoder_lines: Stated | None = Field(COUNT, "Encoder lines (pulses) per rotor turn (catalogue name).",
                                         group=ELECTRICAL, kind="integer", positive=True)
    encoder_bits: Stated | None = Field(COUNT, "Encoder resolution in bits per rotor turn (catalogue name).",
                                        group=ELECTRICAL, kind="integer", positive=True)

    # --- mechanical ---
    mass: Stated | None = Field(MASS, "Mass as sold, without optional accessories.", group=MECHANICAL, positive=True)
    rotor_inertia: Stated | None = Field(INERTIA, "The rotor's moment of inertia on the INPUT (rotor) side. Read as "
                                         "output-side it is off by gear_ratio²; a source stating the reflected value "
                                         "says at='output'.", group=MECHANICAL, positive=True, side="rotor", reflect=-2)
    coulomb_friction: Stated | None = Field(TORQUE, "Coulomb friction torque at the output.", group=MECHANICAL)
    viscous_friction: Stated | None = Field(DAMPING, "Viscous friction coefficient at the output.", group=MECHANICAL)
    torque_ripple: Stated | None = Field(TORQUE, "Amplitude of a rotor-periodic torque harmonic at the output (cogging); "
                                         "the note says which harmonic.", group=MECHANICAL)
    outer_diameter: Stated | None = Field(LENGTH, "Largest diameter of a round housing.", group=MECHANICAL, positive=True)
    axial_length: Stated | None = Field(LENGTH, "Overall length of a round housing along its axis.", group=MECHANICAL,
                                        positive=True)
    flange_size: Stated | None = Field(LENGTH, "Square flange edge (catalogue name).", group=MECHANICAL, positive=True)
    body_length: Stated | None = Field(LENGTH, "Body length behind the flange, without the shaft (catalogue name).",
                                       group=MECHANICAL, positive=True)
    shaft_diameter: Stated | None = Field(LENGTH, "Output shaft diameter (catalogue name).", group=MECHANICAL,
                                          positive=True)
