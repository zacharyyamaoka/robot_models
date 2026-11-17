"""A belt driven through a drum: the conveyor that carries parcels past an arm, as a ladder of models (PEP 0004).

The ladder climbs from an ideal belt (the drum's turns become belt travel, a rigid mass moves under a force) through
friction on the belt, the belt's stretch between drum and load, and friction that grows with the belt's distance, to a
grey-box residual and a black-box network. Its rungs are numbered by the actuator's layers (sim2real's decision, the
same M2 and M3 friction rungs, here as forces on the belt); there is no M1 because the type has no Datasheet class yet.

Spec: PEP 0004 §5.12 and §6.11.
"""

from __future__ import annotations

from robot_models import references
from robot_models.ladder import Alias, DeploymentOnly, Ladder, Parameter, Rung, Wear

# --- vocabulary, grouped by the rung that first reads it -------------------------------------------------------------

VOCABULARY = (
    # M0.ideal: the drum turns the motor's rotation into belt travel; one rigid mass moves
    Parameter(
        name="drum_radius", unit="m",
        definition="Radius of the drive drum the belt wraps.",
    ),
    Parameter(
        name="gear_ratio", unit="",
        definition="Motor turns per drum turn.",
    ),
    Parameter(
        name="moving_mass", unit="kg",
        definition="The moving mass referred to the belt: belt, drum and reflected rotor.",
    ),
    # M2.coulomb_viscous
    Parameter(
        name="coulomb_friction", unit="N",
        definition="Coulomb friction force on the belt: the constant force that opposes its motion.",
    ),
    Parameter(
        name="viscous_friction", unit="N·s/m",
        definition="Viscous friction coefficient on the belt.",
    ),
    # M3.stribeck
    Parameter(
        name="breakaway_force", unit="N",
        definition="Static friction on the belt: the force that starts it moving from rest.",
    ),
    Parameter(
        name="stribeck_velocity", unit="m/s",
        definition="Belt speed over which friction falls from breakaway_force to coulomb_friction.",
    ),
    # M4.stretch
    Parameter(
        name="stretch_stiffness", unit="N/m",
        definition="The belt's stiffness along its length, between drum and load.",
    ),
    Parameter(
        name="stretch_damping", unit="N·s/m",
        definition="Damping of the belt's stretch between drum and load.",
    ),
    # M4.wear
    Parameter(
        name="friction_growth", unit="N/km",
        definition=("How fast coulomb_friction grows per kilometre of belt travel (idlers and slider bed "
                    "wearing)."),
    ),
)

# --- the rungs, M0 upward ---------------------------------------------------------------------------------------------

RUNGS = (
    Rung(
        level=0,
        name="ideal",
        title="An ideal belt: kinematics and a rigid mass",
        base=None,
        adds=("drum_radius", "gear_ratio", "moving_mass"),
        equation="x = θ_motor·drum_radius/gear_ratio; (moving_mass + load)·ẍ = F",
        references=(references.ZHANG_XIA_2011,),
        # WHY no M1 (PEP 0004 §5.12): rungs are numbered by the actuator's layers, and a datasheet rung needs a
        # Datasheet class, which the conveyor does not have yet.
        note="numbered by the actuator's layers (sim2real's decision); no M1: no Datasheet class yet.",
    ),
    Rung(
        level=2,
        name="coulomb_viscous",
        title="Coulomb and viscous friction on the belt",
        base="M0.ideal",
        adds=("coulomb_friction", "viscous_friction"),
        equation="F_f = coulomb_friction·sign(ẋ) + viscous_friction·ẋ",
        references=(references.ARMSTRONG_1994, references.ZHANG_XIA_2011),
    ),
    Rung(
        level=3,
        name="stribeck",
        title="Static friction (Stribeck)",
        base="M2.coulomb_viscous",
        adds=("breakaway_force", "stribeck_velocity"),
        # WHY the equation squares the Stribeck velocity ratio and has no exponent parameter (PEP 0004 §5.12): no
        # conveyor source (sim2real's conveyor card, robotparts' stribeck rung it points at) states an exponent, so it
        # is held at the actuator's default of 2 instead of being an optional parameter as in the actuator's M3.stribeck.
        equation=("F_f = (coulomb_friction + (breakaway_force − coulomb_friction)·exp(−(|ẋ|/stribeck_velocity)²))"
                  "·sign(ẋ) + viscous_friction·ẋ"),
        references=(references.ARMSTRONG_1994,),
        note="the actuator's M3.stribeck as forces on the belt.",
    ),
    Rung(
        level=4,
        name="stretch",
        title="Belt stretch: drum and load as two masses",
        base="M3.stribeck",
        adds=("stretch_stiffness", "stretch_damping"),
        equation=("load: (moving_mass + load)·ẍ_l = stretch_stiffness·(x_d − x_l) + stretch_damping·(ẋ_d − ẋ_l) "
                  "− F_f; drum: m_d·ẍ_d = gear_ratio·τ/drum_radius − the same stretch force"),
        references=(references.SPONG_1987, references.HE_2017),
    ),
    Rung(
        level=4,
        name="wear",
        title="Friction that grows with belt distance",
        base="M4.stretch",
        adds=("friction_growth",),
        wears=(
            Wear(parameter="coulomb_friction", usage="belt distance", usage_unit="km", law="linear",
                 rate="friction_growth"),
        ),
        equation="coulomb_friction(s) = coulomb_friction + friction_growth·s",
        references=(references.LU_MEEKER_1993, references.ZHANG_XIA_2011),
    ),
    Rung(
        level=6,
        name="residual",
        title="Grey-box residual on M4.wear",
        base="M4.wear",
        kind="residual",
        equation=("M4.wear + an MLP residual drive force from 3 samples of belt speed error, belt speed and "
                  "motor current"),
        references=(references.NGUYEN_TUONG_2010, references.ZENG_2020),
    ),
    Rung(
        level=7,
        name="learned",
        title="Black-box network",
        base=None,
        kind="learned",
        equation="an MLP over 3 samples of position error and belt speed → drive force",
        references=(references.HWANGBO_2019, references.HWANGBO_2019_ARXIV),
    ),
)

# --- other names for the same quantities ------------------------------------------------------------------------------

ALIASES = (
    Alias(name="drum_radius_m", canonical="drum_radius"),
    Alias(name="drum_radius_mm", canonical="drum_radius", unit="mm"),
    Alias(name="moving_mass_kg", canonical="moving_mass"),
    Alias(name="coulomb_n", canonical="coulomb_friction"),
    Alias(name="viscous_n_s_m", canonical="viscous_friction"),
    Alias(name="static_n", canonical="breakaway_force"),
    Alias(name="stretch_stiffness_n_m", canonical="stretch_stiffness"),
    Alias(name="stretch_damping_n_s_m", canonical="stretch_damping"),
    Alias(name="coulomb_growth_n_per_km", canonical="friction_growth"),
)

# --- what a deployment sets, not the type ----------------------------------------------------------------------------

DEPLOYMENT = (
    DeploymentOnly(
        name="load_mass_kg", role="assembly",
        why="The parcels on the belt: the deployment's.",
    ),
    DeploymentOnly(
        name="encoder_counts_per_m", role="config",
        why="Derived from the encoder, the ratio and the drum radius.",
    ),
)

LADDER = Ladder(
    type="conveyor",
    title="A belt driven through a drum.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=ALIASES,
    deployment=DEPLOYMENT,
)
