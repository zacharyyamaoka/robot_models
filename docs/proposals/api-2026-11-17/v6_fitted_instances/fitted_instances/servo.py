"""integrated_servo as ONE family of model classes: the source today's `actuators/servo/ladder.py` is derived from.

A class body lists what its rung ADDS: `Parameter(unit, definition)` is new and required, `required=False` is read
when stated, and `Parameter()` makes a parameter the base already states required (M1 states coulomb_friction, M2 needs
it). The docstring is the rung's title. The fallback is the MRO; the ladder is the declaration order, by level.
"""

from __future__ import annotations

from robot_models import references as refs
from robot_models.actuators.servo import ladder as today  # aliases, packed and deployment names: not models (README)
from robot_models.actuators.servo.integrated_servo import IntegratedServo as IntegratedServoDatasheet
from robot_models.ladder import Wear as WearLaw

from .model import BlackBox, Model, ModelFamily, Parameter

# WHY module constants: three parameters first appear on two branches (M4.ripple and M5.dcmotor), so they are written
# once and placed on both; the family refuses two different definitions of one name.
HARMONICS = Parameter("N·m", "Rotor-periodic torque harmonics at the output, a list of (rotor_harmonic, amplitude, "
                      "phase): cogging and commutation ripple.", shape="list")
THERMAL_RESISTANCE = Parameter("K/W", "Winding-to-ambient thermal resistance: winding temperature rise per watt of "
                               "copper loss.")
THERMAL_TIME_CONSTANT = Parameter("s", "Time constant of the winding temperature's first-order rise.")


class IntegratedServo(ModelFamily, type="integrated_servo"):
    """A joint in one housing: a motor, its drive, a reducer or none, and an encoder."""

    datasheet = IntegratedServoDatasheet
    aliases, packed, deployment = today.ALIASES, today.PACKED, today.DEPLOYMENT

    class Ideal(Model, level=0, name="ideal"):
        """The ideal actuator: the stated inertia and ratio, no losses"""
        output_inertia = Parameter("kg·m²", "The part's own inertia about its axis, referred to the output (MuJoCo's "
                                   "armature).")
        gear_ratio = Parameter("", "Rotor turns per output turn, > 0: 10 for a 10:1 reducer.")
        equation = "tau_r = 0; armature = output_inertia"
        references = (refs.SICILIANO_2009, refs.MUJOCO_ARMATURE)

    class Datasheet(Ideal, level=1, name="datasheet", datasheet=IntegratedServoDatasheet):
        """The datasheet: the stated values, nothing fitted"""
        equation = ("tau_r = coulomb_friction·sign(v) + viscous_friction·v when stated; armature = output_inertia, or "
                    "gear_ratio²·(rotor_inertia + gearbox_input_inertia)")
        references = (refs.ARMSTRONG_1994, refs.MUJOCO_ARMATURE)
        note = "States every IntegratedServo field and needs none beyond M0 (robotparts' M1: required false)."

    class CoulombViscous(Datasheet, level=2, name="coulomb_viscous"):
        """Coulomb and viscous friction, fitted"""
        coulomb_friction = Parameter()  # stated on M1, needed from here
        viscous_friction = Parameter()
        equation = "tau_r = coulomb_friction·sign(v) + viscous_friction·v"
        references = (refs.ARMSTRONG_1994, refs.OLSSON_1998)

    class Stribeck(CoulombViscous, level=3, name="stribeck"):
        """Stribeck friction: breakaway falling to Coulomb with speed"""
        breakaway_torque = Parameter("N·m", "Static friction at the output: the torque that starts it moving from "
                                     "rest, the top of the Stribeck curve.")
        stribeck_velocity = Parameter("rad/s", "Output speed over which friction falls from breakaway_torque to "
                                      "coulomb_friction.")
        stribeck_exponent = Parameter("", "Shape exponent of the Stribeck curve, how sharply friction falls with "
                                      "speed; 2 when absent.", required=False)
        equation = ("tau_r = (coulomb_friction + (breakaway_torque − coulomb_friction)·exp(−(|v|/stribeck_velocity)"
                    "^stribeck_exponent))·sign(v) + viscous_friction·v")
        references = (refs.ARMSTRONG_1994, refs.OLSSON_1998)

    class Load(CoulombViscous, level=3, name="load"):
        """Friction that grows with the transmitted load"""
        friction_load_coefficient = Parameter("", "Extra output friction per N·m of load torque the part transmits: "
                                              "Fc + μ·abs(τ_load).")
        equation = "tau_r = (coulomb_friction + friction_load_coefficient·|tau_load|)·sign(v) + viscous_friction·v"
        references = (refs.KIRCANSKI_1997, refs.OLSSON_1998)

    class LuGre(Stribeck, level=3, name="lugre"):
        """LuGre dynamic friction"""
        sigma0 = Parameter("N·m/rad", "LuGre bristle stiffness: output torque per radian of bristle deflection.")
        sigma1 = Parameter("N·m·s/rad", "LuGre bristle damping: output torque per rad/s of bristle deflection rate.")
        equation = ("dz/dt = v − sigma0·|v|·z/g(v), g(v) the Stribeck level of M3.stribeck; tau_r = sigma0·z + "
                    "sigma1·dz/dt + viscous_friction·v (LuGre's sigma2)")
        references = (refs.CANUDAS_1995,)

    class WeakeningCapture(Stribeck, level=3, name="weakening_capture"):
        """Stribeck with weakening and a capture level at rest"""
        friction_weakening = Parameter("N·m·s/rad", "How fast the static friction level falls with output speed, up "
                                       "to friction_weakening_limit.")
        friction_weakening_limit = Parameter("rad/s", "The output speed up to which the friction level keeps "
                                             "weakening.")
        friction_capture = Parameter("N·m", "The friction level that holds a slowed output until it has stayed slow "
                                     "for friction_capture_dwell.")
        friction_capture_speed = Parameter("rad/s", "Below this output speed a joint counts as slow for capture.")
        friction_capture_dwell = Parameter("s", "How long a joint must stay slow before the static level returns.")
        equation = ("level(v) = breakaway_torque − friction_weakening·min(|v|, friction_weakening_limit) − "
                    "(breakaway_torque − coulomb_friction)·(1 − exp(−(|v|/stribeck_velocity)²)); friction_capture "
                    "holds below friction_capture_speed until friction_capture_dwell has passed")
        references = (refs.ARMSTRONG_1994, refs.CANUDAS_1995)
        note = "sim2real's servo M3, the rack twin's friction: its structure only, the twin is never published."

    class Ripple(CoulombViscous, level=4, name="ripple"):
        """Rotor ripple on Coulomb and viscous friction"""
        harmonics = HARMONICS
        equation = ("tau_r = Σ_k amplitude_k·sin(rotor_harmonic_k·gear_ratio·θ + phase_k) + coulomb_friction·sign(v) "
                    "+ viscous_friction·v")
        references = (refs.JAHNS_1996,)

    class Drivetrain(WeakeningCapture, level=4, name="drivetrain"):
        """The drivetrain: ripple, backlash and compliance on the full friction model"""
        harmonics = HARMONICS
        backlash = Parameter()
        torsional_stiffness = Parameter("N·m/rad", "Drivetrain stiffness at the output with the rotor held: output "
                                        "torque per radian of twist.")
        composes = (("foc_controller", "M1.delay_loop"),)
        equation = ("output_inertia·q̈ = tau_drive − tau_r; tau_r = level(v)·sign(v) + viscous_friction·v + the "
                    "harmonics; the output across a backlash gap and torsional_stiffness; tau_drive from "
                    "foc_controller M1.delay_loop")
        references = (refs.JAHNS_1996, refs.NORDIN_2002, refs.TUTTLE_1996, refs.KRISHNAN_2010)
        note = "sim2real's servo M4, the rack twin (today's model), less its heat: that is M5.envelope's."

    class DcMotor(LuGre, level=5, name="dcmotor"):
        """MuJoCo's DC motor: electrics, heat, cogging and LuGre friction"""
        kt = Parameter()
        resistance_line_to_line = Parameter()
        harmonics = HARMONICS.optional()
        thermal_resistance = THERMAL_RESISTANCE.optional()
        thermal_time_constant = THERMAL_TIME_CONSTANT.optional()
        equation = ("MuJoCo's own: mjDYN_DCMOTOR (electrical dynamics), mjGAIN_DCMOTOR (K or K/R), mjBIAS_DCMOTOR "
                    "(back-EMF, cogging, LuGre friction)")
        references = (refs.KRAUSE_2013, refs.MUJOCO_DCMOTOR)
        note = ("robotparts' rung: its terms are MuJoCo's arrays (the packed names); motorconst and resistance "
                "required, the rest optional, as its model.json marks them.")

    class Envelope(Drivetrain, level=5, name="envelope"):
        """The drive's ceiling, the motor's electrics and heat on the drivetrain"""
        kt = Parameter()
        resistance_line_to_line = Parameter()
        thermal_resistance = THERMAL_RESISTANCE
        thermal_time_constant = THERMAL_TIME_CONSTANT
        composes = (("foc_controller", "M2.ceiling_heat"),)
        equation = ("kt_out = kt·gear_ratio, R = resistance_line_to_line/2 per phase: motoring torque ≤ "
                    "kt_out·(voltage_available − |v|·kt_out/1.5)/R; any torque ≤ kt_out·I_max (the deployment's "
                    "current cap); winding rise ΔT over ambient: thermal_time_constant·dΔT/dt = "
                    "thermal_resistance·1.5·i²R − ΔT (the three phases' copper loss, i the phase-current amplitude)")
        references = (refs.KRAUSE_2013, refs.KRISHNAN_2010, refs.MUJOCO_DCMOTOR)
        note = ("sim2real's servo M5 (the envelope ladder): the rack twin plus the drive's ceiling and the motor's "
                "electrics; the heat the twin carries at its M4 sits here, robotparts' layer 5.")

    class Wear(Envelope, level=5, name="wear"):
        """Wear: friction grows with hours, backlash opens with load"""
        friction_growth = Parameter("N·m/h", "How fast the friction level grows per deployment hour (seals, grease, "
                                    "bearings); on a Stribeck curve breakaway and Coulomb grow together.")
        backlash_wear = Parameter("rad/(N·m·rev)", "How fast backlash opens per torque-weighted output revolution "
                                  "(Archard).")
        wears = (WearLaw("breakaway_torque", today.DEPLOYMENT_HOURS, today.HOURS_UNIT, "linear", "friction_growth"),
                 WearLaw("coulomb_friction", today.DEPLOYMENT_HOURS, today.HOURS_UNIT, "linear", "friction_growth"),
                 WearLaw("backlash", today.TORQUE_REVOLUTIONS, today.TORQUE_REVOLUTIONS_UNIT, "archard", "backlash_wear"))
        equation = ("backlash(u) = backlash + backlash_wear·Σ|τ|·Δrev; breakaway_torque and coulomb_friction both "
                    "grow by friction_growth·h (the Stribeck drop stays)")
        references = (refs.ARCHARD_1953, refs.FLODIN_1997, refs.LU_MEEKER_1993)

    class Residual(Wear, level=6, name="residual"):
        """Grey-box residual on M5.wear"""
        kind = "residual"
        equation = "M5.wear + an MLP residual torque from 3 samples of position error, velocity and torque command"
        references = (refs.NGUYEN_TUONG_2010, refs.ZENG_2020, refs.KOVALEV_2026)

    class Learned(BlackBox, level=7, name="learned"):
        """Black-box network"""
        # WHY a polynomial stand-in for today's MLP: it fits in closed form with numpy and still has no named
        # parameters, which is what has to look the same from outside (fit, predict, param_count, select_best).
        kind = "learned"
        degree = 3
        equation = "an MLP over 3 samples of position error and velocity → output torque"
        references = (refs.HWANGBO_2019, refs.HWANGBO_2019_ARXIV, refs.ZHU_2025)
