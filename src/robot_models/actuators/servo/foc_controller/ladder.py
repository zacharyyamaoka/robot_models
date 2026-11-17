"""The ladder of a field-oriented-control (FOC) motor drive board: the electronics that turn a torque command into
the q-axis current in a motor's windings.

    from robot_models.actuators.servo.foc_controller.ladder import LADDER
    LADDER.parameters("M2.ceiling_heat")   # what the voltage-ceiling and heating rung needs, M1's names included

Rungs: M0 the commanded current as is, M1 the transport delay and first-order current loop, M2 the voltage ceiling and
the board's heating (and M2.wear, the board's cooling ageing with hours), M6 the grey-box residual, M7 the learned net.
A deployment's own settings (torque cap, host loop rate, modulation-limit setting) are not on a rung: see `deployment`.

Spec: PEP 0004 §5.5 and §6.4. Sources: sim2real's `driver` card (its rungs, parameters and equations).
"""

from robot_models import references
from robot_models.ladder import Alias, DeploymentOnly, Ladder, Packed, Parameter, Rung, Wear

# --- the vocabulary, grouped by the rung that first reads each name ---------------------------------------------------

# M1.delay_loop: the command's path to the current.
COMMAND_LATENCY = Parameter(
    name="command_latency",
    unit="s",
    shape="scalar",
    definition="Time from a command reaching the drive to its first response: the transport delay before the "
               "current loop.",
)
CURRENT_LOOP_BANDWIDTH = Parameter(
    name="current_loop_bandwidth",
    unit="Hz",
    shape="scalar",
    definition="The drive's closed-loop current (torque) bandwidth: the corner of its first-order current loop.",
)
TORQUE_SCALE = Parameter(
    name="torque_scale",
    unit="",
    shape="scalar",
    definition="Torque the drive delivers over the torque it was commanded: its torque-constant calibration error.",
)

# M2.ceiling_heat: what the bus can supply and what the board does with the losses.
VOLTAGE_AVAILABLE = Parameter(
    name="voltage_available",
    unit="V",
    shape="scalar",
    definition="The largest phase voltage the drive can apply: its bus voltage times its modulation limit.",
)
BOARD_THERMAL_RESISTANCE = Parameter(
    name="board_thermal_resistance",
    unit="K/W",
    shape="scalar",
    definition="The board's temperature rise per watt it dissipates.",
)
BOARD_THERMAL_TIME_CONSTANT = Parameter(
    name="board_thermal_time_constant",
    unit="s",
    shape="scalar",
    definition="Time constant of the board temperature's first-order rise.",
)

# M2.wear: how the board's cooling ages.
BOARD_THERMAL_GROWTH = Parameter(
    name="board_thermal_growth",
    unit="K/W/h",
    shape="scalar",
    definition="How fast board_thermal_resistance grows per deployment hour (its cooling ageing).",
)

VOCABULARY = (
    COMMAND_LATENCY,
    CURRENT_LOOP_BANDWIDTH,
    TORQUE_SCALE,
    VOLTAGE_AVAILABLE,
    BOARD_THERMAL_RESISTANCE,
    BOARD_THERMAL_TIME_CONSTANT,
    BOARD_THERMAL_GROWTH,
)

# --- the rungs, M0 upward ---------------------------------------------------------------------------------------------

RUNGS = (
    Rung(
        level=0,
        name="ideal",
        title="An ideal drive: the commanded current",
        base=None,
        equation="i_q = i_cmd (a deployment's torque cap clamps it)",
        references=(references.KRISHNAN_2010,),
        # WHY no adds: the cap that clamps i_cmd is the deployment's (torque_cap_nm, see `deployment`), never the
        # type's.
    ),
    Rung(
        level=1,
        name="delay_loop",
        title="Transport delay, a first-order current loop and its torque scale",
        base="M0.ideal",
        adds=("command_latency", "current_loop_bandwidth", "torque_scale"),
        equation="i_q = torque_scale·LPF(current_loop_bandwidth)[i_cmd(t − command_latency)]",
        references=(references.KRISHNAN_2010,),
    ),
    Rung(
        level=2,
        name="ceiling_heat",
        title="Voltage ceiling and board heating",
        base="M1.delay_loop",
        adds=("voltage_available", "board_thermal_resistance", "board_thermal_time_constant"),
        equation="M1.delay_loop clamped to the voltage ceiling (voltage_available − ω·kt/1.5)/R of the motor it "
                 "drives; board_thermal_time_constant·dΔT/dt = board_thermal_resistance·P_loss − ΔT, ΔT the "
                 "board's rise over ambient",
        references=(references.KRAUSE_2013, references.KRISHNAN_2010),
        # WHY kt and R are not added here: they are the motor's own (bldc_motor), read through the servo rung that
        # composes this one; ω is the rotor speed and the 1.5 is sim2real's driver-card form of the back-EMF term.
        # WHY ΔT, not T: board_thermal_resistance is a rise per watt, so the state is the rise over ambient, as the
        # winding's is on the servo's M5.envelope and the motor's M5.dcmotor (PEP 0004 amendment 1).
    ),
    Rung(
        level=2,
        name="wear",
        title="Board cooling that ages with hours",
        base="M2.ceiling_heat",
        adds=("board_thermal_growth",),
        wears=(
            Wear(
                parameter="board_thermal_resistance",
                usage="deployment hours",
                usage_unit="h",
                law="linear",
                rate="board_thermal_growth",
            ),
        ),
        equation="board_thermal_resistance(h) = board_thermal_resistance + board_thermal_growth·h",
        references=(references.LU_MEEKER_1993, references.MEEKER_1998),
    ),
    Rung(
        level=6,
        name="residual",
        title="Grey-box residual on M2.wear",
        base="M2.wear",
        kind="residual",
        equation="M2.wear + an MLP residual q-current from 3 samples of current command, rotor speed and bus voltage",
        references=(references.NGUYEN_TUONG_2010,),
    ),
    Rung(
        level=7,
        name="learned",
        title="Black-box network",
        base=None,
        kind="learned",
        equation="an MLP over 3 samples of current command, rotor speed, bus voltage and board temperature → "
                 "q-current",
        references=(references.HWANGBO_2019,),
    ),
)

# --- other names, packed names and what a deployment sets -------------------------------------------------------------

ALIASES = (
    Alias(
        name="command_delay_s",
        canonical="command_latency",
        note="The twin's transport delay; the registry's command_latency is measured host to first response over a "
             "named bus.",
    ),
    Alias(
        name="current_loop_hz",
        canonical="current_loop_bandwidth",
        note="The twin's first-order loop corner, which is its −3 dB bandwidth.",
    ),
    Alias(name="voltage_available_v", canonical="voltage_available"),
    Alias(name="board_thermal_growth_per_h", canonical="board_thermal_growth"),
)

PACKED = (
    Packed(
        name="board_thermal",
        holds=("board_thermal_resistance", "board_thermal_time_constant"),
        ordered=True,
        note="sim2real's driver card: (K/W, s).",
    ),
)

DEPLOYMENT = (
    DeploymentOnly(
        name="torque_cap_nm",
        role="config",
        why="The runner's torque cap: the deployment's config.",
    ),
    DeploymentOnly(
        name="telemetry_rate_hz",
        role="config",
        why="The host loop's rate: the deployment's config.",
    ),
    DeploymentOnly(
        name="max_voltage_factor",
        role="config",
        why="The firmware's modulation-limit setting: the deployment's config.",
    ),
)

LADDER = Ladder(
    type="foc_controller",
    title="A field-oriented-control motor drive board.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=ALIASES,
    packed=PACKED,
    deployment=DEPLOYMENT,
)
