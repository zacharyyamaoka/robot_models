"""The ladder core (PEP 0004): a rung needs everything its base needs, falls back down its bases, and names only the
type's own parameters. Held red: each planted violation (a loop of bases, an unknown base, a name outside the
vocabulary, a parameter added twice down one chain, an optional name held again, a unit in a name, a packed array read
as one parameter) must refuse, so a core that accepts everything cannot pass.

The toy type here is synthetic; the real ladders are judged by test_ladders_acceptance.py.
"""

from __future__ import annotations

import sys
import textwrap
from dataclasses import replace

import pytest

from robot_models.actuators.servo import BldcMotor, Gearbox, IntegratedServo
from robot_models.ladder import (
    Alias,
    DeploymentOnly,
    Ladder,
    Packed,
    Parameter,
    Reference,
    Rung,
    Wear,
    datasheet_vocabulary,
    discover,
    needs,
    si_unit,
)

VOCABULARY = (
    Parameter("inertia", "kg·m²", "The part's inertia at its output."),
    Parameter("coulomb_friction", "N·m", "Constant friction torque."),
    Parameter("viscous_friction", "N·m·s/rad", "Viscous friction coefficient."),
    Parameter("breakaway_torque", "N·m", "Static friction: the torque that starts it moving."),
    Parameter("stribeck_velocity", "rad/s", "Speed over which friction falls to Coulomb."),
    Parameter("sigma0", "N·m/rad", "Bristle stiffness."),
    Parameter("friction_load_coefficient", "", "Extra friction per N·m of load."),
    Parameter("friction_growth", "N·m/h", "How fast coulomb_friction grows per hour."),
    Parameter("stribeck_exponent", "", "Shape exponent of the Stribeck curve; 2 when absent."),
    Parameter("thermal_resistance", "K/W", "Winding temperature rise per watt."),
    Parameter("thermal_time_constant", "s", "Time constant of the winding temperature."),
)
RUNGS = (
    Rung(0, "ideal", "Ideal", None, ("inertia",)),
    Rung(1, "datasheet", "Stated values", "M0.ideal", optional=("coulomb_friction", "thermal_resistance")),
    Rung(2, "coulomb_viscous", "Coulomb + viscous", "M1.datasheet", ("coulomb_friction", "viscous_friction")),
    Rung(3, "stribeck", "Stribeck", "M2.coulomb_viscous", ("breakaway_torque", "stribeck_velocity"),
         optional=("stribeck_exponent",)),
    Rung(3, "lugre", "LuGre", "M3.stribeck", ("sigma0",)),
    Rung(3, "load", "Load-dependent", "M2.coulomb_viscous", ("friction_load_coefficient",)),
    Rung(3, "wear", "Friction that grows", "M3.stribeck", ("friction_growth",),
         wears=(Wear("coulomb_friction", "deployment hours", "h", "linear", "friction_growth"),)),
    Rung(5, "heat", "Winding heat", "M3.lugre", ("thermal_resistance", "thermal_time_constant")),
    Rung(6, "residual", "Residual on M3.lugre", "M3.lugre", kind="residual"),
    Rung(7, "learned", "Black-box net", None, kind="learned"),
)


def toy(**changes) -> Ladder:
    fields = {"type": "widget", "title": "A toy actuator.", "vocabulary": VOCABULARY, "rungs": RUNGS,
                  "aliases": (Alias("frictionloss_nm", "coulomb_friction", "static level, not Coulomb"),
                           Alias("sigma2", "viscous_friction")),
                  "packed": (Packed("thermal_r_c", ("thermal_resistance", "thermal_time_constant"), "(K/W, s)",
                                    ordered=True),),
                  "deployment": (DeploymentOnly("kp_nm_rad", "config", "the drive's gain"),)}
    fields.update(changes)
    return Ladder(**fields)


# --- what a ladder answers ---------------------------------------------------------------------------------------


def test_parameters_are_cumulative_through_the_bases():
    ladder = toy()
    assert ladder.parameters("M0.ideal") == ("inertia",)
    assert ladder.parameters("M3.lugre") == ("inertia", "coulomb_friction", "viscous_friction", "breakaway_torque",
                                             "stribeck_velocity", "sigma0")
    assert ladder.parameters("M3.load") == ("inertia", "coulomb_friction", "viscous_friction",
                                            "friction_load_coefficient")


def test_parameters_are_what_a_rung_needs_and_holds_adds_what_it_reads_when_stated():
    ladder = toy()
    # M1 states coulomb_friction and needs nothing beyond M0: a part with no stated friction still reaches it
    assert ladder.parameters("M1.datasheet") == ("inertia",)
    assert ladder.optional("M1.datasheet") == ("coulomb_friction", "thermal_resistance")
    assert ladder.holds("M1.datasheet") == ("inertia", "coulomb_friction", "thermal_resistance")
    # M2 needs it: an optional name below becomes required above, and is then no longer optional
    assert "coulomb_friction" in ladder.parameters("M2.coulomb_viscous")
    assert ladder.optional("M2.coulomb_viscous") == ("thermal_resistance",)
    assert ladder.optional("M3.stribeck") == ("thermal_resistance", "stribeck_exponent")
    assert ladder.optional("M5.heat") == ("stribeck_exponent",)
    assert ladder.parameters("M5.heat")[-2:] == ("thermal_resistance", "thermal_time_constant")


def test_a_residual_needs_its_base_and_a_learned_net_needs_no_named_parameter():
    ladder = toy()
    assert ladder.parameters("M6.residual") == ladder.parameters("M3.lugre")
    assert ladder.parameters("M7.learned") == ()


def test_fallback_walks_down_the_bases_to_the_root():
    ladder = toy()
    assert ladder.fallback("M3.lugre") == ("M3.lugre", "M3.stribeck", "M2.coulomb_viscous", "M1.datasheet",
                                           "M0.ideal")
    assert ladder.fallback("M3.load") == ("M3.load", "M2.coulomb_viscous", "M1.datasheet", "M0.ideal")
    assert ladder.fallback("M0.ideal") == ("M0.ideal",)


def test_keys_are_composed_from_level_and_name():
    rung = Rung(3, "stribeck", "Stribeck", "M2.coulomb_viscous")
    assert rung.key == "M3.stribeck"
    assert toy().keys[:3] == ("M0.ideal", "M1.datasheet", "M2.coulomb_viscous")


def test_an_alias_resolves_to_the_canonical_name():
    ladder = toy()
    assert ladder.canonical("frictionloss_nm") == "coulomb_friction"
    assert ladder.canonical("coulomb_friction") == "coulomb_friction"
    assert ladder.parameter("sigma2").name == "viscous_friction"
    assert ladder.alias("frictionloss_nm").note == "static level, not Coulomb"
    with pytest.raises(KeyError, match="did you mean 'coulomb_friction'"):
        ladder.canonical("coulomb_frictoin")


def test_an_alias_carries_the_unit_its_source_writes():
    ladder = toy(aliases=(Alias("breakaway_nmm", "breakaway_torque", unit="N·mm"),))
    assert ladder.alias("breakaway_nmm").unit == "N·mm"
    assert Alias("sigma2", "viscous_friction").unit == ""  # "": the canonical parameter's own SI unit


def test_a_packed_array_is_never_read_as_one_parameter():
    ladder = toy()
    assert ladder.unpack("thermal_r_c") == ("thermal_resistance", "thermal_time_constant")
    assert ladder.resolve("thermal_r_c") == ("thermal_resistance", "thermal_time_constant")
    assert ladder.resolve("frictionloss_nm") == ("coulomb_friction",)
    assert ladder.packing("thermal_r_c").ordered
    with pytest.raises(KeyError, match="packs thermal_resistance, thermal_time_constant.*unpack"):
        ladder.canonical("thermal_r_c")
    with pytest.raises(KeyError, match="not in the vocabulary"):
        toy(packed=(Packed("lugre", ("sigma0", "sigma9")),))
    with pytest.raises(ValueError, match="cannot also be a packed name"):
        toy(packed=(Packed("sigma2", ("sigma0",)),))
    with pytest.raises(ValueError, match="name the canonical quantities"):
        Packed("thermal", ())


def test_an_unknown_rung_names_the_nearest():
    with pytest.raises(KeyError, match="did you mean 'M3.stribeck'"):
        toy().rung("M3.stribek")


def test_composition_is_read_through_the_bases_and_the_nearest_wins():
    rungs = (*RUNGS[:3], replace(RUNGS[3], composes=(("driver", "M1.delay"),)),
             replace(RUNGS[4], composes=(("driver", "M2.heat"),)), *RUNGS[5:])
    ladder = toy(rungs=rungs)
    assert ladder.composed("M3.lugre") == {"driver": "M2.heat"}
    assert ladder.composed("M3.wear") == {"driver": "M1.delay"}
    assert ladder.composed("M3.load") == {}


# --- held red: each planted violation refuses -------------------------------------------------------------------


def test_a_loop_of_bases_refuses():
    rungs = (*RUNGS[:3], replace(RUNGS[3], base="M3.lugre"), *RUNGS[4:])
    with pytest.raises(ValueError, match="bases loop"):
        toy(rungs=rungs)


def test_an_unknown_base_refuses():
    rungs = (*RUNGS[:4], replace(RUNGS[4], base="M3.stribek"), *RUNGS[5:])
    with pytest.raises(KeyError, match="no rung of this ladder.*did you mean 'M3.stribeck'"):
        toy(rungs=rungs)


def test_a_name_outside_the_vocabulary_refuses():
    rungs = (*RUNGS[:4], replace(RUNGS[4], adds=("sigma0", "sigma1")), *RUNGS[5:])
    with pytest.raises(KeyError, match="'sigma1', which is not in the widget vocabulary.*did you mean 'sigma0'"):
        toy(rungs=rungs)


def test_a_parameter_added_again_down_its_own_chain_refuses():
    rungs = (*RUNGS[:4], replace(RUNGS[4], adds=("sigma0", "coulomb_friction")), *RUNGS[5:])
    with pytest.raises(ValueError, match="M3.lugre adds coulomb_friction, which M2.coulomb_viscous below it already"):
        toy(rungs=rungs)


def test_an_optional_name_held_again_down_its_own_chain_refuses():
    rungs = (*RUNGS[:4], replace(RUNGS[4], optional=("viscous_friction",)), *RUNGS[5:])
    with pytest.raises(ValueError, match="lists viscous_friction as optional, which M2.coulomb_viscous below"):
        toy(rungs=rungs)
    rungs = (*RUNGS[:3], replace(RUNGS[3], optional=("stribeck_exponent", "thermal_resistance")), *RUNGS[4:])
    with pytest.raises(ValueError, match="lists thermal_resistance as optional, which M1.datasheet below"):
        toy(rungs=rungs)
    with pytest.raises(ValueError, match="both required .adds. and optional"):
        Rung(3, "odd", "Odd", "M2.coulomb_viscous", ("sigma0",), optional=("sigma0",))


def test_siblings_may_each_add_the_same_parameter():
    vocabulary = (*VOCABULARY, Parameter("harmonics", "N·m", "Ripple harmonics.", "list"))
    rungs = (*RUNGS, Rung(4, "ripple", "Ripple", "M2.coulomb_viscous", ("harmonics",)),
             Rung(4, "drivetrain", "Drivetrain", "M3.stribeck", ("harmonics",)))
    ladder = toy(vocabulary=vocabulary, rungs=rungs)
    assert "harmonics" in ladder.parameters("M4.ripple") and "harmonics" in ladder.parameters("M4.drivetrain")


def test_a_duplicate_key_refuses():
    with pytest.raises(ValueError, match="two rungs are M3.stribeck"):
        toy(rungs=(*RUNGS, Rung(3, "stribeck", "Again", "M2.coulomb_viscous")))


def test_a_second_root_refuses():
    with pytest.raises(ValueError, match="one root"):
        toy(rungs=(*RUNGS, Rung(0, "other", "Another root", None)))


def test_a_base_on_a_higher_layer_refuses():
    rungs = (*RUNGS, Rung(2, "odd", "Odd", "M3.stribeck"))
    with pytest.raises(ValueError, match="higher layer"):
        toy(rungs=rungs)


def test_a_unit_in_a_parameter_name_refuses():
    with pytest.raises(ValueError, match="carries a unit in its name.*name it 'frictionloss'"):
        Parameter("frictionloss_nm", "N·m", "MuJoCo's frictionloss.")
    with pytest.raises(ValueError, match="carries a unit"):
        Parameter("armature_kg_m2", "kg·m²", "The armature.")
    assert Parameter("fov_h", "rad", "Horizontal field of view.").name == "fov_h"  # a direction, not a unit


def test_a_rung_cites_references_not_strings():
    reference = Reference("Armstrong-Hélouvry, Dupont, Canudas de Wit (1994). A survey. Automatica.",
                          url="https://doi.org/10.1016/0005-1098(94)90209-7", doi="10.1016/0005-1098(94)90209-7")
    rung = Rung(2, "coulomb_viscous", "Coulomb + viscous", "M0.ideal", equation="tau = Fc sign(v) + b v",
                references=[reference])
    assert rung.references == (reference,)
    with pytest.raises(TypeError, match="references holds Reference"):
        Rung(2, "coulomb_viscous", "Coulomb + viscous", "M0.ideal", references=("Armstrong 1994",))
    with pytest.raises(ValueError, match="needs its title"):
        Reference("", url="https://example.org")


def test_needs_reads_a_composed_type_at_its_composed_rung():
    driver = Ladder("driver", "A drive.", (Parameter("command_latency", "s", "Delay."),),
                    (Rung(0, "ideal", "Ideal", None), Rung(1, "delay", "Delay", "M0.ideal", ("command_latency",))))
    rungs = (*RUNGS[:4], replace(RUNGS[4], composes=(("driver", "M1.delay"),)), *RUNGS[5:])
    servo = toy(rungs=rungs)
    found = {"widget": servo, "driver": driver}
    assert needs(found, "widget", "M3.lugre") == {"widget": servo.parameters("M3.lugre"),
                                                  "driver": ("command_latency",)}
    assert needs(found, "widget", "M3.stribeck") == {"widget": servo.parameters("M3.stribeck")}


def test_the_learned_kinds_sit_at_m6_and_m7_only():
    with pytest.raises(ValueError, match="M6 on every type"):
        Rung(5, "residual", "Residual", "M3.stribeck", kind="residual")
    with pytest.raises(ValueError, match="base=None"):
        Rung(7, "learned", "Net", "M6.residual", kind="learned")
    with pytest.raises(ValueError, match="physics rungs stop at M5"):
        Rung(6, "more", "More physics", "M3.stribeck")
    with pytest.raises(ValueError, match="only the root M0 has no base"):
        Rung(2, "floating", "No base", None)
    with pytest.raises(ValueError, match="not named parameters"):
        Rung(6, "residual", "Residual", "M3.lugre", ("sigma0",), kind="residual")
    with pytest.raises(ValueError, match="not named parameters"):
        Rung(6, "residual", "Residual", "M3.lugre", optional=("sigma0",), kind="residual")


def test_an_alias_must_point_at_the_vocabulary_and_never_shadow_it():
    with pytest.raises(KeyError, match="not in the vocabulary"):
        toy(aliases=(Alias("bristle_z", "bristle_deflection"),))
    with pytest.raises(ValueError, match="is canonical"):
        toy(aliases=(Alias("inertia", "coulomb_friction"),))
    with pytest.raises(ValueError, match="not deployment-only"):
        toy(deployment=(DeploymentOnly("sigma2", "config", "a gain"),))
    with pytest.raises(ValueError, match="foc_controller"):
        DeploymentOnly("torque_scale", "driver", "the drive's")


def test_a_vocabulary_entry_no_rung_adds_refuses():
    with pytest.raises(ValueError, match="mass no rung adds"):
        toy(vocabulary=(*VOCABULARY, Parameter("mass", "kg", "Mass as sold.")))
    stated = (*RUNGS[:1], replace(RUNGS[1], optional=("coulomb_friction", "thermal_resistance", "mass")), *RUNGS[2:])
    assert "mass" in toy(vocabulary=(*VOCABULARY, Parameter("mass", "kg", "Mass as sold.")), rungs=stated).holds(
        "M1.datasheet")  # a name only stated is held as optional, not an orphan


def test_a_wear_must_rate_its_own_new_parameter_on_something_the_rung_holds():
    bad_rate = replace(RUNGS[6], wears=(Wear("coulomb_friction", "deployment hours", "h", "linear", "sigma0"),))
    with pytest.raises(ValueError, match="wear rate sigma0 must be a parameter this rung adds"):
        toy(rungs=(*RUNGS[:6], bad_rate, *RUNGS[7:]))
    bad_target = replace(RUNGS[6], wears=(Wear("sigma0", "deployment hours", "h", "linear", "friction_growth"),))
    with pytest.raises(ValueError, match="sigma0 wears, but the rung does not hold it"):
        toy(rungs=(*RUNGS[:6], bad_target, *RUNGS[7:]))
    # an optional name the rung reads may wear too; one rate may wear two levels that move together
    two = replace(RUNGS[6], wears=(Wear("coulomb_friction", "deployment hours", "h", "linear", "friction_growth"),
                                   Wear("breakaway_torque", "deployment hours", "h", "linear", "friction_growth")))
    assert len(toy(rungs=(*RUNGS[:6], two, *RUNGS[7:])).rung("M3.wear").wears) == 2
    with pytest.raises(ValueError, match="law 'exponential'"):
        Wear("coulomb_friction", "deployment hours", "h", "exponential", "friction_growth")


# --- a datasheet is rung M1 --------------------------------------------------------------------------------------


def test_si_units_of_datasheet_fields():
    fields = IntegratedServo.typed_fields()
    assert si_unit(fields["kv"].units) == "rad/(s·V)"
    assert si_unit(fields["gear_ratio"].units) == ""
    assert si_unit(fields["efficiency"].units) == ""
    assert si_unit(fields["rotor_inertia"].units) == "kg·m²"
    assert si_unit(fields["max_output_speed"].units) == "rad/s"
    assert si_unit(fields["backlash"].units) == "rad"


@pytest.mark.parametrize("datasheet", [IntegratedServo, BldcMotor, Gearbox])
def test_datasheet_vocabulary_is_the_typed_fields_in_order(datasheet):
    vocabulary = datasheet_vocabulary(datasheet)
    assert [p.name for p in vocabulary] == list(datasheet.typed_fields())
    assert all(p.definition == datasheet.typed_fields()[p.name].definition for p in vocabulary)


def gearbox_ladder(m1_optional, *more: Rung, vocabulary=()) -> Ladder:
    m0 = Rung(0, "ideal", "Ideal ratio", None, ("gear_ratio", "gearbox_input_inertia"))
    m1 = Rung(1, "datasheet", "The datasheet", "M0.ideal", optional=tuple(m1_optional))
    return Ladder("gearbox", "A gear stage.", (*datasheet_vocabulary(Gearbox), *vocabulary), (m0, m1, *more),
                  datasheet=Gearbox)


def test_m1_datasheet_holds_exactly_the_typed_fields_and_needs_only_m0():
    rest = [name for name in Gearbox.typed_fields() if name not in ("gear_ratio", "gearbox_input_inertia")]
    ladder = gearbox_ladder(rest)
    assert set(ladder.holds("M1.datasheet")) == set(Gearbox.typed_fields())
    assert ladder.parameters("M1.datasheet") == ("gear_ratio", "gearbox_input_inertia")
    with pytest.raises(ValueError, match="no rung adds"):  # a typed field left off M1 is an orphan first
        gearbox_ladder(rest[:-1])
    later = Rung(2, "friction", "Friction", "M1.datasheet", ("backlash",))
    assert gearbox_ladder(rest, later).parameters("M2.friction")[-1] == "backlash"  # stated at M1, needed at M2
    extra = Rung(2, "friction", "Friction", "M1.datasheet", optional=("coulomb_friction",))
    with pytest.raises(ValueError, match="not fields: \\['coulomb_friction'\\]"):
        gearbox_ladder([*rest, "coulomb_friction"], vocabulary=(Parameter("coulomb_friction", "N·m", "Friction."),))
    assert gearbox_ladder(rest, extra, vocabulary=(Parameter("coulomb_friction", "N·m", "Friction."),))


def test_a_datasheet_field_with_another_unit_refuses():
    vocabulary = tuple(replace(p, unit="arcmin") if p.name == "backlash" else p for p in datasheet_vocabulary(Gearbox))
    rest = [name for name in Gearbox.typed_fields() if name not in ("gear_ratio", "gearbox_input_inertia")]
    m0 = Rung(0, "ideal", "Ideal ratio", None, ("gear_ratio", "gearbox_input_inertia"))
    with pytest.raises(ValueError, match="backlash has unit 'arcmin'.*'rad' in SI"):
        Ladder("gearbox", "A gear stage.", vocabulary,
               (m0, Rung(1, "datasheet", "Datasheet", "M0.ideal", optional=tuple(rest))), datasheet=Gearbox)


# --- discovery: every type folder's ladder.py, no registry ------------------------------------------------------

LADDER_FILE = """
from robot_models.ladder import Ladder, Parameter, Rung

LADDER = Ladder("{type}", "A planted type.", (Parameter("{param}", "m", "A length."),),
                (Rung(0, "ideal", "Ideal", None, ("{param}",), composes={composes}),))
"""


@pytest.fixture
def planted(tmp_path, monkeypatch):
    """A package `planted_models` in tmp_path, imported fresh, with a writer for type folders."""
    root = tmp_path / "planted_models"
    root.mkdir()
    (root / "__init__.py").write_text("")
    (root / "ladder.py").write_text("# the core's own name: discovery skips it\n")
    monkeypatch.syspath_prepend(str(tmp_path))

    def write(folder: str, text: str) -> None:
        path = root / folder
        path.mkdir(parents=True, exist_ok=True)
        for parent in [path, *path.relative_to(root).parents]:
            init = root / parent / "__init__.py"
            if not init.exists():
                init.write_text("")
        (path / "ladder.py").write_text(textwrap.dedent(text))

    yield root, write
    for name in [name for name in sys.modules if name.split(".")[0] == "planted_models"]:
        del sys.modules[name]


def test_discovery_finds_every_type_folder(planted):
    root, write = planted
    write("actuators/servo",
          LADDER_FILE.format(type="servo_like", param="length", composes=(("driver_like", "M0.ideal"),)))
    write("actuators/servo/driver", LADDER_FILE.format(type="driver_like", param="width", composes=()))
    found = discover(root, "planted_models")
    assert sorted(found) == ["driver_like", "servo_like"]
    assert found["servo_like"].folder == "actuators/servo"
    assert found["driver_like"].module == "planted_models.actuators.servo.driver.ladder"


def test_discovery_refuses_a_ladder_file_without_a_ladder(planted):
    root, write = planted
    write("sensors/encoder", "X = 1\n")
    with pytest.raises(TypeError, match="defines no LADDER"):
        discover(root, "planted_models")


def test_discovery_refuses_two_ladders_of_one_type(planted):
    root, write = planted
    write("a", LADDER_FILE.format(type="twin", param="length", composes=()))
    write("b", LADDER_FILE.format(type="twin", param="length", composes=()))
    with pytest.raises(ValueError, match="two ladders are type 'twin'"):
        discover(root, "planted_models")


def test_discovery_refuses_a_composition_of_an_unknown_type_or_rung(planted):
    root, write = planted
    write("a", LADDER_FILE.format(type="servo_like", param="length", composes=(("drivr", "M0.ideal"),)))
    with pytest.raises(KeyError, match="composes type 'drivr'"):
        discover(root, "planted_models")
    del sys.modules["planted_models.a.ladder"]
    write("a", LADDER_FILE.format(type="servo_like", param="length", composes=(("driver_like", "M1.delay"),)))
    write("b", LADDER_FILE.format(type="driver_like", param="width", composes=()))
    with pytest.raises(KeyError, match="driver_like has no rung 'M1.delay'"):
        discover(root, "planted_models")
