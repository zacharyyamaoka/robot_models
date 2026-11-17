"""The round-2 access shape (docs/proposals/api-2026-11-17/ROUND2.md) on the real package, named as PEP 0004
amendment 2 records: `class IntegratedServo(ModelFamily)`, every level of it a ModelFamily too (`IntegratedServo.m3`),
no `.models` and no `.Datasheet` on it, M1's class `DatasheetModel` reading `IntegratedServoDatasheet`'s fields.

The logs here are synthetic velocity-plateau sweeps generated in this file from the package's own rung physics; the
round-2 fixture and its hidden truths stay in docs/proposals (check_scenario.py package_v6 runs that scenario).
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

import robot_models
from robot_models import IntegratedServo
from robot_models.actuators.servo import IntegratedServo as ServoDatasheetAlias
from robot_models.actuators.servo import IntegratedServoDatasheet
from robot_models.datasheet import Datasheet, Stated
from robot_models.ladder import Ladder, ladders
from robot_models.model_family import BlackBox, Fits, Model, ModelFamily, Parameter, Selection, Tree

SRC = Path(robot_models.__file__).resolve().parent
PLATEAUS = (0.03, 0.06, 0.1, 0.18, 0.3, 0.5, 0.8, 1.3, 2.0, 3.0)
STRIBECK = {"coulomb_friction": 0.42, "breakaway_torque": 0.65, "stribeck_velocity": 0.25, "viscous_friction": 0.035}
COULOMB_VISCOUS = {"coulomb_friction": 0.42, "viscous_friction": 0.035}


def sweep(truth: dict, split: str, seed: int = 7, noise: float = 0.004) -> dict[str, np.ndarray]:
    """Both directions of each plateau, 25 samples each; "fit" keeps plateaus 1, 3, 5, 7, 9 and "test" the others."""
    rng = np.random.default_rng(seed)
    index = np.repeat(np.arange(len(PLATEAUS)), 25)
    v = np.concatenate([np.array(PLATEAUS)[index], -np.array(PLATEAUS)[index]])
    plateau = np.concatenate([index + 1, -(index + 1)])
    rung = IntegratedServo.m3.stribeck if "breakaway_torque" in truth else IntegratedServo.m2.coulomb_viscous
    tau = rung._of(truth).predict(v) + rng.normal(0.0, noise, v.shape)
    keep = (np.abs(plateau) % 2 == 1) == (split == "fit")
    return {"v": v[keep], "tau": tau[keep], "plateau": plateau[keep]}


# --- the names -----------------------------------------------------------------------------------------------------


def test_the_servo_is_a_model_family_and_robot_models_knows_no_part_type():
    assert issubclass(IntegratedServo, ModelFamily) and IntegratedServo.id == "integrated_servo"
    assert issubclass(IntegratedServo.m3, ModelFamily) and issubclass(IntegratedServo["M3"], ModelFamily)
    assert not hasattr(IntegratedServo, "models") and not hasattr(IntegratedServo, "Datasheet")
    assert not hasattr(robot_models, "PartType")
    for path in SRC.rglob("*.py"):
        assert "class PartType" not in path.read_text(encoding="utf-8"), path


def test_m1_is_datasheet_model_reading_the_datasheet_class():
    m1 = IntegratedServo.m1.datasheet
    assert m1.__name__ == "DatasheetModel" and m1.key == "M1.datasheet"
    assert set(IntegratedServoDatasheet.typed_fields()) <= set(m1.parameters)
    assert IntegratedServo.v1_ladder().datasheet is IntegratedServoDatasheet
    assert issubclass(IntegratedServoDatasheet, Datasheet)


def test_the_old_datasheet_name_stays_an_alias_where_robotparts_imports_it():
    from robot_models.actuators.servo.integrated_servo import IntegratedServo as module_alias

    assert ServoDatasheetAlias is IntegratedServoDatasheet and module_alias is IntegratedServoDatasheet
    assert ladders()["integrated_servo"].datasheet is IntegratedServoDatasheet


def test_calling_the_family_refuses_and_names_where_values_live():
    with pytest.raises(TypeError, match=r"model family, not a value.*IntegratedServo\.m<level>\.<word>\(\.\.\.\)"):
        IntegratedServo(peak_torque=Stated(3.5, "N·m"))
    with pytest.raises(TypeError, match=r"IntegratedServo\.m3 is a model family"):
        IntegratedServo.m3()


def test_a_datasheet_refusal_still_names_the_field_on_the_type():
    with pytest.raises(ValueError, match=r"^IntegratedServo\.rated_torque: unit 'N' "):
        IntegratedServoDatasheet(rated_torque=Stated(1.05, "N"))
    assert IntegratedServoDatasheet.type_name() == "IntegratedServo" and Datasheet.type_name() == "Datasheet"


# --- his access shape (ROUND2.md), as written ---------------------------------------------------------------------


def test_models_by_key_by_attribute_and_by_level():
    models = IntegratedServo
    assert models.m3 is models["M3"] is models.m3  # one family object per level
    assert models["M3.stribeck"] is models.m3.stribeck is models.stribeck is IntegratedServo.Stribeck
    assert models.m3.keys() == ("M3.stribeck", "M3.load", "M3.lugre", "M3.weakening_capture")
    assert models["M3"].keys() == models.m3.keys()
    assert models.m3["M3.load"] is models.m3.load
    with pytest.raises(AttributeError, match="no M9 models"):
        models.m9  # noqa: B018
    with pytest.raises(KeyError, match="'M3.nope': no model or level"):
        models["M3.nope"]
    with pytest.raises(AttributeError, match="has no model 'stribeck'"):
        models.m2.stribeck  # noqa: B018


@pytest.mark.parametrize("level", [None, "m3"])
def test_every_family_has_fit_select_best_ladder_and_tree(level):
    family = IntegratedServo if level is None else getattr(IntegratedServo, level)
    for call in ("fit", "select_best", "ladder", "tree"):
        assert callable(getattr(family, call)), call
    assert isinstance(family.tree(), Tree) and family.ladder() == tuple(family)


def test_the_family_iterates_ladders_and_fits_its_own_models():
    assert list(IntegratedServo) == list(IntegratedServo.values()) and len(IntegratedServo) == 14
    assert IntegratedServo.ladder()[0].key == "M0.ideal"
    assert [m.key for m in IntegratedServo.ladder(IntegratedServo.m3.stribeck)] == [
        "M3.stribeck", "M2.coulomb_viscous", "M1.datasheet", "M0.ideal"]
    assert IntegratedServo.tree().text().splitlines()[0].startswith("integrated_servo · IntegratedServo · 14 models")
    assert IntegratedServo.m3.tree().text().splitlines()[0].startswith("integrated_servo · IntegratedServo.m3 · 4 models")
    log = sweep(COULOMB_VISCOUS, "fit")
    assert set(IntegratedServo.fit(log)) | set(IntegratedServo.fit(log).skipped) == set(IntegratedServo.keys())


def test_iterating_the_type_gives_each_model_with_its_param_count():
    rows = [(model.key, model.param_count) for model in IntegratedServo]
    assert rows[:4] == [("M0.ideal", 2), ("M1.datasheet", 2), ("M2.coulomb_viscous", 4), ("M3.stribeck", 6)]
    assert rows[-1] == ("M7.learned", 8) and len(rows) == 14


def test_the_tree_is_one_line_up_and_down_and_branches_where_the_type_does(tmp_path):
    lines = IntegratedServo.tree().text().splitlines()
    assert lines[1] == "├── M0.ideal -> M1.datasheet -> M2.coulomb_viscous"
    assert "│   │   └── M3.weakening_capture -> M4.drivetrain -> M5.envelope -> M5.wear -> M6.residual" in lines
    assert lines[-1] == "└── M7.learned"
    level = IntegratedServo.m3.tree().text().splitlines()
    assert level[1:] == ["├── M3.stribeck", "│   ├── M3.lugre", "│   └── M3.weakening_capture", "└── M3.load"]
    page = IntegratedServo.tree().show(tmp_path / "tree.html")
    assert "graph BT" in page.read_text(encoding="utf-8") and "M3_lugre --> M3_stribeck" in page.read_text()


# --- one source, additive vocabulary ------------------------------------------------------------------------------


def test_the_derived_ladder_equals_todays_record():
    derived = IntegratedServo.v1_ladder()
    assert isinstance(derived, Ladder) and derived == ladders()["integrated_servo"]


def test_a_rung_cannot_redefine_remove_or_invent_an_inherited_parameter():
    stribeck = IntegratedServo.m3.stribeck
    with pytest.raises(TypeError, match="redefines inherited"):
        type("Newtons", (stribeck,), {"coulomb_friction": Parameter("N", "Coulomb friction, in newtons.")},
             level=3, name="newtons")
    with pytest.raises(TypeError, match="removes an inherited parameter"):
        type("Forgetful", (stribeck,), {"breakaway_torque": None}, level=3, name="forgetful")
    with pytest.raises(TypeError, match="names no inherited parameter"):
        type("Ghost", (stribeck,), {"ghost": Parameter()}, level=3, name="ghost")
    with pytest.raises(TypeError, match="the Model API uses that name"):
        type("Clash", (stribeck,), {"fit": Parameter("N·m", "A parameter named like a call.")}, level=3, name="clash")
    assert len(IntegratedServo) == 14  # none of the refused rungs joined


def _scratch_type():
    class Gear(ModelFamily, id="scratch_gear"):
        """A scratch type: two rungs."""

        class Ideal(Model, level=0, name="ideal"):
            """Ideal"""
            gear_ratio = Parameter("", "Rotor turns per output turn.")

        class Lossy(Ideal, level=1, name="lossy"):
            """Lossy"""
            efficiency = Parameter("", "Power out over power in.")

    return Gear


def test_a_rung_declared_later_joins_its_type_and_the_derived_record():
    gear = _scratch_type()
    before = gear.v1_ladder().keys

    class Worn(gear.m1.lossy, level=2, name="worn"):
        """Worn"""
        wear_rate = Parameter("1/h", "How fast efficiency falls per hour.")

    assert gear.m2.worn is Worn and Worn.family is gear
    assert before == ("M0.ideal", "M1.lossy") and gear.v1_ladder().keys == ("M0.ideal", "M1.lossy", "M2.worn")


def test_a_rung_word_that_shadows_a_group_call_refuses():
    gear = _scratch_type()
    for word in ("fit", "keys", "id", "aliases"):
        with pytest.raises(TypeError, match=f"'{word}' is a ModelFamily call"):
            type("Clash", (gear.m1.lossy,), {"__doc__": "x"}, level=2, name=word)


# --- per model and batch: analytical and black box with one interface -------------------------------------------------


def test_a_model_fit_returns_an_instance_that_predicts_numpy_and_is_frozen():
    log = sweep(STRIBECK, "fit")
    m3 = IntegratedServo.m3.stribeck.fit(log)
    assert isinstance(m3, IntegratedServo.Stribeck) and m3.fitted_count == 4
    assert m3.breakaway_torque == pytest.approx(STRIBECK["breakaway_torque"], rel=0.05)
    assert IntegratedServo.Stribeck.breakaway_torque.unit == "N·m"  # the class holds the Parameter, the instance its value
    v = np.linspace(-3, 3, 1001)
    assert m3.predict(v).shape == v.shape
    assert m3.predict(v)[123] == pytest.approx(m3.predict(float(v[123])), rel=1e-12)
    with pytest.raises(AttributeError, match="values are fixed"):
        m3.breakaway_torque = 1.0


def test_the_black_box_has_the_physics_outside():
    learned = IntegratedServo.m7.learned
    assert issubclass(learned, BlackBox) and learned.param_count == 8
    log = sweep(STRIBECK, "fit")
    net, m3 = learned.fit(log), IntegratedServo.m3.stribeck.fit(log)
    for model in (net, m3):
        assert isinstance(model.predict(1.0), float) and model.predict(log["v"]).shape == log["v"].shape
        assert model.score(log) < 0.05 and model.param_count > 0


def test_the_batch_fit_and_select_best_choose_on_held_out_plateaus():
    fits = IntegratedServo.fit(sweep(STRIBECK, "fit"))
    assert isinstance(fits, Fits)
    assert {"M2.coulomb_viscous", "M3.stribeck", "M7.learned"} <= set(fits)
    assert "tau_load" in fits.skipped["M3.load"] and "sigma0" in fits.skipped["M3.lugre"]
    best = IntegratedServo.select_best(sweep(STRIBECK, "test"), fits)
    assert isinstance(best, Selection) and best.chosen.key == "M3.stribeck"
    assert "M7.learned" in [m.key for m in best.models]  # the black box competed with the same call
    assert fits.select_best(sweep(STRIBECK, "test")).chosen.key == "M3.stribeck"
    assert "<- chosen" in str(best) and "M3.load" in str(fits)


def test_the_simpler_rung_wins_when_the_held_out_plateaus_do_not_need_more():
    fits = IntegratedServo.fit(sweep(COULOMB_VISCOUS, "fit"))
    assert fits.select_best(sweep(COULOMB_VISCOUS, "test")).chosen.key == "M2.coulomb_viscous"


def test_a_level_group_fits_and_chooses_only_its_own_models():
    log, held_out = sweep(STRIBECK, "fit"), sweep(STRIBECK, "test")
    level = IntegratedServo.m3.fit(log)
    assert set(level) == {"M3.stribeck"} and set(level.skipped) == {"M3.load", "M3.lugre", "M3.weakening_capture"}
    every = IntegratedServo.fit(log)
    assert [m.key for m in IntegratedServo.m2.select_best(held_out, every).models] == ["M2.coulomb_viscous"]
    assert IntegratedServo.m3.select_best(held_out, every).chosen.key == "M3.stribeck"


# --- the stack ----------------------------------------------------------------------------------------------------


def test_values_hand_to_mujoco_and_sim2real_in_canonical_si_names():
    m3 = IntegratedServo.m3.stribeck.fit(sweep(STRIBECK, "fit"))
    assert set(m3.values()) >= {"coulomb_friction", "breakaway_torque", "stribeck_velocity", "viscous_friction"}
    assert m3.mujoco()["frictionloss"] == m3.breakaway_torque and m3.mujoco()["damping"] == m3.viscous_friction


def test_the_ladders_and_datasheets_still_import_without_numpy():
    code = ("import sys, robot_models.ladder, robot_models.datasheet, robot_models.actuators.servo\n"
            "from robot_models.ladder import ladders; ladders()\n"
            "sys.exit('numpy' in sys.modules)\n")
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=False,
                            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    assert result.returncode == 0, result.stderr or "importing the ladders pulled in numpy"


def test_fitting_leaves_no_bytecode_beside_the_vendored_rung_code(monkeypatch):
    from robot_models import model_family

    monkeypatch.setattr(sys, "dont_write_bytecode", False)  # as a plain `python` run would
    before = set((SRC / "actuators" / "rungs").rglob("__pycache__"))
    model_family._reference.cache_clear()  # load every rung's model.py again
    IntegratedServo.fit(sweep(STRIBECK, "fit"))
    assert set((SRC / "actuators" / "rungs").rglob("__pycache__")) == before
