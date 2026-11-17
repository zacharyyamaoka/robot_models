"""Typed datasheets from sources (PEP 0003): a wrong unit or an unknown name refuses loudly, a part's values resolve
from its sources by an explicit rule, every disagreement stays visible, and a source file stays literal-only.

Every source here is a synthetic file written into tmp_path: robot_models never reads robotparts (the real parts are
robotparts' user/test_datasheets.py). Held red: a planted bad value must fail the same checks a good file passes.
"""

from __future__ import annotations

import textwrap

import pytest

from robot_models.actuators.servo import BldcMotor, Gearbox, IntegratedServo
from robot_models.datasheet import (
    EXTRA,
    KEY_SPECS,
    ColumnError,
    Datasheet,
    Source,
    Sources,
    Stated,
    literal,
    load_source,
    units,
)

HEADER = "from robot_models.datasheet import Source, Stated\n\n"


def part(tmp_path, **files: str) -> Sources:
    """A part folder with sources/<name>.py for each keyword; returns its Sources."""
    folder = tmp_path / "some-part"
    (folder / "sources").mkdir(parents=True)
    for name, body in files.items():
        (folder / "sources" / f"{name}.py").write_text(HEADER + textwrap.dedent(body), encoding="utf-8")
    (folder / "some-part.py").write_text("")
    return Sources(folder / "some-part.py")


MAKER = """
    source = Source("Maker page", kind="manufacturer", url="https://maker.example/p", columns=("A", "B"))

    nominal_voltage = Stated(24, "V", "Nominal Voltage")
    rated_torque = Stated({"A": 1.05, "B": 1}, "N·m", "Nominal Torque")
    peak_torque = Stated({"A": 3.82, "B": 3.47}, "N·m", "Stall Torque")
    gear_ratio = Stated(10, ":1", "Gear Rate")
    kv = Stated({"A": 16.75, "B": 15.25}, "rpm/V", "Speed Constant", at="output")
    kt = Stated({"A": 0.39, "B": 0.62}, "N·m/A", "Torque Constant", at="output")
    mass = Stated(140, "g", "Motor Weight")

    extra = {"noise": Stated(60, "dB", "Noise", bound="max")}
"""

SHOP = """
    source = Source("Shop listing", kind="retailer", url="https://shop.example/p")

    rated_torque = Stated(1.05, "N·m", "Rated torque")
    peak_torque = Stated(4.0, "N·m", "Peak torque")
    mass = Stated(0.15, "kg", "Weight")
"""

ESTIMATE = """
    source = Source("Our estimates", kind="estimated")

    rotor_inertia = Stated(0.0014, "kg·m²", at="output", basis="the rack's fit")
"""


# --- the type ----------------------------------------------------------------------------------------------------

def test_types_mirror_the_parts_tree_and_extend_each_other():
    assert set(BldcMotor.typed_fields()) < set(IntegratedServo.typed_fields())
    assert {"gear_ratio", "backlash", "max_axial_load"} <= set(IntegratedServo.typed_fields()) - set(BldcMotor.typed_fields())
    assert Gearbox.KEY_SPECS == ("gear_ratio", "rated_torque", "peak_torque")
    groups = dict(IntegratedServo.rows())
    assert groups[KEY_SPECS] == ["peak_torque", "rated_torque", "max_output_speed", "mass"]
    assert sum(len(names) for names in groups.values()) == len(IntegratedServo.typed_fields())
    assert EXTRA not in groups


def test_a_wrong_unit_refuses_loudly_naming_the_field_and_its_units():
    with pytest.raises(ValueError, match=r"IntegratedServo\.rated_torque: unit 'N' .*'N·m'"):
        IntegratedServo(rated_torque=Stated(1.05, "N"))
    with pytest.raises(ValueError, match="rotor_inertia: unit 'kg·m'"):
        BldcMotor(rotor_inertia=Stated(0.14, "kg·m"))


def test_an_unknown_name_refuses_loudly():
    with pytest.raises(TypeError, match="nominal_votlage"):
        IntegratedServo(nominal_votlage=Stated(24, "V"))
    with pytest.raises(ValueError, match="extra 'mass' is a typed field"):
        IntegratedServo(extra={"mass": Stated(140, "g")})


def test_a_typo_in_a_source_refuses_with_the_nearest_field(tmp_path):
    sources = part(tmp_path, maker='source = Source("M", kind="manufacturer")\nnominal_votlage = Stated(24, "V")\n')
    with pytest.raises(KeyError, match="nominal_votlage is not a field of IntegratedServo.*did you mean nominal_voltage"):
        IntegratedServo.from_sources(sources, prefer=[sources.maker])


def test_a_custom_value_is_written_in_extra_and_a_typed_name_there_refuses(tmp_path):
    sources = part(tmp_path, maker=MAKER)
    sheet = IntegratedServo.from_sources(sources, prefer=[sources.maker["A"]])
    assert sheet.extra["noise"].text() == "< 60 dB"
    bad = part(tmp_path / "x", maker='source = Source("M", kind="manufacturer")\nextra = {"mass": Stated(1, "kg")}\n')
    with pytest.raises(KeyError, match="extra 'mass' is a typed field"):
        IntegratedServo.from_sources(bad, prefer=[bad.maker])


# --- resolving ---------------------------------------------------------------------------------------------------

def test_prefer_gives_each_field_from_the_first_source_that_states_it(tmp_path):
    sources = part(tmp_path, maker=MAKER, shop=SHOP, estimates=ESTIMATE)
    sheet = IntegratedServo.from_sources(sources, prefer=[sources.shop, sources.maker["A"], sources.estimates])
    assert sheet.peak_torque.value == 4.0 and sheet.peak_torque.source.name == "shop"
    assert sheet.nominal_voltage.value == 24 and sheet.nominal_voltage.source.label == "maker·A"
    # the accessor still reaches the source's own words and the source
    assert sheet.nominal_voltage.as_written == "Nominal Voltage"
    assert sheet.nominal_voltage.source.url == "https://maker.example/p"
    assert sources.maker.nominal_voltage.as_written == "Nominal Voltage"


def test_pick_takes_one_field_from_a_named_source(tmp_path):
    sources = part(tmp_path, maker=MAKER, shop=SHOP)
    sheet = IntegratedServo.from_sources(sources, prefer=[sources.maker["A"], sources.shop],
                                         pick={"peak_torque": sources.maker["B"]})
    assert sheet.peak_torque.value == 3.47 and sheet.peak_torque.source.column == "B"
    assert sheet.rated_torque.value == 1.05
    with pytest.raises(KeyError, match="states no backlash"):
        IntegratedServo.from_sources(sources, prefer=[sources.maker["A"]], pick={"backlash": sources.shop})


def test_a_value_that_differs_by_column_refuses_until_a_column_is_named(tmp_path):
    sources = part(tmp_path, maker=MAKER)
    assert sources.maker.nominal_voltage.value == 24  # the same in every column: fine
    with pytest.raises(ColumnError, match=r"maker\['A'\]\.rated_torque"):
        _ = sources.maker.rated_torque
    assert sources.maker["B"].rated_torque.value == 1
    with pytest.raises(ColumnError):
        IntegratedServo.from_sources(sources, prefer=[sources.maker])


def test_a_rotor_side_value_stated_at_the_output_is_carried_back_through_the_gear(tmp_path):
    sources = part(tmp_path, maker=MAKER, estimates=ESTIMATE)
    sheet = IntegratedServo.from_sources(sources, prefer=[sources.maker["B"], sources.estimates])
    assert sheet.kv.value == pytest.approx(152.5) and sheet.kv.is_derived
    assert sheet.kv.derived_from.value == 15.25 and sheet.kv.derived_from.at == "output"
    assert sheet.kt.value == pytest.approx(0.062)
    assert sheet.rotor_inertia.si == pytest.approx(0.0014 / 100)  # output kg·m² / 10² = rotor
    assert "gear_ratio 10" in sheet.kv.basis
    no_gear = part(tmp_path / "x", maker='source = Source("M", kind="manufacturer")\n'
                                          'kv = Stated(15.25, "rpm/V", at="output")\n')
    with pytest.raises(ValueError, match="no source gives gear_ratio"):
        IntegratedServo.from_sources(no_gear, prefer=[no_gear.maker])


def test_every_disagreement_is_listed_never_silently_resolved(tmp_path):
    sources = part(tmp_path, maker=MAKER, shop=SHOP)
    sheet = IntegratedServo.from_sources(sources, prefer=[sources.maker["A"], sources.shop])
    found = {item.name: item for item in sheet.disagreements()}
    assert {stated.value for stated in found["peak_torque"].others} == {3.47, 4.0}
    assert "rated_torque" in found  # column B's 1 N·m against A's 1.05
    # the same mass in g and in kg is agreement, not a disagreement: compared in SI
    assert sheet.mass.value == 140 and sources.shop.mass.value == 0.15
    assert {stated.value for stated in found["mass"].others} == {0.15}  # 150 g ≠ 140 g
    assert sheet.agrees("mass", sources.maker["B"].mass)


def test_a_source_for_another_model_is_shown_never_preferred(tmp_path):
    sources = part(tmp_path, sibling='source = Source("Sibling", kind="manufacturer", applies_to="X-2")\n'
                                     'nominal_voltage = Stated(48, "V")\n', shop=SHOP)
    with pytest.raises(ValueError, match="is for X-2"):
        BldcMotor.from_sources(sources, prefer=[sources.sibling])
    sheet = BldcMotor.from_sources(sources, prefer=[sources.shop])
    assert sheet.nominal_voltage is None and "nominal_voltage" in sheet.gaps()
    assert [stated.value for stated in sheet.statements("nominal_voltage")] == [48]
    picked = BldcMotor.from_sources(sources, prefer=[sources.shop], pick={"nominal_voltage": sources.sibling})
    assert picked.nominal_voltage.value == 48
    # shown beside the values, never counted as a disagreement: a sibling's winding is expected to differ
    sibling_mass = part(tmp_path / "m", shop=SHOP, sibling='source = Source("S", kind="manufacturer", applies_to="X-2")\n'
                                                           'mass = Stated(2, "kg")\n')
    sheet = BldcMotor.from_sources(sibling_mass, prefer=[sibling_mass.shop])
    assert [stated.value for stated in sheet.statements("mass")] == [0.15, 2] and sheet.disagreements() == []


def test_gaps_are_every_typed_field_no_source_states(tmp_path):
    sources = part(tmp_path, shop=SHOP)
    sheet = BldcMotor.from_sources(sources, prefer=[sources.shop])
    assert set(sheet.gaps()) == set(BldcMotor.typed_fields()) - {"rated_torque", "peak_torque", "mass"}


# --- held red: a planted bad value -------------------------------------------------------------------------------

@pytest.mark.parametrize("line, message", [
    ('rotor_inertia = Stated(0.14, "kg·m")', "rotor_inertia: unit 'kg·m'"),       # a unit typo
    ('mass = Stated(-140, "g")', "mass is -140: it must be > 0"),                   # a sign slip
    ('pole_pairs = Stated(14.5, "")', "pole_pairs is 14.5: a whole number"),         # not a count
    ('rated_torque = Stated("1.05", "N·m")', "rated_torque is '1.05': a number"),     # text where a number goes
    ('backlash = Stated(15, "arcmin", at="output")', "backlash says at='output'"),    # a side the field has not
])
def test_a_planted_bad_value_fails_the_source_check(tmp_path, line, message):
    good = part(tmp_path / "good", maker=MAKER)
    IntegratedServo.from_sources(good, prefer=[good.maker["A"]])  # the same check passes the good file
    bad = part(tmp_path / "bad", maker=textwrap.dedent(MAKER) + "\n" + line + "\n")
    with pytest.raises((ValueError, TypeError), match=message):
        IntegratedServo.from_sources(bad, prefer=[bad.maker["A"]])


def test_a_source_file_is_checked_when_loaded(tmp_path):
    with pytest.raises(ValueError, match="gives columns A, C but the header has A, B"):
        part(tmp_path / "a", maker='source = Source("M", kind="manufacturer", columns=("A", "B"))\n'
                                   'rated_torque = Stated({"A": 1, "C": 2}, "N·m")\n')
    with pytest.raises(ValueError, match="kind 'blog'"):
        part(tmp_path / "b", maker='source = Source("M", kind="blog")\n')
    with pytest.raises(ValueError, match="needs `source = Source"):
        part(tmp_path / "c", maker='mass = Stated(1, "kg")\n')
    with pytest.raises(ValueError, match="is not a Stated"):
        part(tmp_path / "d", maker='source = Source("M", kind="manufacturer")\nmass = 1.0\n')


# --- literal only ------------------------------------------------------------------------------------------------

def test_a_literal_source_file_passes_and_a_planted_computation_fails(tmp_path):
    good = tmp_path / "good.py"
    good.write_text('"""doc"""\n' + HEADER + textwrap.dedent(MAKER))
    assert literal.problems(good) == []
    for planted in ['rated_torque = Stated(1.05 * 10, "N·m")\n', 'import os\n', 'x = 1\n',
                    'mass = Stated(MASS, "g")\n', 'extra = {name: Stated(1, "g")}\n']:
        bad = tmp_path / "bad.py"
        bad.write_text(HEADER + planted)
        assert literal.problems(bad), planted


def test_units_convert_to_si_and_refuse_an_unknown_unit():
    assert units.to_si(15.25, "rpm/V") == pytest.approx(15.25 * 2 * 3.141592653589793 / 60)
    assert units.to_si(0.14, "kg·cm²") == pytest.approx(1.4e-5)
    assert units.to_si(5.6, "V/krpm") == pytest.approx(0.0534761, rel=1e-5)
    with pytest.raises(ValueError, match="unknown unit 'N.M'"):
        units.to_si(1, "N.M")


def test_load_source_binds_each_value_to_its_source(tmp_path):
    sources = part(tmp_path, maker=MAKER)
    maker = load_source(tmp_path / "some-part" / "sources" / "maker.py")
    assert maker.name == "maker" and maker.folder == tmp_path / "some-part"
    assert maker["A"].peak_torque.source.label == "maker·A"
    assert maker["A"].peak_torque.anchor == "text=Stall%20Torque"
    assert isinstance(sources.maker, Source) and issubclass(IntegratedServo, Datasheet)
