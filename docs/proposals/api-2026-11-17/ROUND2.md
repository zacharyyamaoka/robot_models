# Round 2: ModelFamily → Model → Parameters (2026-11-17)

Zach's feedback on round 1 ([[Daily Note - Nov 17 2026]] line 60): V1 Subclass ladder is closest. "I 100% like
organizing everything around a type." "The key idea is we want a ModelFamily, Model, Parameters." "A model should
100% be a class", and subclassing fits "as we only ever want vocabulary to be additive".

## The shape every variant must give (his calls; exact spellings may differ)

```python
from robot_models import IntegratedServo          # a ModelFamily: the type
IntegratedServo.models["M3.stribeck"]             # by key
IntegratedServo.models.m3.stribeck                # by attribute; "m3" is itself a grouping of models
IntegratedServo.ladder()[0]                       # an ordered ladder
IntegratedServo.tree().print()                    # text tree; a ladder is one line up and down, a tree branches
IntegratedServo.tree().show()                     # a visually richer display (HTML/mermaid/...)
for model in IntegratedServo: print(model.key, model.param_count)
model.fit(log); model.predict(v)                  # per model
IntegratedServo.fit(log); IntegratedServo.select_best(held_out)   # batch: the family deals with the complications
```

- **ModelFamily** is composed of Models; **Model** is a class composed of Parameters; **Parameter** carries a value
  plus its unit, definition and other info.
- A rung subclasses its base, so a model's vocabulary only grows.
- Analytical and black-box models look the same from outside (`fit`, `predict`, `param_count`).

## Same scenario and check as round 1

[fixture.py](fixture.py), [check_scenario.py](check_scenario.py) with amendment 1 (fit on plateaus 1,3,5,7,9;
choose on the held-out ones; case B must choose M2).

## Weighted criteria (fixed before the variants exist; sum 100)

| id | criterion | weight | 5 means |
|---|---|---|---|
| fr1 | **His access shape** | 25 | every call above works as written (or one obvious spelling away), reads naturally, iteration and `param_count` included |
| fr2 | **Per-model and batch operations** | 20 | `model.fit`, `Family.fit`, `Family.select_best(held_out)` are one call each and the batch result is easy to read |
| fr3 | **One source, additive vocabulary** | 15 | the classes ARE the source: today's `Ladder` is derived from them and matches `ladders()["integrated_servo"]` exactly; a subclass cannot remove or rename; nothing drops silently |
| fr4 | **Analytical and black box, one interface** | 15 | a learned model (a small numpy black box is enough) fits, predicts, counts its parameters and competes in `select_best` with the same calls |
| fr5 | **Migration** | 10 | the v1 ruler, robotparts' vendored rung bytes and sim2real's parity test keep working |
| fr6 | **Plays with the stack** | 15 | type checkers see each model's parameters; numpy-vectorized predict; values hand to MuJoCo/sim2real without glue |

Hard gates as round 1 (G1 type data only, G2 numpy + robot_msgs + stdlib, G3 the check passes without reading the
truth, G4 one vocabulary per type).

## The three directions: where the values live

| | name | values live on | consequence |
|---|---|---|---|
| v6 | Fitted instances | a Model **instance** (`Stribeck(coulomb_friction=...)`); classes are the type | V1 refined: the family's batch calls return instances |
| v7 | Family per unit | a ModelFamily **instance**, one per physical unit (`servo = IntegratedServo(datasheet)`), holding one fitted instance of each Model | the unit is the object you carry around; batch ops mutate/return it |
| v8 | Parameters sets | a **Parameters** object (a typed value set) passed to stateless Model classes (`Stribeck.predict(params, v)`) | values are data you can store, diff and share; models are pure physics |
