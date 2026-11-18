# Feedback ledger (newest first)

- **2026-11-18 · Bad · standalone + legible** — Code shown to a reader must run exactly as shown, and an API must read at a glance:
  - every displayed block runs top to bottom in a fresh `python -I` with nothing pre-imported: every name imported or defined on the page, the data's origin shown; a harness's hidden setup namespace doesn't count;
  - prove it with `check_standalone.py`, which extracts exactly the displayed code, red first;
  - pseudo-code only when incredibly obvious, never in an "API at a glance";
  - plain words: verbs a Python user already knows (update / compute / reset, a function call, group_by / agg) and readable outputs, never `Collection(15 aliases over 5 states)`. Jargon (fold, state, merge, close, strata, alias) gets a plain one-line meaning at first use, or better, isn't needed;
  - an API design round carries a weighted "legible at a glance" criterion.

  Zach: "I cannot read this pseudo code because its not valid … where did you import it from", then "I am not seeing what is going on at a glance. I don't know what you mean by fold etc." The global rule lives in his readable-Python memory.
  <sub>rate note: `rate - Nov 18 2026, 2.29.31PM.md` · [open](obsidian://advanced-uri?vault=zach_brain&filepath=Settings%2FFeedback%2Frate%2Frate%20-%20Nov%2018%202026%2C%202.29.31PM.md&openmode=true)</sub>
- **2026-11-18 · Bad · readability** — Python shown to Zach in a report or babble gallery must read like good Python:
  - syntax-highlighted (Pygments at build time), with output in a quieter style;
  - notebook rhythm: one statement, then its own output, with a multi-line block only where it's the natural unit (a loop, a def);
  - one concept per line: name intermediates, no comprehensions inside f-strings, ~90-character lines;
  - blank lines between ideas inside a block;
  - when the point is an API, a short runnable "API at a glance" snippet first, so designs compare at a glance.

  Zach: "if this was python code I would say its not very readable right now!" The global rule lives in his readable-Python memory.
  <sub>rate note: `rate - Nov 18 2026, 9.55.12AM.md` · [open](obsidian://advanced-uri?vault=zach_brain&filepath=Settings%2FFeedback%2Frate%2Frate%20-%20Nov%2018%202026%2C%209.55.12AM.md&openmode=true)</sub>
