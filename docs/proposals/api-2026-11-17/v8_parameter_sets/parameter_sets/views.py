"""Views of a family for v8: the text tree, its mermaid source and a self-contained HTML page (never opened)."""

from __future__ import annotations

import html
from pathlib import Path
from typing import Any

class Tree:
    """The family as a tree: a run of single children prints on one line (a -> b: b subclasses a)."""

    def __init__(self, family: Any) -> None:
        self.family = family

    def _children(self, node: Any | None) -> list[Any]:
        return [m for m in self.family.ladder() if m.base is node]

    def _chains(self, node: Any) -> tuple[list[Any], list[Any]]:
        chain, below = [node], self._children(node)
        while len(below) == 1:
            chain.append(below[0])
            below = self._children(below[0])
        return chain, below

    def __str__(self) -> str:
        lines = [f"{self.family.type_name} · {len(self.family)} models (a -> b: b subclasses a; [n] param_count)"]

        def draw(node: Any, prefix: str, last: bool) -> None:
            chain, below = self._chains(node)
            keys = " -> ".join(f"{m.key} [{m.param_count}]" for m in chain)
            lines.append(prefix + ("└── " if last else "├── ") + keys)
            for i, child in enumerate(below):
                draw(child, prefix + ("    " if last else "│   "), i == len(below) - 1)

        roots = self._children(None)
        for i, root in enumerate(roots):
            draw(root, "", i == len(roots) - 1)
        return "\n".join(lines)

    def print(self) -> None:
        print(self)

    def mermaid(self) -> str:
        edges = [f'  {m.base.__name__} --> {m.__name__}["{m.key} · {m.param_count}"]' for m in self.family.ladder()
                 if m.base] + [f'  {m.__name__}["{m.key} · {m.param_count}"]' for m in self._children(None)]
        return "graph TD\n" + "\n".join(edges)

    def show(self, path: str | Path) -> Path:
        """A self-contained HTML page (no scripts, no network): the tree with each model's own parameters and units.
        Returns the path; never opens a window."""
        def node(model: Any) -> str:
            own = "".join(f"<li><code>{html.escape(n)}</code> [{html.escape(model.declared[n].unit or '1')}]"
                          f"{' (optional)' if n in model.optional else ''}</li>"
                          for n in (*model.adds, *model.optional)[:8])
            kids = "".join(node(c) for c in self._children(model))
            run = "runs" if model.runnable() else "declared"
            return (f"<details open><summary><b>{model.key}</b> · {html.escape(model.title)} · {model.param_count} "
                    f"params · {run}</summary><ul>{own}</ul>{kids}</details>")
        body = "".join(node(m) for m in self._children(None))
        page = (f"<!doctype html><meta charset=utf-8><title>{self.family.type_name} models</title><style>"
                f"body{{font:14px system-ui;margin:16px}}details{{margin-left:18px;border-left:2px solid #ccc;padding-left:8px}}"
                f"</style><h1>{self.family.type_name}</h1>{body}<h2>mermaid</h2><pre>{self.mermaid()}</pre>")
        Path(path).write_text(page)
        return Path(path)


def describe(cls: Any) -> str:
    """A model class on a few lines: key, base, title, equation, its own declarations, what it inherits, physics."""
    def line(verb: str, d: Any) -> str:
        unit = f" [{d.unit or '1'}]" if hasattr(d, "unit") else f" [{d.count} weights]"
        return f"  {verb} {d.name}{unit}: {d.definition}"
    own = [cls.declared[n] for n in (*cls.adds, *cls.optional)] or [
        d for d in cls.declared.values() if hasattr(d, "count")]
    lines = [f"{cls.key} · class {cls.__name__}({cls.base.__name__ if cls.base else 'Model'}) · {cls.title}",
             f"  {cls.equation}"]
    lines += [line("reads" if getattr(d, "optional", False) else "needs", d) for d in own]
    lines.append(f"  inherits {', '.join(n for n in cls.needs() if n not in cls.adds) or 'nothing'}; "
                 f"param_count {cls.param_count}; reads {len(cls.declared) - len(cls.needs())} more when stated")
    held = f", held to {cls.code}/model.py" if cls.code else ""
    lines.append(f"  physics: numpy{held}" if cls.runnable() else "  physics: declared, not runnable here")
    return "\n".join(lines)
