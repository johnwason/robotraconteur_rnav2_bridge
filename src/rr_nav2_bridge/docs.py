"""Serve robdef documentation to clients.

Robot Raconteur transmits service definitions without their ## comments, so the root
object exposes documentation(name), backed by this index of the original robdef text.
"""
from __future__ import annotations

import re

_DECL = re.compile(r"^\s*(struct|object|enum|namedarray)\s+(\w+)")
_MEMBER = re.compile(
    r"^\s*(?:field|property|function|wire|pipe|memory|event|objref|callback)\s+(.+)$")
_OBJREF = re.compile(r"^\s*objref\s+(\w+)\s+(\w+)")


class DocIndex:
    def __init__(self, text: str):
        self.text = text
        self.entries: dict[str, str] = {}
        self.objref_types: dict[str, str] = {}
        self._parse()

    def _parse(self):
        doc: list[str] = []
        current = kind = None
        for line in self.text.splitlines():
            s = line.strip()
            if s.startswith("##"):
                doc.append(s[2:].removeprefix(" "))
                continue
            if not s or s.startswith("#"):
                continue
            entry = "\n".join(doc + ["", s]).lstrip("\n")
            if s.startswith("service "):
                self.entries[""] = "\n".join(doc)
            elif (m := _DECL.match(line)):
                kind, current = m.group(1), m.group(2)
                self.entries[current] = entry
            elif s == "end":
                current = kind = None
            elif current and kind == "enum":
                self.entries[f"{current}.{s.split('=')[0].strip().rstrip(',')}"] = entry
            elif current and (m := _MEMBER.match(line)):
                name = m.group(1).split("(")[0].split("[")[0].split()[-1]
                self.entries[f"{current}.{name}"] = entry
                if (o := _OBJREF.match(line)):
                    self.objref_types[o.group(2)] = o.group(1)
            doc = []

    def lookup(self, name: str, root: str = "Nav2Navigator") -> str:
        name = (name or "").strip()
        if not name:
            return self.text
        candidates = [name, f"{root}.{name}"]
        if "." in name:
            head, rest = name.split(".", 1)
            t = self.objref_types.get(head)
            if t:
                candidates.append(f"{t}.{rest}")
        for c in candidates:
            if c in self.entries:
                return f"{c}:\n{self.entries[c]}"
        close = sorted(k for k in self.entries if k and name.split(".")[-1] in k)[:10]
        hint = f" Similar: {', '.join(close)}." if close else ""
        raise KeyError(f"No documentation entry named '{name}'.{hint} "
                       f"Use documentation(\"\") for everything.")
