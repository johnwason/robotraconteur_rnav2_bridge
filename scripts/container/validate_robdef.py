#!/usr/bin/env python3
"""Validate the bridge robdef: parse + verify against the standard robdefs, and require a
## docstring on the service, every struct/enum/object, and every member/field."""
import glob
import os
import sys

import RobotRaconteur as RR
import RobotRaconteurCompanion
from RobotRaconteur.Client import RRN

ROBDEF = sys.argv[1] if len(sys.argv) > 1 else "/opt/rr_nav2/robdef/experimental.nav2_bridge.robdef"
std_dir = os.path.join(os.path.dirname(RobotRaconteurCompanion.__file__), "StdRobDef")

texts = [open(f).read() for f in sorted(glob.glob(os.path.join(std_dir, "*.robdef")))]
texts.append(open(ROBDEF).read())
RRN.RegisterServiceTypes(texts)  # parses and verifies all imports/types

d = RR.ServiceDefinition()
d.FromString(texts[-1])
missing = []
if not d.DocString.strip():
    missing.append(d.Name)
for coll in (d.Structures, d.Objects):
    for i in range(len(coll)):
        e = coll[i]
        if not e.DocString.strip():
            missing.append(e.Name)
        for j in range(len(e.Members)):
            m = e.Members[j]
            if not m.DocString.strip():
                missing.append(f"{e.Name}.{m.Name}")
for i in range(len(d.Enums)):
    if not d.Enums[i].DocString.strip():
        missing.append(d.Enums[i].Name)
if missing:
    print("Missing docstrings:", ", ".join(missing))
    sys.exit(1)
print(f"OK {d.Name}: {len(d.Structures)} structs, {len(d.Enums)} enums, "
      f"{len(d.Objects)} objects, "
      f"{sum(len(d.Objects[i].Members) for i in range(len(d.Objects)))} object members")
