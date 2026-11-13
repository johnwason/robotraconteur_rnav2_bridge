#!/usr/bin/env python3
"""Host-side smoke check: connect to the bridge and print its pulled service definition
summary (verifies docstrings reach clients). Run with .venv/bin/python -I."""
import sys
import time

from RobotRaconteur.Client import RRN

url = sys.argv[1] if len(sys.argv) > 1 else "rr+tcp://127.0.0.1:62354?service=nav2"
deadline = time.time() + 30
while True:
    try:
        c = RRN.ConnectService(url)
        break
    except Exception as e:  # service still starting
        if time.time() > deadline:
            raise
        time.sleep(1)
print("connected")
doc = c.documentation("")
print("documentation(''):", len(doc.splitlines()), "lines")
print(c.documentation("navigate_to_pose").splitlines()[1])
RRN.DisconnectService(c)
