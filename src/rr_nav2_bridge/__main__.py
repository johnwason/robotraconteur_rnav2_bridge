"""Entry point: python3 -m rr_nav2_bridge

Environment:
  RR_NAV2_PORT        RR TCP port (default 62354)
  RR_NAV2_ROBDEF_DIR  directory with experimental.nav2_bridge.robdef
"""
from __future__ import annotations

import glob
import logging
import os
import signal
import sys
import threading

import RobotRaconteur as RR
import RobotRaconteurCompanion
from RobotRaconteur.Client import RRN

from .convert import Types
from .ros_side import RosSide
from .service import Nav2Navigator

SERVICE_NAME = "nav2"
NODE_NAME = "experimental.nav2_bridge"
ROBDEF_DIR = os.environ.get("RR_NAV2_ROBDEF_DIR", "/opt/rr_nav2/robdef")

log = logging.getLogger("rr_nav2_bridge")


def register_types():
    std = os.path.join(os.path.dirname(RobotRaconteurCompanion.__file__), "StdRobDef")
    texts = [open(f).read() for f in sorted(glob.glob(os.path.join(std, "*.robdef")))]
    # RR does not transmit ## comments to clients; the text is also served by
    # Nav2Navigator.documentation().
    with open(os.path.join(ROBDEF_DIR, "experimental.nav2_bridge.robdef")) as f:
        ours = f.read()
    texts.append(ours)
    RRN.RegisterServiceTypes(texts)
    return ours


def main(argv=None):
    logging.basicConfig(level=os.environ.get("RR_NAV2_LOG_LEVEL", "INFO"),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    port = int(os.environ.get("RR_NAV2_PORT", "62354"))

    robdef_text = register_types()
    types = Types(RRN)
    ros = RosSide()
    obj = Nav2Navigator(ros, types, robdef_text)

    # TCP only. No local/intra transports, and no multicast announce (it would not cross
    # the published-port boundary anyway; clients connect by URL).
    flags = (RR.RobotRaconteurNodeSetupFlags_TCP_TRANSPORT_START_SERVER
             | RR.RobotRaconteurNodeSetupFlags_ENABLE_TCP_TRANSPORT
             | RR.RobotRaconteurNodeSetupFlags_TCP_PORT_OVERRIDE
             | RR.RobotRaconteurNodeSetupFlags_NODENAME_OVERRIDE)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    with RR.RobotRaconteurNodeSetup(NODE_NAME, port, flags):
        ros.start()
        ctx = RRN.RegisterService(SERVICE_NAME, "experimental.nav2_bridge.Nav2Navigator", obj)
        log.info("RR service '%s' listening on TCP port %d (rr+tcp://<host>:%d?service=%s)",
                 SERVICE_NAME, port, port, SERVICE_NAME)
        try:
            stop.wait()
        finally:
            log.info("shutting down")
            shutdown = getattr(obj, "shutdown", None)
            if shutdown:
                shutdown()
            ros.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
