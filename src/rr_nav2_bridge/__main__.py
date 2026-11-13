"""Entry point: python3 -m rr_nav2_bridge

Environment:
  RR_NAV2_PORT        RR TCP port (default 62354)
  RR_NAV2_ROBDEF_DIR  directory with experimental.nav2_bridge.robdef
  RR_NAV2_AUTH=1      require RR user authentication; users file at RR_NAV2_USERS_FILE
                      (default /opt/rr_nav2/secrets/users.txt, mounted read-only)
  RR_NAV2_TLS=1       load this node's TLS certificate and require TLS on every
                      connection. RR TLS needs a node certificate issued for
                      RR_NAV2_NODE_ID, mounted into the RR certificate store.
  RR_NAV2_NODE_ID     fixed NodeID (UUID); required with RR_NAV2_TLS=1
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


def security_policy():
    """ServiceSecurityPolicy requiring a valid user, or None when auth is off."""
    if os.environ.get("RR_NAV2_AUTH") != "1":
        return None
    path = os.environ.get("RR_NAV2_USERS_FILE", "/opt/rr_nav2/secrets/users.txt")
    with open(path) as f:
        text = f.read()
    if not any(line.strip() and not line.startswith("#") for line in text.splitlines()):
        raise SystemExit(f"RR_NAV2_AUTH=1 but {path} has no users")
    auth = RR.PasswordFileUserAuthenticator(text)
    return RR.ServiceSecurityPolicy(auth, {"requirevaliduser": "true",
                                           "allowobjectlock": "false"})


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
    setup_argv = ["rr_nav2_bridge"]
    overrides = 0
    node_id = os.environ.get("RR_NAV2_NODE_ID", "").strip()
    if node_id:
        setup_argv.append(f"--robotraconteur-nodeid={node_id}")
        overrides |= RR.RobotRaconteurNodeSetupFlags_NODEID_OVERRIDE
    if os.environ.get("RR_NAV2_TLS") == "1":
        if not node_id:
            log.error("RR_NAV2_TLS=1 requires RR_NAV2_NODE_ID (the NodeID of the certificate)")
            return 2
        flags |= (RR.RobotRaconteurNodeSetupFlags_LOAD_TLS_CERT
                  | RR.RobotRaconteurNodeSetupFlags_REQUIRE_TLS)
    policy = security_policy()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    with RR.RobotRaconteurNodeSetup(NODE_NAME, port, flags, overrides,
                                    argv=setup_argv) as setup:
        if (os.environ.get("RR_NAV2_TLS") == "1"
                and not setup.tcp_transport.IsTlsNodeCertificateLoaded):
            log.error("RR_NAV2_TLS=1 but no TLS certificate could be loaded for NodeID %s; "
                      "mount it as the RR certificate store (see docs/usage.md)", node_id)
            ros.shutdown()
            return 2
        ros.start()
        args = [SERVICE_NAME, "experimental.nav2_bridge.Nav2Navigator", obj]
        if policy is not None:
            args.append(policy)
        RRN.RegisterService(*args)
        log.info("RR service '%s' listening on TCP port %d (rr+tcp://<host>:%d?service=%s)"
                 " auth=%s tls=%s", SERVICE_NAME, port, port, SERVICE_NAME,
                 policy is not None, os.environ.get("RR_NAV2_TLS") == "1")
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
