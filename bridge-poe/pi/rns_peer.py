#!/usr/bin/env python3
"""
Gate A2 — Pi-side Reticulum peer (reference Python RNS).

The production-shaped role is `server`: the Pi owns the destination the leaf pushes
images to, and receives them as RNS Resources. `client` is here only to test the
reverse direction (Pi -> board).

API mirrors microReticulum's C++ (it's a port), so this doubles as the interop oracle:
if a Resource sent by the C++ board concludes COMPLETE here, the implementations
interoperate at L3.

Grounded in markqvist/Reticulum Examples/Filetransfer.py.

Usage:
  pip install rns
  # put the Gate-A2 interface in ~/.reticulum/config (see ../pi/reticulum-config.example)
  ./rns_peer.py server                 # prints its destination hash; receives Resources
  ./rns_peer.py client <dest_hex>      # (optional) sends a 50 KB Resource to <dest_hex>
"""
import sys
import time
import hashlib
import RNS

APP_NAME = "trailcam_poe"
ASPECT   = "resource"

# ----------------------------------------------------------------------------- server
def res_started(resource):
    RNS.log(f"[poe] incoming resource, size ~{resource.size} bytes")

def res_concluded(resource):
    if resource.status == RNS.Resource.COMPLETE:
        data = resource.data.read()
        RNS.log(f"[poe] RESOURCE COMPLETE: {len(data)} bytes  "
                f"sha256={hashlib.sha256(data).hexdigest()[:16]}")
    else:
        RNS.log(f"[poe] RESOURCE FAILED status={resource.status}", RNS.LOG_ERROR)

def link_established(link):
    RNS.log("[poe] leaf/board linked")
    link.set_resource_strategy(RNS.Link.ACCEPT_ALL)
    link.set_resource_started_callback(res_started)
    link.set_resource_concluded_callback(res_concluded)

def server(configpath):
    RNS.Reticulum(configpath)
    identity = RNS.Identity()
    destination = RNS.Destination(
        identity, RNS.Destination.IN, RNS.Destination.SINGLE, APP_NAME, ASPECT)
    destination.set_link_established_callback(link_established)
    RNS.log("[poe] SERVER destination hash: " + RNS.prettyhexrep(destination.hash))
    RNS.log("[poe] ^ paste the 32 hex chars into the board's a2_config.h, reflash")
    while True:                       # announce so the board can learn a path
        destination.announce()
        RNS.log("[poe] announced")
        time.sleep(15)

# ----------------------------------------------------------------------------- client
def send_concluded(resource):
    ok = resource.status == RNS.Resource.COMPLETE
    RNS.log(f"[poe] send {'COMPLETE' if ok else 'FAILED'}",
            RNS.LOG_INFO if ok else RNS.LOG_ERROR)

def client(dest_hex, configpath):
    RNS.Reticulum(configpath)
    dest_hash = bytes.fromhex(dest_hex)
    if not RNS.Transport.has_path(dest_hash):
        RNS.Transport.request_path(dest_hash)
        RNS.log("[poe] requesting path...")
        while not RNS.Transport.has_path(dest_hash):
            time.sleep(0.1)
    server_identity = RNS.Identity.recall(dest_hash)
    server_destination = RNS.Destination(
        server_identity, RNS.Destination.OUT, RNS.Destination.SINGLE, APP_NAME, ASPECT)
    link = RNS.Link(server_destination)
    def on_up(lnk):
        RNS.log("[poe] link up, sending 50 KB Resource")
        RNS.Resource(bytes(50 * 1024), lnk, callback=send_concluded)
    link.set_link_established_callback(on_up)
    while True:
        time.sleep(1)

# ------------------------------------------------------------------------------- main
if __name__ == "__main__":
    cfg = None  # default ~/.reticulum
    if len(sys.argv) >= 2 and sys.argv[1] == "server":
        server(cfg)
    elif len(sys.argv) >= 3 and sys.argv[1] == "client":
        client(sys.argv[2], cfg)
    else:
        print(__doc__)
        sys.exit(1)
