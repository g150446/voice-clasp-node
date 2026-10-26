"""BLE names of the Voice Clasp node (the wristband), old and new.

The advertised name moves from `HarnessNode` to `ClaspNode` with firmware 0.0.117. Until
every node has been updated, the tools accept both names (and `-Variant` suffixes such as
`ClaspNode-Plus2`). Matching is by name only, never by service UUID: these tools can flash
firmware, and other boards share the same audio UUIDs.
"""

from __future__ import annotations

from bleak import BleakScanner

NODE_NAME = "ClaspNode"
LEGACY_NODE_NAME = "HarnessNode"
NODE_NAMES = (NODE_NAME, LEGACY_NODE_NAME)


def is_node_name(name: str | None) -> bool:
    """True for a node name of either brand, with or without a `-Variant` suffix."""
    value = name or ""
    return any(value == base or value.startswith(f"{base}-") for base in NODE_NAMES)


def wants_any_node(name: str | None) -> bool:
    """A default / brand name means "whichever node is advertising"; anything else is exact."""
    return name is None or name in NODE_NAMES


async def find_node(name: str | None = None, timeout: float = 15.0):
    """Find a node by name.

    With no name (or either brand name) the first node advertising under either name is
    returned. Any other name, e.g. from `--device HarnessNode-Plus2`, is matched exactly.
    """
    if not wants_any_node(name):
        return await BleakScanner.find_device_by_name(name, timeout=timeout)
    return await BleakScanner.find_device_by_filter(
        lambda device, adv: is_node_name(device.name or adv.local_name),
        timeout=timeout,
    )
