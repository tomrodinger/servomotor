"""How a bus worker obtains its :class:`Transport`.

In production each bus opens its assigned serial port with a
:class:`SerialTransport`.  Tests (and the no-hardware demo) install a factory
that hands out :class:`SimulatedTransport` objects from a shared
:class:`RackSimulator`.  The worker calls the factory at the start of each job
and closes the transport at the end.
"""

from __future__ import annotations

from typing import Callable, Optional

from .transport import Transport, SerialTransport

# A factory takes (bus_id, port) and returns an open Transport.
TransportFactory = Callable[[str, Optional[str]], Transport]


def serial_factory(bus_id: str, port: Optional[str]) -> Transport:
    if not port:
        raise RuntimeError("No serial port assigned to bus %s" % bus_id)
    return SerialTransport(port)
