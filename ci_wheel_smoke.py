from __future__ import annotations

import argparse
import math
import sys
import sysconfig
import tempfile
import types
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("expected_version")
    parser.add_argument(
        "--require-free-threaded",
        action="store_true",
    )
    args = parser.parse_args()

    if args.require_free_threaded:
        assert sysconfig.get_config_var("Py_GIL_DISABLED") == 1

    cv2 = types.ModuleType("cv2")
    cv2.imwrite = lambda filename, image: True

    cv2_typing = types.ModuleType("cv2.typing")
    cv2_typing.MatLike = object
    cv2.typing = cv2_typing

    sys.modules["cv2"] = cv2
    sys.modules["cv2.typing"] = cv2_typing

    msgpackrpc = types.ModuleType("msgpackrpc")
    msgpackrpc_future = types.ModuleType("msgpackrpc.future")

    class Future:
        pass

    class Address:
        def __init__(self, host, port):
            self.host = host
            self.port = port

    class Client:
        def __init__(self, address, **kwargs):
            self.address = address
            self.kwargs = kwargs

        def call(self, *args, **kwargs):
            raise AssertionError(
                "Smoke test must not make an RPC call"
            )

    msgpackrpc.Address = Address
    msgpackrpc.Client = Client
    msgpackrpc_future.Future = Future

    sys.modules["msgpackrpc"] = msgpackrpc
    sys.modules["msgpackrpc.future"] = msgpackrpc_future

    import airsim
    from airsim.client import VehicleClient
    from airsim.pfm import read_pfm, write_pfm
    from airsim.types import Quaternionr, Vector3r
    from airsim.utils import (
        to_eularian_angles,
        to_quaternion,
    )
    from airsim.version import __gitsha__, __version__

    assert __version__ == args.expected_version
    assert len(__gitsha__) == 7
    assert all(
        character in "0123456789abcdef"
        for character in __gitsha__.lower()
    )

    vector = Vector3r(1.0, 2.0, 3.0)
    assert tuple(vector) == (1.0, 2.0, 3.0)
    assert vector.to_msgpack() == {
        "x_val": 1.0,
        "y_val": 2.0,
        "z_val": 3.0,
    }

    restored = Vector3r.from_msgpack(
        {
            "x_val": 4.0,
            "y_val": 5.0,
            "z_val": 6.0,
        }
    )
    assert tuple(restored) == (4.0, 5.0, 6.0)

    quaternion = Quaternionr(
        x_val=0.0,
        y_val=0.0,
        z_val=0.0,
        w_val=1.0,
    )
    assert math.isclose(
        quaternion.get_length(),
        1.0,
    )

    q = to_quaternion(0.0, 0.0, 0.0)
    roll, pitch, yaw = to_eularian_angles(q)

    assert math.isclose(q.w_val, 1.0)
    assert math.isclose(roll, 0.0, abs_tol=1e-12)
    assert math.isclose(pitch, 0.0, abs_tol=1e-12)
    assert math.isclose(yaw, 0.0, abs_tol=1e-12)

    client = VehicleClient(
        ip="192.0.2.1",
        port=41451,
        timeout_value=17,
        reconnect_limit=3,
    )

    assert isinstance(client.client, Client)
    assert client.client.address.host == "192.0.2.1"
    assert client.client.address.port == 41451
    assert client.client.kwargs["timeout"] == 17
    assert client.client.kwargs["reconnect_limit"] == 3
    assert client.get_client_version() == 1
    assert client.get_min_required_server_version() == 1

    image = np.array(
        [
            [1.0, 2.0],
            [3.0, 4.0],
        ],
        dtype=">f4",
    )

    with tempfile.TemporaryDirectory() as tmp:
        filename = Path(tmp) / "test.pfm"
        write_pfm(str(filename), image)
        restored_image, scale = read_pfm(str(filename))

    assert scale == 1.0
    assert np.array_equal(restored_image, image)

    print("AirSim installed-wheel smoke-test OK")
    print("Installed from:", airsim.__file__)
    print("Version:", __version__)
    print("Git SHA:", __gitsha__)
    print("Python:", sys.version)
    print("Platform:", sys.platform)


if __name__ == "__main__":
    main()
