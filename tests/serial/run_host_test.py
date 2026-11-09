import shutil
import subprocess
import sys
import time
from pathlib import Path

repo_root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(repo_root))

from tools.uploader.upload import NativeSerial


TRANSFER_SIZE = 8192


def test_byte(index):
    return (index * 37 + (index >> 3)) & 0xFF


def read_exact(serial, size, timeout=15):
    data = bytearray()
    deadline = time.monotonic() + timeout
    while len(data) < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                "timed out receiving serial test data: got {} of {} bytes".format(
                    len(data), size
                )
            )
        data.extend(serial.read_some(remaining))
    return bytes(data)


def wait_for_ready(serial, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = serial.read_some(min(1, deadline - time.monotonic()))
        if b"R" in value:
            return
    raise TimeoutError("timed out waiting for the board's ready byte")


def main():
    flash_options = ("--flash", "--flash-custom")
    if len(sys.argv) not in (2, 3) or (len(sys.argv) == 3 and sys.argv[2] not in flash_options):
        raise SystemExit(
            "usage: run_host_test.py SERIAL_PORT [--flash | --flash-custom]"
        )

    expected = bytes(test_byte(i) for i in range(TRANSFER_SIZE))
    serial = NativeSerial(sys.argv[1])
    try:
        if len(sys.argv) == 3:
            pio = shutil.which("pio")
            if pio is None:
                raise RuntimeError("PlatformIO CLI 'pio' was not found on PATH")
            environment = (
                "air105_core_custom_buffers"
                if sys.argv[2] == "--flash-custom"
                else "air105_core"
            )
            subprocess.run([
                pio,
                "run",
                "-d",
                str(repo_root / "tests" / "serial" / "air105"),
                "-e",
                environment,
                "-t",
                "upload",
                "--upload-port",
                sys.argv[1],
            ], check=True)

        wait_for_ready(serial)

        serial.write_all(b"T")
        transmitted = read_exact(serial, TRANSFER_SIZE + 1)
        if transmitted[:-1] != expected or transmitted[-1:] != b"t":
            raise AssertionError("board TX data did not match the test pattern")

        serial.write_all(b"E" + expected)
        echoed = read_exact(serial, TRANSFER_SIZE + 5)
        if echoed[:TRANSFER_SIZE] != expected:
            raise AssertionError("board RX echo did not match the test pattern")
        if echoed[TRANSFER_SIZE:TRANSFER_SIZE + 1] != b"e":
            raise AssertionError("board detected an RX data mismatch")

        overflow_count = int.from_bytes(echoed[-4:], "little")
        if overflow_count != 0:
            raise AssertionError("board dropped {} RX bytes".format(overflow_count))

        print("8192-byte TX and RX checks passed; RX overflow count is zero")
    finally:
        serial.close()


if __name__ == "__main__":
    main()