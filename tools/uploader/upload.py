import argparse
import binascii
import hashlib
import os
import select
import struct
import time
import zlib


class PacketType:
    FW_HEADER = 0x20
    FW_DATA = 0x21
    ERASE_FLASH = 0x22
    FLASH_ID = 0x23
    ACK = 0x80
    CHIP_SN = 0x81
    DEVICE_SN = 0x82
    FLASH_ID_RESULT = 0xA3


class _PosixSerial:
    def __init__(self, path, baudrate=115200):
        import fcntl
        import termios

        self._fcntl = fcntl
        self._termios = termios
        self.fd = None
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            settings = termios.tcgetattr(fd)
            settings[0] = 0
            settings[1] = 0
            settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            settings[3] = 0
            speed = {115200: termios.B115200}.get(baudrate)
            if speed is None:
                raise ValueError("unsupported native serial baud rate: {}".format(baudrate))
            settings[4] = speed
            settings[5] = speed
            settings[6][termios.VMIN] = 0
            settings[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, settings)
            self.fd = fd
        except Exception:
            os.close(fd)
            raise

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def set_rts(self, enabled):
        try:
            command = self._termios.TIOCMBIS if enabled else self._termios.TIOCMBIC
            self._fcntl.ioctl(self.fd, command, struct.pack("I", self._termios.TIOCM_RTS))
        except (AttributeError, OSError):
            status = bytearray(struct.pack("I", 0))
            self._fcntl.ioctl(self.fd, self._termios.TIOCMGET, status, True)
            bits = struct.unpack("I", status)[0]
            if enabled:
                bits |= self._termios.TIOCM_RTS
            else:
                bits &= ~self._termios.TIOCM_RTS
            self._fcntl.ioctl(
                self.fd, self._termios.TIOCMSET, struct.pack("I", bits)
            )

    def write_all(self, data):
        view = memoryview(data)
        while view:
            _, writable, _ = select.select([], [self.fd], [], 1.0)
            if not writable:
                raise TimeoutError("serial write timed out")
            written = os.write(self.fd, view)
            view = view[written:]
        self._termios.tcdrain(self.fd)

    def read_some(self, timeout):
        readable, _, _ = select.select([self.fd], [], [], max(0, timeout))
        if not readable:
            return b""
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""

    def read_all(self):
        while self.read_some(0):
            pass


class _WindowsSerial:
    _MAXDWORD = 0xFFFFFFFF
    _GENERIC_READ = 0x80000000
    _GENERIC_WRITE = 0x40000000
    _OPEN_EXISTING = 3
    _FILE_ATTRIBUTE_NORMAL = 0x80
    _SETRTS = 3
    _CLRRTS = 4

    def __init__(self, path, baudrate=115200):
        import ctypes
        from ctypes import wintypes

        class DCB(ctypes.Structure):
            _fields_ = [
                ("DCBlength", wintypes.DWORD),
                ("BaudRate", wintypes.DWORD),
                ("fFlags", wintypes.DWORD),
                ("wReserved", wintypes.WORD),
                ("XonLim", wintypes.WORD),
                ("XoffLim", wintypes.WORD),
                ("ByteSize", wintypes.BYTE),
                ("Parity", wintypes.BYTE),
                ("StopBits", wintypes.BYTE),
                ("XonChar", ctypes.c_char),
                ("XoffChar", ctypes.c_char),
                ("ErrorChar", ctypes.c_char),
                ("EofChar", ctypes.c_char),
                ("EvtChar", ctypes.c_char),
                ("wReserved1", wintypes.WORD),
            ]

        class COMMTIMEOUTS(ctypes.Structure):
            _fields_ = [
                ("ReadIntervalTimeout", wintypes.DWORD),
                ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
                ("ReadTotalTimeoutConstant", wintypes.DWORD),
                ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
                ("WriteTotalTimeoutConstant", wintypes.DWORD),
            ]

        class COMSTAT(ctypes.Structure):
            _fields_ = [
                ("Flags", wintypes.DWORD),
                ("cbInQue", wintypes.DWORD),
                ("cbOutQue", wintypes.DWORD),
                ("dwFlowStat", wintypes.DWORD),
            ]

        self._ctypes = ctypes
        self._wintypes = wintypes
        self._dcb_type = DCB
        self._timeouts_type = COMMTIMEOUTS
        self._comstat_type = COMSTAT
        self._kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self._configure_api()

        port_name = path if path.startswith("\\\\.\\") else "\\\\.\\" + path
        self.handle = self._kernel32.CreateFileW(
            port_name,
            self._GENERIC_READ | self._GENERIC_WRITE,
            0,
            None,
            self._OPEN_EXISTING,
            self._FILE_ATTRIBUTE_NORMAL,
            None,
        )
        if self.handle == ctypes.c_void_p(-1).value:
            raise ctypes.WinError(ctypes.get_last_error())

        try:
            dcb = DCB()
            dcb.DCBlength = ctypes.sizeof(DCB)
            self._check(self._kernel32.GetCommState(self.handle, ctypes.byref(dcb)))
            dcb.BaudRate = baudrate
            dcb.ByteSize = 8
            dcb.Parity = 0
            dcb.StopBits = 0
            flow_mask = (1 << 1) | (1 << 2) | (1 << 3) | (3 << 4)
            flow_mask |= (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)
            flow_mask |= (1 << 10) | (1 << 11) | (3 << 12) | (1 << 14)
            dcb.fFlags = (dcb.fFlags & ~flow_mask) | 1 | (1 << 12)
            self._check(self._kernel32.SetCommState(self.handle, ctypes.byref(dcb)))

            timeouts = COMMTIMEOUTS(
                self._MAXDWORD, 0, 0, 0, 5000
            )
            self._check(
                self._kernel32.SetCommTimeouts(
                    self.handle, ctypes.byref(timeouts)
                )
            )
        except Exception:
            self.close()
            raise

    def _configure_api(self):
        ctypes = self._ctypes
        wintypes = self._wintypes
        kernel32 = self._kernel32
        kernel32.CreateFileW.argtypes = [
            wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, wintypes.LPVOID,
            wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE,
        ]
        kernel32.CreateFileW.restype = wintypes.HANDLE
        kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        kernel32.CloseHandle.restype = wintypes.BOOL
        kernel32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(self._dcb_type)]
        kernel32.GetCommState.restype = wintypes.BOOL
        kernel32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(self._dcb_type)]
        kernel32.SetCommState.restype = wintypes.BOOL
        kernel32.SetCommTimeouts.argtypes = [
            wintypes.HANDLE, ctypes.POINTER(self._timeouts_type)
        ]
        kernel32.SetCommTimeouts.restype = wintypes.BOOL
        kernel32.EscapeCommFunction.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        kernel32.EscapeCommFunction.restype = wintypes.BOOL
        kernel32.ClearCommError.argtypes = [
            wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD),
            ctypes.POINTER(self._comstat_type),
        ]
        kernel32.ClearCommError.restype = wintypes.BOOL
        kernel32.ReadFile.argtypes = [
            wintypes.HANDLE, wintypes.LPVOID, wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID,
        ]
        kernel32.ReadFile.restype = wintypes.BOOL
        kernel32.WriteFile.argtypes = kernel32.ReadFile.argtypes
        kernel32.WriteFile.restype = wintypes.BOOL
        kernel32.FlushFileBuffers.argtypes = [wintypes.HANDLE]
        kernel32.FlushFileBuffers.restype = wintypes.BOOL

    def _check(self, result):
        if not result:
            raise self._ctypes.WinError(self._ctypes.get_last_error())

    def close(self):
        if getattr(self, "handle", None) is not None:
            self._kernel32.CloseHandle(self.handle)
            self.handle = None

    def set_rts(self, enabled):
        command = self._SETRTS if enabled else self._CLRRTS
        self._check(self._kernel32.EscapeCommFunction(self.handle, command))

    def write_all(self, data):
        view = memoryview(data)
        while view:
            chunk = bytes(view[:4096])
            buffer = self._ctypes.create_string_buffer(chunk)
            written = self._wintypes.DWORD()
            self._check(self._kernel32.WriteFile(
                self.handle, buffer, len(chunk), self._ctypes.byref(written), None
            ))
            if written.value == 0:
                raise TimeoutError("serial write made no progress")
            view = view[written.value:]
        self._check(self._kernel32.FlushFileBuffers(self.handle))

    def _queued_bytes(self):
        errors = self._wintypes.DWORD()
        status = self._comstat_type()
        self._check(self._kernel32.ClearCommError(
            self.handle, self._ctypes.byref(errors), self._ctypes.byref(status)
        ))
        return status.cbInQue

    def read_some(self, timeout):
        deadline = time.monotonic() + max(0, timeout)
        while True:
            available = self._queued_bytes()
            if available:
                size = min(available, 4096)
                buffer = self._ctypes.create_string_buffer(size)
                received = self._wintypes.DWORD()
                self._check(self._kernel32.ReadFile(
                    self.handle, buffer, size,
                    self._ctypes.byref(received), None
                ))
                return buffer.raw[:received.value]
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return b""
            time.sleep(min(0.002, remaining))

    def read_all(self):
        while self.read_some(0):
            pass


class NativeSerial:
    def __init__(self, path, baudrate=115200):
        if os.name == "nt":
            self._backend = _WindowsSerial(path, baudrate)
        elif os.name == "posix":
            self._backend = _PosixSerial(path, baudrate)
        else:
            raise RuntimeError("the native uploader requires a POSIX or Windows host")

    def close(self):
        self._backend.close()

    def set_rts(self, enabled):
        self._backend.set_rts(enabled)

    def write_all(self, data):
        self._backend.write_all(data)

    def read_some(self, timeout):
        return self._backend.read_some(timeout)

    def read_all(self):
        self._backend.read_all()


class Uploader:
    def __init__(self, port, offset):
        self.port = NativeSerial(port)
        self.offset = offset
        self.receive_buffer = bytearray()

    def close(self):
        self.port.close()

    @staticmethod
    def make_packet(command, payload):
        packet = struct.pack("<BBH", 0x02, command, len(payload)) + payload
        return packet + struct.pack("<H", binascii.crc_hqx(packet, 0xFFFF))

    def receive_packet(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        buffer = self.receive_buffer

        while time.monotonic() < deadline:
            if len(buffer) < 4:
                buffer.extend(self.port.read_some(deadline - time.monotonic()))
                continue

            marker = buffer.find(b"\x02")
            if marker < 0:
                buffer.clear()
                continue
            if marker:
                del buffer[:marker]
            if len(buffer) < 4:
                continue

            payload_length = struct.unpack_from("<H", buffer, 2)[0]
            if payload_length > 256:
                raise ValueError("received packet payload over 256 bytes")
            packet_length = payload_length + 6
            while len(buffer) < packet_length and time.monotonic() < deadline:
                buffer.extend(self.port.read_some(deadline - time.monotonic()))
            if len(buffer) < packet_length:
                return False, b""

            packet = bytes(buffer[:packet_length])
            del buffer[:packet_length]
            expected_crc = struct.unpack_from("<H", packet, packet_length - 2)[0]
            actual_crc = binascii.crc_hqx(packet[:-2], 0xFFFF)
            if expected_crc == actual_crc:
                return packet[1], packet[4:-2]
            print("Invalid packet CRC; expected {:04X}, got {:04X}".format(
                actual_crc, expected_crc
            ))

        return False, b""

    def reset_device(self):
        self.port.set_rts(True)
        time.sleep(0.1)
        self.port.set_rts(False)
        time.sleep(0.1)
        self.port.set_rts(True)

    def start_boot(self):
        print(">>> Starting bootloader")
        self.port.set_rts(True)
        self.port.read_all()
        time.sleep(0.1)
        self.port.set_rts(False)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            self.port.write_all(b"\xF8" * 64)
            command, data = self.receive_packet(0.1)
            if command == PacketType.CHIP_SN:
                print(">>> Received chip identification ({})".format(data.hex()))
                return
        raise TimeoutError("timed out waiting for bootloader")

    def boot_stage2(self):
        print(">>> Initializing stage2")
        self.port.write_all(b"\x7C" * 16)
        command, _ = self.receive_packet()
        if command != PacketType.DEVICE_SN:
            raise ValueError("stage2 handshake failed; received {!r}".format(command))

    def get_flash_id(self):
        self.port.write_all(self.make_packet(PacketType.FLASH_ID, b""))
        command, data = self.receive_packet()
        if command != PacketType.FLASH_ID_RESULT or len(data) < 3:
            raise ValueError("failed to read flash ID: {}".format(data.hex()))
        flash_id = int.from_bytes(data[:3], "little")
        manufacturer = (flash_id >> 16) & 0xFF
        flash_size = flash_id & 0xFF
        flash_size = 1 << flash_size if 0x13 <= flash_size < 0x20 else flash_size
        flash_type = (flash_id >> 8) & 0xFF
        return flash_id, manufacturer, flash_size, flash_type

    def erase_flash(self, sector_count):
        payload = struct.pack("<III", 0, sector_count, 4096)
        packet = self.make_packet(PacketType.ERASE_FLASH, payload)
        timeout = max(5, sector_count * 0.5)
        while True:
            self.port.write_all(packet)
            command, _ = self.receive_packet(timeout)
            if command == PacketType.ACK:
                break
            if command is not False:
                raise ValueError("error erasing flash")
        time.sleep(0.5)

    def write_chunk(self, address, data):
        print(">>> Writing @0x{:08X}".format(address))
        packet = self.make_packet(PacketType.FW_DATA, struct.pack("<I", address) + data)
        for retry in range(1, 6):
            self.port.read_all()
            self.port.write_all(packet)
            command, response = self.receive_packet(2)
            if command == PacketType.ACK and (not response or response[0] != ord(")")):
                time.sleep(0.05)
                return
            print(">>> Error writing chunk, retry {}/5".format(retry))
            time.sleep(0.5 * retry)
        raise ValueError("error writing chunk after 5 retries")

    def upload(self, firmware):
        if len(firmware) % 4096:
            firmware += bytes(4096 - len(firmware) % 4096)
            print(">>> Padding firmware to {} bytes".format(len(firmware)))

        digest = hashlib.sha256(firmware).digest().ljust(64, b"\x00")
        header = struct.pack("<6I", 0x5555AAAA, 0, self.offset,
                             len(firmware), 0, 2) + digest
        header += struct.pack("<I", zlib.crc32(header[4:]) & 0xFFFFFFFF)

        self.reset_device()
        self.start_boot()
        self.boot_stage2()
        flash_id, manufacturer, flash_size, flash_type = self.get_flash_id()
        print(">>> Flash ID: {:06X} ({:02X})".format(flash_id, manufacturer))
        print(">>> Flash Size: {} bytes".format(flash_size))
        print(">>> Flash Type: {:02X}".format(flash_type))

        self.port.write_all(self.make_packet(PacketType.FW_HEADER, header))
        command, response = self.receive_packet()
        if command != PacketType.ACK:
            raise ValueError("error sending firmware header: {}".format(response.hex()))
        if response and response[0] == ord(")"):
            raise ValueError("bootloader rejected firmware header: {}".format(response))

        sectors = (len(firmware) >> 12) + 2
        print(">>> Erasing {} flash sectors".format(sectors))
        self.erase_flash(sectors)
        print(">>> Sending firmware")
        for start in range(0, len(firmware), 4096):
            self.write_chunk(self.offset + start, firmware[start:start + 4096])

        print(">>> Upload finished; resetting device")
        self.reset_device()
        self.port.set_rts(False)
        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(
        description="Upload unsigned AIR105 firmware through the native Linux serial API"
    )
    parser.add_argument("port")
    parser.add_argument("filename")
    parser.add_argument("offset", nargs="?", default="0x1001000")
    parser.add_argument("rsa_private_key", nargs="?")
    args = parser.parse_args()

    if args.rsa_private_key:
        parser.error("RSA-signed secure-boot uploads are not supported by the native uploader")

    with open(args.filename, "rb") as firmware_file:
        firmware = bytearray(firmware_file.read())
    offset = int(args.offset, 16)
    print(">>> Firmware: {} bytes".format(len(firmware)))

    uploader = Uploader(args.port, offset)
    try:
        uploader.upload(firmware)
    finally:
        uploader.close()


if __name__ == "__main__":
    main()