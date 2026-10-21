"""Upload a raw application image to the native HTTP Ethernet updater."""

import argparse
import getpass
import http.client
import ipaddress
import math
import os
from pathlib import Path
import re
import socket
import stat
import sys
import threading
import time


DEFAULT_PORT = 3232
DEFAULT_TIMEOUT = 330.0
SOCKET_TIMEOUT = 30.0
CHUNK_SIZE = 16384
MAX_RESPONSE_SIZE = 4096


class UploadError(Exception):
    """The device did not confirm a complete, accepted upload."""


def parse_address(value):
    """Accept DNS names, IPv4, IPv6, and IPv6 with an interface scope."""
    address = value.strip()
    if address.startswith("[") and address.endswith("]"):
        address = address[1:-1]
    if not address or any(ord(character) <= 32 or ord(character) == 127 for character in address):
        raise argparse.ArgumentTypeError("provide a hostname or IP address without whitespace")
    try:
        parsed = ipaddress.ip_address(address)
    except ValueError:
        if any(character in address for character in ":/%[]\\@?#"):
            raise argparse.ArgumentTypeError("provide a hostname or IP literal, not a URL or host:port") from None
        try:
            hostname = address.encode("idna").decode("ascii").lower()
        except UnicodeError as error:
            raise argparse.ArgumentTypeError("invalid DNS hostname") from error
        labels = hostname.rstrip(".").split(".")
        if (
            len(hostname.rstrip(".")) > 253
            or all(character in "0123456789." for character in hostname)
            or any(not re.fullmatch(r"[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?", label) for label in labels)
        ):
            raise argparse.ArgumentTypeError("invalid DNS hostname")
        return hostname
    if isinstance(parsed, ipaddress.IPv6Address) and parsed.scope_id is not None:
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", parsed.scope_id):
            raise argparse.ArgumentTypeError("invalid IPv6 interface scope")
    return str(parsed)


def parse_port(value):
    try:
        port = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("port must be an integer") from error
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return port


def parse_timeout(value):
    try:
        timeout = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("timeout must be a positive number of seconds") from error
    if not math.isfinite(timeout) or timeout <= 0:
        raise argparse.ArgumentTypeError("timeout must be finite and greater than zero")
    return timeout


def create_parser():
    parser = argparse.ArgumentParser(
        description="Upload an application .bin with native HTTP OTE. "
        "The password and firmware are unencrypted: trusted LAN/VPN only.",
        allow_abbrev=False,
    )
    parser.add_argument("-i", "--ip", "--host", dest="host", required=True, type=parse_address,
                        help="DNS hostname, IPv4, or IPv6 literal (optionally %%interface)")
    parser.add_argument("-p", "--port", type=parse_port, default=DEFAULT_PORT,
                        help="device TCP port (default: 3232)")
    parser.add_argument("-f", "--file", required=True, type=Path, metavar="FILE.bin",
                        help="application image, not a merged flash or bootloader image")
    parser.add_argument("--timeout", type=parse_timeout, default=DEFAULT_TIMEOUT,
                        help="whole-upload deadline in seconds (default: 330; I/O idle limit: 30)")
    parser.add_argument("--password-env", default="OTE_PASSWORD", metavar="NAME",
                        help="password environment variable (default: OTE_PASSWORD); otherwise prompt")
    return parser


def upload_firmware(address, port, password, firmware_path, timeout=DEFAULT_TIMEOUT, progress=None):
    """Return the device's success message; never retry an uncertain upload."""
    address = parse_address(address)
    port = parse_port(port)
    timeout = parse_timeout(timeout)
    firmware_path = Path(firmware_path)
    if firmware_path.suffix.lower() != ".bin":
        raise UploadError("firmware filename must end in .bin")
    if not password or any(ord(character) < 32 or ord(character) == 127 for character in password):
        raise UploadError("password must be nonempty and contain no control characters")
    # http.client serializes headers as Latin-1; preserve the firmware's UTF-8 bytes.
    authorization = "Bearer " + password.encode("utf-8").decode("latin-1")

    with firmware_path.open("rb") as firmware:
        metadata = os.fstat(firmware.fileno())
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_size == 0:
            raise UploadError("firmware must be a nonempty regular file")
        size = metadata.st_size
        if firmware.read(1) != b"\xe9":
            raise UploadError("not a raw ESP application image (missing image magic)")
        firmware.seek(0)
        deadline = time.monotonic() + timeout
        connection = http.client.HTTPConnection(address, port, timeout=min(SOCKET_TIMEOUT, timeout))
        transport_socket = None
        deadline_timer = None

        def set_remaining_timeout():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise UploadError("upload deadline exceeded; check the device before retrying")
            if transport_socket is not None and transport_socket.fileno() != -1:
                transport_socket.settimeout(min(SOCKET_TIMEOUT, remaining))

        def interrupt_at_deadline():
            # Also bound HTTP header parsing, where a slow peer could otherwise
            # reset the socket idle timeout repeatedly inside http.client.
            try:
                transport_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

        try:
            connection.connect()
            transport_socket = connection.sock
            set_remaining_timeout()
            deadline_timer = threading.Timer(max(0, deadline - time.monotonic()), interrupt_at_deadline)
            deadline_timer.daemon = True
            deadline_timer.start()
            connection.putrequest("POST", "/ote")
            connection.putheader("Authorization", authorization)
            connection.putheader("Content-Type", "application/octet-stream")
            connection.putheader("Content-Length", str(size))
            connection.putheader("Connection", "close")
            connection.endheaders()
            sent = 0
            send_error = None
            if progress is not None:
                progress(0, size)
            while sent < size:
                chunk = firmware.read(min(CHUNK_SIZE, size - sent))
                if not chunk:
                    raise UploadError("firmware was truncated during upload")
                set_remaining_timeout()
                try:
                    connection.send(chunk)
                except OSError as error:
                    send_error = error
                    break
                sent += len(chunk)
                if progress is not None:
                    progress(sent, size)

            set_remaining_timeout()
            try:
                with connection.getresponse() as response:
                    # HTTPConnection relinquishes its socket on Connection: close.
                    # Keep our reference to update the response stream's timeout.
                    parts = []
                    received = 0
                    while received < MAX_RESPONSE_SIZE:
                        set_remaining_timeout()
                        part = response.read1(min(1024, MAX_RESPONSE_SIZE - received))
                        if not part:
                            if response.length not in (None, 0):
                                raise UploadError("no complete final response; check the device before retrying")
                            break
                        received += len(part)
                        parts.append(part)
                    message = b"".join(parts).decode("utf-8", errors="replace").strip()
                    set_remaining_timeout()
                    if response.status != 200:
                        raise UploadError(f"HTTP {response.status} {response.reason}: {message}")
                    if sent != size or send_error is not None:
                        raise UploadError("device returned success before the complete image was sent")
                    return message
            except (OSError, http.client.HTTPException) as error:
                if send_error is not None:
                    raise UploadError("connection closed during upload; check the device before retrying") from send_error
                raise UploadError("no complete final response; check the device before retrying") from error
        finally:
            if deadline_timer is not None:
                deadline_timer.cancel()
                deadline_timer.join()
            connection.close()


def main(arguments=None):
    parser = create_parser()
    options = parser.parse_args(arguments)
    try:
        password = os.environ.get(options.password_env)
        if password is None:
            if not sys.stdin.isatty():
                raise UploadError(f"set {options.password_env} when no interactive terminal is available")
            password = getpass.getpass("Current OTE password: ")
        print("Warning: HTTP sends the password and firmware unencrypted. Trusted LAN/VPN only.",
              file=sys.stderr)
        last_percent = -1

        def progress(sent, total):
            nonlocal last_percent
            percent = sent * 100 // total
            if percent // 10 != last_percent // 10:
                print(f"Uploaded {percent}% ({sent}/{total} bytes)", flush=True)
                last_percent = percent

        message = upload_firmware(options.host, options.port, password, options.file,
                                  options.timeout, progress)
        print(f"HTTP 200: {message}")
        print("Image accepted. Wait for restart, then verify the firmware and NTP service.")
        return 0
    except (UploadError, OSError, http.client.HTTPException, ValueError) as error:
        print(f"OTE update failed: {error}", file=sys.stderr)
        print("No automatic retry. Check whether the device accepted the image or restarted before retrying.",
              file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        print("OTE update cancelled; check the device before retrying.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
