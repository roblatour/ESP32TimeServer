"""Standard-library tests: python -B -m unittest discover -s testing\\ote -v."""

import argparse
from contextlib import contextmanager
import importlib.util
import io
import os
from pathlib import Path
import socket
import stat
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("ote_upload", ROOT / "tools" / "ote_upload.py")
ote = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ote)
IMAGE = b"\xe9" + bytes(range(256)) * 200


class ImageStream(io.BytesIO):
    def fileno(self):
        return -1


@contextmanager
def firmware_image(data=IMAGE, declared_size=None, mode=stat.S_IFREG):
    stream = ImageStream(data)
    metadata = SimpleNamespace(st_size=len(data) if declared_size is None else declared_size, st_mode=mode)
    with mock.patch.object(Path, "open", return_value=stream), mock.patch.object(ote.os, "fstat", return_value=metadata):
        yield


def fake_connection(status=200, reason="OK", message=b"OTE image validated\n"):
    connection = mock.MagicMock()
    connection.sock.fileno.return_value = 10
    response = mock.MagicMock()
    response.status = status
    response.reason = reason
    response.length = 0
    response.read1.side_effect = [message, b""]
    response.__enter__.return_value = response
    connection.getresponse.return_value = response
    return connection


class AddressTests(unittest.TestCase):
    def test_literal_and_dns_addresses(self):
        cases = {
            "192.0.2.10": "192.0.2.10",
            "2001:db8::1": "2001:db8::1",
            "[2001:db8::1]": "2001:db8::1",
            "fe80::1%12": "fe80::1%12",
            "[fe80::1%Ethernet]": "fe80::1%Ethernet",
            "::ffff:192.0.2.10": "::ffff:c000:20a",
            "ESP32TimeServer.local": "esp32timeserver.local",
            "device.example.": "device.example.",
            "localhost": "localhost",
            "täst.example": "xn--tst-qla.example",
        }
        for value, expected in cases.items():
            with self.subTest(value=value):
                self.assertEqual(ote.parse_address(value), expected)

    def test_invalid_addresses(self):
        for value in ("", "http://device.local", "device.local:3232", "bad name",
                      "host\r\nAuthorization:x", "-bad.local", "bad..local", "999.1.1.1",
                      "fe80::1%bad/name", "[::1", "name/path", "a" * 64 + ".local"):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                ote.parse_address(value)

    def test_ports(self):
        self.assertEqual(ote.parse_port("3232"), 3232)
        for value in ("0", "-1", "65536", "bad", "1.5"):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                ote.parse_port(value)

    def test_timeouts(self):
        self.assertEqual(ote.parse_timeout("2.5"), 2.5)
        for value in ("0", "-1", "nan", "inf", "bad"):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                ote.parse_timeout(value)

    def test_cli_options(self):
        options = ote.create_parser().parse_args(
            ["--host", "device.local", "-f", "app.bin", "-p", "4321",
             "--timeout", "45", "--password-env", "TEST_OTE_PASSWORD"]
        )
        self.assertEqual((options.host, options.port, options.timeout),
                         ("device.local", 4321, 45.0))
        self.assertEqual(options.password_env, "TEST_OTE_PASSWORD")
        options = ote.create_parser().parse_args(["-i", "[fe80::1%12]", "-f", "app.bin"])
        self.assertEqual(options.host, "fe80::1%12")


class UploadTests(unittest.TestCase):
    def upload(self, connection=None, **options):
        connection = fake_connection() if connection is None else connection
        with firmware_image(), mock.patch.object(ote.http.client, "HTTPConnection", return_value=connection):
            result = ote.upload_firmware("device.local", 3232, "test-password", "app.bin", **options)
        return result, connection

    def test_exact_protocol_and_progress(self):
        progress = mock.Mock()
        result, connection = self.upload(progress=progress)
        self.assertEqual(result, "OTE image validated")
        connection.putrequest.assert_called_once_with("POST", "/ote")
        self.assertEqual(connection.putheader.call_args_list, [
            mock.call("Authorization", "Bearer test-password"),
            mock.call("Content-Type", "application/octet-stream"),
            mock.call("Content-Length", str(len(IMAGE))),
            mock.call("Connection", "close"),
        ])
        self.assertEqual(b"".join(call.args[0] for call in connection.send.call_args_list), IMAGE)
        self.assertEqual(progress.call_args_list[0], mock.call(0, len(IMAGE)))
        self.assertEqual(progress.call_args_list[-1], mock.call(len(IMAGE), len(IMAGE)))
        connection.close.assert_called_once()

    def test_unicode_password_is_encoded_as_utf8_bytes(self):
        connection = fake_connection()
        with firmware_image(), mock.patch.object(ote.http.client, "HTTPConnection", return_value=connection):
            ote.upload_firmware("127.0.0.1", 3232, "pässword", "app.bin")
        value = dict(call.args for call in connection.putheader.call_args_list)["Authorization"]
        self.assertEqual(value.encode("latin-1"), "Bearer pässword".encode("utf-8"))

    def test_rejections_include_device_error_and_never_retry(self):
        for status, reason in ((401, "Unauthorized"), (409, "Conflict"), (413, "Content Too Large"),
                               (400, "Bad Request"), (408, "Request Timeout"), (500, "Internal Server Error")):
            connection = fake_connection(status, reason, b"explicit device error")
            with self.subTest(status=status), self.assertRaisesRegex(ote.UploadError, f"HTTP {status}.*explicit device error"):
                self.upload(connection)
            connection.connect.assert_called_once()
            connection.close.assert_called_once()

    def test_broken_pipe_still_reports_early_http_rejection(self):
        connection = fake_connection(401, "Unauthorized", b"OTE authentication required")
        connection.send.side_effect = BrokenPipeError("closed")
        with self.assertRaisesRegex(ote.UploadError, "HTTP 401.*authentication required"):
            self.upload(connection)

    def test_early_success_is_not_accepted(self):
        connection = fake_connection()
        connection.send.side_effect = BrokenPipeError("closed")
        with self.assertRaisesRegex(ote.UploadError, "before the complete image"):
            self.upload(connection)

    def test_no_final_response_is_uncertain_not_success(self):
        connection = fake_connection()
        connection.getresponse.side_effect = ote.http.client.RemoteDisconnected("closed")
        with self.assertRaisesRegex(ote.UploadError, "no complete final response"):
            self.upload(connection)
        connection.close.assert_called_once()

    def test_incomplete_response_body_is_not_accepted(self):
        connection = fake_connection()
        connection.getresponse.return_value.length = 100
        with self.assertRaisesRegex(ote.UploadError, "no complete final response"):
            self.upload(connection)

    def test_send_and_response_failure_explains_connection_closed(self):
        connection = fake_connection()
        connection.send.side_effect = BrokenPipeError("closed")
        connection.getresponse.side_effect = ote.http.client.RemoteDisconnected("closed")
        with self.assertRaisesRegex(ote.UploadError, "connection closed during upload"):
            self.upload(connection)

    def test_connect_failure_still_closes_connection(self):
        connection = fake_connection()
        connection.connect.side_effect = TimeoutError("connect timeout")
        with self.assertRaises(TimeoutError):
            self.upload(connection)
        connection.close.assert_called_once()

    def test_invalid_image_and_password_do_not_connect(self):
        for data, size, mode, expected in (
            (b"", 0, stat.S_IFREG, "nonempty regular"),
            (b"bad image", 9, stat.S_IFREG, "missing image magic"),
            (IMAGE, len(IMAGE), stat.S_IFDIR, "nonempty regular"),
        ):
            with self.subTest(expected=expected), firmware_image(data, size, mode), \
                    mock.patch.object(ote.http.client, "HTTPConnection") as factory, \
                    self.assertRaisesRegex(ote.UploadError, expected):
                ote.upload_firmware("127.0.0.1", 3232, "test-password", "app.bin")
            factory.assert_not_called()
        for password in ("", "bad\r\nHeader:x", "bad\x00", "bad\x7f"):
            with self.subTest(password=repr(password)), self.assertRaises(ote.UploadError):
                ote.upload_firmware("127.0.0.1", 3232, password, "app.bin")
        with self.assertRaisesRegex(ote.UploadError, "end in .bin"):
            ote.upload_firmware("127.0.0.1", 3232, "test-password", "app.txt")

    def test_truncated_file_is_not_silently_sent_short(self):
        connection = fake_connection()
        with firmware_image(IMAGE, len(IMAGE) + 1), \
                mock.patch.object(ote.http.client, "HTTPConnection", return_value=connection), \
                self.assertRaisesRegex(ote.UploadError, "truncated"):
            ote.upload_firmware("127.0.0.1", 3232, "test-password", "app.bin")
        connection.close.assert_called_once()

    def test_deadline_and_socket_timeout(self):
        connection = fake_connection()
        with mock.patch.object(ote.time, "monotonic", side_effect=[0, 2]), \
                self.assertRaisesRegex(ote.UploadError, "deadline exceeded"):
            self.upload(connection, timeout=1)
        connection.close.assert_called_once()
        _, connection = self.upload(timeout=5)
        for call in connection.sock.settimeout.call_args_list:
            self.assertGreater(call.args[0], 0)
            self.assertLessEqual(call.args[0], 5)

    def test_deadline_interrupts_socket_and_watchdog_is_stopped(self):
        connection = fake_connection()
        with mock.patch.object(ote.threading, "Timer") as timer_factory:
            self.upload(connection)
            callback = timer_factory.call_args.args[1]
            callback()
        connection.sock.shutdown.assert_called_once_with(socket.SHUT_RDWR)
        timer_factory.return_value.start.assert_called_once()
        timer_factory.return_value.cancel.assert_called_once()
        timer_factory.return_value.join.assert_called_once()

    def test_response_body_is_bounded_and_closed(self):
        connection = fake_connection()
        response = connection.getresponse.return_value
        response.read1.side_effect = lambda count: b"x" * count
        result, _ = self.upload(connection)
        self.assertEqual(len(result), ote.MAX_RESPONSE_SIZE)
        response.__exit__.assert_called_once()


class MainTests(unittest.TestCase):
    def test_environment_password(self):
        with mock.patch.dict(os.environ, {"TEST_OTE_PASSWORD": "from-environment"}), \
                mock.patch.object(ote, "upload_firmware", return_value="accepted") as upload, \
                mock.patch.object(ote.getpass, "getpass") as prompt, \
                mock.patch("sys.stdout", new_callable=io.StringIO), \
                mock.patch("sys.stderr", new_callable=io.StringIO):
            result = ote.main(["-i", "device.local", "-f", "app.bin", "--password-env", "TEST_OTE_PASSWORD"])
        self.assertEqual(result, 0)
        self.assertEqual(upload.call_args.args[2], "from-environment")
        prompt.assert_not_called()

    def test_prompt_when_environment_absent(self):
        with mock.patch.dict(os.environ, {}, clear=True), \
                mock.patch.object(ote.sys.stdin, "isatty", return_value=True), \
                mock.patch.object(ote.getpass, "getpass", return_value="from-prompt") as prompt, \
                mock.patch.object(ote, "upload_firmware", return_value="accepted") as upload, \
                mock.patch("sys.stdout", new_callable=io.StringIO), \
                mock.patch("sys.stderr", new_callable=io.StringIO):
            result = ote.main(["-i", "device.local", "-f", "app.bin"])
        self.assertEqual(result, 0)
        self.assertEqual(upload.call_args.args[2], "from-prompt")
        prompt.assert_called_once()

    def test_noninteractive_missing_password_fails_without_prompt(self):
        with mock.patch.dict(os.environ, {}, clear=True), \
                mock.patch.object(ote.sys.stdin, "isatty", return_value=False), \
                mock.patch.object(ote.getpass, "getpass") as prompt, \
                mock.patch("sys.stderr", new_callable=io.StringIO) as output:
            result = ote.main(["-i", "device.local", "-f", "app.bin"])
        self.assertEqual(result, 1)
        self.assertIn("set OTE_PASSWORD", output.getvalue())
        prompt.assert_not_called()

    def test_failure_does_not_print_password(self):
        with mock.patch.dict(os.environ, {"OTE_PASSWORD": "private-test-value"}), \
                mock.patch.object(ote, "upload_firmware", side_effect=ote.UploadError("HTTP 401 Unauthorized")), \
                mock.patch("sys.stderr", new_callable=io.StringIO) as output:
            result = ote.main(["-i", "device.local", "-f", "app.bin"])
        self.assertEqual(result, 1)
        self.assertIn("No automatic retry", output.getvalue())
        self.assertNotIn("private-test-value", output.getvalue())


class LoopbackTests(unittest.TestCase):
    def check_upload(self, host, family=socket.AF_INET, status=200):
        received = {}

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def do_POST(self):
                received["path"] = self.path
                received["headers"] = self.headers
                received["body"] = self.rfile.read(int(self.headers["Content-Length"]))
                body = b"accepted\n" if status == 200 else b"wrong project\n"
                self.send_response(status)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Connection", "close")
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *_arguments):
                pass

        class Server(HTTPServer):
            address_family = family

        try:
            server = Server(("::1" if family == socket.AF_INET6 else "127.0.0.1", 0), Handler)
        except OSError:
            if family == socket.AF_INET6:
                self.skipTest("IPv6 loopback is unavailable")
            raise
        server.timeout = 3
        thread = threading.Thread(target=server.handle_request, daemon=True)
        thread.start()
        self.assertTrue(thread.is_alive())
        try:
            with firmware_image():
                if status == 200:
                    result = ote.upload_firmware(host, server.server_port, "test-password", "app.bin", timeout=5)
                    self.assertEqual(result, "accepted")
                else:
                    with self.assertRaisesRegex(ote.UploadError, "HTTP 400.*wrong project"):
                        ote.upload_firmware(host, server.server_port, "test-password", "app.bin", timeout=5)
        finally:
            thread.join(timeout=5)
            server.server_close()
        self.assertFalse(thread.is_alive())
        self.assertEqual(received["path"], "/ote")
        self.assertEqual(received["body"], IMAGE)
        self.assertEqual(received["headers"]["Authorization"], "Bearer test-password")
        self.assertEqual(received["headers"]["Content-Type"], "application/octet-stream")
        self.assertEqual(received["headers"]["Content-Length"], str(len(IMAGE)))
        self.assertIsNone(received["headers"]["Transfer-Encoding"])

    def test_ipv4_upload_over_real_http(self):
        self.check_upload("127.0.0.1")

    def test_dns_hostname_upload_over_real_http(self):
        self.check_upload("localhost")

    def test_ipv6_upload_over_real_http(self):
        self.check_upload("::1", socket.AF_INET6)

    def test_http_failure_over_real_http(self):
        self.check_upload("127.0.0.1", status=400)

    def test_watchdog_bounds_slow_response_headers(self):
        stop = threading.Event()

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                self.rfile.read(int(self.headers["Content-Length"]))
                try:
                    self.wfile.write(b"HTTP/1.1 200 OK\r\nX-Slow: ")
                    for _ in range(50):
                        if stop.wait(0.02):
                            break
                        self.wfile.write(b"x")
                        self.wfile.flush()
                except OSError:
                    pass

            def log_message(self, *_arguments):
                pass

        server = HTTPServer(("127.0.0.1", 0), Handler)
        server.timeout = 2
        thread = threading.Thread(target=server.handle_request, daemon=True)
        thread.start()
        self.assertTrue(thread.is_alive())
        started = time.monotonic()
        try:
            with firmware_image(), self.assertRaises(ote.UploadError):
                ote.upload_firmware("127.0.0.1", server.server_port, "test-password",
                                    "app.bin", timeout=0.2)
            self.assertLess(time.monotonic() - started, 0.8)
        finally:
            stop.set()
            thread.join(timeout=3)
            server.server_close()
        self.assertFalse(thread.is_alive())


if __name__ == "__main__":
    unittest.main()
