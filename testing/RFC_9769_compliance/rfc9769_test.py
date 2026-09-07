#!/usr/bin/env python3

#
# Copyright Rob Latour, 2026
# License: MIT
# Website: https://github.com/roblatour/ESP32TimeServer
#
# ==============================================================================
# Checks for RFC 9769 compliance
#
# Note: This script does not ensure the NTP server is providing accurate results
#       rather that the response it is providing is populated in a RFC 
#       compliant format.
# ==============================================================================
#
# CLI options (--target, --timeout, --retries, --verbose)

import time
import struct
import socket
import argparse
import sys

# ==============================================================================
# CONFIGURATION (CLI overrides these defaults)
# ==============================================================================

DEFAULT_TARGET_IP = "192.168.1.24"
DEFAULT_PORT = 123
DEFAULT_TIMEOUT = 3.0
DEFAULT_RETRIES = 3
NTP_ERA_OFFSET = 2208988800

# ==============================================================================
# UTILITIES
# ==============================================================================

def get_current_ntp_timestamp():
    """Generates a standard 64-bit NTP timestamp (32-bit seconds, 32-bit fraction)."""
    now = time.time() + NTP_ERA_OFFSET
    seconds = int(now)
    fraction = int(round((now - seconds) * (2**32))) & 0xFFFFFFFF
    return struct.pack("!II", seconds & 0xFFFFFFFF, fraction)

def parse_ntp_timestamp(data):
    """Parses an 8-byte NTP timestamp block into a human-readable float.
    Returns None if the timestamp is all zeros."""
    if len(data) != 8:
        raise ValueError("NTP timestamp block must be 8 bytes")
    seconds, fraction = struct.unpack("!II", data)
    if seconds == 0 and fraction == 0:
        return None
    return float(seconds) + float(fraction) / (2**32)

def pack_ntp_header(li, version, mode, stratum, poll, precision, root_delay, root_dispersion, ref_id):
    """Pack the 12-byte fixed header fields explicitly and return bytes."""
    # li: 2 bits, version: 3 bits, mode: 3 bits -> single byte
    li_vn_mode = ((li & 0x3) << 6) | ((version & 0x7) << 3) | (mode & 0x7)
    # pack: 1 byte LI/VN/Mode, 1 byte stratum, 1 byte poll, 1 byte precision
    # root_delay: 32-bit signed fixed-point (16.16) -> use signed int
    # root_dispersion: 32-bit unsigned fixed-point (16.16) -> use unsigned int
    # ref_id: 32-bit unsigned
    header = struct.pack("!BBBB", li_vn_mode, stratum & 0xFF, poll & 0xFF, precision & 0xFF)
    header += struct.pack("!iI", int(root_delay) & 0xFFFFFFFF, int(root_dispersion) & 0xFFFFFFFF)
    header += struct.pack("!I", int(ref_id) & 0xFFFFFFFF)
    return header

def build_ntp_payload(version=4, mode=3, origin_ts=b'\x00'*8, receive_ts=b'\x00'*8, transmit_ts=None):
    """Constructs a fixed 48-byte NTP payload block matching RFC 5905 specifications."""
    li = 0
    stratum = 0
    poll = 4
    precision = -20 & 0xFF  # pack as unsigned byte but interpret as signed
    root_delay = 0
    root_dispersion = 0
    ref_id = 0
    ref_ts = b'\x00' * 8

    if transmit_ts is None:
        transmit_ts = get_current_ntp_timestamp()

    header = pack_ntp_header(li, version, mode, stratum, poll, precision, root_delay, root_dispersion, ref_id)
    payload = header + ref_ts + origin_ts + receive_ts + transmit_ts
    # Ensure payload is at least 48 bytes
    if len(payload) < 48:
        payload = payload.ljust(48, b'\x00')
    return payload

# ==============================================================================
# NETWORK HELPERS
# ==============================================================================

def send_recv_with_retries(sock, payload, addr, timeout, retries, verbose=False):
    """Send payload and wait for a response with retries. Returns (raw_data, addr, t4_bytes, t4_float)."""
    last_exc = None
    for attempt in range(1, retries + 1):
        try:
            if verbose:
                print(f" |-- Attempt {attempt}/{retries}: sending packet to {addr}")
            sock.sendto(payload, addr)
            sock.settimeout(timeout)
            raw_data, remote = sock.recvfrom(2048)
            # Capture local arrival time immediately and pack as NTP bytes
            t4_local = time.time() + NTP_ERA_OFFSET
            t4_local_bytes = struct.pack("!II", int(t4_local) & 0xFFFFFFFF, int(round((t4_local - int(t4_local)) * (2**32))) & 0xFFFFFFFF)
            return raw_data, remote, t4_local_bytes, t4_local
        except socket.timeout as e:
            last_exc = e
            if verbose:
                print(f" |-- Timeout on attempt {attempt}")
            time.sleep(0.1 * attempt)
        except Exception as e:
            last_exc = e
            if verbose:
                print(f" |-- Exception on attempt {attempt}: {e}")
            time.sleep(0.1)
    raise last_exc

def parse_response_metadata(raw_data):
    """Return a dict with parsed header metadata and timestamps (raw bytes and parsed floats/None)."""
    if len(raw_data) < 48:
        raise ValueError("Truncated NTP response packet payload.")
    li_vn_mode = raw_data[0]
    leap = (li_vn_mode >> 6) & 0x3
    version = (li_vn_mode >> 3) & 0x7
    mode = li_vn_mode & 0x7
    stratum = raw_data[1]
    poll = raw_data[2]
    precision = struct.unpack("!b", raw_data[3:4])[0]
    root_delay = struct.unpack("!i", raw_data[4:8])[0]
    root_dispersion = struct.unpack("!I", raw_data[8:12])[0]
    ref_id = struct.unpack("!I", raw_data[12:16])[0]
    ref_ts_raw = raw_data[16:24]
    origin_raw = raw_data[24:32]
    receive_raw = raw_data[32:40]
    transmit_raw = raw_data[40:48]

    ref_ts = parse_ntp_timestamp(ref_ts_raw)
    origin = parse_ntp_timestamp(origin_raw)
    receive = parse_ntp_timestamp(receive_raw)
    transmit = parse_ntp_timestamp(transmit_raw)

    return {
        "leap": leap,
        "version": version,
        "mode": mode,
        "stratum": stratum,
        "poll": poll,
        "precision": precision,
        "root_delay": root_delay,
        "root_dispersion": root_dispersion,
        "ref_id": ref_id,
        "ref_ts_raw": ref_ts_raw,
        "ref_ts": ref_ts,
        "origin_raw": origin_raw,
        "origin": origin,
        "receive_raw": receive_raw,
        "receive": receive,
        "transmit_raw": transmit_raw,
        "transmit": transmit
    }

# ==============================================================================
# TEST RUNNERS (FIXED RFC 9769 HANDSHAKE)
# ==============================================================================

def run_standard_test(sock, target, version, timeout, retries, verbose=False):
    """Tests standard NTP behaviour where Receive/Origin are clear."""
    print(f"\n[+] Running Standard NTPv{version} Validation Test...")

    t1_bytes = get_current_ntp_timestamp()
    # Standard Request: Origin=0, Receive=0, Transmit=T1
    ntp_payload = build_ntp_payload(version=version, mode=3, origin_ts=b'\x00'*8, receive_ts=b'\x00'*8, transmit_ts=t1_bytes)

    if verbose:
        print(" |-- Sending Standard Request packet...")
    try:
        raw_data, _remote, t4_local_bytes, t4_local = send_recv_with_retries(sock, ntp_payload, target, timeout, retries, verbose=verbose)
    except Exception:
        print(" |-- [FAIL] No response received from the server after retries.")
        return None

    try:
        meta = parse_response_metadata(raw_data)
    except ValueError as e:
        print(f" |-- [FAIL] {e}")
        return None

    if verbose:
        print(f" |-- Received Payload metadata: NTPv{meta['version']}, Mode={meta['mode']}, Stratum={meta['stratum']}, LI={meta['leap']}")
        if meta['ref_ts'] is not None:
            print(f" |-- Server Reference TS: {meta['ref_ts']:.6f}")
        else:
            print(" |-- Server Reference TS: (zero)")

    # Strict loopback: server should echo our T1 into Origin exactly (raw bytes)
    origin_match = (meta['origin_raw'] == t1_bytes)
    if origin_match:
        print(" |-- [PASS] Standard loopback handshake validation verified (raw bytes match).")
        return {
            "t1_raw": t1_bytes,
            "t2_raw": meta['receive_raw'],
            "t3_raw": meta['transmit_raw'],
            "t4_local_bytes": t4_local_bytes,
            "t4_local_float": t4_local
        }
    else:
        # Provide helpful diagnostics: show expected vs got (hex)
        print(" |-- [FAIL] Loopback check mismatched (origin raw bytes differ).")
        if verbose:
            print(f"     Expected (T1 hex): {t1_bytes.hex()}")
            print(f"     Got      (Origin): {meta['origin_raw'].hex()}")
        return None


def run_interleaved_test(sock, target, version, previous_exchange, timeout, retries, verbose=False, transmit_tolerance_ms=5.0):
    """Tests Interleaved NTP behaviour with RFC 9769 handshake mapping."""
    print(f"\n[+] Running Interleaved NTPv{version} Validation Test...")

    if not previous_exchange:
        print(" |-- [SKIPPED] Missing previous transaction history to execute Interleaved step.")
        return

    # Extract historical elements (use raw bytes)
    t1_prev_bytes = previous_exchange["t1_raw"]
    t2_prev_bytes = previous_exchange["t2_raw"]
    t3_prev_bytes = previous_exchange["t3_raw"]
    t4_prev_bytes = previous_exchange["t4_local_bytes"]

    # Structural RFC 9769 Payload mapping (interleaved)
    # origin_ts <- previous server T2 (t2_prev_bytes)
    # receive_ts <- previous client T4 (t4_prev_bytes)
    # transmit_ts <- previous client T1 (t1_prev_bytes)
    ntp_payload = build_ntp_payload(
        version=version,
        mode=3,
        origin_ts=t2_prev_bytes,
        receive_ts=t4_prev_bytes,
        transmit_ts=t1_prev_bytes
    )

    if verbose:
        print(" |-- Sending Interleaved Handshake Request packet...")
    try:
        raw_data, _remote, _t4_bytes, _t4_float = send_recv_with_retries(sock, ntp_payload, target, timeout, retries, verbose=verbose)
    except Exception:
        print(" |-- [FAIL] No response received from the server after retries.")
        return

    try:
        meta = parse_response_metadata(raw_data)
    except ValueError as e:
        print(f" |-- [FAIL] {e}")
        return

    # Expected origin is our previous T4 (raw)
    expected_origin_raw = t4_prev_bytes
    expected_transmit_raw = t3_prev_bytes

    origin_match = (meta['origin_raw'] == expected_origin_raw)
    # For transmit match, allow small tolerance in case of rounding; prefer raw equality if possible
    transmit_match = False
    if meta['transmit_raw'] == expected_transmit_raw:
        transmit_match = True
    else:
        # fallback: compare as floats with tolerance (ms)
        t_transmit = meta['transmit']
        t_expected = parse_ntp_timestamp(expected_transmit_raw)
        if t_transmit is not None and t_expected is not None:
            delta = abs(t_transmit - t_expected)
            transmit_match = (delta <= (transmit_tolerance_ms / 1000.0))

    if verbose:
        print(f" |-- Server Echoed Origin TS:   {meta['origin'] if meta['origin'] is not None else '(zero)'} (expected prior client T4)")
        print(f" |-- Server Receive TS (T2):     {meta['receive'] if meta['receive'] is not None else '(zero)'}")
        print(f" |-- Server Transmit TS (T3):    {meta['transmit'] if meta['transmit'] is not None else '(zero)'} (expected prior server T3)")

    if origin_match and transmit_match:
        print(" |-- [PASS] RFC 9769 interleaved timestamp validation verified.")
    else:
        print(" |-- [FAIL] Server did not return expected interleaved timestamp history.")
        if verbose:
            if not origin_match:
                print(f"     Origin mismatch: expected T4 hex {expected_origin_raw.hex()}, got {meta['origin_raw'].hex()}")
            if not transmit_match:
                print(f"     Transmit mismatch: expected prior T3 hex {expected_transmit_raw.hex()}, got {meta['transmit_raw'].hex()}")

# ==============================================================================
# MAIN TEST TOOL ENTRY POINT
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description="NTP RFC9769 validation tool.")
    parser.add_argument("--target", "-t", default=DEFAULT_TARGET_IP, help="Target IP address of the NTP server")
    parser.add_argument("--port", "-p", type=int, default=DEFAULT_PORT, help="Target UDP port (default 123)")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="Socket timeout seconds")
    parser.add_argument("--retries", type=int, default=DEFAULT_RETRIES, help="Number of send/recv retries")
    parser.add_argument("--verbose", action="store_true", help="Verbose debug output")
    args = parser.parse_args()

    target = (args.target, args.port)
    timeout = args.timeout
    retries = args.retries
    verbose = args.verbose

    print(" ")
    print("\n==============================================================================")
    print(" ")

    banner = "                     NTP RFC 9769 Protocol Validation Test                   "
    print(banner)
    print(" ")
    # --- Centered IP address line ---
    ip_line = f"Testing IP: {args.target}"
    print(ip_line.center(len(banner)))

    # --- Centered local date/time line (12-hour format with am/pm) ---
    local_dt = time.strftime("%Y-%m-%d %I:%M:%S %p")
    dt_line = f"Local Time: {local_dt}"
    print(dt_line.center(len(banner)))
    print(" ")
    print("\n------------------------------------------------------------------------------")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # --- TEST CASE 1: NTPv3 Standard ---
        ntpv3_history = run_standard_test(sock, target, version=3, timeout=timeout, retries=retries, verbose=verbose)
        time.sleep(1.0)

        # --- TEST CASE 2: NTPv3 Interleaved ---
        run_interleaved_test(sock, target, version=3, previous_exchange=ntpv3_history, timeout=timeout, retries=retries, verbose=verbose)
        time.sleep(1.0)

        # --- TEST CASE 3: NTPv4 Standard ---
        ntpv4_history = run_standard_test(sock, target, version=4, timeout=timeout, retries=retries, verbose=verbose)
        time.sleep(1.0)

        # --- TEST CASE 4: NTPv4 Interleaved ---
        run_interleaved_test(sock, target, version=4, previous_exchange=ntpv4_history, timeout=timeout, retries=retries, verbose=verbose)

    finally:
        sock.close()

    print("\n------------------------------------------------------------------------------")
    print(" ")
    print("                NTP RFC 9769 Protocol Validation Test Complete")
    print(" ")
    print("                          Copyright Rob Latour, 2026")
    print("                                License: MIT")
    print("              Website: https://github.com/roblatour/ESP32TimeServer")
    print(" ")
    print("\n==============================================================================")
    print(" ")

if __name__ == "__main__":
    main()
