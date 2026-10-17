#
# Copyright Rob Latour, 2026
# License: MIT
# Website: https://github.com/roblatour/ESP32TimeServer
#


"""Query NTP servers repeatedly and compare their reported clock offsets."""

from __future__ import annotations

import argparse
import ipaddress
import re
import statistics
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Sequence

# Change this value to set the default sample count.
SAMPLE_COUNT = 20
PING_TIMEOUT_SECONDS = 5
RESULTS_FILE = Path("results.txt")
INTERNAL_NETWORKS = (
    ipaddress.IPv4Network("10.0.0.0/8"),
    ipaddress.IPv4Network("172.16.0.0/12"),
    ipaddress.IPv4Network("192.168.0.0/16"),
    ipaddress.IPv4Network("169.254.0.0/16"),
)
CASE_INSENSITIVE_OPTIONS = frozenset(
    {"--samples", "--servers", "--timeout", "--output", "-h", "--help", "-help"}
)

OFFSET_PATTERN = re.compile(
    r"server\s+(?P<server>\S+),.*?\boffset\s+(?P<offset>[+-]?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)


def query_offset(server: str, timeout_seconds: float) -> float:
    """Run ntpdate and return the offset reported for one server."""
    try:
        completed = subprocess.run(
            ["ntpdate", "-q", server],
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            check=False,
        )
    except FileNotFoundError as error:
        raise RuntimeError(
            "The 'ntpdate' command was not found. Install it or add it to PATH."
        ) from error
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(
            f"ntpdate timed out after {timeout_seconds:g} seconds for {server}."
        ) from error

    command_output = "\n".join(
        part for part in (completed.stdout, completed.stderr) if part
    )
    match = OFFSET_PATTERN.search(command_output)
    if match is None:
        detail = command_output.strip() or "No output was returned."
        raise RuntimeError(
            f"Could not parse an offset from ntpdate output for {server}. "
            f"Exit code: {completed.returncode}. Output: {detail}"
        )

    return float(match.group("offset"))


def ping_servers(servers: Sequence[str]) -> bool:
    """Return whether every server responds to a single ping request."""
    unresponsive_servers: list[str] = []
    for server in servers:
        if sys.platform.startswith("win"):
            command = [
                "ping",
                "-n",
                "1",
                "-w",
                str(PING_TIMEOUT_SECONDS * 1000),
                server,
            ]
        else:
            command = ["ping", "-c", "1", server]

        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=PING_TIMEOUT_SECONDS + 1,
                check=False,
            )
        except FileNotFoundError:
            print(
                "The 'ping' command was not found. Install it or add it to PATH.",
                file=sys.stderr,
            )
            return False
        except subprocess.TimeoutExpired:
            unresponsive_servers.append(server)
        else:
            if completed.returncode != 0:
                unresponsive_servers.append(server)

    if not unresponsive_servers:
        return True

    for server in unresponsive_servers:
        print(
            f"{server} did not respond to its ping - testing will not proceed.",
            file=sys.stderr,
        )
    return False


def format_offset(offset: float) -> str:
    """Format an offset in seconds, retaining its sign."""
    return f"{offset:+.6f} sec"


def collect_offsets(
    servers: Sequence[str], sample_count: int, timeout_seconds: float
) -> tuple[dict[str, list[float]], dict[str, list[str]]]:
    """Collect successful offsets and record any failures for each server."""
    offsets = {server: [] for server in servers}
    failures = {server: [] for server in servers}

    for sample_number in range(1, sample_count + 1):
        print(f"Collecting sample {sample_number}/{sample_count}...")
        for server in servers:
            try:
                offset = query_offset(server, timeout_seconds)
            except RuntimeError as error:
                message = f"Sample {sample_number}: {error}"
                failures[server].append(message)
                print(f"  {server}: FAILED - {error}", file=sys.stderr)
            else:
                offsets[server].append(offset)
                print(f"  {server}: {format_offset(offset)}")

    return offsets, failures


def build_report(
    offsets: dict[str, list[float]],
    failures: dict[str, list[str]],
    requested_samples: int,
) -> str:
    """Create a text report of per-server offset statistics and comparison."""
    lines = [
        "NTP Server Alignment Results",
        "=" * 29,
        f"Generated: {datetime.now().astimezone().isoformat(timespec='seconds')}",
        f"Requested samples per server: {requested_samples}",
        "",
        "Per-server results",
        "-" * 18,
    ]

    averages: dict[str, float] = {}
    for server, samples in offsets.items():
        lines.append(f"Server: {server}")
        lines.append(f"  Successful samples: {len(samples)}/{requested_samples}")

        if samples:
            average = statistics.mean(samples)
            averages[server] = average
            standard_deviation = statistics.stdev(samples) if len(samples) > 1 else 0.0
            lines.extend(
                [
                    f"  Average offset: {format_offset(average)}",
                    f"  Minimum offset: {format_offset(min(samples))}",
                    f"  Maximum offset: {format_offset(max(samples))}",
                    f"  Standard deviation: {standard_deviation:.6f} sec",
                    "  Individual offsets: "
                    + ", ".join(format_offset(offset) for offset in samples),
                ]
            )
        else:
            lines.append("  No successful samples; an average cannot be calculated.")

        if failures[server]:
            lines.append(f"  Failed samples: {len(failures[server])}")
            lines.extend(f"    {failure}" for failure in failures[server])
        lines.append("")

    lines.extend(["Comparison of server averages", "-" * 29])
    if len(averages) < 2:
        lines.append(
            "Comparison unavailable: offsets were successfully collected from fewer "
            "than two servers."
        )
        return "\n".join(lines) + "\n"

    lines.append("Pairwise absolute differences between average offsets:")
    average_items = list(averages.items())
    pairwise_differences: list[float] = []
    for left_index, (left_server, left_average) in enumerate(average_items):
        for right_server, right_average in average_items[left_index + 1 :]:
            difference = abs(left_average - right_average)
            pairwise_differences.append(difference)
            lines.append(
                f"  {left_server} vs {right_server}: {difference:.6f} sec "
                f"({difference * 1000:.3f} ms)"
            )

    spread = max(averages.values()) - min(averages.values())
    lines.extend(
        [
            "",
            f"Maximum spread between server averages: {spread:.6f} sec "
            f"({spread * 1000:.3f} ms)",
            (
                "Alignment assessment: The server averages "
                f"{'are' if len(averages) == 3 else 'among responding servers are'} "
                f"within {spread * 1000:.3f} ms of one another."
            ),
        ]
    )
    if len(averages) < len(offsets):
        lines.append(
            "Note: This assessment excludes servers that returned no successful samples."
        )

    return "\n".join(lines) + "\n"


def normalize_option_names(arguments: Sequence[str]) -> list[str]:
    """Normalize supported command options without changing their values."""
    normalized_arguments: list[str] = []
    for argument in arguments:
        option, separator, value = argument.partition("=")
        normalized_option = option.lower()
        if normalized_option in CASE_INSENSITIVE_OPTIONS:
            normalized_arguments.append(
                normalized_option + (separator + value if separator else "")
            )
        else:
            normalized_arguments.append(argument)
    return normalized_arguments


def validate_servers(servers: Sequence[str]) -> list[str]:
    """Return normalized server addresses after enforcing allowed IPv4 ranges."""
    normalized_servers: list[str] = []
    errors: list[str] = []

    for server in servers:
        try:
            address = ipaddress.IPv4Address(server)
        except ipaddress.AddressValueError:
            errors.append(
                f"{server} is not a valid IPv4 address - testing will not proceed."
            )
            continue

        normalized_address = str(address)
        normalized_servers.append(normalized_address)
        if not any(address in network for network in INTERNAL_NETWORKS):
            errors.append(
                f"{normalized_address} does not appear to be an internal network - "
                "testing will not proceed."
            )

    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        raise SystemExit(2)

    return normalized_servers


def parse_arguments(command_line: Sequence[str] | None = None) -> argparse.Namespace:
    """Return validated command-line options."""
    parser = argparse.ArgumentParser(
        description="Query internal IPv4 NTP servers and compare their average clock offsets.",
        add_help=False,
    )
    parser.add_argument(
        "-h",
        "--help",
        "-help",
        action="help",
        help="Show this help message and exit.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=SAMPLE_COUNT,
        help=f"Queries per server (default: {SAMPLE_COUNT}).",
    )
    parser.add_argument(
        "--servers",
        nargs="+",
        required=True,
        metavar="IP_ADDRESS",
        help="Two or more internal IPv4 NTP server addresses to query.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Maximum seconds to wait for each ntpdate command (default: 30).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=RESULTS_FILE,
        help=f"Report file path (default: {RESULTS_FILE}).",
    )

    supplied_arguments = normalize_option_names(
        sys.argv[1:] if command_line is None else command_line
    )
    if not supplied_arguments or "?" in supplied_arguments:
        parser.print_help()
        raise SystemExit(0)

    arguments = parser.parse_args(supplied_arguments)
    if arguments.samples < 1:
        parser.error("--samples must be at least 1.")
    if arguments.timeout <= 0:
        parser.error("--timeout must be greater than zero.")

    normalized_servers = validate_servers(arguments.servers)
    if len(normalized_servers) < 2:
        parser.error("Provide at least two server IP addresses with --servers.")
    if len(set(normalized_servers)) != len(normalized_servers):
        parser.error("Each IP address supplied with --servers must be unique.")
    arguments.servers = normalized_servers
    return arguments


def confirm_output_overwrite(output: Path) -> bool | None:
    """Return whether testing may proceed after handling an existing output file."""
    if not output.exists():
        return True

    while True:
        try:
            response = input(
                f"{output} output file already exists. "
                "Is it ok to overwrite it and proceed (y/N)? "
            )
        except EOFError:
            print(
                "No response was available. The file was not changed and the tests "
                "were not run.",
                file=sys.stderr,
            )
            return None

        normalized_response = response.strip().lower()
        if normalized_response == "y":
            try:
                output.unlink()
            except OSError as error:
                print(
                    f"Failed to delete existing output file {output}: {error}",
                    file=sys.stderr,
                )
                return None
            return True
        if normalized_response in {"", "n"}:
            print("The file was not changed and the tests were not run.")
            return False


def main() -> int:
    """Collect NTP data, write the report, and return a process status."""
    arguments = parse_arguments()
    if not ping_servers(arguments.servers):
        return 1

    may_proceed = confirm_output_overwrite(arguments.output)
    if may_proceed is None:
        return 1
    if not may_proceed:
        return 0

    offsets, failures = collect_offsets(
        arguments.servers, arguments.samples, arguments.timeout
    )
    report = build_report(offsets, failures, arguments.samples)

    try:
        arguments.output.write_text(report, encoding="utf-8")
    except OSError as error:
        print(f"Failed to write {arguments.output}: {error}", file=sys.stderr)
        return 1

    print(f"\nReport written to {arguments.output.resolve()}")
    return 0 if any(offsets.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
