#!/usr/bin/env python3
"""Measure and decode the real ESP32-C5 RTP/H.264 stream on the Wi-Fi host."""
from __future__ import annotations

import argparse
from fractions import Fraction
import json
from pathlib import Path
import socket
import statistics
import struct
import time

import av
import cv2
import numpy as np


def decoder():
    context = av.CodecContext.create("h264", "r")
    context.thread_count = 1
    context.thread_type = av.codec.context.ThreadType.SLICE
    context.flags |= av.codec.context.Flags.low_delay
    return context


def parse_rtp(packet: bytes):
    if len(packet) < 12 or packet[0] >> 6 != 2:
        return None
    offset = 12 + (packet[0] & 15) * 4
    if offset > len(packet):
        return None
    if packet[0] & 16:
        if offset + 4 > len(packet):
            return None
        offset += 4 + struct.unpack_from("!H", packet, offset + 2)[0] * 4
    end = len(packet)
    if packet[0] & 32:
        if not packet[-1] or packet[-1] > end - offset:
            return None
        end -= packet[-1]
    if offset >= end:
        return None
    sequence, timestamp, ssrc = struct.unpack_from("!HII", packet, 2)
    return packet[1] & 127, bool(packet[1] & 128), sequence, timestamp, ssrc, packet[offset:end]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="192.168.4.2")
    parser.add_argument("--source", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--startup-timeout", type=float, default=45)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.25)
    codec = decoder()
    av.logging.set_level(av.logging.FATAL)
    source_counts = {}
    nalu_counts = {}
    decode_error_messages = []
    video_packets = metadata_packets = wire_bytes = missing = reordered = 0
    decode_errors = discarded_units = session_changes = 0
    startup_decode_errors = steady_decode_errors = 0
    current_ssrc = next_sequence = unit_timestamp = None
    units = []
    fragment = bytearray()
    damaged = False
    arrivals = []
    timestamps = []
    frame_ids = []
    changes = []
    last_gray = last_image = None
    width = height = 0
    start = time.perf_counter()
    failure = None
    recording = (args.output / "stream.h264").open("wb")
    print(f"LISTEN {args.bind}:{args.port} source={args.source}", flush=True)
    try:
        while True:
            now = time.perf_counter()
            if arrivals and now - arrivals[0] >= args.seconds:
                break
            if not arrivals and now - start > args.startup_timeout:
                failure = "no decoded video before startup timeout"
                break
            if arrivals and now - arrivals[-1] > 5:
                failure = "video stalled for more than five seconds"
                break
            try:
                packet, source = sock.recvfrom(65536)
            except socket.timeout:
                continue
            source_counts[source[0]] = source_counts.get(source[0], 0) + 1
            if source[0] != args.source:
                continue
            parsed = parse_rtp(packet)
            if parsed is None:
                continue
            kind, marker, sequence, timestamp, ssrc, payload = parsed
            wire_bytes += len(packet)
            if kind == 98:
                metadata_packets += 1
                if len(payload) >= 16 and payload[:3] == b"RM\x01":
                    frame_ids.append(struct.unpack_from("!I", payload, 8)[0])
                continue
            if kind != 96:
                continue
            video_packets += 1
            if current_ssrc != ssrc:
                if current_ssrc is not None:
                    session_changes += 1
                current_ssrc = ssrc
                next_sequence = None
                unit_timestamp = None
                units.clear()
                fragment.clear()
                damaged = False
                codec = decoder()
            if next_sequence is not None and sequence != next_sequence:
                delta = (sequence - next_sequence) & 0xFFFF
                if delta >= 0x8000:
                    reordered += 1
                    continue
                missing += delta
                damaged = True
                fragment.clear()
            next_sequence = (sequence + 1) & 0xFFFF
            if unit_timestamp is not None and timestamp != unit_timestamp:
                if units or fragment:
                    discarded_units += 1
                units.clear()
                fragment.clear()
                damaged = False
            unit_timestamp = timestamp
            nalu_type = payload[0] & 31
            if 1 <= nalu_type <= 23:
                nalu_counts[nalu_type] = nalu_counts.get(nalu_type, 0) + 1
                units.append(b"\x00\x00\x00\x01" + payload)
            elif nalu_type == 28 and len(payload) > 2:
                if payload[1] & 128:
                    kind_id = payload[1] & 31
                    nalu_counts[kind_id] = nalu_counts.get(kind_id, 0) + 1
                    fragment = bytearray([(payload[0] & 0xE0) | (payload[1] & 31)])
                elif not fragment:
                    damaged = True
                if fragment:
                    fragment.extend(payload[2:])
                if payload[1] & 64:
                    if fragment:
                        units.append(b"\x00\x00\x00\x01" + bytes(fragment))
                    fragment.clear()
            else:
                damaged = True
            if not marker:
                continue
            if damaged or fragment or not units:
                discarded_units += 1
                frames = []
            else:
                encoded = b"".join(units)
                recording.write(encoded)
                access_unit = av.Packet(encoded)
                access_unit.pts = timestamp
                access_unit.dts = timestamp
                access_unit.time_base = Fraction(1, 90000)
                try:
                    frames = codec.decode(access_unit)
                except av.error.FFmpegError as error:
                    decode_errors += 1
                    if arrivals:
                        steady_decode_errors += 1
                    else:
                        startup_decode_errors += 1
                    if len(decode_error_messages) < 3:
                        decode_error_messages.append(str(error))
                    frames = []
            units.clear()
            fragment.clear()
            damaged = False
            unit_timestamp = None
            for frame in frames:
                arrival = time.perf_counter()
                arrivals.append(arrival)
                timestamps.append(timestamp)
                image = frame.to_ndarray(format="bgr24")
                width, height = frame.width, frame.height
                gray = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (160, 120))
                if last_gray is not None:
                    changes.append(float(np.abs(gray.astype(np.int16) - last_gray.astype(np.int16)).mean()))
                last_gray, last_image = gray, image
                if len(arrivals) == 1:
                    cv2.imwrite(str(args.output / "first-frame.png"), image)
                if len(arrivals) % 150 == 0:
                    elapsed = arrivals[-1] - arrivals[0]
                    print(f"DECODED {len(arrivals)} fps={(len(arrivals)-1)/elapsed:.3f} missing={missing}", flush=True)
    finally:
        sock.close()
        recording.close()
    elapsed = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0.0
    timestamp_seconds = ((timestamps[-1] - timestamps[0]) & 0xFFFFFFFF) / 90000 if len(timestamps) > 1 else 0.0
    gaps = [(b-a)*1000 for a, b in zip(arrivals, arrivals[1:])]
    report = {
        "transport": "ESP32-C5 Wi-Fi RTP/H264", "bind": args.bind, "source": args.source,
        "source_packets": source_counts, "decoded_frames": len(arrivals),
        "width": width, "height": height, "seconds": elapsed,
        "decoded_fps": (len(arrivals)-1)/elapsed if elapsed else 0,
        "source_timestamp_fps": (len(arrivals)-1)/timestamp_seconds if timestamp_seconds else 0,
        "video_packets": video_packets, "metadata_packets": metadata_packets,
        "rtp_missing_packets": missing, "rtp_reordered_or_duplicate_packets": reordered,
        "discarded_access_units": discarded_units, "decode_errors": decode_errors,
        "startup_decode_errors": startup_decode_errors,
        "decode_errors_after_first_frame": steady_decode_errors,
        "session_changes": session_changes, "received_rtp_bytes": wire_bytes,
        "nalu_counts": nalu_counts, "decode_error_messages": decode_error_messages,
        "largest_arrival_gap_ms": max(gaps, default=0),
        "p95_arrival_gap_ms": float(np.percentile(gaps, 95)) if gaps else 0,
        "mean_absolute_frame_change": statistics.mean(changes) if changes else 0,
        "metadata_first_frame_id": frame_ids[0] if frame_ids else None,
        "metadata_last_frame_id": frame_ids[-1] if frame_ids else None,
        "failure": failure,
    }
    if last_image is not None:
        cv2.imwrite(str(args.output / "preview.png"), last_image)
    (args.output / "report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2), flush=True)
    return 0 if failure is None and len(arrivals) > 1 else 2


if __name__ == "__main__":
    raise SystemExit(main())
