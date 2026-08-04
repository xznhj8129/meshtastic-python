"""Configure Meshtastic's frame-aware MAVLink mesh mode.

The generated Python bindings on this branch predate the canonical
Serial_Mode.MAVLINK enum value 11. Proto3 preserves open enum values, so this
helper can configure the mode without regenerating every protobuf binding.

serial.peer_node is a legacy field from the point-to-point prototype. Current
firmware ignores it because MAVLink mesh routing is symmetric and any-to-any.
"""

from __future__ import annotations

import argparse
import time
import warnings
from typing import Iterator, Optional, Tuple

from meshtastic.protobuf import module_config_pb2

MAVLINK_SERIAL_MODE = 11
MODE_FIELD = 7
LEGACY_PEER_NODE_FIELD = 9


def _encode_varint(value: int) -> bytes:
    if value < 0:
        raise ValueError("varints must be non-negative")
    encoded = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            encoded.append(byte | 0x80)
        else:
            encoded.append(byte)
            return bytes(encoded)


def _decode_varint(data: bytes, offset: int) -> Tuple[int, int]:
    value = 0
    shift = 0
    while offset < len(data) and shift < 70:
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, offset
        shift += 7
    raise ValueError("truncated or oversized protobuf varint")


def _iter_wire_fields(data: bytes) -> Iterator[Tuple[int, int, Optional[int]]]:
    offset = 0
    while offset < len(data):
        key, offset = _decode_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 0:
            raise ValueError("invalid protobuf field number 0")
        if wire_type == 0:
            value, offset = _decode_varint(data, offset)
            yield field_number, wire_type, value
        elif wire_type == 1:
            offset += 8
            yield field_number, wire_type, None
        elif wire_type == 2:
            length, offset = _decode_varint(data, offset)
            offset += length
            yield field_number, wire_type, None
        elif wire_type == 5:
            offset += 4
            yield field_number, wire_type, None
        else:
            raise ValueError(f"unsupported protobuf wire type {wire_type}")
        if offset > len(data):
            raise ValueError("truncated protobuf field")


def _merge_varint_field(message, field_number: int, value: int) -> None:
    key = field_number << 3
    message.MergeFromString(_encode_varint(key) + _encode_varint(value))


def read_peer_node(serial_config) -> int:
    """Read a legacy encoded peer_node value, if present.

    Current frame-aware mesh firmware ignores this field. The reader remains for
    inspection and backwards compatibility with old configurations.
    """
    peer = 0
    for field_number, wire_type, value in _iter_wire_fields(serial_config.SerializeToString()):
        if field_number == LEGACY_PEER_NODE_FIELD and wire_type == 0 and value is not None:
            peer = value
    return peer


def configure_serial(
    serial_config,
    peer_node: Optional[int] = None,
    *,
    enabled: bool = True,
    rxd: Optional[int] = None,
    txd: Optional[int] = None,
    baud: Optional[int] = None,
) -> None:
    """Enable frame-aware MAVLink mesh mode on a SerialConfig message.

    peer_node is accepted only for source compatibility. It is not written and
    has no routing effect in current firmware.
    """
    if peer_node is not None:
        if not 0 <= peer_node <= 0xFFFFFFFF:
            raise ValueError("peer_node must fit in uint32")
        if peer_node != 0:
            warnings.warn("peer_node is deprecated and ignored by MAVLink mesh mode", DeprecationWarning, stacklevel=2)

    serial_config.enabled = enabled
    if rxd is not None:
        serial_config.rxd = rxd
    if txd is not None:
        serial_config.txd = txd
    if baud is not None:
        enum_name = f"BAUD_{baud}"
        try:
            serial_config.baud = module_config_pb2.ModuleConfig.SerialConfig.Serial_Baud.Value(enum_name)
        except ValueError as exc:
            raise ValueError(f"unsupported serial baud {baud}") from exc

    _merge_varint_field(serial_config, MODE_FIELD, MAVLINK_SERIAL_MODE)


def _parse_int(value: str) -> int:
    return int(value, 0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure Meshtastic MAVLink mesh mode")
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument("--port", help="Meshtastic serial device, for example /dev/ttyACM0")
    transport.add_argument("--host", help="Meshtastic TCP hostname or address")
    parser.add_argument("--peer", type=_parse_int, help="deprecated compatibility option; ignored")
    parser.add_argument("--rxd", type=int, help="serial RX GPIO")
    parser.add_argument("--txd", type=int, help="serial TX GPIO")
    parser.add_argument("--baud", type=int, help="serial baud, for example 57600 or 115200")
    args = parser.parse_args()

    if args.host:
        from meshtastic.tcp_interface import TCPInterface

        interface = TCPInterface(args.host)
    else:
        from meshtastic.serial_interface import SerialInterface

        interface = SerialInterface(devPath=args.port)

    try:
        node = interface.localNode
        configure_serial(
            node.moduleConfig.serial,
            args.peer,
            rxd=args.rxd,
            txd=args.txd,
            baud=args.baud,
        )
        node.writeConfig("serial")
        time.sleep(0.5)
        print("Set serial.mode=MAVLINK mesh mode")
        if args.peer is not None:
            print("Note: --peer is deprecated and was ignored")
    finally:
        interface.close()


if __name__ == "__main__":
    main()
