"""Configure Meshtastic's MAVLink serial bridge with older generated Python bindings.

The canonical schema adds Serial_Mode.MAVLINK = 11 and SerialConfig.peer_node = 9.
Proto3 preserves unknown fields, so this helper can configure both values without
forcing an unrelated wholesale refresh of every generated Python protobuf file.
"""

from __future__ import annotations

import argparse
import time
from typing import Iterator, Optional, Tuple

from meshtastic.protobuf import module_config_pb2

MAVLINK_SERIAL_MODE = 11
PEER_NODE_FIELD = 9
MODE_FIELD = 7


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
    """Return the last encoded peer_node value, or zero when not present."""
    peer = 0
    for field_number, wire_type, value in _iter_wire_fields(serial_config.SerializeToString()):
        if field_number == PEER_NODE_FIELD and wire_type == 0 and value is not None:
            peer = value
    return peer


def configure_serial(
    serial_config,
    peer_node: int = 0,
    *,
    enabled: bool = True,
    rxd: Optional[int] = None,
    txd: Optional[int] = None,
    baud: Optional[int] = None,
) -> None:
    """Set MAVLink mode and its fixed peer on a SerialConfig message.

    A peer of zero preserves firmware discovery mode, where the first non-empty
    SERIAL_APP sender becomes the peer until reboot.
    """
    if not 0 <= peer_node <= 0xFFFFFFFF:
        raise ValueError("peer_node must fit in uint32")

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

    # Field 7 is known to older bindings but value 11 is new. Proto3 enum fields are
    # open, so parsing the canonical value sets it even when the local name table lags.
    _merge_varint_field(serial_config, MODE_FIELD, MAVLINK_SERIAL_MODE)

    # Field 9 is unknown to the old generated class and is therefore retained in the
    # message's unknown-field set. CopyFrom and serialization preserve it end to end.
    _merge_varint_field(serial_config, PEER_NODE_FIELD, peer_node)


def _parse_int(value: str) -> int:
    return int(value, 0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure Meshtastic MAVLink serial bridge")
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument("--port", help="Meshtastic serial device, for example /dev/ttyACM0")
    transport.add_argument("--host", help="Meshtastic TCP hostname or address")
    parser.add_argument("--peer", type=_parse_int, default=0, help="fixed node number, or 0 for first-sender discovery")
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
        print(f"Set serial.mode=MAVLINK and serial.peer_node=0x{args.peer:08x}")
    finally:
        interface.close()


if __name__ == "__main__":
    main()
