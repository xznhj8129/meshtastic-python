from meshtastic.mavlink_config import MAVLINK_SERIAL_MODE, configure_serial, read_peer_node
from meshtastic.protobuf import module_config_pb2


def test_configure_serial_preserves_canonical_peer_field():
    serial = module_config_pb2.ModuleConfig.SerialConfig()
    configure_serial(serial, 0x12345678, rxd=44, txd=43, baud=115200)

    assert serial.enabled is True
    assert serial.mode == MAVLINK_SERIAL_MODE
    assert serial.rxd == 44
    assert serial.txd == 43
    assert read_peer_node(serial) == 0x12345678

    restored = module_config_pb2.ModuleConfig.SerialConfig()
    restored.ParseFromString(serial.SerializeToString())
    assert restored.mode == MAVLINK_SERIAL_MODE
    assert read_peer_node(restored) == 0x12345678


def test_peer_zero_selects_discovery_mode():
    serial = module_config_pb2.ModuleConfig.SerialConfig()
    configure_serial(serial, 0)
    assert serial.mode == MAVLINK_SERIAL_MODE
    assert read_peer_node(serial) == 0


def test_last_peer_value_wins():
    serial = module_config_pb2.ModuleConfig.SerialConfig()
    configure_serial(serial, 0x11111111)
    configure_serial(serial, 0x22222222)
    assert read_peer_node(serial) == 0x22222222
