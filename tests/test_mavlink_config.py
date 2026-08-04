import warnings

from meshtastic.mavlink_config import MAVLINK_SERIAL_MODE, configure_serial, read_peer_node
from meshtastic.protobuf import module_config_pb2


def test_configure_serial_sets_mesh_mode_and_uart():
    serial = module_config_pb2.ModuleConfig.SerialConfig()
    configure_serial(serial, rxd=44, txd=43, baud=115200)

    assert serial.enabled is True
    assert serial.mode == MAVLINK_SERIAL_MODE
    assert serial.rxd == 44
    assert serial.txd == 43
    assert read_peer_node(serial) == 0

    restored = module_config_pb2.ModuleConfig.SerialConfig()
    restored.ParseFromString(serial.SerializeToString())
    assert restored.mode == MAVLINK_SERIAL_MODE
    assert read_peer_node(restored) == 0


def test_legacy_peer_argument_is_ignored():
    serial = module_config_pb2.ModuleConfig.SerialConfig()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        configure_serial(serial, 0x12345678)

    assert serial.mode == MAVLINK_SERIAL_MODE
    assert read_peer_node(serial) == 0
    assert len(caught) == 1
    assert issubclass(caught[0].category, DeprecationWarning)


def test_legacy_peer_zero_is_silent_and_ignored():
    serial = module_config_pb2.ModuleConfig.SerialConfig()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        configure_serial(serial, 0)

    assert serial.mode == MAVLINK_SERIAL_MODE
    assert read_peer_node(serial) == 0
    assert caught == []
