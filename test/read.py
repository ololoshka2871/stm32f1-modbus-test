#!/usr/bin/env python3

from pymodbus.client import ModbusSerialClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian

SLAVE_ADDR = 0x01

client = ModbusSerialClient('/dev/ttyUSB0', baudrate=57600, parity='N')
dev_id = client.read_holding_registers(0, 1, slave=SLAVE_ADDR)
hw_ver = client.read_holding_registers(0x0010, 2, slave=SLAVE_ADDR)
mcu_id = client.read_holding_registers(0x0012, 2, slave=SLAVE_ADDR)
sw_ver = client.read_holding_registers(0x0020, 2, slave=SLAVE_ADDR)
channels_cfg = client.read_holding_registers(0x0100, 20, slave=SLAVE_ADDR)
pwm_freq = client.read_holding_registers(0x0200, 1, slave=SLAVE_ADDR)

hw_ver = BinaryPayloadDecoder.fromRegisters(hw_ver.registers, byteorder=Endian.Little)
mcu_id = BinaryPayloadDecoder.fromRegisters(mcu_id.registers, byteorder=Endian.Little)
sw_ver = BinaryPayloadDecoder.fromRegisters(sw_ver.registers, byteorder=Endian.Little)

print(
    f"""dev_id = 0x{dev_id.registers[0]:0X}
hw_ver = 0x{hw_ver.decode_32bit_uint():08X}
mcu_id = 0x{mcu_id.decode_32bit_uint():08X}
sw_ver = 0x{sw_ver.decode_32bit_uint():08X}
channels_cfg = {channels_cfg.registers}
pwm_freq = {pwm_freq.registers[0]}
"""
)