#!/usr/bin/env python3

# Simple PTCBP parser

import io
import struct
import enum
from collections import namedtuple
from typing import BinaryIO, Optional, Union

import packbits

CMD_SCHEMA = (
    # cmd, mnemonic, param_schema, (get_len_from_param, set_len_to_param, min_param_len)
    (b'\x00', 'nop', None, None),
    (b'\x1b@', 'reset', None, None),
    (b'\x1biS', 'get_status', None, None),
    (b'\x1bia', 'use_command_set', 'B', None),
    (b'\x1biz', 'set_print_parameters', '4BI2B', None),
    (b'\x1biM', 'set_page_mode', 'B', None),
    (b'\x1biK', 'set_page_mode_advanced', 'B', None),
    (b'\x1bid', 'set_page_margin', 'H', None),
    (b'M', 'compression', 'B', None),
    (b'\x0c', 'print_page', None, None),
    (b'\x1a', 'print', None, None),
    (b'g', 'data2', 'H', (lambda params: params[0], lambda params, l: params.__setitem__(0, l), 1)),
    # TODO is it really for RLE data or just an alias to 'g'?
    (b'G', 'data', 'H', (lambda params: params[0], lambda params, l: params.__setitem__(0, l), 1)),
    (b'Z', 'zerofill', None, None),
)

MNEMONICS = {e[1][:]: e for e in CMD_SCHEMA}
OPS_FLAT = {e[0][:]: e for e in CMD_SCHEMA}
def _build_op_tree() -> dict:
    tree = {}
    for e in CMD_SCHEMA:
        current_level = tree
        for byte in e[0][:-1]:
            if current_level.get(byte) is None:
                current_level[byte] = {}
            current_level = current_level[byte]
        current_level[e[0][-1]] = e
    return tree

OPS = _build_op_tree()

PrintParameters = namedtuple('PrintParameters', ('active_fields', 'media_type', 'width_mm', 'length_mm', 'length_px', 'is_follow_up', 'sbz'))

class CompressionType(enum.IntEnum):
    none = 0
    rle = 2

class CommandSet(enum.IntEnum):
    escp = 0
    ptcbp = 1
    ptouch_template = 3

class PageMode(enum.IntFlag):
    auto_cut = 1 << 6
    mirror = 1 << 7

class PageModeAdvanced(enum.IntFlag):
    half_cut = 1 << 2
    no_page_chaining = 1 << 3
    no_cutting_on_special_tape = 1 << 4
    cut_on_last_label = 1 << 5
    high_resolution = 1 << 6
    preserve_buffer = 1 << 7

class MediaType(enum.IntEnum):
    unloaded = 0x00
    laminated = 0x01
    non_laminated = 0x03
    heat_shrink_tube = 0x11
    continuous_tape = 0x4a
    die_cut_labels = 0x4b
    unknown = 0xff

class PrintParameterField(enum.IntFlag):
    media_type = 1 << 1
    width = 1 << 2
    length = 1 << 3
    quality = 1 << 6
    recovery = 1 << 7

# TODO other enums
COMPRESSIONS = (
    ('none', lambda b: b, lambda b: b),
    None,
    ('rle', packbits.encode, packbits.decode),
)

COMPRESSIONS_TABLE = {c[0]: c[1:] for c in COMPRESSIONS if c is not None}

class Data(object):
    def __init__(self, data: bytes, compress: str='none', decompress: str='none') -> None:
        for c in (compress, decompress):
            if c not in COMPRESSIONS_TABLE:
                raise ValueError(f'Unknown compression type {c}')
        self.compress = compress
        self.data = COMPRESSIONS_TABLE[decompress][1](data)

    def getvalue(self) -> bytes:
        return COMPRESSIONS_TABLE[self.compress][0](self.data)

    def getvalue_raw(self) -> bytes:
        return self.data


class Opcode(object):
    def __init__(self, op: Optional[bytearray] = None,
                       op_mnemonic: Optional[str] = None,
                       params: Optional[Union[list, tuple, bytearray]]=None,
                       data: Optional[Data]=None,
                       paramschema: Optional[str]=None,) -> None:

        if op is None and op_mnemonic is None:
            raise ValueError('op and op_mnemonic cannot both be None')
        if op is None:
            self.op_mnemonic = op_mnemonic
        else:
            self.op = op
        if paramschema is None:
            op_bytes = bytes(self.op)
            self.paramschema = struct.Struct(f'<{OPS_FLAT[op_bytes][2]}') if op_bytes in OPS_FLAT and OPS_FLAT[op_bytes][2] is not None else None
        else:
            self.paramschema = struct.Struct(f'<{paramschema}') if paramschema is not None else None

        self.params = params
        self.data = data

    @property
    def op_mnemonic(self):
        return OPS_FLAT[bytes(self.op)][1] if bytes(self.op) in OPS_FLAT else None

    @op_mnemonic.setter
    def op_mnemonic(self, val):
        op = MNEMONICS[val][0] if val in MNEMONICS else None
        if op is None:
            raise ValueError(f'Unknown mnemonic {val}')
        self.op = op

    def serialize(self, to: BinaryIO) -> None:
        to.write(self.op)
        op_bytes = bytes(self.op)
        params = self.params
        d = None
        if self.data is not None:
            if self.paramschema is None or op_bytes not in OPS_FLAT:
                raise ValueError('Data attaching not supported')
            d = self.data.getvalue()
            # convert to list to allow writing and update length
            if params is None:
                params = []
            else:
                params = list(params)
            if len(params) < OPS_FLAT[op_bytes][3][2]:
                params.extend(None for _ in range(OPS_FLAT[op_bytes][3][2]))
            OPS_FLAT[op_bytes][3][1](params, len(d))

        if self.paramschema is not None:
            if params is not None:
                to.write(self.paramschema.pack(*params))
        # Raw arguments, useful for raw data
        elif params is not None:
            to.write(params)
        if d is not None:
            to.write(d)

    def serialize_as_bytes(self) -> bytes:
        buf = io.BytesIO()
        self.serialize(buf)
        return buf.getvalue()

    @classmethod
    def deserialize(cls, ptcbp_stream: BinaryIO, data_compress: str='none') -> object:
        op = bytearray()
        current_level = OPS
        while True:
            byte = ptcbp_stream.read(1)
            if len(byte) == 0:
                if len(op) == 0:
                    return None
                else:
                    raise IOError('Unexpected end of stream')
            byte = byte[0]
            if byte not in current_level:
                raise ValueError(f'Unknown byte 0x{byte:02x} at position {ptcbp_stream.tell():d}')
            op.append(byte)
            current_level = current_level[byte]
            if not isinstance(current_level, dict):
                break
        if current_level[2] is not None:
            schema = struct.Struct(current_level[2])
            params_raw = ptcbp_stream.read(schema.size)
            if len(params_raw) != schema.size:
                raise IOError('Unexpected end of stream')
            params = schema.unpack(params_raw)
        else:
            params = None
        if current_level[3] is not None:
            data_len = current_level[3][0](params)
            data_raw = ptcbp_stream.read(data_len)
            if len(data_raw) != data_len:
                raise IOError('Unexpected end of stream')
            data = Data(data_raw, compress=data_compress, decompress=data_compress)
        else:
            data = None
        return cls(op=op, params=params, data=data)

    @classmethod
    def deserialize_from_bytes(cls, ptcbp_bytes: bytes, data_compress: str='none') -> object:
        buf = io.BytesIO(ptcbp_bytes)
        return cls.deserialize(buf, data_compress)

# Simplified API
def serialize_control(mnemonic: str, *params) -> bytes:
    return Opcode(op_mnemonic=mnemonic, params=params or None).serialize_as_bytes()

def serialize_control_obj(mnemonic, params=None):
    return Opcode(op_mnemonic=mnemonic, params=params).serialize_as_bytes()

def serialize_data(data, compress='none', use_data2=False):
    if use_data2 and compress == 'none':
        # Some printers seem to use data2 to transfer uncompressed raster lines.
        mnemonic = 'data2'
    else:
        mnemonic = 'data'
    return Opcode(op_mnemonic=mnemonic, data=Data(data, compress=compress)).serialize_as_bytes()
