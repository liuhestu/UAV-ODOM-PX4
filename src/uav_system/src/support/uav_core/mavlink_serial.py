"""Fixed MAVLink v1 SERIAL_CONTROL wire encoder (common.xml CRC extra 220)."""
import struct


def encode(data, seq, system, component, flags=6):
    raw=data.encode('ascii')
    if len(raw)>70: raise ValueError('SERIAL_CONTROL data exceeds 70 bytes')
    payload=struct.pack('<IHBBB70s',0,0,10,flags,len(raw),raw.ljust(70,b'\0'))
    header=bytes([79,seq,system,component,126])
    crc=0xffff
    for b in header+payload+bytes([220]):
        tmp=b^(crc&255); tmp^=(tmp<<4)&255
        crc=((crc>>8)^(tmp<<8)^(tmp<<3)^(tmp>>4))&65535
    return payload,crc
