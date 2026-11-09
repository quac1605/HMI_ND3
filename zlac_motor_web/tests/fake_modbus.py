"""PTY-only ZLAC emulator for integration tests. Never opens a real serial device."""
import os
import pty
import select
import struct
import threading
import time


def crc(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0xa001 if value & 1 else value >> 1
    return struct.pack('<H', value)


class FakeModbus:
    def __init__(self):
        self.master,self.slave = pty.openpty()
        self.device = os.ttyname(self.slave)
        self.rpm = [0,0]
        self.counts = [0.,0.]
        self.history = []
        self.drop_feedback = False
        self.corrupt_ack = False
        self.done = threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.thread.start()

    def run(self):
        buf=b''; previous=time.monotonic()
        while not self.done.is_set():
            now=time.monotonic(); dt=now-previous; previous=now
            self.counts=[c+r/60*16384*dt for c,r in zip(self.counts,self.rpm)]
            if not select.select([self.master],[],[],.01)[0]: continue
            try: buf+=os.read(self.master,1024)
            except OSError: continue
            while len(buf)>=8:
                size=9+buf[6] if buf[1]==0x10 else 8
                if len(buf)<size: break
                f,buf=buf[:size],buf[size:]
                if crc(f[:-2])!=f[-2:]: continue
                reg=struct.unpack('>H',f[2:4])[0]
                if f[1]==0x06:
                    reply=f[:-2]
                elif f[1]==0x10:
                    self.rpm=list(struct.unpack('>hh',f[7:11]))
                    self.history.append((time.monotonic(),tuple(self.rpm)))
                    reply=f[:6]
                elif f[1]==0x03:
                    if self.drop_feedback: continue
                    if reg in (0x2030,0x2060): data=struct.pack('>H',4096)
                    else:
                        data=struct.pack('>IIhh',int(self.counts[0])&0xffffffff,
                                         int(self.counts[1])&0xffffffff,self.rpm[0]*10,self.rpm[1]*10)
                    reply=bytes([1,3,len(data)])+data
                else: continue
                checksum=crc(reply)
                if self.corrupt_ack and f[1] in (0x06,0x10): checksum=b'\x00\x00'
                os.write(self.master,reply+checksum)

    def close(self):
        self.done.set(); self.thread.join(1)
        os.close(self.master); os.close(self.slave)
