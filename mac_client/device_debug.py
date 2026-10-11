"""HarnessNode 0.0.112+: acknowledged, connection-owned diagnostic sessions."""
import asyncio
import struct
from contextlib import asynccontextmanager

RX = '00000003-0000-1000-8000-00805f9b34fb'

class DebugRejected(RuntimeError):
    pass

class DebugControl:
    def __init__(self):
        self.responses = asyncio.Queue()
        self.lock = asyncio.Lock()

    def wrap(self, callback):
        def notify(sender, packet):
            p = bytes(packet)
            if len(p) == 8 and p[:3] == b'\x00\x55\xd8':
                self.responses.put_nowait(p)
            callback(sender, packet)
        return notify

    async def mode(self, client, op):
        async with self.lock:
            while not self.responses.empty(): self.responses.get_nowait()
            await client.write_gatt_char(RX, bytes([0x55, op]), response=True)
            while True:
                p = await asyncio.wait_for(self.responses.get(), timeout=3)
                if p[3] != op: continue
                if p[4]: raise DebugRejected(f'Debug mode rejected: status={p[4]}')
                result = dict(mode='debug' if p[5] else 'normal',
                              remaining_seconds=struct.unpack_from('<H', p, 6)[0])
                if op == 1 and p[5] != 1: raise RuntimeError('Debug mode was not enabled')
                if op == 2 and p[5] != 0: raise RuntimeError('Debug mode was not disabled')
                return result

    @asynccontextmanager
    async def session(self, client):
        # Also attempt cleanup when the enable ACK is lost: the device may have
        # accepted it. Disconnect and the ten-minute device deadline are backups.
        cleanup = True
        try:
            try:
                await self.mode(client, 1)
            except DebugRejected:
                cleanup = False  # Firmware explicitly refused ownership.
                raise
            yield self
        finally:
            if cleanup and client.is_connected:
                try: await self.mode(client, 2)
                except Exception as exc:
                    print(f'Debug cleanup failed; owner disconnect/timeout will restore normal mode: {exc}', flush=True)
