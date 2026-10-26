import asyncio
import unittest
from device_debug import DebugControl

class Client:
    is_connected = True
    def __init__(self, debug, fail_enable=False):
        self.debug = debug; self.commands = []; self.fail_enable = fail_enable
    async def write_gatt_char(self, uuid, command, response):
        self.commands.append(bytes(command))
        op = command[1]
        mode = int(op == 1)
        status = 3 if self.fail_enable and op == 1 else 0
        self.debug.wrap(lambda *_: None)(None, bytes([0, 0x55, 0xD8, op, status, mode, 0x58 if mode else 0, 2 if mode else 0]))

class DebugTest(unittest.IsolatedAsyncioTestCase):
    async def test_cleanup_success_and_error(self):
        for raises in (False, True):
            d = DebugControl(); c = Client(d)
            try:
                async with d.session(c):
                    if raises: raise ValueError('failed capture')
            except ValueError: pass
            self.assertEqual(c.commands, [b'\x55\x01', b'\x55\x02'])
    async def test_rejected_enable_never_enters(self):
        d = DebugControl(); c = Client(d, True)
        with self.assertRaises(RuntimeError):
            async with d.session(c): self.fail('entered without enable ACK')
        self.assertEqual(c.commands, [b'\x55\x01'])
    async def test_enable_ack_failure_cleanup(self):
        d = DebugControl(); c = Client(d)
        original = d.mode
        async def mode(client, op):
            if op == 1:
                c.commands.append(bytes([0x55, op]))
                raise asyncio.TimeoutError('lost ACK')
            return await original(client, op)
        d.mode = mode
        with self.assertRaises(asyncio.TimeoutError):
            async with d.session(c): self.fail('entered without ACK')
        self.assertEqual(c.commands, [b'\x55\x01', b'\x55\x02'])

    async def test_disconnect_cleanup_backup(self):
        d = DebugControl(); c = Client(d)
        async with d.session(c): c.is_connected = False
        self.assertEqual(c.commands, [b'\x55\x01'])
    async def test_query_and_forwarding(self):
        d = DebugControl(); c = Client(d); forwarded = []
        d.wrap(lambda _, p: forwarded.append(bytes(p)))(None, b'\x00\x55\x14')
        self.assertEqual(forwarded, [b'\x00\x55\x14'])
        self.assertEqual(await d.mode(c, 0), {'mode': 'normal', 'remaining_seconds': 0})

if __name__ == '__main__': unittest.main()
