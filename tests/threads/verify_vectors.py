#!/usr/bin/env python3
"""Check interrupt vector ownership in the linked firmware, not just compilation."""
import pathlib
import struct
import subprocess
import sys
import tempfile

elf = pathlib.Path(sys.argv[1])
prefix = sys.argv[2] if len(sys.argv) > 2 else 'arm-none-eabi-'
expected = {
    14: 'PendSV_Handler', 15: 'SysTick_Handler', 70: 'TIM6_DAC_IRQHandler',
    36: 'CAN1_RX0_IRQHandler', 80: 'CAN2_RX0_IRQHandler',
    53: 'USART1_IRQHandler', 87: 'USART6_IRQHandler',
    73: 'DMA2_Stream1_IRQHandler', 74: 'DMA2_Stream2_IRQHandler',
    85: 'DMA2_Stream6_IRQHandler', 86: 'DMA2_Stream7_IRQHandler',
}
symbols = {}
for line in subprocess.check_output([prefix+'nm', '-n', str(elf)], text=True).splitlines():
    parts = line.split()
    if len(parts) == 3:
        symbols[parts[2]] = int(parts[0], 16)
with tempfile.TemporaryDirectory() as tmp:
    binary = pathlib.Path(tmp)/'vectors.bin'
    subprocess.run([prefix+'objcopy', '-O', 'binary', '--only-section=.isr_vector', str(elf), str(binary)], check=True)
    raw = binary.read_bytes()
    for index, name in expected.items():
        actual = struct.unpack_from('<I', raw, index*4)[0]
        assert name in symbols and actual & ~1 == symbols[name] & ~1, (name, hex(actual))
assert not subprocess.check_output([prefix+'nm', '-u', str(elf)], text=True).strip(), 'Unresolved symbols'
print(f'PASS: {len(expected)} vectors and no unresolved symbols: {elf}')
