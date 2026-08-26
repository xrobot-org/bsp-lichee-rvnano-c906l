import mmap
import struct

BASE = 0x8FFFF000
with open('/dev/mem', 'rb', buffering=0) as mem:
    page = mmap.mmap(mem.fileno(), 0x1000, access=mmap.ACCESS_READ, offset=BASE)
    words = [struct.unpack_from('I', page, offset)[0] for offset in range(0, 132, 4)]
    print(' '.join(f'{word:08x}' for word in words))
