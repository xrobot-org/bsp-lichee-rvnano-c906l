# Application protocol tests

Run on a Linux host from the BSP root:

```sh
cmake -S User/tests -B /tmp/sg200x-app-tests -G Ninja \
  -DCMAKE_C_FLAGS="-O2 -Wall -Wextra -Werror" \
  -DCMAKE_CXX_FLAGS="-O2 -Wall -Wextra -Werror"
cmake --build /tmp/sg200x-app-tests
ctest --test-dir /tmp/sg200x-app-tests --output-on-failure
```

The camera test writes the Linux bridge's version 1 wire layout directly into
anonymous memory at `0x8FFDE000`. It verifies frame metadata, a known CRC-32
check vector, empty/ready ownership, rejection of corrupt headers and payloads,
and release after errors. A guard page after the region catches reads beyond
the shared mailbox. Output frame views must remain unchanged on rejection.

The sender protocol in `tools/sg2002-camera-bridge/bridge.c` remains unchanged:
64-byte header, payload at offset 64, empty = 0, ready = 1. Physical cross-core
cache coherence and camera streaming still require a board run.
