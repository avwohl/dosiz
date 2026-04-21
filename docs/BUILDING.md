# Building dosemu

## Prerequisites

- C++20 compiler (GCC 12+ or Clang 15+)
- CMake 3.24+
- git (with submodule support)
- SDL2 development headers (`libsdl2-dev` on Debian/Ubuntu)
- dosbox-staging build dependencies: libpng-dev, zlib1g-dev, libopusfile-dev,
  libsdl2-net-dev, libspeexdsp-dev (see dosbox-staging README for the full
  list — changes by upstream version)

## Clone

```sh
git clone --recurse-submodules https://github.com/avwohl/dosemu.git
cd dosemu
```

If you forgot `--recurse-submodules`:

```sh
git submodule update --init --recursive
```

## Build

```sh
cmake -S src -B build
cmake --build build -j$(nproc)
```

Or via the top-level Makefile:

```sh
make
```

The resulting binary lives at `build/dosemu`.

## Run tests

```sh
make test
```

## Phase status

- **Phase 0 (current)**: scaffolding only. `dosemu --help` runs; no DOS program
  execution yet.
- **Phase 1+**: see `../docs/ROADMAP.md` once it lands.
