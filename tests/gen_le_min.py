#!/usr/bin/env python3
# Emits tests/LE_MIN.EXE: a minimal Linear Executable test fixture.
#
# Shape:
#   Object 1 (code, BIG=32-bit): 8 bytes, contains a `mov eax, imm32`
#       whose imm32 is patched by a type-7 fixup to point at obj 2 + 0.
#   Object 2 (data): 8 bytes, contains "LE-DATA\0".
#
# One 4K physical data page holds both objects packed at the start.
# One fixup record: type 0x07 (32-bit offset) internal ref, source at
# page1+0x01 (imm32 field of the `B8` opcode), target = obj 2 offset 0.
#
# After the loader applies fixups, the 32-bit word at obj1 host seg
# + 1 should equal host_seg[obj2] * 16 + 0.
import os, struct, sys

OUT = os.path.join(os.path.dirname(__file__), "LE_MIN.EXE")

# ---- MZ stub ---------------------------------------------------------------
MZ_SIZE = 0x80                       # padded so LE header starts at 0x80
mz = bytearray(MZ_SIZE)
mz[0:2] = b"MZ"
# We don't run the MZ stub, but set a plausible page/reloc count so
# generic MZ readers don't choke.  lfanew (offset 0x3C) is the only
# field dosemu actually uses here.
struct.pack_into("<H", mz, 0x02, 0)       # bytes-in-last-page = 0 -> exactly pages*512
struct.pack_into("<H", mz, 0x04, 1)       # pages (512-byte) = 1
struct.pack_into("<H", mz, 0x06, 0)       # reloc count
struct.pack_into("<H", mz, 0x08, 4)       # header paragraphs (64 bytes)
struct.pack_into("<H", mz, 0x18, 0x40)    # reloc table offset (empty, past header)
struct.pack_into("<I", mz, 0x3C, MZ_SIZE) # lfanew -> LE header at 0x80

# ---- LE header (0xB0 bytes) ------------------------------------------------
LE_HDR_SIZE  = 0xB0
OBJ_ENTRY    = 24
NUM_OBJECTS  = 2
PAGE_SIZE    = 4096
NUM_PAGES    = 1

# Layout after the LE header, all relative to le_off:
obj_tbl_off      = LE_HDR_SIZE                             # 0xB0
page_tbl_off     = obj_tbl_off + NUM_OBJECTS * OBJ_ENTRY   # 0xE0
fixup_page_off   = page_tbl_off + NUM_PAGES * 4            # 0xE4
fixup_record_off = fixup_page_off + (NUM_PAGES + 1) * 4    # 0xEC

# Fixup record for page 1 (the only page).  One internal type-7 fixup:
#   byte source type  = 0x07  (32-bit offset)
#   byte target flags = 0x00  (internal ref, 8-bit obj num, 16-bit target off)
#   word source off   = 0x0001  (imm32 of B8 at page+0)
#   byte target obj   = 0x02
#   word target off   = 0x0000
fixup_record = bytes([0x07, 0x00]) + struct.pack("<H", 0x0001) \
             + bytes([0x02]) + struct.pack("<H", 0x0000)

# Fixup page table: (num_pages + 1) entries of dword offsets into the
# fixup record table.  Page 1's fixups occupy [0, len(fixup_record));
# the sentinel entry marks the end.
fixup_page_table = struct.pack("<II", 0, len(fixup_record))

# Everything before data_pages_off is before the one 4K data page.
# Pack data_pages to start right after the fixup record table (no need
# to align on disk for our loader; page_size only affects per-page
# copy granularity).
le_hdr_region_size = (LE_HDR_SIZE + NUM_OBJECTS*OBJ_ENTRY + NUM_PAGES*4
                      + (NUM_PAGES+1)*4 + len(fixup_record))
data_pages_file_off = MZ_SIZE + le_hdr_region_size

le = bytearray(LE_HDR_SIZE)
le[0:2]  = b"LE"
le[0x02] = 0                              # byte order = little
le[0x03] = 0                              # word order = little
struct.pack_into("<I", le, 0x04, 0)       # format level
struct.pack_into("<H", le, 0x08, 3)       # cpu_type: 80386
struct.pack_into("<H", le, 0x0A, 1)       # os_type: OS/2 (value doesn't matter)
struct.pack_into("<I", le, 0x0C, 0)       # module version
struct.pack_into("<I", le, 0x10, 0x0000)  # module flags
struct.pack_into("<I", le, 0x14, NUM_PAGES)      # num pages
struct.pack_into("<I", le, 0x18, 1)              # entry obj (1-based)
struct.pack_into("<I", le, 0x1C, 0)              # entry EIP
struct.pack_into("<I", le, 0x20, 2)              # stack obj
struct.pack_into("<I", le, 0x24, 0x1000)         # stack ESP
struct.pack_into("<I", le, 0x28, PAGE_SIZE)      # page size
# last_page_sz: how many bytes of the last page are valid.  With both
# objects packed inside one page, the data extends only to the end of
# obj2 (16 bytes from page start).  But the loader also uses this to
# clip the final per-page copy; setting it to 16 means we only copy
# 16 bytes per page — which is fine since the objects are 8 bytes each
# and the page's "meaningful" content ends there.
struct.pack_into("<I", le, 0x2C, 16)             # last page size
struct.pack_into("<I", le, 0x30, len(fixup_page_table) + len(fixup_record))  # fixup section size
struct.pack_into("<I", le, 0x34, 0)              # fixup checksum
struct.pack_into("<I", le, 0x38, 0)              # loader section size
struct.pack_into("<I", le, 0x3C, 0)              # loader checksum
struct.pack_into("<I", le, 0x40, obj_tbl_off)
struct.pack_into("<I", le, 0x44, NUM_OBJECTS)
struct.pack_into("<I", le, 0x48, page_tbl_off)
struct.pack_into("<I", le, 0x4C, 0)              # iter pages off (unused)
struct.pack_into("<I", le, 0x50, 0)              # resource table
struct.pack_into("<I", le, 0x54, 0)              # num resources
struct.pack_into("<I", le, 0x58, 0)              # resident name table
struct.pack_into("<I", le, 0x5C, 0)              # entry table
struct.pack_into("<I", le, 0x60, 0)              # module directives
struct.pack_into("<I", le, 0x64, 0)              # num module directives
struct.pack_into("<I", le, 0x68, fixup_page_off)
struct.pack_into("<I", le, 0x6C, fixup_record_off)
struct.pack_into("<I", le, 0x70, 0)              # imported modules
struct.pack_into("<I", le, 0x74, 0)              # num imported modules
struct.pack_into("<I", le, 0x78, 0)              # import proc names
struct.pack_into("<I", le, 0x7C, 0)              # per-page checksum
struct.pack_into("<I", le, 0x80, data_pages_file_off)  # data pages (absolute file offset)
struct.pack_into("<I", le, 0x84, 0)              # preload pages
struct.pack_into("<I", le, 0x88, 0)              # non-resident name tbl
struct.pack_into("<I", le, 0x8C, 0)              # non-resident name tbl len
struct.pack_into("<I", le, 0x90, 0)              # non-resident name tbl checksum
struct.pack_into("<I", le, 0x94, 2)              # auto data obj (1-based)
struct.pack_into("<I", le, 0x98, 0)              # debug info off
struct.pack_into("<I", le, 0x9C, 0)              # debug info len
struct.pack_into("<I", le, 0xA0, 0)              # instance preload
struct.pack_into("<I", le, 0xA4, 0)              # instance demand
struct.pack_into("<I", le, 0xA8, 0)              # heap size

# ---- Object table ----------------------------------------------------------
# Flags bits: 0x01=R 0x02=W 0x04=X 0x40=discardable 0x4000=BIG
def obj_entry(virt_size, virt_base, flags, first_page, page_count):
    return struct.pack("<IIIII", virt_size, virt_base, flags,
                        first_page, page_count) + b"\x00\x00\x00\x00"

objs = obj_entry(0x10, 0x10000, 0x0005 | 0x4000, 1, 1)  # obj1 code+read, BIG, page 1
objs += obj_entry(0x10, 0x20000, 0x0003, 1, 1)           # obj2 data R+W, page 1 (shared)

# ---- Object page map table -------------------------------------------------
# LE entry: 3-byte big-endian page number + 1-byte type.
# Only 1 physical page, type=0 (legal).
page_map = bytes([0x00, 0x00, 0x01, 0x00])

# ---- The 4K data page ------------------------------------------------------
page = bytearray(PAGE_SIZE)
# Obj 1 code at page offset 0 (32-bit; entry_eip = 0):
#   B8 XX XX XX XX       mov eax, imm32   (imm32 patched by type-7 fixup
#                                          to point at obj 2 + 0)
#   B4 4C                mov ah, 4Ch      (AH=4Ch = DOS terminate-with-code)
#   CD 21                int 21h           (PM INT 21h handler runs, AL=0
#                                          so exit code 0)
#   90 90                nop nop
page[0] = 0xB8
page[1:5] = b"\xDE\xAD\xBE\xEF"   # placeholder imm32; fixup should overwrite
page[5:7] = b"\xB4\x4C"
page[7] = 0xCD
page[8] = 0x21
page[9:11] = b"\x90\x90"
# Obj 2 data overlaps starting at page offset 11 in the same physical
# page -- fine, since both objects refer to page 1.  Their virt_bases
# differ (0x10000 vs 0x20000), so fixup targets resolve based on
# host_base.  We keep DATA_PAGE_BYTES at 16 so the copy-out writes
# past any meaningful code.
page[11:16] = b"LE-DT"
# Truncate per last_page_sz = 16 bytes on the wire (the loader clips the
# copy to 16 bytes; we emit only 16 bytes on disk for this page).
DATA_PAGE_BYTES = 16

# ---- Assemble file ---------------------------------------------------------
body = (bytes(le) + objs + page_map + fixup_page_table + fixup_record
        + bytes(page[:DATA_PAGE_BYTES]))
out = bytes(mz) + body

with open(OUT, "wb") as fp:
    fp.write(out)

sys.stderr.write(f"wrote {OUT} ({len(out)} bytes)\n")
