#!/usr/bin/env python3
"""
Extract imu660ra_config_file from SeekFree zf_device_config.lib.

The .lib file is an AR archive containing an ELF object file.
This script parses both formats to extract the 8192-byte IMU configuration data
and outputs it as a C array ready to be included in the CCS project.

Usage:
    python extract_config.py [path_to_zf_device_config.lib]

Output:
    user_driver/imu_config_data.h — C header with imu660ra_config_file[] array
"""

import struct
import sys
import os

# ========== AR Archive Parser ==========

def parse_ar(data, pos):
    """Parse an AR archive entry at `pos`. Returns (name, data, next_pos)."""
    if pos + 60 > len(data):
        return None, None, len(data)

    header = data[pos:pos+60]
    # Name: first 16 bytes, typically "name/        " or "#1/NN     " (BSD format)
    name_field = header[:16]
    size_field = header[48:58]  # decimal size

    try:
        size = int(size_field.rstrip())
    except ValueError:
        return None, None, len(data)

    pos += 60
    obj_data = data[pos:pos+size]
    pos += size
    if pos & 1:  # align to 2 bytes
        pos += 1

    # Parse name
    if name_field.startswith(b'#1/'):
        # BSD format: "#1/NN     " where NN is name length
        name_len = int(name_field[3:].rstrip())
        name = obj_data[:name_len].rstrip(b'\x00').decode('ascii', errors='replace')
        obj_data = obj_data[name_len:]  # strip name from data
    else:
        name = name_field.rstrip(b' /').decode('ascii', errors='replace')

    return name, obj_data, pos


# ========== ELF Parser (32-bit LE, ARM) ==========

def parse_elf(obj_data):
    """Parse 32-bit LE ELF object. Returns dict of sections and symbols."""
    if obj_data[:4] != b'\x7fELF':
        raise ValueError("Not an ELF file")

    ei_class = obj_data[4]  # 1=32-bit, 2=64-bit
    if ei_class != 1:
        raise ValueError("Only 32-bit ELF supported")

    # Parse ELF header
    e_shoff     = struct.unpack_from('<I', obj_data, 0x20)[0]  # section header offset
    e_shentsize = struct.unpack_from('<H', obj_data, 0x2E)[0]  # section header entry size
    e_shnum     = struct.unpack_from('<H', obj_data, 0x30)[0]  # section header count
    e_shstrndx  = struct.unpack_from('<H', obj_data, 0x32)[0]  # section name string table index

    # Read section headers
    sections = []
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        sh_name_idx = struct.unpack_from('<I', obj_data, off)[0]
        sh_type     = struct.unpack_from('<I', obj_data, off + 4)[0]
        sh_flags    = struct.unpack_from('<I', obj_data, off + 8)[0]
        sh_addr     = struct.unpack_from('<I', obj_data, off + 12)[0]
        sh_offset   = struct.unpack_from('<I', obj_data, off + 16)[0]
        sh_size     = struct.unpack_from('<I', obj_data, off + 20)[0]
        sh_link     = struct.unpack_from('<I', obj_data, off + 24)[0]
        sh_info     = struct.unpack_from('<I', obj_data, off + 28)[0]
        sections.append({
            'name_idx': sh_name_idx, 'type': sh_type, 'flags': sh_flags,
            'addr': sh_addr, 'offset': sh_offset, 'size': sh_size,
            'link': sh_link, 'info': sh_info,
        })

    # Read section name string table
    strtab = sections[e_shstrndx]
    str_data = obj_data[strtab['offset']:strtab['offset'] + strtab['size']]

    def section_name(idx):
        end = str_data.find(b'\x00', idx)
        return str_data[idx:end].decode('ascii', errors='replace')

    for s in sections:
        s['name'] = section_name(s['name_idx'])

    # Find symbol table and its string table
    symbols = []
    for s in sections:
        if s['type'] == 2:  # SHT_SYMTAB
            symtab_data = obj_data[s['offset']:s['offset'] + s['size']]
            symstr_sec = sections[s['link']]
            symstr_data = obj_data[symstr_sec['offset']:symstr_sec['offset'] + symstr_sec['size']]

            for i in range(0, len(symtab_data), 16):
                st_name  = struct.unpack_from('<I', symtab_data, i)[0]
                st_value = struct.unpack_from('<I', symtab_data, i + 4)[0]
                st_size  = struct.unpack_from('<I', symtab_data, i + 8)[0]
                st_info  = symtab_data[i + 12]
                st_shndx = struct.unpack_from('<H', symtab_data, i + 14)[0]

                sym_end = symstr_data.find(b'\x00', st_name)
                sym_name = symstr_data[st_name:sym_end].decode('ascii', errors='replace')
                symbols.append({
                    'name': sym_name, 'value': st_value, 'size': st_size,
                    'info': st_info, 'shndx': st_shndx,
                })

    return sections, symbols


# ========== Main ==========

def main():
    lib_path = sys.argv[1] if len(sys.argv) > 1 else (
        r"D:\BaiduNetdiskDownload\msp3507_for_imu\msp3507_for_imu\msp3507_imu660ra\libraries\zf_device\zf_device_config.lib"
    )

    if not os.path.exists(lib_path):
        print(f"ERROR: File not found: {lib_path}")
        sys.exit(1)

    with open(lib_path, 'rb') as f:
        data = f.read()

    print(f"Opened {lib_path} ({len(data)} bytes)")

    # Skip AR magic "!<arch>\n"
    if data[:8] != b'!<arch>\n':
        print("ERROR: Not an AR archive")
        sys.exit(1)

    pos = 8
    found_config = None

    while pos < len(data) - 60:
        name, obj_data, pos = parse_ar(data, pos)
        if name is None:
            break

        if '.o' not in name and 'zf_device_config' not in name:
            continue

        print(f"\nProcessing: {name} ({len(obj_data)} bytes)")

        try:
            sections, symbols = parse_elf(obj_data)
        except Exception as e:
            print(f"  ELF parse failed: {e}")
            continue

        # Print sections
        print(f"  Sections ({len(sections)}):")
        for s in sections:
            if s['size'] > 0:
                print(f"    {s['name']:20s} off=0x{s['offset']:06X} size={s['size']:6d} type={s['type']}")

        # Look for imu660ra_config_file
        for sym in symbols:
            if 'imu660ra_config_file' == sym['name']:
                print(f"\n  Found symbol: {sym['name']}")
                print(f"    value=0x{sym['value']:X} size={sym['size']} shndx={sym['shndx']}")

                # Get the section where this symbol lives
                target_sec = sections[sym['shndx']]
                data_offset = target_sec['offset'] + sym['value']
                config_data = obj_data[data_offset:data_offset + sym['size']]

                print(f"    Section: {target_sec['name']}")
                print(f"    Extracted {len(config_data)} bytes from offset 0x{data_offset:X}")

                found_config = config_data
                break

        if found_config:
            break

    if found_config is None:
        print("\nERROR: Could not find imu660ra_config_file symbol in .lib")
        print("Trying alternative: scanning for 8192-byte data sections...")

        # Fallback: look for any 8192-byte section
        pos = 8
        while pos < len(data) - 60:
            name, obj_data, pos = parse_ar(data, pos)
            if name and '.o' in name:
                try:
                    sections, symbols = parse_elf(obj_data)
                    for s in sections:
                        if s['size'] == 8192:
                            config_data = obj_data[s['offset']:s['offset'] + s['size']]
                            print(f"  Found 8192-byte section '{s['name']}' in {name}")
                            found_config = config_data
                            break
                except:
                    pass
            if found_config:
                break

    if found_config is None:
        print("\nERROR: Could not extract config data. Please compile the Keil project first,")
        print("        then use a debugger to read the imu660ra_config_file from flash.")
        sys.exit(1)

    # Generate output: C header file
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'user_driver')
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'imu_config_data.h')

    with open(output_path, 'w') as f:
        f.write("// Auto-generated by extract_config.py — DO NOT EDIT\n")
        f.write("// IMU660RA configuration data (DSP firmware)\n")
        f.write(f"// Source: {os.path.basename(lib_path)}\n")
        f.write(f"// Size: {len(found_config)} bytes\n\n")
        f.write(f"const unsigned char imu660ra_config_file[{len(found_config)}] = {{\n    ")
        for i, byte in enumerate(found_config):
            f.write(f"0x{byte:02X}, ")
            if (i + 1) % 16 == 0:
                f.write(f"\n    ")
        f.write("\n};\n")

    print(f"\nSuccess! Config data written to:")
    print(f"  {output_path}")
    print(f"\nNext steps:")
    print(f"  1. Copy {output_path} to your CCS project's user_driver/ folder")
    print(f"  2. In imu.c, change '#if 1' to '#if 0' to use the real config")


if __name__ == '__main__':
    main()
