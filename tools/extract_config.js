/**
 * Extract imu660ra_config_file from SeekFree zf_device_config.lib
 * Parses GNU AR archive → ELF object → finds symbol → extracts data → writes C header
 *
 * Usage: node extract_config.js [path_to_lib]
 */

const fs = require('fs');
const path = require('path');

const LIB_PATH = process.argv[2] ||
    "D:/BaiduNetdiskDownload/msp3507_for_imu/msp3507_for_imu/msp3507_imu660ra/libraries/zf_device/zf_device_config.lib";

const data = fs.readFileSync(LIB_PATH);
console.log(`Opened ${LIB_PATH} (${data.length} bytes)`);

// ── Parse GNU AR Archive ──
const entries = [];
const longNames = {};
let pos = 8; // skip "!<arch>\n"

while (pos + 60 <= data.length) {
    const hdr = data.subarray(pos, pos + 60);
    const nameRaw = hdr.subarray(0, 16).toString('ascii').trim();
    const sizeStr = hdr.subarray(48, 58).toString('ascii').trim();
    const size = parseInt(sizeStr, 10);
    if (isNaN(size)) break;

    pos += 60;
    const entryData = data.subarray(pos, pos + size);
    pos += size;
    if (pos & 1) pos++; // align to 2 bytes

    // Resolve GNU long filename: "/N" → offset N in "//" table
    let name = nameRaw;
    if (nameRaw.startsWith('/') && nameRaw.length > 1 && !isNaN(nameRaw.substring(1))) {
        const off = parseInt(nameRaw.substring(1), 10);
        name = longNames[off] || nameRaw;
    }

    entries.push({ name, size, data: entryData });

    // Store long filename table
    if (nameRaw === '//') {
        const text = entryData.toString('ascii');
        let start = 0;
        let idx = 0;
        while (start < text.length) {
            const end = text.indexOf('/\n', start);
            if (end < 0) { longNames[idx] = text.substring(start).trim(); break; }
            longNames[idx] = text.substring(start, end).trim();
            idx = end + 2 - start; // doesn't matter, just keeping it simple
            start = end + 2;
        }
        // Simpler: split by "/\n"
        const parts = text.split('/\n');
        let off = 0;
        for (const p of parts) {
            if (p) { longNames[off] = p; off += p.length + 2; }
        }
    }
}

console.log(`Found ${entries.length} archive entries`);
for (const e of entries) console.log(`  "${e.name}" (${e.size} bytes)`);

// ── Find the .o file ──
let elfBuf = null;
for (const e of entries) {
    if (e.name.endsWith('.o') && e.data[0] === 0x7f) {
        console.log(`\nUsing: ${e.name}`);
        elfBuf = e.data;
        break;
    }
}
if (!elfBuf) {
    console.error('ERROR: No ELF object found');
    process.exit(1);
}

// ── Parse ELF (32-bit LE) ──
const e_shoff     = elfBuf.readUInt32LE(0x20);
const e_shentsize = elfBuf.readUInt16LE(0x2E);
const e_shnum     = elfBuf.readUInt16LE(0x30);
const e_shstrndx  = elfBuf.readUInt16LE(0x32);

// Read section headers
const sections = [];
for (let i = 0; i < e_shnum; i++) {
    const off = e_shoff + i * e_shentsize;
    sections.push({
        name_idx:  elfBuf.readUInt32LE(off),
        type:      elfBuf.readUInt32LE(off + 4),
        flags:     elfBuf.readUInt32LE(off + 8),
        addr:      elfBuf.readUInt32LE(off + 12),
        offset:    elfBuf.readUInt32LE(off + 16),
        size:      elfBuf.readUInt32LE(off + 20),
        link:      elfBuf.readUInt32LE(off + 24),
    });
}

const strtab = sections[e_shstrndx];
const strBuf = elfBuf.subarray(strtab.offset, strtab.offset + strtab.size);
const secName = (idx) => {
    const end = strBuf.indexOf(0, idx);
    return strBuf.toString('ascii', idx, end < 0 ? strBuf.length : end);
};
for (const s of sections) s.name = secName(s.name_idx);

console.log(`Sections:`);
for (const s of sections) {
    if (s.size) console.log(`  ${s.name.padEnd(22)} off=0x${s.offset.toString(16).padStart(6,'0')} size=${s.size.toString().padEnd(6)}`);
}

// ── Find imu660ra_config_file symbol ──
let found = null;
for (const s of sections) {
    if (s.type !== 2) continue; // SHT_SYMTAB = 2
    const symBuf = elfBuf.subarray(s.offset, s.offset + s.size);
    const symStrSec = sections[s.link];
    const symStrBuf = elfBuf.subarray(symStrSec.offset, symStrSec.offset + symStrSec.size);

    console.log(`\nSymbols (from ${s.name}):`);
    for (let i = 0; i < symBuf.length; i += 16) {
        const st_name  = symBuf.readUInt32LE(i);
        const st_value = symBuf.readUInt32LE(i + 4);
        const st_size  = symBuf.readUInt32LE(i + 8);
        const st_shndx = symBuf.readUInt16LE(i + 14);

        const end = symStrBuf.indexOf(0, st_name);
        const symName = symStrBuf.toString('ascii', st_name, end < 0 ? symStrBuf.length : end);

        if (st_size > 100) console.log(`  ${symName.padEnd(30)} value=0x${st_value.toString(16)} size=${st_size} shndx=${st_shndx}`);

        if (symName === 'imu660ra_config_file') {
            const tSec = sections[st_shndx];
            const dOff = tSec.offset + st_value;
            found = elfBuf.subarray(dOff, dOff + st_size);
            console.log(`\n  >>> FOUND! section=${tSec.name} offset=0x${dOff.toString(16)} size=${st_size}`);
        }
    }
}

if (!found) {
    console.error('\nERROR: imu660ra_config_file not found');
    process.exit(1);
}

// ── Write C header ──
const outputDir = path.join(__dirname, '..', 'user_driver');
fs.mkdirSync(outputDir, { recursive: true });
const outputPath = path.join(outputDir, 'imu_config_data.h');

let cSource = `// Auto-generated — DO NOT EDIT
// IMU660RA DSP firmware config (${found.length} bytes)

const unsigned char imu660ra_config_file[${found.length}] = {
`;
for (let i = 0; i < found.length; i++) {
    if (i % 16 === 0) cSource += '    ';
    cSource += `0x${found[i].toString(16).toUpperCase().padStart(2, '0')}, `;
    if ((i + 1) % 16 === 0) cSource += '\n';
}
cSource += '\n};\n';

fs.writeFileSync(outputPath, cSource);
console.log(`\nDone! Wrote ${outputPath}`);
console.log(`Now edit imu.c: change '#if 1' to '#if 0'`);
