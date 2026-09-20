#!/usr/bin/env python3
"""Check a simulator export against the benchmark schema and independent byte decoding."""
import argparse
import ast
import csv
import json
import math
from pathlib import Path
import struct

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('run', type=Path, help='Run directory, received CSV, or metadata file')
args = parser.parse_args()
chosen = args.run.resolve()
if chosen.is_dir():
    csv_path = chosen/'telemetry.csv'
    metadata_path = chosen/'metadata.txt'
elif chosen.suffix.lower() == '.csv':
    csv_path = chosen
    metadata_path = chosen.with_name(chosen.stem+'-metadata.txt')
    if chosen.name == 'telemetry.csv' and not metadata_path.exists(): metadata_path = chosen.parent/'metadata.txt'
else:
    metadata_path = chosen
    csv_path = chosen.parent/'telemetry.csv'
metadata = dict(line.split('=', 1) for line in metadata_path.read_text().splitlines() if '=' in line) if metadata_path.exists() else {}
csv_path = Path(metadata.get('csvFile', str(csv_path)))
raw_path = Path(metadata.get('packetFile', str(csv_path.parent/'packets.bin')))
tx_csv_path = Path(metadata.get('transmittedCsvFile', str(csv_path.parent/'transmitted-telemetry.csv')))
tx_raw_path = Path(metadata.get('transmittedPacketFile', str(csv_path.parent/'transmitted-packets.bin')))
args.run = csv_path.parent
def verify_pair(csv_name, raw_name, receiver=False):
    with (args.run / csv_name).open(newline='') as stream:
        reader = csv.DictReader(stream)
        header = reader.fieldnames
        rows = list(reader)
    assert header and len(header) == 43, 'Expected 43 columns'
    for benchmark in (Path(__file__).parent / 'benchmarking/telem').glob('*.csv'):
        with benchmark.open(newline='') as stream:
            assert next(csv.reader(stream)) == header, f'Header differs: {benchmark}'
    payloads = (args.run / raw_name).read_bytes()
    assert len(payloads) == len(rows)*128, 'Binary/CSV packet counts differ'
    def near(value, expected):
        assert math.isclose(float(value), expected, rel_tol=1e-6, abs_tol=1e-6), (value, expected)

    states = ['GROUND_TESTING', 'PRE_FLIGHT', 'FLIGHT', 'APOGEE', 'MAIN', 'END']
    for i, row in enumerate(rows):
        assert None not in row and len(row) == 43, f'Malformed CSV row {i}'
        packet = payloads[i*128:(i+1)*128]
        assert sum(packet[:127]) & 255 == packet[127], f'Checksum failure at {i}'
        u32 = lambda offset: struct.unpack_from('<I', packet, offset)[0]
        f32 = lambda offset: struct.unpack_from('<f', packet, offset)[0]
        assert int(row['flight_time']) == u32(80)
        assert int(row['pktnum']) == struct.unpack_from('<H', packet, 84)[0]
        assert row['state'] == 'state.' + states[packet[63]]
        near(row['lat'], struct.unpack_from('<i', packet, 32)[0]*1e-7)
        near(row['lon'], struct.unpack_from('<i', packet, 36)[0]*1e-7)
        for key, offset in [('gpsalt', 40), ('barofilteredalt', 59), ('roll_gyro_int', 64),
                            ('pitch_gyro_int', 68), ('yaw_gyro_int', 72), ('accel_integrated_velo', 122)]:
            near(row[key], f32(offset))
        bits = int.from_bytes(packet[10:16], 'little')
        assert ast.literal_eval(row['servos']) == [(bits >> (12*j)) & 4095 for j in range(4)]
        for j, value in enumerate(ast.literal_eval(row['accelerometer'])):
            near(value, int.from_bytes(packet[16+3*j:19+3*j], 'little', signed=True)/12800*9.80665)
        for j, value in enumerate(ast.literal_eval(row['gyro'])):
            near(value, struct.unpack_from('<h', packet, 25+2*j)[0]*0.03051757812*(-1 if j == 1 else 1))
        raw_temp = int.from_bytes(packet[56:59], 'little')
        near(row['temp'], (2000+(raw_temp-0x91E3*256)*0x6FEC/(1 << 23))/100)
        for key in ['rssi', 'rxrssi', 'gnd_lat', 'gnd_lon', 'gnd_fix', 'gnd_alt']:
            assert row[key] == '', f'Unmodeled field should be blank: {key}'
        if i:
            elapsed = (int(row['flight_time'])-int(rows[i-1]['flight_time'])) & 0xffffffff
            steps = elapsed // 60
            assert elapsed >= 60 and elapsed % 60 == 0
            if not receiver: assert steps == 1
            assert (int(row['pktnum'])-int(rows[i-1]['pktnum'])) & 0xffff == steps % 65536
            near(float(row['timestamp'])-float(rows[i-1]['timestamp']), steps * 0.06)

    return rows, payloads

has_transmitted = tx_raw_path.exists()
rows, payloads = verify_pair(csv_path, raw_path, receiver=has_transmitted)
if has_transmitted:
    tx_rows, tx_bytes = verify_pair(tx_csv_path, tx_raw_path)
    assert int(metadata['generated']) == len(tx_rows)
    assert int(metadata['received']) == len(rows)
    assert len(tx_rows) == len(rows) + int(metadata['dropped']) + int(metadata['pending'])
    delay = int(metadata['downlinkDelayMs'])/1000
    j = 0
    for i, row in enumerate(rows):
        packet = payloads[i*128:(i+1)*128]
        while j < len(tx_rows) and tx_bytes[j*128:(j+1)*128] != packet: j += 1
        assert j < len(tx_rows), 'Received packet missing from transmitted stream'
        assert math.isclose(float(row['timestamp'])-float(tx_rows[j]['timestamp']), delay, abs_tol=1e-6)
        j += 1
else:
    assert rows, 'Legacy export unexpectedly has no packets'
print(json.dumps({'result': 'PASS', 'csv': str(csv_path),
                  'columns': 43, 'packets': len(rows), 'payload_bytes': len(payloads),
                  'transmitted_packets': len(tx_rows) if has_transmitted else len(rows),
                  'first_boot_ms': int(rows[0]['flight_time']) if rows else None,
                  'last_boot_ms': int(rows[-1]['flight_time']) if rows else None,
                  'states': sorted(set(row['state'] for row in rows))}, indent=2))
