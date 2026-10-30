"""Offline PCAP timing for this SDK's little-endian framing; no SDK/device I/O.

Framing verified against libaubo_sdk.so.2.5.0 a04ad0aa...:
SOF ee cc ff ff, uint32 body length, uint32 command, payload, CRC, END ff ff ff ff.
Command 7 is RealTimeRobotJointAngle in the bundled communicationmatetype.h.
Packet capture time is host arrival, never controller sampling time.
"""
import argparse
from collections import Counter, deque
import json
from pathlib import Path
import socket
import struct

SOF = b'\xee\xcc\xff\xff'
END = b'\xff'*4
MAX_FRAME = 2*1024*1024


class FrameStream:
    def __init__(self):
        self.next_seq = None
        self.waiting = {}
        self.buffer = bytearray()
        self.times = deque()
        self.stats = Counter()

    def _drop(self, count):
        del self.buffer[:count]
        stamps = []
        while count and self.times:
            n, stamp = self.times.popleft()
            used = min(n, count)
            stamps.append(stamp)
            count -= used
            if n > used:
                self.times.appendleft((n-used, stamp))
        return stamps

    def feed(self, seq, payload, stamp):
        if not payload:
            return []
        if self.next_seq is None:
            self.next_seq = seq
        # Unwrap sequence numbers around the current contiguous frontier.
        seq = self.next_seq + ((seq-self.next_seq+2**31) % 2**32-2**31)
        if seq < self.next_seq:
            self.stats['overlapping_segments'] += 1
            payload = payload[self.next_seq-seq:]
            seq = self.next_seq
        if not payload:
            return []
        if seq > self.next_seq:
            self.stats['out_of_order_segments'] += 1
            old = self.waiting.get(seq)
            if old is None or len(payload) > len(old[0]):
                self.waiting[seq] = (payload, stamp)
            if sum(len(v[0]) for v in self.waiting.values()) > MAX_FRAME:
                self.stats['reassembly_overflow'] += 1
                self.waiting.clear()
                self.buffer.clear()
                self.times.clear()
                self.next_seq = None
            return []
        self.buffer.extend(payload)
        self.times.append((len(payload), stamp))
        self.next_seq += len(payload)
        while self.waiting:
            first = min(self.waiting)
            if first > self.next_seq:
                break
            data, received = self.waiting.pop(first)
            data = data[max(0, self.next_seq-first):]
            if data:
                self.buffer.extend(data)
                self.times.append((len(data), received))
                self.next_seq += len(data)
        rows = []
        while len(self.buffer) >= 8:
            offset = self.buffer.find(SOF)
            if offset < 0:
                self.stats['resync_bytes'] += len(self.buffer)-3
                self._drop(len(self.buffer)-3)
                break
            if offset:
                self.stats['resync_bytes'] += offset
                self._drop(offset)
            if len(self.buffer) < 8:
                break
            length = struct.unpack_from('<I', self.buffer, 4)[0]
            if not 12 <= length <= MAX_FRAME:
                self.stats['invalid_lengths'] += 1
                self._drop(1)
                continue
            if len(self.buffer) < length+8:
                break
            if self.buffer[length+4:length+8] != END:
                self.stats['invalid_footers'] += 1
                self._drop(1)
                continue
            command = struct.unpack_from('<I', self.buffer, 8)[0]
            stamps = self._drop(length+8)
            rows.append(dict(command=command, frame_bytes=length+8,
                             first_arrival_wall_s=min(stamps),
                             complete_arrival_wall_s=max(stamps),
                             tcp_deliverable_wall_s=max(stamp, max(stamps))))
            self.stats['frames'] += 1
        return rows


def packets(path):
    """Classic tcpdump PCAP, Ethernet/IPv4/TCP. Never silently parse pcapng."""
    with Path(path).open('rb') as f:
        header = f.read(24)
        modes = {b'\xd4\xc3\xb2\xa1':('<',1e6), b'\xa1\xb2\xc3\xd4':('>',1e6),
                 b'\x4d\x3c\xb2\xa1':('<',1e9), b'\xa1\xb2\x3c\x4d':('>',1e9)}
        if len(header) != 24 or header[:4] not in modes:
            raise ValueError('Expected a complete classic PCAP header')
        endian, scale = modes[header[:4]]
        linktype = struct.unpack_from(endian+'I', header, 20)[0]
        if linktype != 1:
            raise ValueError('Expected Ethernet capture (use -i enp3s0, not -i any)')
        while True:
            record = f.read(16)
            if not record:
                break
            if len(record) != 16:
                raise ValueError('Truncated PCAP record header; stop capture before analysis')
            sec, frac, size, original = struct.unpack(endian+'IIII', record)
            if size > 4*1024*1024:
                raise ValueError('Unexpected PCAP record size')
            data = f.read(size)
            if len(data) != size or size != original:
                raise ValueError('Truncated packet; use -s 0')
            ip = 14
            if len(data) < ip:
                continue
            ethertype = struct.unpack_from('!H', data, 12)[0]
            while ethertype in (0x8100, 0x88a8) and len(data) >= ip+4:
                ethertype = struct.unpack_from('!H', data, ip+2)[0]
                ip += 4
            if ethertype != 0x0800 or len(data) < ip+20 or data[ip+9] != 6:
                continue
            if struct.unpack_from('!H', data, ip+6)[0] & 0x3fff:
                raise ValueError('Fragmented IPv4 is not supported by this analyzer')
            ihl = (data[ip] & 15)*4
            total = struct.unpack_from('!H', data, ip+2)[0]
            tcp = ip+ihl
            if ihl < 20 or len(data) < tcp+20 or len(data) < ip+total:
                raise ValueError('Incomplete IPv4/TCP header')
            src = socket.inet_ntoa(data[ip+12:ip+16])
            dst = socket.inet_ntoa(data[ip+16:ip+20])
            sport, dport, seq = struct.unpack_from('!HHI', data, tcp)
            thl = (data[tcp+12] >> 4)*4
            if thl < 20 or tcp+thl > ip+total:
                raise ValueError('Invalid TCP header length')
            yield dict(wall_s=sec+frac/scale, src=src, dst=dst, sport=sport,
                       dport=dport, seq=seq, payload=data[tcp+thl:ip+total])


def analyze(paths, *, host='192.168.1.5', port=8899):
    streams, rows = {}, []
    for path in paths:
        for packet in packets(path):
            if packet['src'] != host or packet['sport'] != port:
                continue
            flow = f"{host}:{port}->{packet['dst']}:{packet['dport']}"
            stream = streams.setdefault(flow, FrameStream())
            for row in stream.feed(packet['seq'], packet['payload'], packet['wall_s']):
                rows.append(dict(row, flow=flow))
    summary = {}
    for flow, stream in streams.items():
        selected = sorted((r for r in rows if r['flow']==flow and r['command']==7),
                          key=lambda r:r['tcp_deliverable_wall_s'])
        gaps = [dict(start_wall_s=a['tcp_deliverable_wall_s'],
                     end_wall_s=b['tcp_deliverable_wall_s'],
                     gap_ms=(b['tcp_deliverable_wall_s']-a['tcp_deliverable_wall_s'])*1000)
                for a,b in zip(selected,selected[1:])]
        summary[flow] = dict(stats=dict(stream.stats), pending_segments=len(stream.waiting),
            unparsed_bytes=len(stream.buffer),
            command_counts=dict(Counter(r['command'] for r in rows if r['flow']==flow)),
            joint_push_count=len(selected),
            largest_joint_push_gaps=sorted(gaps,key=lambda r:r['gap_ms'],reverse=True)[:20])
    return dict(scope='host_arrival_and_contiguous_TCP_delivery; not_controller_sample_time; CRC_not_verified',
                notes='Check tcpdump dropped-packet totals separately. Overlap is not proof of loss. '
                      'Resync at capture start is expected; incomplete capture limits conclusions.',
                flows=summary), rows


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('pcap', nargs='+', help='capture files in chronological order')
    p.add_argument('--output', required=True)
    args = p.parse_args(argv)
    summary, rows = analyze(args.pcap)
    output = Path(args.output)
    with output.open('x') as f:
        json.dump(summary, f, indent=2)
    with output.with_suffix('.frames.jsonl').open('x') as f:
        for row in rows:
            f.write(json.dumps(row)+'\n')
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
