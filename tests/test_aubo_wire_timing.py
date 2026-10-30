"""Synthetic wire messages; no network and no controller."""
import struct
import tempfile
from pathlib import Path
import unittest

from teleop.aubo.wire_timing import SOF, END, FrameStream, packets, analyze


def frame(command=7, payload=b'xyz'):
    return SOF+struct.pack('<II',len(payload)+12,command)+payload+b'\0'*4+END


class WireTests(unittest.TestCase):
    def test_split_and_coalesced_messages_preserve_arrival_times_and_type(self):
        parser = FrameStream()
        one, two = frame(7), frame(26)
        self.assertEqual(parser.feed(100,one[:9],10.), [])
        rows = parser.feed(109,one[9:]+two,10.03)
        self.assertEqual([r['command'] for r in rows],[7,26])
        self.assertEqual(rows[0]['first_arrival_wall_s'],10.)
        self.assertEqual(rows[0]['complete_arrival_wall_s'],10.03)
        self.assertEqual(rows[1]['first_arrival_wall_s'],10.03)

    def test_reordering_and_retransmission_cannot_double_count_pushes(self):
        parser = FrameStream(); data=frame()
        parser.feed(100,data[:5],1.)
        self.assertEqual(parser.feed(110,data[10:],1.01), [])
        rows=parser.feed(105,data[5:10],1.02)
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['complete_arrival_wall_s'],1.02)
        self.assertEqual(parser.feed(100,data,1.03), [])
        self.assertEqual(parser.stats['frames'],1)
        self.assertEqual(parser.stats['out_of_order_segments'],1)

    def test_partial_capture_and_invalid_header_resync_without_fake_frames(self):
        parser = FrameStream()
        rows=parser.feed(100,b'partial'+SOF+struct.pack('<I',3)+b'junk'+frame(),1.)
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['command'],7)
        self.assertEqual(parser.stats['invalid_lengths'],1)

    def test_sequence_wrap_preserves_contiguous_stream(self):
        parser = FrameStream(); data=frame()
        self.assertEqual(parser.feed(2**32-5,data[:5],1.), [])
        self.assertEqual(len(parser.feed(0,data[5:],1.01)),1)

    def test_later_complete_frame_waits_for_prior_tcp_bytes(self):
        parser = FrameStream(); one=frame(); two=frame()
        parser.feed(100,one[:5],1.)
        self.assertEqual(parser.feed(110,one[10:]+two,1.01), [])
        rows=parser.feed(105,one[5:10],1.02)
        self.assertEqual(rows[1]['complete_arrival_wall_s'],1.01)
        self.assertEqual(rows[1]['tcp_deliverable_wall_s'],1.02)

    def test_truncated_capture_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'wire.pcap'
            path.write_bytes(struct.pack('<IHHIIII',0xa1b2c3d4,2,4,0,0,65535,1)+
                             struct.pack('<IIII',1,0,100,100)+b'short')
            with self.assertRaisesRegex(ValueError,'Truncated packet'):
                list(packets(path))

    def test_pcap_groups_joint_push_separately_from_other_responses(self):
        def packet(payload, seq):
            ethernet=b'\0'*12+b'\x08\0'
            ip=bytearray(20); ip[0]=0x45; ip[9]=6
            struct.pack_into('!H',ip,2,40+len(payload))
            ip[12:16]=bytes([192,168,1,5]); ip[16:20]=bytes([192,168,1,100])
            tcp=bytearray(20); struct.pack_into('!HHI',tcp,0,8899,50000,seq); tcp[12]=0x50
            return ethernet+ip+tcp+payload
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'wire.pcap'
            with path.open('wb') as f:
                f.write(struct.pack('<IHHIIII',0xa1b2c3d4,2,4,0,0,65535,1))
                seq=100
                for micros,command in [(0,7),(10000,26),(50000,7)]:
                    data=packet(frame(command),seq); seq+=len(frame(command))
                    f.write(struct.pack('<IIII',100,micros,len(data),len(data))+data)
            summary,rows=analyze([path])
            flow=next(iter(summary['flows'].values()))
            self.assertEqual(flow['joint_push_count'],2)
            self.assertEqual(len(rows),3)
            self.assertAlmostEqual(flow['largest_joint_push_gaps'][0]['gap_ms'],50.)


if __name__ == '__main__':
    unittest.main()
