# Implementing a Data-Flow Control algorithm.

from intercom_binaural import Intercom_binaural
import struct
import numpy as np

if __debug__:
    import sys


class Intercom_dfc(Intercom_binaural):

    def init(self, args):
        Intercom_binaural.init(self, args)
        self.total_bps = 16 * self.number_of_channels
        self.bps_received_chunk = [self.total_bps] * self.cells_in_buffer
        self.packet_format = f"!BBHB{self.frames_per_chunk//8}B"
        self.sending_bps = self.total_bps
        self.report = self.total_bps
        self.report_got = self.total_bps
        self.min_bps = self.number_of_channels * args.minimum_bitplanes
        self.adapt_factor = args.adapt_factor
        self.ignored_bps = 0
        self.x = 0
        
            
    
    def cr(self, x): #change representation
        is_n = (x[:]>>15)*0xffff
        x[:] = ~is_n & x[:] | is_n & (0x8000 - x[:])

    def update_sending_bps(self):
        if self.sending_bps - self.report_got > self.min_bps:
            self.sending_bps = self.report_got if self.report_got > self.min_bps else self.min_bps
        else:
            self.sending_bps += self.min_bps
            self.sending_bps = self.sending_bps if self.sending_bps < self.total_bps else self.total_bps
        #print(self.sending_bps, self.report)

    def up_report(self):
        self.report = self.bps_received_chunk[self.played_chunk_number]
        self.bps_received_chunk[self.played_chunk_number] = 0
        
    def buffer(self, chunk_number, bitplane_number, bitplane):
        bitplane = np.unpackbits(np.asarray(bitplane, dtype=np.uint8)).astype(np.int16)
        self._buffer[chunk_number%self.cells_in_buffer][:, bitplane_number%self.number_of_channels] |= (bitplane << bitplane_number//self.number_of_channels)
        return chunk_number
        
    def receive_and_buffer(self):
        message, source_address = self.receiving_sock.recvfrom(self.MAX_MESSAGE_SIZE)
        ig_bps, new_report, chunk_number, bitplane_number, *bitplane = struct.unpack(self.packet_format, message)
        self.report_got = int(self.report_got*(1-self.adapt_factor) + new_report*self.adapt_factor) + 1
        self.bps_received_chunk[chunk_number % self.cells_in_buffer] += 1 + ig_bps
        return self.buffer(chunk_number, bitplane_number, bitplane)

    def send_bps(self, indata, bitplane_number):
        bitplane = (indata[:, bitplane_number%self.number_of_channels] >> bitplane_number // self.number_of_channels) & 1
        import random
        if not np.any(bitplane):
            self.ignored_bps += 1
            print("0",end="")
        else:
            bitplane = np.packbits(bitplane.astype(np.uint8))
            message = struct.pack(self.packet_format, self.ignored_bps, self.report, self.recorded_chunk_number, bitplane_number, *bitplane)
            self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))
            self.ignored_bps = 0
            
    def send(self, indata):
        for bitplane_number in range(self.number_of_channels*16-1, self.total_bps - self.sending_bps , -1):
            self.send_bps(indata, bitplane_number)
        self.send_bps(indata, 0)
        self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER
    
    def get_chunk(self):
        chunk = self._buffer[self.played_chunk_number]
        self._buffer[self.played_chunk_number % self.cells_in_buffer] = self.generate_zero_chunk()
        self.played_chunk_number = (self.played_chunk_number + 1) % self.cells_in_buffer
        return chunk
            
    def record_send_and_play_stereo(self, indata, outdata, frames, time, status):
        indata[:,0] -= indata[:,1]
        self.cr(indata)
        self.update_sending_bps()
        self.send(indata)
        
        chunk = self.get_chunk()
        self.up_report()
        self.cr(chunk)
        chunk[:,0] += chunk[:,1]
        outdata[:] = chunk
        if __debug__:
            sys.stderr.write("."); sys.stderr.flush()

    def record_send_and_play(self, indata, outdata, frames, time, status):
        self.cr(indata)
        self.update_sending_bps()
        self.send(indata)
        
        chunk = self.get_chunk()
        self.up_report()
        self.cr(chunk)
        outdata[:] = chunk
        if __debug__:
            sys.stderr.write("."); sys.stderr.flush()

    def add_args(self):
        parser = Intercom_binaural.add_args(self)
        parser.add_argument("-mb", "--minimum_bitplanes", help="Minium number of bitplanes per channel we are sending in case of slow conexion", type=int, default=2)
        parser.add_argument("-af", "--adapt_factor", help="Decide how affected is by last report. Must be between 0 and 1.", type=float, default=0.2)
        return parser

if __name__ == "__main__":
    intercom = Intercom_dfc()
    parser = intercom.add_args()
    args = parser.parse_args()
    intercom.init(args)
    intercom.run()
