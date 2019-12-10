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
        self.bps_received_chunk = [0] * self.cells_in_buffer
        self.packet_format = f"!BHB{self.frames_per_chunk//8}B"
        self.sending_bps = self.total_bps
        self.report_total = 0
        self.report_got = self.total_bps
        self.min_bps = self.number_of_channels * args.minimum_bitplanes
        self.max_incr = self.total_bps // 2
        self.increment = self.max_incr
    
    def cr(self, x): #change representation
        #return
        for i in range(len(x)):
            for j in range(len(x[i])):
                if x[i][j] >> 15:
                    x[i][j] = 0x8000 - x[i][j]

    def update_sending_bps(self):
        if self.report_got < self.sending_bps:
            self.sending_bps = self.report_got if self.report_got > self.min_bps else self.min_bps
            self.increment = self.increment // 2 + 1
        else:
            more_bps = self.sending_bps + self.increment
            self.sending_bps = more_bps if more_bps < self.total_bps else self.total_bps
            self.increment = self.increment * 2 if self.increment < self.max_incr else self.max_incr

        if __debug__:
            print(self.sending_bps, self.report_got, self.increment)

    def decr_report(self):
        chunk_number = self.played_chunk_number % self.cells_in_buffer
        self.report_total -= self.bps_received_chunk[chunk_number]
        self.bps_received_chunk[chunk_number] = 0
        
    def buffer(self, chunk_number, bitplane_number, bitplane):
        bitplane = np.unpackbits(np.asarray(bitplane, dtype=np.uint8)).astype(np.int16)
        self._buffer[chunk_number%self.cells_in_buffer][:, bitplane_number%self.number_of_channels] |= (bitplane << bitplane_number//self.number_of_channels)
        return chunk_number
        
    def receive_and_buffer(self):
        message, source_address = self.receiving_sock.recvfrom(self.MAX_MESSAGE_SIZE)
        self.report_got, chunk_number, bitplane_number, *bitplane = struct.unpack(self.packet_format, message)
        self.bps_received_chunk[chunk_number % self.cells_in_buffer] += 1
        self.report_total += 1
        return self.buffer(chunk_number, bitplane_number, bitplane)

    def record_and_send(self, indata):        
        for bitplane_number in range(self.number_of_channels*16-1, -1 + self.total_bps - self.sending_bps , -1):
            bitplane = (indata[:, bitplane_number%self.number_of_channels] >> bitplane_number // self.number_of_channels) & 1
            bitplane = np.packbits(bitplane.astype(np.uint8))
            message = struct.pack(self.packet_format, self.report_total // self.chunks_to_buffer, self.recorded_chunk_number, bitplane_number, *bitplane)
            self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))
        self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER
    
    def get_chunk(self):
        chunk = self._buffer[self.played_chunk_number]
        self._buffer[self.played_chunk_number % self.cells_in_buffer] = self.generate_zero_chunk()
        self.played_chunk_number = (self.played_chunk_number + 1) % self.cells_in_buffer
        return chunk
            
    def record_send_and_play_stereo(self, indata, outdata, frames, time, status):

        for i in range(len(indata)):
            for j in range(len(indata[i])):
                pass#indata[i][j] = -4
        print(indata)
        
        indata[:,0] -= indata[:,1]
        self.update_sending_bps()
        self.cr(indata)
        self.record_and_send(indata)
        
        self._buffer[self.played_chunk_number][:,0] += self._buffer[self.played_chunk_number % self.cells_in_buffer][:,1]
        self.decr_report()
        chunk = self.get_chunk()
        self.cr(chunk)

        print(chunk)
        print()
        print()
        
        outdata[:] = chunk
        if __debug__:
            sys.stderr.write("."); sys.stderr.flush()

    def record_send_and_play(self, indata, outdata, frames, time, status):
        self.update_sending_bps()
        self.cr(indata)
        self.record_and_send(indata)
        
        self.decr_report()
        chunk = self.get_chunk()
        self.cr(chunk)
        outdata[:] = chunk
        if __debug__:
            sys.stderr.write("."); sys.stderr.flush()

    def add_args(self):
        parser = Intercom_binaural.add_args(self)
        parser.add_argument("-mb", "--minimum_bitplanes", help="Minium number of bitplanes per channel we are sending in case of slow conexion", type=int, default=2)
        return parser

if __name__ == "__main__":
    intercom = Intercom_dfc()
    parser = intercom.add_args()
    args = parser.parse_args()
    intercom.init(args)
    intercom.run()
