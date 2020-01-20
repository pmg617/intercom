
import struct
import sounddevice as sd
import numpy as np
import argparse
import socket
import pywt

if __debug__:
    import sys


class Intercom_dwt():

    MAX_MESSAGE_SIZE = 32768
    MAX_CHUNK_NUMBER = 65536
    
    def init(self, args):
        self.number_of_channels = args.number_of_channels
        self.frames_per_second = args.frames_per_second
        self.frames_per_chunk = args.frames_per_chunk
        self.listening_port = args.mlp
        self.destination_IP_addr = args.ia
        self.destination_port = args.ilp
        self.bytes_per_chunk = self.frames_per_chunk * self.number_of_channels * np.dtype(np.int16).itemsize
        self.samples_per_chunk = self.frames_per_chunk * self.number_of_channels
        self.sending_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiving_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listening_endpoint = ("0.0.0.0", self.listening_port)
        self.receiving_sock.bind(self.listening_endpoint)

        self.chunks_to_buffer = args.chunks_to_buffer
        self.cells_in_buffer = self.chunks_to_buffer * 2
        self._buffer = [self.generate_zero_chunk()] * self.cells_in_buffer
        self.total_bps = 16 * self.number_of_channels
        self.bps_received_chunk = [self.total_bps] * self.cells_in_buffer
        self.packet_format = f"!BBHB{self.frames_per_chunk//8}B"
        self.sending_bps = self.total_bps
        self.report = self.total_bps
        self.report_got = self.total_bps
        self.min_bps = self.number_of_channels * args.minimum_bitplanes
        self.adapt_factor = args.adapt_factor
        self.ignored_bps = 0

        self.actual = self.generate_zero_chunk()
        self.next = self.generate_zero_chunk()
        self.spad = self.frames_per_chunk//16
        
        if self.number_of_channels == 2:
            self.record_send_and_play = self.record_send_and_play_stereo
            
        if __debug__:
            print(f"number_of_channels={self.number_of_channels}")
            print(f"frames_per_second={self.frames_per_second}")
            print(f"frames_per_chunk={self.frames_per_chunk}")
            print(f"samples_per_chunk={self.samples_per_chunk}")
            print(f"listening_port={self.listening_port}")
            print(f"destination_IP_address={self.destination_IP_addr}")
            print(f"destination_port={self.destination_port}")
            print(f"bytes_per_chunk={self.bytes_per_chunk}")
            print(f"chunks_to_buffer={self.chunks_to_buffer}")
            print(f"min_bps={self.min_bps}")
            print(f"adapt_factor={self.adapt_factor}")
    
    def generate_zero_chunk(self):
        return np.zeros((self.frames_per_chunk, self.number_of_channels), np.int16)
    
    def cr(self, x): #change representation
        is_n = (x[:]>>15)*0xffff
        x[:] = ~is_n & x[:] | is_n & (0x8000 - x[:])

    def update_sending_bps(self):
        if self.sending_bps - self.report_got > self.min_bps:
            self.sending_bps = self.report_got if self.report_got > self.min_bps else self.min_bps
        else:
            self.sending_bps += self.min_bps
            self.sending_bps = self.sending_bps if self.sending_bps < self.total_bps else self.total_bps

    def up_report(self):
        self.report = self.bps_received_chunk[self.played_chunk_number]
        self.bps_received_chunk[self.played_chunk_number] = 0
        
    def buffer(self, chunk_number, bitplane_number, bitplane):
        bitplane = np.unpackbits(np.asarray(bitplane, dtype=np.uint8)).astype(np.int16)
        self._buffer[chunk_number][:, bitplane_number%self.number_of_channels] |= (bitplane << bitplane_number//self.number_of_channels)
        return chunk_number
        
    def receive_and_buffer(self):
        message, source_address = self.receiving_sock.recvfrom(self.MAX_MESSAGE_SIZE)
        ig_bps, new_report, chunk_number, bitplane_number, *bitplane = struct.unpack(self.packet_format, message)
        self.report_got = int(self.report_got*(1-self.adapt_factor) + new_report*self.adapt_factor) + 1
        chunk_number %= self.cells_in_buffer
        self.bps_received_chunk[chunk_number] += 1 + ig_bps
        return self.buffer(chunk_number, bitplane_number, bitplane)

    def send_bps(self, indata, bitplane_number):      
        bitplane = (indata[:, bitplane_number % self.number_of_channels] >> bitplane_number // self.number_of_channels) & 1
        if not np.any(bitplane) and bitplane_number != self.total_bps - self.sending_bps:
            self.ignored_bps += 1
        else:
            bitplane = np.packbits(bitplane.astype(np.uint8))
            message = struct.pack(self.packet_format, self.ignored_bps, self.report, self.recorded_chunk_number, bitplane_number, *bitplane)
            self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))
            self.ignored_bps = 0
            
    def send(self, indata):
        for bitplane_number in range(self.number_of_channels*16-1, -1 + self.total_bps - self.sending_bps , -1):
            self.send_bps(indata, bitplane_number)
        self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER
    
    def get_chunk(self):
        chunk = self._buffer[self.played_chunk_number]
        self._buffer[self.played_chunk_number] = self.generate_zero_chunk()
        self.played_chunk_number = (self.played_chunk_number + 1) % self.cells_in_buffer
        return chunk

    def decompose(self, indata, chnl):
        #indata[:,chnl], coeff_slices = pywt.coeffs_to_array(pywt.wavedec(indata[:,chnl],  "bior3.5", "periodization")); return indata, coeff_slices
        leftpad = self.actual[self.frames_per_chunk-self.spad:, chnl]
        self.actual = self.next
        self.next = indata
        
        sending = np.copy(self.actual)
        coeff = pywt.wavedec(np.concatenate((leftpad, self.actual[:,chnl], self.next[:self.spad,chnl]), axis=0),  "bior3.5", "periodization")
        sp = self.spad
        for i in range(len(coeff)-1, 0, -1):
            sp >>= 1
            coeff[i] = coeff[i][sp:len(coeff[i])-sp]
        coeff[0] = coeff[0][sp:len(coeff[0])-sp]
        sending[:,chnl], coeff_slices = pywt.coeffs_to_array(coeff)
        return sending, coeff_slices

    def recompose(self, data, coeff_slices, chnl):
        data[:,chnl] = np.around(pywt.waverec(pywt.array_to_coeffs(data[:,chnl], coeff_slices, output_format="wavedec"), "bior3.5", "periodization"))
        
    def record_send_and_play_stereo(self, indata, outdata, frames, time, status):
        indata[:,0] -= indata[:,1]
        sending, coeff_slices = self.decompose(indata, 1)
        self.cr(sending)
        self.update_sending_bps()
        self.send(sending)
        chunk = self.get_chunk()
        self.up_report()
        self.cr(chunk)
        self.recompose(chunk, coeff_slices, 1)
        chunk[:,0] += chunk[:,1]
        outdata[:] = chunk        
        if __debug__:
            sys.stderr.write("."); sys.stderr.flush()

    def record_send_and_play(self, indata, outdata, frames, time, status):
        sending, coeff_slices = self.decompose(indata, 0)
        self.cr(sending)
        self.update_sending_bps()
        self.send(sending)
        chunk = self.get_chunk()
        self.up_report()
        self.cr(chunk)
        self.recompose(chunk, coeff_slices, 0)
        outdata[:] = chunk
        if __debug__:
            sys.stderr.write("."); sys.stderr.flush()

    def run(self):
        self.recorded_chunk_number = 0
        self.played_chunk_number = 0
        with sd.Stream(samplerate=self.frames_per_second, blocksize=self.frames_per_chunk, dtype=np.int16, channels=self.number_of_channels, callback=self.record_send_and_play):
            print("-=- Press CTRL + c to quit -=-")
            first_received_chunk_number = self.receive_and_buffer()
            self.played_chunk_number = (first_received_chunk_number - self.chunks_to_buffer) % self.cells_in_buffer
            while True:
                self.receive_and_buffer()
                
    def add_args(self):
        parser = argparse.ArgumentParser(description="Real-time intercom", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        parser.add_argument("-s", "--frames_per_chunk", help="Samples per chunk.", type=int, default=1024)
        parser.add_argument("-r", "--frames_per_second", help="Sampling rate in frames/second.", type=int, default=5000) #default=44100)
        parser.add_argument("-c", "--number_of_channels", help="Number of channels.", type=int, default=2)
        parser.add_argument("-p", "--mlp", help="My listening port.", type=int, default=4444)
        parser.add_argument("-i", "--ilp", help="Interlocutor's listening port.", type=int, default=4444)
        parser.add_argument("-a", "--ia", help="Interlocutor's IP address or name.", type=str, default="localhost")
        parser.add_argument("-cb", "--chunks_to_buffer", help="Number of chunks to buffer", type=int, default=32)
        parser.add_argument("-mb", "--minimum_bitplanes", help="Minium number of bitplanes per channel we are sending in case of slow conexion", type=int, default=2)
        parser.add_argument("-af", "--adapt_factor", help="Decide how affected is by last report. Must be between 0 and 1.", type=float, default=0.2)
        return parser

if __name__ == "__main__":
    intercom = Intercom_dwt()
    intercom.init(intercom.add_args().parse_args())
    intercom.run()
