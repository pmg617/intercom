# Adding a buffer.

import sounddevice as sd
import numpy as np
import struct
from intercom_buffer import Intercom_buffer

if __debug__:
    import sys

class Intercom_bitplanes(Intercom_buffer):

    def init(self, args):
        Intercom_buffer.init(self, args)
        self.packet_format = f"HHH{self.frames_per_chunk // 8}B"
        

    # chunk number, channel, bit
    
    def run(self):

        self.recorded_chunk_number = 0
        self.played_chunk_number = 0

        def receive_and_buffer():
            message, source_address = self.receiving_sock.recvfrom(self.MAX_MESSAGE_SIZE) # Cojer mensaje de la red
            chunk_number, channel, bit, *data = struct.unpack(self.packet_format, message) # Extraer información
            bitplane = np.asarray(np.unpackbits(np.asarray(data, dtype=np.uint8)), dtype=np.uint16) # Desempaquetar bits
            self._buffer[chunk_number % self.cells_in_buffer][:, channel] |= bitplane << bit # Insertar bits en el buffer
            return chunk_number

        def record_send_and_play(indata, outdata, frames, time, status):
            #División y envío de indata
            for i in range(15, -1, -1): #iterar bits
                for j in range(self.number_of_channels):
                    message = struct.pack(self.packet_format, self.recorded_chunk_number, j, i, *np.packbits((indata[:,j] >> i) & 1) ) # Empaquetar mensaje
                    self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port)) # Enviar mensaje
            self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER # Incrementar número de chunk gravado
                    
            #Reproducir
            chunk = self._buffer[self.played_chunk_number % self.cells_in_buffer] # Extraer chunk del buffer
            self._buffer[self.played_chunk_number % self.cells_in_buffer] = self.generate_zero_chunk() # Eliminar chunk del buffer
            self.played_chunk_number = (self.played_chunk_number + 1) % self.cells_in_buffer # Incrementar chunk reproducido en 1
            outdata[:] = chunk # Enviar chunk a Stream
            
            if __debug__:
                sys.stderr.write("."); sys.stderr.flush()

        with sd.Stream(samplerate=self.frames_per_second, blocksize=self.frames_per_chunk, dtype=np.int16,
                       channels=self.number_of_channels, callback=record_send_and_play):
            #Se crea un hilo paralelo en bucle con record_send_and_play
            print("-=- Press CTRL + c to quit -=-")
            self.played_chunk_number = (receive_and_buffer() - self.chunks_to_buffer) % self.cells_in_buffer # Empezamos a reproducir la mitad atrás
            while True:
                receive_and_buffer()
                
    def add_args(self):
        parser = Intercom_buffer.add_args(self)
        return parser

if __name__ == "__main__":
    intercom = Intercom_bitplanes()
    parser = intercom.add_args()
    args = parser.parse_args()
    intercom.init(args)
    intercom.run()
