import struct
import numpy as np
# packet_data = np.zeros((8,1),np.uint8)
# packet_data[0] = 0
# packet_data[1] = 1
# packet_data[2] = 2
# feedback_pos= struct.unpack("S",packet_data[1:2])
a = 127
buffer  =  struct.pack( "b" ,  a)
print(buffer)
b = struct.unpack("b",buffer)
print(b)