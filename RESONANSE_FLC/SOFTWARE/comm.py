

import serial

class IO:

	def __init__(self, port):
		self.ser = serial.Serial(port, 38400, timeout=0, parity=serial.PARITY_EVEN, rtscts=1)

	def get_temperature(self):
		packet = bytearray()
		packet.append(0xBE)
		packet.append(0x01)
		packet.append(0x00)
		packet.append(0xED)
		ser.write(packet)
		answer = self.ser.read_until(0xED,7)
		if answer[0]==b'\xBE':
			if answer[1]==b'\x02':
				return struct.unpack_from('>f', answer[2:])
	
	
	def write_epprom(self,address,data):
		head = bytearray()
		head.append(0xBE)
		head.append(0x03)
		head.append(len(address)+len(data))
		head.append(address)
		packet = head + data
		packet.append(0xED)
		self.ser.write(packet)
		answer = self.ser.read_until(0xED,7)
		if answer[0]==b'\xBE':
			if answer[1]==b'\x04':
				return true
 
