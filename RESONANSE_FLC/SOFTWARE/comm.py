

def get_temperature(self):
	packet = bytearray()
	packet.append(0xBE)
	packet.append(0x01)
	packet.append(0x00)
	packet.append(0xED)
	ser.write(packet)
	answer = ser.read_until(0xED,7)
	if answer[0]==b'\xBE':
		if answer[1]==b'\x02':
			return struct.unpack_from('>f', answer[2:])
	
	

 
