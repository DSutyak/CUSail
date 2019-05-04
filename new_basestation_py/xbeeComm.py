import serial
import json
from xbee import XBee
import re
import pprint
import time

serial_port = serial.Serial('COM6', 9600) #Marissa - COM3, Troy - COM5
xbee = XBee(serial_port)
f = open("live_data.txt", 'w')

# Initialize gui
#window= Tk()
#start= mclass(window)
#window.mainloop()

header = "----------NAVIGATION----------"
regex = "(?:'rf_data': b')((.|\n)*)'"

data = ""
while True:
	try:
		print("Waiting...")
		packet = str(xbee.wait_read_frame())
		print("Packet Recieved!")
		match = re.search(regex, packet)
		if match:
			line = match.group(1)
			data += line
			if (header in data):
				header_start = data.find(header)
				header_end = header_start + len(header)
				data_to_send = data[0:header_start]
				data_arr = data_to_send.split("\\n")
				#data_assoc = []
				data_assoc = {}
				for datum in data_arr:
					if (":" in datum):
						print(datum)
						label, value = datum.split(":")
						data_assoc[label] = value
					
				print ("Parse data cycle to GUI")
				# Update gui with data
				print (json.dumps(data_assoc))
				print ("\n\n")
				f.write(json.dumps(data_assoc)+"\n")
				#pp.pprint(data_arr)
				data = data[header_end:len(data)]
		else:
			print ("Regex failed to match")
			print(packet)
	except KeyboardInterrupt:
		break


serial_port.close()