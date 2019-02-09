import mpl
import serial
from xbee import XBee
import re
import pprint
import time

serial_port = serial.Serial('COM3', 9600) #Marissa - COM3, Troy - COM5
xbee = XBee(serial_port)
pp = pprint.PrettyPrinter(indent=4)


# Initialize gui
#window= Tk()
#start= mclass(window)
#window.mainloop()

header = "----------NAVIGATION----------"
regex = "(?:'rf_data': b')((.|\n)*)'"

data = ""
while True:
	try:
		packet = str(xbee.wait_read_frame())
		print (packet)
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
						#data_assoc.append(tuple(datum.split(":")))
					    label, value = datum.split(":")
					    data_assoc[label] = value
					
				pp.pprint ("Parse data cycle to GUI")
				# Update gui with data
				pp.pprint (data_assoc)
				#pp.pprint(data_arr)
				data = data[header_end:len(data)]
		else:
			pp.pprint ("Regex failed to match")
	except KeyboardInterrupt:
		break
	time.sleep(1)


serial_port.close()