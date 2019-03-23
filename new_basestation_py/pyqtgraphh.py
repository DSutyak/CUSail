from PyQt5 import QtGui  # (the example applies equally well to PySide)
import pyqtgraph as pg
import time as time
import json
import pprint
global past_point
past_point = (0,0)

pp = pprint.PrettyPrinter(indent=4)

## Always start by initializing Qt (only once per application)
app = QtGui.QApplication([])

## Define a top-level widget to hold everything
w = QtGui.QWidget()


## Create some widgets to be placed inside
btn = QtGui.QPushButton('Plot')
btn2 = QtGui.QPushButton('Update')
text = QtGui.QLineEdit('Enter Buoy/Waypoint')
listw = QtGui.QListWidget()
plot = pg.PlotWidget()
plot.setLimits(minXRange=500,maxXRange=500,minYRange=500,maxYRange=500)
display1 = QtGui.QLabel('Wind Angle: <x,y,z>')
display2 = QtGui.QLabel('Roll, Pitch, Yaw: <x,y,z>')



def update():
	f = open("live_data.txt")
	raw_data = list(f)[-1]
	data = json.loads(raw_data)
	pp.pprint(data)
	print("\n")
	x = float(data['X position'][0:-2])
	y = float(data['Y position'][0:-2])
	# print(x)
	# print("\n")
	# print(y)
	# print("\n")
	global past_point
	# listw.addItem(text.text())
	# arr = text.text().split(',')
	# x = float(arr[0])
	# y = float(arr[1])
	plot.plot([past_point[0], x], [past_point[1], y])
	past_point = (x,y)
	display1.setText("Wind Angle: " + data["Wind w.r.t North"][0:-2])
	display2.setText("Roll, Pitch, Yaw: <"+data["Roll"][0:-2]+","+data["Pitch"][0:-2]+","+data["Boat direction"][0:-2]+" >")
	

def clicked():
	global past_point
	entry = text.text().strip().replace(" ","")
	try:
		arr = entry.split(',')
		if (not isinstance(float(arr[0]),float) or not isinstance(float(arr[1]),float)):
			raise Exception
		#if(entry.count(",") > 1):
			#raise Exception
		listw.addItem(entry)
		x = float(arr[0])
		y = float(arr[1])

		plot.plot([x+2, x-2], [y+2, y-2])
		plot.plot([x+2, x-2], [y-2, y+2])

	except Exception as e:
		print("Could not convert string to float: '" + entry + "'")



btn.clicked.connect(clicked)
btn2.clicked.connect(update)
## Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)

## Add widgets to the layout in their proper positions
layout.addWidget(btn2, 1, 0)  # button goes in upper-left
layout.addWidget(text, 0, 0)  # text edit goes in middle-left
layout.addWidget(listw, 2, 0)  # list widget goes in bottom-left
layout.addWidget(display1, 3, 0)  # display1 widget goes in bottom-left
layout.addWidget(display2, 3, 1)  # display2 widget goes in bottom-middle
layout.addWidget(plot, 0, 1, 3, 1)  # plot goes on right side, spanning 3 rows

## Display the widget as a new window
w.show()

## Start the Qt event loop
app.exec_()
