from PyQt5 import QtGui  # (the example applies equally well to PySide)
import pyqtgraph as pg
import time as time
import json
import pprint
import random
global past_point
past_point = (0,0)

pg.setConfigOption('background', 'w')
pp = pprint.PrettyPrinter(indent=4)

## Always start by initializing Qt (only once per application)
app = QtGui.QApplication([])

## Define a top-level widget to hold everything
w = QtGui.QWidget()


## Create some widgets to be placed inside
btn = QtGui.QPushButton('Waypoint')
btn2 = QtGui.QPushButton('Update')
btn3 = QtGui.QPushButton('Buoy')
text = QtGui.QLineEdit('Enter Buoy/Waypoint')
listw = QtGui.QListWidget()
listb = QtGui.QListWidget()
plot = pg.PlotWidget()
plot.setLimits(minXRange=500,maxXRange=500,minYRange=500,maxYRange=500)
display1 = QtGui.QLabel('Wind Angle: <x,y,z>')
display2 = QtGui.QLabel('Tail Vector: <x,y,z>')



def update():
    f = open("live_data.txt")
    raw_data = list(f)[-1]
    data = json.loads(raw_data)
    pp.pprint(data)
    print("\n")
    x = float(data['X position'][0:-2])
    y = float(data['Y position'][0:-2])
    print(x)
    print("\n")
    print(y)
    print("\n")
    global past_point
    # listw.addItem(text.text())
    # arr = text.text().split(',')
    # x = float(arr[0])
    # y = float(arr[1])
    plot.plot([past_point[0], x], [past_point[1], y])
    past_point = (x,y)

brush_list = [pg.mkColor(c) for c in "rgbcmykwrg"]
#pen = random.choice(brush_list)

def waypoint():
    pen = random.choice(brush_list)
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

        plot.plot([x+4, x-4], [y+4, y-4], pen = 'y')
        plot.plot([x+4, x-4], [y-4, y+4], pen = 'y')

    except Exception as e:
        print("Could not convert string to float: '" + entry + "'")


def buoy():
    #pen = random.choice(brush_list)  --> for cool, fun color times
    entry = text.text().strip().replace(" ","")
    try:
        arr = entry.split(',')
        if (not isinstance(float(arr[0]),float) or not isinstance(float(arr[1]),float)):
            raise Exception
        #if(entry.count(",") > 1):
            #raise Exception
        listb.addItem(entry)
        x = float(arr[0])
        y = float(arr[1])

        plot.plot([x+4, x-4], [y+4, y-4], pen = 'c')
        plot.plot([x+4, x-4], [y-4, y+4], pen = 'c')

    except Exception as e:
        print("Could not convert string to float: '" + entry + "'")



btn.clicked.connect(waypoint)
btn2.clicked.connect(update)
btn3.clicked.connect(buoy)
## Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)

## Add widgets to the layout in their proper positions
## goes row, col, rowspan, colspan
layout.addWidget(btn, 1, 0)  # button goes in mid-left is waypoints
layout.addWidget(btn2, 0, 0, 1, 2)  # button2 goes in upper-left
layout.addWidget(btn3, 1, 1)  # button3 goes in upper-left is buoy
layout.addWidget(text, 2, 0, 1, 2)  # text edit goes in middle-left
layout.addWidget(listw, 4, 0)  # list widget goes in bottom-left
layout.addWidget(listb, 4, 1)  # list widget goes in bottom-left
layout.addWidget(display1, 5, 0)  # display1 widget goes in bottom-left
layout.addWidget(display2, 5, 1)  # display2 widget goes in bottom-middle
layout.addWidget(plot, 0, 2, 5, 1)  # plot goes on right side, spanning 3 rows

## Display the widget as a new window
w.show()

## Start the Qt event loop
app.exec_()
