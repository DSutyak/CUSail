import sys
from PyQt5 import QtGui
from PyQt5.QtGui import * # (the example applies equally well to PySide)
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QApplication, QWidget, QSpinBox
# from PyQt5 import QtWidgets
from PIL import Image
import pyqtgraph as pg
import time as time
import json
import pprint
import random
import PIL
import numpy
global past_point
import os
past_point = (0,0)

pg.setConfigOption('background', 'w')
pp = pprint.PrettyPrinter(indent=4)

## Always start by initializing Qt (only once per application)
app = QtGui.QApplication(sys.argv)

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
display1 = QtGui.QLabel('Wind Direction: <x,y,z>')
display2 = QtGui.QLabel('Roll, Pitch, Yaw: <x,y,z>')

#Create Image

#files = os.listdir("/Users/mahikakudlugi/Desktop/CUSail/new_basestation_py/")
    #for file in files:
        #if file is "sailboat_cartoon.jpg":
# img = Image.open("sailboat_cartoon.jpg")
# img = img.rotate(90)

#im = Image.open("sailboat_cartoon.jpg")
#im = numpy.asarray(Image.open('sailboat_cartoon.jpg','rb'))
#I = numpy.asarray(Image.open('sailboat_cartoon.jpg'))
#im = Image.fromarray(numpy.uint8(I))

#im = Image.open("/Users/mahikakudlugi/Desktop/CUSail/new_basestation_py/sailboat_cartoon.jpg")
#np_im = numpy.array(im)
# im2arr = numpy.array(img)
# image = pg.image(im2arr)

class CompassWidget(QWidget):

    angleChanged = pyqtSignal(float)

    def __init__(self, parent = None):

        QWidget.__init__(self, parent)

        self._angle = 0.0
        self._margins = 10
        self._pointText = {0: "N", 45: "NE", 90: "E", 135: "SE", 180: "S",
                           225: "SW", 270: "W", 315: "NW"}

    def paintEvent(self, event):

        painter = QPainter()
        painter.begin(self)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.fillRect(event.rect(), self.palette().brush(QPalette.Window))
        self.drawMarkings(painter)
        self.drawNeedle(painter)

        painter.end()

    def drawMarkings(self, painter):

        painter.save()
        painter.translate(self.width()/2, self.height()/2)
        scale = min((self.width() - self._margins)/120.0,
                    (self.height() - self._margins)/120.0)
        painter.scale(scale, scale)

        font = QFont(self.font())
        font.setPixelSize(10)
        metrics = QFontMetricsF(font)

        painter.setFont(font)
        painter.setPen(self.palette().color(QPalette.Shadow))

        i = 0
        while i < 360:

            if i % 45 == 0:
                painter.drawLine(0, -40, 0, -50)
                painter.drawText(-metrics.width(self._pointText[i])/2.0, -52,
                                 self._pointText[i])
            else:
                painter.drawLine(0, -45, 0, -50)

            painter.rotate(15)
            i += 15

        painter.restore()

    def drawNeedle(self, painter):

        painter.save()
        painter.translate(self.width()/2, self.height()/2)
        painter.rotate(self._angle)
        scale = min((self.width() - self._margins)/120.0,
                    (self.height() - self._margins)/120.0)
        painter.scale(scale, scale)

        painter.setPen(QPen(Qt.NoPen))
        painter.setBrush(self.palette().brush(QPalette.Shadow))

        painter.drawPolygon(
            QPolygon([QPoint(-10, 0), QPoint(0, -45), QPoint(10, 0),
                      QPoint(0, 45), QPoint(-10, 0)])
            )

        painter.setBrush(self.palette().brush(QPalette.Highlight))

        painter.drawPolygon(
            QPolygon([QPoint(-5, -25), QPoint(0, -45), QPoint(5, -25),
                      QPoint(0, -30), QPoint(-5, -25)])
            )

        painter.restore()

    def sizeHint(self):
        return QSize(300, 300)

    def angle(self):
        return self._angle

    # @pyqtSlot(float)
    def setAngle(self, angle):

        if angle != self._angle:
            self._angle = angle
            self.angleChanged.emit(angle)
            self.update()

    angle = pyqtProperty(float, angle, setAngle)

def update():
    f = open("live_data.txt")
    raw_data = list(f)[-1]
    data = json.loads(raw_data)
    pp.pprint(data)
    print("\n")
    x = float(data['X position'][0:-2])
    y = float(data['Y position'][0:-2])
    lati = float(data['latitude'][0:-2])
    longi = float(data['longitude'][0:-2])
    wind_dir = float(data["Wind w.r.t North"][0:-2])
    roll = float(data["Roll"][0:-2])
    pitch = float(data["Pitch"][0:-2])
    boat_dir = float(data["Boat direction"][0:-2])
    waypoint_number = int(data["Next Waypoint #"][0:-2])
    waypoint_x = (data["Next Waypoint X"])
    waypoint_y = (data["Next Waypoint Y"])
    waypoint_distance = (data["Distance to Waypoint"][0:-2])
    waypoint_angle = (data["Angle to Waypoint"][0:-2])

    # print(x)
    # print("\n")
    # print(y)
    # print("\n")
    global past_point
    # listw.addItem(text.text())
    # arr = text.text().split(',')
    # x = float(arr[0])
    # y = float(arr[1])
    wind_compass.setAngle(wind_dir)
    boat_compass.setAngle(boat_dir)
    plot.plot([past_point[0], x], [past_point[1], y])
    past_point = (x,y)
    # display1.setText("Wind Angle: " + data["Wind w.r.t North"][0:-2])
    display2.setText("Roll, Pitch, Yaw: <"+data["Roll"][0:-2]+","+data["Pitch"][0:-2]+","+data["Boat direction"][0:-2]+" >")



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

        plot.plot([x+4, x-4], [y+4, y-4], pen = 'k')
        plot.plot([x+4, x-4], [y-4, y+4], pen = 'k')

    except Exception as e:
        print("Could not convert string to float: '" + entry + "'")


def buoy():
    pen = random.choice(brush_list)
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


wind_compass = CompassWidget()
boat_compass = CompassWidget()
# spinBox.valueChanged[float].connect(compass.setAngle)

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
layout.addWidget(display1, 6, 0)  # display1 widget goes in bottom-left
layout.addWidget(display2, 6, 1)  # display2 widget goes in bottom-middle
layout.addWidget(plot, 0, 3, 5, 1)  # plot goes on right side, spanning 3 rows
layout.addWidget(wind_compass, 5, 0)
layout.addWidget(boat_compass, 5, 1)
# layout.addWidget(image, 0, 3, 5, 1)

#layout.addWidget()
# layout.addWidget(spinBox, 5, 0)
## Display the widget as a new window
w.show()

## Start the Qt event loop
app.exec_()
