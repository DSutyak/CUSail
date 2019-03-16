from PyQt5 import QtGui  # (the example applies equally well to PySide)
import pyqtgraph as pg
<<<<<<< HEAD
pastpoint = (0,0)

=======
import time as time
global past_point
past_point = (0,0)
>>>>>>> cf2be9d42c27c98e5ff4e9d2c080cefaa0832de6
## Always start by initializing Qt (only once per application)
app = QtGui.QApplication([])

## Define a top-level widget to hold everything
w = QtGui.QWidget()


## Create some widgets to be placed inside
<<<<<<< HEAD
btn = QtGui.QPushButton('Plot')
text = QtGui.QLineEdit('Enter Buoy/Waypoint')
=======
btn = QtGui.QPushButton('press me')
text = QtGui.QLineEdit('0,0')
>>>>>>> cf2be9d42c27c98e5ff4e9d2c080cefaa0832de6
listw = QtGui.QListWidget()
plot = pg.PlotWidget()
display1 = QtGui.QLabel('Wind Vector: <x,y,z>')
display2 = QtGui.QLabel('Tail Vector: <x,y,z>')


def clicked():
    global pastpoint
    input = text.text()
    arr = input.split(',')
    listw.addItem(input)
    x = float(arr[0])
    y = float(arr[1])
    plot.plot([pastpoint[0],x],[pastpoint[1],y])
    pastpoint = (x,y)

btn.clicked.connect(clicked)


plot.plot([1,2,3],[4,5,6])
#time.sleep(5)
plot.plot([3,-2],[6,-2])

def clicked():
    global past_point
    listw.addItem(text.text())
    arr = text.text().split(',')
    x = float(arr[0])
    y = float(arr[1])
    plot.plot([past_point[0], x], [past_point[1], y])
    past_point = (x,y)

btn.clicked.connect(clicked)



## Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)

## Add widgets to the layout in their proper positions
layout.addWidget(btn, 1, 0)  # button goes in upper-left
layout.addWidget(text, 0, 0)  # text edit goes in middle-left
layout.addWidget(listw, 2, 0)  # list widget goes in bottom-left
layout.addWidget(display1, 3, 0)  # display1 widget goes in bottom-left
layout.addWidget(display2, 3, 1)  # display2 widget goes in bottom-middle
layout.addWidget(plot, 0, 1, 3, 1)  # plot goes on right side, spanning 3 rows

## Display the widget as a new window
w.show()

## Start the Qt event loop
app.exec_()
