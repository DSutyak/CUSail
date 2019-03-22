from PyQt5 import QtGui  # (the example applies equally well to PySide)
import pyqtgraph as pg
import time as time
global past_point
past_point = (0,0)
## Always start by initializing Qt (only once per application)
app = QtGui.QApplication([])

## Define a top-level widget to hold everything
w = QtGui.QWidget()


## Create some widgets to be placed inside
btn = QtGui.QPushButton('press me')
text = QtGui.QLineEdit('0,0')
listw = QtGui.QListWidget()
plot = pg.PlotWidget()

def update():
    global past_point
    listw.addItem(text.text())
    arr = text.text().split(',')
    x = float(arr[0])
    y = float(arr[1])
    plot.plot([past_point[0], x], [past_point[1], y])
    past_point = (x,y)

def clicked():
	listw.addItem(text.text())
	arr = text.text().split(',')
	x = float(arr[0])
	y = float(arr[1])
	plot.plot([x], [y])

btn.clicked.connect(clicked)

## Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)

## Add widgets to the layout in their proper positions
layout.addWidget(btn, 0, 0)   # button goes in upper-left
layout.addWidget(text, 1, 0)   # text edit goes in middle-left
layout.addWidget(listw, 2, 0)  # list widget goes in bottom-left
layout.addWidget(plot, 0, 1, 3, 1)  # plot goes on right side, spanning 3 rows

## Display the widget as a new window
w.show()

## Start the Qt event loop
app.exec_()