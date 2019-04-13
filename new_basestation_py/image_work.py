from matplotlib.image import imread
from PyQt5 import QtGui  # (the example applies equally well to PySide)
import pyqtgraph as pg

## Always start by initializing Qt (only once per application)
app = QtGui.QApplication([])

## Define a top-level widget to hold everything
w = QtGui.QWidget()


img = imread('compass.jpg')
print(type(img))


imv = pg.ImageView()
imv.show()
imv.setImage(img)

