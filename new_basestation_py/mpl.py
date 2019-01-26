import matplotlib
matplotlib.use('TkAgg')
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from tkinter import *

class mclass:
    def __init__(self,  window):
        self.window = window
        self.window.resizable(0,0)
        # self.graph = Canvas(window, width=600, height=600)
        # self.graph.create_line(0, 600, 600, 0, fill="red", dash=(4, 4))
        # self.box = Entry(window)
        self.waypoints= Canvas (window, width=200, height=600, bg="red")
        self.info = Frame (window, bg="white", height=100, width=800)
        # self.graph.grid(row=0, column=0)
        self.waypoints.grid(row=0, column=1)
        self.info.grid(row=1, column=0, columnspan=2)

        p = np.array ([])

        fig = Figure(figsize=(6,6))
        a = fig.add_subplot(111)
        a.plot(p, range(0),color='blue')

        a.set_title ("Boat Position", fontsize=16)

        self.canvas = FigureCanvasTkAgg(fig, master=self.window)
        # self.canvas.autoscale(enable=True)
        self.canvas.get_tk_widget().grid(row=0, column=0)
        self.canvas.draw()


    def update (self, data):
        arr=[]

window= Tk()
start= mclass (window)
window.mainloop() 