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
        self.plot = Canvas(window, width=600, height=600)
        self.plot.create_line(0, 600, 600, 0, fill="red", dash=(4, 4))
        # self.box = Entry(window)
        self.waypoints= Canvas (window, width=200, height=600, bg="red")
        self.info = Frame (window, bg="white", height=100, width=800)
        self.plot.grid(row=0, column=0)
        self.waypoints.grid(row=0, column=1)
        self.info.grid(row=1, column=0, columnspan=2)

    def plot (self):
        x=np.array ([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        v= np.array ([16,16.31925,17.6394,16.003,17.2861,17.3131,19.1259,18.9694,22.0003,22.81226])
        p= np.array ([16.23697,     17.31653,     17.22094,     17.68631,     17.73641 ,    18.6368,
            19.32125,     19.31756 ,    21.20247  ,   22.41444   ,  22.11718  ,   22.12453])

        fig = Figure(figsize=(6,6))
        a = fig.add_subplot(111)
        a.scatter(v,x,color='red')
        a.plot(p, range(2 +max(x)),color='blue')
        a.invert_yaxis()

        a.set_title ("Estimation Grid", fontsize=16)
        a.set_ylabel("Y", fontsize=14)
        a.set_xlabel("X", fontsize=14)

        canvas = FigureCanvasTkAgg(fig, master=self.window)
        canvas.get_tk_widget().grid(row=1, column=0, columnspan=2)
        canvas.draw()

window= Tk()
start= mclass (window)
window.mainloop() 