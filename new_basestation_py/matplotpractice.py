import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox

fig, ax = plt.subplots()
# plt.subplots_adjust(bottom=0.2)
# t = np.arange(-2.0, 2.0, 0.001)
# s = t ** 2
# initial_text = "t ** 2"
# l, = plt.plot(t, s, lw=2)


def submit(text):
    arr = text.split(',')
    x = float(arr[0])
    y = float(arr[1])
    ax.plot([x], [y], marker='o', markersize=13, color="red")
    #, int(arr[1])) #, int(arr[1]), marker='o', markersize=3, color="red")
    # ax.set_ylim(np.min(ydata), np.max(ydata))
    # plt.draw()
    # plt.show()

axbox = plt.axes([0.1, 0.05, 0.8, 0.075])
text_box = TextBox(axbox, 'Plot Point')
text_box.on_submit(submit)

plt.show()


# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.widgets import Button

# freqs = np.arange(2, 20, 3)

# fig, ax = plt.subplots()
# plt.subplots_adjust(bottom=0.2)
# t = np.arange(0.0, 1.0, 0.001)
# s = np.sin(2*np.pi*freqs[0]*t)
# l, = plt.plot(t, s, lw=2)


# class Index(object):
#     ind = 0

#     def next(self, event):
#         self.ind += 1
#         i = self.ind % len(freqs)
#         ydata = np.sin(2*np.pi*freqs[i]*t)
#         l.set_ydata(ydata)
#         plt.draw()

#     def prev(self, event):
#         self.ind -= 1
#         i = self.ind % len(freqs)
#         ydata = np.sin(2*np.pi*freqs[i]*t)
#         l.set_ydata(ydata)
#         plt.draw()

# callback = Index()
# axprev = plt.axes([0.7, 0.05, 0.1, 0.075])
# axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
# bnext = Button(axnext, 'Next')
# bnext.on_clicked(callback.next)
# bprev = Button(axprev, 'Previous')
# bprev.on_clicked(callback.prev)

# plt.show()