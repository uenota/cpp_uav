#!/usr/bin/env python

from matplotlib import pyplot as plt
from matplotlib.widgets import Button
#import rospy

class PolygonBuilder(object):
    def __init__(self, line, draw_button, calc_button, clear_button):
        self.line = line
        self.xs = list()
        self.ys = list()
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)
        self.draw_button = draw_button
        self.calc_button = calc_button
        self.clear_button = clear_button

        self.draw_button.on_clicked(self.draw_polygon)
        self.calc_button.on_clicked(self.calculate_path)
        self.clear_button.on_clicked(self.clear_figure)


    def __call__(self, event):
        print('click', event)
        if event.inaxes!=self.line.axes: return
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)


    def draw_polygon(self, event):
        print("draw_polygon", event)
        if len(self.xs)<3:
            print("Can't make a polygon.")
            return
        for x0, x1, y0, y1 in zip(self.xs, self.xs[1:]+self.xs[0:1],
                                  self.ys, self.ys[1:]+self.ys[0:1]):
            print(x0, x1, y0, y1)
        self.line.set_data(self.xs+self.xs[0:1],self.ys+self.ys[0:1])
        print(self.line.get_xdata())
        print(self.line.get_ydata())
        self.line.figure.canvas.draw()


    def calculate_path(self, event):
        # call srv for calculating a path to plot
        print("calculate_path", event)


    def clear_figure(self, event):
        print("clear_figure", event)
        self.xs = []
        self.ys = []
        self.line.set_data(self.xs,self.ys)
        self.line.figure.canvas.draw()


def init_node():
    rospy.init_node('specify_node', anonymous=True)
    RATE = rospy.Rate(10)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('specify_rect')

    line, = ax.plot([],[])

    axdraw = plt.axes([0.41, 0.01, 0.15, 0.075])
    axcalc = plt.axes([0.61, 0.01, 0.15, 0.075])
    axclear = plt.axes([0.81, 0.01, 0.15, 0.075])

    draw_button = Button(axdraw, 'Draw Polygon')
    calc_button = Button(axcalc, 'Calculate CP')
    clear_button = Button(axclear, 'Clear')

    fig.subplots_adjust(bottom=0.15)

    polygon_builder = PolygonBuilder(line, draw_button, calc_button, clear_button)

    plt.show()


if __name__ == '__main__':
    init_node()