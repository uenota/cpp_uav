#!/usr/bin/env python

from matplotlib import pyplot as plt
from matplotlib.widgets import Button
import rospy

class PolygonBuilder(object):
    def __init__(self, figure, draw_button, calc_button, clear_button):
        self.figure = figure
        self.xs = list()
        self.ys = list()
        self.cid = figure.canvas.mpl_connect('button_press_event', self)
        self.draw_button = draw_button
        self.calc_button = calc_button
        self.clear_button = clear_button
        
        self.draw_button.on_clicked(self.draw_polygon)
        self.calc_button.on_clicked(self.calculate_path)
        self.clear_button.on_clicked(self.clear_figure)

  
    def __call__(self, event):
        print('click', event)
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)


    def draw_polygon(self, event):
        print("draw_polygon", event)
        for i in zip(self.xs, self.ys):
            print(i)

    
    def calculate_path(self, event):
        # call srv for calculating a path to plot
        print("calculate_path", event)        


    def clear_figure(self, event):
        print("clear_figure", event)                


def init_node():
    rospy.init_node('specify_node', anonymous=True)
    RATE = rospy.Rate(10)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('specify_rect')

    axdraw = plt.axes([0.81, 0.3, 0.15, 0.075])
    axcalc = plt.axes([0.81, 0.2, 0.15, 0.075])
    axclear = plt.axes([0.81, 0.1, 0.15, 0.075])

    draw_button = Button(axdraw, 'Draw Polygon')
    calc_button = Button(axcalc, 'Calculate CP')
    clear_button = Button(axclear, 'Clear')

    polygon_builder = PolygonBuilder(fig, draw_button, calc_button, clear_button)
    
    plt.show()


if __name__ == '__main__':
    init_node()