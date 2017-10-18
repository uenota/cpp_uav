#!/usr/bin/env python

from matplotlib import pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import TextBox
import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from cpp_uav.srv import Torres16

class PolygonBuilder(object):
    def __init__(self, line, dot):
        self.line = line
        self.dot = dot
        self.xs = list()
        self.ys = list()
        self.is_polygon_drawn = False
        self.start = None
        self.end = None

        self.footprint_length = Float64(1.0)
        self.footprint_width = Float64(1.0)
        self.horizontal_overwrap = Float64(1.0)
        self.vertical_overwrap = Float64(1.0)

        line.figure.canvas.mpl_connect('button_press_event', self)
        dot.figure.canvas.mpl_connect('button_press_event', self)

        axdraw = plt.axes([0.41, 0.01, 0.15, 0.075])
        axcalc = plt.axes([0.61, 0.01, 0.15, 0.075])
        axclear = plt.axes([0.81, 0.01, 0.15, 0.075])

        self.draw_button = Button(axdraw, 'Draw Polygon')
        self.calc_button = Button(axcalc, 'Calculate CP')
        self.clear_button = Button(axclear, 'Clear')

        self.draw_button.on_clicked(self.draw_polygon)
        self.calc_button.on_clicked(self.calculate_path)
        self.clear_button.on_clicked(self.clear_figure)

        self.footprint_length_box = TextBox(plt.axes([0.85, 0.5, 0.1, 0.075]),
                                            "Length", initial=str(self.footprint_length.data))
        self.footprint_width_box = TextBox(plt.axes([0.85, 0.39, 0.1, 0.075]),
                                           "Width", initial=str(self.footprint_width.data))
        self.horizontal_overwrap_box = TextBox(plt.axes([0.85, 0.28, 0.1, 0.075]),
                                               "Horizontal\nOverwrap", initial=str(self.horizontal_overwrap.data))
        self.vertical_overwrap_box = TextBox(plt.axes([0.85, 0.17, 0.1, 0.075]),
                                             "Vertical\nOverwrap", initial=str(self.vertical_overwrap.data))

        self.footprint_length_box.on_text_change(self.footprint_length_update)
        self.footprint_width_box.on_text_change(self.footprint_width_update)
        self.horizontal_overwrap_box.on_text_change(self.horizontal_overwrap_update)
        self.vertical_overwrap_box.on_text_change(self.vertical_overwrap_update)


    def __call__(self, event):
        if event.inaxes!=self.line.axes:
            return
        if not self.is_polygon_drawn:
            self.xs.append(event.xdata)
            self.ys.append(event.ydata)
            self.dot.set_data(self.xs, self.ys)
            self.dot.figure.canvas.draw()
        elif not self.start:
            self.start = Point()
            self.start.x = event.xdata
            self.start.y = event.ydata
            self.dot.set_data(self.xs+[event.xdata],
                              self.ys+[event.ydata])
            self.dot.figure.canvas.draw()
        elif not self.end:
            self.end = Point()
            self.end.x = event.xdata
            self.end.y = event.ydata
            self.dot.set_data(self.xs+[self.start.x, event.xdata],
                              self.ys+[self.start.y, event.ydata])
            self.dot.figure.canvas.draw()
        else:
            print("Unable to regist points anymore.")


    def draw_polygon(self, event):
        if len(self.xs)<3:
            print("Unable to make a polygon.")
            return
        self.line.set_data(self.xs+self.xs[0:1],self.ys+self.ys[0:1])
        self.line.figure.canvas.draw()
        self.is_polygon_drawn = True


    def calculate_path(self, event):
        rospy.wait_for_service("cpp_torres16")
        if self.start and not self.end:
            self.end = self.start
        vertices = []
        for x, y in zip(self.xs, self.ys):
            point = Point()
            point.x = x
            point.y = y
            vertices.append(point)
        try:
            calc_path_srv = rospy.ServiceProxy("cpp_torres16", Torres16)
            self.waypoints = calc_path_srv(vertices, self.start, self.end,
                                           self.footprint_length,
                                           self.footprint_width,
                                           self.horizontal_overwrap,
                                           self.vertical_overwrap)
        except rospy.ServiceException, e:
            print(str(e))


    def clear_figure(self, event):
        self.xs = []
        self.ys = []
        self.is_polygon_drawn = False
        self.start = None
        self.end = None

        self.dot.set_data(self.xs, self.ys)
        self.line.set_data(self.xs,self.ys)
        self.dot.figure.canvas.draw()
        self.line.figure.canvas.draw()

    def footprint_length_update(self, event):
        if event.isdigit():
            self.footprint_length.data = float(event)


    def footprint_width_update(self, event):
        if event.isdigit():
            self.footprint_width.data = float(event)


    def horizontal_overwrap_update(self, event):
        if event.isdigit():
            self.horizontal_overwrap.data = float(event)


    def vertical_overwrap_update(self, event):
        if event.isdigit():
            self.vertical_overwrap.data = float(event)


def init_node():
    rospy.init_node('specify_node', anonymous=True)
    RATE = rospy.Rate(10)

    fig = plt.figure()
    ax = fig.add_subplot(111)

    solid_line, dot = ax.plot([],[],'-',[],[],'o')

    fig.subplots_adjust(bottom=0.15, right=0.73)

    polygon_builder = PolygonBuilder(solid_line, dot)

    plt.show()


if __name__ == '__main__':
    init_node()