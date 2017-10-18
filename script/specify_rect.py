#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This module offers GUI to specify a polygon for coverage path planning.
"""

from __future__ import print_function

# matplotlib
# matplotlib 2.1.0 or newer is required to import TextBox
from matplotlib import pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import TextBox

# rospy
import rospy

# messages
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

# service
from cpp_uav.srv import Torres16


class PolygonBuilder(object):
    """
    GUI class to specify a polygon.
    """
    def __init__(self, line, dot):
        # Edges and vertices of polygon
        self.line = line
        self.dot = dot

        # coordinates of vertices
        self.vertices_x = list()
        self.vertices_y = list()

        # True if polygon is illustrated on GUI
        self.is_polygon_drawn = False

        # Start and end point of trajectory
        self.start = None
        self.end = None

        # Waypoints of coverage path
        self.waypoints = list()

        # Coverage specification
        # cf. torres et, al. 2016
        self.footprint_length = Float64(1.0)
        self.footprint_width = Float64(1.0)
        self.horizontal_overwrap = Float64(1.0)
        self.vertical_overwrap = Float64(1.0)

        # Register __call__ as callback function for line and dot
        line.figure.canvas.mpl_connect('button_press_event', self)
        dot.figure.canvas.mpl_connect('button_press_event', self)

        # Create buttons
        self.draw_button = Button(plt.axes([0.41, 0.01, 0.15, 0.075]),
                                  'Draw Polygon')
        self.calc_button = Button(plt.axes([0.61, 0.01, 0.15, 0.075]),
                                  'Calculate CP')
        self.clear_button = Button(plt.axes([0.81, 0.01, 0.15, 0.075]),
                                   'Clear')

        # Register callback functions for buttons
        self.draw_button.on_clicked(self.draw_polygon)
        self.calc_button.on_clicked(self.calculate_path)
        self.clear_button.on_clicked(self.clear_figure)

        # Create textboxes
        self.footprint_length_box = TextBox(plt.axes([0.85, 0.5, 0.1, 0.075]),
                                            "Length",
                                            initial=str(self.footprint_length.data))
        self.footprint_width_box = TextBox(plt.axes([0.85, 0.39, 0.1, 0.075]),
                                           "Width",
                                           initial=str(self.footprint_width.data))
        self.horizontal_overwrap_box = TextBox(plt.axes([0.85, 0.28, 0.1, 0.075]),
                                               "Horizontal\nOverwrap",
                                               initial=str(self.horizontal_overwrap.data))
        self.vertical_overwrap_box = TextBox(plt.axes([0.85, 0.17, 0.1, 0.075]),
                                             "Vertical\nOverwrap",
                                             initial=str(self.vertical_overwrap.data))

        # Register callback functions for textboxes
        self.footprint_length_box.on_text_change(self.footprint_length_update)
        self.footprint_width_box.on_text_change(self.footprint_width_update)
        self.horizontal_overwrap_box.on_text_change(self.horizontal_overwrap_update)
        self.vertical_overwrap_box.on_text_change(self.vertical_overwrap_update)


    def __call__(self, event):
        # Return if click event doesn't happen in same axis as which a line lies on
        if event.inaxes != self.line.axes:
            return
        # true if polygon is not drawn
        if not self.is_polygon_drawn:
            self.vertices_x.append(event.xdata)
            self.vertices_y.append(event.ydata)
            # illustrate a dot
            self.dot.set_data(self.vertices_x, self.vertices_y)
            self.dot.figure.canvas.draw()
        # true if start point is not set
        elif not self.start:
            self.start = Point()
            self.start.x = event.xdata
            self.start.y = event.ydata
            self.dot.set_data(self.vertices_x+[event.xdata],
                              self.vertices_y+[event.ydata])
            self.dot.figure.canvas.draw()
        # true if end point is not set
        elif not self.end:
            self.end = Point()
            self.end.x = event.xdata
            self.end.y = event.ydata
            self.dot.set_data(self.vertices_x+[self.start.x, event.xdata],
                              self.vertices_y+[self.start.y, event.ydata])
            self.dot.figure.canvas.draw()
        else:
            print("Unable to register points anymore.")


    def draw_polygon(self, event):
        """
        Callback function for "Draw Polygon" button.
        """
        # Return if vertices is less than 3
        if len(self.vertices_x) < 3:
            print("Unable to make a polygon.")
            return
        # Draw a polygon
        self.line.set_data(self.vertices_x+self.vertices_x[0:1],
                           self.vertices_y+self.vertices_y[0:1])
        self.line.figure.canvas.draw()
        # Set flag as True
        self.is_polygon_drawn = True


    def calculate_path(self, event):
        """
        Callback function for "Calculate Path" button.
        """
        rospy.wait_for_service("cpp_torres16")
        # Set end point as same as start point 
        # if start point is set and not end point is set
        if self.start and not self.end:
            self.end = self.start
        # Create a list of vertices
        vertices = []
        for x, y in zip(self.vertices_x, self.vertices_y):
            point = Point()
            point.x = x
            point.y = y
            vertices.append(point)
        # Call service
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
        """
        Callback function for "Clear" button.
        """
        # Clear lists of vertices' coordinates
        self.vertices_x = []
        self.vertices_y = []
        # Set flag as False
        self.is_polygon_drawn = False
        # Clear start point and end point
        self.start = None
        self.end = None

        # Reflesh a canvas
        self.dot.set_data(self.vertices_x, self.vertices_y)
        self.line.set_data(self.vertices_x, self.vertices_y)
        self.dot.figure.canvas.draw()
        self.line.figure.canvas.draw()

    def footprint_length_update(self, event):
        """
        This callback function is called
        when content of "Length" is changed.
        """
        # Update parameter if input is digit
        if event.isdigit():
            self.footprint_length.data = float(event)


    def footprint_width_update(self, event):
        """
        This callback function is called
        when content of "Width" is changed.
        """
        # Update parameter if input is digit        
        if event.isdigit():
            self.footprint_width.data = float(event)


    def horizontal_overwrap_update(self, event):
        """
        This callback function is called
        when content of "Horizontal overwrap" is changed.
        """
        # Update parameter if input is digit        
        if event.isdigit():
            self.horizontal_overwrap.data = float(event)


    def vertical_overwrap_update(self, event):
        """
        This callback function is called
        when content of "Vertical overwrap" is changed.
        """
        # Update parameter if input is digit
        if event.isdigit():
            self.vertical_overwrap.data = float(event)


def init_node():
    """
    Initialize a node
    """
    rospy.init_node('specify_node', anonymous=True)

    fig = plt.figure()
    axis = fig.add_subplot(111)

    solid_line, dot = axis.plot([], [], '-', [], [], 'o')

    fig.subplots_adjust(bottom=0.15, right=0.73)

    polygon_builder = PolygonBuilder(solid_line, dot)

    plt.show()


if __name__ == '__main__':
    init_node()
