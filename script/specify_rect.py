#!/usr/bin/env python
# -*- coding: utf-8 -*-

## @package cpp_uav
#  This module offers GUI to specify a polygon for coverage path planning.


from __future__ import print_function

# math
import math

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

## GUI class to specify a polygon
class PolygonBuilder(object):

    ## Constructor
    #  @param axis Axis objcect where polygon is shown
    def __init__(self, axis):
        ## @var axis
        #  Axis object where polygon is shown
        self.axis = axis

        ## @var line
        #  Line2D object representing edges of a polygon
        self.line = axis.plot([], [], "-")[0]

        ## @var dot
        #  Line2D object representing  vertices of a polygon
        self.dot = axis.plot([], [], "o")[0]

        ## @var path
        #  Line2D object representing coverage path
        self.path = axis.plot([], [], "-")[0]

        ## @var vertices_x
        #  List of x coordinates of vertices
        self.vertices_x = list()
        ## @var vertices_y
        #  List of y coordinates of vertices
        self.vertices_y = list()

        ## @var is_polygon_drawn
        #  True if polygon is illustrated on window
        self.is_polygon_drawn = False

        ## @var start
        #  Start point of trajectory
        self.start = None
        ## @var end
        #  End point of trajectory
        self.end = None

        ## @var text_start
        #  Label for start point
        self.text_start = ""
        ## @var text_end
        #  Label for end point
        self.text_end = ""

        ## @var waypoints
        #  List of waypoints in coverage path
        self.waypoints = list()

        ## @var image_resolution_h
        #  Vertical resolution of image
        self.image_resolution_h = 640
        ## @var image_resolution_w
        #  Horizontal resolution of image
        self.image_resolution_w = 320
        ## @var angle_of_view
        #  Camera's angle of view
        self.angle_of_view = 45.0
        ## @var height
        #  Flight height of UAV
        self.height = 30.0

        # Coverage specification
        # cf. torres et, al. 2016
        ## @var aspect_ratio
        #  Image's aspect ratio
        self.aspect_ratio = float(self.image_resolution_w) / self.image_resolution_h

        ## @var footprint_width
        #  Width of camera footprint
        self.footprint_width = Float64(2*self.height*math.tan(self.angle_of_view/2))
        ## @var footprint_length
        #  Length of camera footprint
        self.footprint_length = Float64(self.footprint_width.data / self.aspect_ratio)
        ## @var horizontal_overwrap
        #  Horizontal overwrap of image
        self.horizontal_overwrap = Float64(0.3)
        ## @var vertical_overwrap
        #  Vertical overwrap of image
        self.vertical_overwrap = Float64(0.2)

        # Register __call__ as callback function for line and dot
        self.line.figure.canvas.mpl_connect('button_press_event', self)
        self.dot.figure.canvas.mpl_connect('button_press_event', self)

        # Create buttons
        ## @var draw_button
        #  Button object for "Draw Polygon" button
        self.draw_button = Button(plt.axes([0.8, 0.80, 0.15, 0.075]),
                                  'Draw Polygon')
        ## @var calc_button
        #  Button object for "Calculate CP" button
        self.calc_button = Button(plt.axes([0.8, 0.69, 0.15, 0.075]),
                                  'Calculate CP')
        ## @var clear_button
        #  Button object for "Clear" button
        self.clear_button = Button(plt.axes([0.8, 0.58, 0.15, 0.075]),
                                   'Clear')

        # Register callback functions for buttons
        self.draw_button.on_clicked(self.draw_polygon)
        self.calc_button.on_clicked(self.calculate_path)
        self.clear_button.on_clicked(self.clear_figure)

        # Create textboxes
        ## @var image_resolution_h_box
        #  TextBox object for variable image_resolution_h
        self.image_resolution_h_box = TextBox(plt.axes([0.4, 0.2, 0.1, 0.075]),
                                              "Image Height [px]",
                                              initial=str(self.image_resolution_h))
        ## @var image_resolution_w_box
        #  TextBox object for variable image_resolution_w
        self.image_resolution_w_box = TextBox(plt.axes([0.8, 0.2, 0.1, 0.075]),
                                              "Image Width [px]",
                                              initial=str(self.image_resolution_w))
        ## @var angle_of_view_box
        #  TextBox object for variable angle_of_view
        self.angle_of_view_box = TextBox(plt.axes([0.4, 0.1, 0.1, 0.075]),
                                         "Angle of view [rad]",
                                         initial=str(self.angle_of_view))
        ## @var height_box
        #  TextBox object for variable height
        self.height_box = TextBox(plt.axes([0.8, 0.1, 0.1, 0.075]),
                                  "Height [m]",
                                  initial=str(self.height))
        ## @var horizontal_overwrap_box
        #  TextBox object for variable horizontal_overwrap
        self.horizontal_overwrap_box = TextBox(plt.axes([0.4, 0.01, 0.1, 0.075]),
                                               "Horizontal Overwrap [%]",
                                               initial=str(self.horizontal_overwrap.data))
        ## @var vertical_overwrap_box
        #  TextBox object for variable vertical_overwrap
        self.vertical_overwrap_box = TextBox(plt.axes([0.8, 0.01, 0.1, 0.075]),
                                             "Vertical Overwrap [%]",
                                             initial=str(self.vertical_overwrap.data))

        # Register callback functions for textboxes
        self.image_resolution_h_box.on_submit(self.image_resolution_h_update)
        self.image_resolution_w_box.on_submit(self.image_resolution_w_update)
        self.angle_of_view_box.on_submit(self.angle_of_view_update)
        self.height_box.on_submit(self.height_update)
        self.horizontal_overwrap_box.on_submit(self.horizontal_overwrap_update)
        self.vertical_overwrap_box.on_submit(self.vertical_overwrap_update)

        # Texts about coverage specification
        ## @var aspect_ratio_text
        #  Label for displaying aspect ratio of image
        self.aspect_ratio_text = plt.text(0.05, 6.5, "Aspect ratio:\n    "+str(self.aspect_ratio))
        ## @var footprint_length_text
        #  Label for displaying length of camera footprint
        self.footprint_length_text = plt.text(0.05, 5.5,
                                              "Footprint Length [m]:\n    "
                                              +str(round(self.footprint_length.data, 2)))
        ## @var footprint_width_text
        #  Label for displaying width of camera footprint
        self.footprint_width_text = plt.text(0.05, 4.5,
                                             "Footprint Width [m]:\n    "
                                             +str(round(self.footprint_width.data, 2)))


    ## Callback function for button_press_event
    # @param event MouseEvent object
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

            self.text_start = self.axis.text(event.xdata, event.ydata, "Start")
            self.dot.set_data(self.vertices_x+[event.xdata],
                              self.vertices_y+[event.ydata])
            self.dot.figure.canvas.draw()
        # true if end point is not set
        elif not self.end:
            self.end = Point()
            self.end.x = event.xdata
            self.end.y = event.ydata

            self.text_end = self.axis.text(event.xdata, event.ydata, "Goal")
            self.dot.set_data(self.vertices_x+[self.start.x, event.xdata],
                              self.vertices_y+[self.start.y, event.ydata])
            self.dot.figure.canvas.draw()
        else:
            print("Unable to register points anymore.")


    ## Update values of coverage parameters
    def update_params(self):
        self.aspect_ratio = float(self.image_resolution_w) / self.image_resolution_h
        self.footprint_width = Float64(2*self.height*math.tan(self.angle_of_view/2))
        self.footprint_length = Float64(self.footprint_width.data / self.aspect_ratio)


    ## Update texts of coverage parameters
    def update_param_texts(self):
        self.aspect_ratio_text.set_text("Aspect ratio:\n    "+str(self.aspect_ratio))
        self.footprint_length_text.set_text("Footprint Length [m]:\n    "
                                            +str(round(self.footprint_length.data, 2)))
        self.footprint_width_text.set_text("Footprint Width [m]:\n    "
                                           +str(round(self.footprint_width.data, 2)))
        self.footprint_length_text.figure.canvas.draw()


    ## Callback function for "Draw Polygon" button
    #  @param event MouseEvent object
    def draw_polygon(self, event):
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


    ## Callback function for "Calculate CP" button
    #  @param event MouseEvent object
    def calculate_path(self, event):
        if not self.is_polygon_drawn:
            return
        rospy.wait_for_service("cpp_torres16")
        # Set end point as same as start point
        # if start point is set and not end point is set
        if self.start and not self.end:
            self.end = self.start
        # Create a list of vertices
        vertices = []
        xs = []
        ys = []
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
                                           self.vertical_overwrap).waypoints
            for point in self.waypoints:
                xs.append(point.x)
                ys.append(point.y)
            self.path.set_data(xs, ys)
            self.path.figure.canvas.draw()
        except rospy.ServiceException, e:
            print(str(e))


    ## Callback function for "Clear" button
    #  @param event MouseEvent object
    def clear_figure(self, event):
        # Clear lists of vertices' coordinates
        self.vertices_x = []
        self.vertices_y = []
        # Set flag as False
        self.is_polygon_drawn = False
        # Clear start point and end point
        self.start = None
        self.end = None

        # Clear waypoints
        self.waypoints = []

        # Reflesh a canvas
        self.dot.set_data(self.vertices_x, self.vertices_y)
        self.line.set_data(self.vertices_x, self.vertices_y)
        self.path.set_data([], [])
        self.dot.figure.canvas.draw()
        self.line.figure.canvas.draw()
        self.path.figure.canvas.draw()

        self.text_start.remove()
        self.text_end.remove()


    ## Called when content of "Image Height" is submitted
    #  @param event Content of TextBox
    def image_resolution_h_update(self, event):
        # Update parameter if input is digit
        if event.isdigit():
            self.image_resolution_h = int(event)
            self.update_params()
            self.update_param_texts()
        else:
            self.image_resolution_h_box.set_val(str(self.image_resolution_h))


    ## Called when content of "Image Width" is submitted
    #  @param event Content of TextBox
    def image_resolution_w_update(self, event):
        # Update parameter if input is digit
        if event.isdigit():
            self.image_resolution_w = int(event)
            self.update_params()
            self.update_param_texts()
        else:
            self.image_resolution_w_box.set_val(str(self.image_resolution_w))


    ## Called when content of "Angle of view" is submitted
    #  @param event Content of TextBox
    def angle_of_view_update(self, event):
        # Update parameter if input is digit
        if event.isdigit():
            self.angle_of_view = float(event)
            self.update_params()
            self.update_param_texts()
        else:
            self.angle_of_view_box.set_val(str(self.angle_of_view))


    ## Called when content of "Height" is submitted
    #  @param event Content of TextBox
    def height_update(self, event):
        # Update parameter if input is digit
        if event.isdigit():
            self.height = float(event)
            self.update_params()
            self.update_param_texts()
        else:
            self.height_box.set_val(str(self.height))


    ## Called when content of "Horizontal overwrap" is submitted
    #  @param event Content of TextBox
    def horizontal_overwrap_update(self, event):
        # Update parameter if input is digit
        if event.isdigit():
            if 0 < float(event) < 1.0:
                self.horizontal_overwrap.data = float(event)
        else:
            self.horizontal_overwrap_box.set_val(str(self.horizontal_overwrap))

    ## Called when content of "Vertical overwrap" is submitted
    #  @param event Content of TextBox
    def vertical_overwrap_update(self, event):
        # Update parameter if input is digit
        if event.isdigit():
            if 0 < float(event) < 1.0:
                self.vertical_overwrap.data = float(event)
        else:
            self.vertical_overwrap_box.set_val(str(self.vertical_overwrap))


## Initialize a ros node
def init_node():
    """
    Initialize a node
    """
    rospy.init_node('specify_node', anonymous=True)

    fig = plt.figure(figsize=(8,6))
    axis = fig.add_subplot(111)

    fig.subplots_adjust(top=0.95, bottom=0.35, right=0.79)

    polygon_builder = PolygonBuilder(axis)

    plt.show()


if __name__ == '__main__':
    init_node()
