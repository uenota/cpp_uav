#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""! @package cpp_uav
This module offers GUI to specify a polygon for coverage path planning.
"""

# Import python3's print to suppress warning raised by pylint
from __future__ import print_function

# math
import math

# rospy
import rospy

# messages
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

# service
from cpp_uav.srv import Torres16

# Check maplotlib's version
# Exit if older than 2.1.0
import matplotlib
if matplotlib.__version__ >= "2.1.0":
    # matplotlib
    # matplotlib 2.1.0 or newer is required to import TextBox
    from matplotlib import pyplot as plt
    from matplotlib.widgets import Button
    from matplotlib.widgets import TextBox
    from matplotlib.widgets import RadioButtons
else:
    print("Matplotlib 2.1.0 or newer is required to run this node.")
    print("Please update or install Matplotlib.")
    exit(1)


class PolygonBuilder(object):
    """!
    GUI class to specify a polygon
    """

    def __init__(self):
        """! Constructor
        @param axis Axis object where polygon is shown
        """

        ## @var fig
        #  Figure instance
        self.fig = plt.figure(figsize=(8, 6))

        ## @var axis
        #  Axis object where polygon is shown
        self.axis = self.fig.add_subplot(111)

        self.fig.subplots_adjust(top=0.95, bottom=0.35, right=0.79)

        ## @var is_polygon_drawn
        #  True if polygon is illustrated on window
        self.is_polygon_drawn = False

        ## @var server_node
        #  Instance of server
        self.server_node = None

        ## @var lines
        #  Dictionary to store Line2D objects
        #  - line: Line2D object representing edges of a polygon
        #  - dot: Line2D object representing  vertices of a polygon
        #  - path: Line2D object representing coverage path
        self.lines = {"line": self.axis.plot([], [], "-")[0],
                      "dot": self.axis.plot([], [], "o")[0],
                      "path": self.axis.plot([], [], "-")[0]}

        # Register __call__ as callback function for line and dot
        self.lines["line"].figure.canvas.mpl_connect('button_press_event', self)
        self.lines["dot"].figure.canvas.mpl_connect('button_press_event', self)

        ## @var points
        #  Dictionary to store points
        #  - vertices_x: List of x coordinates of vertices
        #  - vertices_y: List of y coordinates of vertices
        #  - start: geometry_msgs/Point object stores info about start point
        #  - end: geometry_msgs/Point object stores info about end point
        #  - waypoints: List of waypoints returned by a coverage path planner
        self.points = {"vertices_x": list(),
                       "vertices_y": list(),
                       "start": None,
                       "end": None,
                       "waypoints": list()}

        ## @var shooting_cond
        #  Dictionary of shooting condition
        #  - image_resolution_h [px]: Vertical resolution of image
        #  - image_resolution_w [px]: Horizontal resolution of image
        #  - angle_of_view [deg]: Camera's angle of view
        #  - height [m]: Flight height
        self.shooting_cond = {"image_resolution_h": 640,
                              "image_resolution_w": 320,
                              "angle_of_view": 45.0,
                              "height": 30.0}

        footprint_width = Float64(2*self.shooting_cond["height"]*
                                  math.tan(self.shooting_cond["angle_of_view"]/2))
        aspect_ratio = \
            float(self.shooting_cond["image_resolution_w"])/self.shooting_cond["image_resolution_h"]

        ## @var coverage_params
        #  Dictionary of coverage params
        #  - footprint_width [m]: Width of footprint
        #  - footprint_length [m]: Length of footprint
        #  - aspect_ratio []: Aspect of image
        #  - horizontal_overwrap [%]: Horizontal overwrap of footprint
        #  - vertical_overwrap [%]: Vertical overwrap of footprint
        #  cf. torres et, al. 2016
        self.coverage_params = {"footprint_width":
                                    footprint_width,
                                "footprint_length":
                                    Float64(footprint_width.data * aspect_ratio),
                                "aspect_ratio":
                                    aspect_ratio,
                                "horizontal_overwrap": Float64(0.3),
                                "vertical_overwrap": Float64(0.2)}

        ## @var algorithm_names
        #  Dictionary of algorithm names and corresponding server names
        self.algorithm_names = {"Torres et, al. 2016":
                                    {"name":"cpp_torres16", "message":Torres16}}

        ## @var algorithm_name
        #  An algorithm name used to calculate path
        self.algorithm_name = self.algorithm_names.keys()[0]

        ## @var algorithm_name_rdbutton
        #  Radio button to choose an algorithm
        self.algorithm_name_rdbutton = RadioButtons(plt.axes([0.75, 0.01, 0.22, 0.21]),
                                                    self.algorithm_names.keys())

        # Register a callback function for a radio button
        self.algorithm_name_rdbutton.on_clicked(self.algorithm_name_update)

        ## @var buttons
        #  Dictionary of buttons
        #  - draw_button: Button object to evoke draw_polygon method
        #  - calc_button: Button object to evoke calculate_path method
        #  - clear_button: Button object to evoke clear_figure method
        self.buttons = {"draw_button":
                            Button(plt.axes([0.8, 0.80, 0.15, 0.075]),
                                   'Draw Polygon'),
                        "calc_button":
                            Button(plt.axes([0.8, 0.69, 0.15, 0.075]),
                                   'Calculate CP'),
                        "clear_button":
                            Button(plt.axes([0.8, 0.58, 0.15, 0.075]),
                                   'Clear')
                       }

        # Register callback functions for buttons
        self.buttons["draw_button"].on_clicked(self.draw_polygon)
        self.buttons["calc_button"].on_clicked(self.calculate_path)
        self.buttons["clear_button"].on_clicked(self.clear_figure)

        # Create textboxes
        ## @var text_boxes
        #  Dictionary of text boxes
        #  - image_resolution_h_box: TextBox object to get input for image_resolution_h
        #  - image_resolution_w_box: TextBox object to get input for image_resolution_w
        #  - angle_of_view_box: TextBox object to get input for angle_of_view
        #  - height_box: TextBox object to get input for height
        #  - horizontal_overwrap_box: TextBox object to get input for horizontal_overwrap
        #  - vertical_overwrap_box: TextBox object to get input for vertical_overwrap
        self.text_boxes = {"image_resolution_h_box":
                               TextBox(plt.axes([0.25, 0.2, 0.1, 0.075]),
                                       "Image Height [px]",
                                       initial=str(self.shooting_cond["image_resolution_h"])),
                           "image_resolution_w_box":
                               TextBox(plt.axes([0.6, 0.2, 0.1, 0.075]),
                                       "Image Width [px]",
                                       initial=str(self.shooting_cond["image_resolution_w"])),
                           "angle_of_view_box":
                               TextBox(plt.axes([0.25, 0.1, 0.1, 0.075]),
                                       "Angle of view [deg]",
                                       initial=str(self.shooting_cond["angle_of_view"])),
                           "height_box":
                               TextBox(plt.axes([0.6, 0.1, 0.1, 0.075]),
                                       "Height [m]",
                                       initial=str(self.shooting_cond["height"])),
                           "horizontal_overwrap_box":
                               TextBox(plt.axes([0.25, 0.01, 0.1, 0.075]),
                                       "Horizontal Overwrap [%]",
                                       initial=\
                                           str(self.coverage_params["horizontal_overwrap"].data)),
                           "vertical_overwrap_box":
                               TextBox(plt.axes([0.6, 0.01, 0.1, 0.075]),
                                       "Vertical Overwrap [%]",
                                       initial=str(self.coverage_params["vertical_overwrap"].data))
                          }

        # Register callback functions for textboxes
        self.text_boxes["image_resolution_h_box"].on_submit(self.image_resolution_h_update)
        self.text_boxes["image_resolution_w_box"].on_submit(self.image_resolution_w_update)
        self.text_boxes["angle_of_view_box"].on_submit(self.angle_of_view_update)
        self.text_boxes["height_box"].on_submit(self.height_update)
        self.text_boxes["horizontal_overwrap_box"].on_submit(self.horizontal_overwrap_update)
        self.text_boxes["vertical_overwrap_box"].on_submit(self.vertical_overwrap_update)

        ## @var labels
        #  Dictionary of labels on figure
        #  - aspect_ratio_text: Text object to display aspect_ratio
        #  - footprint_length_text: Text object to display footprint_length
        #  - footprint_width_text: Text object to display footprint_width
        #  - algorithm_name_text: Text object to display algorithm_name
        #  - start_point: Text object to display start point
        #  - end_point: Text object to display end point
        self.labels = {"aspect_ratio_text":
                           plt.text(2, 6.5,
                                    "Aspect ratio:\n    "
                                    +str(self.coverage_params["aspect_ratio"])),
                       "footprint_length_text":
                           plt.text(2, 5.5,
                                    "Footprint Length [m]:\n    "
                                    +str(round(self.coverage_params["footprint_length"].data, 2))),
                       "footprint_width_text":
                           plt.text(2, 4.5,
                                    "Footprint Width [m]:\n    "
                                    +str(round(self.coverage_params["footprint_width"].data, 2))),
                       "algorithm_name_text":
                           plt.text(1.5, 3,
                                    "Algorithm:\n    "
                                    +self.algorithm_name),
                       "start_point":
                           None,
                       "end_point":
                           None
                      }

        # plot a figure
        plt.show()


    def __call__(self, event):
        """!
        Callback function for button_press_event
        @param event MouseEvent object
        """
        # Return if click event doesn't happen in same axis as which a line lies on
        if event.inaxes != self.lines["line"].axes:
            return
        # true if polygon is not drawn
        if not self.is_polygon_drawn:
            self.points["vertices_x"].append(event.xdata)
            self.points["vertices_y"].append(event.ydata)
            # illustrate a dot
            self.lines["dot"].set_data(self.points["vertices_x"],
                                       self.points["vertices_y"])
            self.lines["dot"].figure.canvas.draw()
        # true if start point is not set
        elif not self.points["start"]:
            self.points["start"] = Point()
            self.points["start"].x = event.xdata
            self.points["start"].y = event.ydata

            self.labels["start_point"] = self.axis.text(event.xdata, event.ydata, "Start")
            self.lines["dot"].set_data(self.points["vertices_x"]+[event.xdata],
                                       self.points["vertices_y"]+[event.ydata])
            self.lines["dot"].figure.canvas.draw()
        # true if end point is not set
        elif not self.points["end"]:
            self.points["end"] = Point()
            self.points["end"].x = event.xdata
            self.points["end"].y = event.ydata

            self.labels["end_point"] = self.axis.text(event.xdata, event.ydata, "Goal")
            self.lines["dot"].set_data(self.points["vertices_x"]+[self.points["start"].x, event.xdata],
                                       self.points["vertices_y"]+[self.points["start"].y, event.ydata])
            self.lines["dot"].figure.canvas.draw()
        else:
            rospy.logwarn("Unable to register points anymore.")


    def update_params(self):
        """!
        Update values of coverage parameters
        """
        self.coverage_params["aspect_ratio"] = \
            float(self.shooting_cond["image_resolution_w"])/self.shooting_cond["image_resolution_h"]
        self.coverage_params["footprint_width"] = \
            Float64(2*self.shooting_cond["height"]*
                    math.tan(self.shooting_cond["angle_of_view"]/2))
        self.coverage_params["footprint_length"] = \
            Float64(self.coverage_params["footprint_width"].data/
                    self.coverage_params["aspect_ratio"])


    def update_param_texts(self):
        """!
        Update texts of coverage parameters
        """
        self.labels["aspect_ratio_text"].set_text("Aspect ratio:\n    "+
                                                  str(self.coverage_params["aspect_ratio"]))
        self.labels["footprint_length_text"].set_text("Footprint Length [m]:\n    "+
                                                      str(round(self.coverage_params["footprint_length"].data, 2)))
        self.labels["footprint_width_text"].set_text("Footprint Width [m]:\n    "+
                                                     str(round(self.coverage_params["footprint_width"].data, 2)))
        self.labels["footprint_length_text"].figure.canvas.draw()


    def draw_polygon(self, event):
        """!
        Callback function for "Draw Polygon" button
        @param event MouseEvent object
        """
        # Return if vertices is less than 3
        if len(self.points["vertices_x"]) < 3:
            rospy.logerr("Unable to make a polygon.")
            return
        # Draw a polygon
        self.lines["line"].set_data(self.points["vertices_x"]+self.points["vertices_x"][0:1],
                                    self.points["vertices_y"]+self.points["vertices_y"][0:1])
        self.lines["line"].figure.canvas.draw()
        # Set flag as True
        self.is_polygon_drawn = True


    def calculate_path(self, event):
        """!
        Callback function for "Calculate CP" button
        @param event MouseEvent object
        """
        if not self.is_polygon_drawn:
            return

        # assign server node if server node is None
        if not self.server_node:
            rospy.loginfo("Waiting for %s.", self.algorithm_name)
            try:
                rospy.wait_for_service(self.algorithm_names[self.algorithm_name]["name"],
                                       timeout=5.0)
            except rospy.ROSException:
                rospy.logerr("%s not found.", self.algorithm_name)
                return
            try:
                self.server_node = rospy.ServiceProxy(
                    self.algorithm_names[self.algorithm_name]["name"],
                    self.algorithm_names[self.algorithm_name]["message"])
            except rospy.ServiceException as ex:
                rospy.logerr(str(ex))
                return

        # Set end point as same as start point
        # if start point is set and not end point is set
        if self.points["start"] and not self.points["end"]:
            # it's ok that start and end are the same object
            self.points["end"] = self.points["start"]
        # Create a list of vertices
        vertices = []
        waypoint_xs = []
        waypoint_ys = []
        for x_coord, y_coord in zip(self.points["vertices_x"],
                                    self.points["vertices_y"]):
            point = Point()
            point.x = x_coord
            point.y = y_coord
            vertices.append(point)
        # Call service
        try:
            self.points["waypoints"] = self.server_node(vertices,
                                                        self.points["start"],
                                                        self.points["end"],
                                                        self.coverage_params["footprint_length"],
                                                        self.coverage_params["footprint_width"],
                                                        self.coverage_params["horizontal_overwrap"],
                                                        self.coverage_params["vertical_overwrap"]).waypoints
            for point in self.points["waypoints"]:
                waypoint_xs.append(point.x)
                waypoint_ys.append(point.y)
            self.lines["path"].set_data(waypoint_xs, waypoint_ys)
            self.lines["path"].figure.canvas.draw()
        except rospy.ServiceException as ex:
            rospy.logerr(str(ex))
            return


    def clear_figure(self, event):
        """!
        Callback function for "Clear" button
        @param event MouseEvent object
        """

        if self.points["start"]:
            self.labels["start_point"].remove()
        if self.points["end"] and self.points["start"] != self.points["end"]:
            self.labels["end_point"].remove()

        # Clear lists of vertices' coordinates
        self.points["vertices_x"] = []
        self.points["vertices_y"] = []
        # Set flag as False
        self.is_polygon_drawn = False
        # Clear start point and end point
        self.points["start"] = None
        self.points["end"] = None

        # Clear waypoints
        self.points["waypoints"] = []

        # Refresh a canvas
        self.lines["dot"].set_data(self.points["vertices_x"], self.points["vertices_y"])
        self.lines["line"].set_data(self.points["vertices_x"], self.points["vertices_y"])
        self.lines["path"].set_data([], [])

        self.lines["dot"].figure.canvas.draw()
        self.lines["line"].figure.canvas.draw()
        self.lines["path"].figure.canvas.draw()


    def image_resolution_h_update(self, event):
        """!
        Called when content of "Image Height" is submitted
        @param event Content of TextBox
        """
        # Update parameter if input is digit
        if event.isdigit():
            self.shooting_cond["image_resolution_h"] = int(event)
            self.update_params()
            self.update_param_texts()
        else:
            self.text_boxes["image_resolution_h_box"].\
                set_val(str(self.shooting_cond["image_resolution_h"]))


    def image_resolution_w_update(self, event):
        """!
        Called when content of "Image Width" is submitted
        @param event Content of TextBox
        """
        # Update parameter if input is digit
        if event.isdigit():
            self.shooting_cond["image_resolution_w"] = int(event)
            self.update_params()
            self.update_param_texts()
        else:
            self.text_boxes["image_resolution_w_box"].\
                set_val(str(self.shooting_cond["image_resolution_w"]))


    def angle_of_view_update(self, event):
        """!
        Called when content of "Angle of view" is submitted
        @param event Content of TextBox
        """
        # Update parameter if input is digit
        if event.isdigit():
            self.shooting_cond["angle_of_view"] = float(event)
            self.update_params()
            self.update_param_texts()
        else:
            self.text_boxes["angle_of_view_box"]\
                .set_val(str(self.shooting_cond["angle_of_view"]))


    def height_update(self, event):
        """!
        Called when content of "Height" is submitted
        @param event Content of TextBox
        """
        # Update parameter if input is digit
        if event.isdigit():
            self.shooting_cond["height"] = float(event)
            self.update_params()
            self.update_param_texts()
        else:
            self.text_boxes["height_box"].\
                set_val(str(self.shooting_cond["height"]))


    def horizontal_overwrap_update(self, event):
        """!
        Called when content of "Horizontal overwrap" is submitted
        @param event Content of TextBox
        """
        # Update parameter if input is digit
        if event.isdigit():
            if 0 < float(event) < 1.0:
                self.coverage_params["horizontal_overwrap"].data = float(event)
        else:
            self.text_boxes["horizontal_overwrap_box"].\
                set_val(str(self.coverage_params["horizontal_overwrap"]))


    def vertical_overwrap_update(self, event):
        """!
        Called when content of "Vertical overwrap" is submitted
        @param event Content of TextBox
        """
        # Update parameter if input is digit
        if event.isdigit():
            if 0 < float(event) < 1.0:
                self.coverage_params["vertical_overwrap"].data = float(event)
        else:
            self.text_boxes["vertical_overwrap_box"].\
                set_val(str(self.coverage_params["vertical_overwrap"]))


    def algorithm_name_update(self, label):
        """!
        Callback function for algorithm_name_rdbutton
        @param label Label of a chosen item
        """
        self.algorithm_name = label
        rospy.loginfo("Waiting for %s.", self.algorithm_name)
        try:
            rospy.wait_for_service(self.algorithm_names[self.algorithm_name]["name"], timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("%s not found.", self.algorithm_name)
            return
        try:
            self.server_node = rospy.ServiceProxy(
                self.algorithm_names[self.algorithm_name]["name"],
                self.algorithm_names[self.algorithm_name]["message"])
        except rospy.ServiceException as ex:
            rospy.logerr(str(ex))
            return


def init_node():
    """!
    Initialize a node
    """
    rospy.init_node('specify_node', anonymous=True)

    # Call PolygonBuilder's constructor
    PolygonBuilder()


if __name__ == '__main__':
    init_node()
