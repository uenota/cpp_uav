#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2017 Takaki Ueno
# Released under the MIT license

"""! @package cpp_uav
This module offers GUI to specify a polygon for coverage path planning.
"""

# Import python3's print to suppress warning raised by pylint
from __future__ import print_function

# python libraries
import math

import numpy as np

# Import urllib considering version of Python
try:
    from urllib.request import urlopen, HTTPError
except ImportError:
    from urllib2 import urlopen, HTTPError

# PIL
from PIL import Image

# rospy
import rospy

# messages
from geometry_msgs.msg import Point, Polygon
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix

# service
from cpp_uav.srv import Torres16

# Check maplotlib's version
# Exit if older than 2.1.0
import matplotlib
if matplotlib.__version__ >= "2.1.0":
    # matplotlib
    # matplotlib 2.1.0 or newer is required to import TextBox
    from matplotlib import pyplot as plt
    from matplotlib.patches import Polygon
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
        """

        # @var fig
        #  Figure instance
        self.fig = plt.figure(figsize=(10, 8))

        # @var axis
        #  Axis object where polygon is shown
        self.axis = self.fig.add_subplot(111)

        # adjust the size of graph
        self.axis.set_ylim([-100, 100])
        self.axis.set_xlim([-100, 100])

        # disable ticks of axis
        self.axis.axis('off')

        # set aspect ratio so that aspect ration of graph become 1
        self.axis.set_aspect('equal', adjustable="box")

        self.fig.subplots_adjust(top=0.95, bottom=0.35, right=0.79)

        # @var is_polygon_drawn
        #  True if polygon is illustrated on window
        self.is_polygon_drawn = False

        # @var server_node
        #  Instance of server
        self.server_node = None

        # @var lines
        #  Dictionary to store Line2D objects
        #  - line: Line2D object representing edges of a polygon
        #  - dot: Line2D object representing  vertices of a polygon
        #  - path: Line2D object representing coverage path
        self.lines = {"line": self.axis.plot([], [], "-")[0],
                      "dot": self.axis.plot([], [], "o")[0],
                      "path": self.axis.plot([], [], "-")[0],
                      "subpolygons": None}

        # Register __call__ as callback function for line and dot
        self.lines["line"].figure.canvas.mpl_connect(
            'button_press_event', self)
        self.lines["dot"].figure.canvas.mpl_connect('button_press_event', self)

        # @var points
        #  Dictionary to store points
        #  - vertices_x: List of x coordinates of vertices
        #  - vertices_y: List of y coordinates of vertices
        #  - start: geometry_msgs/Point object stores info about start point
        #  - waypoints: List of waypoints returned by a coverage path planner
        self.points = {"vertices_x": list(),
                       "vertices_y": list(),
                       "start": None,
                       "waypoints": list()}

        self.subpolygons = []
        self.patches = []

        # @var shooting_cond
        #  Dictionary of shooting condition
        #  - image_resolution_h [px]: Vertical resolution of image
        #  - image_resolution_w [px]: Horizontal resolution of image
        #  - angle_of_view [deg]: Camera's angle of view
        #  - height [m]: Flight height
        self.shooting_cond = {"image_resolution_h": 640,
                              "image_resolution_w": 320,
                              "angle_of_view": 45.0,
                              "height": 30.0}

        footprint_width = Float64(2 * self.shooting_cond["height"] *
                                  math.tan(self.shooting_cond["angle_of_view"] / 2))
        aspect_ratio = \
            float(self.shooting_cond["image_resolution_w"]) \
            / self.shooting_cond["image_resolution_h"]

        # @var coverage_params
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
                                    Float64(footprint_width.data *
                                            aspect_ratio),
                                "aspect_ratio":
                                    aspect_ratio,
                                "horizontal_overwrap": Float64(0.3),
                                "vertical_overwrap": Float64(0.2)}

        # @var map_param
        #  Dictionary of map params
        #  This program uses Google Static Map API.
        #  See the following links for the details of fields of this dictionary
        #  - https://developers.google.com/maps/documentation/static-maps/intro?hl=ja#Usage
        #  - https://www.ajaxtower.jp/googlestaticmaps/
        self.map_param = {"url": "https://maps.google.com/maps/api/staticmap",
                          "zoom": 10,
                          "size": self.get_canvas_size(),
                          "sensor": "false",
                          "maptype": "hybrid",
                          "latitude": 43.042827,
                          "longitude": 141.202698,
                          "background_image": None}

        # @var buttons
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
                                   'Clear'),
                        "zoom_in_button":
                            Button(plt.axes([0.15, 0.425, 0.05, 0.05]),
                                   '+'),
                        "zoom_out_button":
                            Button(plt.axes([0.15, 0.35, 0.05, 0.05]),
                                   '-')}

        # Register callback functions for buttons
        self.buttons["draw_button"].on_clicked(self.draw_polygon)
        self.buttons["calc_button"].on_clicked(self.calculate_path)
        self.buttons["clear_button"].on_clicked(self.clear_figure)
        self.buttons["zoom_in_button"].on_clicked(self.zoom_in)
        self.buttons["zoom_out_button"].on_clicked(self.zoom_out)

        # @var maptype_rdbutton
        #  Radio Button for selecting type of map
        #  See the following link for the details of map type
        #  https://developers.google.com/maps/documentation/static-maps/intro?hl=ja#MapTypes
        self.maptype_rdbutton = RadioButtons(plt.axes([0.07, 0.5, 0.13, 0.2]),
                                             ("roadmap", "satellite",
                                              "terrain", "hybrid"),
                                             active=3)
        self.maptype_rdbutton.on_clicked(self.maptype_update)

        # Create textboxes
        # @var text_boxes
        #  Dictionary of text boxes
        #  - image_resolution_h_box: TextBox object to get input for image_resolution_h
        #  - image_resolution_w_box: TextBox object to get input for image_resolution_w
        #  - angle_of_view_box: TextBox object to get input for angle_of_view
        #  - height_box: TextBox object to get input for height
        #  - horizontal_overwrap_box: TextBox object to get input for horizontal_overwrap
        #  - vertical_overwrap_box: TextBox object to get input for vertical_overwrap
        self.text_boxes = {"image_resolution_h_box":
                           TextBox(plt.axes([0.25, 0.25, 0.1, 0.05]),
                                   "Image Height [px]",
                                   initial=str(self.shooting_cond["image_resolution_h"])),
                           "image_resolution_w_box":
                               TextBox(plt.axes([0.6, 0.25, 0.1, 0.05]),
                                       "Image Width [px]",
                                       initial=str(self.shooting_cond["image_resolution_w"])),
                           "angle_of_view_box":
                               TextBox(plt.axes([0.25, 0.175, 0.1, 0.05]),
                                       "Angle of view [deg]",
                                       initial=str(self.shooting_cond["angle_of_view"])),
                           "height_box":
                               TextBox(plt.axes([0.6, 0.175, 0.1, 0.05]),
                                       "Height [m]",
                                       initial=str(self.shooting_cond["height"])),
                           "horizontal_overwrap_box":
                               TextBox(plt.axes([0.25, 0.1, 0.1, 0.05]),
                                       "Horizontal Overwrap [%]",
                                       initial=str(self.coverage_params["horizontal_overwrap"].data)),
                           "vertical_overwrap_box":
                               TextBox(plt.axes([0.6, 0.1, 0.1, 0.05]),
                                       "Vertical Overwrap [%]",
                                       initial=str(self.coverage_params["vertical_overwrap"].data)),
                           "latitude_box":
                               TextBox(plt.axes([0.85, 0.175, 0.1, 0.05]),
                                       "Latitude",
                                       initial=str(self.map_param["latitude"])),
                           "longitude_box":
                               TextBox(plt.axes([0.85, 0.25, 0.1, 0.05]),
                                       "Longitude",
                                       initial=str(self.map_param["longitude"]))}

        # Register callback functions for textboxes
        self.text_boxes["image_resolution_h_box"].on_submit(
            self.image_resolution_h_update)
        self.text_boxes["image_resolution_w_box"].on_submit(
            self.image_resolution_w_update)
        self.text_boxes["angle_of_view_box"].on_submit(
            self.angle_of_view_update)
        self.text_boxes["height_box"].on_submit(self.height_update)
        self.text_boxes["horizontal_overwrap_box"].on_submit(
            self.horizontal_overwrap_update)
        self.text_boxes["vertical_overwrap_box"].on_submit(
            self.vertical_overwrap_update)
        self.text_boxes["latitude_box"].on_submit(self.latitude_update)
        self.text_boxes["longitude_box"].on_submit(self.longitude_update)

        # @var labels
        #  Dictionary of labels on figure
        #  - aspect_ratio_text: Text object to display aspect_ratio
        #  - footprint_length_text: Text object to display footprint_length
        #  - footprint_width_text: Text object to display footprint_width
        #  - mode_text: Text object to display mode
        #  - start_point: Text object to display start point
        #  - SP: Start of waypoints
        #  - EP: End of waypoints
        self.labels = {"aspect_ratio_text":
                       plt.text(-0.5, 5,
                                "Aspect ratio:\n    " + str(self.coverage_params["aspect_ratio"])),
                       "footprint_length_text":
                           plt.text(-0.5, 4,
                                    "Footprint Length [m]:\n    " +
                                    str(round(self.coverage_params["footprint_length"].data, 2))),
                       "footprint_width_text":
                           plt.text(-0.5, 3,
                                    "Footprint Width [m]:\n    " +
                                    str(round(self.coverage_params["footprint_width"].data, 2))),
                       "start_point": None,
                       "SP": None,
                       "EP": None}

        # wait for the position of drone
        rospy.loginfo("Waiting for mavros/global_position/global.")
        try:
            initial_global_pos = rospy.client.wait_for_message(
                "mavros/global_position/global", NavSatFix, timeout=3)
            self.map_param["latitude"] = initial_global_pos.latitude
            self.map_param["longitude"] = initial_global_pos.longitude
        except rospy.ROSException:
            pass

        self.draw_map()

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
            # set start point
            self.points["start"] = Point()
            self.points["start"].x = event.xdata
            self.points["start"].y = event.ydata

            # set and display start point and its label
            self.labels["start_point"] = self.axis.text(
                event.xdata, event.ydata, "Start", color="red", fontsize=16)
            self.lines["dot"].set_data(self.points["vertices_x"] + [event.xdata],
                                       self.points["vertices_y"] + [event.ydata])
            self.lines["dot"].figure.canvas.draw()
        else:
            rospy.logwarn("Unable to register points anymore.")

    def get_canvas_size(self):
        """!
        Return current canvas size
        """
        bbox = self.axis.get_window_extent().transformed(
            self.fig.dpi_scale_trans.inverted())
        width, height = bbox.width, bbox.height
        width *= self.fig.dpi
        height *= self.fig.dpi
        return int(width), int(height)

    def draw_map(self):
        """!
        Draw map using current parameters
        """
        try:
            # create url request with the shape like the following
            # https://maps.google.com/maps/api/staticmap?center=latitude,longitude&zoom=zoom&size=widthxheight&sensor=sensor&maptype=maptype
            image_responce = urlopen(self.map_param["url"]
                                     + "?center=" +
                                     str(self.map_param["latitude"]) +
                                     "," + str(self.map_param["longitude"])
                                     + "&zoom=" + str(self.map_param["zoom"])
                                     + "&size=" +
                                     str(self.map_param["size"][0]) +
                                     "x" + str(self.map_param["size"][1])
                                     + "&sensor=" + self.map_param["sensor"]
                                     + "&maptype=" + self.map_param["maptype"])
            self.map_param["background_image"] = Image.open(image_responce)
            self.axis.imshow(self.map_param["background_image"],
                             extent=[self.axis.get_xlim()[0],
                                     self.axis.get_xlim()[1],
                                     self.axis.get_ylim()[0],
                                     self.axis.get_ylim()[1]])
        except HTTPError:
            return

    def update_params(self):
        """!
        Update values of coverage parameters
        """
        self.coverage_params["aspect_ratio"] = float(
            self.shooting_cond["image_resolution_w"]) / self.shooting_cond["image_resolution_h"]
        self.coverage_params["footprint_width"] = Float64(
            2 * self.shooting_cond["height"] * math.tan(self.shooting_cond["angle_of_view"] / 2))
        self.coverage_params["footprint_length"] = Float64(
            self.coverage_params["footprint_width"].data / self.coverage_params["aspect_ratio"])

    def update_param_texts(self):
        """!
        Update texts of coverage parameters
        """
        self.labels["aspect_ratio_text"].set_text("Aspect ratio:\n    " +
                                                  str(self.coverage_params["aspect_ratio"]))
        self.labels["footprint_length_text"].set_text("Footprint Length [m]:\n    " +
                                                      str(round(self.coverage_params["footprint_length"].data, 2)))
        self.labels["footprint_width_text"].set_text("Footprint Width [m]:\n    " +
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
        self.lines["line"].set_data(self.points["vertices_x"] + self.points["vertices_x"][0:1],
                                    self.points["vertices_y"] + self.points["vertices_y"][0:1])
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

        if not self.points["start"]:
            rospy.logwarn("Choose start point.")
            return

        # assign server node if server node is None
        if not self.server_node:
            rospy.loginfo("Waiting for Server Node.")
            try:
                rospy.wait_for_service("cpp_torres16",
                                       timeout=5.0)
            except rospy.ROSException:
                rospy.logerr("Server not found.")
                return
            try:
                self.server_node = rospy.ServiceProxy(
                    "cpp_torres16",
                    Torres16)
            except rospy.ServiceException as ex:
                rospy.logerr(str(ex))
                return

        # Create a list of vertices
        vertices = []
        waypoint_xs = []
        waypoint_ys = []

        # fill the list of vertices that is passed to server node
        for x_coord, y_coord in zip(self.points["vertices_x"],
                                    self.points["vertices_y"]):
            point = Point()
            point.x = x_coord
            point.y = y_coord
            vertices.append(point)

        # Call service
        try:
            ret = self.server_node(vertices,
                                   self.points["start"],
                                   self.coverage_params["footprint_length"],
                                   self.coverage_params["footprint_width"],
                                   self.coverage_params["horizontal_overwrap"],
                                   self.coverage_params["vertical_overwrap"])
            self.points["waypoints"] = ret.path
            self.subpolygons = ret.subpolygons

            # fill the lists of waypoints' coordinate to draw path
            for num, point in enumerate(self.points["waypoints"]):
                if num == 0:
                    waypoint_xs.append(self.points["start"].x)
                    waypoint_ys.append(self.points["start"].y)
                    self.labels["SP"] = self.axis.text(
                        point.x, point.y, "SP", color="red", fontsize=16)
                waypoint_xs.append(point.x)
                waypoint_ys.append(point.y)
                if num == len(self.points["waypoints"]) - 1:
                    waypoint_xs.append(self.points["start"].x)
                    waypoint_ys.append(self.points["start"].y)
                    self.labels["EP"] = self.axis.text(
                        point.x, point.y, "EP", color="red", fontsize=16)
            for subpolygon in self.subpolygons:
                ar = np.ndarray([len(subpolygon.points), 2])
                for num, point in enumerate(subpolygon.points):
                    ar[num][0] = point.x
                    ar[num][1] = point.y
                patch = Polygon(xy=ar, alpha=0.5, edgecolor="navy")
                self.axis.add_patch(patch)
                self.patches.append(patch)
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

        # Clear lists of vertices' coordinates
        self.points["vertices_x"] = []
        self.points["vertices_y"] = []
        # Set flag as False
        self.is_polygon_drawn = False
        # Clear start point
        self.points["start"] = None

        # Clear waypoints
        self.points["waypoints"] = []
        self.points["subpolygons"] = []

        # Clear point data
        self.lines["dot"].set_data(
            self.points["vertices_x"], self.points["vertices_y"])
        self.lines["line"].set_data(
            self.points["vertices_x"], self.points["vertices_y"])
        self.lines["path"].set_data([], [])

        if self.labels["start_point"]:
            self.labels["start_point"].remove()
        if self.labels["SP"]:
            try:
                self.labels["SP"].remove()
                self.labels["EP"].remove()
            except ValueError as e:
                pass

        for patch in self.patches:
            patch.remove()

        self.patches = []

        # Refresh a canvas
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
        # Update parameter if input is digit and valid value
        if event.isdigit() and 0 < float(event) < 1.0:
            self.coverage_params["horizontal_overwrap"].data = float(event)
        else:
            self.text_boxes["horizontal_overwrap_box"].\
                set_val(str(self.coverage_params["horizontal_overwrap"]))

    def vertical_overwrap_update(self, event):
        """!
        Called when content of "Vertical overwrap" is submitted
        @param event Content of TextBox
        """
        # Update parameter if input is digit and valid value
        if event.isdigit() and 0 < float(event) < 1.0:
            self.coverage_params["vertical_overwrap"].data = float(event)
        else:
            self.text_boxes["vertical_overwrap_box"].\
                set_val(str(self.coverage_params["vertical_overwrap"]))

    def latitude_update(self, val):
        """!
        Called when content of "Latitude" is submitted
        @param val Content of TextBox
        """
        try:
            if -90 <= float(val) <= 90:
                self.map_param["latitude"] = float(val)
                text = plt.text(-5, 7.5, "Loading Map...",
                                fontsize=24, color="magenta")
                text.figure.canvas.draw()
                self.draw_map()
                text.remove()
            else:
                self.text_boxes["latitude_box"].\
                    set_val(str(self.map_param["latitude"]))
        except ValueError:
            self.text_boxes["latitude_box"].\
                set_val(str(self.map_param["latitude"]))

    def longitude_update(self, val):
        """!
        Called when content of "Longitude" is submitted
        @param val Content of TextBox
        """
        try:
            if -180 <= float(val) <= 180:
                self.map_param["longitude"] = float(val)
                text = plt.text(-5, 7.5, "Loading Map...",
                                fontsize=24, color="magenta")
                text.figure.canvas.draw()
                self.draw_map()
                text.remove()
            else:
                self.text_boxes["longitude_box"].\
                    set_val(str(self.map_param["longitude"]))
        except ValueError:
            self.text_boxes["longitude_box"].\
                set_val(str(self.map_param["longitude"]))

    def zoom_in(self, event):
        """!
        Callback function for "+" button
        @param event MouseEvent object
        """
        if self.map_param["zoom"] == 21:
            return
        text = plt.text(-5, 7.5, "Loading Map...",
                        fontsize=24, color="magenta")
        text.figure.canvas.draw()
        self.map_param["zoom"] += 1
        self.draw_map()
        text.remove()

    def zoom_out(self, event):
        """!
        Callback function for "-" button
        @param event MouseEvent object
        """
        if self.map_param["zoom"] == 2:
            return
        text = plt.text(-5, 7.5, "Loading Map...",
                        fontsize=24, color="magenta")
        text.figure.canvas.draw()
        self.map_param["zoom"] -= 1
        self.draw_map()
        text.remove()

    def maptype_update(self, event):
        """!
        Called when the selection of radiobutton is changed
        @param event Label of selected button
        """
        self.map_param["maptype"] = event
        text = plt.text(-5, 7.5, "Loading Map...",
                        fontsize=24, color="magenta")
        text.figure.canvas.draw()
        self.draw_map()
        text.remove()


def init_node():
    """!
    Initialize a node
    """
    rospy.init_node('specify_node', anonymous=True)

    # Call PolygonBuilder's constructor
    PolygonBuilder()


if __name__ == '__main__':
    init_node()
