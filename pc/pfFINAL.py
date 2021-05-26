 #**
 #************************************************************************
 #* @file pfFINAL.py
 #* @author James Debeyser
 #* @date 25.05.2021 (Last Updated)
 #* @brief Python Script for receiving and processing UWB/RSSI data
 #*        over serial and interfacing with web dashboard Tago.
 #**********************************************************************
 #**

import tkinter as tk
import random
import time
import numpy as np
import serial
import serial.tools.list_ports
import math
from PyQt5 import QtCore, QtGui, QtWidgets
import sys
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox
)
import threading
from threading import Thread
from statistics import mean
from math import atan2,degrees,cos,sin,radians
import tago

# Globals to monitor Serial Communications
global connected    #Connection State
global ser          #Serial Object
global SerialQ      #Serial Queue

#Tago Globals
global uwb_tag1     #Global to monitor Tag1 object
global uwb_x1       #Global to monitor Tag1 x
global uwb_y1       #Global to monitor Tag1 y
global uwb_gridx1   #Global to write grid dimensions x t1
global uwb_gridy1   #Global to write grid dimensions y t1

global uwb_tag2     #Global to monitor Tag2 object
global uwb_x2       #Global to monitor Tag2 x
global uwb_y2       #Global to monitor Tag2 y
global uwb_gridx2   #Global to monitor grid dimensions x t2
global uwb_gridy2   #Global to monitor grid dimensions y t2

#RSSI Globals
global position_x
global position_y

#Kalman Globals
global x_true       #contains raster scan of data points for kalman
global x_true_ready #100 data points for kalman
global start_time   #monitor uptime

global grid_x       #grid dimension x
global grid_y       #grid dimension y
global list_tags    #list of tag objects

#not used - vars for local gui
slaves = {}
bubbles = []
window = tk.Tk()
tags = []

# Class to initalise a kalman filter
class Kalman:
    def __init__(self, x_init, cov_init, meas_err, proc_err):
        self.ndim = len(x_init)
        self.A = np.array([(1, 0, dt, 0), (0, 1, 0, dt), (0, 0, 1, 0), (0, 0, 0, 1)]);
        self.H = np.array([(1, 0, 0, 0), (0, 1, 0, 0)])
        self.x_hat =  x_init
        self.cov = cov_init
        self.Q_k = np.eye(ndim)*proc_err
        self.R = np.eye(len(self.H))*meas_err

    #update kalman estimate
    def update(self, obs):

        # Make prediction
        self.x_hat_est = np.dot(self.A,self.x_hat)
        self.cov_est = np.dot(self.A,np.dot(self.cov,np.transpose(self.A))) + self.Q_k

        # Update estimate
        self.error_x = obs - np.dot(self.H,self.x_hat_est)
        self.error_cov = np.dot(self.H,np.dot(self.cov_est,np.transpose(self.H))) + self.R
        self.K = np.dot(np.dot(self.cov_est,np.transpose(self.H)),np.linalg.inv(self.error_cov))
        self.x_hat = self.x_hat_est + np.dot(self.K,self.error_x)
        if ndim>1:
            self.cov = np.dot((np.eye(self.ndim) - np.dot(self.K,self.H)),self.cov_est)
        else:
            self.cov = (1-self.K)*self.cov_est


# Class to maintain tag information
class tag:
    def __init__(self, id, num_slaves, slaves, rssis, uwbs):
        self.id = id
        self.num_slaves = num_slaves
        self.slaves = slaves #dictionary of slaves
        self.rssis = rssis #dictionary or list? {"node-id": rssi}
        self.uwbs = uwbs #dictionary or list? {"node-id": uwb dist}

        #monitors tag position
        self.rssi_position = (200,200)
        self.uwb_position = (200,200)

    #update rssi position from slave node
    def update_rssi(self, slave, rssi):
        self.rssis[slave] = rssi

    #update uwb position from slave node
    def update_uwbs(self, slave, uwb):
        self.uwbs[slave] = uwb

    #update rssi x,y position
    def update_rssi_position(self, rssi_position):
        self.rssi_position = rssi_position

    #uppdate uwb x,y position
    def update_uwb_position(self, uwb_position):
        self.uwb_position = uwb_position

    #update rssi position based off of least squares estimate
    def rssi_least_squares(self):
        last_slave_pos = self.slaves.get(list(self.slaves)[-1])
        #list of slave positions removing last element which is las_slave_pos
        positions = np.array([self.slaves.get(x) for x in slaves])
        positions = positions[:-1]

        a = []
        for i in range(len(positions)):
            a.append([2 * (last_slave_pos[0] - positions[i][0]), 2 * (last_slave_pos[1] - positions[i][1])])
        a = np.array(a)

        b = []
        for i in range(len(positions)):
            b.append([rssi_dist_est(self.rssis[list(self.slaves)[i]]) ** 2 - rssi_dist_est(self.rssis[list(self.slaves)[-1]]) ** 2 - positions[i][0] ** 2 + last_slave_pos[0] ** 2 - positions[i][1] ** 2 + last_slave_pos[1] ** 2])
        b = np.array(b)

        m, c = np.linalg.lstsq(a, b)[0]
        self.rssi_position = (m,c)

    # --- Here down is UWB positioning implementation --- #

    # update uwb x,y based off multilat
    # calculate perpendicular lines (representing constant magnitude)
    # find all intersections
    # average intersections to give final x,y
    def uwb_xy_calculate(self):
        #list of intersection points (x,y)
        intersection_points = []
        perp_lines = []

        #calculate all perpendicular lines
        count = 0
        for i in range(len(list(slaves))):
            perp_lines.append(
                self.get_perp_line(self.line_gradient((0, 0), self.slaves.get(list(slaves)[count])), self.slaves.get(list(self.slaves)[count]),
                              self.uwbs.get(list(self.slaves)[count])))
            count = count + 1

        #calculate all intersection points
        points = []
        count = 1
        list_index = 1
        for i in range(len(perp_lines) - 1):
            if len(slaves) - count == 0:
                break
            else:
                for j in range(len(perp_lines) - count):
                    if perp_lines[i][2] == "y" and perp_lines[list_index][2] == "m":
                        y = perp_lines[i][1]
                        x = (y - perp_lines[list_index][1]) / perp_lines[list_index][0]
                    elif perp_lines[i][2] == "y" and perp_lines[list_index][2] == "x":
                        y = perp_lines[i][1]
                        x = perp_lines[list_index][0]
                    elif perp_lines[i][2] == "x" and perp_lines[list_index][2] == "m":
                        x = perp_lines[i][0]
                        y = perp_lines[list_index][0] * x + perp_lines[list_index][1]
                    elif perp_lines[i][2] == "x" and perp_lines[list_index][2] == "y":
                        y = perp_lines[list_index][1]
                        x = perp_lines[i][0]
                    elif perp_lines[i][2] == "m" and perp_lines[list_index][2] == "x":
                        x = perp_lines[list_index][0]
                        y = perp_lines[i][0] * x + perp_lines[i][1]
                    elif perp_lines[i][2] == "m" and perp_lines[list_index][2] == "y":
                        y = perp_lines[list_index][1]
                        x = (y - perp_lines[i][1]) / perp_lines[i][0]

                    points.append((x, y))
                    list_index = list_index + 1
                count = count + 1
                list_index = count

        # calculate averages of points
        average = [0, 0]
        for point in points:
            average[0] = average[0] + point[0]
            average[1] = average[1] + point[1]

        average[0] = average[0] / len(points)
        average[1] = average[1] / len(points)

        self.uwb_position = average

    # calculate gradient of line based off two points
    def line_gradient(self, p1, p2):
        #P1/2 (x,y)
        if (p2[0] == 0 and p1[0] == 0):
            return "y"

        if (p2[1] == 0 and p1[1] == 0):
            return "x"

        m = ((p2[1] - p1[1])) / (p2[0] - p1[0])
        c = (p2[1] - (m * p2[0]))
        #print("M and C", m, " ", c)
        return m

    #get perpendicular line of a line about a point of distance from center
    def get_perp_line(self, line_gradient, node, distance):
        #need the y point to get tangent line
        #node (x,y)
        #distance (k)
        #line_gradient m or null
        #line typue x,y or m

        #determine line type
        if node[0] == 0:
            line_type = "y"
        elif node[1] == 0:
            line_type = "x"
        else:
            line_type = "m"

        xDiff = node[0]
        yDiff = node[1]
        angle = degrees(atan2(yDiff, xDiff))


        #need to switch negatvies for correct grid placement
        distance = (distance * -1)
        #angle and magnitude to point.
        (xpoint, ypoint) = (distance*cos(radians(angle))),distance*(sin(radians(angle)))

        (xmid_point, ymidpoint) = ((0+node[0])/2,(0+node[1]/2))

        xfinal, yfinal = (xpoint + xmid_point, ypoint+ymidpoint)


        if line_type == "x":
            #vertical line at x = ?
            return(xfinal,0,"x")
        elif line_type == "y":
            #horizontal line at y = ?
            return(0,yfinal,"y")
        else:
            #Gradient line
            mfinal = 1/(line_gradient*-1)
            cfinal = (mfinal*(-1*xfinal))+yfinal
            return (mfinal,cfinal,"m")

    # find point of intersection of two lines
    # line1/2 = (m, c, type)
    # type = "x","y","m"
    # need to itterate through lines 5!
    def find_line_intersection(self, line1, line2):
        xi, yi = 0, 0
        if line1 == line2:
            return -1
        else:
            if line1[2] == "m" and line2[2] == "m":
                xi = (line1[1] - line2[1]) / (line2[0] - line1[0])
                yi = line1[0] * xi + line1[1]
                #print(xi, yi)
            elif line1[2] == "x" and line2[2] == "m":
                # Vertical line in x
                yi = line2[0] * line1[0] + line2[1]
                xi = line1[0]
            elif line1[2] == "m" and line2[2] == "x":
                yi = line1[0] * line2[0] + line1[1]
                xi = line2[0]
            elif line1[2] == "y" and line2[2] == "m":
                xi = (line1[0] - line2[1]) / line2[0]
                yi = line1[1]
            elif line1[2] == "m" and line2[2] == "y":
                #print("Detials: ",line1[0], line1[1], line2[0])
                xi = (line2[0] - line1[1]) / line1[0]
                yi = line2[1]
            elif line1[2] == "x" and line2[2] == "y" or line1[2] == "y" and line2[2] == "x":
                if line1[0] == 0:
                    # must be y
                    yi = line1[1]
                else:
                    xi = line1[0]
                if line2[0] == 0:
                    # must be y
                    yi = line2[1]
                else:
                    xi = line2[0]
        return (xi, yi)

# calculate rssi distance estimate based off equation
def rssi_dist_est(rssiValue):
    dist = 10**((10-rssiValue)/(10*2))
    return dist

# Queue class to monitor throughput of serial messages
class Queue:
    # Constructor creates a list
    def __init__(self) -> object:
        self.queue = list()

    # Adding elements to queue
    def enqueue(self, data):
        self.queue.insert(0, data)

    # Removing the last element from the queue
    def dequeue(self):
        if len(self.queue) > 0:
            return self.queue.pop()
        return "Queue Empty!"

    # Getting the size of the queue
    def is_empty(self):
        if len(self.queue) == 0:
            return True
        else:
            return False

# class for the serial port
class SerialPort:
    # initialise the required information for the serial connection
    def __init__(self, serial, portnum, baudrate, parity, timeout, stopbit):
        self.serial = serial
        self.portnum = portnum
        self.baudrate = baudrate
        self.parity = parity
        self.timeout = timeout
        self.stopbit = stopbit
        self.connected = False

    # Set up the serial port
    def serial_setup(self):
        self.serial = serial.Serial()
        self.serial.baudrate = self.baudrate
        self.serial.port = self.portnum
        self.serial.parity = self.parity
        self.serial.stopbits = self.stopbit
        self.serial.timeout = self.timeout

        # Attempt to open port throw error if no device connected
        try:
            self.serial.open()
            print("Device connected")
            self.connected = True
            return 1
        except BaseException:
            print("No device connected")
            self.connected = False


# Class to manage GUI objects - not implemented
class Bubble():
    def __init__(self, canvas, x, y, size, color='red', type='person'):
        self.canvas = canvas

        self.x = x
        self.y = y

        self.start_x = x
        self.start_y = y

        self.size = size
        self.color = color

        self.type = type

        self.circle = canvas.create_oval([x, y, x+size, y+size], outline=color, fill=color)

    def move(self):
        #USED FOR UWB
        if self.type == 'person':
            global position_x
            global position_y

            x_vel = (position_x*2) - self.x
            y_vel = (position_y*2) - self.y
            self.canvas.move(self.circle, x_vel, y_vel)
            coordinates = self.canvas.coords(self.circle)

            self.x = coordinates[0]
            self.y = coordinates[1]

            # if outside screen move to start position
            if self.y < -self.size:
                self.x = self.start_x
                self.y = self.start_y
                self.canvas.coords(self.circle, self.x, self.y, self.x + self.size, self.y + self.size)
        else:
            self.khalman_move()

    #USED FOR RSSI
    def khalman_move(self):
        global x_true_ready
        global x_true

        if x_true_ready == 100:

            obs_err = np.random.normal(0, np.ones(ndim_obs) * meas_error, (nsteps, ndim_obs))  # observations
            obs = x_true[:, 0:2] + obs_err

            # observations are going to come from reading!

            # init filter
            proc_error = 0.1;
            init_error = 100.0;
            x_init = np.array(
                [xcoord + init_error, ycoord + init_error, vx, vy])  # introduced initial xcoord error of 50 cm
            cov_init = init_error * np.eye(ndim)

            # filtering
            x_hat = np.zeros((ndim, nsteps))
            k = Kalman(x_init, cov_init, meas_error / 10, proc_error)
            for t in range(nsteps):
                k.update(obs[t])
                x_hat[:, t] = k.x_hat

            k_pos_x = mean(x_hat[0])
            k_pos_y = mean(x_hat[1])


            #MOVE khaman_person
            x_vel = (k_pos_x * 2) - self.x
            y_vel = (k_pos_y * 2) - self.y
            self.canvas.move(self.circle, x_vel, y_vel)
            coordinates = self.canvas.coords(self.circle)

            self.x = coordinates[0]
            self.y = coordinates[1]

            # if outside screen move to start position
            if self.y < -self.size:
                self.x = self.start_x
                self.y = self.start_y
                self.canvas.coords(self.circle, self.x, self.y, self.x + self.size, self.y + self.size)

        #otherwise do nothing

# send uwb location of tag1 to web dashboard
def send_uwb_location1():
    global uwb_tag1
    global uwb_x1
    global uwb_y1
    global uwb_gridx1
    global uwb_gridy1
    time.sleep(5)
    while(1):
        if uwb_x1 < 0:
            uwb_x1 = 0
        if uwb_y1 < 0:
            uwb_y1 = 0

        my_device = tago.Device('14740807-1986-4247-aae7-02fd3fcf03e2')
        if uwb_tag1 == 1:
            data = [{
                "variable": "tag1",
                "metadata": {
                    "x": uwb_x1 / uwb_gridx1,
                    "y": uwb_y1 / uwb_gridy1
                }
            }]
        result = my_device.insert(data)
        print(result)
        time.sleep(0.6)

# send uwb location of tag2 to web dashboard
def send_uwb_location2():
    global uwb_tag2
    global uwb_x2
    global uwb_y2
    global uwb_gridx2
    global uwb_gridy2
    uwb_tag2 = 1
    uwb_x2 = 1
    uwb_y2 = 1
    uwb_gridx2 = 1
    uwb_gridy2 = 1

    time.sleep(5)
    while(1):
        if uwb_x2 < 0:
            uwb_x2 = 0
        if uwb_y2 < 0:
            uwb_y2 = 0
        my_device = tago.Device('14740807-1986-4247-aae7-02fd3fcf03e2')
        if uwb_tag2 == 2:
            data = [{
                "variable": "tag2",
                "metadata": {
                    "x": uwb_x2 / uwb_gridx2,
                    "y": uwb_y2 / uwb_gridy2
                }
            }]
        result = my_device.insert(data)
        print(result)
        time.sleep(0.6)


# send rssi location to web dashboard - not using
def send_rssi_location(tag, x, y, grid_x, grid_y):
    my_device = tago.Device('14740807-1986-4247-aae7-02fd3fcf03e2')
    if tag == 1:
        data = [{
            "variable": "tag1rssi",
            "metadata": {
                "x": x / grid_x,
                "y": y / grid_y
            }
        }]
    elif tag == 2:
        data = [{
            "variable": "tag2rssi",
            "metadata": {
                "x": x / grid_x,
                "y": y / grid_y
            }
        }]
    elif tag == 3:
        data = [{
            "variable": "tag3rssi",
            "metadata": {
                "x": x / grid_x,
                "y": y / grid_y
            }
        }]
    result = my_device.insert(data)
    print(result)

# function thread to move gui objects - not used
def move():
    for item in bubbles:
        item.move()

    window.after(33, move)

# thread function to read incoming serial messages
def serial_read():
    global list_tags
    global grid_x
    global grid_y

    global uwb_tag1
    global uwb_x1
    global uwb_y1
    global uwb_gridx1
    global uwb_gridy1

    global uwb_tag2
    global uwb_x2
    global uwb_y2
    global uwb_gridx2
    global uwb_gridy2

    count = 0
    while connected == True:
        time.sleep(0.0001)
        try:
            line = ser.serial.readline()
            line = line.decode("utf-8")
            packet_array = line.split(",")
            print(packet_array, len(packet_array))

            #size is the number of nodes
            size = (len(packet_array) - 1) / 3
            print(size)
            #change size to the number of nodes using...
            if size % 5 == 0 and (packet_array[0] == '1' or packet_array[0] == '2' or packet_array[0] == '3'):
                #itterate over the number of nodes
                for i in range(int(size)):
                    for tag in list_tags:
                        #edit tag data
                        if tag.id == int(packet_array[0]):

                            for slave in list(tag.slaves.keys()):
                                if packet_array[1+ ((i)*3)] == slave:
                                    #Got correct Slave
                                    tag.uwbs[slave] = int(float(packet_array[2+ ((i)*3)]) / 10)
                                    tag.rssis[slave] = int(packet_array[3+ ((i)*3)])
                            tag.rssi_least_squares()
                            tag.uwb_xy_calculate()
                            print("UWB: ", tag.uwb_position, "RSSI: ", tag.rssi_position)
                            #send_uwb_location(int(tag.id), int(tag.uwb_position[0]), int(tag.uwb_position[1]), int(grid_x), int(grid_y))
                            if tag.id == 1:
                                uwb_tag1 = int(tag.id)
                                uwb_x1 = int(tag.uwb_position[0])
                                uwb_y1 = int(tag.uwb_position[1])
                                uwb_gridx1 = int(grid_x)
                                uwb_gridy1 = int(grid_y)
                            if tag.id == 2:
                                uwb_tag2 = int(tag.id)
                                uwb_x2 = int(tag.uwb_position[0])
                                uwb_y2 = int(tag.uwb_position[1])
                                uwb_gridx2 = int(grid_x)
                                uwb_gridy2 = int(grid_y)

        except Exception as e:
            count = count + 1
            print("Exception in serial_read()! ", count)
    print("Exiting serial_read()")

# function to initalise gui objects - not used
def grid_gui():
    time.sleep(.0001)
    start_x = 400
    start_y = 400

    print("in grid_gui")
    window.geometry("800x800")

    canvas = tk.Canvas(window, height=800, width=800)
    canvas.grid(row=0, column=0, sticky='w')

    person = Bubble(canvas, start_x, start_y, 40, 'red', 'person')
    bubbles.append(person)
    khaman_person = Bubble(canvas, start_x, start_y, 40, 'yellow', 'khaman_person')
    bubbles.append(khaman_person)

    #Top Left Node
    coord = [0, 0, 40, 40]
    rect_tl = canvas.create_rectangle(coord, outline="Blue", fill="Blue")
    T = tk.Text(window, height=2, width=30)

    #Bot Left Node
    coord = [0, 800, 40, 800-40]
    rect_bl = canvas.create_rectangle(coord, outline="Yellow", fill="Yellow")

    #Bot Right Node
    coord = [800, 800, 800-40, 800-40]
    rect_br = canvas.create_rectangle(coord, outline="Blue", fill="Blue")

    #Top Right Node
    coord = [800, 0, 800-40, 40]
    rect_tr = canvas.create_rectangle(coord, outline="Green", fill="Green")
    move()
    window.mainloop()
    print("HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n")
    while (1):
        continue

# setup serial connection
def setup_connection(port_name):
    global connected
    global ser
    global SerialQ
    ser = SerialPort(serial, port_name, 115200, 'N', .5, 1)
    ret = ser.serial_setup()
    SerialQ = Queue()
    if ret == 1:
        print("Device Connected\n")
        connected = True
        read = Thread(target=serial_read())
        read.setDaemon(True)
        read.start()
        print("Read Started")
    else:
        print("Device Not Connected\n")

# Main
if __name__ == "__main__":
    global connected
    global x_true_ready
    global x_true
    global position_x
    global position_y
    global start_time
    x_true_ready = 0
    connected = False
    position_x = 200
    position_y = 200
    start_time = time.time()
    global grid_x
    global grid_y

    # Slave Setup
    num_slaves = input("How many slaves: ")
    global list_tags
    list_tags = []
    uwbs = {}
    rssis = {}
    slaves = {}
    for i in range(int(num_slaves)):
        slave_id = input("Slave ID: ")
        slave_position = input("Slave Position x,y: ")
        x,y = slave_position.split(',')
        slaves[str(slave_id)] = (int(x),int(y))
        rssis[str(slave_id)] = -73
        uwbs[str(slave_id)] = 0

    num_tags = input("How many tags: ")

    # Tag Objects Setup
    count = 1
    for j in range(int(num_tags)):
        atag = tag(count,int(num_slaves), slaves, rssis, uwbs)
        list_tags.append(atag)
        count = count + 1

    for atag in list_tags:
        atag.uwb_xy_calculate()
        atag.rssi_least_squares()

    grid_size = input("Grid size x,y: ")
    wow = grid_size.split(',')
    x = wow[0]
    y = wow[1]
    print(x,y)
    grid_x = int(x)
    grid_y = int(y)

    #Setup Serial
    port_list = [comport.device for comport in serial.tools.list_ports.comports()]
    count = 0
    for port in port_list:
        print(count, " ", port)
        count = count + 1

    port_input = input("Select Port: ")
    selected_port = port_list[int(port_input)]

    #Thread for tago
    tago_thread_t1 = Thread(target=send_uwb_location1)
    tago_thread_t1.setDaemon(True)
    tago_thread_t1.start()

    if int(num_tags) == 2:
        tago_thread_t2 = Thread(target=send_uwb_location2)
        tago_thread_t2.setDaemon(True)
        tago_thread_t2.start()

    # Thread for serial messages
    setup_connection(selected_port)


    #KALMAN
    ndim = 4
    ndim_obs = 2
    nsteps = 100
    xcoord = 200.0
    ycoord = 200.0
    vx = 0#20.0  # cm.s
    vy = 0#20.0  # cm/s
    dt = 20.0  # sec
    meas_error = 50.0  # cm
    x_true = np.array([(xcoord + i * vx, ycoord + i * vy, vx, vy) for i in range(nsteps)])


    #Thread for position
    #position_thread = Thread(target=least_squares)
    #position_thread.setDaemon(True)
    #position_thread.start()

    #Thread For GUI
    #grid_thread = Thread(target=grid_gui())
    #grid_thread.setDaemon(True)
    #grid_thread.start()

