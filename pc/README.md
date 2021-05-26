# PC Software - Python Script

Requirments `pfFINAL.py`
```
Modules:
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
```

Running `pfFINAL.py`
```
Developed with python 8.10
python pfFINAL.py

Inputs:
Number of Slaves (UWB Ranging Nodes)
For each slave: 
    - Slave ID
    - Slave Position (x,y) in grid
How many tags - current support is 2 with web dashboard (can be easily more)
Grid Size (x,y) maximum rectangular dimensions
Select Serial Port of Master Node

Outputs:
Positioning Calculations of RSSI/UWB
Data Pushed to Tago
```
