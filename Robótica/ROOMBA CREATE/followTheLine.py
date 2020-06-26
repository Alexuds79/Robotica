'''
Created on 21 may. 2020

@author: Alejandro Mateos Pedraza - 70969732N
         Javier López Sánchez - 70921610Y
'''

import create
import time
import matplotlib.pyplot as plt
import numpy as np
from cmath import pi
plt.ion()
robot = create.Create('sim')


# 0 -> Black surface
#4094 -> White surface
#1000 -> Gray surface

x = 0
y = 0
xdata = [x]
ydata = [y]
last_angle = 0
xIncrement = 0
yIncrement = 0
counter = 0
xfinal = 0
yfinal = 0
aux = 0

#Rotate until you detect the black line
def findLine(): 
        global aux
        aux += 1
            
        cliff_front_right_signal = robot.getSensor("CLIFF_FRONT_RIGHT_SIGNAL")
        cliff_front_left_signal = robot.getSensor("CLIFF_FRONT_LEFT_SIGNAL")
        cliff_right_signal = robot.getSensor("CLIFF_RIGHT_SIGNAL")
        cliff_left_signal = robot.getSensor("CLIFF_LEFT_SIGNAL")
                
        if aux < 36 and cliff_front_right_signal != 0 and cliff_front_left_signal != 0 and cliff_right_signal != 0 and cliff_left_signal != 0:   
            robot.go(0,10)
            time.sleep(1)
            findLine()

        elif aux >= 36:
            if cliff_front_right_signal != 0 and cliff_front_left_signal != 0 and cliff_right_signal != 0 and cliff_left_signal != 0:
                robot.go(10,0)
                time.sleep(1)
                findLine()
            else:
                aux = 0
            
                    
#Follow the way
def followTheWay():
        cliff_front_right_signal = robot.getSensor("CLIFF_FRONT_RIGHT_SIGNAL")
        cliff_front_left_signal = robot.getSensor("CLIFF_FRONT_LEFT_SIGNAL")
        cliff_right_signal = robot.getSensor("CLIFF_RIGHT_SIGNAL")
        cliff_left_signal = robot.getSensor("CLIFF_LEFT_SIGNAL")
                
        if (cliff_front_right_signal == 0):
            robot.go(10,10)
        elif (cliff_front_left_signal == 0):
            robot.go(10,10)
        elif (cliff_right_signal == 0):
            robot.go(0,10)
        elif (cliff_left_signal == 0):
            robot.go(0,10)
        else:
            findLine()

class DynamicUpdate():
    #Suppose we know the x range
    min_x = -3
    max_x = 3
    
    def on_launch(self):
        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([], [], 'o--')
        
        #Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        self.ax.set_xlim(self.min_x, self.max_x)
        
        #Other stuff
        self.ax.grid()
        ...
        
    def on_running(self, xdata, ydata):
        #Update data (with the new and the old points)
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
    
    def __call__(self):       
        global xdata
        global ydata
        global x
        global y
        global xIncrement
        global yIncrement
        global last_angle
        global xfinal
        global yfinal
        global aux
        
        self.on_launch()
            
        while 1:
            angle = robot.getSensor("ANGLE") * pi/180
            distance = robot.getSensor("DISTANCE") /1000
        
            xIncrement = distance * np.cos(last_angle + angle)
            yIncrement = distance * np.sin(last_angle + angle)
            
            last_angle += angle
            last_angle_degrees = last_angle*180/pi
            
            if last_angle_degrees >= 360:
                last_angle_degrees = last_angle_degrees - 360
                last_angle = last_angle - (360*pi/180)
            
            xfinal = xdata[-1] + xIncrement
            yfinal = ydata[-1] + yIncrement
            
            xdata.append(xfinal)
            ydata.append(yfinal)
                    
            self.on_running(xdata, ydata) 
            
            aux = 0
            findLine()
            followTheWay()
            
            time.sleep(1)
        return xdata, ydata

'''
MAIN PROGRAM
'''
d = DynamicUpdate()
d()

robot.stop()

