#!/usr/bin/env python
import rospy, math
from matplotlib import pylab
import numpy

#####################################
#                                   #
#   Defining the sigmoid Function   #
#                                   #
#####################################

def sigmoid(x):
    sigmoid_return = 1 / (1 + numpy.exp(-x))
    return sigmoid_return
#   Linspace is used as array with start and
#   stop value. In the line below -10 is the
#   start value and 10 is the stop value
#   and 10 is the value of total number of
#   steps with end point included true as
#   default
x = pylab.linspace(-10,10,10)
y = pylab.linspace(0,1,10)

#   the line below plots a graph between the
#   the value of x and sigmoid of x. 'r' is used
#   as red color to plot the graph and label is also
#   defined
pylab.plot(x, sigmoid(x), 'r', label = ' x linspace(-10,10,10)')

#   the graph is plotted in the form of grid
pylab.grid()

#   the title of the graph is Sigmoid Function
pylab.title('Sigmoid Function')

#   the line below writes the formula for the sigmoid
#   function where first two parameters defines the
#   location where to write the equation. Next is the
#   equation and last is the font size
pylab.text(4,0.8,r'$\sigma(x)=\frac{1}{1+e^{-x}}$', fontsize = 12)

#   the line below defines the labels of the graph
pylab.xlabel('X Axis')
pylab.ylabel('Y Axis')

#   the line below defines the location of legend
pylab.legend(loc = 'lower right')

#   finally the graph is shown
pylab.show()

