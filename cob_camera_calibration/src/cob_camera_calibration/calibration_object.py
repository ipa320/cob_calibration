#!/usr/bin/env python
import numpy as np

class CalibrationObject:
    '''
    Base Class for calibration objects
    
    A calibration object is defined by its pattern_size and square_size.
    '''
    
    def __init__(self, (rows, cols), square_size):
        self.pattern_size = (rows, cols)
        self.square_size = square_size

    def get_pattern_points(self):
        raise NotImplementedError()
    
class Checkerboard(CalibrationObject):
    '''
    Checkerboard calibration object
    
    Pattern size is number of inner checkerboard corners in horizontal and
    vertical direction. Square size is the size of a square (im m)
    '''
    def get_pattern_points(self):
        '''
        Returns the location of checkerboard corners in object centered
        coordinate system. z coordinate is always 0.
        '''
        # implementation adapted from opencv examples
        pattern_points = np.zeros((np.prod(self.pattern_size), 3), np.float32)
        pattern_points[:,:2] = np.indices(self.pattern_size).T.reshape(-1, 2)
        pattern_points *= self.square_size
        return pattern_points

class SymmerticCirclegrid(CalibrationObject):
    '''
    Symmetric Circlegrid calibration object (as defined by opencv)
    '''
    def get_pattern_points(self):
        '''
        Returns the location of circle centers in object centered
        coordinate system. z coordinate is always 0.
        '''
        # implementation adapted from opencv examples
        pattern_points = np.zeros((np.prod(self.pattern_size), 3), np.float32)
        pattern_points[:,:2] = np.indices(self.pattern_size).T.reshape(-1, 2)
        pattern_points *= self.square_size
        return pattern_points
