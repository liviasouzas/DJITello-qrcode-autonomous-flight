import numpy as np
import cv2

class PID_control:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
    def calculations(self, error, dt):
        self.integral += error*dt
        derivative = (error - self.prev_error)/dt
        PID = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        
        self.prev_error = error

        return PID