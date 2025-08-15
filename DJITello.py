# Based on TELLO SDK 2.0 User Guide pdf - Author: Livia Souza
# livia.souza.s@outlook.com
# The main objective of this lib is to implement tools for use in tello computational vision

import cv2
import socket
import threading
import time
from pyzbar.pyzbar import decode
import numpy as np

class tello_command:

    def __init__(self, ip, tello_port, video_port):
        
        self.IP = ip 
        self.TELLO_PORT = tello_port
        self.TELLO_VIDEO  = video_port

        self.last_command_time = time.time()

        self.stream_thread = None
        self.video_capture = None
        self.current_frame = None  
        self.stream_window_open = True  

        self.keepalive_active = False
        self.keepalive_thread = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', tello_port))

        self.running = True
        self.response_thread = threading.Thread(target = self.receive_response, daemon =True)
        self.response_thread.start()

        self._init_tello()
        self.init_keepalive()


    def init_keepalive(self):
        if not self.keepalive_active:
            self.keepalive_active = True
            self.keepalive_thread = threading.Thread(target=self.keepalive_loop, daemon= True)
            self.keepalive_thread.start()

    def keepalive_loop(self):
        while self.keepalive_active and self.running:
            if time.time() - self.last_command_time > 5:
                self.sock.sendto(b'command', (self.IP, self.TELLO_PORT))
                time.sleep(1)
    

    """Enter SDK mode (Software Development Kit)"""
    def _init_tello(self):
        self.send_command('command')
        time.sleep(2)

    """Send and receive UDP messages (commands) - using this you can create your own commands"""
    def send_command(self, command):
        self.last_command_time = time.time()
        print(f"sending command: {command}")
        self.sock.sendto(command.encode(), (self.IP, self.TELLO_PORT))
        time.sleep(0.1)

    def receive_response(self):
        while self.running:
            try: 
                data, _ = self.sock.recvfrom(1024)
                print(f'Tello Answer:{data.decode()}')
            except Exception as error:
                print(f"Receiving message error: {error}")



    """Camera commands"""
    def streamon(self):
        self.send_command('streamon')
        time.sleep(5)

        video_url = f'udp://@0.0.0.0:{self.TELLO_VIDEO}?fifo_size=5000&overrun_nonfatal=1&timeout=1000000&flush_packets=1' #&buffer_size=65536'
        self.video_capture = cv2.VideoCapture(video_url)
        if not self.video_capture.isOpened():
            print('streamon Failed')
            exit()
        

        self.frame_width = self.video_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.frame_height = self.video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.stream_thread = threading.Thread(target = self.video_streaming, daemon = True)
        self.stream_thread.start()        

    def video_streaming(self):
            while self.running and self.video_capture and self.stream_window_open:
                status, frame = self.video_capture.read()
                if not status:
                    print('Frame error')
                    break

                self.current_frame = frame
                                
                center_x, center_y, polygon, value = self.qr_decode()

                if polygon is not None and len(polygon) >=4:
                    points = [(point.x, point.y) for point in polygon]
                    points = np.array(points, dtype=np.int32)
                    cv2.polylines(frame, [points], isClosed=True, color=(0, 255, 0), thickness=2)  
                    if center_x is not None and center_y is not None:
                        cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)                        
                    if value is not None:
                        cv2.putText(frame, value, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, (0, 255, 0), 2, cv2.LINE_AA)

                cv2.imshow('Tello Video Stream', frame)

                if cv2.waitKey(1) == 13:
                    self.stream_window_open = False
                    break
                
            cv2.destroyWindow('Tello Video Stream')
        

    def qr_decode(self):

        if self.current_frame is None:
            return None, None, None, None
        else:
            qrcodes = decode(self.current_frame)
            if qrcodes:
                qr = qrcodes[0]
                value = qrcodes[0].data.decode('utf-8')
                polygon = qrcodes[0].polygon

                if len(polygon) >= 4:
                    left_top = polygon[0]
                    right_bottom = polygon[2]
                    center_x = (left_top.x + right_bottom.x) / 2
                    center_y = (left_top.y + right_bottom.y) / 2
                    return center_x, center_y, polygon, value        
            return None, None, None, None
        
    def streamoff(self):   
        """end stream and release resources"""           
        self.send_command('streamoff')
        if self.video_capture:
            self.video_capture.release()
            self.video_capture = None  
       
    """flight move commands""" 
    
    def takeoff(self):
        self.send_command('takeoff')
    
    def land(self):
        self.send_command('land')
    
    def emergency(self):
        self.send_command('emergency')
    
    def move(self, direction, distance):
        """Direction: forward, back, left, right, up, down"""
        self.send_command(f'{direction} {distance}')
    
    def hover(self):
        """stops and hover"""
        self.send_command('stop')

    def rc_control(self, left_right, forward_back, up_down, yaw):
        "speed"
        self.send_command(f'rc {left_right} {forward_back} {up_down} {yaw}')
        
    def set_speed(self, speed):
        if 10<= speed <= 100:
            self.send_command(f'speed {speed}')
        else:
            print('Invalid speed! The value must be between 10 and 100 cm/s')


    """status commands"""
    def current_height(self):
        self.send_command('height?')
        time.sleep(0.5)

    
    """release all process"""          
    def stop_keepalive(self):
        self.keepalive_active = False
        if self.keepalive_thread:
            self.keepalive_thread.join(timeout=1)
        
    def close_all(self):
        self.running = False
        self.stop_keepalive()
        self.stream_window_open = False
        if self.stream_thread and self.stream_thread.is_alive():
            self.stream_thread.join(timeout=1)

        self.streamoff()
        self.sock.close()




    


    

    
