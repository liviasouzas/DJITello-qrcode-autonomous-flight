from DJITello import tello_command
import time
import numpy as np
from PID import PID_control 
import csv
import os
from scipy.interpolate import interp1d



def save_mission_data(data, filename = 'mission_data'):
    keys = data[0].keys() if data else []
    with open(filename, 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(data)

def calibration(height_cm):
    """interval for tello of 80cm to 120cm of mission"""
    heights = [80, 90, 100,110,120]
    pixels_per_cm = [14.08, 11.91, 10.02, 9.31, 7.12]
    func_pixels_per_height = interp1d(heights, pixels_per_cm, kind='linear', fill_value='extrapolate')

    return float(func_pixels_per_height(height_cm))

def qrcodes_navigation(drone):

    step = 200
    drone.set_speed(20)

    qr_1_check = False
    qr_2_check = False
    qr_3_check = False
    qr_4_check = False

    while True:
        
        qr_center_x, qr_center_y, _, qr_value = drone.qr_decode()
        
        if qr_value == '1' and qr_1_check is False:
            drone.move('forward',step)
            time.sleep(15)
            qr_1_check = True

        if qr_value == '2' and qr_2_check is False:
            drone.send_command('ccw 90')
            time.sleep(6)
            drone.move('forward',step)
            time.sleep(15)
            qr_2_check = True

        if qr_value == '3' and qr_3_check is False:
            drone.send_command('ccw 90')
            time.sleep(6)
            drone.move('forward',step)
            time.sleep(15)
            qr_3_check = True
        
        if qr_value == '4' and qr_4_check is False:
            drone.send_command('ccw 90')
            time.sleep(6)
            drone.move('forward',100)
            time.sleep(10)
            drone.send_command('ccw 90')
            time.sleep(5)
            drone.move('forward',100)
            time.sleep(10)
            qr_4_check = True

        if qr_value == '5':
            qr_landing_found = True
            return qr_landing_found

def control(drone, qr_found, pixels_to_cm, max_speed, tolerance, mission_time):

    
    if qr_found is True:          
      

        mission_log = []

        centralized_x = False
        centralized_y = False
        frame_center_y = drone.frame_height/2 
        frame_center_x = drone.frame_width/2 
        pid_x = PID_control(Kp=0.2, Ki =0.002, Kd=0)
        pid_y = PID_control(Kp =0.2, Ki=0.003, Kd=0)
        
        last_time = time.time()
        start_adjustment_time = time.time()
        stabilization_start_time = None
        stabilization_duration = 5
     
        count_qr_lost = 0


        try:
            while True:

                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                if current_time - start_adjustment_time > 300:
                    print("Timeout!!! Landing with partial precision")
                    mission_log.append({
                        "searchtime": mission_time,
                        "timestamp": round(current_time - start_adjustment_time, 3),
                        "error_x": None,
                        "error_y": None,
                        "pid_error_x": None,
                        "pid_error_y": None,
                        "qr_center_x": None,
                        "qr_center_y": None,
                        "frame_center_x": frame_center_x,
                        "frame_center_y": frame_center_y})
                    drone.land()
                    break

                qr_center_x, qr_center_y, _, qr_value = drone.qr_decode()
                if qr_center_x is None or qr_center_y is None:
                    count_qr_lost += 1
                    print(f"QR lost. Trying to find...attempt {count_qr_lost}")
                    continue
                else:
                    count_qr_lost = 0
                

                error_x = (frame_center_x - qr_center_x)/pixels_to_cm
                error_y = (frame_center_y - qr_center_y)/pixels_to_cm
                centralized_x = abs(error_x) <= tolerance
                centralized_y = abs(error_y) <= tolerance
                if centralized_x and centralized_y:
                    if stabilization_start_time is None:
                        stabilization_start_time = time.time()
                    elif time.time() - stabilization_start_time >= stabilization_duration:
                        print("QR CODE CENTRALIZED AND STABILIZED. LANDINGGGGG")
                        drone.land()
                        break
                else:
                    stabilization_start_time = None

                pid_error_x = pid_x.calculations(error_x,dt)
                pid_error_y = pid_y.calculations(error_y, dt)
                command_x = 0
                command_y = 0
                if not centralized_x:
                    command_x = np.clip(pid_error_x,-max_speed,max_speed)
                if not centralized_y:
                    command_y = np.clip(pid_error_y,-max_speed,max_speed)
                if not centralized_x or not centralized_y:
                    print(command_x, command_y)
                    drone.rc_control(int(command_x), int(command_y), 0,0)
                    time.sleep(2)
                    drone.rc_control(0,0,0,0)



                mission_log.append({
                    "searchtime":mission_time,
                    "timestamp": round(current_time - start_adjustment_time, 3),
                    "error_x": error_x,
                    "error_y": error_y,
                    "pid_error_x": pid_error_x,
                    "pid_error_y": pid_error_y,
                    "qr_center_x": qr_center_x,
                    "qr_center_y": qr_center_y,
                    "frame_center_x": frame_center_x,
                    "frame_center_y": frame_center_y
                })

                        
        finally:
            if mission_log:
                print("Saving mission log...")
                save_mission_data(mission_log)
            else:
                print('Could not save ')


def mission(drone):
    
    qr_landing_found = False
    max_speed = 3
    tolerance = 8
   
    try:

        drone.streamon()     
        time.sleep(5)
        drone.takeoff()   
        time.sleep(10)

        """Calibrate the pixels"""
        height = drone.current_height()
        height_cm = int(height.replace('dm', '').strip()) * 10 if height else 100
        pixels_to_cm = calibration(height_cm)
        print(f'height: {height_cm} cm | pixels_to_cm: {pixels_to_cm:.2f}')

        
        """ Mission qr codes - considering squared area of search in cm"""

        start_mission_time = time.time()
        qr_landing_found = qrcodes_navigation(drone)
        end_mission_time = time.time()
        mission_time = round(end_mission_time - start_mission_time)
        print(f'Time of mission:{mission_time} seconds')

        time.sleep(5)

        """Phase of adjust to landing using control to landing in last qr code"""
        if qr_landing_found:
            control(drone, qr_landing_found, pixels_to_cm, max_speed, tolerance, mission_time)
      
    finally:
        drone.close_all()
    

def main():
    drone = tello_command("192.168.10.1", 8889, 11111)
    mission(drone)
    

if __name__ == "__main__":
    main()