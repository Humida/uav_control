import time
import threading
import numpy as np
import asyncio
import time
from ultralytics import YOLO
from mavsdk import System
from mavsdk.offboard import (OffboardError, Attitude)
import cv2

# GLOBAL VARIABLE
box_global = [0.0, 0.0, 0.0, 0.0]
system_running = True
id_target  = 0
model = YOLO("model.engine")
drone = System()

# YOLO MODEL TRACKING
def track_yolo(index):
    cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw, format = YUY2, width=640, height=480, framerate=30/1  ! videoconvert ! video/x-raw,format=BGR ! appsink")
    while True:
        # Lấy ảnh từ camera UAV
        success, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Xử lý ảnh bằng YOLOv8 để phát hiện đối tượng
        results = model.track(frame, persist = True)
     
        data = results[0]
        boxes = data.boxes.cpu().numpy()
    
        id_target = select_id_vehicle(boxes.id, boxes.conf)
        
        if id_target == None:
            cv2.imshow('Tracking', frame)
        else:
            box= get_box_coordinate(boxes=boxes, id_target= id_target)
            global box_global
            box_global = box
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
            cv2.putText(frame, f"tank: {id_target}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA, False)
            cv2.imshow('Tracking', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            global system_running
            system_running = False
            break

def select_id_vehicle(ids, confs):
    conf_max = 0
    id = 0
    if ids is not None:
        for id, conf in zip(ids, confs):
            if id_target == id:
                return id_target
            elif conf > conf_max:
                conf_max = conf
                id = id
                
        return id
    elif ids is None and id_target == 0:
        return None 

def get_box_coordinate(boxes, id_target):
    for (id, box) in zip(boxes.id, boxes.xyxy):
                if id == id_target:
                    x1, y1, x2, y2 = box.astype(int)
                    return ([x1, y1, x2, y2])
                
    return [0,0,0,0]

# MAVSDK
async def control_mavsdk():
    await drone.connect(system_address="serial:///dev/ttyUSB0:115200")
    
    async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connect to drone --")
                break
            
    await drone.action.arm()
    
          
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
    print(f"-- Start offboard")
        
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
        {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    async def takeoff(drone):
        thrust = 0  
        for i in range(300):
            thrust = int(i)*0.001
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, thrust))
            await asyncio.sleep(0.002)            
    await takeoff(drone = drone)
    await asyncio.sleep(10)
   
   
    await drone.offboard.set_attitude(Attitude(0.0, 35, 0.0, 0.3))
    await asyncio.sleep(30)
    # while system_running:
    #     print("run run run")
    #     global box_global
    #     print(box_global)
    #     await control_uav(drone = drone, box_global = box_global)
    #     await asyncio.sleep(0.1)
        
    # thrust = 0.1
    # for i in range(100):
    #     thrust = thrust - 0.001*int(i)
    #     await drone.offbroard.set_attitude(Attitude(0.0, 0.0, 0.0, thrust))
    #     await asyncio.sleep(0.01)
    #     break
           
async def control_uav(drone, box_global):
    # Điều khiển UAV dựa trên vị trí tương đối của đối tượng
    x1, y1, x2, y2 = box_global
    xO = 319
    yO = 319
    x_center = round(int(x1 + x2)/2)
    y_center = round(int(y1 + y2)/2)
    pitch_angle = (int(x_center- xO)/320)*90
    yaw_angle = (int(y_center- yO)/320)*90
    
    
    pitch_angle_current = 0
    yaw_angle_current = 0
    
    # euler_angle_result = await get_euler_angle(drone= drone)
    async for attitude in drone.telemetry.attitude_euler():
        print("Euler Angles (roll, pitch, yaw): ", attitude)
        pitch_angle_current = attitude.pitch_deg
        yaw_angle_current = attitude.yaw_deg
        break
    
    print(f"Euter angle pitch: {pitch_angle_current}")
    print(f"Euter angle yaw : {yaw_angle_current}")        
    
    if pitch_angle > 0 and yaw_angle > 0:
        print("aaaa")
        pitch_angle_current = pitch_angle_current - 1
        yaw_angle_current = yaw_angle_current - 1
        print(f"Euter angle pitch: {pitch_angle_current}")
        print(f"Euter angle yaw : {yaw_angle_current}") 
        await drone.offboard.set_attitude(Attitude(pitch_angle_current, 0.0, yaw_angle_current, 0.15))
        await asyncio.sleep(0.2)
        
    elif pitch_angle > 0 and yaw_angle < 0:
        print('bbb')
        pitch_angle_current = pitch_angle_current - 1
        yaw_angle_current = yaw_angle_current + 1
        print(f"Euter angle pitch: {pitch_angle_current}")
        print(f"Euter angle yaw : {yaw_angle_current}") 
        await drone.offboard.set_attitude(Attitude(pitch_angle_current, 0.0, yaw_angle_current, 0.1))
        await asyncio.sleep(0.2)
        
    elif pitch_angle < 0 and yaw_angle < 0:
        print('ccc')
        pitch_angle_current = pitch_angle_current + 1
        yaw_angle_current = yaw_angle_current + 1
        print(f"Euter angle pitch: {pitch_angle_current}")
        print(f"Euter angle yaw : {yaw_angle_current}") 
        await drone.offboard.set_attitude(Attitude(pitch_angle_current, 0.0, yaw_angle_current, 0.15))
        await asyncio.sleep(0.2)
        
    else:
        print('ddd')
        pitch_angle_current = pitch_angle_current + 10
        yaw_angle_current = yaw_angle_current - 10
        print(f"Euter angle pitch: {pitch_angle_current}")
        print(f"Euter angle yaw : {yaw_angle_current}") 
        await drone.offboard.set_attitude(Attitude(pitch_angle_current, 0.0, yaw_angle_current, 0.1))
        await asyncio.sleep(0.2)

async def takeoff(drone):
        thrust = 0  
        for i in range(100):
            thrust = int(i)*0.001
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, thrust))
            await asyncio.sleep(0.2)
                   
async def land(drone):
    thrust = 0.1
    for i in range(100):
        thrust = 0.1 - 0.01*int(i)
        await drone.offbroard.set_attitude(Attitude(0.0, 0.0, 0.0, thrust))
        
def run(index):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(control_mavsdk())
    loop.close()
 
if __name__ == "__main__":
    yolo = threading.Thread(target= track_yolo, args=(10,))
    mavsdk = threading.Thread(target = run, args=(10,))
    
    mavsdk.start()
    yolo.start()
    
    # yolo.join()
    # mavsdk.join()