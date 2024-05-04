import time
import threading
import numpy as np
import asyncio
import time
from ultralytics import YOLO
from mavsdk import System
from mavsdk.offboard import (OffboardError, Attitude)
from deep_sort_realtime.deepsort_tracker import DeepSort
from collections import Counter
import cv2
# from deep_sort_realtime.deepsort_tracker import DeepSort

# GLOBAL VARIABLE
box_global = [0.0, 0.0, 0.0, 0.0]
tracking = False
system_running = True
tracker = cv2.TrackerCSRT_create()
model = YOLO("model.engine")
drone = System()

# YOLO MODEL TRACKING
def track_yolo(index):
    cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw, format = YUY2, width=640, height=480, framerate=30/1  ! videoconvert ! video/x-raw,format=BGR ! appsink")
    global tracking
    global box_global
    count = 0
    while True: 
        success, frame = cap.read()
    
        if not success:
            break
        
        if tracking == False:
            print('aaa')
            count = count +1 
            results = model(frame)
            data = results[0]
            boxes = data.boxes.cpu().numpy()  
            for (bbox, conf, cls) in zip(boxes.xyxy, boxes.conf, boxes.cls):

                x1,y1,x2,y2 = map(int, bbox)
            
                
                if int(cls) is None:
                    if conf < 0.8:
                        continue
                else:
                    if conf < 0.8:  
                        continue
                
                print(x1, y1, x2, y2) 
                if count >50:
                    tracker.init(frame, (x1,y1,abs(x1 - x2),abs(y1 -y2)))
                    tracking = True
                    count = 0
                    break
        
        if tracking:
            success, bbox = tracker.update(frame)
            if success:
            # Theo dõi thành công
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                box_global = [int(bbox[0]), int(bbox[1]),int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])]
                cv2.rectangle(frame, p1, p2, (255, 0, 0), 2)
            else:
                # Theo dõi thất bại
                tracking = False
                cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

            # Hiển thị kết quả
            cv2.imshow("Frame", frame)
            
            # Thoát nếu nhấn 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            cv2.imshow("Frame", frame)
        
            
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
        return    # thrust = 0.1
    
    # for i in range(100):
    #     thrust = thrust - 0.001*int(i)
    #     await drone.offbroard.set_attitude(Attitude(0.0, 0.0, 0.0, thrust))
    #     await asyncio.sleep(0.01)
    #     break
    
    async def takeoff(drone):
        thrust = 0  
        for i in range(300):
            thrust = int(i)*0.001
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, thrust))
            await asyncio.sleep(0.002)            
    await takeoff(drone = drone)
    await asyncio.sleep(10)
   
   
    await drone.offboard.set_attitude(Attitude(0.0, -10, 0.0, 0.3))
    await asyncio.sleep(10)
    
    while system_running:
        print("run run run")
        global box_global
        print(box_global)
        await control_uav(drone = drone, box_global = box_global)
        await asyncio.sleep(0.1)
           
async def control_uav(drone, box_global):
    # Điều khiển UAV dựa trên vị trí tương đối của đối tượng
    
    x1, y1, x2, y2 = box_global
    xO = 319
    yO = 319
    x_center = round(int(x1 + x2)/2)
    y_center = round(int(y1 + y2)/2)
    pitch_angle = (int(x_center- xO)/320)*90
    yaw_angle = (int(y_center- yO)/320)*90
    
    print(box_global)
    
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
        await drone.offboard.set_attitude(Attitude(0.0, pitch_angle_current, yaw_angle_current, 0.3))
        await asyncio.sleep(0.2)
        
    elif pitch_angle > 0 and yaw_angle < 0:
        print('bbb')
        pitch_angle_current = pitch_angle_current - 1
        yaw_angle_current = yaw_angle_current + 1
        print(f"Euter angle pitch: {pitch_angle_current}")
        print(f"Euter angle yaw : {yaw_angle_current}") 
        await drone.offboard.set_attitude(Attitude(0.0, pitch_angle_current, yaw_angle_current, 0.3))
        await asyncio.sleep(0.2)
            # thrust = 0.1
    
    # for i in range(100):
    #     thrust = thrust - 0.001*int(i)
    #     await drone.offbroard.set_attitude(Attitude(0.0, 0.0, 0.0, thrust))
    #     await asyncio.sleep(0.01)
    #     break
    elif pitch_angle < 0 and yaw_angle < 0:
        print('ccc')
        pitch_angle_current = pitch_angle_current + 1
        yaw_angle_current = yaw_angle_current + 1
        print(f"Euter angle pitch: {pitch_angle_current}")
        print(f"Euter angle yaw : {yaw_angle_current}") 
        await drone.offboard.set_attitude(Attitude(0.0, pitch_angle_current, yaw_angle_current, 0.3))
        await asyncio.sleep(0.2)
        
    else:
        print('ddd')
        pitch_angle_current = pitch_angle_current + 10
        yaw_angle_current = yaw_angle_current - 10
        print(f"Euter angle pitch: {pitch_angle_current}")
        print(f"Euter angle yaw : {yaw_angle_current}") 
        await drone.offboard.set_attitude(Attitude(0.0, pitch_angle_current, yaw_angle_current, 0.3))
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