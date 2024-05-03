import asyncio
import time
from ultralytics import YOLO
from mavsdk import System
from mavsdk.offboard import (OffboardError, Attitude)
import cv2


id_target = 0
async def run():

    # Khởi tạo hệ thống MAVSDK
    model = YOLO("model.engine")
    drone = System()
    cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-raw, format = YUY2, width=640, height=480, framerate=30/1  ! videoconvert ! video/x-raw,format=BGR ! appsink")
    
    while True:
        # Lấy ảnh từ camera UAV
        success, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Xử lý ảnh bằng YOLOv8 để phát hiện đối tượng
        results = model.track(frame, persist = True, conf = 0.5)
    
          
        data = results[0]
        boxes = data.boxes.cpu().numpy()
    
        id_target = select_id_vehicle(boxes.id, boxes.conf)
        
        if id_target == None:
            cv2.imshow('Tracking', frame)
        else:
            box= get_box_coordinate(boxes=boxes, id_target= id_target)
            control_uav(drone, box)
            x1, y1, x2, y2 = box
            print(f"{x1}  {y1}  {x2}  {y2}")
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
            cv2.putText(frame, f"tank: {id_target}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA, False)
            cv2.imshow('Tracking', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            await land(drone)
            break
        
async def control_uav(drone, box):
    # Điều khiển UAV dựa trên vị trí tương đối của đối tượng
    x1, y1, x2, y2 = box
    xO = 319
    yO = 319
    x_center = round(int(x1 + x2)/2)
    y_center = round(int(y1 + y2)/2)
    pitch_angle = (int(x_center- xO)/320)*90
    yaw_angle = (int(y_center- yO)/320)*90    
    await drone.offboard.set_attitude(Attitude(pitch_angle, 0.0, yaw_angle, 0.1))
    
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

async def arm_drone(drone):
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
    

    await takeoff(drone = drone)
    
if __name__ == "__main__":
    asyncio.run(run())
