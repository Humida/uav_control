import cv2
from deep_sort_realtime.deepsort_tracker import DeepSort
from ultralytics import YOLO
from collections import Counter

# load model
model = YOLO('model.engine')

# Read video file
video = cv2.VideoCapture('v4l2src device=/dev/video0 ! video/x-raw, format = YUY2, width=640, height=480, framerate=30/1  ! videoconvert ! video/x-raw,format=BGR ! appsink')

tracking_class = 0
target_id  = 0
tracker = DeepSort(max_age=30)
store_id_track = []

while True:
    # Read a new frame
    success, frame = video.read()
    if not success:
        break
    
    detection = []
    
    results = model(frame)
    data = results[0]
    boxes = data.boxes.cpu().numpy()  
    
    for (bbox, conf, cls) in zip(boxes.xyxy, boxes.conf, boxes.cls):

        x1, y1, x2, y2 = map(int, bbox)

        
        if int(cls) is None:
            print('a')
            if conf < 0.5:
                continue
        else:
            if int(cls) != int(tracking_class) or conf < 0.5:  
              continue
        x, y, w, h = bbox        
        detection.append([ [x1, y1, x2-x1, y2 - y1], conf, cls])
        

    tracks = tracker.update_tracks(detection, frame = frame)
    

    count = 1
    for track in tracks:
        
        if not track.is_confirmed():
            continue
        
        track_id = track.track_id
        store_id_track.append(track_id)
                
        if track_id == target_id:
            count = 1
            ltrb = track.to_ltrb()
            x1, y1, x2, y2 = map(int, ltrb)
            label = "{}".format(track_id)
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1) 
            cv2.putText(frame, label, (x1 + 5, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            continue
        
        count = 0
        # elif track_id != target_id:
        #     store_id_track.append(track_id)
        #     counter = Counter(store_id_track)
        #     most_frequent_element = counter.most_common(1)[0][0]
        #     print(most_frequent_element)
        #     print(store_id_track)
        #     target_id = most_frequent_element
        #     store_id_track = []

    if count == 0:
        
        counter = Counter(store_id_track)
        most_frequent_element = counter.most_common(1)[0][0]
        target_id = most_frequent_element
        store_id_track = []
       
        
    # Display result
    cv2.imshow('Tracking with deepsort', frame)

    # Exit if ESC pressed
    if cv2.waitKey(1) & 0xFF == 27:
        break



# Release resources
video.release()
cv2.destroyAllWindows()
