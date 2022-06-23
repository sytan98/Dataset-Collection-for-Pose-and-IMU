# Importing all necessary libraries 
import cv2 
import os 

# Read the video from specified path 
# cam = cv2.VideoCapture("./data/2022-06-16T17-07-10/Frames.m4v") 
# image_dir = 'D:/Imperial/FYP/captured_data/campus_v2/val/images'
cam = cv2.VideoCapture("./data/2022-06-20T22-00-18/Frames.m4v") 
image_dir = 'D:/Imperial/FYP/captured_data/campus_v3/test/images'
try: 
	
	# creating a folder named data 
	if not os.path.exists(image_dir): 
		os.makedirs(image_dir) 

# if not created then raise error 
except OSError: 
	print ('Error: Creating directory of data') 

# frame 
currentframe = 0

while(True): 

    # reading from frame 
    ret,frame = cam.read() 
    if ret: 
        # if video is still left continue creating images 
        name = image_dir +'/frame_' + str(currentframe) + '.jpg'
        resized = cv2.resize(frame, (341, 256), interpolation = cv2.INTER_AREA)
        # writing the extracted images 
        # if currentframe % 3 == 0:
        print ('Creating...' + name) 
        cv2.imwrite(name, resized) 

        # increasing counter so that it will 
        # show how many frames are created 
        currentframe += 1
    else: 
        break

# Release all space and windows once done 
cam.release() 
cv2.destroyAllWindows() 
