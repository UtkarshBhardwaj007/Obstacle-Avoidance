#import dronekit-sitl
import dronekit
from dronekit import *
import time
from datetime import datetime
import cv2
from cv2 import *
import numpy as np
import os
import sys 
import pickle
import copy
from math import radians
from geopy.distance import vincenty 
from math import *

vehicle = connect("127.0.0.1:14550")

min_disp = 2
num_disp = 10
block_size = 10
mapcolor = 2
camera_r = 2
camera_l = 4
width = 320
height =240
FPS = 120
median=3

stereoRemap = pickle.load( open( 'calibration_paramsstereoRemap.p', 'rb' ) )
mapxL = stereoRemap['mapxL']
mapyL = stereoRemap['mapyL']
mapxR = stereoRemap['mapxR']
mapyR = stereoRemap['mapyR']

widthPixel = width
heightPixel = height

img_l = cv2.VideoCapture(camera_l)
img_r = cv2.VideoCapture(camera_r)
img_l.set(3, width) # Set resolution width
img_l.set(4, height) # Set resolution height
img_r.set(3, width) # Set resolution width
img_r.set(4, height) # Set resolution hight

n = 1

def arm_and_takeoff(aTargetAltitude):
    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode  = VehicleMode("GUIDED")

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

def distance(lat1, lat2, lon1, lon2): 
    dist=vincenty((lat1,lon1),(lat2,lon2)).km
    return dist 
    """ 
    # The math module contains a function named 
    # radians which converts from degrees to radians. 
    lon1 = radians(lon1) 
    lon2 = radians(lon2) 
    lat1 = radians(lat1) 
    lat2 = radians(lat2) 
    
    # Haversine formula  
    dlon = lon2 - lon1  
    dlat = lat2 - lat1 
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2

    c = 2 * asin(sqrt(a))  
     
    # Radius of earth in kilometers. Use 3956 for miles 
    r = 6371
       
    # calculate the result 
    return(c * r)
    """
# velocity_x and velocity_y are parallel to the North and East directions, UP is negative

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame -> Positions are relative to the vehicle’s current position i.e. x=1,y=2,z=3 is 1m North, 2m East and 3m below the current position. Velocity is in NED frame.
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)
    # send command to vehicle on 1 Hz cycle
    """
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    """


waypoint_list = [[28.753890,77.116342], [1,1]]
# waypoint_list=[]
# with  open("qw",'r+') as wp_file:
#   lines=wp_file.readlines()
#   count=0
#   for line in lines:
#       if count==0:
#           count+=1
#           continue
#       else:
#           param=line.strip().split()
#           waypoint_list.append((float(param[8]),float(param[9])))
    
#   waypoint_list.append((1,1))
    
#print(waypoint_list)
i = 0
waypoint = waypoint_list[i]

def forward_vel(zz = 0):
    waypoint=waypoint_list[zz]
    velocity_threshold=0.002
    vmax=1
    v = [0,0]
    radius_of_waypoint = 0.007
    pos1 = [vehicle.location.global_frame.lat,vehicle.location.global_frame.lon]
    d_L = distance(pos1[0],waypoint[0],pos1[1],waypoint[1])
    d_l1 = distance(waypoint[0], pos1[0], waypoint[1], waypoint[1])
    d_l2 = distance(pos1[0], pos1[0], waypoint[1], pos1[1])
    print("DL",d_L)
    try:    
        v=[(1 - velocity_threshold/d_l1)*vmax, (1 - velocity_threshold/d_l2)*vmax]
    except Exception as err:
        print("Error in forward_vel",err)   
    if d_L < radius_of_waypoint:
        zz += 1
        #vehicle.mode  = VehicleMode("GUIDED")
        vehicle.mode=VehicleMode("LAND")
    return (v,zz)
def forward_vel2(blocknum,velocity_max, zz=0):  
    v=(1-2/blocknum)*velocity_max
    return (v,zz)

def forward_vel3(block_num,center_block_meanval,velocity_max):
    if block_num ==3:
        dist=(-4/115)*(center_block_meanval -137) +8
        vel= velocity_max*(1-(8/dist))
        return (dist,vel)
    if block_num ==2:
        dist=(-8/73)*(center_block_meanval -137) +8
        vel= velocity_max*(1-(8/dist))
        return (dist,vel)
    if block_num ==1:
        dist=(-1/4)*(center_block_meanval -64) +16
        vel= velocity_max*(1-(8/dist))
        return (dist,vel)
kiki = forward_vel()
resha = kiki[0]*sin(pi*vehicle.heading/180) + kiki[1]*cos(pi*vehicle.heading/180)
velocity_along_x = kiki[0]*cos(pi*vehicle.heading/180) - kiki[1]*sin(pi*vehicle.heading/180)
# weight_vel = sqrt(resha**2 + velocity_along_x**2)

# def findgrp(ini_height, fin_height, ini_width, fin_width):
#       i=0
#       j=0
#       k=0
#       g1 = 0
        
#       for i in range (int(ini_height), int(fin_height)):
#           for j in range (int(ini_width), int(fin_width)):
#               l=img[i,j]
#               g1+=l
#               k+=1
#       g1 = (g1/k)
#       if (g1<60):
#           return (1)
#       elif (g1>=60 and g1<125):
#           return (2)
#       elif (g1>=125 and g1<=255):
#           return (3)  
    
def calculateDisparity(framePair):

        isSGBM=1
        if(isSGBM):
            window_size=3
            stereo = cv2.StereoSGBM_create(minDisparity =2,
                                            numDisparities =64,
                                            preFilterCap=63,
                                            blockSize = 3,
                                            uniquenessRatio =15,
                                            speckleWindowSize =180,
                                            speckleRange = 2,
                                            disp12MaxDiff = 1,
                                            #fullDP = True,
                                            P1 =64*3*window_size**2,
                                            P2 = 80*3*window_size**2,
                                            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY )
            stereoR=cv2.ximgproc.createRightMatcher(stereo) # Create another stereo for right this time


            #stereo = cv2.StereoSGBM_create(minDisparity=0, numDisparities=64,
            #blockSize=2*1+1)  
        else:
            print("steroBM")
            stereo = cv2.StereoBM_create(numDisparities=16,blockSize=9)
        #print stereo
        # Compu te disparity.    
        lmbda = 8000
        sigma = 1.0
        visual_multiplier = 1.0
         
        wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereo)
        wls_filter.setLambda(lmbda)
        wls_filter.setSigmaColor(sigma)

        disparity = stereo.compute(gray_l, gray_r)
        disp= stereo.compute(gray_l,gray_r)#.astype(np.float32)/ 16
        # disp= ((disp.astype(np.float32)/ 16)-min_disp)/num_disp 
        dispL= disp
        dispR= stereoR.compute(gray_r,gray_l)
        dispL= np.int16(dispL)
        dispR= np.int16(dispR)

        # Using the WLS filter
        filteredImg= wls_filter.filter(dispL,gray_l,None,dispR)
        filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
        filteredImg = np.uint8(filteredImg)
        disparity =filteredImg
        np.set_printoptions(threshold=np.inf,linewidth=np.inf)
        #@print disparity
        #cv2.waitKey(0)
        #print("xxxxxxxxxxxxx",disparity.min(), disparity.max())   
        return disparity
        if 1: # post process for plot.
            # Convert to plot value.
            disparity = disparity/16 #Scale to (0~255) int16
            #print("xxxxxxxxxxxxx",disparity.min(), disparity.max())                
            #disparity = disparity.astype(np.float16)/disparity.max().astype(np.float16)*255.0
            disparity = disparity.astype(np.float16)/55.0*255.0
            #print(disparity.min(), disparity.max())                
            disparity = disparity.astype(np.uint8)

            #print(disparity.min(), disparity.max())
            #disparity = cv2.medianBlur(disparity, 5)
        return disparit
print("takeoff")
arm_and_takeoff(10)
print("takeoffcompleted")
if vehicle.mode == VehicleMode("GUIDED"):
    lat,lon=waypoint_list[0]
    point = dronekit.LocationGlobalRelative(lat,lon,4) #lat, long, alt
    #vehicle.simple_goto(point)
    #time.sleep(3)

i = 0
j = 0
rows, cols = (3, 3)
a = [[0 for i in range(cols)] for j in range(rows)]
midpt_of_roi_x = [[0 for i in range(cols)] for j in range(rows)]
midpt_of_roi_y = [[0 for i in range(cols)] for j in range(rows)]
vel_scaling_in_down = [[0 for i in range(cols)] for j in range(rows-1)]
vel_scaling_in_right = [[0 for i in range(cols-1)] for j in range(rows)]

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_names=os.listdir("./videos")
out = cv2.VideoWriter('./videos/video'+str(len(video_names))+'.mp4',fourcc, 10.0, (660, 205))

video_names1=os.listdir("./videos")

out1 = cv2.VideoWriter('./depth_maps/depth_map'+str(len(video_names1))+'.mp4',fourcc, 10.0, (220, 205))

#start2=time.time()
while True:
    
    start=time.time()
    try:
        retR, frameR= img_r.read()
    except:
        print("can't read right frame")
    try:
        retL, frameL= img_l.read()
    except:
        print("can't read left frame")

    isDisplay=1
    isRemap=1
    isBlend=0
    isDisparity=1
    isShow=1

    frame_l =cv2.remap(frameL, mapxL, mapyL, cv2.INTER_LINEAR)
    frame_r =cv2.remap(frameR, mapxR, mapyR, cv2.INTER_LINEAR)
    frame_l1 =cv2.remap(frameL, mapxL, mapyL, cv2.INTER_LINEAR)
    frame_r1 =cv2.remap(frameR, mapxR, mapyR, cv2.INTER_LINEAR)
    #print("hello3")
    for line in range(0, int(frameL.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
        frameR[line*20,:]= (0,0,255)
        frameL[line*20,:]= (0,0,255)
    for line in range(0, int(frameL.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
        frame_l[line*20,:]= (0,0,255)
        frame_r[line*20,:]= (0,0,255)        
    #orignal_image= np.hstack((frameL, frameR))
    
    #imgTwin = np.hstack((frame_l, frame_r))
    
    gray_l = cv2.cvtColor(frame_l1,cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(frame_r1,cv2.COLOR_BGR2GRAY)
    
    fp=[gray_l,gray_r]
    disparity=calculateDisparity(fp)

    isMedian=1
    if isMedian:
        disparity = cv2.medianBlur(disparity, median)
    #tem=np.stack((disparity,disparity,disparity),axis=-1)
    #print(tem.shape)
    #out.write(tem)
    
    
    img = disparity.copy()
    print("hello") 
    z = img.shape
    width = z[1]
    height = z[0]
    #print("fov1",1/(time.time()-start))
    img=img[20:225,70:290].astype("uint8")

    kiki = forward_vel()
    #print("fov11",1/(time.time()-start))

    resha = kiki[1]
    velocity_along_x = kiki[0]
    #weight_vel = sqrt(resha**2 + velocity_along_x**2)
    #print("fov22",1/(time.time()-start))

    height,width=img.shape
    for i in range (3):
        for j in range (3):
            
            #a[i][j] = findgrp(i*height/3, (i+1)*height/3, j*width/3, (j+1)*width/3)
            
            temp= np.mean(img[int(i*height/3): int((i+1)*height/3),int( j*width/3): int((j+1)*width/3)])
            #consider maximum fixel value for center block          
            if  i==1 and j==1:
                temp=np.max(img[int(i*height/3): int((i+1)*height/3),int( j*width/3): int((j+1)*width/3)])      
            if (temp<60):
                a[i][j]=1
            elif (temp>=60 and temp<125):
                a[i][j]=2
            elif (temp>=125 and temp<=255):
                a[i][j]=3
            
            midpt_of_roi_x[i][j] = ((2*i)+1)*height/6
            midpt_of_roi_y[i][j] = ((2*j)+1)*width/6

    cent_x = midpt_of_roi_x[1][1]
    cent_y = midpt_of_roi_y[1][1]

    #print("fov2",1/(time.time()-start))    
    for i in range (3):
        for j in range (3):
            midpt_of_roi_x[i][j] = cent_x - midpt_of_roi_x[i][j]
            midpt_of_roi_y[i][j] = cent_y - midpt_of_roi_y[i][j]
    np.array(a)
    np.array(midpt_of_roi_x)
    np.array(midpt_of_roi_y)
    for i in range (2):
        for j in range (3):
            if (i<1):
                if (j==1):
                    vel_scaling_in_down[i][j] = a[i][j] / midpt_of_roi_x[i][j]
                else:
                    vel_scaling_in_down[i][j] = a[i][j] / (2*midpt_of_roi_x[i][j])
            else:
                if (j==1):
                    vel_scaling_in_down[i][j] = a[i+1][j] / midpt_of_roi_x[i+1][j]
                else:
                    vel_scaling_in_down[i][j] = a[i+1][j] / (2*midpt_of_roi_x[i+1][j])
    #print("fov3",1/(time.time()-start))
    for i in range (3):
        for j in range (2):
            if (j<1):
                if (i==1):
                    vel_scaling_in_right[i][j] = a[i][j] / midpt_of_roi_y[i][j]
                else:
                    vel_scaling_in_right[i][j] = a[i][j] / (2*midpt_of_roi_y[i][j])
            else:
                if (i==1):
                    vel_scaling_in_right[i][j] = a[i][j+1] / midpt_of_roi_y[i][j+1]
                else:
                    vel_scaling_in_right[i][j] = a[i][j+1] / (2*midpt_of_roi_y[i][j+1])
    #print("fov4",1/(time.time()-start))
    img=np.stack((img,img,img),axis=-1)
    out1.write(img) 
    #print(img.shape)
    #out.write(img)
    temp=img.copy()
    #print("fov5",1/(time.time()-start))
    centerwin_dist_val=0    
    for i in range(rows):
        for j in range(cols):
            if  i==1 and j==1:
                centerwin_dist_val=np.max(img[i*int(height/rows):(i+1)*int(height/rows),j*int(width/cols):(j+1)*int(width/cols)])
                temp[i*int(height/rows):(i+1)*int(height/rows),j*int(width/cols):(j+1)*int(width/cols)]=np.max(img[i*int(height/rows):(i+1)*int(height/rows),j*int(width/cols):(j+1)*int(width/cols)])
                
            else:           
                temp[i*int(height/rows):(i+1)*int(height/rows),j*int(width/cols):(j+1)*int(width/cols)]=np.mean(img[i*int(height/rows):(i+1)*int(height/rows),j*int(width/cols):(j+1)*int(width/cols)])

    
    down = np.mean(vel_scaling_in_down)
    right = np.mean(vel_scaling_in_right)
    down = down * (height/18)
    right = right * (width/18)
    # print(down, right)
    line_x=down*(height/2)
    line_y=right*(width/2)
    if down>=0:
        cv2.line(temp,(int(width/2+line_y),int(height/2+line_x)),(int(width/2),int(height/2)),(255,0,0),2)
        cv2.line(img,(int(width/2+line_y),int(height/2+line_x)),(int(width/2),int(height/2)),(255,0,0),2)
    elif down<0:
        cv2.line(temp,(int(width/2+line_y),int(height/2+line_x)),(int(width/2),int(height/2)),(0,0,255),2)
        cv2.line(img,(int(width/2+line_y),int(height/2+line_x)),(int(width/2),int(height/2)),(0,0,255),2)
    
    #cv2.imshow("temp",temp.astype("uint8"))
    #cv2.imshow("dtpe1",img)
    frame_l1=frame_l1[20:225,70:290]
    final=np.hstack((img,frame_l1,temp))
    #print(final.shape)
    scaling_factor=3
    summ = np.sum(a)
    print("forward velocity to reach dest: ",velocity_along_x,"right velocity to reach dest: ", resha,"net right velocity", resha + 8*right,"net down velocit not used rn:", 0.8*down)
    print("height",vehicle.location.global_relative_frame.alt)
    if (vehicle.location.global_relative_frame.alt>=3 and vehicle.location.global_relative_frame.alt<=20):
        
        if (a[1][1] != 1):
            #send_ned_velocity(velocity_along_x, resha + 0.5*right, 0.5*down, None)
            #velocity_along_x=forward_vel2(a[1][1],.5)
            #print("temp",centerwin_dist_val)
            #dist,velocity_along_x=forward_vel3(a[1][1],centerwin_dist_val,.5)
            dist=0
            FONT=cv2.FONT_HERSHEY_SIMPLEX
            text="region "+str(a[1][1])+" dist: "+" {0:.4f}".format(dist)   
            cv2.putText(final,text,(20,20),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            text1="velocity: "+" {0:.4f}".format(velocity_along_x)+" {0:.4f}".format(resha +right*scaling_factor) +" "+ "{0:.4f}".format(0.0000)
            cv2.putText(final,text1,(20,200),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            text2="Repulsion: "+" {0:.4f}".format(velocity_along_x)+" {0:.4f}".format(right*scaling_factor) +" "+ "{0:.4f}".format(down)
            cv2.putText(final,text2,(20,180),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            #send_ned_velocity(2,0, 0, None)
            
            #send_ned_velocity(velocity_along_x, resha, 0, None)
            #send_ned_velocity(velocity_along_x,right*scaling_factor, 0, None)
        else:   
            FONT=cv2.FONT_HERSHEY_SIMPLEX
            text="region "+str(a[1][1])
            cv2.putText(final,text,(20,20),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            text1="velocity: "+" {0:.4f}".format(velocity_along_x)+" {0:.4f}".format(resha) +" "+ "{0:.4f}".format(0.0000)
            cv2.putText(final,text1,(20,200),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            text2="Repulsion: "+" {0:.4f}".format(velocity_along_x)+" {0:.4f}".format(right*scaling_factor) +" "+ "{0:.4f}".format(down)
            cv2.putText(final,text1,(20,180),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            
                
            #send_ned_velocity(2, 0, 0, None)       
            send_ned_velocity(velocity_along_x*cos(pi*(vehicle.heading/180))-resha*sin(pi*vehicle.heading/180),0,None)
    else:
        if (a[1][1] != 1):
            #send_ned_velocity(velocity_along_x, resha + 0.5*right, 0.5*down, None)
            #velocity_along_x=forward_vel2(a[1][1],.5)
            #print("temp",centerwin_dist_val)
            dist,velocity_along_x=forward_vel3(a[1][1],centerwin_dist_val,.5)
            FONT=cv2.FONT_HERSHEY_SIMPLEX
            text="region "+str(a[1][1])+" dist: "+" {0:.4f}".format(dist)   
            cv2.putText(final,text,(20,20),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            text1="velocity: "+" {0:.4f}".format(velocity_along_x)+" {0:.4f}".format(resha +right*scaling_factor) +" "+ " {0:.4f}".format(0.0000)
            cv2.putText(final,text1,(20,200),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            text2="Repulsion: "+"{0:.4f}".format(velocity_along_x)+" {0:.4f}".format(right*scaling_factor) +" "+ "{0:.4f}".format(down)
            cv2.putText(final,text2,(20,180),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            print("part1")
            
            
        else:   
            FONT=cv2.FONT_HERSHEY_SIMPLEX
            text="region "+str(a[1][1]) 
            cv2.putText(final,text,(20,20),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            text1="velocity: "+"{0:.4f}".format(velocity_along_x)+" {0:.4f}".format(resha) +" "+ " {0:.4f}".format(0.0000)
            cv2.putText(final,text1,(20,200),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            text2="Repulsion: "+"{0:.4f}".format(velocity_along_x)+" {0:.4f}".format(right*scaling_factor) +" "+ "{0:.4f}".format(down)
            cv2.putText(final,text2,(20,180),FONT,.4,(0,255,0),1,cv2.LINE_AA)
            print("part2")
                
            
                
                        
            
    # elif (vehicle.location.global_relative_frame.alt>15):
    #   while not (vehicle.location.global_relative_frame.alt< 14):
    #       send_ned_velocity(velocity_along_x, resha, 0.5*(vehicle.location.global_relative_frame.alt-14)/14, None)
    # elif (vehicle.location.global_relative_frame.alt < 3):
    #   while not (vehicle.location.global_relative_frame.alt>=4):
    #       send_ned_velocity(velocity_along_x, resha, 0.5*(vehicle.location.global_relative_frame.alt-4)/vehicle.location.global_relative_frame.alt, None)
    cv2.imshow('together',final)
    out.write(final.astype('uint8'))
    if cv2.waitKey(1)== 27 :
        break
    #if time.time()-start2>180:
        #break
    
    fps = 1/(time.time() - start)
    print("fps",fps)
img_l.release()
cv2.destroyAllWindows()
out.release()
out1.release()
