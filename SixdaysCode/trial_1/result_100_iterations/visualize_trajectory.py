#!/usr/bin/env python
# coding: utf-8

# In[1]:


import cv2
import math
import numpy as np
import vtk
import numba
from numba import jit
from numba import njit,prange
from vtk.util.numpy_support import vtk_to_numpy


# In[2]:


cap = cv2.VideoCapture("/home/tomlucas/ZaVi/Dokumentation/Paper/kolleripin19/Visualize_TRajectory/video.MP4")
if not cap.isOpened():
    cap.open("/home/tomlucas/ZaVi/Dokumentation/Paper/kolleripin19/Visualize_TRajectory/video.MP4")
print(cap.get(3))
def get_coords(event, x, y, flags, param):
    print((x,y))
cv2.namedWindow("image")
#cv2.setMouseCallback("image", get_coords)


# In[3]:


reader = vtk.vtkPLYReader()
print("Opening PC")
reader.SetFileName("Rennbahn_smoothed.ply")
#reader.ReadAllScalarsOn()
#reader.ReadAllVectorsOn()
"""reader.Update()
polydata = reader.GetOutput()
points = polydata.GetPoints()
print(polydata.GetVerts())
print(points)
array = points.GetData()
numpy_nodes = vtk_to_numpy(array)
print(numpy_nodes.shape)


# In[4]:


numpy_nodes=numpy_nodes[::100,:]
numpy_nodes=numpy_nodes.reshape(1,-1,3)
print(numpy_nodes.shape)
shift=np.array([57.2363,-11.602,2.10503])
numpy_nodes[:,:]+=shift
numpy_nodes*=1"""


# In[5]:


estimation=np.fromfile("trajectory",float)
estimation=estimation.reshape((1,-1,36))
velocity=np.linalg.norm(estimation[:,:,3:6],axis=2)
estimation=estimation[:,:,0:3]

input_mask=np.fromfile("input_mask",bool)
print(input_mask.shape)
#estimation[:,3]=1


# In[6]:


print(np.max(estimation))


@jit(nopython=True)
def euler_matrix(roll,pitch,yaw):
    ca=math.cos(yaw)
    sa=math.sin(yaw)
    cb=math.cos(pitch)
    sb=math.sin(pitch)
    cc=math.cos(roll)
    sc=math.sin(roll)
    return np.array([[ca*cb,ca*sb*sc-sa*cc,ca*sb*cc+sa*sc],[sa*cb,sa*sb*sc+ca*cc,sa*sb*cc-ca*sc],[-sb,cb*sc,cb*cc]])




# In[7]:
roll=-4.31
pitch=-0.01
yaw=0.0
x=-38.5
y=4.5
z=14.5
rvec=np.array([roll,pitch,yaw],dtype='float32')
tvec=np.array([x,y,z],dtype='float32')
                    
from pynput import keyboard
from pynput import mouse
import pynput
def on_press(key):
    global roll,pitch,yaw,rvec
    try:        
        print(key.char)
        if key.char =="r":
            roll+=0.01
        if key.char=="f":
            roll-=0.01
        if key.char =="t":
            pitch+=0.01
        if key.char=="g":
            pitch-=0.01
        if key.char =="z":
            yaw+=0.01
        if key.char=="h":
            yaw-=0.01
        if key.char=="u":
            tvec[0]+=0.5
        if key.char=="j":
            tvec[0]-=0.5
        if key.char=="i":
            tvec[1]+=0.5
        if key.char=="k":
            tvec[1]-=0.5
        if key.char=="o":
            tvec[2]+=0.5
        if key.char=="l":
            tvec[2]-=0.5
        if key.char=='q':
            cap.release()
    except AttributeError:
        print('special key {0} pressed'.format(
            key))
    rvec, jacob=cv2.Rodrigues(euler_matrix(roll,pitch,yaw))
    print(roll,pitch,yaw)
    print(tvec)
        
listener = keyboard.Listener(
    on_press=on_press)
#listener.start()


# In[8]:


#objectPoints=np.array([[0,0,0],[0,0,0],[0,0,0],[0,0,0]], dtype='float32')
#imagePoints=np.array([[0,0],[0,0],[0,0],[0,0]], dtype='float32')
#transform=cv2.getPerspectiveTransform(objectPoints, imagePoints)
#[[ 1.42953945]
# [-0.04289905]
# [-0.04940806]]
#[ -25. -425.  220.]

cam_matrix=np.array([[ 676.66944111 ,   0.       ,   952.56192542],
 [   0.     ,     630.5457264   ,541.34202629],
 [   0.      ,      0.         ,   1.        ]]
)
dist_coeff=np.array([[-0.17785299  ,0.05237014 , 0.00626027,  0.00769161 ,-0.00639858]])

# In[9]:

"""obj_pts=np.array([[6.,18.,0.],[70.,18.,0.],[38.,18.,0.],[38.,30.,0.]],np.float32)
img_pts=np.array([[268.,545.],[1695.,545.],[952.,507.],[952.,632.]],np.float32)



width=7
height=7
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((width*height,3), np.float32)
objp[:,:2] = np.mgrid[0:width,0:height].T.reshape(-1,2)*0.11
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
objpoints.append(obj_pts)
imgpoints = [img_pts] # 2d points in image plane.
cap = cv2.VideoCapture("calib2.MP4")
#for fname in images:
while cap.isOpened():
    for i in range(10):
        cap.read()
    if(not cap.isOpened()):
        break
    ret, img = cap.read()
    if(not ret):
        break
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv.imshow("gray",gray)
    #cv.waitKey(0)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (width,height),None, cv2.CALIB_CB_NORMALIZE_IMAGE)
   
    # If found, add object points, image points (after refining them)
    #print(corners)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (width,height), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(1)
cv2.destroyAllWindows()

cap = cv2.VideoCapture("/home/tomlucas/ZaVi/Dokumentation/Paper/kolleripin19/Visualize_TRajectory/video.MP4")
if not cap.isOpened():
    cap.open("/home/tomlucas/ZaVi/Dokumentation/Paper/kolleripin19/Visualize_TRajectory/video.MP4")
print(cap.get(3))

ret, img=cap.read()
print(ret)
cv2.waitKey(1)
print(img.shape[1::-1])

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[:2], cam_matrix, dist_coeff)
print(mtx)
print(dist)"""
#rvec=np.array(rvecs[0])
#tvec=tvecs[0]
print(tvec)
print(rvec)
#print("Calibration resulted in "+str(ret))
print("Grabbing IMages")

cap.set(cv2.CAP_PROP_POS_FRAMES, 8221)
print("Starting Images")
cv2.startWindowThread()
#@jit(nopython=True)



traj_freq=100.*0.001
cam_freq=24.
img=cap.read()
start_time=cap.get(cv2.CAP_PROP_POS_MSEC)
print(estimation.shape)
roi_size=60
roi_higher_size=440
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.mp4',fourcc, 24., (1920,1080))
def do_loop():
    global rvec,tvec,cam_matrix,dist,cap, cur_frame, cam_freq, traj_freq
    while(cap.isOpened()):
        
        # Capture frame-by-frame
        ret, image = cap.read()
        cur_point=int((cap.get(cv2.CAP_PROP_POS_MSEC)-start_time)*traj_freq)
        if(cur_point > 0 and cur_point < estimation.shape[1]):
            imagePoints, jacobian= cv2.projectPoints(estimation[0,cur_point], rvec, tvec, cam_matrix, dist_coeff)
            #print(imagePoints.shape)
            imagePoints[:,0,0]=np.clip(imagePoints[:,0,0],0,1919)
            imagePoints[:,0,1]=np.clip(imagePoints[:,0,1],0,1079)
            imagePoints=imagePoints[:,0,::-1].astype(int)
                    

            #print(np.max(imagePoints[:,0]),np.max(imagePoints[:,1]))
            #image[imagePoints[:,0],imagePoints[:,1]]=(0,0,255)
            cv2.circle(image, (imagePoints[0,1],imagePoints[0,0]), 5,  (0,0,255) if input_mask[cur_point] else (255,0,0), thickness=-1, lineType=8, shift=0) 
            roi=image[imagePoints[0,0]-roi_size:imagePoints[0,0]+roi_size,imagePoints[0,1]-roi_size:imagePoints[0,1]+roi_size] 
            roi=cv2.resize(roi,(roi_higher_size,roi_higher_size))
            image[-roi_higher_size:,-roi_higher_size:]=roi       
            cv2.rectangle(image,(imagePoints[0,1]-roi_size,imagePoints[0,0]-roi_size),(int(imagePoints[0,1]+roi_size),imagePoints[0,0]+roi_size),(255,0,0),5)            
            cv2.rectangle(image,(image.shape[1]-roi_higher_size,image.shape[0]-roi_higher_size),(image.shape[1],image.shape[0]),(255,0,0),5)
            cv2.rectangle(image,(int(image.shape[0]*0.5)-100,900-100),(int(image.shape[0]*0.5)+700,900+80),(255,255,255),-1)
            cv2.putText(image,"Velocity = " + str(round(velocity[0,cur_point],2))+"m/s",(int(image.shape[0]*0.5),900),cv2.FONT_HERSHEY_SIMPLEX,2, (0,0,0),8)
         
        # Display the resulting frame
        cv2.imshow('image',image)
        out.write(image)
  
do_loop()

# When everything done, release the capture
cap.release()
out.release()
Cv2.destroyAllWindows()

print(rvec)
print(tvec)

# # 

# In[ ]:




