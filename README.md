# RADAR Tracking
## Summary
## Contents

1. [Radar Tracking ver.1](#1-radar-tracking-ver1)
2. [Radar Tracking ver.2](#2-radar-tracking-ver2)
3. [Experiments](#3-experiments)
4. [Running on ROS](#4-running-on-ros)
5. [Appnedices](#5-appendices)




## 1. Radar Tracking ver.1
Radar Tracking version 1 is composed of two parts.     
First part: velocity is obtained from point data of radar by Clustering.    
Because radar get the point data randomly from object, the position and velocity from radar has noise and radar error.   
Therefore we need filtering.   
Second part: position and velocity are filtered by Moving average filtering. 


### 1.1 Obtaining velocity by Clustering
#### 1.1.1 Process diagram
<img src="https://user-images.githubusercontent.com/97038348/167978919-af3d5de5-3e3c-4743-bf78-3edc998449e4.PNG" width="90%" height="90%"/>

 *1. Obtain the point data (x, y, time) from radar*       
 *2. Clustering the points and Detect the objects*    
 *3. Find the center points of objects*   
 *4. Compare the center points to past center points*   
 *5. Clustering the current and past center points and Detect the same object*   
 *6. Obtain the velocity*   


#### 1.1.2 Flowchart
 
 <p align="center"><img src="https://user-images.githubusercontent.com/97038348/166400471-83d88f2a-ff8e-434e-b5ca-2c000356aa05.PNG" width="60%" height="60%"/>
 <p align="center"><img src="https://user-images.githubusercontent.com/97038348/166400476-17d1bc53-cd59-4e24-9c0e-f6224e7b76f4.PNG" width="60%" height="60%"/>


 There are two Clustering. The function of **'Clustering'** is detecting objects and finding the center of each object. The function of **'ClusteringVel'** is   identifying the same objects in current objects and past objects and calculating the velocity.

 
   
   

 
 ### 1.2 Filtering position and velocity
  #### 1.2.1 Window set diagram
 
<img src="https://user-images.githubusercontent.com/97038348/167054433-f13742da-ad87-43cc-bdce-950b1c4ed50c.PNG" width="90%" height="90%"/>

The main concept for filtering is **"window"**. The position and velocity obtained in the previous process become an one element of window.    
The number of elements in window is **"size of window"**.   
**"Skip count"** means number of times that have not been updated consecutively. This number is used for deleting the window.  
 The **"window set"** consists of windows, and the number of windows is equal to the number of objects currently observed.
#### 1.2.2 Formula of moving average filter
<img src="https://user-images.githubusercontent.com/97038348/167053294-ee42ca01-9129-4031-84b3-cccb81380603.PNG" width="80%" height="80%"/>
 
 There are three formula to calculate the average of window. Wieghted moving average and Current-Weighted moving average consider more weight to the recently observed values.   
  
  *# You can select one by adjusting the PARAMETER in RadarTrackingVer1.py*

#### 1.2.3 Flowchart
<p align="center"><img src="https://user-images.githubusercontent.com/97038348/166400479-eb2387c2-22e9-487c-b6cc-180fd53db2a8.PNG" width="60%" height="60%"/>
<p align="center"><img src="https://user-images.githubusercontent.com/97038348/166400483-b2fc2601-7bb3-4e16-aa74-e8a782a9ac6f.PNG" width="60%" height="60%"/>
 
When the new data(position and velocity) is obtained, the new data is compared to the first element of windows in window set by **'ClusteringFilter'**. If the matched window is founded, the new data becomes an first element of that window. Else, new window is created and the new data becomes an first element of the new window. Because Noise always makes new window, window that have not been updated for a long time will be deleted by counting the number of times that have not been updated. The window of object being out of detecting range is also deleted.
 
 
 ### 1.3 PARAMETERS
 
 #### 1.3.1 Clustering parameters

 * **Clustering**  
   eps = size of object  
   min_samples = 2  &nbsp;&nbsp;&nbsp;&nbsp;*# The minimum number of points for clustering. if you increase this number, accuracy would increase.*
 * **ClusteringVel**  
   eps = The range of object moving for time difference + Radar Error    
   min_samples = 1  &nbsp;&nbsp;&nbsp;*# ClusteringVel judges whether it's the same object or not. So, 1 is enough.* 
 * **ClusteringFilter**
   eps = The range of object moving for time difference + Radar Error    
   min_samples = 1  &nbsp;&nbsp;&nbsp;*# ClusteringFilter judges whether it's the same object or not. So, 1 is enough.*    
  
 
 #### 1.3.2 parameter setting
 
 you have to set the parameters depending on what you want to detect.   
  &nbsp;&nbsp;&nbsp;&nbsp; *# You can set these values by adjusting the PARAMETER in RadarTrackingVer1.py* 
 
 <img src="https://user-images.githubusercontent.com/97038348/168008762-bc02b001-799d-446c-9f54-48c350a3fc4e.png" width="30%" height="30%"/>

 * **objectSize** = eps for Clustering
 * **objectMovingRange** = eps for ClusteringVel
 * **filteringRange** = eps for ClusteringFilter
 * **sizeOfWindow** = number of elements in window
 * **maxNumOfSkip** = the number of maximum skip count of window. If skip count over this number, that window will be deleted.
 * **filterMode** =
   * 0: Simple moving average
   * 1: Weighted moving average
   * 2: Current-Weighted moving average
 * **weight** = weight for Current-Weighted moving average
 
 
 
## 2. Radar Tracking ver.2
 
 Radar Tracking version 2 is composed of two parts.     
 First part: the point data from radar is filtered by Moving average filtering..    
 Because radar get the point data randomly from object, the position from radar has noise and radar error.   
 Therefore we need filtering.   
 Second part: velocity is obtained from filtered position.   
 The main difference of Radar Tracking version 1 and version 2 is the order of obtaining velocity and filtering.
 
  ### 2.1 Filtering position
 #### 2.1.1 Process diagram
<img src="https://user-images.githubusercontent.com/97038348/167998744-a5520981-acdb-44ee-83dc-a4cb9c66a010.PNG" width="60%" height="60%"/>

 *1. Obtain the point data (x, y, time) from radar*       
 *2. Clustering the points and Detect the objects*    
 *3. Find the center points of objects*   
 
 #### 2.1.2 Window set diagram
 
 <img src="https://user-images.githubusercontent.com/97038348/167997924-78b74675-0a16-4a8c-94a0-98842ae0bf71.PNG" width="90%" height="90%"/>
 
 The main concept for filtering is **"window"**. The position obtained in the previous process become an one element of window.    
The number of elements in window is **"size of window"**.   
**"Skip count"** means number of times that have not been updated consecutively. This number is used for deleting the window.  
 The **"window set"** consists of windows, and the number of windows is equal to the number of objects currently observed.
 
 #### 2.1.3 Formula of moving average filter
 
<img src="https://user-images.githubusercontent.com/97038348/167053294-ee42ca01-9129-4031-84b3-cccb81380603.PNG" width="80%" height="80%"/>
 
 There are three formula to calculate the average of window. Wieghted moving average and Current-Weighted moving average consider more weight to the recently observed values.   
  
  *# You can select one by adjusting the PARAMETER in RadarTrackingVer2.py*

 
 ### 2.2 Obtaining velocity
 #### 2.2.1 Velocity Window set diagram
 
 <img src="https://user-images.githubusercontent.com/97038348/167997927-8556de63-b46e-4e26-b2e7-429ee3218f75.PNG" width="90%" height="90%"/>
 
 #### 2.2.2 Flowchart
 
 ### 2.3 PARAMETERS
  #### 2.3.1 Clustering parameters

 * **Clustering**  
   eps = size of object  
   min_samples = 2  &nbsp;&nbsp;&nbsp;&nbsp;*# The minimum number of points for clustering. if you increase this number, accuracy would increase.*
 * **ClusteringVel**  
   eps = The range of object moving for time difference + Radar Error    
   min_samples = 1  &nbsp;&nbsp;&nbsp;*# ClusteringVel judges whether it's the same object or not. So, 1 is enough.* 
 * **ClusteringFilter**
   eps = The range of object moving for time difference + Radar Error    
   min_samples = 1  &nbsp;&nbsp;&nbsp;*# ClusteringFilter judges whether it's the same object or not. So, 1 is enough.*    
  
 
 #### 2.3.2 parameter setting
 
 you have to set the parameters depending on what you want to detect.   
  &nbsp;&nbsp;&nbsp;&nbsp; *# You can set these values by adjusting the PARAMETER in RadarTrackingVer2.py* 
 
 <img src="https://user-images.githubusercontent.com/97038348/167069487-bbfe54b3-4f10-401d-9804-f9c4521d514e.png" width="30%" height="30%"/>

 * **objectSize** = eps for Clustering
 * **objectMovingRange** = eps for ClusteringVel
 * **filteringRange** = eps for ClusteringFilter
 * **sizeOfWindow** = number of elements in window
 * **maxNumOfSkip** = the number of maximum skip count of window. If skip count over this number, that window will be deleted.
 * **filterMode** =
   * 0: Simple moving average
   * 1: Weighted moving average
   * 2: Current-Weighted moving average
 * **weight** = weight for Current-Weighted moving average
 
 
 ## 3. Experiments
 
 Radar Tracking was tested on people moving in the Y-axis direction.

 
 <p align="center"><img src="https://user-images.githubusercontent.com/97038348/168016836-3ccce287-880f-4b96-bf47-f4c3e0060de5.png" width="60%" height="100%"/>
 
 ### 3.1 Radar Tracking ver.1
  * X position
  <p align="center"><img src="https://user-images.githubusercontent.com/97038348/168017771-780e61f2-219d-4452-b23f-fff93272590d.png" width="120%" height="120%"/>
   
   * Y position
  <p align="center"><img src="https://user-images.githubusercontent.com/97038348/168017778-81411d06-d8dd-43d8-bad8-70405f5c23f4.png" width="100%" height="60%"/>
   
   * X velocity
  <p align="center"><img src="https://user-images.githubusercontent.com/97038348/168017792-35be4d2c-5097-4f57-9cb6-3485413bcca3.png" width="100%" height="60%"/>
   
   * Y veloctiy
  <p align="center"><img src="https://user-images.githubusercontent.com/97038348/168017806-0f6e1778-7021-430a-a6ab-7e06587b7fd0.png" width="100%" height="60%"/>
  
 ### 3.2 Radar Tracking ver.2
   
 ### 3.3 Experiment Data
   
 
## 4. Running on ROS
 ## 3.1 Set up

install numpy and sklearn!

    $ sudo apt install python3-pip
    $ sudo pip3 install numpy
    $ sudo pip3 install scikit-learn
 
 ## 3.2 Run the Radar Tracking
 

 
launch and get the data from radar

    $ roslaunch ti_mmwave_rospkg 6843_multi_3d_0.launch
    
rosrun the Clustering.py

    $ rosrun ti_mmwave_rospkg Clustering.py
    
RadarTracking.py subscribe following topic

    Subscribed Topic : /ti_mmwave/radar_scan
    
    msg Type : ti_mmwave_rospkg/RadarScan
    
        point_id - The number of scaned points
        x - x-coordinate of the point
        y - y-coordinate of the point
        z - z-coordinate of the point
        header.stamp.secs - second
        header.stamp.nsecs - nano second
        
        
RadarTracking.py publish following topic   
 **/object_tracking** is from Filtering Algorithm   
 **/object_velocity** is from Clustering Algorithm   
    
    Published Topic : /object_tracking, /object_velocity
    
    msg Type : object_msgs/Objects
 
        Objects - [Object 1, Object 2, ... ]
 
    msg Type : Object_msgs/Object
 
        name - object name in Window Set
        position - [X, Y, Z]  
                    X: x-coordinate of the object
                    Y: y-coordinate of the object
                    Z: z-coordinate of the object (= 0)
        velocity - [Vx, Vy, Vz]  
                    Vx: x-velocity of the object
                    Vy: y-velocity of the object
                    Vz: z-velocity of the object (= 0)
 
 
        


    
### Reference
https://github.com/nabihandres/RADAR_Coop_2022/edit/main/README.md
https://dev.ti.com/tirex/explore/node?node=AKeUoo3vxtyjSnRGQDBVLg__VLyFKFf__LATEST
    
 
 
 

 ## 5. Appendices
 ### 5.1 Scikit-learn.DBSCAN (Density-Based Spatial Clustering of Applications with Noise)

<p align="center"><img src="https://user-images.githubusercontent.com/97038348/164612786-42ffa687-5be8-4672-89ee-0071e1899df9.png" width="50%" height="50%"/>

### Parameters
* **eps**: The maximum distance between two samples for one to be considered as in the neighborhood of the other, default=0.5
* **min_samples**: The number of samples in a neighborhood for a point to be considered as a core point, default=5
* **metric**: The metric to use when calculating distance between insatances in a feature array, default='euclidean'


<img src="https://user-images.githubusercontent.com/97038348/164665803-93d061e6-0b21-4176-8621-e106bf3c597f.png" width="30%" height="30%"/>

### Attributes
 * **core_sample_indices_**: Indices of core samples
 * **components_**: Copy of each core sample found by training
 * **labels_**: Cluster labels for each point in the dataset given to fit(). Noisy samples are give the label -1
 

### Examples
    
    from sklearn.cluster import DBSCAN
    import numpy as np
    
    model = DBSCAN(eps=0.5, min_samples=1)
    # vector array
    data = np.array([[1,1],[2,0],[3,0],[1,0]])       
    Clustering = model.fit_predict(data)
    print(Clustering)
    
### Reference
 
https://scikit-learn.org/stable/modules/clustering.html#dbscan   (2.3.7. DBSCAN)   
https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
 
 
 ### 5.2 Moving Average Filter
<p align="center"><img src="https://user-images.githubusercontent.com/97038348/167051870-63823a77-ae42-410d-a3ea-eee8e1ce440b.png" width="40%" height="40%"/>  
 
A moving average is a calculation to analyze data points by creating a seires of averages of window. When new element is obtained, the window is modified by **"shifting forward"** that is, excluding the first number of the window and including the next value in the window. A moving average is commonly used with time series data to smooth out short-term fluctuations and highlight longer-term trends. 
 
 #### Reference
https://en.wikipedia.org/wiki/Moving_average

