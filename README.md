# Radar Tracking


## Scikit-learn.DBSCAN (Density-Based Spatial Clustering of Applications with Noise)


<p align="center"><img src="https://user-images.githubusercontent.com/97038348/164612786-42ffa687-5be8-4672-89ee-0071e1899df9.png" width="40%" height="40%"/>

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

## Clustering Algorithm

### Flowchart of Clustering Algorithm
 
<p align="center"><img src="https://user-images.githubusercontent.com/97038348/166400471-83d88f2a-ff8e-434e-b5ca-2c000356aa05.PNG" width="60%" height="60%"/>
<p align="center"><img src="https://user-images.githubusercontent.com/97038348/166400476-17d1bc53-cd59-4e24-9c0e-f6224e7b76f4.PNG" width="60%" height="60%"/>


There are two Clustering. The function of **'Clustering'** is detecting objects and finding the center of each object. The function of **'ClusteringVel'** is identifying the same objects in current objects and past objects and calculating the velocity.

### Clustering parameters

 * **Clustering**  
   eps = size of object  
   min_samples = 2  &nbsp;&nbsp;&nbsp;&nbsp;*# The minimum number of points from one object. if you increase this number, accuracy would increase.*
 * **ClusteringVel**  
   eps = The range of object moving for time difference + Radar Error    
   min_samples = 1  &nbsp;&nbsp;&nbsp;*# ClusteringVel judges whether it's the same object or not. So, 1 is enough.*
   
   
## Moving Average Filter
<p align="center"><img src="https://user-images.githubusercontent.com/97038348/167051870-63823a77-ae42-410d-a3ea-eee8e1ce440b.png" width="40%" height="40%"/>  
 
A moving average is a calculation to analyze data points by creating a seires of averages of window. When new element is obtained, the window is modified by **"shifting forward"** that is, excluding the first number of the window and including the next value in the window. A moving average is commonly used with time series data to smooth out short-term fluctuations and highlight longer-term trends. 


### Window Set
<img src="https://user-images.githubusercontent.com/97038348/167054433-f13742da-ad87-43cc-bdce-950b1c4ed50c.PNG" width="80%" height="80%"/>

This schematic diagram shows the structure of window set. The number of windows is same with the number of currently detected objects. 
 
### Formula
<img src="https://user-images.githubusercontent.com/97038348/167053294-ee42ca01-9129-4031-84b3-cccb81380603.PNG" width="80%" height="80%"/>

### Flowchart of Moving Average Filter
<p align="center"><img src="https://user-images.githubusercontent.com/97038348/166400479-eb2387c2-22e9-487c-b6cc-180fd53db2a8.PNG" width="60%" height="60%"/>
<p align="center"><img src="https://user-images.githubusercontent.com/97038348/166400483-b2fc2601-7bb3-4e16-aa74-e8a782a9ac6f.PNG" width="60%" height="60%"/>
 
### Reference
https://en.wikipedia.org/wiki/Moving_average
## Running on ROS

install numpy and sklearn!

    $ sudo apt install python3-pip
    $ sudo pip3 install numpy
    $ sudo pip3 install scikit-learn

launch and get the data from radar

    $ roslaunch ti_mmwave_rospkg 6843_multi_3d_0.launch
    
rosrun the Clustering.py

    $ rosrun ti_mmwave_rospkg Clustering.py
    
Clustering.py subscribe following topic

    Subscribed Topic : /ti_mmwave/radar_scan
    
    msg Type : ti_mmwave_rospkg/RadarScan
    
        point_id - The number of scaned points
        x - x-coordinate of the point
        y - y-coordinate of the point
        z - z-coordinate of the point
        header.stamp.secs - second
        header.stamp.nsecs - nano second
        
        
Clustering.py publish following topic
    
    Published Topic : 
    
    msg Type : 


    
### Reference
https://github.com/nabihandres/RADAR_Coop_2022/edit/main/README.md
https://dev.ti.com/tirex/explore/node?node=AKeUoo3vxtyjSnRGQDBVLg__VLyFKFf__LATEST
    
