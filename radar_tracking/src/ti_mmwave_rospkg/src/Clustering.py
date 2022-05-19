#!/usr/bin/env python
from operator import ge
import rospy
from std_msgs.msg import String
from ti_mmwave_rospkg.msg import RadarScan
from sklearn.cluster import DBSCAN
import numpy as np

model = DBSCAN(eps=0.4, min_samples=2)
modelVel = DBSCAN(eps=0.1, min_samples=1) 
getPoint=list()
pastPoint=list()
getTime=list()
pastTimeList=list()

empty=np.empty(shape=[0,2])


def clustering(getPointArr):
    curPointArr=empty

    #Clustering the points fram radar
    Clustering = model.fit_predict(getPointArr)

    #Calculate the center of core points of each object and Assign the x, y of center to currentPointArr
    for object in range(0,max(Clustering)+1):
        sumX=0
        sumY=0
        avgX=0
        avgY=0
        n=0
        corePointIndex=model.core_sample_indices_
        for index in corePointIndex:
            if Clustering[index]==object:
                sumX+=getPointArr[index][0]
                sumY+=getPointArr[index][1]
                n+=1
        avgX=sumX/n
        avgY=sumY/n
        curPointArr=np.append(curPointArr,[[avgX,avgY]],axis=0)   
    return curPointArr, Clustering


def velocity(curPointArr,pastPointArr,curTimeAvg,pastTimeAvg):

    curPointNum=len(curPointArr)
    pastPointNum=len(pastPointArr)
    uniCurPast=empty                             #unionCurrentPast
    curPointVel=np.array([])                     #currentPointVelocity
    timeDifference=curTimeAvg-pastTimeAvg
    print(uniCurPast)
    #print(timeDifference)
    if curPointArr != [[]] and pastPointArr != [[]]:
        uniCurPast=np.append(uniCurPast,curPointArr,axis=0)
        uniCurPast=np.append(uniCurPast,pastPointArr,axis=0)
        print(uniCurPast)
        ClusteringVel = modelVel.fit_predict(uniCurPast)
        corePointArr=modelVel.components_        #core points for Clusteringvel
        for curPoint in range(0,curPointNum):
            for pastPoint in range(curPointNum,curPointNum+pastPointNum):
                if ClusteringVel[curPoint] == ClusteringVel[pastPoint]:
                    vel=(corePointArr[curPoint]-corePointArr[pastPoint])/timeDifference
                    curPointVel=np.append(curPointVel,corePointArr[curPoint][0])
                    curPointVel=np.append(curPointVel,corePointArr[curPoint][1])
                    curPointVel=np.append(curPointVel,vel[0])
                    curPointVel=np.append(curPointVel,vel[1])
      
        
        for i in range(len(curPointVel)/4):
            print("x",curPointVel[4*i],"y",curPointVel[4*i+1],"x_vel",curPointVel[4*i+2],"y_vel",curPointVel[4*i+3])
        
        
        print(ClusteringVel)


        
        

    

def callback(data):
    
    #Get the points from radar
    getPointId=data.point_id
    getPointX=data.x
    getPointY=data.y
    
    time=data.header.stamp.secs+float(data.header.stamp.nsecs)/1000000000

    if getPointId == 0:
                
        getPointArr=np.array(getPoint)            #raw data for clustering
        pastPointArr=np.array(pastPoint)          #x, y value of past points
        
        curTimeArr=np.array(getTime)  
        curTime=np.mean(curTimeArr)
        
        pastTimeArr=np.array(pastTimeList)
        pastTime = np.mean(pastTimeArr)
        
        
        #Clustering the points from radar and Get the center points
        curPointArr,Clustering = clustering(getPointArr)

        #Get the point and velocity    
        velocity(curPointArr,pastPointArr,curTime,pastTime)

        #Publish the matrix
        pub=rospy.Publisher('aaa', String, queue_size=10)
        pub.publish("hello")
        
        #Initialize the variable
        del getPoint[:]
        del pastPoint[:]
        del getTime[:]
        del pastTimeList[:] 

        #Assign current value to past value      
        for i in curPointArr:
            pastPoint.append([i[0],i[1]])
        for i in curTimeArr:
            pastTimeList.append(i)


        cnt=0
        for point in getPointArr:
            if Clustering[cnt] != -1:

                temp_x=point[0]
                temp_y=point[1]
                print("id:",  cnt, "x:", round(temp_x,4), "y:", round(temp_y,4), "object:", Clustering[cnt])
                cnt+=1
        
        print("==================================================")
       
        
    getPoint.append([getPointX,getPointY])
    getTime.append(time)
    
    
    

    
    





def listener():

   
    rospy.init_node('clustering')

    rospy.Subscriber("/ti_mmwave/radar_scan", RadarScan, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
