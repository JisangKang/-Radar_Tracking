#!/usr/bin/env python
from operator import ge
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3
from ti_mmwave_rospkg.msg import RadarScan
from object_msgs.msg import Objects
from object_msgs.msg import Object
from sklearn.cluster import DBSCAN
import numpy as np

############################################################################################################################################
##################PARAMETERS################################################################################################################
objectSize=0.4           
filteringRange=0.3
sizeOfWindow=15
maxNumOfSkip=20
filterMode=2
weight=5
############################################################################################################################################
############################################################################################################################################

model = DBSCAN(eps=objectSize, min_samples=2)
modelFilter = DBSCAN(eps=filteringRange, min_samples=2)

getPoint=list()
getTime=list()
WindowList=[0 for i in range(sizeOfWindow*2)]
cntWindowSkip=[0]
velWindowList=[0 for i in range(2*5)]
filteredPointVel=list()
filtering=list()

empty=np.empty(shape=[0,2])


def clustering(getPointArr):
    centerPointArr=empty

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
        centerPointArr=np.append(centerPointArr,[[avgX,avgY]],axis=0)
          
    return centerPointArr


def movingAverageFilter(pointArr,WindowArr,VelWindowArr,Time,numWindow):
    
    skipWindowList=[i for i in range(0,numWindow)]
    updateWindowList=[]

    if len(pointArr) != 0:
        
        #Window Set
        for point in pointArr:
            if WindowArr[0][0][0] == 0: #initial
                WindowArr[0][0]=point
                
            else :
                for window in range(0,numWindow):

                    #making a 2x2 matrix for clustering 
                    clusterData = np.empty(shape=[2,2])
                    clusterData[0][0] = WindowArr[window][0][0]
                    clusterData[0][1] = WindowArr[window][0][1]
                    clusterData[1][0] = point[0]
                    clusterData[1][1] = point[1]

                    ClusteringFilter = modelFilter.fit_predict(clusterData)

                    if ClusteringFilter[0] == 0: #Object belong to this window

                        #push the point element to window
                        for push in range(sizeOfWindow-1,0,-1):
                            WindowArr[window][push]=WindowArr[window][push-1]
                        WindowArr[window][0]=point
                        cntWindowSkip[window] = 0
                        updateWindowList.append(window)
                        
                        break

                    elif ClusteringFilter[0] == -1:
                        
                        #there are no window related with point element
                        if window == numWindow-1:
                            WindowArr=np.append(WindowArr,[np.zeros(shape=[sizeOfWindow,2])],axis=0)
                            WindowArr[len(WindowArr)-1][0]=point
                            cntWindowSkip.append(0)
                            VelWindowArr=np.append(VelWindowArr,[np.zeros(shape=[2,5])],axis=0)
                            
        #Plus the count of windowArray which is not updated
        for skip in list(set(skipWindowList)-set(updateWindowList)):
            cntWindowSkip[skip] += 1

        #Abandon windowArray which exceed num
        while (maxNumOfSkip in cntWindowSkip):
            index=cntWindowSkip.index(maxNumOfSkip)
            del cntWindowSkip[index]
            WindowArr=np.delete(WindowArr,index,axis=0)
            VelWindowArr=np.delete(VelWindowArr,index,axis=0)
        
        #Get the filtered point and velocity when windowArray has full number of elements
        #Velocity Window Set
        for win in range(0,len(WindowArr)):
            if WindowArr[win][sizeOfWindow-1][0] != 0 and cntWindowSkip[win] == 0:
                if filterMode == 0: # Simple
                    filtering=list(WindowArr[win].mean(axis=0))

                elif filterMode == 1: # Weighted
                    Weighted = np.zeros(shape=2)
                    den=0
                    for i in range(0,sizeOfWindow):
                        Weighted += np.multiply(WindowArr[win][i],(sizeOfWindow-i))
                        den += (i+1)
                    Weighted = Weighted/den
                    filtering=list(Weighted)

                elif filterMode == 2: # Current-Weighted
                    filtering=list((WindowArr[win].mean(axis=0)*sizeOfWindow+WindowArr[win][0]*(weight-1))/(sizeOfWindow+weight-1))
                
                #Push 
                VelWindowArr[win][1]=VelWindowArr[win][0]
                #Insert new point (position and velocity)
                VelWindowArr[win][0][0]=filtering[0]
                VelWindowArr[win][0][1]=filtering[1]
                VelWindowArr[win][0][4]=Time
                VelWindowArr[win][0][2]=(VelWindowArr[win][0][0]-VelWindowArr[win][1][0])/(VelWindowArr[win][0][4]-VelWindowArr[win][1][4])
                VelWindowArr[win][0][3]=(VelWindowArr[win][0][1]-VelWindowArr[win][1][1])/(VelWindowArr[win][0][4]-VelWindowArr[win][1][4])
                filteredPointVel.append(list([VelWindowArr[win][0][0],VelWindowArr[win][0][1],VelWindowArr[win][0][2],VelWindowArr[win][0][3]]))

        print(WindowArr)
        print(cntWindowSkip)
        print(VelWindowArr)
        print((VelWindowArr[win][0][4]-VelWindowArr[win][1][4]))
        
    return WindowArr, VelWindowArr
                        


            



def callback(data):

    #Get the points from radar
    getPointId=data.point_id
    getPointX=data.x
    getPointY=data.y
    getPointTime=data.header.stamp.secs+float(data.header.stamp.nsecs)/1000000000

    if getPointId == 0:

        getPointArr=np.array(getPoint)            

        #current Time is average time of points  
        curTimeArr=np.array(getTime)  
        curTime=np.mean(curTimeArr)
        
        numWindow=int(len(WindowList)/(sizeOfWindow*2))
        pastWindowArr=np.empty(shape=[numWindow,sizeOfWindow,2])
        pastVelWindowArr=np.empty(shape=[numWindow,2,5])

        for i in range(0,numWindow):
            for j in range(0,sizeOfWindow):
                for k in range(0,2):
                    pastWindowArr[i][j][k]=WindowList[sizeOfWindow*2*i+2*j+k]

        for i in range(0,numWindow):
            for j in range(0,2):
                for k in range(0,5):
                    pastVelWindowArr[i][j][k]=velWindowList[2*5*i+5*j+k]

        #Clustering the points from radar and Get the center points
        centerPointArr = clustering(getPointArr)

        #Moving Average Filtering(make window set and velocity window set)
        curWindowArr, curVelWindowArr = movingAverageFilter(centerPointArr,pastWindowArr,pastVelWindowArr,curTime,numWindow)

        #Publish the matrix
        if filteredPointVel !=[]:
            pub=rospy.Publisher('object_tracking', Objects, queue_size=1)
            pl=Objects()

            for i in range(0,len(filteredPointVel)):
                
                p = Object()
                p.name = str("Object ")+str(i+1)
                p.position = Vector3(filteredPointVel[i][0],filteredPointVel[i][1],0)
                p.velocity = Vector3(filteredPointVel[i][2],filteredPointVel[i][3],0)
                pl.objects.append(p)
            pub.publish(pl)
        print("==============================")
        
        
        #Initialize the variable
        del getPoint[:]
        del getTime[:]
        del WindowList[:]
        del velWindowList[:]
        del filteredPointVel[:]

        for i in range(0,len(curWindowArr)):
            for j in range(0,sizeOfWindow):
                for k in range(0,2):
                    WindowList.append(curWindowArr[i][j][k])
        for i in range(0,len(curVelWindowArr)):
            for j in range(0,2):
                for k in range(0,5):
                    velWindowList.append(curVelWindowArr[i][j][k])
        


    getPoint.append([getPointX,getPointY])
    getTime.append(getPointTime)

    
    

def listener():

    #Making a clustering node
    rospy.init_node('clustering')

    #Subscribing the /ti_mmwave/radar_scan topic from RADAR
    rospy.Subscriber("/ti_mmwave/radar_scan", RadarScan, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()