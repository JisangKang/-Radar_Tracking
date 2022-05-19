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
objectMovingRange=0.3
filteringRange=0.3
sizeOfWindow=15
maxNumOfSkip=20
filterMode=2
weight=3
############################################################################################################################################
############################################################################################################################################

model = DBSCAN(eps=objectSize, min_samples=2)
modelVel = DBSCAN(eps=objectMovingRange, min_samples=1) 
modelFilter = DBSCAN(eps=filteringRange, min_samples=2)
getPoint=list()
pastPoint=list()
getTime=list()
pastTimeList=list()
filterList=[0 for i in range(sizeOfWindow*4)]
cntWindowSkip=[0]
filteredPointVel=list()

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
    

    #Calculate the velocity when currentPointArray and pastPointArray are not empty
    if curPointNum != 0 and pastPointNum != 0:
        uniCurPast=np.append(uniCurPast,curPointArr,axis=0)
        uniCurPast=np.append(uniCurPast,pastPointArr,axis=0)
        
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
      
        return curPointVel
        
        
def movingAverageFilter(curPointVel,WindowArr,numWindow): #windowArray is (object x filtering num x 4) matrix

    #for deleting the noise or old data
    skipWindowList=[i for i in range(0,numWindow)]
    updateWindowList=[]
    
    #Run filtering when curPointVel occur
    if type(curPointVel) != type(None) and curPointVel != []:

        #Run filtering once for each curPointVel's element
        for pv in range(len(curPointVel)/4):
            
            pointVel=[curPointVel[4*pv],curPointVel[4*pv+1],curPointVel[4*pv+2],curPointVel[4*pv+3]]
            
            if WindowArr[0][0][0] == 0: #initial
                WindowArr[0][0]=pointVel
            else : 
                for window in range(0,numWindow):

                    #making a 2x2 matrix for clustering 
                    clusterData = np.empty(shape=[2,2])
                    clusterData[0][0] = WindowArr[window][0][0]
                    clusterData[0][1] = WindowArr[window][0][1]
                    clusterData[1][0] = pointVel[0]
                    clusterData[1][1] = pointVel[1]

                    ClusteringFilter = modelFilter.fit_predict(clusterData)
                    
                    #find the window related with curPointVel's element
                    if ClusteringFilter[0] == 0:

                        #push the curPointVel's element to window
                        for push in range(sizeOfWindow-1,0,-1):
                            WindowArr[window][push]=WindowArr[window][push-1]
                        WindowArr[window][0]=pointVel
                        cntWindowSkip[window] = 0
                        updateWindowList.append(window)
                        
                        break

                    elif ClusteringFilter[0] == -1:
                        
                        #there are no window related with curPointVel's element
                        if window == numWindow-1:
                            WindowArr=np.append(WindowArr,[np.zeros(shape=[sizeOfWindow,4])],axis=0)
                            WindowArr[len(WindowArr)-1][0]=pointVel
                            cntWindowSkip.append(0)

        #Plus the count of windowArray which is not updated
        for skip in list(set(skipWindowList)-set(updateWindowList)):
            cntWindowSkip[skip] += 1

        #Abandon windowArray which exceed criterian num
        while (maxNumOfSkip in cntWindowSkip):
            index=cntWindowSkip.index(maxNumOfSkip)
            del cntWindowSkip[index]
            WindowArr=np.delete(WindowArr,index,axis=0)

        #Get the filtered point and velocity when windowArray has full number of elements
        for win in range(0,len(WindowArr)):
            if WindowArr[win][sizeOfWindow-1][0] != 0 and cntWindowSkip[win] == 0:

                if filterMode == 0: # Simple
                    filteredPointVel.append(list(WindowArr[win].mean(axis=0)))

                elif filterMode == 1: # Weighted
                    Weighted = np.zeros(shape=4)
                    den=0
                    for i in range(0,sizeOfWindow):
                        Weighted += np.multiply(WindowArr[win][i],(sizeOfWindow-i))
                        den += (i+1)
                    Weighted = Weighted/den
                    filteredPointVel.append(list(Weighted))

                elif filterMode == 2: # Current-Weighted
                    filteredPointVel.append(list((WindowArr[win].mean(axis=0)*sizeOfWindow+WindowArr[win][0]*(weight-1))/(sizeOfWindow+weight-1)))

        print("Window Set:")
        print(WindowArr)
        print("Window Skip:") 
        print(cntWindowSkip)
        print("=====================================================")
    return WindowArr
        
           
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
        
        numWindow=int(len(filterList)/(sizeOfWindow*4))
        
        pastFilterArr=np.empty(shape=[numWindow,sizeOfWindow,4])
        
        for i in range(0,numWindow):
            for j in range(0,sizeOfWindow):
                for k in range(0,4):
                    pastFilterArr[i][j][k]=filterList[sizeOfWindow*4*i+4*j+k]

        #Clustering the points from radar and Get the center points
        curPointArr,Clustering = clustering(getPointArr)

        #Get the point and velocity    
        curPointVel=velocity(curPointArr,pastPointArr,curTime,pastTime)

        #Moving Average Filtering
        curFilterArr=movingAverageFilter(curPointVel,pastFilterArr,numWindow)
        
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
        
        #Initialize the variable
        del getPoint[:]
        del pastPoint[:]
        del getTime[:]
        del pastTimeList[:]
        del filterList[:] 
        del filteredPointVel[:]

        #Assign current value to past value      
        for i in curPointArr:
            pastPoint.append([i[0],i[1]])
        for i in curTimeArr:
            pastTimeList.append(i)
        for i in range(0,len(curFilterArr)):
            for j in range(0,sizeOfWindow):
                for k in range(0,4):
                    filterList.append(curFilterArr[i][j][k])
       
    getPoint.append([getPointX,getPointY])
    getTime.append(time)
    
    
def listener():

    #Making a clustering node
    rospy.init_node('clustering')

    #Subscribing the /ti_mmwave/radar_scan topic from RADAR
    rospy.Subscriber("/ti_mmwave/radar_scan", RadarScan, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()



    

