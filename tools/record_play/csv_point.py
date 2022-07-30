import math

class CsvPoint(object):
    
    def __init__(self,point_list = []):
        self.point = point_list
    
    def GetPointAsList(self):
        if self.point != []:
            return self.point
        else:
            print("point list is empty.")
            return []
    
    def SetX(self,x):
        self.point[0] = x
    
    def GetX(self):
        return self.point[0]

    def SetY(self,y):
        self.point[1] = y
    
    def GetY(self):
        return self.point[1]
    
    def SetTime(self,time):
        self.point[7] = time
    
    def GetTime(self):
        return self.point[7]
    
    def SetTheta(self,theta):
        self.point[8] = theta
    
    def GetTheta(self):
        return self.point[8]

     