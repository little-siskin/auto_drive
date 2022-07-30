import csv
import os.path as op
import os
import yaml
import math
from math import sin, cos, radians, degrees, asin, sqrt,floor,atan2
import io
from csv_point import CsvPoint
from geo_worker import GeoWorker
import copy

APOLLO_ROOT = os.path.join(os.path.dirname(__file__), '../../../')
log_dir = os.path.dirname(
        os.path.abspath(__file__)) + "/../../../data/log/"

INF = "INF"

class CsvLine(object):
    def __init__(self,point_1,point_2,conf_path):
        self.line = []
        self.k = 0
        self.b = 0
        self.head = []
        self.cen = []
        self.point_1 = point_1
        self.point_2 = point_2
        self.cen_point = self.point_2
        self.conf_path = conf_path
        
        self.ParserYaml2Dict()
        self.WorkKAndBOut()

    def AppendPoint(self,point):
        self.line.append(copy.copy(point.GetPointAsList()))

    def GetLineAsLists(self):
        if self.line != []:
            #print("len of line",len(self.line))
            return self.line
        else:
            print("line lists is empty.")
            return []

    def GetAAndBCoordination(self):
        return self.point_1.GetX(),self.point_1.GetY(),self.point_2.GetX(),self.point_2.GetY()

    def GetSinglePointsNum(self):
        return self.single_points_num
    
    def GetTimeOfCen(self):
        return self.cen_point.GetTime()
    
    def SetTimeOfCen(self,time):
        self.cen_point.SetTime(time)
    
    def GetIncOfTime(self):
        return self.time_inc
    


    def ParserYaml2Dict(self):
        if op.exists(self.conf_path):
            #file = open(self.conf_path, 'r', encoding="utf-8")
            file = io.open(self.conf_path, "r", encoding='utf-8')
            file_data = file.read()
            data_dict = yaml.load(file_data, yaml.FullLoader)
            #data_dict = yaml.load(file_data)
            self.conf_dict = data_dict

            self.single_points_num = data_dict['points_num']
            self.time_inc = data_dict['time_inc']
            self.inc = data_dict['inc']
            self.type = data_dict['type']
            self.zone = data_dict['zone']

    
    def WorkKAndBOut(self):
        
        x_1 = self.point_1.GetPointAsList()[0]
        y_1 = self.point_1.GetPointAsList()[1]
        x_2 = self.point_2.GetPointAsList()[0]
        y_2 = self.point_2.GetPointAsList()[1]
        m_1 = y_2 - y_1
        m_2 = x_2 - x_1
        if m_2 != 0:
            k = m_1 / m_2
            b = (y_1 * m_2 - x_1 * m_1) / m_2
            self.k = k
            self.b = b
        else:
            self.k = INF
            self.b = y_1
            #print('the two x-coordi are the same')

    def SetAAndBCordination(self,x_1,y_1,x_2,y_2):
        self.point_1.SetX(x_1)
        self.point_1.SetY(y_1)
        self.point_2.SetX(x_2)
        self.point_2.SetY(y_2)
        self.cen_point.SetX(x_2)
        self.cen_point.SetY(y_2)


    def ConstrucPointList(self,x, y, z, speed, acceleration, curvature, curvature_change_rate, time, theta, gear, s, throttle,
                     brake, steering):
        point_list = []
        point_list.append(x)
        point_list.append(y)
        point_list.append(z)
        point_list.append(speed)
        point_list.append(acceleration)
        point_list.append(curvature)
        point_list.append(curvature_change_rate)
        point_list.append(time)
        point_list.append(theta)
        point_list.append(gear)
        point_list.append(s)
        point_list.append(throttle)
        point_list.append(brake)
        point_list.append(steering)
        return point_list

    def GetCorrespondY(self , x, k, b):
        #print(k,x,b)
        return k * x + b

    def GetCorrespondX(self, y, k, b):
        if k == INF:
            return(float(self.point_1.GetX()))
        return (y - b) / k

    
    def CalcuAngle(self):

        a_x = self.point_1.GetPointAsList()[0]
        a_y = self.point_1.GetPointAsList()[1]
        b_x = self.point_2.GetPointAsList()[0]
        b_y = self.point_2.GetPointAsList()[1]

        vec_ab = [b_x - a_x,b_y-a_y]
        self.vec_ab = vec_ab
        vec_unit = [100.0, 0]

        m_1 = vec_ab[0]*vec_unit[0] + vec_ab[1]*vec_unit[1]
        m_2 = ((vec_ab[0]**2 + vec_ab[1]**2)**(0.5))*((vec_unit[0]**2 + vec_unit[1]**2)**(0.5))
        
        cos = m_1/m_2

        angle = math.acos(cos)

        if vec_ab[1] < 0:
            angle = - angle
        #print(angle)
        #print(angle*180/math.pi)
        return angle

    def GenLineByIncY(self,p_driection):
        k = self.k
        b = self.b
        total = self.single_points_num
        inc = self.inc
        time_inc = self.time_inc

        if not p_driection:
            inc = -inc
        
        #print("by y")
        y_start = self.cen_point.GetY() - int((total / 2)) * inc
        x_start = self.GetCorrespondX(y_start, k, b)
        time = self.cen_point.GetTime() - total * time_inc
        x_temp = x_start
        y_temp = y_start

        temp_point = CsvPoint(self.cen_point.GetPointAsList())
        temp_point.SetY(y_temp)
        temp_point.SetX(x_temp)
        temp_point.SetTime(time)
    
        for i in range(1, total + 1, 1):
            self.AppendPoint(temp_point)
            y_temp = y_temp + inc
            x_temp = self.GetCorrespondX(y_temp, k, b)
            time = time + time_inc
            temp_point = CsvPoint(self.cen_point.GetPointAsList())
            temp_point.SetY(y_temp)
            temp_point.SetX(x_temp)
            temp_point.SetTime(time)


        return self.GetLineAsLists()

    def GenLineByIncX(self,p_driection):
        k = self.k
        b = self.b
        total = self.single_points_num
        inc = self.inc
        time_inc = self.time_inc

        if not p_driection:
            inc = -inc
        
        #print("by x")
        x_start = self.cen_point.GetX() - int((total / 2)) * inc
        y_start = self.GetCorrespondY(x_start, k, b)
        time = self.cen_point.GetTime() - total * time_inc
        #print("total",total)
        x_temp = x_start
        y_temp = y_start

        #temp_point = self.cen_point
        temp_point = CsvPoint(self.cen_point.GetPointAsList())
        temp_point.SetY(y_temp)
        temp_point.SetX(x_temp)
        temp_point.SetTime(time)

        for i in range(1, total + 1, 1):
            self.AppendPoint(temp_point)
            x_temp = x_temp + inc
            y_temp = self.GetCorrespondY(x_temp, k, b)
            time = time + time_inc
            temp_point = CsvPoint(self.cen_point.GetPointAsList())
            temp_point.SetY(y_temp)
            temp_point.SetX(x_temp)
            temp_point.SetTime(time)


        return self.GetLineAsLists()

    def GenerateSinglePath(self,is_positive_direction):
        self.WorkKAndBOut()
        rad_angle = self.CalcuAngle()
        abs_rad_angle = abs(rad_angle)
        #print(rad_angle)
        M_PI = math.pi
        #print(self.vec_ab)

        line_list = []
        if (M_PI/4<abs_rad_angle and abs_rad_angle<M_PI*3/4):
            if self.vec_ab[1]>0:
                #line_list = self.GenLineByIncY(True) #True is positive direction
                line_list = self.GenLineByIncY(is_positive_direction) #True is positive direction
            else:
                #line_list = self.GenLineByIncY(False)
                line_list = self.GenLineByIncY(not is_positive_direction)
        else:
            if self.vec_ab[0]>0:
                #line_list = self.GenLineByIncX(True)
                line_list = self.GenLineByIncX(is_positive_direction)
            else:
                #line_list = self.GenLineByIncX(False)
                line_list = self.GenLineByIncX(not is_positive_direction)

        if self.type == 1:
            pass
        elif self.type == 2:
            #temp_point_1 = CsvPoint(line_list[0])
            temp_point_1 = CsvPoint(self.point_1.GetPointAsList())
            #temp_point_2 = CsvPoint(line_list[len(line_list)-1])
            temp_point_2 = CsvPoint(self.point_2.GetPointAsList())
            worker = GeoWorker(temp_point_1,temp_point_2)
            norm_rad_azimuth = worker.GetNormRadAzimuth()
            for i in range(0,len(line_list)):
                line_list[i][8] = norm_rad_azimuth

        return line_list