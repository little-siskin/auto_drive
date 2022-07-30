import math
import yaml
import os.path as op
import numpy as np
import io
from csv_point import CsvPoint
from geo_worker import GeoWorker
import copy

M_PI = math.pi

class CsvSemic(object):
    def __init__(self,point_1,point_2,conf_path):
        self.point_1 = point_1
        self.point_2 = point_2
        self.conf_path = conf_path
        self.semicircle_list = []
        self.semicircles_list = []
        self.point_list_1 = point_1.GetPointAsList()
        self.point_list_2 = point_2.GetPointAsList()
        self.ParserYaml2Dict()
        self.WorkRAndCenAndThetaOut(self.point_1,self.point_2)
    
    def WorkRAndCenAndThetaOut(self,point_1,point_2):
        self.L = ((point_1.GetX() - point_2.GetX())**2 + \
                            (point_1.GetY() - point_2.GetY())**2)**(0.5)
        self.R = self.L/2.0
        self.cen_x = (point_1.GetX() + point_2.GetX())/2.0
        self.cen_y = (point_1.GetY() + point_2.GetY())/2.0
        v_1 = np.array([point_1.GetX()-point_2.GetX(),point_1.GetY()-point_2.GetY()])
        v_2 = np.array([1.0,0.0])
        Lv_1 = np.sqrt(v_1.dot(v_1))
        Lv_2 = np.sqrt(v_2.dot(v_2))
        cos_theta = v_1.dot(v_2)/(Lv_1*Lv_2)
        self.theta = np.arccos(cos_theta)
        #print(v_1)
        if v_1[1] < 0:
            self.theta = - self.theta
        #print(self.theta)

    def ParserYaml2Dict(self):
        if op.exists(self.conf_path):
            file = io.open(self.conf_path, "r", encoding='utf-8')
            file_data = file.read()
            data_dict = yaml.load(file_data, yaml.FullLoader)
            self.conf_dict = data_dict
            
            self.theta_inc = float(data_dict['theta_inc'])

    def GenSemicircle(self,is_clockwise):
        theta_array = np.array([])
        if is_clockwise:
            theta_array = np.arange(self.theta, self.theta+M_PI, self.theta_inc)
        else:
            theta_array = np.arange(self.theta, self.theta-M_PI, -self.theta_inc)
        #print(theta_array)
        x = self.cen_x + self.R * np.cos(theta_array)
        y = self.cen_y + self.R * np.sin(theta_array)
        x = x.tolist()
        #print(x)
        y = y.tolist()
        semicircle_list = []
        #point_temp = CsvPoint(self.point_1.GetPointAsList())
        for i in range(0,len(x)):
            self.point_1.SetX(x[i])
            self.point_1.SetY(y[i])
            point_temp = copy.deepcopy(CsvPoint(self.point_1.GetPointAsList()))
            semicircle_list.append(point_temp.GetPointAsList())
        return self.HandleSemicTheta(semicircle_list)
    
    def HandleSemicTheta(self,semicircle_list):
        length = len(semicircle_list)
        temp_point_1 = CsvPoint(semicircle_list[0])
        temp_point_2 = CsvPoint(semicircle_list[1])
        worker = GeoWorker(temp_point_1,temp_point_2)
        semicircle_list[0][8] = worker.GetNormRadAzimuth()
        for i in range(0,length-1):
            worker.SetPoint_1(semicircle_list[i])
            worker.SetPoint_2(semicircle_list[i+1])
            semicircle_list[i+1][8] = worker.GetNormRadAzimuth()
        return semicircle_list

    def GetSemicAsList(self):
        return self.semicircle_list

    