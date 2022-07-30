"""
by Xiuyan Liu. 2020.3.20
"""

import csv
import os.path as op
import os
import yaml
import math
import io
from path_gener import PathGener
#import matplotlib.pyplot as plt
import numpy as np

APOLLO_ROOT = os.path.join(os.path.dirname(__file__), '../../../')
log_dir = os.path.dirname(
        os.path.abspath(__file__)) + "/../../../data/log/"


class PathManager(object):
    def __init__(self,record_file,conf_file,manager_conf_file):
        self.manager_conf_file = manager_conf_file
        self.lines_distance = 0.0
        self.lines_num = 1
        self.parser_yaml_2_dict()

        self.AB_Gener = PathGener(record_file,conf_file)


    def parser_yaml_2_dict(self):
        if op.exists(self.manager_conf_file):
            #file = open(self.conf_path, 'r', encoding="utf-8")
            file = io.open(self.manager_conf_file, "r", encoding='utf-8')
            file_data = file.read()
            #data_dict = yaml.load(file_data, yaml.FullLoader)
            data_dict = yaml.load(file_data)
            self.conf_dict = data_dict

            self.lines_distance = float(data_dict['lines_distance'])
            self.lines_num = int(data_dict['lines_num'])

    def work_symbol_delta_x_out(self,vec_rad_angle,lines_distance):
        return lines_distance*math.sin(vec_rad_angle)
    
    def work_symbol_delta_y_out(self,vec_rad_angle,lines_distance):
        return lines_distance*math.cos(vec_rad_angle)

    def generate_muti_path(self,lines_num):
        M_PI = 3.1415926
        rows = []
        rad_angle = self.AB_Gener.calcu_angle()
        symbol_delta_x = self.work_symbol_delta_x_out(rad_angle,self.lines_distance)
        symbol_delta_y = self.work_symbol_delta_y_out(rad_angle,self.lines_distance)
        x_1,y_1,x_2,y_2 = self.AB_Gener.get_a_b_coordination()

        points_num = self.AB_Gener.get_single_points_num()
        cen_time = self.AB_Gener.get_time_of_cen()
        inc_time = self.AB_Gener.get_inc_of_time()
        cen_time_now = cen_time - inc_time * points_num * lines_num

        if (-(M_PI/2)<rad_angle and rad_angle<(M_PI/2)):
            counter = (lines_num-1)/2
            for i in range(counter,0,-1):
                self.AB_Gener.assign_a_b_cordination(x_1-symbol_delta_x*i,y_1+symbol_delta_y*i
                                                    ,x_2-symbol_delta_x*i,y_2+symbol_delta_y*i)
                self.AB_Gener.assign_time_of_cen(cen_time_now)
                rows.extend(self.AB_Gener.generate_single_path())
                cen_time_now = cen_time_now + inc_time * points_num
            self.AB_Gener.assign_a_b_cordination(x_1,y_1,x_2,y_2)
            #cen_time_now = cen_time_now + inc_time * points_num
            self.AB_Gener.assign_time_of_cen(cen_time_now)
            rows.extend(self.AB_Gener.generate_single_path())
            for i in range(1,counter+1,1):
                self.AB_Gener.assign_a_b_cordination(x_1+symbol_delta_x*i,y_1-symbol_delta_y*i
                                                    ,x_2+symbol_delta_x*i,y_2-symbol_delta_y*i)
                cen_time_now = cen_time_now + inc_time * points_num
                self.AB_Gener.assign_time_of_cen(cen_time_now)
                rows.extend(self.AB_Gener.generate_single_path())
        
        else:
            counter = (lines_num-1)/2
            for i in range(counter,0,-1):
                self.AB_Gener.assign_a_b_cordination(x_1+symbol_delta_x*i,y_1-symbol_delta_y*i
                                                    ,x_2+symbol_delta_x*i,y_2-symbol_delta_y*i)
                self.AB_Gener.assign_time_of_cen(cen_time_now)
                rows.extend(self.AB_Gener.generate_single_path())
                cen_time_now = cen_time_now + inc_time * points_num
            self.AB_Gener.assign_a_b_cordination(x_1,y_1,x_2,y_2)
            #cen_time_now = cen_time_now + inc_time * points_num
            self.AB_Gener.assign_time_of_cen(cen_time_now)
            rows.extend(self.AB_Gener.generate_single_path())
            for i in range(1,counter+1,1):
                self.AB_Gener.assign_a_b_cordination(x_1-symbol_delta_x*i,y_1+symbol_delta_y*i
                                                    ,x_2-symbol_delta_x*i,y_2+symbol_delta_y*i)
                cen_time_now = cen_time_now + inc_time * points_num
                self.AB_Gener.assign_time_of_cen(cen_time_now)
                rows.extend(self.AB_Gener.generate_single_path())
        
        return rows

    def generate_muti_path_file(self,dst_path,lines_num):
        
        # debug code

        x_1,y_1,x_2,y_2 = self.AB_Gener.get_a_b_coordination()
        self.AB_Gener.assign_a_b_cordination(x_1+1000,y_1,x_2,y_2)

        rows_0 = self.generate_muti_path(lines_num)
        plt_rows = []
        for i in range(0,len(rows_0)):
            pair = [rows_0[i][0],rows_0[i][1]]
            plt_rows.append(pair)
        #self.draw_n_plt(plt_rows)

        rows = []
        rows.extend(rows_0)
        path = dst_path
        if op.exists(path):
            print('exists')
            os.remove(path)
        #file = open(path, 'w', newline='')  #python3
        file = open(path, 'wb') #python2
        path_file = csv.writer(file)
        path_file.writerow(self.AB_Gener.get_head_title())
        path_file.writerows(rows)

    def get_lines_num(self):
        return self.lines_num
    """
    def draw_n_plt(self,rows):
        plt.axis("equal")
        paths = rows
        xy = np.array(paths)
        plt.plot(xy[:,0],xy[:,1])
        plt.show()
    """

def main():
    garage_path = log_dir + 'garage.csv'
    #garage_path = op.dirname(os.path.abspath(__file__)) + '/log/garage_ab.csv'
    dst_muti_path = op.dirname(os.path.abspath(__file__)) + '/log/garage_ab.csv'
    conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/path_conf.yaml'
    manager_conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/manager_conf.yaml'
    MutiAB_Manager = PathManager(garage_path,conf_path,manager_conf_path)
    MutiAB_Manager.generate_muti_path_file(dst_muti_path,MutiAB_Manager.get_lines_num())



if __name__ == '__main__':
    main()