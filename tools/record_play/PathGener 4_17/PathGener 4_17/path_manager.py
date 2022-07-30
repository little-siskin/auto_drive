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
import copy

from csv_point import CsvPoint
from csv_line import CsvLine
from csv_semic import CsvSemic

APOLLO_ROOT = os.path.join(os.path.dirname(__file__), '../../../')
log_dir = os.path.dirname(
        os.path.abspath(__file__)) + "/../../../data/log/"


class PathManager(object):
    def __init__(self,record_file,conf_file,semic_conf_file,manager_conf_file):
        self.manager_conf_file = manager_conf_file
        self.conf_file = conf_file
        self.semic_conf_file = semic_conf_file
        self.lines_distance = 0.0
        self.lines_num = 1
        self.garage_path = record_file
        self.head = []

        self.ParserYaml2Dict()
        self.ExtractPointOfAAndBAndHead()
        self.line = CsvLine(self.point_a,self.point_b,conf_file)

    
    def Convert2MatchList(self,point_list):
        point_list[0] = float(point_list[0])
        point_list[1] = float(point_list[1])
        point_list[2] = float(point_list[2])
        point_list[3] = float(point_list[3])
        point_list[4] = float(point_list[4])
        point_list[5] = float(point_list[5])
        point_list[6] = float(point_list[6])
        point_list[7] = float(point_list[7])
        point_list[8] = float(point_list[8])
        point_list[9] = float(point_list[9])
        point_list[10] = float(point_list[10])
        point_list[11] = float(point_list[11])
        point_list[12] = float(point_list[12])
        point_list[13] = float(point_list[13])
        return point_list
    
    def ExtractPointOfAAndBAndHead(self):
        if op.exists(self.garage_path):
            path_file = open(self.garage_path)
            list_lines = csv.reader(path_file)
            list_lines = list(list_lines)
            self.points_list = list_lines
            len_of_lines = len(list_lines)
            self.point_a = CsvPoint(self.Convert2MatchList(list_lines[1]))
            self.point_b = CsvPoint(self.Convert2MatchList(list_lines[len_of_lines - 1]))
            self.head = list_lines[0]
        else:
            print('file do not exists')
            return

    def ParserYaml2Dict(self):
        if op.exists(self.manager_conf_file):
            #file = open(self.conf_path, 'r', encoding="utf-8")
            file = io.open(self.manager_conf_file, "r", encoding='utf-8')
            file_data = file.read()
            data_dict = yaml.load(file_data, yaml.FullLoader)
            #data_dict = yaml.load(file_data)
            self.conf_dict = data_dict

            self.lines_distance = float(data_dict['lines_distance'])
            self.lines_num = int(data_dict['lines_num'])
            self.type = int(data_dict['type'])
            self.semicircle_type = int(data_dict['semicircle_type'])

    def WorkSymbolDeltaXOut(self,vec_rad_angle,lines_distance):
        return lines_distance*math.sin(vec_rad_angle)
    
    def WorkSymbolDeltaYOut(self,vec_rad_angle,lines_distance):
        return lines_distance*math.cos(vec_rad_angle)
    
    #paths are restored from up to down (by x) or
    # from right to left(by y) as the index of paths increaseing.
    def GenerateMutiPath(self,lines_num):
        M_PI = math.pi
        rows = []
        rad_angle = self.line.CalcuAngle()
        symbol_delta_x = self.WorkSymbolDeltaXOut(rad_angle,self.lines_distance)
        symbol_delta_y = self.WorkSymbolDeltaYOut(rad_angle,self.lines_distance)
        x_1,y_1,x_2,y_2 = self.line.GetAAndBCoordination()

        points_num = self.line.GetSinglePointsNum()
        cen_time = self.line.GetTimeOfCen()
        inc_time = self.line.GetIncOfTime()
        cen_time_now = cen_time - inc_time * points_num * lines_num

        counter = (lines_num-1)/2
        for i in range(counter,0,-1):
            temp_line = CsvLine(self.point_a,self.point_b,self.conf_file)
            temp_line.SetAAndBCordination(x_1-symbol_delta_x*i,y_1+symbol_delta_y*i
                                                ,x_2-symbol_delta_x*i,y_2+symbol_delta_y*i)
            temp_line.SetTimeOfCen(cen_time_now)
            #rows.extend(temp_line.GenerateSinglePath())
            rows.append(temp_line.GenerateSinglePath(is_positive_direction=True))
            cen_time_now = cen_time_now + inc_time * points_num
        self.line.SetAAndBCordination(x_1,y_1,x_2,y_2)
        self.line.SetTimeOfCen(cen_time_now)
        #rows.extend(self.line.GenerateSinglePath())
        rows.append(self.line.GenerateSinglePath(is_positive_direction=True))
        for i in range(1,counter+1,1):
            temp_line = CsvLine(self.point_a,self.point_b,self.conf_file)
            temp_line.SetAAndBCordination(x_1+symbol_delta_x*i,y_1-symbol_delta_y*i
                                                ,x_2+symbol_delta_x*i,y_2-symbol_delta_y*i)
            cen_time_now = cen_time_now + inc_time * points_num
            temp_line.SetTimeOfCen(cen_time_now)
            #rows.extend(temp_line.GenerateSinglePath())
            rows.append(temp_line.GenerateSinglePath(is_positive_direction=True))
        
        return rows

    
    def GenerateMutiPathAlternate(self,paths):
        alter_paths = copy.deepcopy(paths)
        #print(paths[0])
        for i in range(0,len(alter_paths)):
            if i % 2 == 1:
                for j in range(0,len(alter_paths[i])/2):
                    alter_paths[i][j][0],alter_paths[i][len(alter_paths[i])-j-1][0] = \
                                        alter_paths[i][len(alter_paths[i])-j-1][0],alter_paths[i][j][0] 
                    alter_paths[i][j][1],alter_paths[i][len(alter_paths[i])-j-1][1] = \
                                        alter_paths[i][len(alter_paths[i])-j-1][1],alter_paths[i][j][1]
                for j in range(0,len(alter_paths[i])):
                    if alter_paths[i][j][8] >= 0:
                        alter_paths[i][j][8] = alter_paths[i][j][8] - math.pi
                    else:
                        alter_paths[i][j][8] = alter_paths[i][j][8] + math.pi
        return alter_paths
    
    def GenNSemicircle(self,paths_list,is_semic_direction_to_right_or_up):
        total = len(paths_list[0])
        paths = paths_list
        
        semicircles_list = []

        #print(len(paths))
        if is_semic_direction_to_right_or_up == True:
            for i in range(0,len(paths)-1):
                if i % 2 == 1:
                    #angle of central vector is from point_2 to point_1 
                    semic_temp = CsvSemic(CsvPoint(paths[i][0]),CsvPoint(paths[i+1][0]),self.semic_conf_file)
                    semicircles_list.append(semic_temp.GenSemicircle(is_clockwise=True))
                else:
                    semic_temp = CsvSemic(CsvPoint(paths[i][total-1]),CsvPoint(paths[i+1][total-1]),self.semic_conf_file)
                    semicircles_list.append(semic_temp.GenSemicircle(is_clockwise=False))
        else:
            for i in range(0,len(paths)-1):
                if i % 2 == 0:
                    #angle of central vector is from point_2 to point_1 
                    semic_temp = CsvSemic(CsvPoint(paths[i+1][0]),CsvPoint(paths[i][0]),self.semic_conf_file)
                    semicircles_list.append(semic_temp.GenSemicircle(is_clockwise=False))
                else:
                    semic_temp = CsvSemic(CsvPoint(paths[i+1][total-1]),CsvPoint(paths[i][total-1]),self.semic_conf_file)
                    semicircles_list.append(semic_temp.GenSemicircle(is_clockwise=True))

        return semicircles_list
        
    
    def MergeMutiPaths(self,muti_paths):
        merged_list = []
        for i in range(0,len(muti_paths)):
            merged_list.extend(muti_paths[i])
        return self.HandlePathsTime(merged_list)
    
    def MergeMutiSemic(self,muti_semic):
        merged_list = []
        for i in range(0,len(muti_semic)):
            merged_list.extend(muti_semic[i])
        return self.HandlePathsTime(merged_list)

    def MergeMutiPathsAndSemics(self,muti_paths,muti_semic,is_semic_direction_to_right_or_up):
        merged_list = []
        if is_semic_direction_to_right_or_up:
            for i in range(0,len(muti_paths)-1):
                merged_list.extend(muti_paths[i])
                merged_list.extend(muti_semic[i])
            merged_list.extend(muti_paths[len(muti_paths)-1])
        else:
            merged_list.extend(muti_paths[len(muti_paths)-1])
            for i in range(len(muti_paths)-2,-1,-1):
                merged_list.extend(muti_semic[i])
                merged_list.extend(muti_paths[i])
        return self.HandlePathsTime(merged_list)
    
    def HandlePathsTime(self,merged_list):
        time_inc = 0.1
        temp_time = merged_list[0][7]
        temp_time = temp_time - 10000000*time_inc
        for i in range(0,len(merged_list)):
            merged_list[i][7] = temp_time
            temp_time = temp_time + time_inc
        return merged_list
        


    def GenerateMutiPathFile(self,dst_path,lines_num):
        
        # debug code
        #x_1,y_1,x_2,y_2 = self.line.GetAAndBCoordination()
        #x_1 = 320090.007084
        #y_1 = 5051884.67964
        #x_2 = 320090.013683
        #y_2 = 5051884.86174
        #self.line.SetAAndBCordination(x_1,y_1,x_2,y_2)
        #-----
        paths = []
        rows = []
        if self.type == 1:
            paths = self.GenerateMutiPath(lines_num)
            rows = self.MergeMutiPaths(paths)
        elif self.type == 2:
            if self.semicircle_type == 1:
                paths = self.GenerateMutiPath(lines_num)
                paths_alter = self.GenerateMutiPathAlternate(paths)
                rows = self.GenNSemicircle(paths,is_semic_direction_to_right_or_up=True)
                rows = self.MergeMutiPathsAndSemics(paths_alter,rows,is_semic_direction_to_right_or_up=True)
            elif self.semicircle_type == 2:
                paths = self.GenerateMutiPath(lines_num)
                paths_alter = self.GenerateMutiPathAlternate(paths)
                rows = self.GenNSemicircle(paths,is_semic_direction_to_right_or_up=False)
                rows = self.MergeMutiPathsAndSemics(paths_alter,rows,is_semic_direction_to_right_or_up=False)
        rows_0 = rows
        plt_rows = []
        for i in range(0,len(rows_0)):
            pair = [rows_0[i][0],rows_0[i][1]]
            plt_rows.append(pair)
        #self.draw_n_plt(plt_rows)

        rows = []
        rows.extend(rows_0)
        path = dst_path
        if op.exists(path):
            print('Exists csv file, replaced.')
            os.remove(path)
        #file = open(path, 'w', newline='')  #python3
        file = open(path, 'wb') #python2
        path_file = csv.writer(file)
        path_file.writerow(self.head)
        path_file.writerows(rows)

    def get_lines_num(self):
        return self.lines_num

"""    
    def draw_n_plt(self,rows):
        plt.axis("equal")
        paths = rows
        #print(rows)
        xy = np.array(paths)
        plt.plot(xy[:,0],xy[:,1])
        plt.show()
"""

def main():
    #garage_path = log_dir + 'garage.csv'
    garage_path = op.dirname(os.path.abspath(__file__)) + '/log/garage_ab.csv'
    dst_muti_path = op.dirname(os.path.abspath(__file__)) + '/log/garage_muti_ab.csv'
    conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/path_conf.yaml'
    manager_conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/manager_conf.yaml'
    semic_conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/semic_conf.yaml'
    MutiABManager = PathManager(garage_path,conf_path,semic_conf_path,manager_conf_path)
    MutiABManager.GenerateMutiPathFile(dst_muti_path,MutiABManager.get_lines_num())



if __name__ == '__main__':
    main()




















































































































































"""
import csv
import os.path as op
import os
import yaml
import math
import io
from path_gener import PathGener
import matplotlib.pyplot as plt
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
        M_PI = math.pi
        rows = []
        rad_angle = self.AB_Gener.calcu_angle()
        symbol_delta_x = self.work_symbol_delta_x_out(rad_angle,self.lines_distance)
        symbol_delta_y = self.work_symbol_delta_y_out(rad_angle,self.lines_distance)
        #print(symbol_delta_x,symbol_delta_y)
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
        self.AB_Gener.assign_a_b_cordination(x_1,y_1,x_1,y_1-100)

        rows_0 = self.generate_muti_path(lines_num)
        plt_rows = []
        for i in range(0,len(rows_0)):
            pair = [rows_0[i][0],rows_0[i][1]]
            plt_rows.append(pair)
        self.draw_n_plt(plt_rows)

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
    
    def draw_n_plt(self,rows):
        plt.axis("equal")
        paths = rows
        xy = np.array(paths)
        plt.plot(xy[:,0],xy[:,1])
        plt.show()
    

def main():
    #garage_path = log_dir + 'garage.csv'
    garage_path = op.dirname(os.path.abspath(__file__)) + '/log/garage_ab.csv'
    dst_muti_path = op.dirname(os.path.abspath(__file__)) + '/log/garage_muti_ab.csv'
    conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/path_conf.yaml'
    manager_conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/manager_conf.yaml'
    MutiAB_Manager = PathManager(garage_path,conf_path,manager_conf_path)
    MutiAB_Manager.generate_muti_path_file(dst_muti_path,MutiAB_Manager.get_lines_num())



if __name__ == '__main__':
    main()

"""