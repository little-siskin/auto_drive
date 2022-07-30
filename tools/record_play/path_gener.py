import csv
import os.path as op
import os
import yaml
import math
import io

APOLLO_ROOT = os.path.join(os.path.dirname(__file__), '../../../')
log_dir = os.path.dirname(
        os.path.abspath(__file__)) + "/../../../data/log/"

class PathGener(object):

    def __init__(self,record_file,conf_file):
        self.conf_path = conf_file
        self.garage_path = record_file
        self.points_list = []
        self.path_buffer = []
        self.conf_dict = {}
        self.A = []
        self.B = []
        self.cen = []
        self.head = []
        self.heading = 0
        self.k = 0
        self.b = 0
        self.inc = 0
        self.time_inc = 0
        self.single_line_total = 0
        self.type = 'default'
        self.vec_ab = []
        self.lat_distance = 0.0
        self.inc_k = 0.0

    def parser_yaml_2_dict(self):
        if op.exists(self.conf_path):
            #file = open(self.conf_path, 'r', encoding="utf-8")
            file = io.open(self.conf_path, "r", encoding='utf-8')
            file_data = file.read()
            #data_dict = yaml.load(file_data, yaml.FullLoader)
            data_dict = yaml.load(file_data)
            self.conf_dict = data_dict

            self.single_line_total = data_dict['total']
            self.time_inc = data_dict['time_inc']
            self.inc = data_dict['inc']
            self.type = data_dict['type']
            self.inc_k = data_dict['inc_k']




    def extract_point_of_a_b_head(self):
        self.parser_yaml_2_dict()
        if op.exists(self.garage_path):
            path_file = open(self.garage_path)
            list_lines = csv.reader(path_file)
            list_lines = list(list_lines)
            self.points_list = list_lines
            len_of_lines = len(list_lines)
            self.A = list_lines[1]
            self.B = list_lines[len_of_lines - 1]
            self.head = list_lines[0]
            self.cen = self.B
        else:
            print('file do not exists')

    def work_k_b_out(self):
        self.extract_point_of_a_b_head()
        point_a = self.A
        point_b = self.B
        x_1 = float(point_a[0])
        y_1 = float(point_a[1])
        x_2 = float(point_b[0])
        y_2 = float(point_b[1])
        m_1 = y_2 - y_1
        m_2 = x_2 - x_1
        if m_2 != 0:
            k = m_1 / m_2
            b = (y_1 * m_2 - x_1 * m_1) / m_2
            self.k = k
            self.b = b

        else:
            print('the two points are the same')

    def assign_point(self,x, y, z, speed, acceleration, curvature, curvature_change_rate, time, theta, gear, s, throttle,
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

    def generate_single_path(self,inc_k):
        self.work_k_b_out()
        self.k = self.k + inc_k
        self.get_line_heading()
        rad_angle = self.calcu_angle()
        abs_rad_angle = abs(rad_angle)
        M_PI = math.pi
        #print(self.vec_ab)

        if (M_PI/4<abs_rad_angle and abs_rad_angle<M_PI*3/4):
            if self.vec_ab[1]>0:
                return self.gen_line_by_inc_y(True) #True is positive dirction
            return self.gen_line_by_inc_y(False)
        else:
            if self.vec_ab[0]>0:
                return self.gen_line_by_inc_x(True)
            return self.gen_line_by_inc_x(False)


    def gen_line_by_inc_y(self,p_driection):
        k = self.k
        b = self.b
        total = self.single_line_total
        inc = self.inc
        cen = self.cen
        time_inc = self.time_inc

        if not p_driection:
            inc = -inc

        cen_x = float(cen[0])
        cen_y = float(cen[1])
        cen_z = float(cen[2])
        speed = float(cen[3])
        # acceleration = float(cen[4])
        acceleration = 0.0
        # curvature = float(cen[5])
        curvature = 0.0
        # curvature_change_rate = float(cen[6])
        curvature_change_rate = 0.0
        time = float(cen[7])
        # theta = float(cen[8])
        theta = self.heading
        gear = int(cen[9])
        s = float(cen[10])
        throttle = float(cen[11])
        brake = float(cen[12])
        steering = float(cen[13])
        """
        forward if inc > 0
        """
        points_list = []
        point_list = []
        y_start = cen_y - int((total / 2)) * inc
        x_start = self.get_x(y_start, k, b)
        time = time - total * time_inc*2
        x_temp = x_start
        y_temp = y_start

        point_list = self.assign_point(x_start, y_start, cen_z, speed, acceleration, curvature, curvature_change_rate,
                                       time, theta, gear, s, throttle, brake, steering)

        for i in range(1, total + 1, 1):
            points_list.append(point_list)
            y_temp = y_temp + inc
            x_temp = self.get_x(y_temp, k, b)
            time = time + time_inc
            point_list = self.assign_point(x_temp, y_temp, cen_z, speed, acceleration, curvature, curvature_change_rate,
                                           time, theta, gear, s, throttle, brake, steering)

        return points_list

    def gen_line_by_inc_x(self,p_driection):
        k = self.k
        b = self.b
        total = self.single_line_total
        inc = self.inc
        cen = self.cen
        time_inc = self.time_inc

        if not p_driection:
            inc = -inc

        cen_x = float(cen[0])
        cen_y = float(cen[1])
        cen_z = float(cen[2])
        speed = float(cen[3])
        # acceleration = float(cen[4])
        acceleration = 0.0
        # curvature = float(cen[5])
        curvature = 0.0
        # curvature_change_rate = float(cen[6])
        curvature_change_rate = 0.0
        time = float(cen[7])
        # theta = float(cen[8])
        theta = self.heading
        gear = int(cen[9])
        s = float(cen[10])
        throttle = float(cen[11])
        brake = float(cen[12])
        steering = float(cen[13])
        """
        forward if inc > 0
        """
        points_list = []
        point_list = []
        x_start = cen_x - int((total / 2)) * inc
        y_start = self.get_y(x_start, k, b)
        #time = time - int(total / 2) * time_inc
        time = time - total * time_inc*2
        x_temp = x_start
        y_temp = y_start

        point_list = self.assign_point(x_start, y_start, cen_z, speed, acceleration, curvature, curvature_change_rate,time, theta, gear, s, throttle, brake, steering)

        for i in range(1, total + 1, 1):
            points_list.append(point_list)
            x_temp = x_temp + inc
            y_temp = self.get_y(x_temp, k, b)
            time = time + time_inc
            point_list = self.assign_point(x_temp, y_temp, cen_z, speed, acceleration, curvature, curvature_change_rate,time, theta, gear, s, throttle, brake, steering)

        return points_list

    def get_y(self , x, k, b):
        return k * x + b

    def get_x(self, y, k, b):
        return (y - b) / k

    def calcu_angle(self):
        a_x = float(self.A[0])
        a_y = float(self.A[1])
        b_x = float(self.B[0])
        b_y = float(self.B[1])

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


    def get_line_heading(self):
        a = self.A
        b = self.B
        point_a = [float(a[0]),float(a[1])]
        point_b = [float(b[0]),float(b[1])]
        #average
        heading = 0
        #for i in range(1,len(self.points_list)):
        #    heading = heading + float(self.points_list[i][8])

        #self.heading = heading/(len(self.points_list)-1)
	self.heading = float(self.points_list[len(self.points_list)-1][8])



    def generate_path_file(self,dst_path):
        rows_0 = self.generate_single_path(0*self.inc_k)
        #rows_1 = self.generate_single_path(1*self.inc_k)
        #rows_2 = self.generate_single_path(2*self.inc_k)
        rows = []
        rows.extend(rows_0)
        #rows.extend(rows_1)
        #rows.extend(rows_2)
        k = self.k
        b = self.b
        total = self.single_line_total
        inc = self.inc
        cen = self.cen
        head = self.head
        time_inc = self.time_inc

        path = dst_path
        if op.exists(path):
            print('exists')
            os.remove(path)
        #file = open(path, 'w', newline='')  #python3
        file = open(path, 'wb') #python2
        path_file = csv.writer(file)
        path_file.writerow(head)
        path_file.writerows(rows)


def main():
    garage_path = log_dir + 'garage.csv'
    dst_path = op.dirname(os.path.abspath(__file__)) + '/log/garage_ab.csv'
    conf_path = op.dirname(os.path.abspath(__file__)) + '/conf/conf.yaml'
    AB_Gener = PathGener(garage_path,conf_path)
    AB_Gener.generate_path_file(dst_path)



if __name__ == '__main__':
    main()
