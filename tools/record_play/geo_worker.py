from pyproj import Proj,Transformer
from math import sin, cos, radians, degrees, asin, sqrt,floor,atan2
from csv_point import CsvPoint
from geographiclib.geodesic import Geodesic
import math

zone = 52
hemisphere = 'N'
EARTH_RADIUS = 6371
M_PI = math.pi

class GeoWorker(object):

    def __init__(self,point_1,point_2):
        self.point_1 = point_1
        self.point_2 = point_2
        self.point_list_1 = self.point_1.GetPointAsList()
        self.point_list_2 = self.point_2.GetPointAsList()
        self.transformer = Transformer.from_crs("epsg:32652","epsg:4326")

    def SetPoint_1(self,point_list):
        self.point_1 = CsvPoint(point_list)
        self.point_list_1 = point_list
    
    def SetPoint_2(self,point_list):
        self.point_2 = CsvPoint(point_list)
        self.point_list_2 = point_list

    def CalDegreeAzimuth(self):
        self.wg84_lat_1,self.wg84_lon_1= self.transformer.transform(self.point_list_1[0], self.point_list_1[1])
        self.wg84_lat_2,self.wg84_lon_2 = self.transformer.transform(self.point_list_2[0], self.point_list_2[1])
        self.dict = Geodesic.WGS84.Inverse(self.wg84_lat_1,self.wg84_lon_1,self.wg84_lat_2,self.wg84_lon_2,outmask = 512)
        curAngle = 0
        angle = self.dict['azi2']
        #print("angle",angle)
        if angle < 0 :
            curAngle = (angle + 360) % 360
        else:
            curAngle = angle % 360
        self.azimuth = curAngle
        #print("azimuth",self.azimuth)
        """
        print(self.wg84_lon_1,self.wg84_lat_1)
        print(self.wg84_lon_2,self.wg84_lat_2)
        self.azimuth = self.CalcuAzimuth(self.wg84_lon_2,self.wg84_lat_2,self.wg84_lon_1,self.wg84_lat_1)
        print("azimuth",self.azimuth)
        """

        return self.azimuth

    def GetNormRadAzimuth(self):
        azimuth = self.CalDegreeAzimuth()
        angle = (math.pi/2 - radians(azimuth))
        new_angle = math.fmod(angle+M_PI,M_PI*2.0)
        if new_angle < 0:
            new_angle = new_angle + M_PI
        else:
            new_angle = new_angle - M_PI
        #print("rad normal azimuth",new_angle)
        return new_angle

    def GetAngleAzimuth(self):
        return self.azimuth



























































"""
    def HaverSin(self,theta):
        v = sin(theta/2.0)
        return v*v

    def GetDistance(self,lonA,latA,lonB,latB):

        lonA = radians(lonA)
        latA = radians(latA)
        lonB = radians(lonB)
        latB = radians(latB)

        vLon = abs(lonB - lonA)
        vLat = abs(latB - latA)
        h = self.HaverSin(vLat) + cos(latA) * cos(latB)*self.HaverSin(vLon)
        distance = 2 * EARTH_RADIUS * asin(sqrt(h))
        print("distance = {}".format(distance * 1000.0))
        return distance * 1000.0
    
    def CalcuAzimuth(self,lonA,latA,lonB,latB):
        x = latA - latB
        y = lonA - lonB

        angle = -1
        if y == 0 and x > 0:
            angle = 0
        if y == 0 and x < 0:
            angle = 180.0
        if x == 0 and y > 0:
            angle = 90.0
        if x == 0 and y < 0:
            angle = 270.0

        if angle == -1 :
            disLon = self.GetDistance(lonA,latB,lonB,latB)
            disLat = self.GetDistance(lonB,latA,lonB,latB)
            if x > 0 and y > 0:
                angle = degrees(atan2(disLon,disLat))
            if x < 0 and y > 0:
                angle = degrees(atan2(disLat,disLon)) + 90.0
            if x < 0 and y < 0:
                angle = degrees(atan2(disLon,disLat)) + 180.0
            if x > 0 and y < 0:
                angle = degrees(atan2(disLat,disLon)) + 270.0
    
        curAngle = 0
        if angle < 0 :
            curAngle = (angle + 360) % 360
        else:
            curAngle = angle % 360

        return curAngle

    def TransformUtmIntoLonLat(self, x, y, zone, hemisphere):
 
        h_north = False
        h_south = False
        if (hemisphere == 'N'):
            h_north = True
        elif (hemisphere == 'S'):
            h_south = True
        else:
            print("Unknown hemisphere: " + hemisphere)
    
        proj_in = Proj(proj = 'utm', zone = zone, ellps = 'WGS84', south = h_south, north = h_north, errcheck = True)
    
        lon, lat = proj_in(x, y, inverse = True)
    
        lon = floor(lon * 1000000000000) / 1000000000000
        lat = floor(lat * 1000000000000) / 1000000000000
    
        lon = "%.12f" % lon
        lat = "%.12f" % lat
    
        return float(lon), float(lat)

"""