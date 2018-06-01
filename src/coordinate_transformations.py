import math
import pandas as pd
import csv
import numpy as np

#Data from camera.config file provided
cam_lat= 45.90414414
cam_lon = 11.02845385
cam_alt = 227.5819
qs = 0.362114
qx = 0.374050
qy = 0.592222
qz = 0.615007

# Equatorial Radius
a = 6378137.0
#Polar Radius
b = 6356752.3142

#fuction to convert from LLA to ECEF
def LLA_ECEF(lat, lon, alt):

    #Converting to radians
    lat =lat* math.pi/180
    lon =lon* math.pi/180

    #ellipsoidal flattening
    f = (a - b)/a

    #Eccentricity
    e = math.sqrt(f*(2-f))
    N = a / math.sqrt(1 - e**2 * (math.sin(lat))**2)

    #The ECEF coordinates X,Y and Z
    x = (alt + N) * math.cos(lon) * math.cos(lat)
    y = (alt + N) * math.cos(lon) * math.sin(lat)
    z = (alt + (1-e**2)*N) * math.sin(lon)

    return x, y, z

#fuction to convert from ECEF to ENU
def ECEF_ENU(x, y, z, lat, lon, cam_x, cam_y, cam_z):

    lat *= math.pi/180
    lon *= math.pi/180

    p1 = [[-math.sin(lon), math.cos(lon), 0],
           [-math.cos(lon)*math.sin(lat), -math.sin(lat)*math.sin(lon), math.cos(lat)],
           [math.cos(lat)*math.cos(lon), math.cos(lat)*math.sin(lon), math.sin(lat)]]

    p = np.matmul(p1, [x-cam_x, y-cam_y, z-cam_z])

    return p[0], p[1], p[2]


def ENU_CAM(x, y, z, qs, qx, qy, qz):

    rq = [[qs ** 2 + qx ** 2 - qy ** 2 - qz ** 2, 2 * qx * qy - 2 * qs * qz, 2 * qx * qz + 2 * qs * qy],
           [2 * qx * qy + 2 * qs * qz, qs ** 2 - qx ** 2 + qy ** 2 - qz ** 2, 2 * qy * qz - 2 * qs * qx],
           [2 * qx * qz - 2 * qs * qy, 2 * qy * qz + 2 * qs * qx, qs ** 2 - qx ** 2 - qy ** 2 + qz ** 2]]

    # p_ENU = np.matmul(p, [[1,0,0], [0,1,0], [0,0,-1]])
    p = np.matmul(rq, [x, y, -z])

    return p[0], p[1], p[2]


def main():

    #Camera coordincates in ECEF
    cam_x, cam_y, cam_z = LLA_ECEF(cam_lat, cam_lon, cam_alt)

    df = pd.read_table("../final_project_data/final_project_point_cloud.fuse", header=None, sep=" ")
    df = df.values
    with open("../final_project_point_cloud.csv", "w") as output:
        writer = csv.writer(output, lineterminator='\n')
        for i in range(0, df.shape[0]):
            lat = df[i][0]
            lon = df[i][1]
            alt = df[i][2]
            x_ECEF, y_ECEF, z_ECEF = LLA_ECEF(lat, lon, alt)
            x_ENU, y_ENU, z_ENU = ECEF_ENU(x_ECEF, y_ECEF, z_ECEF, lat, lon, cam_x, cam_y, cam_z)
            x_CAM, y_CAM, z_CAM = ENU_CAM(x_ENU, y_ENU, z_ENU, qs, qx, qy, qz)

            writer.writerow([x_CAM, y_CAM, z_CAM, df[i][3]])



if __name__ == '__main__':
    main()
