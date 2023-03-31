from RobotRaconteur.Client import *
import time
import numpy

def main():
    url='rr+tcp://localhost:59823?service=optitrack_mocap'

    mocap_cli = RRN.ConnectService(url)

    sensor_data_srv = mocap_cli.fiducials_sensor_data.Connect(-1)

    packet_num=0
    packet_num_total=1000
    st = time.time()
    while True:
        data = sensor_data_srv.ReceivePacketWait()
        packet_num+=1
        if packet_num>=packet_num_total:
            break
    print("Frame No.:",data.sensor_data.seqno)
    print("Total markers:",len(data.fiducials.recognized_fiducials))
    for i in range(len(data.fiducials.recognized_fiducials)):
        print("ID:",data.fiducials.recognized_fiducials[i].fiducial_marker)
        print("position:",data.fiducials.recognized_fiducials[i].pose.pose.pose[0]['position'])
        print("orientation:",data.fiducials.recognized_fiducials[i].pose.pose.pose[0]['orientation'])
        print("===========================================")
    et = time.time()
    print("Ave FPS:",packet_num_total/(et-st))

if __name__=='__main__':
    main()