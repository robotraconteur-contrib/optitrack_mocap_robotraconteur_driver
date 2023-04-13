import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import general_robotics_toolbox as rox
import argparse
import sys
import threading
import numpy as np
import time
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil

from natnet_lib.DataDescriptions import *
from natnet_lib.MoCapData import *
from natnet_lib.NatNetClient import *

class OptitrackDriver(object):
    def __init__(self,serveraddr='127.0.0.1',clientaddr='127.0.0.1',use_multiclass=True) -> None:
        
        ## streaming setup
        optionsDict = {}
        optionsDict["clientAddress"] = clientaddr
        optionsDict["serverAddress"] = serveraddr
        optionsDict["use_multicast"] = use_multiclass

        ## setup NatNetClient obj
        self.streaming_client = NatNetClient()
        self.streaming_client.set_client_address(optionsDict["clientAddress"])
        self.streaming_client.set_server_address(optionsDict["serverAddress"])
        self.streaming_client.set_use_multicast(optionsDict["use_multicast"])

        ## mocap data and RR
        self._fiducials=RRN.GetStructureType('com.robotraconteur.fiducial.RecognizedFiducials')
        self._fiducial=RRN.GetStructureType('com.robotraconteur.fiducial.RecognizedFiducial')
        self._fiducials_sensor_data=RRN.GetStructureType('com.robotraconteur.fiducial.FiducialSensorData')
        self._namedposecovtype = RRN.GetStructureType('com.robotraconteur.geometry.NamedPoseWithCovariance')
        self._namedposetype = RRN.GetStructureType('com.robotraconteur.geometry.NamedPose')
        self._posetype = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Pose')
        self._sensordatatype = RRN.GetStructureType('com.robotraconteur.sensordata.SensorDataHeader')
        self._tstype = RRN.GetPodDType('com.robotraconteur.datetime.TimeSpec2')
        self._sensor_data_util = SensorDataUtil(RRN)
        self.current_fiducials_sensor_data=None

        ## streaming thread setup
        self._lock = threading.RLock()
        self._streaming = False

    def srv_start_driver(self):
        
        ## running streaming thread
        is_running = self.streaming_client.run_mocap(self.send_sensor_data)
        if not is_running:
            print("ERROR: Could not start streaming client.")
            try:
                sys.exit(1)
            except SystemExit:
                print("...")
            finally:
                print("exiting")
        
        time.sleep(1)
        ## check connection
        if self.streaming_client.connected() is False:
            print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
            try:
                sys.exit(2)
            except SystemExit:
                print("...")
            finally:
                print("exiting")
        
        self.print_configuration()

        print("\n")
        print("Optitrack RR Service Ready...")
    
    def send_sensor_data(self,mocap_data):

        # clear previous list
        fiducials = self._fiducials()
        fiducials.recognized_fiducials=[]

        ## get rigid body
        for i in range(len(mocap_data.rigid_body_data.rigid_body_list)):
            rec_fiducials = self._fiducial()
            rigid_body = mocap_data.rigid_body_data.rigid_body_list[i]
            rec_fiducials.fiducial_marker = 'rigid'+str(int(rigid_body.id_num))
            rec_fiducials.pose = self._namedposecovtype()
            rec_fiducials.pose.pose = self._namedposetype()
            rec_fiducials.pose.pose.pose = np.zeros((1,),dtype=self._posetype)
            rec_fiducials.pose.pose.pose[0]['position']['x'] = rigid_body.pos[0]*1000 ## mm
            rec_fiducials.pose.pose.pose[0]['position']['y'] = rigid_body.pos[1]*1000 ## mm
            rec_fiducials.pose.pose.pose[0]['position']['z'] = rigid_body.pos[2]*1000 ## mm
            quat = [rigid_body.rot[3],rigid_body.rot[0],rigid_body.rot[1],rigid_body.rot[2]]
            rec_fiducials.pose.pose.pose[0]['orientation']['w'] = quat[0]
            rec_fiducials.pose.pose.pose[0]['orientation']['x'] = quat[1]
            rec_fiducials.pose.pose.pose[0]['orientation']['y'] = quat[2]
            rec_fiducials.pose.pose.pose[0]['orientation']['z'] = quat[3]
            fiducials.recognized_fiducials.append(rec_fiducials)
        
        ## get markers
        # print(len(mocap_data.labeled_marker_data.labeled_marker_list))
        for i in range(len(mocap_data.labeled_marker_data.labeled_marker_list)):
            rec_fiducials = self._fiducial()
            lbmarker = mocap_data.labeled_marker_data.labeled_marker_list[i]
            model_id,marker_id = lbmarker.get_marker_id()
            rec_fiducials.fiducial_marker = 'marker'+str(int(marker_id))+'_rigid'+str(int(model_id))
            rec_fiducials.pose = self._namedposecovtype()
            rec_fiducials.pose.pose = self._namedposetype()
            rec_fiducials.pose.pose.pose = np.zeros((1,),dtype=self._posetype)
            rec_fiducials.pose.pose.pose[0]['position']['x'] = lbmarker.pos[0]*1000 ## mm
            rec_fiducials.pose.pose.pose[0]['position']['y'] = lbmarker.pos[1]*1000 ## mm
            rec_fiducials.pose.pose.pose[0]['position']['z'] = lbmarker.pos[2]*1000 ## mm
            fiducials.recognized_fiducials.append(rec_fiducials)

        fiducials_sensor_data = self._fiducials_sensor_data()
        fiducials_sensor_data.sensor_data = self._sensordatatype()
        fiducials_sensor_data.sensor_data.seqno = int(mocap_data.prefix_data.frame_number)
        nanosec = mocap_data.suffix_data.stamp_data_received*100
        fiducials_sensor_data.sensor_data.ts = np.zeros((1,),dtype=self._tstype)
        fiducials_sensor_data.sensor_data.ts[0]['nanoseconds'] = int(nanosec%1e9)
        fiducials_sensor_data.sensor_data.ts[0]['seconds'] = int(nanosec/1e9)
        fiducials_sensor_data.fiducials = fiducials

        self.fiducials_sensor_data.AsyncSendPacket(fiducials_sensor_data, lambda: None)
        self.current_fiducials_sensor_data = fiducials_sensor_data

    def srv_stop_streaming(self):

        # self._streaming = False
        # self.data_t.join()
        self.streaming_client.shutdown()
    
    def capture_fiducials(self):

        if self.current_fiducials_sensor_data is not None:
            return self.current_fiducials_sensor_data.fiducials
        else:
            return self._fiducials()
    
    def print_configuration(self):

        print("Connection Configuration:")
        print("  Client:          %s"% self.streaming_client.local_ip_address)
        print("  Server:          %s"% self.streaming_client.server_ip_address)
        print("  Command Port:    %d"% self.streaming_client.command_port)
        print("  Data Port:       %d"% self.streaming_client.data_port)

        if self.streaming_client.use_multicast:
            print("  Using Multicast")
            print("  Multicast Group: %s"% self.streaming_client.multicast_address)
        else:
            print("  Using Unicast")

        #NatNet Server Info
        application_name = self.streaming_client.get_application_name()
        nat_net_requested_version = self.streaming_client.get_nat_net_requested_version()
        nat_net_version_server = self.streaming_client.get_nat_net_version_server()
        server_version = self.streaming_client.get_server_version()

        print("  NatNet Server Info")
        print("    Application Name %s" %(application_name))
        print("    NatNetVersion  %d %d %d %d"% (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))
        print("    ServerVersion  %d %d %d %d"% (server_version[0], server_version[1], server_version[2], server_version[3]))
        print("  NatNet Bitstream Requested")
        print("    NatNetVersion  %d %d %d %d"% (nat_net_requested_version[0], nat_net_requested_version[1],\
        nat_net_requested_version[2], nat_net_requested_version[3]))

def main():
    parser = argparse.ArgumentParser(description="ATI force torque sensor driver service for Robot Raconteur")
    parser.add_argument("--server-ip", type=str, default="127.0.0.1", help="the ip address of the Natnet (i.e. Motiv) server")
    parser.add_argument("--client-ip", type=str, default="127.0.0.1", help="the ip address of the Natnet client")
    parser.add_argument("--use-multicast", type=bool, default=True, help="Multicast v.s. Unicast")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args,_ = parser.parse_known_args()

    # not yet know what this do
    rr_args = ["--robotraconteur-jumbo-message=true"] + sys.argv
    RRC.RegisterStdRobDefServiceTypes(RRN)

    optitrack_obj = OptitrackDriver(args.server_ip,args.client_ip,args.use_multicast)

    with RR.ServerNodeSetup("com.robotraconteur.fiducial.FiducialSensor",59823,argv=rr_args):
        
        service_ctx = RRN.RegisterService("optitrack_mocap","com.robotraconteur.fiducial.FiducialSensor",optitrack_obj)
        optitrack_obj.srv_start_driver()

        if args.wait_signal:  
            #Wait for shutdown signal if running in service mode          
            print("Press Ctrl-C to quit...")
            import signal
            signal.sigwait([signal.SIGTERM,signal.SIGINT])
        
        else:    
            #Wait for the user to shutdown the service
            if (sys.version_info > (3, 0)):
                input("Server started, press enter to quit...")
            else:
                raw_input("Server started, press enter to quit...")

        optitrack_obj.srv_stop_streaming()

if __name__ == "__main__":
    main()