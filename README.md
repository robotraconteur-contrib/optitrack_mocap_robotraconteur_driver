# optitrack_mocap_robotraconteur_driver
The Robotraconteur driver for Optitrack motion capture systems.

## Requiredment

- Optitrack
- Motic >= 3.0.0
- Python >= 3.0.0 (tested with 3.8.0)
- RobotRaconteur
- RobotRaconteurCompanion
- general_robotics_toolbox

## Optitrack Setup

### Hardware, Calibration and Motiv

Please follow Optitrack tutorial to setup your optitrack system. The RR driver support Optitrack version > 3.

[Optitrack Getting Started](https://docs.optitrack.com/quick-start-guides/quick-start-guide-getting-started)

Note: If you are having trouble calirating the system, please check if your calibration wand is in the right configuration (i.e. all 3 markers in either A or B config).

### Streaming

Switch on `Setting->Streaming->NatNet Enable` before streaming data.

Please refer to Optitrack doc for more detail. [Here](https://docs.optitrack.com/motive/data-streaming)

## Usage 

Start the driver using default configuration:

```
python robotraconteur_optitrack_driver.py
```

Start the driver using different NatNet (i.e. Motiv) Server IP:

```
python robotraconteur_optitrack_driver.py --servier-ip natnet_server_ip_addr
```

Start the driver using different NatNet (i.e. Motiv) Client IP:

```
python robotraconteur_optitrack_driver.py --client-ip natnet_server_ip_addr
```

Start the driver using Unicast:

```
python robotraconteur_optitrack_driver.py --use-multicast False
```

## Example Client

Optitrack data are sent using `pipe`, in type `FiducialSensorData` with object `FiducialSensor`. The marker data includes

1. Rigid Body (defined in Motiv) Position and Orientation.
2. Labeled Marker Position

Example client to access the optitrack data:

```
from RobotRaconteur.Client import *
import time

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
et = time.time()
print("Ave FPS:",packet_num_total/(et-st))
```