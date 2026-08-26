# =====================================================================================================================
#
#        MULTIDRONE H2020 PROJECT
#        
#        Sensor Data Publisher (SDAP) - Emulator for Drone Sensor Data
#
#        Developed by Pantelis I. Kaplanoglou 
#        Last Revision Date: 12/07/2018
#
# =====================================================================================================================
import os
import csv
import rospy
import numpy as np

from std_msgs.msg import Header
from multidrone_msgs.msg import GimbalStatus, CameraStatus

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point


# ==============================================================================================
class DroneDataItem(object):
    # ------------------------------------------------------------------------------------
    def __init__(self):
        self.ID = None
        self.ImageName = None
        self.ImageFileName = None
        self.Timestamp = None
        self.SensorWidth = None
        self.SensorHeight = None
        self.FocalLength = None
        self.FOV = None
        self.DroneX = None
        self.DroneY = None
        self.DroneZ = None
        self.CameraRoll = None
        self.CameraPitch = None
        self.CameraYaw = None
        self.RunningTime = None
        # ------------------------------------------------------------------------------------


# ==============================================================================================


# ==============================================================================================
class DroneSensorDataPublisher(object):
    MAX_SUPPORTED_DRONES = 3

    # ------------------------------------------------------------------------------------
    # def __init__(self, p_sEmulationType, p_oConfiguration):
    def __init__(self, p_sEmulationType):
        # ........................... |  Instance Attributes | ...........................
        self.EmulationType = p_sEmulationType

        if self.EmulationType == "airsim":
            # assert "airsim_demo_data" in p_oConfiguration
            self.DataFolder = rospy.get_param("airsim_demo_data")
            # self.DataFolder     = p_oConfiguration["airsim_demo_data"]
            self.ImageFolder = os.path.join(self.DataFolder, "RGB")
            self.DataFileName = os.path.join(self.DataFolder, "drone_data.txt")
        elif (self.EmulationType == "dji") or (self.EmulationType == "dji_live"):
            # assert "dji_demo_data" in p_oConfiguration
            self.DataFolder = rospy.get_param("dji_demo_data")
            # self.DataFolder     = p_oConfiguration["dji_demo_data"]
            self.ImageFolder = os.path.join(self.DataFolder, "frames")
            self.DataFileName = os.path.join(self.DataFolder, "dji_log.txt")

        self.MetadataFolder = os.path.join(self.DataFolder, "metadata")
        self.HeatmapFolder = os.path.join(self.DataFolder, "heatmaps")
        self.VisualizationFolder = os.path.join(self.DataFolder, "visualize")
        self.SourceFolder = os.path.join(self.DataFolder, "source")
        self.Data = []
        self.GimbalPublishers = [None] * (DroneSensorDataPublisher.MAX_SUPPORTED_DRONES + 1)
        self.CameraPublishers = [None] * (DroneSensorDataPublisher.MAX_SUPPORTED_DRONES + 1)
        self.TelemetryPublishers = [None] * (DroneSensorDataPublisher.MAX_SUPPORTED_DRONES + 1)
        # ................................................................................
        if not os.path.exists(self.MetadataFolder):
            os.makedirs(self.MetadataFolder)
        if not os.path.exists(self.HeatmapFolder):
            os.makedirs(self.HeatmapFolder)
        if not os.path.exists(self.VisualizationFolder):
            os.makedirs(self.VisualizationFolder)
        if not os.path.exists(self.SourceFolder):
            os.makedirs(self.SourceFolder)

        # TODO: PANTELIS: [2018-07-12]: This emulator could support emulation of sensor data during a multiple drone flight
        # for nDroneID in range(1, DroneSensorDataPublisher.MAX_SUPPORTED_DRONES + 1):
        #     self.GimbalPublishers[nDroneID]     = rospy.Publisher("drone_%d/gimbal/status" % nDroneID, GimbalStatus, queue_size=1)
        #     self.CameraPublishers[nDroneID]     = rospy.Publisher("drone_%d/camera/status"  % nDroneID, CameraStatus, queue_size=1)
        #     self.TelemetryPublishers[nDroneID]  = rospy.Publisher("drone_%d/ual/pose" % nDroneID    , PoseStamped, queue_size=1)

    # ----------------------------------------------------------------------------
    def LoadData(self):
        if self.EmulationType == "airsim":
            self.LoadDataAirSim()
        elif self.EmulationType == "dji":
            self.LoadDataDJI()

    # ----------------------------------------------------------------------------
    def LoadDataDJI(self):
        if os.path.isfile(self.DataFileName):
            with open(self.DataFileName, "rb") as oDataFile:
                oReader = csv.reader(oDataFile, delimiter=";", quotechar="|")
                for nRowIndex, oRow in enumerate(oReader):

                    if (nRowIndex > 0) and ((nRowIndex % 2) == 0) and (np.floor(nRowIndex // 2) + 1 <= 1922):
                        if len(oRow) > 2:
                            oNewItem = DroneDataItem()
                            self.Data.append(oNewItem)

                            for nColIndex, oColumn in enumerate(oRow):
                                # print(nColIndex, oColumn)
                                if nColIndex == 0:
                                    # 0 Id
                                    oNewItem.ID = int(oColumn)
                                    nCorrespondingFrameNumber = np.floor(oNewItem.ID // 2) + 1
                                    oNewItem.ImageName = "frame_%04d" % nCorrespondingFrameNumber
                                    oNewItem.ImageFileName = os.path.join(self.ImageFolder, oNewItem.ImageName + ".jpg")
                                elif nColIndex == 1:
                                    # 1 Time(seconds)
                                    oNewItem.RunningTime = oColumn
                                    oNewItem.Timestamp = float(oColumn)
                                elif nColIndex == 2:
                                    # 2 sensor_width(mm)
                                    oNewItem.SensorWidth = float(oColumn)
                                elif nColIndex == 3:
                                    # 3 sensor_height(mm)
                                    oNewItem.SensorHeight = float(oColumn)
                                elif nColIndex == 4:
                                    # 4 focal_length(mm)
                                    oNewItem.FocalLength = float(oColumn)
                                elif nColIndex == 5:
                                    # 5 FOV;
                                    oNewItem.FOV = float(oColumn)
                                elif nColIndex == 6:
                                    # 6 Latitude
                                    oNewItem.DroneX = float(oColumn)
                                elif nColIndex == 7:
                                    # 7 Longitude
                                    oNewItem.DroneY = float(oColumn)
                                elif nColIndex == 8:
                                    # 8 Altitude(meters)
                                    oNewItem.DroneZ = float(oColumn)
                                elif nColIndex == 9:
                                    # 9 Roll
                                    oNewItem.CameraRoll = float(oColumn)
                                elif nColIndex == 10:
                                    # 10 Pitch
                                    oNewItem.CameraPitch = float(oColumn)
                                elif nColIndex == 11:
                                    # 11 Yaw
                                    oNewItem.CameraYaw = float(oColumn)

        else:
            # Only an image folder is provider this will have a subfolder frames
            self.ImageFolder = os.path.join(self.DataFolder, "frames")

            sImageFiles = sorted(os.listdir(self.ImageFolder))
            for sImageName in sImageFiles:
                if not sImageName.endswith("heatmap.png"):
                    oNewItem = DroneDataItem()
                    self.Data.append(oNewItem)
                    oNewItem.ImageName = sImageName
                    oNewItem.ImageFileName = os.path.join(self.ImageFolder, oNewItem.ImageName)
                    # ----------------------------------------------------------------------------

    def LoadDataAirSim(self):
        if os.path.isfile(self.DataFileName):
            with open(self.DataFileName, "rb") as oDataFile:
                oReader = csv.reader(oDataFile, delimiter=";", quotechar="|")
                for nRowIndex, oRow in enumerate(oReader):

                    if nRowIndex > 0:
                        if len(oRow) > 2:
                            oNewItem = DroneDataItem()
                            self.Data.append(oNewItem)
                            for nColIndex, oColumn in enumerate(oRow):
                                # print(nColIndex, oColumn)
                                if nColIndex == 0:
                                    oNewItem.ID = int(oColumn)
                                elif nColIndex == 1:
                                    oNewItem.ImageName = oColumn
                                    oNewItem.ImageFileName = os.path.join(self.ImageFolder, oNewItem.ImageName + ".png")
                                elif nColIndex == 2:
                                    oNewItem.Timestamp = float(oColumn)
                                elif nColIndex == 3:
                                    oNewItem.SensorWidth = float(oColumn)
                                elif nColIndex == 4:
                                    oNewItem.SensorHeight = float(oColumn)
                                elif nColIndex == 5:
                                    oNewItem.FocalLength = float(oColumn)
                                elif nColIndex == 6:
                                    oNewItem.FOV = float(oColumn)
                                elif nColIndex == 7:
                                    oNewItem.DroneX = float(oColumn)
                                elif nColIndex == 8:
                                    oNewItem.DroneY = float(oColumn)
                                elif nColIndex == 9:
                                    oNewItem.DroneZ = float(oColumn)
                                elif nColIndex == 10:
                                    oNewItem.CameraRoll = float(oColumn)
                                elif nColIndex == 11:
                                    oNewItem.CameraPitch = float(oColumn)
                                elif nColIndex == 12:
                                    oNewItem.CameraYaw = float(oColumn)
        else:
            # Only an image folder is provider. This will not have a subfolder named ./RGB
            self.ImageFolder = self.DataFolder

            sImageFiles = sorted(os.listdir(self.ImageFolder))
            for sImageName in sImageFiles:
                if not sImageName.endswith("heatmap.png"):
                    oNewItem = DroneDataItem()
                    self.Data.append(oNewItem)
                    oNewItem.ImageName = sImageName
                    oNewItem.ImageFileName = os.path.join(self.ImageFolder, oNewItem.ImageName)
                    # ------------------------------------------------------------------------------------

    def Emulate(self, p_oDataItem):
        if p_oDataItem.Timestamp is None:
            nRosTimestamp = rospy.Time.now()
            oHeader = Header(stamp=nRosTimestamp)
        else:
            # nRosTimestamp = rospy.Time.from_seconds(p_oDataItem.Timestamp / 1000.0)
            nRosTimestamp = rospy.Time.now()
            # oHeader = Header(seq = p_oDataItem.ID), stamp=nRosTimestamp)
            oHeader = Header(stamp=nRosTimestamp)

            oGimbalStatus = GimbalStatus(header=oHeader
                                         , pitch=p_oDataItem.CameraPitch
                                         , roll=p_oDataItem.CameraRoll
                                         , yaw=p_oDataItem.CameraYaw)

            sDebugGimbalStatus = "\nGIMBAL STATUS\n" \
                                 + "-" * 40 + "\n" \
                                 + "Timestamp  : %f\n" % p_oDataItem.Timestamp \
                                 + "pitch      : %f\n" % p_oDataItem.CameraPitch \
                                 + "roll       : %f\n" % p_oDataItem.CameraRoll \
                                 + "yaw        : %f\n" % p_oDataItem.CameraYaw

            oCameraStatus = CameraStatus(header=oHeader
                                         # ,focalLength = float(p_oDataItem.FocalLength)
                                         , sensorWidth=p_oDataItem.SensorWidth
                                         , sensorHeight=p_oDataItem.SensorHeight
                                         )  # ,principalPointX = 0.0,principalPointY = 0.0

            sDebugCameraStatus = "\nCAMERA STATUS\n" \
                                 + "-" * 40 + "\n" \
                                 + "focalLength      : %f\n" % p_oDataItem.FocalLength \
                                 + "sensorWidth      : %f\n" % p_oDataItem.SensorWidth \
                                 + "sensorHeight     : %f\n" % p_oDataItem.SensorHeight
            #                      + "principalPointX  : 0.0\n" \
            #                      + "principalPointY  : 0.0\n"

            # oDroneTelemetry = DroneTelemetry( header = oHeader
            #                                  ,geo_position = GeoPoint(  latitude  = p_oDataItem.DroneX
            #                                                            ,longitude = p_oDataItem.DroneY
            #                                                            ,altitude  = p_oDataItem.DroneZ )
            #                                  )

            oDroneTelemetry = PoseStamped(header=oHeader
                                          , pose=Pose(position=Point(x=p_oDataItem.DroneX
                                                                     , y=p_oDataItem.DroneY
                                                                     , z=p_oDataItem.DroneZ)
                                                      ))
            sDebugDroneTelemetry = "\nDRONE TELEMETRY\n" \
                                   + "-" * 40 + "\n" \
                                   + "latitude  (X) : %f\n" % p_oDataItem.DroneX \
                                   + "longitude (Y) : %f\n" % p_oDataItem.DroneY \
                                   + "altitude  (Z) : %f\n" % p_oDataItem.DroneZ

            sDestFileName = os.path.join(self.MetadataFolder, "1_" + p_oDataItem.ImageName + ".metadata.txt")

            with open(sDestFileName, "w") as oTextFile:
                oTextFile.write(sDebugGimbalStatus)
                oTextFile.write(sDebugCameraStatus)
                oTextFile.write(sDebugDroneTelemetry)

                # self.GimbalPublishers[1].publish(oGimbalStatus)
            # self.CameraPublishers[1].publish(oCameraStatus)
            # self.TelemetryPublishers[1].publish(oDroneTelemetry)

            # self.GimbalPublishers[2].publish(oGimbalStatus)
            # self.CameraPublishers[2].publish(oCameraStatus)
            # self.TelemetryPublishers[2].publish(oDroneTelemetry)

            # self.GimbalPublishers[3].publish(oGimbalStatus)
            # self.CameraPublishers[3].publish(oCameraStatus)
            # self.TelemetryPublishers[3].publish(oDroneTelemetry)
        return oHeader
    # ------------------------------------------------------------------------------------

# ==============================================================================================
