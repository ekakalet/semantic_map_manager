#!/usr/bin/env python
# =====================================================================================================================
#
#        MULTIDRONE H2020 PROJECT
#        
#        Visual Semantic Analyzer (VSA) - ROS Node for Crowd Detection
#
#        Developed by Pantelis I. Kaplanoglou / Crowd Detection Model by Maria Tzelepi 
#        Last Revision Date: 09/07/2018
#        Modified 11/03/2020
#
# =====================================================================================================================

import os
import rospy
import rospkg
import time

from sensor_msgs.msg import Image
from visualanalysis_msgs.msg import CrowdHeatmap

import numpy as np
import caffe

import cv2
from cv_bridge import CvBridge
from datetime import datetime
from threading import Thread
from drone_data import DroneSensorDataPublisher

import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.animation import FuncAnimation

import operator
from PIL import Image as Im


# -----------------------------------------------------------------------------------------------
def GetROSNodeBaseFolder(p_sNodeName="ground_visual_analysis"):
    sResult = None

    sAllFolders = os.environ["ROS_PACKAGE_PATH"].split(":")

    for sFolder in sAllFolders:
        if sFolder.endswith("src") and (sResult is None):
            sResult = os.path.join(os.path.join(sFolder, p_sNodeName), "scripts")

    return sResult


# -----------------------------------------------------------------------------------------------


# ==============================================================================================
class ThreadContext(object):
    # ------------------------------------------------------------------------------------
    def __init__(self, p_sName):
        # ........................... |  Instance Attributes | ...........................
        self.Name = p_sName
        self.HasStarted = False
        self.HasFinished = False
        self.Continue = False
        self.ThreadHandle = None
        # ................................................................................

    # ----------------------------------------------------------------------------
    def __threadStart(self, p_oArgs):
        self.HasStarted = True
        self.HasFinished = False
        self.ThreadMain(p_oArgs)
        self.HasFinished = False

    # ----------------------------------------------------------------------------
    def ThreadMain(self, p_oArgs):
        pass

    # ----------------------------------------------------------------------------
    def Resume(self):
        self.Continue = True
        if not self.HasStarted:
            self.ThreadHandle = Thread(target=self.__threadStart, args=(0,))
            self.ThreadHandle.start()
        rospy.loginfo("[>] %s thread started!" % self.Name)

    # ----------------------------------------------------------------------------
    def Terminate(self):
        self.Continue = False
    # ----------------------------------------------------------------------------        


# ==============================================================================================


# ==============================================================================================
class LogFileThread(ThreadContext):
    # ----------------------------------------------------------------------------
    def __init__(self, p_sFileName):
        super(LogFileThread, self).__init__("Log File")
        # ........................... |  Instance Attributes | ...........................
        self.FileName = p_sFileName
        self.Queue = ThreadSafeQueue("LogQ")

        self.LogFile = None
        self.IsShowingOnScreen = False
        # ................................................................................
        self.__createLog()

    # --------------------s--------------------------------------------------------
    def __createLog(self):
        assert self.FileName is not None, "Please provide a valid filename parameter to the class constructor"
        self.LogFile = open(self.FileName, 'w')
        self.LogFile.flush()

    # ----------------------------------------------------------------------------
    def Print(self, p_sMessage, p_bIsShowingOnScreen=True):
        self.Queue.PushMessage(p_sMessage)

    # ----------------------------------------------------------------------------
    def ThreadMain(self, p_oArgs):
        nLineCount = 0

        while self.Continue:
            while (not self.Queue.IsEmpty() and nLineCount < 10):
                # Gets the next frame in queue                
                sMessage = self.Queue.PopMessage()

                if self.IsShowingOnScreen:
                    print(sMessage)
                self.LogFile.write(sMessage + "\n")
                nLineCount += 1
                if nLineCount == 10:
                    nLineCount = 0
                    self.LogFile.flush()
    # ----------------------------------------------------------------------------


# ==============================================================================================


# ==============================================================================================
class FileInputThread(ThreadContext):
    # ----------------------------------------------------------------------------
    def __init__(self, p_Parent):
        super(FileInputThread, self).__init__("File Input")
        # ........................... |  Instance Attributes | ...........................
        self.Eof = False
        self.Parent = p_Parent
        self.Queue = ThreadSafeQueue("FileQ")
        self.Model = None

    # ----------------------------------------------------------------------------
    def ThreadMain(self, p_oArgs):
        self.Eof = False
        for nIndex, oDataItem in enumerate(self.Parent.DroneDataEmulator.Data):
            print("=" * 40, nIndex + 1, "=" * 40)
            oHeader = self.Parent.DroneDataEmulator.Emulate(oDataItem)
            print(oDataItem.ImageFileName)
            oImage = cv2.imread(oDataItem.ImageFileName)

            b, g, r = cv2.split(oImage)  # get b,g,r
            oImage = cv2.merge([r, g, b])  # switch it to rgb

            self.Queue.PushMessage(oImage)
            time.sleep(0.01)
            # self.HasFinished  = True
        self.Eof = True
        # ----------------------------------------------------------------------------


# ==============================================================================================


# ==============================================================================================
class ThreadSafeQueue(object):
    # TODO: PANTELIS: [2018-02-23]: Locking mechanism is just an example. Use of proper operating system objects is needed for pure atomic operations.

    # ------------------------------------------------------------------------------------
    def __init__(self, p_sName="Queue", p_nMaximumQueuedItems=2):
        # ........................... |  Instance Attributes | ...........................
        self.Queue = []
        self.IsQueueLocked = False
        self.Name = p_sName
        self.Count = 0
        self.MaximumQueuedItems = p_nMaximumQueuedItems
        # ................................................................................

    # ----------------------------------------------------------------------------
    def PushMessage(self, p_sMessage):
        while self.IsQueueLocked:
            pass

        if len(self.Queue) > self.MaximumQueuedItems:
            return

        self.IsQueueLocked = True
        try:
            self.Queue.append(p_sMessage)
        finally:
            self.IsQueueLocked = False

    # ----------------------------------------------------------------------------
    def PopMessage(self):
        while self.IsQueueLocked:
            pass

        self.IsQueueLocked = True
        try:
            if len(self.Queue) > 0:
                sMessage = self.Queue.pop(0)
            else:
                sMessage = None
            nCount = len(self.Queue)
        finally:
            self.IsQueueLocked = False

        self.Count += 1

        return sMessage

    # ------------------------------------------------------------------------------------
    def GetCount(self):
        return len(self.Queue)

    # ------------------------------------------------------------------------------------
    def IsEmpty(self):
        bResult = (len(self.Queue) == 0)

        return bResult
    # ------------------------------------------------------------------------------------


# ==============================================================================================


# ==============================================================================================
class MultipleInputOutputCoordinator():
    MAX_SUPPORTED_DRONES = 20

    # ------------------------------------------------------------------------------------
    def __init__(self, p_oParent):
        # ........................... |  Instance Attributes | ...........................
        self.Parent = p_oParent
        self.NumberOfDrones = self.Parent.NumberOfDrones
        self.ImageTopic = self.Parent.EMULATION_IMAGE_TOPIC
        self.Publishers = []
        self.Queues = None
        self.IsEmulating = False
        # ................................................................................

    # ------------------------------------------------------------------------------------
    def SetupInputOutputTopics(self):
        assert (self.NumberOfDrones <= 20), "Current implementation supports up to 20 drones"
        rospy.loginfo("[>] Multiplexer initializing")

        if self.IsEmulating:
            sTopicInTemplate = self.ImageTopic
        else:
            sTopicInTemplate = "drone_%d/ground_shooting_camera/"

        for nIndex in np.arange(0, self.NumberOfDrones):
            sTopicIn = sTopicInTemplate % (nIndex + 1)
            sTopicOut = "drone_%d/visual_analysis/crowd_heatmap" % (nIndex + 1)
            sCallbackIn = "OnReceiveDrone%d" % (nIndex + 1)

            rospy.Subscriber(sTopicIn, Image, getattr(self, sCallbackIn), queue_size=self.Parent.ROS_QUEUE_SIZE)

            oPublisher = rospy.Publisher(sTopicOut, CrowdHeatmap, queue_size=1)
            self.Publishers.append(oPublisher)
            rospy.loginfo(" |__ Subscribed to %s. Publishing to %s" % (sTopicIn, sTopicOut))

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone1(self, p_oROSImageMessage):
        self.Queues[1].PushMessage([1, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone2(self, p_oROSImageMessage):
        self.Queues[2].PushMessage([2, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone3(self, p_oROSImageMessage):
        self.Queues[3].PushMessage([3, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone4(self, p_oROSImageMessage):
        self.Queues[4].PushMessage([4, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone5(self, p_oROSImageMessage):
        self.Queues[5].PushMessage([5, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone6(self, p_oROSImageMessage):
        self.Queues[6].PushMessage([6, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone7(self, p_oROSImageMessage):
        self.Queues[7].PushMessage([7, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone8(self, p_oROSImageMessage):
        self.Queues[8].PushMessage([8, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone9(self, p_oROSImageMessage):
        self.Queues[9].PushMessage([9, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone10(self, p_oROSImageMessage):
        self.Queues[10].PushMessage([10, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone11(self, p_oROSImageMessage):
        self.Queues[11].PushMessage([11, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone12(self, p_oROSImageMessage):
        self.Queues[12].PushMessage([12, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone13(self, p_oROSImageMessage):
        self.Queues[13].PushMessage([13, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone14(self, p_oROSImageMessage):
        self.Queues[14].PushMessage([14, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone15(self, p_oROSImageMessage):
        self.Queues[15].PushMessage([15, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone16(self, p_oROSImageMessage):
        self.Queues[16].PushMessage([16, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone17(self, p_oROSImageMessage):
        self.Queues[17].PushMessage([17, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone18(self, p_oROSImageMessage):
        self.Queues[18].PushMessage([18, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone19(self, p_oROSImageMessage):
        self.Queues[19].PushMessage([19, p_oROSImageMessage])

    # ------------------------------------------------------------------------------------
    def OnReceiveDrone20(self, p_oROSImageMessage):
        self.Queues[20].PushMessage([20, p_oROSImageMessage])
    # ------------------------------------------------------------------------------------


# ==============================================================================================


# ==============================================================================================
class DeepNeuralNetworkEngine(ThreadContext):
    # ------------------------------------------------------------------------------------
    def __init__(self, p_oParent, p_sEngineName):
        super(DeepNeuralNetworkEngine, self).__init__(p_sEngineName)
        # ........................... |  Instance Attributes | ...........................
        self.Parent = p_oParent
        self.NumberOfDrones = self.Parent.NumberOfDrones
        assert self.NumberOfDrones <= MultipleInputOutputCoordinator.MAX_SUPPORTED_DRONES

        self.Multiplexer = self.Parent.Multiplexer
        # .... Thread ....
        self.Bridge = None

        if self.Multiplexer is not None:
            self.Multiplexer.Queues = [ThreadSafeQueue(p_nMaximumQueuedItems=2)
                                       for nIndex in range(0, MultipleInputOutputCoordinator.MAX_SUPPORTED_DRONES + 1)]
        # ................................................................................

    # ----------------------------------------------------------------------------
    def ThreadMain(self, p_oArgs):
        assert self.Multiplexer is not None

        if self.Bridge is None:
            self.Bridge = CvBridge()

        nSourceFrameID = 0
        dStart = None
        nCount = 0.0
        nFramesToPrint = 1
        while self.Continue:
            nQueueIndex = 1
            while nQueueIndex <= self.NumberOfDrones:
                oCurrentQueue = self.Multiplexer.Queues[nQueueIndex]

                if not oCurrentQueue.IsEmpty():
                    if dStart is None:
                        dStart = datetime.now()

                    # Gets the next frame in queue
                    nDroneID, oImageMessage = oCurrentQueue.PopMessage()

                    if nDroneID is not None:
                        # Convert ROS image to CV image
                        # oCVImage = self.Bridge.compressed_imgmsg_to_cv2(oImageMessage)
                        oCVImage = self.Bridge.imgmsg_to_cv2(oImageMessage)

                        # Recall through neural network
                        nInputImage, nInputImageRGB, nHeatmap, nHeatmapGrayscale = self.Predict(oCVImage,
                                                                                                p_bIsConverting=True)

                        # Publish to ROS
                        oHeatmapROS = self.Bridge.cv2_to_imgmsg(nHeatmapGrayscale, "mono8")
                        self.Multiplexer.Publishers[nDroneID - 1].publish(
                            CrowdHeatmap(header=oImageMessage.header, heatmap=oHeatmapROS))

                        # Save input image for debugging
                        if self.IsSavingInputImage:
                            nSourceFrameID = nSourceFrameID + 1
                            cv2.imwrite(os.path.join(self.DebugFolder, "frame%04d.png" % nSourceFrameID), nInputImage)

                        # Copy data to visualizer
                        if self.IsShowingHeatmap:
                            self.Parent.VisualsWindow.Display(nDroneID, nInputImageRGB, nHeatmap)

                        if self.IsVisualizingInput:
                            cv2.imshow("Input Image", nInputImage)
                            cv2.waitKey(1)

                        if (CaffeCrowdDetectionModel.DEBUG_LEVEL >= 1) and (nFramesToPrint != 0):
                            nCount += 1.0
                            dElapsed = datetime.now() - dStart

                            if (nCount % nFramesToPrint) == 0:
                                nFPS = (nCount / (dElapsed.seconds + (dElapsed.microseconds / 1000000.0)))
                                print("[%d] DRONE:%d FPS:%.2f" % (oCurrentQueue.GetCount(), nDroneID, nFPS))
                                nFramesToPrint = int(nFPS * 0.25)

                nQueueIndex += 1
                time.sleep(0.01)

    # ----------------------------------------------------------------------------
    def Predict(self, p_oImage, p_bIsConverting=False):
        # Virtual method implemented by descendands
        return None, None, None, None
    # ----------------------------------------------------------------------------


# ==============================================================================================


# ==============================================================================================            
class CaffeCrowdDetectionModel(DeepNeuralNetworkEngine):
    DEBUG_LEVEL = 1

    # ------------------------------------------------------------------------------------
    def __init__(self, p_oParent, p_sModelPath):
        super(CaffeCrowdDetectionModel, self).__init__(p_oParent, "Crowd detector")

        # ........................... |  Instance Attributes | ...........................
        self.ModelPath = p_sModelPath

        sFiles = os.listdir(self.ModelPath)
        for sFile in sFiles:
            if sFile.endswith("prototxt"):
                self.ModelDefinitionFileName = os.path.join(self.ModelPath, sFile)
            elif sFile.endswith("caffemodel"):
                self.ModelWeightsFileName = os.path.join(self.ModelPath, sFile)
            else:
                self.MeanSubFileName = os.path.join(self.ModelPath, sFile)

        self.Net = None
        self.MeanSub = None
        self.IsGPU = False
        self.NetworkInputSize = self.Parent.NN_INPUT_SIZE
        self.NetworkInputHeight = self.Parent.NN_INPUT_HEIGHT
        self.NetworkInputWidth = self.Parent.NN_INPUT_WIDTH
        self.IsSavingInputImage = self.Parent.IS_SAVING_INPUT_IMAGE

        self.IsVisualizingInput = self.Parent.IS_VISUALIZING_INPUT
        self.IsShowingHeatmap = self.Parent.IS_SHOWING_HEATMAP
        self.DebugFolder = self.Parent.EMULATION_DEBUG_FOLDER
        self.OutputScale = self.Parent.NN_OUTPUT_SCALE
        CaffeCrowdDetectionModel.DEBUG_LEVEL = self.Parent.NN_DEBUG_LEVEL
        # ................................................................................

    # ------------------------------------------------------------------------------------
    def Load(self):
        if self.MeanSubFileName.endswith("npy"):
            self.MeanSub = np.load(self.MeanSubFileName)
        else:
            proto_data = open(self.MeanSubFileName, 'rb').read()
            a = caffe.io.caffe_pb2.BlobProto.FromString(proto_data)
            self.MeanSub = caffe.io.blobproto_to_array(a)[0]

        self.Net = caffe.Net(self.ModelDefinitionFileName, self.ModelWeightsFileName, caffe.TEST)

    # ------------------------------------------------------------------------------------
    def PadToSquare(self, p_oImage, p_nNewSize):
        # print(p_oImage.shape[1], p_oImage.shape[0])
        nAspectRatio = float(p_oImage.shape[1]) / float(p_oImage.shape[0])

        # print("AspectRatio", nAspectRatio)
        resizedframe = cv2.resize(p_oImage, (p_nNewSize, int(p_nNewSize / nAspectRatio)))

        nRows = resizedframe.shape[0]
        nCols = resizedframe.shape[1]

        nDiffRows = p_nNewSize - nRows
        nDiffCols = p_nNewSize - nCols

        nPadRows = nDiffRows // 2
        nPadCols = nDiffCols // 2

        nStartCol = 0
        nEndCol = p_nNewSize
        nStartRow = 0
        nEndRow = p_nNewSize

        if nPadCols > 0:
            nStartCol = nPadCols
            nEndCol = p_nNewSize - nPadCols - nDiffCols % 2

        if nPadRows > 0:
            nStartRow = nPadRows
            nEndRow = p_nNewSize - nPadRows - nDiffRows % 2

        # print("Cols", nPadCols, nStartCol, nEndCol)
        # print("Rows", nPadRows, nStartRow, nEndRow)

        inputframe = np.zeros((p_nNewSize, p_nNewSize, 3), np.uint8)
        inputframe[nStartRow:nEndRow, nStartCol:nEndCol, :] = resizedframe[:, :, :]

        # print(resizedframe.shape, inputframe.shape)

        return inputframe, nPadCols, nPadRows

    # ------------------------------------------------------------------------------------
    def __elapsedSeconds(self, p_dNow, p_dThen):
        nElapsed = p_dNow - p_dThen
        nSecs = nElapsed.total_seconds()
        return nSecs
        # ------------------------------------------------------------------------------------

    def __cropToSquare(self, p_oSourceImage):
        nImageWidth = p_oSourceImage.shape[1]
        nImageHeight = p_oSourceImage.shape[0]

        # nDiffWidth  = nImageWidth - self.NetworkInputSize
        # nDiffHeight = nImageHeight - self.NetworkInputSize
        nDiffWidth = nImageWidth - self.NetworkInputWidth
        nDiffHeight = nImageHeight - self.NetworkInputHeight
        nHorzSpace = nDiffWidth // 2
        nVertSpace = nDiffHeight // 2

        if nDiffHeight > 0:
            nCropStartY = nVertSpace
            nCropEndY = -nVertSpace
        else:
            nCropStartY = 0
            nCropEndY = nImageHeight

        nCropStartX = nHorzSpace
        nCropEndX = -nHorzSpace

        oImage = p_oSourceImage[nCropStartY:nCropEndY, nCropStartX:nCropEndX]

        return oImage

    # ------------------------------------------------------------------------------------
    def __fitToSquare(self, p_oSourceImage):
        nPadCols, nPadRows = 0, 0
        nStartCol, nEndCol, nStartRow, nEndRow = None, None, None, None

        if self.OutputScale > 1:
            p_oSourceImage = cv2.resize(p_oSourceImage, (p_oSourceImage.shape[1] // self.OutputScale
                                                         , p_oSourceImage.shape[0] // self.OutputScale),
                                        interpolation=cv2.INTER_CUBIC)

        nImageWidth = p_oSourceImage.shape[1]
        nImageHeight = p_oSourceImage.shape[0]

        nInputWidth = self.Net.blobs['data'].width
        nInputHeight = self.Net.blobs['data'].height

        if (nImageWidth < nInputWidth) or (nImageHeight < nInputHeight):
            nDiffCols = nInputWidth - nImageWidth
            nDiffRows = nInputHeight - nImageHeight

            nPadCols = nDiffCols // 2
            nPadRows = nDiffRows // 2

            nStartCol = 0
            nEndCol = nInputWidth
            if nPadCols > 0:
                nStartCol = nPadCols
                nEndCol = -nPadCols

            nStartRow = 0
            nEndRow = nInputHeight
            if nPadRows > 0:
                nStartRow = nPadRows
                nEndRow = -nPadRows

            oImage = np.zeros((nInputHeight, nInputWidth, 3), np.uint8)
            oImage[nStartRow:nEndRow, nStartCol:nEndCol, :] = p_oSourceImage[:, :, :]
        else:
            oImage = p_oSourceImage

        return oImage, nPadCols, nPadRows
        # ------------------------------------------------------------------------------------

    def __fit_and_cropToSquare(self, p_oSourceImage):
        nPadCols, nPadRows = 0, 0
        nStartCol, nEndCol, nStartRow, nEndRow = None, None, None, None

        if self.OutputScale > 1:
            p_oSourceImage = cv2.resize(p_oSourceImage, (p_oSourceImage.shape[1] // self.OutputScale
                                                         , p_oSourceImage.shape[0] // self.OutputScale),
                                        interpolation=cv2.INTER_CUBIC)

        nImageWidth = p_oSourceImage.shape[1]
        nImageHeight = p_oSourceImage.shape[0]

        nInputWidth = self.Net.blobs['data'].width
        nInputHeight = self.Net.blobs['data'].height

        nDiffWidth = nImageWidth - nInputWidth
        if nDiffWidth > 0:
            nHorzSpace = nDiffWidth // 2
            oImage_temp = p_oSourceImage[:, nHorzSpace:-nHorzSpace]
        else:
            oImage_temp = p_oSourceImage

        nDiffHeight = nImageHeight - nInputHeight
        if nDiffHeight > 0:
            nVertSpace = nDiffHeight // 2
            oImage_temp = oImage_temp[nVertSpace:-nVertSpace, :]

        nImageWidth = oImage_temp.shape[1]
        nImageHeight = oImage_temp.shape[0]

        if (nImageWidth < nInputWidth) or (nImageHeight < nInputHeight):
            nDiffCols = nInputWidth - nImageWidth
            nDiffRows = nInputHeight - nImageHeight

            nPadCols = nDiffCols // 2
            nPadRows = nDiffRows // 2

            nStartCol = 0
            nEndCol = nInputWidth
            if nPadCols > 0:
                nStartCol = nPadCols
                nEndCol = -nPadCols

            nStartRow = 0
            nEndRow = nInputHeight
            if nPadRows > 0:
                nStartRow = nPadRows
                nEndRow = -nPadRows

            oImage = np.zeros((nInputHeight, nInputWidth, 3), np.uint8)
            oImage[nStartRow:nEndRow, nStartCol:nEndCol, :] = oImage_temp[:, :, :]
        else:
            oImage = oImage_temp

        return oImage, nPadCols, nPadRows
        # ------------------------------------------------------------------------------------

    def __sourceFrameConvertion(self, p_oSourceImageFrame):
        nPadCols, nPadRows = None, None

        # if ((self.NetworkInputSize * self.OutputScale, self.NetworkInputSize * self.OutputScale, 3) != p_oSourceImageFrame.shape):
        #   oImage, nPadCols, nPadRows = self.__fit_and_cropToSquare(p_oSourceImageFrame)

        if ((self.NetworkInputHeight * self.OutputScale, self.NetworkInputWidth * self.OutputScale,
             3) != p_oSourceImageFrame.shape):
            oImage, nPadCols, nPadRows = self.__fit_and_cropToSquare(p_oSourceImageFrame)
        else:
            oImage = p_oSourceImageFrame
            nPadCols = 0
            nPadRows = 0

        # bIsCropping = (self.NetworkInputSize * self.OutputScale) < np.max(p_oSourceImageFrame.shape)

        # Crop to square
        # if bIsCropping:
        #   oImage = self.__cropToSquare(p_oSourceImageFrame)
        # else:
        #   # Fit to square
        #   oImage, nPadCols, nPadRows = self.__fitToSquare(p_oSourceImageFrame)

        # Switch channgels BGR to RGB
        b, g, r = cv2.split(oImage)
        oImageRGB = cv2.merge([r, g, b])
        return oImage, oImageRGB, nPadCols, nPadRows
        # ------------------------------------------------------------------------------------

    def Predict(self, p_oImage, p_bIsConverting=False):
        # Thread-safe lazy initialization of the caffe GPU mode
        if self.IsGPU == False:
            caffe.set_mode_gpu()
            self.IsGPU = True

        dStart = datetime.now()
        nInputWidth = self.Net.blobs['data'].width
        nInputHeight = self.Net.blobs['data'].height

        # Converts the frame input dimensions
        if p_bIsConverting:
            oImage, oImageRGB, nPadCols, nPadRows = self.__sourceFrameConvertion(p_oImage)
        else:
            oImageRGB = p_oImage
            oImage = p_oImage
            nPadCols = 0
            nPadRows = 0

        # nTransposedInputImage = oImageRGB.transpose((2,0,1))
        if CaffeCrowdDetectionModel.DEBUG_LEVEL >= 3:
            print("Prepare Input ...", self.__elapsedSeconds(datetime.now(), dStart))

        # Recalls the frame through the CNN
        # self.Net.blobs['data'].data[0,...] = nTransposedInputImage
        # self.Net.forward()

        # out = self.Net.blobs['prob'].data

        im = oImageRGB / 255.
        transformer = caffe.io.Transformer({'data': self.Net.blobs['data'].data.shape})
        transformer.set_mean('data', np.load(self.MeanSubFileName).mean(1).mean(1))
        transformer.set_transpose('data', (2, 0, 1))
        transformer.set_channel_swap('data', (2, 1, 0))
        transformer.set_raw_scale('data', 255.0)
        self.Net.forward(data=np.asarray([transformer.preprocess('data', im)]))
        out = self.Net.blobs['prob'].data

        nCrowdPrediction = out[:, 1, :, :].reshape(out.shape[2], out.shape[3], 1)
        if CaffeCrowdDetectionModel.DEBUG_LEVEL >= 3:
            print("Inference    ...", self.__elapsedSeconds(datetime.now(), dStart))

        # Rescaled the output to the input dimensions
        nHeatmap = cv2.resize(nCrowdPrediction, (nInputWidth * self.OutputScale, nInputHeight * self.OutputScale))
        if CaffeCrowdDetectionModel.DEBUG_LEVEL >= 3:
            if self.Net.blobs['data'].height == 1:
                print("Classification:", nCrowdPrediction)

        if (nPadRows > 0):
            nHeatmap = nHeatmap[nPadRows * self.OutputScale:-nPadRows * self.OutputScale, :]
        elif (nPadCols > 0):
            nHeatmap = nHeatmap[:, nPadCols * self.OutputScale:-nPadCols * self.OutputScale]

        nHeatmapGrayscale = (nHeatmap[:, :] * 255).astype(np.uint8)
        if CaffeCrowdDetectionModel.DEBUG_LEVEL >= 3:
            print("Rescale Output...", self.__elapsedSeconds(datetime.now(), dStart))

        if CaffeCrowdDetectionModel.DEBUG_LEVEL >= 2:
            print("Image: %dx%d  |  NN Input: %dx%d  | NN Output: %dx%d | Heatmap %dx%d" % (
                p_oImage.shape[0], p_oImage.shape[1]
                , nInputWidth, nInputHeight
                , nCrowdPrediction.shape[0], nCrowdPrediction.shape[1]
                , nHeatmap.shape[0], nHeatmap.shape[1]
            ))

        return oImage, oImageRGB, nHeatmap, nHeatmapGrayscale
        # ----------------------------------------------------------------------------


# ==============================================================================================


# ==============================================================================================
class VisualizationPanel(object):
    # ------------------------------------------------------------------------------------
    def __init__(self):
        self.SourcePanel = None
        self.BlendPanel = None
        self.HeatmapPanel = None
    # ------------------------------------------------------------------------------------


# ==============================================================================================


# ==============================================================================================
class InputAndHeatmapVisualizer(ThreadContext):
    # ------------------------------------------------------------------------------------
    def __init__(self, p_oParent):
        super(InputAndHeatmapVisualizer, self).__init__("Visualization Window")
        # ........................... |  Instance Attributes | ...........................
        self.Parent = p_oParent
        # self.Config                 = self.Parent.Config
        self.Size = p_oParent.NN_INPUT_SIZE * p_oParent.NN_OUTPUT_SCALE
        self.Height = p_oParent.NN_INPUT_HEIGHT * p_oParent.NN_OUTPUT_SCALE
        self.Width = p_oParent.NN_INPUT_WIDTH * p_oParent.NN_OUTPUT_SCALE

        self.IsBlending = None
        self.BlendingAmount = None
        self.BlendingColormap = None
        self.IsRecordingVideo = None
        self.VideoFileName = None
        self.VisualizationColorMap = None
        self.VisualizationVMax = None
        self.NumberOfDrones = None

        self.Image = [np.zeros((self.Height, self.Width, 3), np.uint8)] * 3
        self.BlendedImage = [np.zeros((self.Height, self.Width, 3), np.uint8)] * 3
        self.Heatmap = [np.zeros((self.Height, self.Width), np.uint8)] * 3
        self.Count = [0] * 3
        self.Images = [[]] * 3
        self.Heatmaps = [[]] * 3
        self.Locked = False

        self.Axis = [None] * 9
        self.Panel = [None] * 9

        self.VisPanels = []
        self.HasDisplayedFirst = False
        # ................................................................................
        self.__initSettings()
        if self.IsRecordingVideo:
            assert self.VideoFileName is not None

    # ------------------------------------------------------------------------------------
    def __initSettings(self):
        self.VisualizationColorMap = "seismic"
        if rospy.has_param("visualization_cmap"):
            self.VisualizationColorMap = rospy.get_param("visualization_cmap")
        # if "visualization_cmap" in self.Config:
        #   self.VisualizationColorMap = self.Config["visualization_cmap"]

        self.VisualizationVMax = 1.0
        if rospy.has_param("visualization_vmax"):
            self.VisualizationVMax = float(rospy.get_param("visualization_vmax"))
        # if "visualization_vmax" in self.Config:
        #   self.VisualizationVMax = float(self.Config["visualization_vmax"])
        if self.Parent.EMULATION == "airsim":
            self.VisualizationVMax = self.VisualizationVMax * 0.35

        self.NumberOfDrones = 1
        if rospy.has_param("visualization_number_of_drones"):
            self.NumberOfDrones = int(rospy.get_param("visualization_number_of_drones"))
        # if "visualization_number_of_drones" in self.Config:
        #   self.NumberOfDrones = int(self.Config["visualization_number_of_drones"])

        self.IsBlending = False
        if rospy.has_param("visualization_is_blending"):
            self.IsBlending = (rospy.get_param("visualization_is_blending") == 1)
        # if "visualization_is_blending" in self.Config:
        #   self.IsBlending = (self.Config["visualization_is_blending"] == 1)

        self.IsShowingAll = False
        if rospy.has_param("visualization_show_all"):
            self.IsShowingAll = (rospy.get_param("visualization_show_all") == 1)
        # if "visualization_show_all" in self.Config:
        #   self.IsShowingAll = (self.Config["visualization_show_all"] == 1)

        if self.IsShowingAll:
            self.IsBlending = True
            self.WindowsPerDrone = 3
        else:
            self.WindowsPerDrone = 2

        self.BlendingAmount = 0.5
        if rospy.has_param("visualization_blending_amount"):
            self.BlendingAmount = float(rospy.get_param("visualization_blending_amount"))
        # if "visualization_blending_amount" in self.Config:
        #   self.BlendingAmount = float(self.Config["visualization_blending_amount"])

        self.BlendingColormap = "Reds"
        if rospy.has_param("visualization_blending_color_map"):
            self.BlendingColormap = rospy.get_param("visualization_blending_color_map")
        # if "visualization_blending_color_map" in self.Config:
        #   self.BlendingColormap = self.Config["visualization_blending_color_map"]

        self.IsRecordingVideo = False
        if rospy.has_param("visualization_is_recording"):
            self.IsRecordingVideo = rospy.get_param("visualization_is_recording")
        # if "visualization_is_recording" in self.Config:
        #   self.IsRecordingVideo = self.Config["visualization_is_recording"]

        self.VideoFileName = "visualization.mp4"
        if rospy.has_param("visualization_video_filename"):
            self.VideoFileName = rospy.get_param("visualization_video_filename")
        # if "visualization_video_filename" in self.Config:
        #   self.VideoFileName = self.Config["visualization_video_filename"]

        self.RecordingSecs = 10
        if rospy.has_param("visualization_recording_secs"):
            self.RecordingSecs = rospy.get_param("visualization_recording_secs")
        # if "visualization_recording_secs" in self.Config:
        #   self.RecordingSecs = self.Config["visualization_recording_secs"]

        self.RecordingFPS = 5
        if rospy.has_param("visualization_recording_fps"):
            self.RecordingFPS = rospy.get_param("visualization_recording_fps")
        # if "visualization_recording_fps" in self.Config:
        #   self.RecordingFPS = self.Config["visualization_recording_fps"]

        if self.NumberOfDrones == 1:
            if self.WindowsPerDrone == 3:
                self.FigureSize = [12, 5]
            else:
                self.FigureSize = [10, 7]
        elif self.NumberOfDrones == 2:
            if self.WindowsPerDrone == 3:
                self.FigureSize = [7, 10]
            else:
                self.FigureSize = [7, 7]
        elif self.NumberOfDrones == 3:
            if self.WindowsPerDrone == 3:
                self.FigureSize = [9, 9]
            else:
                self.FigureSize = [9, 7]

        if rospy.has_param("visualization_figure_size"):
            self.FigureSize = rospy.get_param("visualization_figure_size")
        # if "visualization_figure_size" in self.Config:
        #   self.FigureSize = self.Config["visualization_figure_size"]

        self.IsTightLayout = False
        if rospy.has_param("visualization_tight_layout"):
            self.IsTightLayout = (rospy.get_param("visualization_tight_layout") == 1)
        # if "visualization_tight_layout" in self.Config:
        #   self.IsTightLayout = self.Config["visualization_tight_layout"] == 1

        self.IsSafeMode = False
        self.IsAirSim = (self.Parent.EMULATION == "airsim")

    # ------------------------------------------------------------------------------------
    def Display(self, p_nDroneID, p_oImage, p_oHeatmap):
        if (p_oImage is None) or (p_oHeatmap is None):
            return

        self.Locked = True
        self.Image[p_nDroneID - 1] = p_oImage.copy()
        self.Images[p_nDroneID - 1].append(self.Image[p_nDroneID - 1])

        self.Heatmap[p_nDroneID - 1] = p_oHeatmap.copy()
        self.Heatmaps[p_nDroneID - 1].append(self.Heatmap[p_nDroneID - 1])

        self.Count[p_nDroneID - 1] += 1
        self.Locked = False

        if not self.HasDisplayedFirst:
            self.HasDisplayedFirst = True
            # ------------------------------------------------------------------------------------

    def InitiateWindows(self):
        assert self.NumberOfDrones <= 3, "Visualization is supported for 1-3 drones"
        fig = plt.figure(figsize=self.FigureSize)
        fig.suptitle("Crowd Detection")

        # Visualization for only one drone
        if self.NumberOfDrones == 1:
            # create two subplots
            self.Axis[0] = plt.subplot(1, 2, 1)
            self.Axis[1] = plt.subplot(1, 2, 2)
            self.Axis[0].set_title("Drone 1")

            # create two image plots
            self.Panel[0] = self.Axis[0].imshow(self.Image[0])
            self.Panel[1] = self.Axis[1].imshow(self.Heatmap[0], cmap=self.VisualizationColorMap,
                                                vmax=self.VisualizationVMax)

        # Visualization for 2,3 drones
        if self.NumberOfDrones > 1:
            # create two subplots
            self.Axis[0] = plt.subplot(2, self.NumberOfDrones, 1)
            self.Axis[1] = plt.subplot(2, self.NumberOfDrones, 1 + self.NumberOfDrones)
            self.Axis[0].set_title("Drone 1")

            # create two image plots
            self.Panel[0] = self.Axis[0].imshow(self.Image[0])
            self.Panel[1] = self.Axis[1].imshow(self.Heatmap[0], cmap=self.VisualizationColorMap,
                                                vmax=self.VisualizationVMax)

            # create two subplots
            self.Axis[2] = plt.subplot(2, self.NumberOfDrones, 2)
            self.Axis[3] = plt.subplot(2, self.NumberOfDrones, 2 + self.NumberOfDrones)
            self.Axis[2].set_title("Drone 2")

            # create two image plots
            self.Panel[2] = self.Axis[2].imshow(self.Image[1])
            self.Panel[3] = self.Axis[3].imshow(self.Heatmap[1], cmap=self.VisualizationColorMap,
                                                vmax=self.VisualizationVMax)

        # Visualization for 3 drones
        if self.NumberOfDrones >= 3:
            # create two subplots
            self.Axis[4] = plt.subplot(2, self.NumberOfDrones, 3)
            self.Axis[5] = plt.subplot(2, self.NumberOfDrones, 3 + self.NumberOfDrones)
            self.Axis[4].set_title("Drone 3")

            # create two image plots
            self.Panel[4] = self.Axis[4].imshow(self.Image[2])
            self.Panel[5] = self.Axis[5].imshow(self.Heatmap[2], cmap=self.VisualizationColorMap,
                                                vmax=self.VisualizationVMax)
        print("Visualization Initialized")
        # ------------------------------------------------------------------------------------

    def __updateWindowsOld(self, p_oParam):
        if self.Locked:
            return

        # Window for drone 1
        if self.HasDisplayedFirst and self.IsBlending:
            oBlendedImage = self.__superImpose(self.Image[0], self.__visualizeData(self.Heatmap[0]))
            self.Panel[0].set_data(oBlendedImage)
        else:
            self.Panel[0].set_data(self.Image[0])
        self.Panel[1].set_data(self.Heatmap[0])

        # Window for drone 2
        if self.NumberOfDrones >= 2:
            if self.HasDisplayedFirst and self.IsBlending:
                oBlendedImage = self.__superImpose(self.Image[1], self.__visualizeData(self.Heatmap[1]))
                self.Panel[2].set_data(oBlendedImage)
            else:
                self.Panel[2].set_data(self.Image[1])
            self.Panel[3].set_data(self.Heatmap[1])

        # Window for drone 3
        if self.NumberOfDrones >= 3:
            if self.HasDisplayedFirst and self.IsBlending:
                oBlendedImage = self.__superImpose(self.Image[2], self.__visualizeData(self.Heatmap[2]))
                self.Panel[4].set_data(oBlendedImage)
            else:
                self.Panel[4].set_data(self.Image[2])
            self.Panel[5].set_data(self.Heatmap[2])
            # ------------------------------------------------------------------------------------

    def __superImpose(self, p_oImage, p_oHeatmap):

        # suppose img1 and img2 are your two images
        img1 = Im.fromarray(p_oImage)
        img2 = Im.fromarray(p_oHeatmap)

        # suppose img2 is to be shifted by `shift` amount
        shift = ((p_oImage.shape[0] - p_oHeatmap.shape[0]) // 2, 0)

        # compute the size of the panorama
        nw, nh = map(max, map(operator.add, img2.size, shift), img1.size)

        # paste img1 on top of img2
        newimg1 = Im.new('RGBA', size=(nw, nh), color=(0, 0, 0, 0))
        newimg1.paste(img2, shift)
        newimg1.paste(img1, (0, 0))

        # paste img2 on top of img1
        newimg2 = Im.new('RGBA', size=(nw, nh), color=(0, 0, 0, 0))
        newimg2.paste(img1, (0, 0))
        newimg2.paste(img2, shift)

        # blend with alpha=0.5
        oResult = Im.blend(newimg2, newimg1, alpha=self.BlendingAmount)

        return oResult

    # ------------------------------------------------------------------------------------
    def __visualizeData(self, p_oData):
        my_cm = plt.cm.get_cmap(self.BlendingColormap)
        if self.IsAirSim:
            normed_data = p_oData / 0.35
        else:
            normed_data = p_oData

        # normed_data = (p_oData - np.min(p_oData)) / (np.max(p_oData) - np.min(p_oData))
        mapped_data = my_cm(p_oData)
        # which will give you back a MxNx4 array mapped between 0 and 1,
        mapped_datau8 = (255 * my_cm(normed_data)).astype('uint8')

        return mapped_datau8

    # ------------------------------------------------------------------------------------
    def __getPanelIndexes(self):
        nSourceIndex = 0
        if self.IsBlending:
            if self.IsShowingAll:
                nBlendIndex = 1
            else:
                nSourceIndex = None
                nBlendIndex = 0
        else:
            nBlendIndex = None
        if self.IsShowingAll:
            nHeatMapIndex = 2
        else:
            nHeatMapIndex = 1

        return nSourceIndex, nBlendIndex, nHeatMapIndex
        # ------------------------------------------------------------------------------------

    def InitiateAllWindows(self):
        assert self.NumberOfDrones <= 3, "Visualization is supported for 1-3 drones"

        fig = plt.figure(figsize=self.FigureSize)

        if self.NumberOfDrones == 1:
            fig.suptitle("Crowd Detection - Single Drone")
        else:
            if not self.IsTightLayout:
                fig.suptitle("Crowd Detection")

        nSourceIndex, nBlendIndex, nHeatMapIndex = self.__getPanelIndexes()

        self.VisPanels = []
        for nDroneIndex in range(0, self.NumberOfDrones):
            oVisPanel = VisualizationPanel()
            self.VisPanels.append(oVisPanel)

            print("check1")

            oFirstAxis = None

            nAxisIndex = nDroneIndex + 1
            if nSourceIndex is not None:
                if self.NumberOfDrones == 1:
                    oAxis = plt.subplot(1, self.WindowsPerDrone, nAxisIndex)
                else:
                    oAxis = plt.subplot(self.WindowsPerDrone, self.NumberOfDrones, nAxisIndex)
                print("check2")
                oVisPanel.SourcePanel = oAxis.imshow(self.Image[nDroneIndex])
                print("check2-ok")
                if oFirstAxis is None:
                    oFirstAxis = oAxis
                nAxisIndex += self.NumberOfDrones

            if nBlendIndex is not None:
                if self.NumberOfDrones == 1:
                    oAxis = plt.subplot(1, self.WindowsPerDrone, nAxisIndex)
                else:
                    oAxis = plt.subplot(self.WindowsPerDrone, self.NumberOfDrones, nAxisIndex)
                oVisPanel.BlendPanel = oAxis.imshow(self.BlendedImage[nDroneIndex])
                if oFirstAxis is None:
                    oFirstAxis = oAxis
                nAxisIndex += self.NumberOfDrones

            print("check3")

            if nHeatMapIndex is not None:
                if self.NumberOfDrones == 1:
                    oAxis = plt.subplot(1, self.WindowsPerDrone, nAxisIndex)
                else:
                    oAxis = plt.subplot(self.WindowsPerDrone, self.NumberOfDrones, nAxisIndex)
                oVisPanel.HeatmapPanel = oAxis.imshow(self.Heatmap[nDroneIndex], cmap=self.VisualizationColorMap,
                                                      vmax=self.VisualizationVMax)
                if oFirstAxis is None:
                    oFirstAxis = oAxis
                nAxisIndex += self.NumberOfDrones

            print("check4")

            if (oFirstAxis is not None) and (self.NumberOfDrones > 1):
                oFirstAxis.set_title("Drone %d" % (nDroneIndex + 1))

        if self.IsTightLayout:
            fig.tight_layout()
        print("Visualization Initialized")
        # ------------------------------------------------------------------------------------

    def __updateWindows(self, p_oParam):
        if self.Locked:
            return

        nSourceIndex, nBlendIndex, nHeatMapIndex = self.__getPanelIndexes()

        for nDroneIndex in range(0, self.NumberOfDrones):
            if nDroneIndex == 0:
                bMustRender = True
            else:
                bMustRender = (self.NumberOfDrones >= (nDroneIndex + 1))

            if bMustRender:
                oVisPanel = self.VisPanels[nDroneIndex]

                # Source
                if nSourceIndex is not None:
                    oVisPanel.SourcePanel.set_data(self.Image[nDroneIndex])
                # Blending
                if nBlendIndex is not None:
                    if self.HasDisplayedFirst and self.IsBlending:
                        oBlendedImage = self.__superImpose(self.Image[nDroneIndex],
                                                           self.__visualizeData(self.Heatmap[nDroneIndex]))
                        oVisPanel.BlendPanel.set_data(oBlendedImage)
                # Heatmap
                if nHeatMapIndex is not None:
                    oVisPanel.HeatmapPanel.set_data(self.Heatmap[nDroneIndex])

    # ------------------------------------------------------------------------------------
    def ThreadMain(self, p_oArgs):
        if self.IsSafeMode:
            self.InitiateWindows()()

            if self.IsRecordingVideo:
                self.RecordVideo()
            else:
                while self.Continue:
                    oAnimation = FuncAnimation(plt.gcf(), self.__updateWindowsOld(), interval=40)
                    plt.show()
                    plt.pause(0.01)
        else:
            self.InitiateAllWindows()

            if self.IsRecordingVideo:
                self.RecordVideo()
            else:
                while self.Continue:
                    oAnimation = FuncAnimation(plt.gcf(), self.__updateWindows, interval=40)
                    plt.show()
                    plt.pause(0.01)

    # ------------------------------------------------------------------------------------
    def RecordVideo(self):
        sTime = datetime.now().strftime("%Y%m%d_%H%M")
        sFileName = self.VideoFileName % sTime
        sSignalFile = (self.VideoFileName % sTime) + ".OK.txt"

        if not os.path.isfile(sSignalFile):
            nFrames = self.RecordingSecs * self.RecordingFPS
        WriterClass = anim.writers['ffmpeg']
        oWriter = WriterClass(fps=self.RecordingFPS, metadata=dict(artist='AIIA'), bitrate=1800)
        oAnimation = FuncAnimation(plt.gcf(), self.__updateWindows, frames=nFrames, interval=40)  # , blit=True)
        oAnimation.save(sFileName, fps=self.RecordingFPS, extra_args=['-vcodec', 'libx264'])

        with open(sSignalFile, "w") as oFile:
            oFile.write("Seconds:%d  Frames:%d  (%d FPS)" % (self.RecordingSecs, nFrames, self.RecordingFPS))
    # ------------------------------------------------------------------------------------


# ==============================================================================================


# ==============================================================================================
class VisualSemanticAnalyzerNode:
    SourceFrameID = 0

    # ------------------------------------------------------------------------------------
    def __init__(self):
        # ........................... |  Instance Attributes | ...........................
        # self.ModulePath = GetROSNodeBaseFolder()
        rospack = rospkg.RosPack()
        self.ModulePath = os.path.join(rospack.get_path("ground_visual_analysis"), "scripts")
        self.ModelsBasePath = os.path.join(self.ModulePath, "models")

        # with open(os.path.join(self.ModulePath, "crowd_detector.cfg")) as oConfigFile:
        #   self.Config = json.load(oConfigFile)

        self.__initDebugSettings()

        self.Bridge = CvBridge()

        self.NumberOfDrones = 1
        if rospy.has_param("number_of_drones"):
            self.NumberOfDrones = rospy.get_param("number_of_drones")

        if self.NumberOfDrones == 1:
            self.Publisher = rospy.Publisher("drone_1/visual_analysis/crowd_heatmap", CrowdHeatmap, queue_size=1)

        if self.NumberOfDrones == 2:
            self.Publisher = rospy.Publisher("drone_1/visual_analysis/crowd_heatmap", CrowdHeatmap, queue_size=1)
            self.Publisher2 = rospy.Publisher("drone_2/visual_analysis/crowd_heatmap", CrowdHeatmap, queue_size=1)

        if self.NumberOfDrones == 3:
            self.Publisher = rospy.Publisher("drone_1/visual_analysis/crowd_heatmap", CrowdHeatmap, queue_size=1)
            self.Publisher2 = rospy.Publisher("drone_2/visual_analysis/crowd_heatmap", CrowdHeatmap, queue_size=1)
            self.Publisher3 = rospy.Publisher("drone_3/visual_analysis/crowd_heatmap", CrowdHeatmap, queue_size=1)

        # self.Publisher4 = rospy.Publisher("drone_4/visual_analysis/crowd_heatmap", CrowdHeatmap, queue_size=1)
        # self.Publisher5 = rospy.Publisher("drone_5/visual_analysis/crowd_heatmap", CrowdHeatmap, queue_size=1)
        # self.Publisher6 = rospy.Publisher("drone_6/visual_analysis/crowd_heatmap", CrowdHeatmap, queue_size=1)
        # self.Publisher7 = rospy.Publisher("drone_7/visual_analysis/crowd_heatmap", CrowdHeatmap, queue_size=1)
        # self.ServiceOnEnableDebug  = rospy.Service('drone_X/visual_analysis/crowd_debug_on', Empty, self.OnEnableDebug)
        # self.ServiceOnDisableDebug = rospy.Service('drone_X/visual_analysis/crowd_debug_off', Empty, self.OnDisableDebug)
        self.PriorMoment = None

        self.TotalRaw = 0.0
        self.CountRaw = 0.0
        self.TotalJPEG = 0.0
        self.CountJPEG = 0.0
        self.IsJPEG = False
        self.IsRaw = True
        self.IsRecalling = True

        self.ModelName = ""
        if rospy.has_param("model_name"):
            self.ModelName = rospy.get_param("model_name")
        # if "model_name" in self.Config:
        #   self.ModelName = str(self.Config["model_name"]) 

        self.VisualsWindow = InputAndHeatmapVisualizer(self)
        self.FileInput = FileInputThread(self)

        # self.NumberOfDrones = 1
        # if rospy.has_param("number_of_drones"):
        #   self.NumberOfDrones = rospy.get_param("number_of_drones")
        # if "number_of_drones" in self.Config:
        #   self.NumberOfDrones = self.Config["number_of_drones"]

        self.Multiplexer = MultipleInputOutputCoordinator(self)

        self.Model = CaffeCrowdDetectionModel(self, os.path.join(self.ModelsBasePath, self.ModelName))
        self.Model.Load()
        # self.Model.Net.blobs['data'].reshape(1, 3, self.NN_INPUT_SIZE, self.NN_INPUT_SIZE)
        self.Model.Net.blobs['data'].reshape(1, 3, self.NN_INPUT_HEIGHT, self.NN_INPUT_WIDTH)

        if self.LOG_TIMINGS:
            self.Log = LogFileThread(self.__getLogFileName())

        if self.IS_DEMO_MODE:
            # self.DroneDataEmulator = DroneSensorDataPublisher(self.EMULATION, self.Config)
            self.DroneDataEmulator = DroneSensorDataPublisher(self.EMULATION)
            self.DroneDataEmulator.LoadData()

        # ................................................................................

    # ------------------------------------------------------------------------------------
    def __initDebugSettings(self):
        self.IS_BACKUP_VIDEO_STREAMER = False

        # Settings "emulation" should be set for emulation modes 
        sEmulation = "none"
        if rospy.has_param("emulation"):
            sEmulation = rospy.get_param("emulation")
        # if "emulation" in self.Config:
        #   sEmulation = self.Config["emulation"]

        self.EMULATION = sEmulation
        self.IS_DEMO_MODE = False
        if (self.EMULATION == "airsim") or self.EMULATION.startswith("dji"):
            self.IS_DEMO_MODE = True
        self.EMULATION_DEBUG_FOLDER = rospy.get_param("emulation_debug_folder")
        self.EMULATION_IMAGE_TOPIC = rospy.get_param("emulation_image_topic")
        # self.EMULATION_DEBUG_FOLDER = self.Config["emulation_debug_folder"]
        # self.EMULATION_IMAGE_TOPIC  = self.Config["emulation_image_topic"]

        self.VISUALIZATION_COLOR_MAP = "seismic"
        if rospy.has_param("visualization_cmap"):
            self.VISUALIZATION_COLOR_MAP = rospy.get_param("visualization_cmap")
        # if "visualization_cmap" in self.Config:
        #   self.VISUALIZATION_COLOR_MAP = self.Config["visualization_cmap"]

        self.VISUALIZATION_VMAX = 1.0
        if rospy.has_param("visualization_vmax"):
            self.VISUALIZATION_VMAX = float(rospy.get_param("visualization_vmax"))
        # if "visualization_vmax" in self.Config:
        #   self.VISUALIZATION_VMAX = float(self.Config["visualization_vmax"])
        if self.EMULATION == "airsim":
            self.VISUALIZATION_VMAX = self.VISUALIZATION_VMAX * 0.35

        self.VISUALIZATION_NUMBER_OF_DRONES = 1
        if rospy.has_param("visualization_number_of_drones"):
            self.VISUALIZATION_NUMBER_OF_DRONES = float(rospy.get_param("visualization_number_of_drones"))
        # if "visualization_number_of_drones" in self.Config:
        #   self.VISUALIZATION_NUMBER_OF_DRONES = float(self.Config["visualization_number_of_drones"])

        self.VISUALIZATION_IS_BLENDING = False
        if rospy.has_param("visualization_is_blending"):
            self.VISUALIZATION_IS_BLENDING = (rospy.get_param("visualization_is_blending") == 1)
        # if "visualization_is_blending" in self.Config:
        #   self.VISUALIZATION_IS_BLENDING = (self.Config["visualization_is_blending"] == 1) 

        self.VISUALIZATION_SHOW_ALL = False
        if rospy.has_param("visualization_show_all"):
            self.VISUALIZATION_SHOW_ALL = (rospy.get_param("visualization_show_all") == 1)
        # if "visualization_show_all" in self.Config:
        #   self.VISUALIZATION_SHOW_ALL = (self.Config["visualization_show_all"] == 1) 

        self.VISUALIZATION_BLENDING_AMOUNT = 0.5
        if rospy.has_param("visualization_blending_amount"):
            self.VISUALIZATION_BLENDING_AMOUNT = rospy.get_param("visualization_blending_amount")
        # if "visualization_blending_amount" in self.Config:
        #   self.VISUALIZATION_BLENDING_AMOUNT = self.Config["visualization_blending_amount"] 

        self.VISUALIZATION_BLENDING_COLOR_MAP = "Reds"
        if rospy.has_param("visualization_blending_color_map"):
            self.VISUALIZATION_BLENDING_COLOR_MAP = rospy.get_param("visualization_blending_color_map")
        # if "visualization_blending_color_map" in self.Config:
        #   self.VISUALIZATION_BLENDING_COLOR_MAP = self.Config["visualization_blending_color_map"] 

        self.VISUALIZATION_IS_RECORDING = False
        if rospy.has_param("visualization_is_recording"):
            self.VISUALIZATION_IS_RECORDING = rospy.get_param("visualization_is_recording")
        # if "visualization_is_recording" in self.Config:
        #   self.VISUALIZATION_IS_RECORDING = self.Config["visualization_is_recording"] 

        self.VISUALIZATION_VIDEO_FILENAME = "visualization.mp4"
        if rospy.has_param("visualization_video_filename"):
            self.VISUALIZATION_VIDEO_FILENAME = rospy.get_param("visualization_video_filename")
        # if "visualization_video_filename" in self.Config:
        #   self.VISUALIZATION_VIDEO_FILENAME = self.Config["visualization_video_filename"]          

        self.IS_SAVING_HEATMAP = rospy.get_param("is_saving_heatmap")
        self.IS_SAVING_INPUT_IMAGE = rospy.get_param("is_saving_input_image")
        self.IS_SHOWING_HEATMAP = rospy.get_param("is_showing_heatmap")
        self.IS_THRESHOLDING = rospy.get_param("is_thresholding")
        self.LOG_TIMINGS = rospy.get_param("is_logging_timings")
        self.IS_VISUALIZING_INPUT = rospy.get_param("is_visualizing_input")
        self.IS_VISUALIZING_OUTPUT = rospy.get_param("is_visualizing_output")
        self.IS_DEBUGGING = rospy.get_param("is_debugging")
        # self.IS_SAVING_HEATMAP      = self.Config["is_saving_heatmap"]
        # self.IS_SAVING_INPUT_IMAGE  = self.Config["is_saving_input_image"]
        # self.IS_SHOWING_HEATMAP     = self.Config["is_showing_heatmap"]
        # self.IS_THRESHOLDING        = self.Config["is_thresholding"]
        # self.LOG_TIMINGS            = self.Config["is_logging_timings"]
        # self.IS_VISUALIZING_INPUT   = self.Config["is_visualizing_input"]
        # self.IS_VISUALIZING_OUTPUT  = self.Config["is_visualizing_output"]
        # self.IS_DEBUGGING           = self.Config["is_debugging"]

        self.ROS_QUEUE_SIZE = 1
        if rospy.has_param("ros_queue_size"):
            self.ROS_QUEUE_SIZE = rospy.get_param("ros_queue_size")
        # if "ros_queue_size" in self.Config:
        #   self.ROS_QUEUE_SIZE         = self.Config["ros_queue_size"]

        self.NN_INPUT_SIZE = rospy.get_param("nn_input_size")
        self.NN_INPUT_HEIGHT = rospy.get_param("nn_input_height")
        self.NN_INPUT_WIDTH = rospy.get_param("nn_input_width")
        # self.NN_INPUT_SIZE          = self.Config["nn_input_size"]  
        self.NN_OUTPUT_SCALE = 1
        if rospy.has_param("nn_output_scale"):
            self.NN_OUTPUT_SCALE = rospy.get_param("nn_output_scale")
        # if "nn_output_scale" in self.Config:
        #   self.NN_OUTPUT_SCALE = self.Config["nn_output_scale"]

        self.NN_DEBUG_LEVEL = 0
        if rospy.has_param("nn_debug_level"):
            self.NN_DEBUG_LEVEL = rospy.get_param("nn_debug_level")
        # if "nn_debug_level" in self.Config:
        #   self.NN_DEBUG_LEVEL = self.Config["nn_debug_level"]

    # ------------------------------------------------------------------------------------
    def __getLogFileName(self):
        sDateTime = datetime.now().strftime("%Y-%m-%d_%H%M")

        if self.IsJPEG:
            LOG_FILENAME = "JPEGFrameDelay-%s.txt" % sDateTime
        else:
            LOG_FILENAME = "RawFrameDelay-%s.txt" % sDateTime

        # Attempts to read the pipeline string from a settings file for the GStreamer sender
        sFileName = os.path.join(os.path.dirname(os.path.realpath(LOG_FILENAME)), LOG_FILENAME)

        return sFileName

    # ----------------------------------------------------------------------------
    def __datetime_to_float(self, p_dDateTime):
        dEpoch = datetime.utcfromtimestamp(0)
        if p_dDateTime is None:
            total_seconds = 0
        else:
            total_seconds = (p_dDateTime - dEpoch).total_seconds()
        # total_seconds will be in decimals (millisecond precision)
        return total_seconds

    # ----------------------------------------------------------------------------
    def __float_to_datetime(self, p_nFloat):
        return datetime.datetime.fromtimestamp(p_nFloat)
        # ----------------------------------------------------------------------------

    def __pythonToRosTime(self, p_dPythonTime):
        return rospy.Time.from_seconds(self.__datetime_to_float(p_dPythonTime))
        # ------------------------------------------------------------------------------------

    def _rosToPythonTime(self, p_dRosTime):
        return datetime.fromtimestamp(p_dRosTime.to_time())

    # ------------------------------------------------------------------------------------
    def __extractFrameTimestamps(self, p_oFrameMeta):
        dCapture = datetime(p_oFrameMeta.capture_time.year, p_oFrameMeta.capture_time.month,
                            p_oFrameMeta.capture_time.day
                            , p_oFrameMeta.capture_time.hour, p_oFrameMeta.capture_time.minute,
                            p_oFrameMeta.capture_time.second
                            , p_oFrameMeta.capture_time.microsecond)

        dCompress = datetime(p_oFrameMeta.compress_time.year, p_oFrameMeta.compress_time.month,
                             p_oFrameMeta.compress_time.day
                             , p_oFrameMeta.compress_time.hour, p_oFrameMeta.compress_time.minute,
                             p_oFrameMeta.compress_time.second
                             , p_oFrameMeta.compress_time.microsecond)
        return dCapture, dCompress

    # ------------------------------------------------------------------------------------
    def OnStreamerReceiveRawImage(self, p_oTimedFrame):
        # ROS time of the frame capture      
        dCaptureMomentROS = self._rosToPythonTime(p_oTimedFrame.meta.utctime)
        # Python timestamps for the moment of the frame capture and compression
        dCaptureMoment, dCompressMoment = self.__extractFrameTimestamps(p_oTimedFrame.meta)

        # Conversion from ROS image to OpenCV format
        dBeforeConversionMoment = datetime.utcnow()
        oImage = self.Bridge.imgmsg_to_cv2(p_oTimedFrame.image, "8UC3")
        if self.IS_VISUALIZING_INPUT:
            cv2.imshow("image", oImage)
            cv2.waitKey(1)

        # Python time (clock is sync using ntpdate -s time.nist.gov)
        dNow = datetime.utcnow()
        # Use the same date conversion pipeline for ROS time        
        dNowROS = self._rosToPythonTime(self.__pythonToRosTime(datetime.utcnow()))

        if self.LOG_TIMINGS:
            self.LogTimings(dCaptureMoment, dCompressMoment, dBeforeConversionMoment, dNow,
                            p_dCaptureMomentROS=dCaptureMomentROS, p_dNowROS=dNowROS, p_bIsJPEG=False)

        _, nPrediction = self.Model.Predict(oImage)
        self.PublishCrowdHeatmap(nPrediction, p_oTimedFrame.header)

    # ------------------------------------------------------------------------------------
    def OnStreamerReceiveJPEGImage(self, p_oTimedFrame):
        # ROS time of the frame capture  
        dCaptureMomentROS = self._rosToPythonTime(p_oTimedFrame.meta.utctime)
        # Python timestamps for the moment of the frame capture and compression
        dCaptureMoment, dCompressMoment = self.__extractFrameTimestamps(p_oTimedFrame.meta)

        # Decompression from ROS JPEG image to OpenCV format        
        dBeforeDecompressionMoment = datetime.utcnow()
        oImage = self.Bridge.compressed_imgmsg_to_cv2(p_oTimedFrame.image_jpeg)
        if self.IS_VISUALIZING_INPUT:
            cv2.imshow("image", oImage)
            cv2.waitKey(1)

        # Python time (clock is sync using ntpdate -s time.nist.gov)
        dNow = datetime.utcnow()
        # Use the same date conversion pipeline for ROS time           
        dNowROS = self._rosToPythonTime(self.__pythonToRosTime(datetime.utcnow()))

        if self.LOG_TIMINGS:
            self.LogTimings(dCaptureMoment, dCompressMoment, dBeforeDecompressionMoment, dNow,
                            p_dCaptureMomentROS=dCaptureMomentROS, p_dNowROS=dNowROS, p_bIsJPEG=True)

        _, nPrediction = self.Model.Predict(oImage)
        self.PublishCrowdHeatmap(nPrediction, p_oTimedFrame.header)

    # ------------------------------------------------------------------------------------
    def PublishCrowdHeatmap(self, p_oHeatMapArray, p_oHeader=None):
        oImageMessage = self.Bridge.cv2_to_imgmsg(p_oHeatMapArray, "mono8")

        if self.NumberOfDrones == 1:
            self.Publisher.publish(CrowdHeatmap(header=p_oHeader, heatmap=oImageMessage))

        if self.NumberOfDrones == 2:
            self.Publisher.publish(CrowdHeatmap(header=p_oHeader, heatmap=oImageMessage))
            self.Publisher2.publish(CrowdHeatmap(header=p_oHeader, heatmap=oImageMessage))

        if self.NumberOfDrones == 3:
            self.Publisher.publish(CrowdHeatmap(header=p_oHeader, heatmap=oImageMessage))
            self.Publisher2.publish(CrowdHeatmap(header=p_oHeader, heatmap=oImageMessage))
            self.Publisher3.publish(CrowdHeatmap(header=p_oHeader, heatmap=oImageMessage))

        # self.Publisher.publish(CrowdHeatmap(header = p_oHeader, heatmap = oImageMessage))
        # self.Publisher2.publish(CrowdHeatmap(header = p_oHeader, heatmap = oImageMessage))
        # self.Publisher3.publish(CrowdHeatmap(header = p_oHeader, heatmap = oImageMessage))
        # self.Publisher4.publish(CrowdHeatmap(header = p_oHeader, heatmap = oImageMessage))
        # self.Publisher5.publish(CrowdHeatmap(header = p_oHeader, heatmap = oImageMessage))
        # self.Publisher6.publish(CrowdHeatmap(header = p_oHeader, heatmap = oImageMessage))
        # self.Publisher7.publish(CrowdHeatmap(header = p_oHeader, heatmap = oImageMessage))

    # ------------------------------------------------------------------------------------
    def __elapsedSeconds(self, p_dNow, p_dThen):
        nElapsed = p_dNow - p_dThen
        nSecs = nElapsed.total_seconds()
        return nSecs
        # ------------------------------------------------------------------------------------

    def LogTimings(self, p_dCaptureMoment, p_dCompressMoment, p_dBeforeDecompressMoment, p_dNow,
                   p_dCaptureMomentROS=None, p_dNowROS=None, p_bIsJPEG=False):
        nDiffSecs = self.__elapsedSeconds(p_dCaptureMomentROS, p_dCaptureMoment)
        nLatencySecsROS = self.__elapsedSeconds(p_dNowROS, p_dCaptureMomentROS)

        nCompressSecs = self.__elapsedSeconds(p_dCompressMoment, p_dCaptureMoment)
        nDecompressSecs = self.__elapsedSeconds(p_dNow, p_dBeforeDecompressMoment)
        nLatencySecs = self.__elapsedSeconds(p_dNow, p_dCaptureMoment)

        self.TotalJPEG += nLatencySecs
        self.CountJPEG += 1.0
        nAvg = self.TotalJPEG / self.CountJPEG

        if p_bIsJPEG:
            sPrefix = "JPEG"
        else:
            sPrefix = "RAW"

        sMessage = "%s Timestamp [%s]" % (sPrefix, p_dCaptureMoment.strftime('%Y-%m-%d %H:%M:%S %f'))
        self.Log.Print(sMessage)

        if nLatencySecs - nDecompressSecs < 0:
            sMessage = "[WARNING] %s Timings | Compress = %.6f secs | Decomp. = %.6f (%.6f)" % (
                sPrefix, nCompressSecs, nDecompressSecs, nLatencySecs - nDecompressSecs)
        else:
            sMessage = "%s Timings | Compress = %.6f secs | Decomp. = %.6f (%.6f)" % (
                sPrefix, nCompressSecs, nDecompressSecs, nLatencySecs - nDecompressSecs)
        self.Log.Print(sMessage)

        sMessage = "%s Delay   | current  = %.6f secs | average = %.6f secs" % (sPrefix, nLatencySecs, nAvg)
        self.Log.Print(sMessage)

    # ------------------------------------------------------------------------------------
    def EmulateMultidroneSensorData(self):
        nFrameDelayInSecs = rospy.get_param('demo_frame_delay')
        # nFrameDelayInSecs = self.Config['demo_frame_delay']

        # dStart = datetime.now()
        # nCount = 0.0

        for nIndex, oDataItem in enumerate(self.DroneDataEmulator.Data):

            print("=" * 40, nIndex + 1, "=" * 40)
            oHeader = self.DroneDataEmulator.Emulate(oDataItem)

            print(oDataItem.ImageFileName)

            oCVImage = cv2.imread(oDataItem.ImageFileName)

            nInputImage, nInputImageRGB, nHeatmap, nHeatmapGrayscale = self.Model.Predict(oCVImage,
                                                                                          p_bIsConverting=True)

            if self.IS_SHOWING_HEATMAP:
                self.VisualsWindow.Display(1, nInputImageRGB, nHeatmap)
                self.VisualsWindow.Display(2, nInputImageRGB, nHeatmap)
                self.VisualsWindow.Display(3, nInputImageRGB, nHeatmap)
                nThreshold = 0.3

                # Debug thresholding of crowd heatmap
                if self.IS_THRESHOLDING:
                    nHeatmapBinary = np.zeros((nHeatmap.shape[0], nHeatmap.shape[1]), dtype=np.uint8)
                    for i in range(0, nHeatmap.shape[0]):
                        for j in range(0, nHeatmap.shape[1]):
                            if nHeatmap[i, j] >= nThreshold:
                                nHeatmapBinary[i, j] = 255
                            else:
                                nHeatmapBinary[i, j] = 0
                    plt.imshow(nHeatmapBinary, cmap=self.VISUALIZATION_COLOR_MAP)
                    plt.show()

            if self.IS_SAVING_HEATMAP:
                # Save the color image using the desired Color map
                sDestFileName1 = os.path.join(self.DroneDataEmulator.VisualizationFolder,
                                              "1_" + oDataItem.ImageName + ".heatmap.png")
                plt.imsave(sDestFileName1, nHeatmap, cmap=self.VISUALIZATION_COLOR_MAP, vmin=0.0,
                           vmax=self.VISUALIZATION_VMAX)

                # Save the grayscale image
                sDestFileName2 = os.path.join(self.DroneDataEmulator.HeatmapFolder,
                                              "2_" + oDataItem.ImageName + ".gray.heatmap.png")
                cv2.imwrite(sDestFileName2, nHeatmapGrayscale)

            # Save the source image
            if self.IS_SAVING_INPUT_IMAGE:
                sDestFileName3 = os.path.join(self.DroneDataEmulator.SourceFolder,
                                              "3_" + oDataItem.ImageName + ".input.png")
                cv2.imwrite(sDestFileName3, nInputImage)

            dStartPublishCrowdHeatmap = datetime.now()
            self.PublishCrowdHeatmap(nHeatmapGrayscale, oHeader)

            if (nFrameDelayInSecs > 0) and (nFrameDelayInSecs is not None):
                time.sleep(nFrameDelayInSecs)

            # nCount += 1.0
            # dElapsed = datetime.now() - dStartPublishCrowdHeatmap
            # "secs", dElapsed.seconds + (dElapsed.microseconds/1000000.0),

            # print("[%d] DRONE:%d FPS:%.2f" % (len(self.DroneDataEmulator.Data)-nIndex, 0, (nCount / (dElapsed.seconds + (dElapsed.microseconds/1000000.0)))))

    # ------------------------------------------------------------------------------------
    def NodeOnRegisterServices(self):
        pass

    # ------------------------------------------------------------------------------------
    def Start(self):
        rospy.init_node("CrowdDetector", anonymous=True)

        self.NodeOnRegisterServices()

        # Starts the visualization thread
        if self.IS_SHOWING_HEATMAP:
            self.VisualsWindow.Resume()

        if (self.EMULATION == "dji") or (self.EMULATION == "airsim"):
            # Emulation modes
            assert self.IS_DEMO_MODE
            self.EmulateMultidroneSensorData()
        else:
            # Live operation mode
            assert self.Multiplexer is not None
            self.Multiplexer.IsEmulating = (self.EMULATION == "live")
            self.Multiplexer.SetupInputOutputTopics()

        # Starts the neural network thread    
        self.Model.Resume()

        # Starts the logging thread          
        if self.LOG_TIMINGS:
            self.Log.Resume()

        rospy.loginfo("Model:%s" % self.ModelName)
        rospy.loginfo("--------- Ready ---------")

        rospy.spin()
    # ------------------------------------------------------------------------------------


# ==============================================================================================


if __name__ == '__main__':
    node = VisualSemanticAnalyzerNode()
    node.Start()
