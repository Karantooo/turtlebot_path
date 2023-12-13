#!/usr/bin/env python3
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from geometry_msgs.msg import Twist

# Global Variables
recentCurvesList = []     # Register of the curve turns in the last recentCurvesLength frames
recentCurvesLength = 10
 
def thresholding(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0,0,209])
    upper_white = np.array([29,255,255])
    mask_white = cv2.inRange(imgHsv, lower_white, upper_white)
    return mask_white

#TODO: Añadir inversion
def warpImage(img, points, wT, hT, inv = True):
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[wT,0],[0,hT],[wT,hT]])
    if not inv:
      matrix = cv2.getPerspectiveTransform(pts1,pts2)
    else:
      matrix = cv2.getPerspectiveTransform(pts2,pts1)
    imgWarp = cv2.warpPerspective(img,matrix,(wT,hT))
    return imgWarp

def getAreaOfInterest(wT=800, hT=600):
   widthTop = 302 #Ojo
   heightTop = 397
   widthBottom = 198 #Ojo
   heightBottom = 600
   points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                      (widthBottom , heightBottom ), (wT-widthBottom, heightBottom)])
   return points

def drawPoints(img,points):
    for x in range( 0,4):
        cv2.circle(img,(int(points[x][0]),int(points[x][1])),15,(0,0,255),cv2.FILLED)
    return img

# This function finds the center column of the curve in a specified region
# For example, a region of four means that we only take the 1/4 bottom area of the image
# if display = True, then this function also displays the histogram of the brightness per column in the region
def getHistogram(img, display = False, minPercentage = 0.1, region = 4):
    # histValues contains the sum of the brightness of each column of the region
    if region == 1:
      histValues = np.sum(img, axis=0)
    else :
      debugShape = img[((img.shape[0]//region))*3:,:]
      print(debugShape.shape)
      histValues = np.sum(debugShape, axis=0)
      #cv2.imread("Prueba", img[img.shape[0]//region:,:])

    maxValue = np.max(histValues)

    # We define a minimum value to filter the noise
    minValue = minPercentage * maxValue

    # List of indexes where histValues is more or equal than minValue
    indexArray = np.where(histValues >= minValue)

    regionBaseColumn = int(np.average(indexArray)) # in the video is called basePoint

    if display: # If we want to display the histogram

      # We create a empty image with the same size
      imgHist = np.zeros((img.shape[0],img.shape[1],3),np.uint8)
      for x,intensity in enumerate(histValues):
        
        # In the new image we draw the histrogram line per line
        if intensity > minValue: color = (255,0,255)
        else: color = (0,0,255)
        cv2.line(imgHist, (x, img.shape[0]), (x, int(img.shape[0]-(intensity//255//region))), color, 1)
      
      # We also draw in the new image a point of the center of the found curve
      cv2.circle(imgHist,(regionBaseColumn, img.shape[0]), 20, (0,255,255), cv2.FILLED)
      return regionBaseColumn, imgHist
    
    return regionBaseColumn


## Displaying Functions

def getResultImage(imageOriginal, imageWidth, imageHeigth, imageWarp, warpPoints, curveTurn, timer):
    imgResult = imageOriginal.copy()

    imgInvWarp = warpImage(imageWarp, warpPoints, imageWidth, imageHeigth, inv = True)
    imgInvWarp = cv2.cvtColor(imgInvWarp,cv2.COLOR_GRAY2BGR)
    imgInvWarp[0:imageHeigth//3,0:imageWidth] = 0,0,0

    imgLaneColor = np.zeros_like(imageOriginal)
    imgLaneColor[:] = 0, 255, 0
    imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)

    imgResult = cv2.addWeighted(imgResult,1,imgLaneColor,1,0)
    midY = 450

    cv2.putText(imgResult,str(curveTurn),(imageWidth//2-80,85),cv2.FONT_HERSHEY_COMPLEX,2,(255,0,255),3)
    cv2.line(imgResult,
             (imageWidth//2,midY),
             (imageWidth//2+(curveTurn*3),midY),
             (255,0,255),
             5)
    cv2.line(imgResult,
             ((imageWidth // 2 + (curveTurn * 3)), midY-25),
             (imageWidth // 2 + (curveTurn * 3), midY+25),
             (0, 255, 0),
             5)
    for x in range(-30, 30):
      w = imageWidth // 20
      cv2.line(imgResult,
               (w * x + int(curveTurn//50 ), midY-10),
               (w * x + int(curveTurn//50 ), midY+10),
               (0, 0, 255),
               2)

    #fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    #cv2.putText(imgResult,
    #            'FPS '+str(int(fps)),
    #            (20, 40),
    #            cv2.FONT_HERSHEY_SIMPLEX,
    #            1,
    #            (230,50,50),
    #            3)

    return imgResult

def stackImages(scale,imgArray):
    
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]

    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)

    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

class ImageSubscriber(Node):
  
  def __init__(self):
    
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/camera/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def get_lane_curve(self, img, display = False):

    # Convertion of the image in a black and white one that highlights the path
    img_thtresh = thresholding(img)

    # Projection of the image in a plane containing only the area of interest
    wT = img.shape[1]
    hT = img.shape[0]
    points = getAreaOfInterest(wT, hT)

    imgWarp = warpImage(img_thtresh, points, wT, hT, inv = False)

    # Where the center of the image is
    if (display):
      imageCenterColumn, imgCenterHist = getHistogram(imgWarp , display, minPercentage = 0.9, region = 1)
    else:
      imageCenterColumn = getHistogram(imgWarp , display, minPercentage = 0.9, region = 1)

    # Where the center of the path is
    if (display):
      pathCenterColumn, imgPathHist = getHistogram(imgWarp , display, minPercentage = 0.5, region = 4)
    else:
      pathCenterColumn = getHistogram(imgWarp , display, minPercentage = 0.5, region = 4)
       
    # Getting of the curve turn
    curvePreAverage = imageCenterColumn - pathCenterColumn 

    recentCurvesList.append(curvePreAverage) # Register of the curve turn prior averaging
    if len(recentCurvesList) > recentCurvesLength:
      recentCurvesList.pop(0)
    curvePostAverage = int(sum(recentCurvesList)/len(recentCurvesList)) # Averaging with the last turns

    # Displaying
    if (display):
       imgResult = getResultImage(img, wT, hT, imgWarp, points, curvePostAverage, 0)
       imgResult = drawPoints(imgResult, points)
       allImages = stackImages(0.7, [[img, img_thtresh, imgResult], [imgWarp, imgCenterHist, imgPathHist]])
       cv2.imshow("Camera View", allImages)

    print("Giro: " + str(curvePostAverage))

    return curvePostAverage

   
  def listener_callback(self, data):
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame = cv2.resize(current_frame, (800, 600))

    ammount_of_rotation = self.get_lane_curve(current_frame, display = True)

    msg = Twist()
    
    #Tomando un ajuste por minimos cuadrados
    if abs(ammount_of_rotation) <= 175:
       msg.linear.x = -(0.00028)*abs(ammount_of_rotation) + (0.116)
       msg.angular.z = -0.001 * ammount_of_rotation
    else:
      msg.linear.x = 0.0
      msg.angular.z = -0.001 * ammount_of_rotation
    self.cmd_vel_pub.publish(msg)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()