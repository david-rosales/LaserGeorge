import numpy as np
import cv2
import sys
import socket

lower = np.array([0, 60, 100], dtype = "uint8")
upper = np.array([20, 255, 255], dtype = "uint8")

maxTime = 5
maxDist = 300
minDist = 0.1
pointsWindow = 5
DEFAULT_PORT = 65432   

def createSkinMask(frame):
  frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  skinMask = cv2.inRange(frame_hsv, lower, upper)
  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
  #skinMask = cv2.erode(skinMask, kernel, iterations = 2)
  skinMask = cv2.dilate(skinMask, kernel, iterations = 2)
  skinMask = cv2.GaussianBlur(skinMask, (5, 5), 0)
  return skinMask

def getLargestContour(skinMask):
  contours, _ = cv2.findContours(skinMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  if len(contours) > 0:
    contour = sorted(contours, key = lambda x: cv2.contourArea(x), reverse=True)[0]
    return contour
  return None

def getConvexHull(contour):
  return cv2.convexHull(contour)

def pointsInSameGroup(points):
  # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
  points = np.float32(points)
  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
  ret,label,center=cv2.kmeans(points,4,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
  return label.flatten().tolist(), center

def distanceBetween(point1, point2):
  return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)**0.5

def averagePoint(points):
  x_tot = 0
  y_tot = 0
  N = len(points)
  for i in range(N):
    x = points[i][0]
    y = points[i][1]
    x_tot += x
    y_tot += y
  return (int(x_tot * 1.0 / N), int(y_tot * 1.0 / N))

def connectToSocket(host, port=DEFAULT_PORT):
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect((host, port))
  return s

def captureCamera(cam1, cam2, s):
  cap1 = cv2.VideoCapture(cam1)
  cap2 = cv2.VideoCapture(cam2)
  width = cap2.get(cv2.CAP_PROP_FRAME_WIDTH)/4
  height = cap1.get(cv2.CAP_PROP_FRAME_HEIGHT)/4

  last_points1 = []
  last_no_change_counter1 = 1000

  last_points2 = []
  last_no_change_counter2 = 1000

  last_sent = [-1, -1]

  while(True):
    last_no_change_counter1 += 1
    last_no_change_counter2 += 1
    _, frame1 = cap1.read()
    _, frame2 = cap2.read()
    if frame1 is None or frame2 is None:
      #cv2.imshow('frame', frame)
      if last_sent != [-1, -1]:
        coord = "-1.0, -1.0"
        s.sendall(coord.encode())
        print(coord)
      last_sent = [-1, -1]
      continue

    frame1 = cv2.resize(frame1, (0,0), fx=0.25, fy=0.25)

    frame2 = cv2.resize(frame2, (0,0), fx=0.25, fy=0.25)
    frame2 = cv2.flip(frame2, -1)

    skinMask1 = createSkinMask(frame1)
    skinMask2 = createSkinMask(frame2)
    contour1 = getLargestContour(skinMask1)
    contour2 = getLargestContour(skinMask2)

    if contour1 is None or contour2 is None:
      #cv2.imshow('frame', frame)
      if last_sent != [-1, -1]:
        coord = "-1.0, -1.0"
        s.sendall(coord.encode())
        print(coord)
      last_sent = [-1, -1]
      continue

    if cv2.contourArea(contour1) < 1000 or cv2.contourArea(contour2) < 1000:
      #cv2.imshow('frame', frame)
      if last_sent != [-1, -1]:
        coord = "-1.0, -1.0"
        s.sendall(coord.encode())
        print(coord)
      last_sent = [-1, -1]
      continue

    convexHullPoints1 = getConvexHull(contour1)
    convexHullPoints2 = getConvexHull(contour2)

    labels1, centers1 = pointsInSameGroup(convexHullPoints1)
    labels2, centers2 = pointsInSameGroup(convexHullPoints2)
    
    val1 = max(set(labels1), key=labels1.count)
    val2 = max(set(labels2), key=labels2.count)
    #closest = min(centers.tolist(), key=lambda x: distanceBetween(x, last_point))
    left = min(centers1.tolist(), key=lambda center: center[0])
    center1 = left

    bottom = min(centers2.tolist(), key=lambda center: center[1])
    center2 = bottom
    #center1 = centers1[val1]
    #center2 = centers2[val2]
    #print(center)

    if len(last_points1) > 0:
      prev_avg1 = averagePoint(last_points1)
      if last_no_change_counter1 > maxTime:
        last_no_change_counter1 = 0
        last_points1.append(center1)
      elif distanceBetween(center1, prev_avg1) < maxDist:
        last_no_change_counter1 = 0
        last_points1.append(center1)
      if len(last_points1) > pointsWindow:
        last_points1.pop(0)
    else:
      last_points1.append(center1)

    if len(last_points2) > 0:
      prev_avg2 = averagePoint(last_points2)
      if last_no_change_counter2 > maxTime:
        last_no_change_counter2 = 0
        last_points2.append(center2)
      elif distanceBetween(center2, prev_avg2) < maxDist:
        last_no_change_counter2 = 0
        last_points2.append(center2)
      if len(last_points2) > pointsWindow:
        last_points2.pop(0)
    else:
      last_points2.append(center2)

    last_point1 = averagePoint(last_points1)
    last_point2 = averagePoint(last_points2)

    new_point = [last_point2[0]/width, 1 - last_point1[1]/height]

    if distanceBetween(last_sent, new_point) > minDist:
      last_sent = new_point
      coord = str(last_sent[0]) + "," + str(last_sent[1])
      s.sendall(coord.encode())
      print(coord)

    #print(last_point2[0]/width, 1 - last_point1[1]/height)
    #print(1 - last_point1[1]/height)

    if "show" in sys.argv:
      cv2.circle(frame1, (int(last_point1[0]), int(last_point1[1])), 10, (0, 255, 255), -1)
      cv2.circle(frame2, (int(last_point2[0]), int(last_point2[1])), 10, (0, 255, 255), -1)
      cv2.drawContours(frame1, [contour1], -1, (0,255,0), 1)
      cv2.drawContours(frame2, [contour2], -1, (0,255,0), 1)
      cv2.imshow('side', frame1)
      cv2.imshow('top', frame2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  cap1.release()
  cap2.release()


if __name__ == "__main__":
  # DO STUFF
  args = sys.argv
  print(args)
  if len(args) >= 4:
    topCamera = args[1]
    sideCamera = args[2]
    serverIP = args[3]
    print("Starting...")
    s = connectToSocket(serverIP)
    captureCamera(int(topCamera), int(sideCamera), s)
    print("Ending...")
  else:
    print("Usage: python FingerPointer.py 0 1 [show]")