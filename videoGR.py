import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import os
import math
import serial
import time

#MAC: /dev/cu.usbserial-10
#LINUX: /dev/ttyACM2
arduino_port = '/dev/cu.usbserial-10' 
baud_rate = 9600

ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2) #Wait for Serial Port to Open

steering = None
flag = False

def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def gaussian_blur(img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def canny(img, low_threshold, high_threshold):
    return cv2.Canny(img, low_threshold, high_threshold)
    
def region_of_interest(img, vertices):
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    
    return masked_image

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    try:
        draw_lines(line_img, lines)
    except:
        print("\nError: hough_lines() Exception 1")
        pass

    return line_img

def carPosition(lLaneSlope, lLaneIntercept, rLaneSlope, rLaneIntercept, y):
    global x1
    global x2
    try:
        slope = lLaneSlope
        intercept = lLaneIntercept

        x1 = (y - intercept) / slope
        x1 = int(x1)

        slope = rLaneSlope
        intercept = rLaneIntercept
        x2 = (y - intercept) / slope
        x2 = int(x2)
    except Exception as ex:
        print("\ncarPosition() Exception:\n\t", ex)
        pass
        

def mean(numbers):
    return float(sum(numbers) / len(numbers))

def draw_lines(img, lines, color=[0, 255, 0], thickness=5):
    leftLaneSlopeArray = []
    leftLaneInterceptArray = []
    rightLaneSlopeArray = []
    rightLaneInterceptArray = []
    
    img_center = 940
    min_slope = 0.5
    max_slope = 100

    try:
        for line in lines:
            for x1,y1,x2,y2 in line:
                #PolyFit function of Numpy gives you slope and intercept
                slope, intercept = np.polyfit((x1,x2), (y1,y2), 1)
                lSlope = (y2 - y1) / (x2 - x1)
                line_midpoint_x = (x1 + x2) // 2

                if min_slope <= abs(lSlope) <= max_slope:
                    intercept = y1 - lSlope * x1

                    if (line_midpoint_x < img_center):
                        leftLaneSlopeArray += [slope]
                        leftLaneInterceptArray += [intercept]
                    else:
                        rightLaneSlopeArray += [slope]
                        rightLaneInterceptArray += [intercept]
    except:
        print("\nError: draw_lines() Exception 1")
        pass
    
    leftLaneSlopeArrayAverage = np.mean(leftLaneSlopeArray)
    leftLaneInterceptArrayAverage = np.mean(leftLaneInterceptArray)
    
    rightLaneSlopeArrayAverage = np.mean(rightLaneSlopeArray)
    rightLaneInterceptArrayAverage = np.mean(rightLaneInterceptArray)

    # Calculate Car Position
    try:
        carPosition(leftLaneSlopeArrayAverage,
                    leftLaneInterceptArrayAverage,
                    rightLaneSlopeArrayAverage,
                    rightLaneInterceptArrayAverage,
                    900)
    except:
        print("\nError: draw_lines() Exception 2")
        pass
    
    yMin = 700
    yMax = 1000

    try:
        x1Left = int(np.nan_to_num(float((yMin-leftLaneInterceptArrayAverage)/leftLaneSlopeArrayAverage)))
        x2Left = int(np.nan_to_num(float((yMax-leftLaneInterceptArrayAverage)/leftLaneSlopeArrayAverage)))
        cv2.line(img, (x1Left, yMin), (x2Left, yMax), color, 20)
    except:
        print("\nError: draw_lines() Exception 3")
        pass
    
    try:
        x1Right = int(np.nan_to_num(float((yMin-rightLaneInterceptArrayAverage)/rightLaneSlopeArrayAverage)))
        x2Right = int(np.nan_to_num(float((yMax-rightLaneInterceptArrayAverage)/rightLaneSlopeArrayAverage)))
        cv2.line(img, (x1Right, yMin), (x2Right, yMax), color, 20)
    except:
        print("\nError: draw_lines() Exception 4")
        pass

def weighted_img(img, initial_img, a=1, b=1., g=0.):
    return cv2.addWeighted(initial_img, a, img, b, g)

def carControl(pixDeviation):
    global flag
    global steering
    
    if flag is False:
        steering = "C"
        flag = True

    if pixDeviation > 30:
        control = "Steer: Right"
        # Steer Right
        if steering != "R":
            if steering == "L":
                ser.write(b'r')
                ser.write(b'r')
                ser.write(b'r')
                ser.write(b'r')
            else:
                ser.write(b'r')
                ser.write(b'r')
            
            steering = "R"
            print("\nTurning Right")
            print("Steering Position: " + steering)
            
    elif pixDeviation < -30:
        control = "Steer: Left"
        # Steer Left
        if steering != "L":
            if steering == "R":
                ser.write(b'l')
                ser.write(b'l')
                ser.write(b'l')
                ser.write(b'l')
            else:
                ser.write(b'l')
                ser.write(b'l')
            steering = "L"
            print("\nTurning Left")
            print("Steering Position: " + steering)
    else:
        control = "Steer: Straight"
        if steering != "C":
            if steering == "L":
                print("\nTurning Right to Center")
                ser.write(b'r')
                ser.write(b'r')
            else:
                print("\nTurning Left to Center")
                ser.write(b'l')
                ser.write(b'l')
            
            steering = "C"
            print("Steering Position: " + steering)
    
    return control

def finalMask(img):
    image = img
    
        # Lane Centering Guide Lines
    # Left Blue
    left1 = (440 - 240, 900)
    left2 = (440 + 240, 900)
    # Right Blue
    right1 = (1370 - 240, 900)
    right2 = (1370 + 240, 900)
    # Red Box
    boxLeft1 = ((440 - 240) - 100, 800)
    boxLeft2 = ((440 - 240) - 100, 1000)
    boxRight1 = ((1370 + 240) + 100, 800)
    boxRight2 = ((1370 + 240) + 100, 1000)
    # White Midline
    mid = ((905, 900 - 50), (905, 900 + 50))
    # Blue Midlines
    midpoint_markerR = ((1370, 900 - 50), (1370, 900 + 50))
    midpoint_markerL = ((440, 900 - 50), (440, 900 + 50))
    
    try:
        laneMarkerL = ((x1, 9900 - 30), (x1, 9900 + 30))
        laneMarkerR = ((x2, 9900 - 30), (x2, 9900 + 30))
    
        # Draw the red line
        cv2.line(image, left1, left2, (0, 0, 255), 3)
        cv2.line(image, right1, right2, (0, 0, 255), 3)

            # Draw the shorter line for the midpoint marker
        cv2.line(image, midpoint_markerL[0], midpoint_markerL[1], (0, 0, 255), 3)
        cv2.line(image, midpoint_markerR[0], midpoint_markerR[1], (0, 0, 255), 3)
        cv2.line(image, mid[0], mid[1], (255, 255, 255), 2)
        
        try:
            cv2.line(image, laneMarkerL[0], laneMarkerL[1], (0, 255, 0), 3)
            cv2.line(image, laneMarkerR[0], laneMarkerR[1], (0, 255, 0), 3)
        
            # Box
            cv2.line(image, boxLeft1, boxLeft2, (255, 0, 0), 3)
            cv2.line(image, boxRight1, boxRight2, (255, 0, 0), 3)
            cv2.line(image, boxLeft1, boxRight1, (255, 0, 0), 3)
            cv2.line(image, boxLeft2, boxRight2, (255, 0, 0), 3)

            try:
                # Add the text to the image
                midL = "Left Midpoint = 400"
                midR = "Right Midpoint = 1200"

                leftX = f"Left X = {x2:.2f}"
                left = int(x2 - 440)
                lDev = f"Left Deviation = {left:.2f}"
                
                
                rightX = f"Right X = {x1:.2f}"
                right = int(x1 - 1370)
                rDev = f"Right Deviation = {right:.2f}"

                average = (left + right) / 2
                avgDev = f"Avg Deviation = {average:.2f}"

                # Car Control
                steeringInput = carControl(average)

                    # Draw Steering Line
                average = int(average)
                mid = ((905 + average, 900900 - 30), (905 + average, 900900 + 30))
                cv2.line(img, (905 + average, 900900), (905, 900900), (0, 255, 0), 2)
                cv2.line(img, mid[0], mid[1], (0, 255, 0), 2)

                    # Define the font and other text properties
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1
                font_color = (255, 255, 255)  # White color
                font_thickness = 2

                cv2.putText(img, midL, (100, 200), font, font_scale, (0, 0, 255), font_thickness)
                cv2.putText(img, midR, (1400, 200), font, font_scale, (0, 0, 255), font_thickness)

                cv2.putText(img, leftX, (100, 250), font, font_scale, (0, 255, 0), font_thickness)
                cv2.putText(img, rightX, (1400, 250), font, font_scale, (0, 255, 0), font_thickness)

                cv2.putText(img, lDev, (100, 300), font, font_scale, (0, 0, 255), font_thickness)
                cv2.putText(img, rDev, (1400, 300), font, font_scale, (0, 0, 255), font_thickness)

                cv2.putText(img, avgDev, (750, 600), font, font_scale, font_color, font_thickness)
                cv2.putText(img, steeringInput, (750, 650), font, font_scale, font_color, font_thickness)
            except Exception as ex:
                print("\nfinalMask() Exception 3:\n\t", ex)
                pass
        except Exception as ex:
            print("\nfinalMask() Exception 2:\n\t", ex)
            pass
    except Exception as ex:
        print("\nfinalMask() Exception 1:\n\t", ex)
        pass

    return img
    
def process_image(image):
    gray_image = grayscale(image)

    kernel_size = 5
    blurred_gray_image = gaussian_blur(gray_image,kernel_size)

    low_threshold = 180
    high_threshold = 280

    cannyEdge_image = canny(blurred_gray_image, low_threshold, high_threshold)

    # xMin = 0
    # yMin = 0
    # xMax = image.shape[1]
    # yMax = image.shape[0]
    # yLen = 500    

    vertices = np.array([[(0, 1080), (1920, 1080), (1720, 700), (200, 700)]], dtype=np.int32)

    masked_image = region_of_interest(cannyEdge_image, vertices)

    rho = 1
    theta = np.pi/180
    threshold = 100
    min_line_length = 40
    max_line_gap = 20

    houghLine_image = hough_lines(masked_image, rho, theta, threshold, min_line_length, max_line_gap)

    # plt.imshow(gray_image)
    # plt.show()

    # cv2.imshow("result", gray_image)
    # cv2.waitKey(0)
    
    result = weighted_img(houghLine_image, image)

    # Add text and lane center lines
    complete_img = finalMask(result)

    return complete_img

# cap = cv2.VideoCapture("test.mp4")
cap = cv2.VideoCapture(0)
ser.write(b'f')
# _, frame = cap.read()

# final_image = process_image(frame)

# plt.imshow(final_image)
# plt.show()

# '''
while (cap.isOpened()):
    _, frame = cap.read()

    try:
        final_image = process_image(frame)
    except:
        final_image = frame

    try:
        frameRS = cv2.resize(final_image, (1600, 900))
    except:
        frameRs = final_image
        
    cv2.imshow('result', frameRS)
    if cv2.waitKey(10) & 0xFF == ord('q'):
           ser.write(b's')
           ser.write(b'c')
           break
    
    # plt.imshow(final_image)
    # plt.show(block=False)
    # plt.pause(0.5)
    # plt.close()

cap.release()
cv2.destroyAllWindows()

print("\n\n\t\t", steering)

# ser.write(b'S')
# ser.write(b'C')
# ser.close()
#'''
