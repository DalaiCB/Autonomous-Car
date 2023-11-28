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
try:
    arduino_port = '/dev/cu.usbserial-110' 
    baud_rate = 9600

    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2) #Wait for Serial Port to Open
except Exception as ex:
    print("\n\tArduino Port Error: ", ex)
    pass

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

                    if (line_midpoint_x < img_center - 100):
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

                                        # SINGLE LANE FUNCTIONALITY TEST
    if np.isnan(leftLaneInterceptArrayAverage) or np.isnan(leftLaneSlopeArrayAverage):
        leftLaneSlopeArrayAverage = 0
        leftLaneInterceptArrayAverage = 0
    if np.isnan(rightLaneSlopeArrayAverage) or np.isnan(rightLaneInterceptArrayAverage):
        rightLaneSlopeArrayAverage = 0
        rightLaneInterceptArrayAverage = 0

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
    
    yMin = 800
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
            # if steering == "L":
            #     ser.write(b'r')
            #     ser.write(b'r')
            #     ser.write(b'r')
            #     ser.write(b'r')
            # else:
            #     ser.write(b'r')
            #     ser.write(b'r')
            
            steering = "R"
            print("\nTurning Right")
            print("Steering Position: " + steering)
            
    elif pixDeviation < -30:
        control = "Steer: Left"
        # Steer Left
        if steering != "L":
            # if steering == "R":
            #     ser.write(b'l')
            #     ser.write(b'l')
            #     ser.write(b'l')
            #     ser.write(b'l')
            # else:
            #     ser.write(b'l')
            #     ser.write(b'l')
                
            steering = "L"
            print("\nTurning Left")
            print("Steering Position: " + steering)
    else:
        control = "Steer: Straight"
        if steering != "C":
            # if steering == "L":
            #     print("\nTurning Right to Center")
            #     ser.write(b'r')
            #     ser.write(b'r')
            # else:
            #     print("\nTurning Left to Center")
            #     ser.write(b'l')
            #     ser.write(b'l')
            
            steering = "C"
            print("Steering Position: " + steering)
    
    return control

def finalMask(img):
    image = img
    
    height = image.shape[0]
    width = image.shape[1]

    if height == 1080 and width == 1620:
        # Line Values
        y = 900
        leftMidX = 380
        rightMidX = 1300
        midX = 840
        line_x_offset = 200
        # Text Values
        leftText_X = 100
        rightText_X = 1100
        text_Y = 200
        # Colors
        white = (255, 255, 255)
        red = (255, 0 , 0)
        green = (0, 255, 0)
        blue = (0, 0, 255)
    elif height == 1080 and width == 1920:
        # Line Values
        y = 900
        leftMidX = 515
        rightMidX = 1365
        midX = 905
        line_x_offset = 240
        # Text Values
        leftText_X = 100
        rightText_X = 1400
        text_Y = 200
        # Colors
        white = (255, 255, 255)
        red = (255, 0 , 0)
        green = (0, 255, 0)
        blue = (0, 0, 255)

    # Left
    left1 = (leftMidX - line_x_offset, y)
    left2 = (leftMidX + line_x_offset, y)
    # Right
    right1 = (rightMidX - line_x_offset, y)
    right2 = (rightMidX + line_x_offset, y)
    # Box
    boxLeft1 = ((leftMidX - line_x_offset) - 100, y - 100)
    boxLeft2 = ((leftMidX - line_x_offset) - 100, y + 100)
    boxRight1 = ((rightMidX + line_x_offset) + 100, y - 100)
    boxRight2 = ((rightMidX + line_x_offset) + 100, y + 100)
    # White Midline
    mid = ((midX, y - 50), (midX, y + 50))
    # Blue Midlines
    midpoint_markerR = ((rightMidX, y - 50), (rightMidX, y + 50))
    midpoint_markerL = ((leftMidX, y - 50), (leftMidX, y + 50))
    
    try:
        laneMarkerL = ((x1, y - 30), (x1, y + 30))
        laneMarkerR = ((x2, y - 30), (x2, y + 30))
    
        # Draw the red line
        cv2.line(image, left1, left2, blue, 3)
        cv2.line(image, right1, right2, blue, 3)

            # Draw the shorter line for the midpoint marker
        cv2.line(image, midpoint_markerL[0], midpoint_markerL[1], blue, 3)
        cv2.line(image, midpoint_markerR[0], midpoint_markerR[1], blue, 3)
        cv2.line(image, mid[0], mid[1], white, 2)
        
        try:
            cv2.line(image, laneMarkerL[0], laneMarkerL[1], green, 3)
            cv2.line(image, laneMarkerR[0], laneMarkerR[1], green, 3)
        
            # Box
            cv2.line(image, boxLeft1, boxLeft2, red, 3)
            cv2.line(image, boxRight1, boxRight2, red, 3)
            cv2.line(image, boxLeft1, boxRight1, red, 3)
            cv2.line(image, boxLeft2, boxRight2, red, 3)

            try:
                # Add the text to the image
                midL = "Left Midpoint = 515"
                midR = "Right Midpoint = 1365"

                leftX = f"Left X = {x2:.2f}"
                left = int(x1 - leftMidX)
                lDev = f"Left Deviation = {left:.2f}"
                
                
                rightX = f"Right X = {x1:.2f}"
                right = int(x2 - rightMidX)
                rDev = f"Right Deviation = {right:.2f}"

                average = (left + right) / 2
                avgDev = f"Avg Deviation = {average:.2f}"

                # Car Control
                steeringInput = carControl(average)

                    # Draw Steering Line
                average = int(average)
                mid = ((midX + average, y - 30), (midX + average, y + 30))
                cv2.line(img, (midX + average, y), (midX, y), green, 2)
                cv2.line(img, mid[0], mid[1], green, 2)

                    # Define the font and other text properties
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1
                font_thickness = 2

                cv2.putText(img, midL, (leftText_X , text_Y), font, font_scale, blue, font_thickness)
                cv2.putText(img, midR, (rightText_X, text_Y), font, font_scale, blue, font_thickness)

                cv2.putText(img, leftX, (leftText_X , text_Y + 50), font, font_scale, green, font_thickness)
                cv2.putText(img, rightX, (rightText_X, text_Y + 50), font, font_scale, green, font_thickness)

                cv2.putText(img, lDev, (leftText_X , text_Y + 100), font, font_scale, blue, font_thickness)
                cv2.putText(img, rDev, (rightText_X, text_Y + 100), font, font_scale, blue, font_thickness)

                cv2.putText(img, avgDev, ((midX - 150), y - 300), font, font_scale, white, font_thickness)
                cv2.putText(img, steeringInput, ((midX -150), y - 350), font, font_scale, white, font_thickness)
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

    height = image.shape[0]
    width = image.shape[1]

    if height == 1080 and width == 1620:
        vertices = np.array([[(180, 1000), (1500, 1000), (1500, 800), (180, 800)]], dtype=np.int32)
    elif height == 1080 and width == 1920:
        vertices = np.array([[(300, 1000), (1600, 1000), (1480, 700), (515, 700)]], dtype=np.int32)

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

cap = cv2.VideoCapture("test.mp4")
# ser.write(b'f')

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
        #    ser.write(b's')
        #    ser.close()
           break

# if steering == "L":
#     ser.write(b'r')
#     ser.write(b'r')
# elif steering == "R":
#     ser.write(b'l')
#     ser.write(b'l')

# ser.write(b'c')

cap.release()
cv2.destroyAllWindows()
#'''
