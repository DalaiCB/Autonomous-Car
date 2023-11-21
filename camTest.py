import cv2
import matplotlib.pyplot as plt

# Capture video from the default camera (webcam)
cap = cv2.VideoCapture(0)

# Check if the webcam is opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Check if the frame was read successfully
    if not ret:
        print("Error: Could not read frame.")
        break

    plt.imshow(frame)
    plt.show()

    # Display the frame
    cv2.imshow("Camera Feed", frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(0) & 0xFF == ord('q'):
        break

# Release the capture object and close the OpenCV window
cap.release()
cv2.destroyAllWindows()
