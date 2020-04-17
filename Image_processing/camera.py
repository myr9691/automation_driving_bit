import cv2

cap = cv2.VideoCapture(0)

while True:
    retval, img = cap.read()
    if not retval:
        break
    cv2.imshow('camera', img)
    cv2.waitKey(25)
    if key == 27:
        break
if cap.isOpened():
    cap.release()
cv2.destroyAllWindows()
    
