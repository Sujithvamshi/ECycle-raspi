from ast import While
cap = cv2.VideoCapture(0)
while True:
    ret,frame = cap.read()
    if not ret:
        break
    cv2.imshow("frame",frame)
    if cv2.waitKey(1) == ord("q"):
        break
cv2.destroyAllWindows()