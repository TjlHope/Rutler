#!/usr/bin/env python

import cv
from collections import namedtuple

Point = namedtuple('Point', ('x', 'y'))
Size = namedtuple('Size', ('width', 'height'))
Rect = namedtuple('Rect', ('x', 'y', 'width', 'height'))

def init_display(name='Camera'):
    cv.NamedWindow(name, cv.WINDOW_AUTOSIZE)
    return name

def init_camera():
    dev = 0
    camera = cv.CaptureFromCAM(dev)
    #cv.SetCaptureProperty(camera, cv.CV_CAP_PROP_FRAME_WIDTH, 640)
    #cv.SetCaptureProperty(camera, cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
    if not camera:
        print "Error initiating Camera."
        return None
    else:
        return camera

def detect(image):
    image_size = cv.GetSize(image)
    print "image size: {0}".format(image_size)

    # create grayscale version
    print "create grayscale version"
    grayscale = cv.CreateImage(image_size, 8, 1)
    cv.CvtColor(image, grayscale, cv.CV_BGR2GRAY)

    # equalize histogram
    print "equalise histogram"
    cv.EqualizeHist(grayscale, grayscale)

    # detect objects
    print "load face training"
    cascade = cv.Load('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')
    #cascade = cv.Load('/usr/share/opencv/haarcascades/haarcascade_profileface.xml')
    print "perform detect"
    #faces = cv.HaarDetectObjects(grayscale, cascade, cv.CreateMemStorage())
    faces = cv.HaarDetectObjects(grayscale, cascade, cv.CreateMemStorage(),
                                 1.2, 2, cv.CV_HAAR_DO_CANNY_PRUNING, Size(50, 50))
 

    # draw rectangles round faces
    if faces:
        for f in faces:
            i = Rect(*f[0])
            cv.Rectangle(image, Point(int(i.x), int(i.y)),
                         Point(int(i.x + i.width), int(i.y + i.height)),
                         cv.RGB(0, 255, 0), 3, 8, 0)
    return image

if __name__ == '__main__':
    print "init display"
    display = init_display()
    print "init camera"
    camera = init_camera()
    while True:
	print "get frame"
        frame = cv.QueryFrame(camera)
        if frame is None:
            break
	print "flip frame"
        cv.Flip(frame, frame, 1)
	print "detect faces"
        frame = detect(frame)
	print "show image"
        cv.ShowImage(display, frame)
        if cv.WaitKey(10) == 0x1b:
            print "key pressed, exiting..."
            break

