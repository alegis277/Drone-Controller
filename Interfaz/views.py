from django.shortcuts import render,redirect,render_to_response
from django.http import HttpResponse,StreamingHttpResponse
from django.http import Http404
import simplejson as json
import numpy as np
import time
import cv2


# Create your views here.

def index(request):

	return render(request, 'index.html')

# class VideoCamera(object):
# 	def __init__(self):
# 		self.video = cv2.VideoCapture(0)
# 	def __del__(self):
# 		self.video.release()

# 	def get_frame(self):
# 		_,image = self.video.read()

# 		lower = np.array([0, 30, 10])
# 		upper = np.array([200, 255, 204])
# 		image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #Convierte a HSV
# 		mask1 = cv2.inRange(image, lower, upper)

# 		mask1 = cv2.resize(mask1, (800,480))


# 		_,jpeg = cv2.imencode('.jpg',mask1)
# 		return jpeg.tobytes()

# def gen(camera):
# 	while True:
# 		print(time.time())
# 		frame = camera.get_frame()
# 		yield(b'--frame\r\n'
# 		b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

# def img(request):

# 	try:
# 		return StreamingHttpResponse(gen(VideoCamera()),content_type="multipart/x-mixed-replace;boundary=frame")
# 	except HttpResponseServerError as e:
# 		print("aborted")

