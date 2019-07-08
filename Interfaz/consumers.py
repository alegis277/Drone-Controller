from asgiref.sync import async_to_sync
from channels.generic.websocket import WebsocketConsumer
import channels.layers
import simplejson as json
import time
import serial
from MultiWiiPy3 import MultiWiiPy3
import threading
import cv2
import numpy as np
import pygame
import csv
from simplejson import encoder
import warnings
warnings.filterwarnings("ignore")
encoder.FLOAT_REPR = lambda o: format(o, '.3f')
channel_layer = channels.layers.get_channel_layer()




NO_CONNECTION = 0
AUTO_CONNECTION = 1
MANUAL_CONNECTION = 2

droneConnection = MANUAL_CONNECTION



Z_CONTROLLER = 0
PITCH_ROLL_CONTROLLER = 1
BOTH_CONTROLLERS = 2

droneControllers = Z_CONTROLLER


UPPER_CAMERA = 0



global paintDrone, paintObject, paintIntercept, x_drone, y_drone, z_drone, heading_drone, x_object, y_object, z_object, x_intercept, y_intercept, z_intercept, dronePitch, droneRoll, droneYaw, droneThrottle, Kp, Ki, Kd, Kp_adapt, Ki_adapt, Kd_adapt, p_adapt_cst, i_adapt_cst, d_adapt_cst, Batt
global Kp_PR, Ki_PR, Kd_PR, Kp_adapt_PR, Ki_adapt_PR, Kd_adapt_PR, p_adapt_cst_PR, i_adapt_cst_PR, d_adapt_cst_PR


paintDrone = True
paintObject = False
paintIntercept = True
x_drone = -99999
y_drone = -99999
z_drone = -99999
heading_drone = -99999
x_object = -99999
y_object = -99999
z_object = -99999
x_intercept = -99999
y_intercept = -99999
z_intercept = -99999

dronePitch = 1500
droneRoll = 1500
droneYaw = 1500
droneThrottle = 1000


Kp = 0
Ki = 0
Kd = 0
Kp_adapt = 0
Ki_adapt = 0
Kd_adapt = 0
p_adapt_cst = 0
i_adapt_cst = 0
d_adapt_cst = 0

Kp_PR = 0
Ki_PR = 0
Kd_PR = 0
Kp_adapt_PR = 0
Ki_adapt_PR = 0
Kd_adapt_PR = 0
p_adapt_cst_PR = 0
i_adapt_cst_PR = 0
d_adapt_cst_PR = 0
ADAPTATIVE_THRESHOLD = 150


Batt = -99999

### OTHER VARIABLES ###

global inputEnabled, updatePX_CM_constant, PX_CM_constant, CM_BETWEEN_POINTS

inputEnabled = False
updatePX_CM_constant = False
PX_CM_constant = 2 #cm/px
CM_BETWEEN_POINTS = 10 #cm

### TYPES ###

CHANGE_CONTROLLER = 0
EMERGENCY = 1
REQUEST_CONTROLER_DATA = 2;
UPDATE_CONTROLLER = 3;

### MESSAGES ###

PID = 1
PID_ADAPTATIVE = 2

### STATES ###

INITIALIZING = 0
ERROR = 1
READY_TO_CATCH = 2
CATCHING = 3
EMERGENCY_STOP = 4
MANUAL_MODE = 5

global state
state = INITIALIZING


global currentController
currentController = PID


### DEBUGING ###

global lastHeights, lastControlActions, lastDebugTimes
lastHeights = []
lastControlActions = []
lastDebugTimes = []


class bgUpdate(WebsocketConsumer):


	def connect(self):

		self.room_name = 'r'+str(time.time())
		self.room_group_name = 'bgUpdateConsumers'

		# Join room group
		async_to_sync(self.channel_layer.group_add)(
			self.room_group_name,
			self.channel_name
		)

		self.accept()

	def disconnect(self, close_code):
		async_to_sync(self.channel_layer.group_discard)(
			self.room_group_name,
			self.channel_name
		)

	def receive(self, text_data):
		global currentController, state, Kp, Ki, Kd, Kp_adapt, Ki_adapt, Kd_adapt, p_adapt_cst, i_adapt_cst, d_adapt_cst, inputEnabled
		global Kp_PR, Ki_PR, Kd_PR, Kp_adapt_PR, Ki_adapt_PR, Kd_adapt_PR, p_adapt_cst_PR, i_adapt_cst_PR, d_adapt_cst_PR
		text_data_json = json.loads(text_data)
		message = text_data_json['message']
		typeMsg = text_data_json['type']
		
		if typeMsg == CHANGE_CONTROLLER:
			currentController = message
		elif typeMsg == EMERGENCY:
			state = EMERGENCY_STOP if state!=EMERGENCY_STOP else INITIALIZING
			if state == EMERGENCY_STOP:
				inputEnabled = False
		elif typeMsg == REQUEST_CONTROLER_DATA:
			toSend = {}
			toSend['config_req'] = True
			toSend['controller'] = message

			if message == PID:
				toSend['KP'] = Kp
				toSend['KI'] = Ki
				toSend['KD'] = Kd
				toSend['KP_PR'] = Kp_PR
				toSend['KI_PR'] = Ki_PR
				toSend['KD_PR'] = Kd_PR
			elif message == PID_ADAPTATIVE:
				toSend['KP'] = Kp_adapt
				toSend['KI'] = Ki_adapt
				toSend['KD'] = Kd_adapt
				toSend['KP_cte'] = p_adapt_cst
				toSend['KI_cte'] = i_adapt_cst
				toSend['KD_cte'] = d_adapt_cst
				toSend['KP_PR'] = Kp_adapt_PR
				toSend['KI_PR'] = Ki_adapt_PR
				toSend['KD_PR'] = Kd_adapt_PR
				toSend['KP_cte_PR'] = p_adapt_cst_PR
				toSend['KI_cte_PR'] = i_adapt_cst_PR
				toSend['KD_cte_PR'] = d_adapt_cst_PR

			self.send(text_data=json.dumps(toSend))

		elif typeMsg == UPDATE_CONTROLLER:
			if message == PID:
				Kp = text_data_json['KP']
				Ki = text_data_json['KI']
				Kd = text_data_json['KD']
				Kp_PR = text_data_json['KP_PR']
				Ki_PR = text_data_json['KI_PR']
				Kd_PR = text_data_json['KD_PR']
			elif message == PID_ADAPTATIVE:
				Kp_adapt = text_data_json['KP']
				Ki_adapt = text_data_json['KI']
				Kd_adapt = text_data_json['KD']
				p_adapt_cst = text_data_json['KP_cte']
				i_adapt_cst = text_data_json['KI_cte']
				d_adapt_cst = text_data_json['KD_cte']
				Kp_adapt_PR = text_data_json['KP_PR']
				Ki_adapt_PR = text_data_json['KI_PR']
				Kd_adapt_PR = text_data_json['KD_PR']
				p_adapt_cst_PR = text_data_json['KP_cte_PR']
				i_adapt_cst_PR = text_data_json['KI_cte_PR']
				d_adapt_cst_PR = text_data_json['KD_cte_PR']

			

	def updateGUI(self, event):

		self.send(text_data=json.dumps(event))

def threadGUIupdate():
	global paintDrone, paintObject, paintIntercept, x_drone, y_drone, z_drone, heading_drone, x_object, y_object, z_object, x_intercept, y_intercept, z_intercept, currentController, dronePitch, droneRoll, droneYaw, droneThrottle, Kp, Ki, Kd, Batt, state, Kp_adapt, Ki_adapt, Kd_adapt, p_adapt_cst, i_adapt_cst, d_adapt_cst
	global Kp_PR, Ki_PR, Kd_PR, Kp_adapt_PR, Ki_adapt_PR, Kd_adapt_PR, p_adapt_cst_PR, i_adapt_cst_PR, d_adapt_cst_PR
	global PX_CM_constant

	while True:

		options = {}

		options['paintDrone'] = paintDrone
		options['paintObject'] = paintObject
		options['paintIntercept'] = paintIntercept

		options['x_drone'] = x_drone if x_drone==-99999 else PX_CM_constant*x_drone
		options['y_drone'] = y_drone if y_drone==-99999 else PX_CM_constant*y_drone
		options['z_drone'] = z_drone
		options['h_drone'] = heading_drone

		options['x_object'] = x_object if x_object==-99999 else PX_CM_constant*x_object
		options['y_object'] = y_object if y_object==-99999 else PX_CM_constant*y_object
		options['z_object'] = z_object

		options['x_intercept'] = x_intercept if x_intercept==-99999 else PX_CM_constant*x_intercept
		options['y_intercept'] = y_intercept if y_intercept==-99999 else PX_CM_constant*y_intercept
		options['z_intercept'] = z_intercept

		options['currentController'] = currentController
		options['dronePitch'] = dronePitch 
		options['droneRoll'] = droneRoll 
		options['droneYaw'] = droneYaw 
		options['droneThrottle'] = droneThrottle

		if currentController == PID:
			options['Kp'] = Kp 
			options['Ki'] = Ki 
			options['Kd'] = Kd
			options['Kp_PR'] = Kp_PR 
			options['Ki_PR'] = Ki_PR 
			options['Kd_PR'] = Kd_PR
		elif currentController == PID_ADAPTATIVE:
			options['Kp'] = Kp_adapt
			options['Ki'] = Ki_adapt
			options['Kd'] = Kd_adapt
			options['Kp_PR'] = Kp_adapt_PR
			options['Ki_PR'] = Ki_adapt_PR
			options['Kd_PR'] = Kd_adapt_PR



		options['Batt'] = Batt 

		options['state'] = state

		options['type'] = 'updateGUI'

		keys = [key for key in options if options[key]==-99999]
		for key in keys:
			options[key] = "Indeterminate"

		async_to_sync(channel_layer.group_send)('bgUpdateConsumers', options)

		time.sleep(100E-3)


threading.Thread(target=threadGUIupdate, args=()).start()









### CONTROL MODES ###




### JOYSTICK INPUT FUNCTION ###
def inputJoystick(clock, my_joystick):
	global droneThrottle, droneRoll, dronePitch, droneYaw, inputEnabled

	while True:
		for event in pygame.event.get():

			if event.type == pygame.JOYBUTTONDOWN:
				if event.button == 2:
					inputEnabled = not inputEnabled
			else:
				if inputEnabled:

					if droneConnection == MANUAL_CONNECTION:
						droneThrottle = int( 2000 - 500*( 1+my_joystick.get_axis(3) ) )

					roll_t = int( 1500 + 100*( my_joystick.get_axis(0) ) )
					pitch_t = int( 1500 - 100*( my_joystick.get_axis(1) ) )
					yaw_t = int( 1500 + 100*( my_joystick.get_axis(2) ) )

					droneRoll = roll_t if np.abs(roll_t-1500)>10 else 1500
					dronePitch = pitch_t if np.abs(pitch_t-1500)>10 else 1500
					droneYaw = yaw_t if np.abs(yaw_t-1500)>10 else 1500



			clock.tick(20)

### MANUAL CONTROL THREAD ###
def ThreadManual():

	try:
		pygame.init()
		print("Joysticks encontrados: ", pygame.joystick.get_count())
		my_joystick = pygame.joystick.Joystick(0)
		my_joystick.init()
		clock = pygame.time.Clock()
		inputJoystick(clock, my_joystick)
	except Exception as error:
		raise error
		print("\033[1;31mJoystick no encontrado \033[0;0m")

### DRONE TELEOPERATION ###
def ThreadDroneNode():

	global droneThrottle, droneRoll, dronePitch, droneYaw, inputEnabled
	global z_drone, Batt, state

	serialPort = "/dev/tty.HC-05-DevB"
	#serialPort = "/dev/tty.usbmodem376D355931361"
	board = MultiWiiPy3(serialPort)

	board.initArm()
	time.sleep(0.5)
	
	data0, data2 = None, None
	counter = 0

	# Test delay in z receive

	#lastReceivedZTime = time.time()

	while True:

		data0 = board.getData(MultiWiiPy3.MSP_SONAR) 
		if counter==100:
			time.sleep(60E-3)
			data2 = board.getData(MultiWiiPy3.MSP_ANALOG)
			counter = 0

		counter+=1


		if data0 != None and data0['RF_alt']>0:
			z_drone = data0['RF_alt']
		if data2 != None:
			Batt = data2['vbat']
			#print( (time.time() - lastReceivedZTime)*1000 ,'ms')
			#lastReceivedZTime = time.time()


		if state != EMERGENCY_STOP:
			if inputEnabled:
				board.move(droneThrottle, droneRoll, dronePitch, droneYaw)
			else:
				board.init2Arm()
			if droneConnection == MANUAL_CONNECTION:
				state = MANUAL_MODE
		else:
			board.init2Arm()


		time.sleep(40E-3)


### AUTOMATIC CONTROLLERS ###
global zControlActive, PRControlActive
zControlActive = False
PRControlActive = False


def PID_Controller_z():

	global zControlActive, PRControlActive
	global droneThrottle, Kp, Ki, Kd
	global z_intercept, z_drone

	global Kp_adapt, Ki_adapt, Kd_adapt
	global p_adapt_cst, i_adapt_cst, d_adapt_cst
	global currentController

	global lastHeights, lastControlActions, lastDebugTimes

	zError = 0
	zIntegralError = 0
	zDerivativeError = 0
	gConstant = 1620

	zFirstTime = True
	zCounterPID = 0
	lastErrorDerivative = z_intercept - z_drone
	lastTimeDerivative = time.time()

	print("Starting Z controller")

	if currentController == PID:
		Kp_modified = Kp
		Ki_modified = Ki
		Kd_modified = Kd
	elif currentController == PID_ADAPTATIVE:
		Kp_modified = Kp_adapt
		Ki_modified = Ki_adapt
		Kd_modified = Kd_adapt

	droneThrottle = 1550

	lastHeights = []
	lastControlActions = []
	lastDebugTimes = []
	startTime = time.time()

	time.sleep(2)

	while zControlActive:

		zError = z_intercept - z_drone

		

		if currentController == PID_ADAPTATIVE:
			Kp_modified = Kp_modified + zError*p_adapt_cst
			Ki_modified = Ki_modified + zError*i_adapt_cst
			Kd_modified = Kd_modified + zError*d_adapt_cst

		if not zFirstTime:
			zIntegralError += zError*( time.time() - lastTime )
			zIntegralError = np.clip(zIntegralError, -1000, 1000)

			if zCounterPID == 3:
				zDerivativeError = np.clip( ( (z_intercept - z_drone) - lastErrorDerivative ) / ( time.time() - lastTimeDerivative ), -1000, 1000)
				lastErrorDerivative = z_intercept - z_drone
				lastTimeDerivative = time.time()
				zCounterPID = 0


		droneThrottle = int(np.clip(int(Kp_modified*zError + Ki_modified*zIntegralError + Kd_modified*zDerivativeError + gConstant), 1000, 2000))

		lastHeights.append(z_drone)
		lastControlActions.append(droneThrottle)
		lastDebugTimes.append(time.time()-startTime)

		lastTime = time.time()
		zFirstTime = False
		zCounterPID += 1
		time.sleep(20E-3)

	with open('debug.csv', mode='w') as data_file:
		debug_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		debug_writer.writerow(['Time (s)', 'Height (cm)', 'Control Action (1000-2000)'])

		for i in range(len(lastDebugTimes)):
			debug_writer.writerow(['%.2f'%lastDebugTimes[i], str(lastHeights[i]), str(lastControlActions[i])])


def PID_Controller_Pitch_Roll():

	global zControlActive, PRControlActive
	global dronePitch, droneRoll, Kp_PR, Ki_PR, Kd_PR
	global x_intercept, y_intercept, x_drone, y_drone, heading_drone

	global Kp_adapt_PR, Ki_adapt_PR, Kd_adapt_PR
	global p_adapt_cst_PR, i_adapt_cst_PR, d_adapt_cst_PR
	global currentController


	xError = 0
	xIntegralError = 0
	xDerivativeError = 0
	yError = 0
	yIntegralError = 0
	yDerivativeError = 0
	xyConstant = 1500

	xyFirstTime = True
	xyCounterPID = 0
	xLastErrorDerivative = x_intercept - x_drone
	yLastErrorDerivative = y_intercept - y_drone
	xyLastTimeDerivative = time.time()

	if currentController == PID:
		Kp_modified_PR_x = Kp_PR
		Ki_modified_PR_x = Ki_PR
		Kd_modified_PR_x = Kd_PR
		Kp_modified_PR_y = Kp_PR
		Ki_modified_PR_y = Ki_PR
		Kd_modified_PR_y = Kd_PR
	elif currentController == PID_ADAPTATIVE:
		Kp_modified_PR_x = Kp_adapt_PR
		Ki_modified_PR_x = Ki_adapt_PR
		Kd_modified_PR_x = Kd_adapt_PR
		Kp_modified_PR_y = Kp_adapt_PR
		Ki_modified_PR_y = Ki_adapt_PR
		Kd_modified_PR_y = Kd_adapt_PR


	while PRControlActive:

		xError = x_intercept - x_drone
		yError = y_intercept - y_drone


		if currentController == PID_ADAPTATIVE:
			if np.abs(xError) > ADAPTATIVE_THRESHOLD:
				Kp_modified_PR_x = Kp_modified_PR_x + np.abs(xError)*p_adapt_cst_PR
				Ki_modified_PR_x = Ki_modified_PR_x + np.abs(xError)*i_adapt_cst_PR
				Kd_modified_PR_x = Kd_modified_PR_x + np.abs(xError)*d_adapt_cst_PR
				Kp_modified_PR_y = Kp_modified_PR_y + np.abs(yError)*p_adapt_cst_PR
				Ki_modified_PR_y = Ki_modified_PR_y + np.abs(yError)*i_adapt_cst_PR
				Kd_modified_PR_y = Kd_modified_PR_y + np.abs(yError)*d_adapt_cst_PR
			else:
				Kp_modified_PR_x = Kp_modified_PR_x - np.abs(xError)*p_adapt_cst_PR
				Ki_modified_PR_x = Ki_modified_PR_x - np.abs(xError)*i_adapt_cst_PR
				Kd_modified_PR_x = Kd_modified_PR_x - np.abs(xError)*d_adapt_cst_PR
				Kp_modified_PR_y = Kp_modified_PR_y - np.abs(yError)*p_adapt_cst_PR
				Ki_modified_PR_y = Ki_modified_PR_y - np.abs(yError)*i_adapt_cst_PR
				Kd_modified_PR_y = Kd_modified_PR_y - np.abs(yError)*d_adapt_cst_PR



		if not xyFirstTime:
			xIntegralError += xError*( time.time() - xyLastTime )
			yIntegralError += yError*( time.time() - xyLastTime )
			xIntegralError = np.clip(xIntegralError, -1000, 1000)
			yIntegralError = np.clip(yIntegralError, -1000, 1000)

			if xyCounterPID == 3:
				xDerivativeError = np.clip( ( (x_intercept - x_drone) - xLastErrorDerivative ) / ( time.time() - xyLastTimeDerivative ), -1000, 1000)
				yDerivativeError = np.clip( ( (y_intercept - y_drone) - yLastErrorDerivative ) / ( time.time() - xyLastTimeDerivative ), -1000, 1000)
				xLastErrorDerivative = x_intercept - x_drone
				yLastErrorDerivative = y_intercept - y_drone
				xyLastTimeDerivative = time.time()
				xyCounterPID = 0

		xComponent = Kp_modified_PR_x*xError + Ki_modified_PR_x*xIntegralError + Kd_modified_PR_x*xDerivativeError + xyConstant
		yComponent = Kp_modified_PR_y*yError + Ki_modified_PR_y*yIntegralError + Kd_modified_PR_y*yDerivativeError + xyConstant
		rotMatrix = np.matrix([[np.cos(heading_drone),-np.sin(heading_drone)],[np.sin(heading_drone),np.cos(heading_drone)]])

		rotatedComponents = np.array([xComponent, yComponent])*rotMatrix

		droneRoll = int(np.clip(int(rotatedComponents[0,0]), 1000, 2000))
		dronePitch = int(np.clip(int(rotatedComponents[0,1]), 1000, 2000))
		
		xyLastTime = time.time()
		xyFirstTime = False
		xyCounterPID += 1
		time.sleep(100E-3)



def ThreadAuto():

	global zControlActive, PRControlActive, inputEnabled
	global state
	global x_intercept, y_intercept, x_drone, y_drone
	global z_intercept, z_drone
	global dronePitch, droneRoll, droneYaw, droneThrottle
	global updatePX_CM_constant

	state = EMERGENCY_STOP

	time.sleep(1)

	while True:

		if state != EMERGENCY_STOP:

			zControlActive = True
			PRControlActive = True
			inputEnabled = True

			updatePX_CM_constant = True

			time.sleep(1)

			if state == INITIALIZING:

				x_intercept = 0
				y_intercept = 0
				z_intercept = 70

				print("Starting Controllers")

				if(droneControllers == Z_CONTROLLER or droneControllers == BOTH_CONTROLLERS):
					threading.Thread(target=PID_Controller_z, args=()).start()

				if(droneControllers == PITCH_ROLL_CONTROLLER or droneControllers == BOTH_CONTROLLERS):
					threading.Thread(target=PID_Controller_Pitch_Roll, args=()).start()

				while ( ( np.linalg.norm( np.array([x_intercept, y_intercept, z_intercept])-np.array([x_drone, y_drone, z_drone]) ) > 20 ) and state == INITIALIZING ):
					pass

				if state == INITIALIZING:
					state = READY_TO_CATCH
					updatePX_CM_constant = True


		else:
			zControlActive = False
			PRControlActive = False
			inputEnabled = False
			dronePitch = 1500
			droneRoll = 1500
			droneYaw = 1500
			droneThrottle = 1000


### OPENCV X-Y LOCATION ###
def openCV_PositionUpdateThread():

	global x_drone, y_drone, heading_drone
	global updatePX_CM_constant, PX_CM_constant, CM_BETWEEN_POINTS

	capture = cv2.VideoCapture(UPPER_CAMERA)

	width_center = (capture.get(cv2.CAP_PROP_FRAME_WIDTH)/2)
	height_center = (capture.get(cv2.CAP_PROP_FRAME_HEIGHT)/2)

	lower_1 = np.array([102, 79, 116]) #Blue
	upper_1 = np.array([130, 212, 251])

	lower_2 = np.array([80, 28, 93]) #Green
	upper_2 = np.array([101, 74, 212])

	while True:
		_, color_OriginalImage = capture.read()

		OriginalImage = cv2.cvtColor(color_OriginalImage, cv2.COLOR_BGR2HSV)

		mask1 = cv2.inRange(OriginalImage, lower_1, upper_1)
		mask2 = cv2.inRange(OriginalImage, lower_2, upper_2)

		nonZero1 = np.nonzero(mask1)
		nonZero2 = np.nonzero(mask2)

		mass1X = np.mean(nonZero1[0])
		mass1Y = np.mean(nonZero1[1])

		mass2X = np.mean(nonZero2[0])
		mass2Y = np.mean(nonZero2[1])

		try:
			x_drone = int(np.mean([mass1X, mass2X])) - height_center
			y_drone = int(np.mean([mass1Y, mass2Y])) - width_center
			heading_drone = np.arctan2( mass2Y-mass1Y, mass2X-mass1X ) + (90+45)*np.pi/180

			if updatePX_CM_constant:
				PX_CM_constant = CM_BETWEEN_POINTS / ( np.linalg.norm( np.array([mass1X, mass1Y]) - np.array([mass2X, mass2Y]) ) )
				updatePX_CM_constant = False
				print("Calculated constant: %.4f cm/px"%PX_CM_constant)

		except:
			print("Couldn't find Drone")




if droneConnection != NO_CONNECTION:
	threading.Thread(target=ThreadDroneNode, args=()).start()


if droneConnection == MANUAL_CONNECTION or (droneConnection == AUTO_CONNECTION and droneControllers == Z_CONTROLLER):
	threading.Thread(target=ThreadManual, args=()).start()


if droneConnection == AUTO_CONNECTION:
	threading.Thread(target=ThreadAuto, args=()).start()
	if droneControllers == PITCH_ROLL_CONTROLLER or droneControllers == BOTH_CONTROLLERS:
		threading.Thread(target=openCV_PositionUpdateThread, args=()).start()







