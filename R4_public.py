# R4 Public library
import numpy as mp
import math
from collections import namedtuple
import time

# Supporting instrumentation
import inspect
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)	# DEBUG, INFO, WARNING, ERROR, CRITICAL
logger.setLevel(logging.INFO)


# Detection reports in this format
R4_DETECTION = namedtuple("R4_DETECTION", "timestamp sensor src X Y Z Xdir Ydir Zdir Range RangeRate Pwr Az El ID Xsize Ysize Zsize Conf")

R4_FUSION_COEFFICIENTS = namedtuple("R4_FUSION_COEFFICIENTS", "X Y Z Xdir Ydir Zdir Range RangeRate Pwr Az El ID Xsize Ysize Zsize")

# Az +ve left, El +ve up, orientation +ve anti-clockwise
R4_ORIENTATION = namedtuple("R4_ORIENTATION", "Az_deg El_deg Polarisation_deg")

R4_LOCATION = namedtuple("R4_LOCATION", "X Y Z")




# Input descriptors
# This gives the items coordinates and velocity vector
class ObjDesc:
	def __init__(self, timestamp, sensor_name, obj_origin, x=0, y=0, z=0, x_dir=0, y_dir=0, z_dir=0, id=None, test=True):
		self.sensor_name = sensor_name
		self.obj_origin = obj_origin
		self.timestamp = timestamp
		self.ID = id
		self.x = x
		self.y = y
		self.z = z
		self.x_dir = x_dir
		self.y_dir = y_dir
		self.z_dir = z_dir
		self.azimuth_deg = 0
		self.elevation_deg = 0
		self.range = 0
		
		if test:
			self.random()

		self._calculated_params()
		self._build_desc()

	def _build_desc(self):
		self.desc = [ self.timestamp, self.sensor_name, self.obj_origin, self.x, self.y, self.z, self.x_dir, self.y_dir, self.z_dir, self.ID,
					self.azimuth_deg, self.elevation_deg, self.range]

	def _calculated_params(self):	# Calculate angles and range
		logger.info(f"{__name__}.{inspect.currentframe().f_code.co_name}()")
		# Pre calculate squares
		x2 = self.x**2
		y2 = self.y**2
		z2 = self.z**2
		self.range = math.sqrt(x2 + y2 + z2)			# Pythagorus for range
		self.azimuth = math.degrees(math.atan2(self.x, -self.y))	# Azimuth, +ve turning right, 0 = boresight along Y axis
		self.elevation = math.degrees(math.atan2(self.z, math.sqrt(x2 + y2)))  # Elevation, 0 horizontal, +ve upwards	
		self._build_desc()

	def set_data_origin(self, sensor_name, obj_origin):
		self.sensor_name = sensor_name
		self.obj_origin = obj_origin
		self._build_desc()

	def id(self, ID):
		self.ID = ID
		self._build_desc()

	def timestamp(self, timestamp):
		self.timestamp = timestamp
		self._build_desc()

	def random(self):
		self.obj_origin = "Random"
		# Random object generator
		self.x = (np.random.rand() - 0.5) * 30
		self.y = np.random.rand() * 90
		self.z = np.random.rand() * 35
		self.x_dir, self.y_dir, self.z_dir = (np.random.rand(3) - 0.5) * 2
		self._build_desc()

	def release_memory(self):
		self.desc = []
		self.desc = None

# This is a list of Objects from the same radar frame
class FrameDesc:
	def __init__(self, test=False):
		# A frame comprises of a list of descriptors, including attributes for each object
		self.objects = []
		if test:
			for obj in range(5):			# Number of objects
				obj = ObjDesc()				# Randomised object coordinates and direction
				self.add_object(obj)		# Add it to the frame

	def add_object(self, obj_desc):	# Expecting Class ObjDesc
		self.objects.append(obj_desc.desc)

	def clear(self):
		self.objects = []

	def print_object_ages(self, at_description = None):
		oldest = -1.0
		youngest = 1.0e6
		for obj in self.objects:
			try:
				age = time.time() - obj.timestamp
				print(f"FrameDesc.object age: {age}s {at_description}")
				
				oldest = max(oldest, age)
				youngest = min(youngest, age)
		
			except Exception as e:
				logger.debug(f"{e}")
				
		
		return [oldest, youngest]


	def release_memory(self):
		# Release memory for the frame and its objects explicitly
		for obj in self.objects:	
			obj.release_memory()

		del self.objects[:]	# Clear the objects list
