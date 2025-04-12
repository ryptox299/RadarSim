# This application is designed to present a 3D viewer, for data from a streaming source
# Data is presented via one or more input queues

import sys
#import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QTextEdit
from PyQt5.QtGui import QPixmap
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
#from mpl_toolkits.mplot3d.art3d import Poly3DCollection
#from matplotlib.patches import Polygon, Circle, Ellipse
#from matplotlib.lines import Line2D
import asyncio
#from asyncqt import QEventLoop, asyncSlot
import qasync
#from PyQt5.QtCore import QEventLoop  # Import QEventLoop
import matplotlib.colors as mcolors
from PyQt5.QtCore import pyqtSignal
import time
from datetime import datetime
from collections import namedtuple
import R4_public

from dataclasses import dataclass
from dataclasses import fields
import dataclasses


# Supporting instrumentation
import inspect
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.ERROR)	# DEBUG, INFO, WARNING, ERROR, CRITICAL
logger.setLevel(logging.INFO)

# This is the 3D display object for each input object
@dataclass
class Object3D:
	def __init__(self, desc, color = 'magenta'):
		logger.debug(f"Object3D.__init__(): descriptor: {desc}")
		if False:
			self.x = desc[0]
			self.y = desc[1]
			self.z = desc[2]
			self.x_dir = desc[3]
			self.y_dir = desc[4]
			self.z_dir = desc[5]
			self.ID = desc[6]
			
		elif False:
			self.timestamp = desc[0]
			self.sensor_name = desc[1]
			self.obj_origin = desc[2]
			self.x = desc[3]
			self.y = desc[4]
			self.z = desc[5]
			self.x_dir = desc[6]
			self.y_dir = desc[7]
			self.z_dir = desc[8]
			self.ID = desc[9]
			self.azimuth_deg = desc[10]
			self.elevation_deg = desc[11]
			self.range = desc[12]

		else:
			logger.debug(f"Object3D.__init__(): R4_DETECTION to Object3D")
			try:
				self.timestamp = desc.timestamp
				self.sensor_name = desc.sensor
				self.obj_origin = desc.src
				self.x = desc.X
				self.y = desc.Y
				self.z = desc.Z
				self.x_dir = desc.Xdir
				self.y_dir = desc.Ydir
				self.z_dir = desc.Zdir
				self.ID = desc.ID
				self.p = desc.Pwr
				self.RangeRate = desc.RangeRate
				self.azimuth_deg = desc.Az
				self.elevation_deg = desc.El
				self.range = desc.Range
				self.Conf = desc.Conf

			except Exception as e:
				logger.error(f"{__name__} Object3D.__init__(): Conversion from R4_DETECTION failed: {e}")
	
		self.color = color	# Assign the color parameter to the object

	def plot(self, ax, colour, databox=None):


		if True:		# Dummy for performance test
			if databox:		# This attribute is only present on new reports, older reports don't have additional features
				# Add a motion vector line
				ax.quiver(self.x, self.y, self.z, self.x_dir, self.y_dir, self.z_dir, color = 'cyan')

				# Add the axis drops
				ax.plot([self.x, self.x], [self.y, self.y], [self.z, 0], color="r", linewidth=1, linestyle='dotted')

				# Add a green circle at Z=0 below the object
				ax.scatter(self.x, self.y, 0, facecolors='g', edgecolors='g', alpha=0.3)

		if False:		# Dummy for performance test
			return


#		logger.debug(f"Object3D.plot(): colour:{colour}")

		if True:
			if databox:
				if not "anon" in self.ID:
					label_colour = 'b'
#					ax.text(self.x + 1, self.y + 1, self.z + 1, f"{self.sensor_name}_{self.ID}", color=label_colour)
					ax.text(self.x + 1, self.y + 1, self.z + 1, f"{self.ID}", color=label_colour)
					ax.plot([self.x, self.x+1], [self.y, self.y+1], [self.z, self.z+1], color=label_colour, linewidth=0.5, linestyle='dotted')
					msg = ""
#					msg = f"{self.timestamp}:{self.sensor_name}\r\n"
					msg += f"{self.ID}:"
					msg += f" ({self.x:+06.1f},  {self.y:+06.1f},  {self.z:+06.1f})"
					msg += f" #{self.Conf}"
					databox.append(msg)
					pass

		# Position - do this last, so it shows
		ax.scatter(self.x, self.y, self.z, color=colour)


	def release_memory(self):
		if False:
			self.x = None
			self.y = None
			self.z = None
			self.x_dir = None
			self.y_dir = None
			self.z_dir = None
			self.color = None
		
		for f in fields(self):
#			self.f.name) = None
			setattr(self, f.name, None)

# This is a list of 3D display objects from the same frame
class Frame3D:
	def __init__(self, objects, color = 'pink', src = 0):
		self.objects = objects
		self.color = color
		self.src = src
		self.persistence = 0
		
	def plot(self, ax, databox=None):
#		logger.debug(f"Frame3D colour:{self.color}")

		for obj in self.objects:
			if self.persistence == 0:
				obj.plot(ax, self.color, databox)	# Use the frame colour for the object, only label 1st cycle
			else:
				obj.plot(ax, self.color)			# Use the frame colour for the object

	def release_memory(self):
		# Release memory for the frame and its objects explicitly
		for obj in self.objects:	
			obj.release_memory()

		del self.objects[:]	# Clear the objects list
		self.color = None	# Remove references to the color

		
class R4_3D_view(QMainWindow):
	# Define the window_closed signal as a class-level attribute
	window_closed = pyqtSignal()

	def __init__(self, event_loop, external_source_queues = []):
		super().__init__()
		self.event_loop = event_loop

		self.test_mode = False
		if len(external_source_queues) == 0:
			self.test_mode = True
			external_source_queues = [asyncio.Queue(), asyncio.Queue(), asyncio.Queue()]

		self.input_desc_queue = external_source_queues	# Array of queues, each with raw external data stream
		self.tasks = []
		self.Frame3D_list = []								# Array element per input_desc_queue, each holds a list of Frame3D objects
		self.frame_plotted = []
		self.max_persistence = 3							# Persistence cycles

		for ix, _ in enumerate(self.input_desc_queue):
			self.Frame3D_list.append(Frame3D([], 'magenta', 0))
			logger.info(f"Starting task to process queue {ix}")
			self.tasks.append( asyncio.create_task(self.task_process_queue(ix)) )
			self.frame_plotted.append(False)
		
		self.colLUT = self.initialize_color_lookup_table(len(self.input_desc_queue), self.max_persistence)
		
		self.xlim_default = [-20, 20]
		self.ylim_default = [0, 40]
		self.zlim_default = [-10, 30]
		# Viewing angles
		self.az_default = 315
		self.el_default = 15

		self.xlim = self.xlim_default
		self.ylim = self.ylim_default
		self.zlim = self.zlim_default

		self.open()

	def open(self):
		self.initUI()
		self.reset_view()
		self.tasks.append( asyncio.create_task(self.task_GUI_update()) )
		self.tasks.append( asyncio.create_task(self.task_persistence()) )

	def closeEvent(self, event):
		self.window_closed.emit()
#		super().closeEvent(event)
		
	def initUI(self):
		self.setWindowTitle('3D Plot with Controls')
		self.dpi = 100
		self.width = 1024
		self.height = 720
		self.setGeometry(40, 40, self.width, self.height)

		# Create a central widget to hold the 3D plot and buttons
		central_widget = QWidget(self)
		self.setCentralWidget(central_widget)

		self.canvas = PlotCanvas(self, width=self.width/self.dpi, height=self.height/self.dpi, dpi=self.dpi)

		# Add control buttons
		control_button_layout = QHBoxLayout()

		if self.test_mode:
			for ix in range(len(self.input_desc_queue)):
				# Add a data injection button for each queue - for testing and demo purposes
				frame_btn = (QPushButton(f"Add frame to queue {ix}", self))
				frame_btn.clicked.connect(lambda _, arg=ix: self.create_frame(arg))
				control_button_layout.addWidget(frame_btn)

		# Add buttons for different views
		top_view_btn = QPushButton('Plan - Top View', self)
		top_view_btn.clicked.connect(self.top_view)

		side_view_btn = QPushButton('Elevation - Side View', self)
		side_view_btn.clicked.connect(self.side_view)

		zoom_in_btn = QPushButton('Zoom in', self)
		zoom_in_btn.clicked.connect(self.zoom_in)

		zoom_out_btn = QPushButton('Zoom out', self)
		zoom_out_btn.clicked.connect(self.zoom_out)

		reset_view_btn = QPushButton('Reset view', self)
		reset_view_btn.clicked.connect(self.reset_view)

		# Create an outer QHBoxLayout for the buttons
		view_button_layout = QHBoxLayout()
		view_button_layout.addWidget(top_view_btn)
		view_button_layout.addWidget(side_view_btn)
		view_button_layout.addWidget(zoom_in_btn)
		view_button_layout.addWidget(zoom_out_btn)
		view_button_layout.addWidget(reset_view_btn)

		data_widget = QWidget(self)
		self.init_logo()
		self.init_databox()
#		data_widget.setMaximumWidth(2 * self.logo_width)
#		data_widget.setMinimumWidth(20 + self.logo_width)
		data_widget.setMaximumWidth(400)
		data_widget.setMinimumWidth(400)
		
		logger.info(f"initUI(): Setting up databox layout")
		data_layout = QVBoxLayout(data_widget)
		data_layout.addWidget(self.logo)
		data_layout.addWidget(self.databox)
		logger.info(f"initUI(): Setting up databox layout: Done")

		plotter_layout = QVBoxLayout()
		plotter_layout.addWidget(self.canvas)
		plotter_layout.addLayout(view_button_layout)

		main_area_layout = QHBoxLayout()
		main_area_layout.addWidget(data_widget)
		main_area_layout.addLayout(plotter_layout)

		# Setup the main layout
		layout = QVBoxLayout(central_widget)
		layout.addLayout(control_button_layout)
		layout.addLayout(main_area_layout)

		self.show()

	def init_logo(self):
		self.logo = QLabel(self)
		self.logo.img = QPixmap('logo.jpg')
		self.logo_width = self.logo.img.width()
		self.logo.setPixmap(self.logo.img)

	def init_databox(self):
		logger.info(f"init_databox(): Start")
		self.databox = QTextEdit()
		self.databox.setReadOnly(True)
		logger.info(f"init_databox(): Done")

	def databox_prune(self, limit):
		logger.debug(f"init_databox(): databox_prune: Start")
		lines_to_cut = self.databox.document().blockCount() - limit
		cursor = self.databox.textCursor()
		logger.debug(f"databox_prune(): lines_to_cut: {lines_to_cut}")

		if True:
			if lines_to_cut > 0:
				if False:
					self.databox.clear()
					self.databox.append("Box cleared")
			
				else:
					self.databox.setReadOnly(False)
					logger.debug(f"databox_prune(): Cutting {lines_to_cut} lines")
					for line in range(lines_to_cut):
						logger.debug(f"databox_prune(): Deleting line")
						cursor.movePosition(cursor.Start)
#						cursor.select(cursor.BlockUnderCursor)
						cursor.select(cursor.LineUnderCursor)
						text = cursor.selectedText()
						logger.debug(f"Pruning line {line}: {text} ")
						cursor.removeSelectedText()
						cursor.deleteChar()
						pass
					self.databox.setReadOnly(True)

			cursor.movePosition(cursor.End)
			self.databox.setTextCursor(cursor)	# Set the modified cursor back to update the display

		logger.debug(f"databox_prune(): Done")

	def initialize_color_lookup_table(self, N, A):
		# Define strong and faded colors for each stream
		strong_colors = ['red', 'blue', 'green', 'purple', 'orange'][:N]  # You can add more colors if needed

		# Initialize the color lookup table
		colLUT = []

		alpha_step = 1.0/A
		for stream in range(N):
			stream_colors = []
			for age in range(A):
				color_rgba = mcolors.to_rgba(strong_colors[stream], alpha = 1.0-(age*alpha_step))
#				logger.debug(f"stream:{stream} age:{age} colour:{color_rgba}")
				stream_colors.append(color_rgba)

			colLUT.append(stream_colors)

		return colLUT

	def reset_view(self):
		# Set the default
		self.canvas.ax.view_init(elev=self.el_default, azim=self.az_default)

		self.xlim = self.xlim_default
		self.ylim = self.ylim_default
		self.zlim = self.zlim_default

		self.scale_view(1)
		
	def top_view(self):
		# Set the view to top view
		self.canvas.ax.view_init(elev=90, azim=270)
		self.scale_view(1)

	def side_view(self):
		# Set the view to side view
		self.canvas.ax.view_init(elev=-1, azim=359)
		self.scale_view(1)

	def zoom_in(self):
		self.scale_view(0.8)
	
	def zoom_out(self):
		self.scale_view(1.2)

	def scale_view(self, scale):
		self.xlim = [self.xlim[0]*scale, self.xlim[1]*scale]
		self.ylim = [self.ylim[0]*scale, self.ylim[1]*scale]
		self.zlim = [self.zlim[0]*scale, self.zlim[1]*scale]
		
		self.canvas.ax.set_xlim(self.xlim)
		self.canvas.ax.set_ylim(self.ylim)
		self.canvas.ax.set_zlim(self.zlim)

		# Redraw baseline
		self.canvas.ax.plot([self.xlim[0], self.xlim[1]], [0, 0], [0, 0], color="g", linewidth=2)
		# Redraw range boresight
		self.canvas.ax.plot([0, 0], [0, self.ylim[1]], [0, 0], color="g", linewidth=2)

		# Draw ground grid
		ground_grid_colour='g'
		for x in self.canvas.ax.xaxis.get_majorticklocs():
			self.canvas.ax.plot([x, x], self.ylim, [0, 0], color=ground_grid_colour, linewidth=0.5)
		
		for y in self.canvas.ax.yaxis.get_majorticklocs():
			self.canvas.ax.plot(self.xlim, [y, y], [0, 0], color=ground_grid_colour, linewidth=0.5)

		# Re label axes
		self.canvas.ax.xaxis.set_label_text("X axis - Baseline", color=self.canvas.ax.xaxis.color)
		self.canvas.ax.yaxis.set_label_text("Y axis - Distance", color=self.canvas.ax.yaxis.color)
		self.canvas.ax.zaxis.set_label_text("Z axis - Height", color=self.canvas.ax.zaxis.color)
		
		self.canvas.draw()
	

	def toggle_labels(self):
		if self.canvas.is_labels_visible():
			self.canvas.hide_labels()
		else:
			self.canvas.show_labels()

# Create a frame with random object, and add it to the queue
	def create_frame(self, ix):
		frame_desc = FrameDesc(True)	# Random frame
#		asyncio.create_task(self.input_desc_queue[ix].put(frame_desc.objects))
		asyncio.create_task(self.input_desc_queue[ix].put(frame_desc))

	# This task keeps reading frames from the input queue.
	# The latest frame kept in a single frame buffer for the persistence process
	# The task_persistence task will clear objects when they are processed
	# If the input is faster than the persistence, frames get overwritten becuase we want the latest frame
	# This keeps latency to a minimum, no point in using out of date data
	async def task_process_queue(self, ix):		# One task running per input queue
		while True:
			try:
#				await asyncio.sleep(0)			# Give other tasks a chance
				
				t1 = time.time()
				logger.debug(f"{__name__} task_process_queue() loop start at {t1}")

				Qlength = self.input_desc_queue[ix].qsize()
				if Qlength > 1:
					logger.warning(f"task_process_queue():{t1}: Queue:{ix} has {self.input_desc_queue[ix].qsize()} queued items, backing up!")

				if Qlength > 5:
					logger.warning(f"task_process_queue():{t1}: Queue:{ix} has {self.input_desc_queue[ix].qsize()} queued items, flushing queue to reduce latency!")
					
					while self.input_desc_queue[ix].qsize() > 1:			# Leave one in the barrel to be processed
						await self.input_desc_queue[ix].get()				# Read the queue and dump the data, old frames

				latest_frame_desc = await self.input_desc_queue[ix].get()	# Read the queue, get a frame which contains 0 or more objects

				# New report frame received
				logger.debug(f"{__name__}: New frame received with {len(latest_frame_desc.objects)} objects on queue:{ix}")
				# Read the input frame descriptor which contains detection objects
				# Create a Frame3D containing Object3D for the GUI
				objects = []													# Reset the object list for this frame
				for object_desc in latest_frame_desc.objects:
#					logger.debug(f"task_process_queue(): Received frame descriptor on queue {ix}: length {len(object_desc)}")
					logger.debug(f"task_process_queue(): Received frame descriptor on queue {ix}")
					logger.info(f"Rx: {object_desc}")

					objects.append(Object3D(object_desc))						# Add to the objects list as an instance of Object3D class

				self.Frame3D_list.append(Frame3D(objects, self.colLUT[ix][0], ix))	# Create new object, setting colour, add it to the list
				
#				logger.info(f"{__name__} task_process_queue(): Queue {ix} holds {len(self.Frame3D_list)} frames")
#				await asyncio.sleep(0.1)
				
			except asyncio.CancelledError:
				logger.warning(f"{__name__}.{inspect.currentframe().f_code.co_name}: Task stopped")
				pass
				
			except Exception as e:
				logger.error(f"{__name__}.{inspect.currentframe().f_code.co_name}: ")
				logger.error(f"Exception {e}")
				pass
				
	async def task_persistence(self):
		while True:
			logger.debug(f"{__name__} task_persistence() Loop at {time.time()}")
			self.persistence()
#			await asyncio.sleep(0.33)	# Display update and persistence rate
			await asyncio.sleep(0.1)	# Display update and persistence rate

	# This process ages the detections, fades their colours, and eventually deletes the plot
	def persistence(self):
		# Plot the frames, frames age per queue
		logger.debug(f"{__name__} persistence() start at {time.time()}")

		Q = len(self.Frame3D_list)	# Number of Frame3Ds queued
		
		logger.info(f"{__name__} persistence(): Processing {len(self.Frame3D_list)} frames")

		for frame3D in self.Frame3D_list:							# Iterate the frames in the queue
			frame3D.persistence += 1								# Age the frame
			
			if frame3D.persistence < self.max_persistence:
				frame3D.color = self.colLUT[frame3D.src][frame3D.persistence]	# Colour depends on queue and age
#		logger.info(f"{__name__} persistence(): All frames aged")


		# We need to delete the old frames, but this changes the indexing of the queue/array
#			logger.info(f"{__name__} persistence(): Delete ancient frames")
		while (len(self.Frame3D_list) > 0) and (self.Frame3D_list[0].persistence >= self.max_persistence):	# Keep testing head of list
#				logger.info(f"{__name__} persistence(): Deleting frame with new persistence age {self.Frame3D_list[0].persistence}")
			frame3D.release_memory()
			self.Frame3D_list.pop(0)		# Delete when too old

	async def task_GUI_update(self):		# Spinning task to update from the latest data
		while True:
#			await asyncio.sleep(0.1)
			await asyncio.sleep(0.05)

			t1 = time.time()
			logger.debug(f"{__name__} task_GUI_update(): start at {t1}")
		
			current_time = datetime.fromtimestamp(time.time())
			time_string = current_time.strftime("%H:%M:%S:%f")
			self.databox.append(f"{time_string}:> {len(self.Frame3D_list)} frames to plot")
			self.plot_frame()
			
			for i in range(len(self.frame_plotted)):
				self.frame_plotted[i] = True

			t2 = time.time()

			# Update the canvas
			self.scale_view(1)


			if False:
				t3 = time.time()
				logger.info(f"task_GUI_update() precalc:{t2-t1} draw:{t3-t2}  Total:{t3-t1} at {t1}")
				logger.debug(f"task_GUI_update(): at {time_string}")

			# Trim the databox so that the file doesn't become large over time
			if True:
#				self.databox.append(f"Block count before prune {self.databox.document().blockCount()}")
				self.databox_prune(50)
#				self.databox.append(f"Block count after prune {self.databox.document().blockCount()}")

	def plot_frame(self):
		logger.debug(f"{__name__} GUI_3D:plot_frame() start at {time.time()}")
		# Clear the previous frame
		self.canvas.ax.clear()

		# Plot the frames, frames age per queue
		Q = len(self.Frame3D_list)

		for i in reversed(range(len(self.Frame3D_list))):		# Iterate over frames in reverse order to keep latest on top
			frame = self.Frame3D_list[i]
			if frame.persistence < 1:							# Most recent frames
				self.databox.append(f"New frame contains {len(frame.objects)} objects to plot")
				frame.plot(self.canvas.ax, self.databox)		# Only report on the latest
			else:
				frame.plot(self.canvas.ax)

#		frame.plot(self.canvas.ax)



class PlotCanvas(FigureCanvas):
	def __init__(self, parent=None, width=5, height=4, dpi=100):
		self.fig = plt.figure(figsize=(width, height), dpi=dpi)
		self.ax = self.fig.add_subplot(111, projection='3d')

		margin = 0.0	# Set margin width
		self.fig.subplots_adjust(left=margin, right=1-margin, top=1-margin, bottom=margin) # Reduce margins

		FigureCanvas.__init__(self, self.fig)
		self.setParent(parent)

		FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
		FigureCanvas.updateGeometry(self)

		self.draw_axes()

	def draw_axes(self):
		# Customise the grid
		self.ax.xaxis.color="blue"	# X-axis color
		self.ax.yaxis.color="green"	# Y-axis color
		self.ax.zaxis.color="red"	# Y-axis color
		

		self.ax.xaxis.line.set_color(self.ax.xaxis.color)	
		self.ax.yaxis.line.set_color(self.ax.yaxis.color)
		self.ax.zaxis.line.set_color(self.ax.zaxis.color)

#		self.ax.xaxis.grid(color=self.ax.xaxis.color)
#		self.ax.xaxis.grid("None")
#		self.ax.xaxis._axinfo['grid']['color'] = self.ax.xaxis.color
#		self.ax.yaxis._axinfo['grid']['color'] = self.ax.yaxis.color
#		self.ax.zaxis._axinfo['grid']['color'] = self.ax.zaxis.color

		self.ax.xaxis.set_label_text("X axis - Baseline", color=self.ax.xaxis.color)
		self.ax.yaxis.set_label_text("Y axis - Distance", color=self.ax.yaxis.color)
		self.ax.zaxis.set_label_text("Z axis - Height", color=self.ax.zaxis.color)

	def hide_labels(self):
		logger.debug(f"Hiding labels")
		self.ax.set_axis_off()

	def show_labels(self):
		logger.debug(f"Showing labels")
		self.ax.set_axis_on()

	def is_labels_visible(self):
		return not self.ax.xaxis.get_visible()	# Checking the visibility of the x-axis labels


def main():
	app = QApplication(sys.argv)
	event_loop = qasync.QEventLoop(app)
	asyncio.set_event_loop(event_loop)

	asyncio_queues = [asyncio.Queue(), asyncio.Queue()]

	ex = R4_3D_view(event_loop)
#	ex = R4_3D_view(event_loop, asyncio_queues)
	ex.show()

	with event_loop:
		sys.exit(event_loop.run_forever())


if __name__ == '__main__':
	main()
