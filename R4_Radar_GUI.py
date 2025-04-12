#import socket
import threading
#import queue
from PyQt5.QtWidgets import QApplication
#from asyncqt import QEventLoop
import asyncio
import GUI_3D
#import pickle
#import socketserver
#import time
from collections import namedtuple
import sys
from PyQt5.QtCore import pyqtSignal, QThread, QCoreApplication
import qasync

import Object_over_IP as ObjOvrIP
#import random

import R4_public

# Supporting instrumentation
import inspect
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)	# DEBUG, INFO, WARNING, ERROR, CRITICAL
logger.setLevel(logging.DEBUG)


SVR = namedtuple("SVR", "host port")



def run_clients(server_list, ingress_data_queues, egress_data_queues):
	tasks = []
	clients = []
	# Create and start client threads for each server address in the list
	for server_address, ingress_data_queue, egress_data_queue in zip(server_list, ingress_data_queues, egress_data_queues):
		print(f"Creating client for {server_address}")
		client = ObjOvrIP.DataClient(server_address.host, server_address.port, ingress_data_queue, egress_data_queue)
		tasks.append(asyncio.create_task(client.start()))
		tasks.extend(client.tasks)
		clients.append(client)
	print(f"run_client(): Task list contains {len(tasks)} tasks")
	return tasks, clients

async def stop_clients(clients):
	for client in clients:
		await client.close()


# Supported for test bench
def run_servers(server_list, ingress_data_queues, egress_data_queues):
	tasks = []
	servers = []
	# Create and start client threads for each server address in the list
	for server_address, ingress_data_queue, egress_data_queue in zip(server_list, ingress_data_queues, egress_data_queues):
		print(f"Creating server for {server_address}")
		server = ObjOvrIP.DataServer('localhost', server_address.port, ingress_data_queue, egress_data_queue)
		tasks.append(asyncio.create_task(server.start()))
		tasks.extend(server.tasks)
		servers.append(server)
	print(f"run_server(): Task list contains {len(tasks)} tasks")
	return tasks, servers

async def stop_servers(servers):
	for server in servers:
		await server.close()

async def report_queues(queues):
	print(f"R4_Radar_GUI.report_queues(): queues = {queues}")
	while True:
		print(f"Queue content report")
		n = 1;
		for queue in queues:
			print(f"Q{n} holds {queue.qsize()}")
			n += 1
		await asyncio.sleep(1)

# Variant for frame descriptors
async def write_test_data_to_queue(name, queue, count):
#	print(f"{name} write_test_data_to_queue() start")
	while True:
#	for n in range(count):								# Number of frames to generate
#		print(f"write_test_data_to_queue(): {name} frame:{n+1} of {count}")
		
		frame_desc = R4_public.FrameDesc(True)

		print(f"write_test_data_to_queue(): {name} Adding dummy data to queue already containing {queue.qsize()} items")
		await queue.put(frame_desc.objects)

		print(f"write_test_data_to_queue(): {name}: >> frame {frame_desc}")
#		print(f"write_test_data_to_queue(): {name}: >> frame objects {frame_desc.objects}")
#		print(f"write_test_data_to_queue(): {name}: write_test_data_to_queue(): Sleeping")
		await asyncio.sleep(0.1)
#		print(f"write_test_data_to_queue(): {name}: write_test_data_to_queue(): Wakeing")
	await queue.put("BYE")
	print(f"write_test_data_to_queue(): {name}: write_test_data_to_queue(): Done")



async def shutdown(servers, clients, tasks):
	print(f"Shutting down")

	print(f"Closing {len(clients)} clients")
	await stop_clients(clients)

	print(f"Closing {len(servers)} server")
	await stop_servers(servers)

	await asyncio.sleep(5)
	report_running_tasks(tasks)


def report_running_tasks(tasks):
	# Remove completed tasks from the list
	tasks = [task for task in tasks if not task.done()]

	print(f"Task list contains:")
	for item in tasks:
		print(f"{item}")



async def R4_Radar_GUI(event_loop, exit_event):
	# Configuration
	run_GUI = True
	# The following are to support test and development, normal operation, all False
	run_dummy_servers = False
	generate_dummy_plot_data = False	# Inject directly in queue to GUI
	read_dummy_queue_data = False		# Read queue to GUI

	# Maintain a list of the tasks
	tasks = []

	# List of data source servers
	server_list = []

	# Uising append() so that servers are easy to select or remove by commenting out
	if run_dummy_servers:
#		server_list.append(SVR('localhost', 4013))
#		server_list.append(SVR('localhost', 4014))
		server_list.append(SVR('localhost', 4023))
		server_list.append(SVR('localhost', 4024))
	else:
#		node_server='192.168.0.20'
#		node_server='192.168.0.26'
#		node_server='84.92.98.222'
#		node_server='192.168.1.224'
#		node_server='localhost'
		node_server='192.168.10.10'
		node1_server = node_server
		node2_server = node_server
#		server_list.append(SVR(node1_server, 4013))
#		server_list.append(SVR(node1_server, 4014))
#		server_list.append(SVR(node1_server, 4015))
#		server_list.append(SVR(node2_server, 4023))
#		server_list.append(SVR(node2_server, 4024))
#		server_list.append(SVR(node2_server, 4025))

#		server_list.append(SVR('localhost',		4035))
		server_list.append(SVR('192.168.10.10',	4035))


	print(f"Configuration:")
	print(f"Run the GUI						   : {run_GUI}")
	print(f"Generating data via dummy servers  : {run_dummy_servers}")
	print(f"Generating data via GUI input queue: {generate_dummy_plot_data}")
	print(f"Reading data queues (consumer)	   : {read_dummy_queue_data}")
	print(f"Server list: {server_list}\n\n")


	# Queues : These are the interfaces to the applications. They communicate via sockets with remote data sources/sinks
	client_ingress_data_queues = []		# Client side input - goto the remote server
	client_egress_data_queues = []		# Client side output - come from the remote server, and become the GUI inputs
	server_ingress_data_queues = []		# Server side input - dummy server input, to flow through the client to the GUI
	server_egress_data_queues = []		# Server side output - dummy server output, from client input
	qsize = 0							# Default queue size, 0 = unconstrained

	if run_dummy_servers:			# Run test servers?
		# Create a list of data queues for each server
		server_ingress_data_queues = [asyncio.Queue(qsize) for _ in server_list]
		server_egress_data_queues = [asyncio.Queue(qsize) for _ in server_list]
		# Start the servers
		server_tasks, servers = run_servers(server_list, server_ingress_data_queues, server_egress_data_queues)
		tasks.extend(server_tasks)
		await asyncio.sleep(1)


	# Run the clients - to connect to servers
	# Create a list of data queues for each client
	client_ingress_data_queues = [asyncio.Queue(qsize) for _ in server_list]
	client_egress_data_queues = [asyncio.Queue(qsize) for _ in server_list]
	# Run the clients
	client_tasks, clients = run_clients(server_list, client_ingress_data_queues, client_egress_data_queues)
	tasks.extend(client_tasks)
	
	
	if False:				# Monitor the queues
		all_queues = []
		all_queues.extend(server_ingress_data_queues)
		all_queues.extend(server_egress_data_queues)
		all_queues.extend(client_ingress_data_queues)
		all_queues.extend(client_egress_data_queues)
		tasks.append(asyncio.create_task(report_queues(all_queues)))

	def on_window_closed():
#		print(f"on_window_closed() called")
		exit_event.set()

	if run_GUI:			# Run the GUI
		print(f"Starting GUI with {len(client_egress_data_queues)} input streams")
		ex = GUI_3D.R4_3D_view(event_loop, client_egress_data_queues)
		print(f"{type(exit_event)}")
		ex.window_closed.connect(on_window_closed)
#		ex.window_closed.connect(lambda: exit_event.set())
		ex.show()
		print(f"GUI opened")


	if read_dummy_queue_data:		# For testing, read from these queues
		# Run the queue readers, for test data
		for queue in server_egress_data_queues:
			tasks.append(asyncio.create_task(ObjOvrIP.read_queue("server_egress_queue S", queue, exit_event)))

		for queue in client_egress_data_queues:
			tasks.append(asyncio.create_task(ObjOvrIP.read_queue("client_egress_queue C", queue, exit_event)))


	# Run test data sources, data injection
	if run_dummy_servers:			# Dummy servers always need data
		for queue in server_ingress_data_queues:
			tasks.append(asyncio.create_task(write_test_data_to_queue("server_ingress_queue S", queue, 5)))
			pass
			
		# Optional for testing reverse path from client to server
		if False:
			for queue in client_ingress_data_queues:
				tasks.append(asyncio.create_task(write_test_data_to_queue("client_ingress_queue C", queue, 5)))


	if generate_dummy_plot_data:	# Test environment, putting data into these queues is normally done via the clients
		for queue in client_egress_data_queues:
			tasks.append(asyncio.create_task(write_test_data_to_queue("client_egress_queue C", queue, 5)))
			pass



	# Manage runtime and exit
	if run_GUI:
		print(f"Running GUI loop")
#		await asyncio.sleep(30)
		try:
			while True:
				await asyncio.sleep(0.1)
#			with event_loop:
#				event_loop.run_forever()
		except KeyboardInterrupt:
			print("KeyboardInterrupt: Stopping the event loop gracefully...")
			is_runnng = False
			await asyncio.sleep(0.1)
			event_loop.stop()  # Stop the event loop when Ctrl-C is pressed

		print(f"End of GUI loop")

	report_running_tasks(tasks)



def checks():
	try:
		# Attempt to access the __version__ attribute
		installed_version = qasync.__version__
	except AttributeError:
		raise ImportError("Could not determine the version of qasync. Version 0.24.0 is required. Use 'pip install qasync==0.24.0'")

	# Check the version at runtime
	if installed_version != '0.24.0':
		raise ImportError(f"qasync version 0.24.0 is required, but {installed_version} is installed. Use 'pip install qasync==0.24.0'")


def main():
	checks()
	app = QApplication(sys.argv)
	event_loop = qasync.QEventLoop(app)
	asyncio.set_event_loop(event_loop)
	print(f"Event loop check: {type(event_loop)}")
	
	exit_event = asyncio.Event()

	async def main_async(event_loop):
		await R4_Radar_GUI(event_loop, exit_event)
		await exit_event.wait()
		print(f"main_async terminating\n")

	event_loop.run_until_complete(main_async(event_loop))

	with event_loop:
		sys.exit(event_loop.run_forever())
		
		
if __name__ == '__main__':
	main()

