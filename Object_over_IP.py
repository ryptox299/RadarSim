import asyncio
import pickle
import struct

# Supporting instrumentation
import inspect
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)	# DEBUG, INFO, WARNING, ERROR, CRITICAL
logger.setLevel(logging.INFO)

class DataServer:
	def __init__(self, host, port, ingress_queue, egress_queue):
		self.host = '0.0.0.0'		# All connection from anywhere
		self.port = port			# Don't forget to make the inbound firewalls allow the port numbers
		self.server = None
		self.ingress_queue = ingress_queue
		self.egress_queue = egress_queue
		self.tasks = []
		self.close_event = asyncio.Event()	# event to signal that the server should close
		self.connected = False

	async def handle_client(self, reader, writer):	# Server callback when a client connects
		print(f"Server: Client has connected to server ************************************************************************************")
		self.tasks.append(asyncio.create_task(self.read_data(reader)))
		self.tasks.append(asyncio.create_task(self.write_data(writer)))
		self.connected = True
#		await asyncio.gather(*self.tasks)

	async def read_data(self, reader):
		while not self.close_event.is_set():
			try:
				length_data = await reader.read(4)
				if not length_data:
					break
				length = struct.unpack('!I', length_data)[0]
				serialized = await reader.read(length)
				item = pickle.loads(serialized)
#				print(f"Object_over_IP server read_data() added item to egress queue")
				await self.egress_queue.put(item)

			except asyncio.CancelledError:
				break	# Expected when closing

			except Exception as e:
				logger.error(f"{__name__}.{inspect.currentframe().f_code.co_name}: ")
				logger.error(f"Exception {e}")
				print(f"DataServer():read_data(): Exception {e}")
				break
				
#		reader.close()
#		await read.wait_closed()

	async def write_data(self, writer):
		while not self.close_event.is_set():
			try:
				item = await self.ingress_queue.get()
				if item is None:  # Use None as a sentinel to exit.
					break
				serialized = pickle.dumps(item)
				length = struct.pack('!I', len(serialized))
				writer.write(length)
				writer.write(serialized)
				await writer.drain()

			except asyncio.CancelledError:
				break	# Expected when closing
				
		writer.close()
		await writer.wait_closed()

	async def start(self):
		self.server = await asyncio.start_server(
			self.handle_client, self.host, self.port
		)
		async with self.server:
			try:
				await self.server.serve_forever()

			except asyncio.CancelledError:
				pass	# Expected when closing

	async def close(self):
		self.close_event.set()
		if self.server:
			self.server.close()
			await self.server.wait_closed()
			self.server = None # Release reference
#		print(f"Server tasks\n{self.tasks}")
		for task in self.tasks:
			try:
				task.cancel()
				
			except Exception as e:
				logger.error(f"{__name__}.{inspect.currentframe().f_code.co_name}: ")
				logger.error(f"Exception {e}")
				pass				

class DataClient:
	def __init__(self, host, port, ingress_queue, egress_queue):
		self.host = host
		self.port = port
		self.ingress_queue = ingress_queue
		self.egress_queue = egress_queue
		self.tasks = []
		self.close_event = asyncio.Event()	# event to signal that the server should close
		self.connected = False
		self.reconnect_period = 2

	async def start(self):
		while not self.close_event.is_set():
			try:
				if not self.connected:				# Try to establish a new connection
					if self.tasks:					# Clean out residual tasks after a link has dropped
						await self.cancel_tasks()
						
					reader, writer = await asyncio.open_connection(self.host, self.port)
					self.connected = True
					print(f"[{self.host}:{self.port}] Client Connection made to server.")
					self.tasks.append(asyncio.create_task(self.read_data(reader)))
					self.tasks.append(asyncio.create_task(self.write_data(writer)))
#					print(f"[{self.host}:{self.port}] Client connected: {self.connected} Tasks: {len(self.tasks)}\n{self.tasks}", flush=True)
				else:
					await asyncio.sleep(1)	# Sleep before checking again if already connected

			except (ConnectionRefusedError, OSError) as e:
				print(f"[{self.host}:{self.port}] Connection error: {e}", flush=True)
				self.connected = False		# Mark as not connected
				await asyncio.sleep(self.reconnect_period)

			except Exception as e:
				print(f"[{self.host}:{self.port}] Start loop() error: {e}", flush=True)
				self.connected = False		# Mark as not connected

#			finally:
#				await asyncio.sleep(1)

	async def read_data(self, reader):
#		print(f"[{self.host}:{self.port}] read_data() start")
		try:
			while not self.close_event.is_set():
				length_data = await reader.read(4)
				if not length_data:
					break
				length = struct.unpack('!I', length_data)[0]
				serialized = await reader.read(length)
				item = pickle.loads(serialized)
				await self.egress_queue.put(item)
		except Exception as e:
			print(f"[{self.host}:{self.port}] DataClient():read_data(): Exception {e}")
				
		self.connected = False  # Update the connection status
#		print(f"[{self.host}:{self.port}] read_data() exit : connected: {self.connected}")
		print(f"[{self.host}:{self.port}] Connection to server lost")

	async def write_data(self, writer):
		while not self.close_event.is_set():
			try:
				item = await self.ingress_queue.get()
				if item is None:
					break
				serialized = pickle.dumps(item)
				length = struct.pack('!I', len(serialized))
				writer.write(length)
				writer.write(serialized)
				await writer.drain()
			except asyncio.CancelledError:
				break  # Expected when closing

		writer.close()
		await writer.wait_closed()

	async def cancel_tasks(self):
#		print(f"[{self.host}:{self.port}] task_cancel(): {self.tasks}", flush=True)
		for task in self.tasks:
			try:
				task.cancel()
				
			except Exception as e:
				logger.error(f"{__name__}.{inspect.currentframe().f_code.co_name}: ", flush=True)
				logger.error(f"Exception {e}")
				pass				

		await asyncio.sleep(1)
		self.tasks.clear()		# Remove tasks from list

	async def close(self):
		self.close_event.set()
		await asyncio.sleep(1)
		await self.cancel_tasks()


# Test bench

import random

async def write_queue_test_data(name, queue, count):
	while count > 0:
		await asyncio.sleep(random.randint(1,5)/10)
		test_data = [random.randint(1, 100) for _ in range(6)]	  # Generate a list of 6 random integers
		print(f"{name}: >> {test_data}")
		await queue.put(test_data)
		count -= 1
	await queue.put("BYE")

async def timeout(time, exit_event, tasks):
	await asyncio.sleep(time)
	print(f"Timeout expired, closing system")
	is_running = False
	exit_event.set()

	for task in tasks:
		try:
			task.cancel()
			
		except Exception as e:
			logger.error(f"{__name__}.{inspect.currentframe().f_code.co_name}: ")
			logger.error(f"Exception {e}")
			pass				



async def read_queue(name, queue, exit_event):
	while not exit_event.is_set():
		try:
			data = await queue.get()
			print(f"{name}: << {data}")
			
			if data == "BYE":
				break;

		except asyncio.CancelledError:
			break	# Expected when closing

async def main():
	exit_event = asyncio.Event()
	is_running = True
	
	server_ingress_queue = asyncio.Queue()
	server_egress_queue = asyncio.Queue()
	client_ingress_queue = asyncio.Queue()
	client_egress_queue = asyncio.Queue()
	server = DataServer("127.0.0.1", 4015, server_ingress_queue, server_egress_queue)
	client = DataClient("127.0.0.1", 4015, client_ingress_queue, client_egress_queue)

	tasks = []

	tasks.append(asyncio.create_task(server.start()))
	tasks.append(asyncio.create_task(client.start()))

	tasks.append(asyncio.create_task(read_queue("server_egress_queue", server_egress_queue, exit_event)))
	tasks.append(asyncio.create_task(read_queue("client_egress_queue", client_egress_queue, exit_event)))

	tasks.append(asyncio.create_task(write_queue_test_data("server_ingress_queue", server_ingress_queue, 5)))
	tasks.append(asyncio.create_task(write_queue_test_data("client_ingress_queue", client_ingress_queue, 5)))

	await asyncio.sleep(2)	

	await server_ingress_queue.put("Message 1 from Server")
	await server_ingress_queue.put("Message 2 from Server")
	await server_ingress_queue.put("Message 3 from Server")
	await server_ingress_queue.put("Message 4 from Server")

	await client_ingress_queue.put("Message 1 from Client")
	await client_ingress_queue.put("Message 2 from Client")

	tasks.append(asyncio.create_task(timeout(10, exit_event, tasks))) # give it some time to process the messages

#	await exit_event.is_set()
	await asyncio.sleep(15)
	print(f"Closing client")
	await client.close()
	print(f"Closing server")
	await server.close()

	try:
		await asyncio.gather(*tasks)
	except asyncio.CancelledError:
		print("Tasks cancelled")


		
if __name__ == '__main__':
	asyncio.run(main())
