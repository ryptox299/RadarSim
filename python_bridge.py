import asyncio
import json
import pickle
import struct
from R4_public import R4_DETECTION
from Object_over_IP import DataClient

class CustomJSONEncoder(json.JSONEncoder):
    """
    Custom JSON Encoder to handle circular references and unsupported types.
    """
    def default(self, obj):
        try:
            # Attempt to serialize objects using their __dict__
            return obj.__dict__
        except AttributeError:
            pass  # Fall through to default handling

        # Fallback to default serialization
        return super().default(obj)

def convert_frame_to_named_objects(frame):
    """
    Convert a frame (FrameDesc object) into a list of dictionaries with named keys.
    """
    # Access the `objects` field of the FrameDesc object
    objects_list = frame.objects  # Assuming `objects` is the correct attribute

    # Define the keys for the radar data
    keys = [
        "timestamp", "sensor", "src", "X", "Y", "Z", "Xdir", "Ydir", "Zdir",
        "Range", "RangeRate", "Pwr", "Az", "El", "ID", "Xsize", "Ysize", "Zsize", "Conf"
    ]

    # Convert each object into a dictionary
    objects = []
    for obj in objects_list:
        # Ensure `obj` is iterable, otherwise skip
        if not hasattr(obj, "__iter__"):
            continue

        # Convert the object to a dictionary
        named_object = dict(zip(keys, obj))
        objects.append(named_object)

    return {"objects": objects}

async def process_data(sim_ip, sim_port, cpp_ip, cpp_port):
    try:
        # Connect to the radar simulation server
        reader, writer = await asyncio.open_connection(sim_ip, sim_port)

        # Start the TCP server for the C++ program
        cpp_server = await asyncio.start_server(
            lambda r, w: handle_cpp_client(r, w, reader),
            cpp_ip,
            cpp_port,
        )
        async with cpp_server:
            await cpp_server.serve_forever()
    except Exception as e:
        print(f"Error in process_data: {e}")

async def handle_cpp_client(cpp_reader, cpp_writer, sim_reader):
    """
    Handle incoming connections from the C++ program and forward data.
    """
    try:
        while True:
            # Read data from the radar simulation server
            frame_size_data = await sim_reader.read(4)
            if not frame_size_data:
                break

            # Unpack the frame size
            frame_size = struct.unpack('>I', frame_size_data)[0]

            # Read the frame data
            frame_data = await sim_reader.read(frame_size)
            radar_frame = pickle.loads(frame_data)

            # Convert the frame to named objects
            try:
                named_objects_frame = convert_frame_to_named_objects(radar_frame)

                # Serialize radar data to JSON
                json_frame = json.dumps(
                    named_objects_frame,
                    cls=CustomJSONEncoder,  # Use the updated custom encoder
                )
            except Exception as e:
                continue  # Skip this frame and move to the next one

            # Send the JSON data to the C++ program
            cpp_writer.write((json_frame + '\n').encode('utf-8'))  # Append newline for C++ compatibility
            await cpp_writer.drain()  # Ensure the data is sent
    except Exception as e:
        print(f"Error in handle_cpp_client: {e}")
    finally:
        cpp_writer.close()
        await cpp_writer.wait_closed()

if __name__ == "__main__":
    SIM_IP = "192.168.10.10"  # Radar simulation server IP
    SIM_PORT = 4035           # Radar simulation server port
    CPP_IP = "127.0.0.1"      # Localhost for C++ program
    CPP_PORT = 5000           # Port for C++ program

    asyncio.run(process_data(SIM_IP, SIM_PORT, CPP_IP, CPP_PORT))
