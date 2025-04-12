import asyncio
import time
import socket
import pickle
import struct
from R4_public import R4_DETECTION  # Import R4_DETECTION from R4_public to match GUI's namespace

async def simulate_radar_tcp_output(log_file_path, listen_ip='192.168.10.10', listen_port=4035):
    """
    Simulates radar output by reading data and sending it as pickled frames of R4_DETECTION objects.
    """
    try:
        # Open the log file for reading
        reader = open(log_file_path, 'r')

        # Set up a TCP server
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind((listen_ip, listen_port))
        server.listen(1)
        print(f"Simulating radar (TCP Server) listening on {listen_ip}:{listen_port}...")

        conn, addr = await asyncio.get_running_loop().sock_accept(server)
        print(f"GUI (Client) connected from {addr}")

        while True:
            line = reader.readline()
            if not line:
                print("End of log file reached. Closing connection.")
                break

            if "Fusion Egress+:" in line and "R4_DETECTION" in line:
                try:
                    # Extract R4_DETECTION data from the line
                    start_index = line.find("R4_DETECTION(")
                    if start_index != -1:
                        detection_str = line[start_index:]
                        data_str = detection_str[len("R4_DETECTION("):-1]
                        parts = [item.split('=') for item in data_str.split(', ')]
                        detection_data = {part[0].strip(): part[1].strip().strip("'") for part in parts if len(part) == 2}

                        # Construct R4_DETECTION object
                        current_timestamp = time.time()
                        detection_object = R4_DETECTION(
                            timestamp=current_timestamp,
                            sensor=str(detection_data.get('sensor', '')),
                            src=str(detection_data.get('src', '')),
                            X=float(detection_data.get('X', 0.0)),
                            Y=float(detection_data.get('Y', 0.0)),
                            Z=float(detection_data.get('Z', 0.0)),
                            Xdir=float(detection_data.get('Xdir', 0.0)),
                            Ydir=float(detection_data.get('Ydir', 0.0)),
                            Zdir=float(detection_data.get('Zdir', 0.0)),
                            Range=float(detection_data.get('Range', 0.0)),
                            RangeRate=float(detection_data.get('RangeRate', 0.0)),
                            Pwr=float(detection_data.get('Pwr', 0.0)),
                            Az=float(detection_data.get('Az', 0.0)),
                            El=float(detection_data.get('El', 0.0)),
                            ID=str(detection_data.get('ID', '')),
                            Xsize=float(detection_data.get('Xsize', 0.0)),
                            Ysize=float(detection_data.get('Ysize', 0.0)),
                            Zsize=float(detection_data.get('Zsize', 0.0)),
                            Conf=float(detection_data.get('Conf', '').rstrip(')') or 0.0)
                        )

                        # Package the detection object into a frame as a dictionary
                        frame = {'objects': [detection_object]}
                        pickled_frame = pickle.dumps(frame)
                        frame_size = len(pickled_frame)
                        size_bytes = struct.pack(">I", frame_size)

                        # Send the frame to the GUI
                        try:
                            await asyncio.get_running_loop().sock_sendall(conn, size_bytes)
                            await asyncio.get_running_loop().sock_sendall(conn, pickled_frame)
                            print(f"Sent frame with size: {frame_size} and data: {frame}")
                            await asyncio.sleep(0.1)
                        except Exception as send_error:
                            print(f"Error sending frame: {send_error}")
                            break

                except ValueError as ve:
                    print(f"ValueError during conversion: {ve} in line: {line.strip()}")
                except Exception as e:
                    print(f"Error processing R4_DETECTION block: {e} in line: {line.strip()}")

        # Close connections
        conn.close()
        server.close()
        reader.close()
        print("Radar TCP output simulation finished.")

    except FileNotFoundError:
        print(f"Error: Log file not found at {log_file_path}")
    except OSError as e:
        print(f"Error binding to {listen_ip}:{listen_port}: {e}")
    except Exception as e:
        print(f"An unexpected error occurred in the simulation: {e}")

if __name__ == "__main__":
    log_file = 'C:/simdata.txt'  # Updated to use simdata.txt
    listen_ip = '192.168.10.10'
    listen_port = 4035
    asyncio.run(simulate_radar_tcp_output(log_file, listen_ip, listen_port))
