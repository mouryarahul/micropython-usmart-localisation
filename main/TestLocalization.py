import random
import struct

try:
    from uac_localisation.main.misc.datetime import datetime, timedelta
    from uac_localisation.main.utm.conversion import to_latlon, from_latlon
    from uac_localisation.main.misc.utils import distance
    from uac_modem.main.unm3driver import MessagePacket
    from uac_localisation.main.LocalizationProcess import LocalizationProcess
except ImportError:
    from datetime import datetime, timedelta
    from main.utm.conversion import to_latlon, from_latlon
    from main.misc.utils import distance
    from main.unm3driver import MessagePacket
    from main.LocalizationProcess import LocalizationProcess


# Anchor locations in Lat/Lon and anchor_depth (m)
anchor_depth = 1.0  # m
M1 = [56.38547, -4.22764]
A11 = [56.39012, -4.22906]
A12 = [56.38967, -4.23279]
A13 = [56.38886, -4.23593]
sg1 = [M1, A11, A12, A13]
# Convert to UTM
SG1 = []
while sg1:
    anchor = sg1.pop(0)
    x, y, zone, zone_letter = from_latlon(anchor[0], anchor[1])
    SG1.append([x, y, anchor_depth, zone, zone_letter])

M2 = [56.38583, -4.22755]
A21 = [56.3876, -4.23768]
A22 = [56.38632, -4.23893]
A23 = [56.3848, -4.23953]
sg2 = [M2, A21, A22, A23]
# Convert to UTM
SG2 = []
while sg2:
    anchor = sg2.pop(0)
    x, y, zone, zone_letter = from_latlon(anchor[0], anchor[1])
    SG2.append([x, y, anchor_depth, zone, zone_letter])

M3 = [56.38537, -4.22747]
A31 = [56.38309, -4.23871]
A32 = [56.38176, -4.23704]
A33 = [56.38071, -4.23344]
sg3 = [M3, A31, A32, A33]
# Convert to UTM
SG3 = []
while sg3:
    anchor = sg3.pop(0)
    x, y, zone, zone_letter = from_latlon(anchor[0], anchor[1])
    SG3.append([x, y, anchor_depth, zone, zone_letter])

M4 = [56.38596, -4.22716]
A41 = [56.38012, -4.22979]
A42 = [56.38, -4.22563]
A43 = [56.38119, -4.22138]
sg4 = [M4, A41, A42, A43]
# Convert to UTM
SG4 = []
while sg4:
    anchor = sg4.pop(0)
    x, y, zone, zone_letter = from_latlon(anchor[0], anchor[1])
    SG4.append([x, y, anchor_depth, zone, zone_letter])

M5 = [56.38535, -4.22733]
A51 = [56.38302, -4.21781]
A52 = [56.38535, -4.21704]
A53 = [56.38775, -4.2173]
sg5 = [M5, A51, A52, A53]
# Convert to UTM
SG5 = []
while sg5:
    anchor = sg5.pop(0)
    x, y, zone, zone_letter = from_latlon(anchor[0], anchor[1])
    SG5.append([x, y, anchor_depth, zone, zone_letter])

M6 = [56.38543, -4.22781]
A61 = [56.38955, -4.2191]
A62 = [56.39022, -4.22314]
A63 = [56.39031, -4.22588]
sg6 = [M6, A61, A62, A63]
# Convert to UTM
SG6 = []
while sg6:
    anchor = sg6.pop(0)
    x, y, zone, zone_letter = from_latlon(anchor[0], anchor[1])
    SG6.append([x, y, anchor_depth, zone, zone_letter])

Beacon_Segments = [SG1, SG2, SG3, SG4, SG5, SG6]


# Sensor locations in Lat/Lon and anchor_depth(m)
S1 = [56.38587, -4.23374]
S2 = [56.38264, -4.22751]
S3 = [56.38585, -4.22069]
S4 = [56.38893, -4.22799]
S5 = [56.38808, -4.23198]
S6 = [56.38335, -4.23344]
S7 = [56.38392, -4.22275]
S8 = [56.38817, -4.22395]
"""
sensors = [S1, S2, S3, S4, S4, S5, S6, S7, S8]
# Convert to UTM
SENSORS = []
while sensors:
    sensor = sensors.pop(0)
    x, y, zone, zone_letter = from_latlon(sensor[0], sensor[1])
    SENSORS.append([x, y, anchor_depth])
"""
sensor_loc = S1  # in Lat/Lon
sensor_depth = 100.0  # in meters
x, y, zone, zone_letter = from_latlon(sensor_loc[0], sensor_loc[1])
sensor_loc = [x, y, sensor_depth, zone, zone_letter]

# Sound speed
sound_speed = 1530.0  # m/s
fluid_density = 1029.0  # kg/m^3
# Create an instance of LocalizationProcess
sensor = LocalizationProcess()
sensor.sensor_depth = 100.0


# Generate Beacon Signals
timestamp = datetime.now()
data_packet = MessagePacket()  # An instance of MessagePacket
# Broadcast signals to update the Environment Parameters for Location Estimator
msg = b'ULMP' + b'S' + struct.pack('f', sound_speed)
data_packet.source_address = 0
data_packet.destination_address = 255
data_packet.packet_type = MessagePacket.PACKETTYPE_BROADCAST
data_packet.packet_payload = msg
# data_packet.timetuple = timetuple
data_packet.timestamp = (timestamp.year, timestamp.month, timestamp.day, timestamp.hour,
                         timestamp.minute, timestamp.second, timestamp.microsecond)
sensor.handle_incoming_data_packet(data_packet)


timestamp = datetime.now()
data_packet = MessagePacket()  # An instance of MessagePacket
msg = b'ULMP' + b'D' + struct.pack('f', fluid_density)
data_packet.source_address = 0
data_packet.destination_address = 255
data_packet.packet_type = MessagePacket.PACKETTYPE_BROADCAST
data_packet.packet_payload = msg
# data_packet.timetuple = timetuple
data_packet.timestamp = (timestamp.year, timestamp.month, timestamp.day, timestamp.hour,
                         timestamp.minute, timestamp.second, timestamp.microsecond)
sensor.handle_incoming_data_packet(data_packet)

t0 = datetime.now()  # Start of Beacon Cycle
for i in range(len(Beacon_Segments)):
    if i == 0:
        # Start of Beacon Cycle
        data_packet = MessagePacket()
        msg = b'ULMS'
        # Time taken by signal to reach Sensor
        master_to_sensor = distance(Beacon_Segments[i][0][0:3], sensor_loc[0:3])/sound_speed  # in seconds
        timestamp = t0 + timedelta(seconds=master_to_sensor)
        data_packet.source_address = 0
        data_packet.destination_address = 255
        data_packet.packet_type = MessagePacket.PACKETTYPE_BROADCAST
        data_packet.packet_payload = msg
        data_packet.timestamp = (timestamp.year, timestamp.month, timestamp.day, timestamp.hour,
                                 timestamp.minute, timestamp.second, timestamp.microsecond)
        # data_packet.timestamp = (seconds, millis, micros)
        sensor.handle_incoming_data_packet(data_packet)

    # Generate remaining Beacon Signals
    for j in range(0, len(Beacon_Segments[i])):
        lat, lon = to_latlon(Beacon_Segments[i][j][0], Beacon_Segments[i][j][1], Beacon_Segments[i][j][3], Beacon_Segments[i][j][4])
        anchor_depth = Beacon_Segments[i][j][2]
        lat = struct.pack('f', lat)
        lon = struct.pack('f', lon)
        anchor_depth = struct.pack('f', anchor_depth)
        if j == 0:  # Start of Beacon Segment
            time_delay = random.random()  # in seconds
            t0 = t0 + timedelta(seconds=time_delay)  # add some random delay
            master_to_sensor = distance(Beacon_Segments[i][0][0:3], sensor_loc[0:3])/sound_speed  # in seconds
            timestamp = t0 + timedelta(seconds=master_to_sensor)
            m = len(Beacon_Segments[i])
            n = i
            msg = b'ULMB' + struct.pack('B', n) + struct.pack('B', m) + b'L' + lat + lon + anchor_depth

        else:  # Continuation of Beacon Segment
            master_to_anchor = distance(Beacon_Segments[i][0][0:3], Beacon_Segments[i][j][0:3]) / sound_speed  # in seconds
            time_delay = random.random()  # in seconds
            anchor_to_sensor = distance(Beacon_Segments[i][j][0:3], sensor_loc[0:3]) / sound_speed  # in seconds
            total_time_duration = master_to_anchor + time_delay + anchor_to_sensor
            timestamp = t0 + timedelta(seconds=total_time_duration)
            n = i
            msg = b'ULAB' + struct.pack('B', n) + b'L' + lat + lon + anchor_depth + b'D' + struct.pack('f', time_delay)

        data_packet = MessagePacket()  # An instance of MessagePacket
        data_packet.source_address = 0
        data_packet.destination_address = 255
        data_packet.packet_type = MessagePacket.PACKETTYPE_BROADCAST
        data_packet.packet_payload = msg
        data_packet.timestamp = (timestamp.year, timestamp.month, timestamp.day, timestamp.hour,
                                 timestamp.minute, timestamp.second, timestamp.microsecond)
        # data_packet.timestamp = (seconds, millis, micros)
        sensor.handle_incoming_data_packet(data_packet)

# Beacon Cycle Ended
msg = b'ULME'
timestamp = t0 + timedelta(seconds=random.random())
#print("Msg:{} \t\t\t\t\t\t\t\t\t\t Timestamp: {}\n\n".format(msg[0:4], timetuple))
data_packet = MessagePacket()  # An instance of MessagePacket
data_packet.source_address = 0
data_packet.destination_address = 255
data_packet.packet_type = MessagePacket.PACKETTYPE_BROADCAST
data_packet.packet_payload = msg
# data_packet.timetuple = timetuple
data_packet.timestamp = (timestamp.year, timestamp.month, timestamp.day, timestamp.hour,
                         timestamp.minute, timestamp.second, timestamp.microsecond)
# data_packet.timetuple = (seconds, millis, micros)
sensor.handle_incoming_data_packet(data_packet)

print("Sensor's Current Locations: ", sensor.current_location)






