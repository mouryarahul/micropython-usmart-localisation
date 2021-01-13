import struct
try:
    from uac_modem.main.unm3driver import MessagePacket
    from uac_localisation.main.TDOALocalization import TDOALocalization
    from uac_localisation.main.LocationCalculator import estimate_location
    from uac_localisation.main.misc.datetime import datetime, timedelta
except ImportError:
    from main.unm3driver import MessagePacket
    #from main.TDOALocalization import TDOALocalization
    from main.LocationCalculator import estimate_location
    from datetime import datetime, timedelta

# STATES
NOT_STARTED = -1
CYCLE_STARTED = 0
SEGMENT_STARTED = 1
SEGMENT_CONTINUED = 2
SEGMENT_ENDED = 3
CYCLE_ENDED = 4

def pyb_mktime(timestamp: tuple) -> float:
    secs, millis, micros = timestamp
    return secs + millis/1E3 + micros/1E6

def pyb_gmtime(seconds: float) -> tuple:
    secs, rem = divmod(seconds, 1)
    millis, rem = divmod(rem*1000, 1)
    micros, rem = divmod(rem*1000, 1)
    micros += round(rem)
    return secs, millis, micros


class LocalizationProcess:
    def __init__(self):
        self._extern_sensor = None
        self._sound_speed = None
        self._depth = None
        self._fluid_density = None
        self._current_location = None
        self._cycle_start_time = None
        self._cycle_end_time = None
        self.received_beacon_signals = []
        self.intermediate_beacon_signals = []
        self.final_beacon_signals = []
        self.current_beacon_segment = []
        self.current_beacon_segment_number = -1

    def __call__(self):
        return self

    @property
    def extern_sensor(self):
        """ Get the external sensor handle object """
        return self._extern_sensor

    @extern_sensor.setter
    def extern_sensor(self, handle):
        """ Set external sensor handle object """
        self._extern_sensor = handle

    @property
    def current_location(self) -> list:
        """ Get the last estimated location """
        return self._current_location

    #@current_location.setter
    #def current_location(self, location: list):
    #    self._current_location = location

    @property
    def fluid_density(self) -> float:
        """ Get the last estimated location """
        return self._fluid_density

    @fluid_density.setter
    def fluid_density(self, fluid_density: float):
        """ Set the medium density for depth calculation """
        self._fluid_density = fluid_density

    @property
    def sound_speed(self) -> float:
        """ Get the current sound speed parameter """
        return self._sound_speed

    @sound_speed.setter
    def sound_speed(self, sound_speed: float):
        """ Set the sound speed parameter """
        self._sound_speed = sound_speed

    @property
    def sensor_depth(self):
        """ Get current depth parameter """
        return self._depth

    @sensor_depth.setter
    def sensor_depth(self, depth: float):
        """ Set the depth parameter """
        self._depth = depth

    def handle_incoming_data_packet(self, data_packet: MessagePacket) -> None:
        """
        This function handles all incoming data packets and store them into a list
        for further processing. This function an external interface for Localization Process.
        It will be called from the Main Control Layer whenever a Localization related
        data packet is received by the Sensor node.
        :param data_packet: MessagePacket
        :return: None
        """

        # Extract different field of the data_packet
        packet_type = data_packet.packet_type
        source_address = data_packet.source_address
        timestamp = data_packet.timestamp
        packet_payload = data_packet.packet_payload

        if isinstance(packet_payload, bytes):
            packet_bytes = packet_payload
        elif isinstance(packet_payload, list):
            # convert it into bytes
            packet_bytes = b""
            for item in packet_payload:
                packet_bytes += bytes([item])
        else:
            pass

        # print("In 'handle_incoming_data_packet' ")
        # print("packet_bytes:", packet_bytes)

        if packet_type == MessagePacket.PACKETTYPE_BROADCAST:
            # Check first 4 bytes of the data_packet to decide its purpose
            if packet_bytes[0:4] == b'ULMS':  # Indicator for start of beacon signal cycle
                self.received_beacon_signals = []
                self.received_beacon_signals.append([packet_bytes, timestamp])  # store the beacon signal
            elif packet_bytes[0:4] == b'ULMP':  # Package containing Sound Speed and Water Density Info
                self.update_environ_parameters(packet_bytes)
            elif packet_bytes[0:4] == b'ULMB' or packet_bytes[0:4] == b'ULAB':
                self.received_beacon_signals.append([packet_bytes, timestamp])  # store the beacon signal
            elif packet_bytes[0:4] == b'ULME':  # Indicator for End of Beacon Cycle
                self.received_beacon_signals.append([packet_bytes, timestamp])  # store the beacon signal
                # Process the beacon signals and estimate the location
                self.process_beacon_signals_and_estimate_location()
            else:
                pass
        elif packet_type == MessagePacket.PACKETTYPE_UNICAST:
            pass
        else:
            pass

    def update_environ_parameters(self, packet_payload: bytes) -> None:
        if packet_payload[4:5] == b'S':  # Sound Speed Info
            self._sound_speed = struct.unpack('f', packet_payload[5:])[0]
            # print("Updated Sound Speed.")
        elif packet_payload[4:5] == b'D':  # Fluid Density Info
            self._fluid_density = struct.unpack('f', packet_payload[5:])[0]
            # print("Updated Fluid Density.")
        else:
            pass

    def handle_start_of_beacon_segment(self, data_packet: bytes, timetuple: tuple):
        """
        This function handle the beacon signal from Lead anchor.
        :param data_packet: contains beacon info
        :param timetuple: beacon arrival time
        :return: None
        """
        try:
            # print("In 'handle_start_of_beacon_segment' ")
            self.current_beacon_segment_number = struct.unpack('B', data_packet[4:5])[0]
            m = struct.unpack('B', data_packet[5:6])[0]  # number of assistant anchors
            location_indicator = data_packet[6:7].decode('utf8')
            lat = struct.unpack('f', data_packet[7:11])[0]
            lon = struct.unpack('f', data_packet[11:15])[0]
            depth = struct.unpack('f', data_packet[15:19])[0]
            if isinstance(timetuple, tuple):
                if len(timetuple) == 3:
                    arrival_time = pyb_mktime(timetuple)
                elif len(timetuple) == 7:
                    dt = datetime(timetuple[0], timetuple[1], timetuple[2], timetuple[3], timetuple[4], timetuple[5], timetuple[6])
                    arrival_time = dt.timestamp()  # convert datetime to seconds.
                else:
                    raise Exception("timetuple should be tuple of 3 or 7 elements!")
            else:
                raise Exception("timetuple should be tuple of 3 or 7 elements!")
        except Exception as e:
            print("Corrupted data packet: " + str(e) + "\n")
        else:
            # store the previous beacon_segment if any into intermediate_beacon_signals
            if self.current_beacon_segment:
                self.intermediate_beacon_signals.append(self.current_beacon_segment)

            # store the current beacon into current segment segment
            self.current_beacon_segment = []
            if location_indicator == 'L':
                self.current_beacon_segment.append([lat, lon, depth, arrival_time, 0.0])
                # print("Stored Start of Beacon Segment")

    def handle_continue_of_beacon_segement(self, data_packet: bytes, timetuple: tuple):
        """
        This function handle beacon signal from assistant anchors:
        :param data_packet: contains beacon info
        :param timetuple: beacon arrival time
        :return: None
        """
        try:
            # print("In 'handle_continue_of_beacon_segement'")
            segment_number = struct.unpack('B', data_packet[4:5])[0]  # current beacon segment number
            location_indicator = data_packet[5:6].decode('utf8')
            lat = struct.unpack('f', data_packet[6:10])[0]  # number of assistant anchors
            lon = struct.unpack('f', data_packet[10:14])[0]
            depth = struct.unpack('f', data_packet[14:18])[0]
            delay_indicator = data_packet[18:19].decode('utf8')
            delay = struct.unpack('f', data_packet[19:23])[0]
            if isinstance(timetuple, tuple):
                if len(timetuple) == 3:
                    arrival_time = pyb_mktime(timetuple)
                elif len(timetuple) == 7:
                    dt = datetime(timetuple[0], timetuple[1], timetuple[2], timetuple[3], timetuple[4], timetuple[5], timetuple[6])
                    arrival_time = dt.timestamp()  # convert rtc datetime to seconds.
                else:
                    raise Exception("timetuple should be tuple of 3 or 7 elements!")
            else:
                raise Exception("timetuple should be tuple of 3 or 7 elements!")
        except Exception as e:
            print("Corrupted data packet: " + str(e) + "\n")
        else:
            if self.current_beacon_segment_number == segment_number:
                if location_indicator == 'L' and delay_indicator == 'D':
                    self.current_beacon_segment.append([lat, lon, depth, arrival_time, delay])
                    # print("Stored Continue of Beacon Segment")

    def extract_beacon_info(self):
        # print("In 'extract_beacon_info' ")
        STATE = NOT_STARTED
        # Parse the data_packets, extract beacon signals and store them into Array format
        while self.received_beacon_signals:
            beacon = self.received_beacon_signals.pop(0)
            data_packet, timestamp = beacon[0], beacon[1]

            if data_packet[0:4] == b'ULMS':
                STATE = CYCLE_STARTED
                self._cycle_start_time = timestamp
                self.intermediate_beacon_signals = []
                # print("Cycle Started")
                continue

            if STATE >= CYCLE_STARTED:
                if data_packet[0:4] == b'ULMB':
                    STATE = SEGMENT_STARTED
                    self.handle_start_of_beacon_segment(data_packet, timestamp)
                    # print("Start of Beacon Segment")
                    continue

                if STATE >= SEGMENT_STARTED:
                    if data_packet[0:4] == b'ULAB':
                        STATE = SEGMENT_CONTINUED
                        self.handle_continue_of_beacon_segement(data_packet, timestamp)
                        # print("Continue of Beacon Segment")

                if data_packet[0:4] == b'ULME':
                    STATE = CYCLE_ENDED
                    self._cycle_end_time = timestamp
                    # print("End of Beacon Cycle")

                    # store the previous beacon_segment if any into intermediate_beacon_signals
                    if self.current_beacon_segment:
                        self.intermediate_beacon_signals.append(self.current_beacon_segment)
                        # print("Stuffed Current Beacon Segement")

                    break

    def store_beacon_into_array(self):
        # handle segment by segment
        self.final_beacon_signals = []
        while self.intermediate_beacon_signals:
            segment = self.intermediate_beacon_signals.pop(0)
            if len(segment) >= 2:
                self.final_beacon_signals.append(segment)

    def process_beacon_signals_and_estimate_location(self):
        # print("\n Received beacon signals:", self.received_beacon_signals)
        # First:    extract info from raw beacon signals
        self.extract_beacon_info()

        # Second:   store beacon info into array format
        self.store_beacon_into_array()

        # if self._extern_sensor:
        #     # Read Pressure Sensor (External Sensor Payload) to get Depth measurement
        #     # Switch ON the External Sensor Payload and initialize the i2c bus and sensor module
        #     self._extern_sensor.start_acquisition()
        #     while not self._extern_sensor.is_completed():
        #         # Acquire the sensor reading
        #         self._extern_sensor.process_acquisition()
        #     # Get read the last measurement
        #     self._depth = self._extern_sensor.get_latest_depth_as_float(self._fluid_density)
        #     # Switch OFF power to external sensor payload and release the i2c bus
        #     self._extern_sensor.stop_acquisition()
        #     print("Got Depth from Sensor:", self._depth)

        # Third:    estimate the location using beacon info
        if self._sound_speed is None or self._depth is None:
            raise Exception("Sound Speed and Sensor Depth should be updated!")
        """
        tdoa = TDOALocalization(self._sound_speed, self._depth)
        dt1 = datetime.now()
        estimated_location = tdoa.estimate_location(self.final_beacon_signals)
        dt2 = datetime.now()
        """
        print("\n")
        # print("Going to estimate location:")
        # print("Sound speed:", self._sound_speed, " Depth:", self._depth)
        # print("Final Beacon Signals:", self.final_beacon_signals)

        dt1 = datetime.now()
        self._current_location = estimate_location(self._sound_speed, self._depth, self.final_beacon_signals)
        dt2 = datetime.now()

        print("Estimated Location:", self._current_location)
        print("Total time taken (s): ", (dt2-dt1).total_seconds())
