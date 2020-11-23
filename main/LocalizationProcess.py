import struct
try:
    from uac_localisation.main.TDOALocalization import TDOALocalization
    from uac_localisation.main.misc.datetime import datetime, timedelta
    use_rtc = True
except ImportError:
    from main.TDOALocalization import TDOALocalization
    from datetime import datetime, timedelta
    use_rtc = False


# STATES
NOT_STARTED = -1
CYCLE_STARTED = 0
SEGMENT_STARTED = 1
SEGMENT_CONTINUED = 2
SEGMENT_ENDED = 3
CYCLE_ENDED = 4

class LocalizationProcess:
    def __init__(self, soundspeed, depth):
        self.soundspeed = soundspeed
        self.depth = depth
        self.cycle_start_time = None
        self.cycle_end_time = None
        self.received_beacon_signals = []
        self.intermediate_beacon_signals = []
        self.final_beacon_signals = []
        self.current_beacon_segment = []
        self.current_beacon_segment_number = -1

    def handle_incoming_data_packet(self, data_packet: bytes, timestamp=None):
        """
        This function handles all incoming data packets and store them into a list
        for further processing. This function an external interface for Localization Process.
        It will be called from the Main Control Layer whenever a Localization related
        data packet is received by the Sensor node.
        :param data_packet:
        :param timestamp:
        :return: None
        """
        # Check first 4 bytes of the data_packet to decide its purpose
        if data_packet[0:4] == b'ULMS':  # Indicator for start of beacon signal cycle
            self.received_beacon_signals = []
            self.received_beacon_signals.append([data_packet, timestamp])
        elif data_packet[0:4] == b'ULMB' or data_packet[0:4] == b'ULAB':
            self.received_beacon_signals.append([data_packet, timestamp])  # store the beacon signal
        elif data_packet[0:4] == b'ULME':  # Indicator for end of beacon signal cycle
            self.received_beacon_signals.append([data_packet, timestamp])  # store the beacon signal
            # Process the beacon signals and estimate the location
            self.process_beacon_signals_and_estimate_location()
        else:
            pass

    def handle_start_of_beacon_segment(self, data_packet: bytes, rtc_datetime: tuple):
        """
        This function handle the beacon signal from Lead anchor.
        :param data_packet: contains beacon info
        :param rtc_datetime: beacon arrival time
        :return: None
        """
        try:
            self.current_beacon_segment_number = struct.unpack('B', data_packet[4:5])[0]
            m = struct.unpack('B', data_packet[5:6])[0]  # number of assistant anchors
            location_indicator = data_packet[6:7].decode('utf8')
            lat = struct.unpack('f', data_packet[7:11])[0]
            lon = struct.unpack('f', data_packet[11:15])[0]
            depth = struct.unpack('f', data_packet[15:19])[0]
            if use_rtc:
                #arrival_time = datetime.fromrtctodatetime(rtc_datetime).timestamp()  # convert rtc datetime to seconds.
                arrival_time = rtc_datetime.timestamp()  # convert rtc datetime to seconds.
            else:
                arrival_time = rtc_datetime.timestamp()  # convert rtc datetime to seconds.
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

    def handle_continue_of_beacon_segement(self, data_packet: bytes, rtc_datetime: tuple):
        """
        This function handle beacon signal from assistant anchors:
        :param data_packet: contains beacon info
        :param rtc_datetime: beacon arrival time
        :return: None
        """
        try:
            segment_number = struct.unpack('B', data_packet[4:5])[0]  # current beacon segment number
            location_indicator = data_packet[5:6].decode('utf8')
            lat = struct.unpack('f', data_packet[6:10])[0]  # number of assistant anchors
            lon = struct.unpack('f', data_packet[10:14])[0]
            depth = struct.unpack('f', data_packet[14:18])[0]
            delay_indicator = data_packet[18:19].decode('utf8')
            delay = struct.unpack('f', data_packet[19:23])[0]
            if use_rtc:
                #arrival_time = datetime.fromrtctodatetime(rtc_datetime).timestamp()  # convert rtc datetime to seconds.
                arrival_time = rtc_datetime.timestamp()
            else:
                arrival_time = rtc_datetime.timestamp()
        except Exception as e:
            print("Corrupted data packet: " + str(e) + "\n")
        else:
            if self.current_beacon_segment_number == segment_number:
                if location_indicator == 'L' and delay_indicator == 'D':
                    self.current_beacon_segment.append([lat, lon, depth, arrival_time, delay])

    def extract_beacon_info(self):
        STATE = NOT_STARTED
        # Parse the data_packets, extract beacon signals and store them into Array format
        while self.received_beacon_signals:
            beacon = self.received_beacon_signals.pop(0)
            data_packet, timestamp = beacon[0], beacon[1]

            if data_packet[0:4] == b'ULMS':
                STATE = CYCLE_STARTED
                self.cycle_start_time = timestamp
                self.intermediate_beacon_signals = []
                continue

            if STATE >= CYCLE_STARTED:
                if data_packet[0:4] == b'ULMB':
                    STATE = SEGMENT_STARTED
                    self.handle_start_of_beacon_segment(data_packet, timestamp)
                    continue

                if STATE >= SEGMENT_STARTED:
                    if data_packet[0:4] == b'ULAB':
                        STATE = SEGMENT_CONTINUED
                        self.handle_continue_of_beacon_segement(data_packet, timestamp)

                if data_packet[0:4] == b'ULME':
                    STATE = CYCLE_ENDED
                    self.cycle_end_time = timestamp

                    # store the previous beacon_segment if any into intermediate_beacon_signals
                    if self.current_beacon_segment:
                        self.intermediate_beacon_signals.append(self.current_beacon_segment)

                    break

    def store_beacon_into_array(self):
        # handle segment by segment
        self.final_beacon_signals = []
        while self.intermediate_beacon_signals:
            segment = self.intermediate_beacon_signals.pop(0)
            if len(segment) >= 2:
                self.final_beacon_signals.append(segment)

    def process_beacon_signals_and_estimate_location(self):
        # First:    extract info from raw beacon signals
        self.extract_beacon_info()
        # Second:   store beacon info into array format
        self.store_beacon_into_array()
        # Third:    estimate the location using beacon info
        tdoa = TDOALocalization(self.soundspeed, self.depth)
        dt1 = datetime.now()
        estimated_location = tdoa.estimate_location(self.final_beacon_signals)
        dt2 = datetime.now()
        print("Total Time Taken (s): ", (dt2-dt1).total_seconds())
