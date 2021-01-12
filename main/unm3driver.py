#! /usr/bin/env python
#
# MicroPython Driver for NM3
#
# This file is part of nm3-micropython-pybd derived from NM3 Python Driver. 
# https://github.com/bensherlock/nm3-micropython-pybd
# https://github.com/bensherlock/nm3-python-driver
#
#
# MIT License
#
# Copyright (c) 2020 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
"""MicroPython driver for the NM3."""

from collections import deque
import time


class MessagePacket:
    """NM3 Message Packet Structure."""

    # Packet Type "Enums"
    PACKETTYPE_BROADCAST, PACKETTYPE_UNICAST = 'B', 'U'

    PACKETTYPE_NAMES = {
        PACKETTYPE_BROADCAST: 'Broadcast',
        PACKETTYPE_UNICAST: 'Unicast',
    }

    PACKETTYPES = (PACKETTYPE_BROADCAST, PACKETTYPE_UNICAST)

    def __init__(self):
        self._source_address = None
        self._destination_address = None
        self._packet_type = None
        self._packet_payload = None
        self._packet_timestamp_count = None

        # Timestamps for various uses
        self._timestamp = None
        self._timestamp_millis = None
        self._timestamp_micros = None

    def __call__(self):
        return self

    @property
    def source_address(self) -> int:
        """Get the source address."""
        return self._source_address

    @source_address.setter
    def source_address(self,
                       source_address: int):
        """Set the the source address (0-255)."""
        if source_address and (source_address < 0 or source_address > 255):
            raise ValueError('Invalid Address Value (0-255): {!r}'.format(source_address))
        self._source_address = source_address

    @property
    def destination_address(self) -> int:
        """Get the destination address."""
        return self._destination_address

    @destination_address.setter
    def destination_address(self,
                            destination_address: int):
        """Set the the destination address (0-255)."""
        if destination_address and (destination_address < 0 or destination_address > 255):
            raise ValueError('Invalid Address Value (0-255): {!r}'.format(destination_address))
        self._destination_address = destination_address

    @property
    def packet_type(self):
        """Get the packet type (unicast, broadcast)."""
        return self._packet_type

    @packet_type.setter
    def packet_type(self,
                    packet_type):
        """Set the the packet type (unicast, broadcast)."""
        if packet_type not in self.PACKETTYPES:
            raise ValueError('Invalid Packet Type: {!r}'.format(packet_type))
        self._packet_type = packet_type

    @property
    def packet_payload(self):
        """Get the packet payload bytes."""
        return self._packet_payload

    @packet_payload.setter
    def packet_payload(self,
                       packet_payload):
        """Set the the packet payload bytes ."""
        self._packet_payload = packet_payload

    @property
    def packet_timestamp_count(self):
        """Gets the packet timetuple count - an overflowing 32-bit counter at 24MHz."""
        return self._packet_timestamp_count

    @packet_timestamp_count.setter
    def packet_timestamp_count(self,
                               packet_timestamp_count):
        """Sets the packet timetuple count - an overflowing 32-bit counter at 24MHz."""
        self._packet_timestamp_count = packet_timestamp_count

    @property
    def timestamp(self):
        """Get the timetuple of packet arrival taken from a RTC."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, timestamp):
        """Set the timetuple of packet arrival taken from a RTC."""
        self._timestamp = timestamp

    @property
    def timestamp_millis(self) -> int:
        """Get the timetuple of packet arrival taken from a millisecond counter."""
        return self._timestamp_millis

    @timestamp_millis.setter
    def timestamp_millis(self, timestamp_millis: int):
        """Set the timetuple of packet arrival taken from a millisecond counter."""
        self._timestamp_millis = timestamp_millis

    @property
    def timestamp_micros(self) -> int:
        """Get the timetuple of packet arrival taken from a microsecond counter."""
        return self._timestamp_micros

    @timestamp_micros.setter
    def timestamp_micros(self, timestamp_micros: int):
        """Set the timetuple of packet arrival taken from a microsecond counter."""
        self._timestamp_micros = timestamp_micros

    def json(self):
        """Get the packet fields as a json object."""
        jason = {"SourceAddress": self._source_address,
                 "DestinationAddress": self._destination_address,
                 "PacketType": MessagePacket.PACKETTYPE_NAMES[self._packet_type] if self._packet_type else None,
                 "PayloadLength": len(self._packet_payload) if self._packet_payload else 0,
                 "PayloadBytes": self._packet_payload,
                 "PacketTimestampCount": self._packet_timestamp_count,
                 "Timestamp": "%d-%02d-%02dT%02d:%02d:%02d" % self._timestamp[:6] if self._timestamp else None,
                 "TimestampMillis": self._timestamp_millis,
                 "TimestampMicros": self._timestamp_micros
                 }
        return jason

class MessagePacketParser:
    """Message Packet Parser takes bytes and uses a state machine to construct
       MessagePacket structures"""

    PARSERSTATE_IDLE, PARSERSTATE_TYPE, \
    PARSERSTATE_ADDRESS, PARSERSTATE_LENGTH, \
    PARSERSTATE_PAYLOAD, PARSERSTATE_TIMESTAMPFLAG, PARSERSTATE_TIMESTAMP = range(7)

    PARSERSTATE_NAMES = {
        PARSERSTATE_IDLE: 'Idle',
        PARSERSTATE_TYPE: 'Type',
        PARSERSTATE_ADDRESS: 'Address',
        PARSERSTATE_LENGTH: 'Length',
        PARSERSTATE_PAYLOAD: 'Payload',
        PARSERSTATE_TIMESTAMPFLAG: 'TimestampFlag',
        PARSERSTATE_TIMESTAMP: 'Timestamp',
    }

    PARSERSTATES = (PARSERSTATE_IDLE, PARSERSTATE_TYPE,
                    PARSERSTATE_ADDRESS, PARSERSTATE_LENGTH,
                    PARSERSTATE_PAYLOAD, PARSERSTATE_TIMESTAMPFLAG, PARSERSTATE_TIMESTAMP)

    def __init__(self):
        self._parser_state = self.PARSERSTATE_IDLE
        self._current_message_packet = None
        self._current_byte_counter = 0
        self._current_integer = 0
        # Micropython needs a defined size of deque
        self._packet_queue = deque((), 10)

    def __call__(self):
        return self

    def reset(self):
        """Resets the parser state machine."""

        self._parser_state = self.PARSERSTATE_IDLE
        self._current_message_packet = None
        self._current_byte_counter = 0
        self._current_integer = 0

    def process(self, next_byte) -> bool:
        """Process the next byte. Returns True if a packet completes on this byte."""

        # Received Message Structures:
        # '#B25500' + payload bytes + '\r\n'
        # '#U00' + payload bytes + '\r\n'
        # Or for some NM3 firmware versions:
        # '#B25500' + payload bytes + 'T' + timetuple + '\r\n'
        # '#U00' + payload bytes + 'T' + timetuple + '\r\n'
        # Where timetuple is a 10 digit (fixed width) number representing a 32-bit counter value
        # on a 24 MHz clock which is latched when the synch waveform arrives

        return_flag = False

        #print('next_byte: ' + bytes([next_byte]).decode('utf-8'))

        if self._parser_state == self.PARSERSTATE_IDLE:

            if bytes([next_byte]).decode('utf-8') == '#':
                # Next state
                self._parser_state = self.PARSERSTATE_TYPE

        elif self._parser_state == self.PARSERSTATE_TYPE:

            if bytes([next_byte]).decode('utf-8') == 'B':
                self._current_message_packet = MessagePacket()
                self._current_message_packet.source_address = 0
                self._current_message_packet.destination_address = None
                self._current_message_packet.packet_type = MessagePacket.PACKETTYPE_BROADCAST
                self._current_message_packet.packet_payload = []
                self._current_message_packet.packet_timestamp_count = 0

                self._current_byte_counter = 3
                self._current_integer = 0
                self._parser_state = self.PARSERSTATE_ADDRESS

            elif bytes([next_byte]).decode('utf-8') == 'U':
                self._current_message_packet = MessagePacket()
                self._current_message_packet.source_address = None
                self._current_message_packet.destination_address = None
                self._current_message_packet.packet_type = MessagePacket.PACKETTYPE_UNICAST
                self._current_message_packet.packet_payload = []
                self._current_message_packet.packet_timestamp_count = 0

                self._current_byte_counter = 2
                self._current_integer = 0
                self._parser_state = self.PARSERSTATE_LENGTH

            else:
                # Unknown packet type
                self._parser_state = self.PARSERSTATE_IDLE

        elif self._parser_state == self.PARSERSTATE_ADDRESS:
            self._current_byte_counter = self._current_byte_counter - 1

            # Append the next ascii string integer digit
            self._current_integer = (self._current_integer * 10) + int(bytes([next_byte]).decode('utf-8'))

            if self._current_byte_counter == 0:
                self._current_message_packet.source_address = self._current_integer
                self._current_integer = 0
                self._current_byte_counter = 2
                self._parser_state = self.PARSERSTATE_LENGTH

        elif self._parser_state == self.PARSERSTATE_LENGTH:
            self._current_byte_counter = self._current_byte_counter - 1

            # Append the next ascii string integer digit
            self._current_integer = (self._current_integer * 10) + int(bytes([next_byte]).decode('utf-8'))

            if self._current_byte_counter == 0:
                self._current_byte_counter = self._current_integer
                self._parser_state = self.PARSERSTATE_PAYLOAD

        elif self._parser_state == self.PARSERSTATE_PAYLOAD:
            self._current_byte_counter = self._current_byte_counter - 1

            self._current_message_packet.packet_payload.append(next_byte)

            if self._current_byte_counter == 0:
                # Completed this packet
                #self._packet_queue.append(self._current_message_packet)
                #self._current_message_packet = None
                #return_flag = True
                self._parser_state = self.PARSERSTATE_TIMESTAMPFLAG

        elif self._parser_state == self.PARSERSTATE_TIMESTAMPFLAG:

            if bytes([next_byte]).decode('utf-8') == 'T':
                self._current_byte_counter = 10
                self._current_integer = 0
                self._parser_state = self.PARSERSTATE_TIMESTAMP
            else:
                # No timetuple on this message. Completed Packet
                self._packet_queue.append(self._current_message_packet)
                self._current_message_packet = None
                return_flag = True
                self._parser_state = self.PARSERSTATE_IDLE

        elif self._parser_state == self.PARSERSTATE_TIMESTAMP:
            self._current_byte_counter = self._current_byte_counter - 1

            # Append the next ascii string integer digit
            self._current_integer = (self._current_integer * 10) + int(bytes([next_byte]).decode('utf-8'))

            if self._current_byte_counter == 0:
                # Completed this packet
                self._current_message_packet.packet_timestamp_count = self._current_integer
                self._packet_queue.append(self._current_message_packet)
                self._current_message_packet = None
                return_flag = True
                self._parser_state = self.PARSERSTATE_IDLE

        else:
            # Unknown state
            self._parser_state = self.PARSERSTATE_IDLE

        return return_flag

    def has_packet(self) -> bool:
        """Has packets in the queue."""

        if self._packet_queue:
            return True

        return False


    def get_packet(self): # -> union[MessagePacket, None]:
        """Gets the next received packet or None if the queue is empty.
        """
        if not self._packet_queue:
            return None

        # Pop the packet from the queue
        packet = self._packet_queue.popleft()

        return packet


class Nm3ResponseParser:
    """Parser for responses to commands."""

    PARSERSTATE_IDLE, PARSERSTATE_STRING = range(2)

    PARSERSTATE_NAMES = {
        PARSERSTATE_IDLE: 'Idle',
        PARSERSTATE_STRING: 'String',
    }

    PARSERSTATES = (PARSERSTATE_IDLE, PARSERSTATE_STRING)

    def __init__(self):
        self._parser_state = self.PARSERSTATE_IDLE
        self._current_bytes = []
        self._current_byte_counter = 0
        self._delimiter_byte = ord('\n')
        self._has_response_flag = False

    def reset(self):
        """Resets the parser state machine."""
        self._parser_state = self.PARSERSTATE_IDLE
        self._current_bytes = []
        self._current_byte_counter = 0
        self._has_response_flag = False

    def set_delimiter_byte(self, delimiter_byte):
        self._delimiter_byte = delimiter_byte

    def process(self, next_byte) -> bool:
        """Process the next byte. Returns True if a response completes on this byte."""

        return_flag = False

        #print('next_byte: ' + bytes([next_byte]).decode('utf-8'))

        if self._parser_state == self.PARSERSTATE_IDLE:

            if (bytes([next_byte]).decode('utf-8') == '#') or (bytes([next_byte]).decode('utf-8') == '$'):
                # Next state
                self._current_bytes = []
                self._current_byte_counter = 0
                self._parser_state = self.PARSERSTATE_STRING

        elif self._parser_state == self.PARSERSTATE_STRING:
            self._current_bytes.append(next_byte)
            self._current_byte_counter = self._current_byte_counter + 1

            # Check delimiter
            if next_byte == self._delimiter_byte:
                self._has_response_flag = True
                return_flag = True
                self._parser_state = self.PARSERSTATE_IDLE

        else:
            # Unknown
            self._parser_state = self.PARSERSTATE_IDLE

        return return_flag

    def has_response(self):
        return self._has_response_flag

    def get_last_response_string(self):
        return bytes(self._current_bytes).decode('utf-8')

# Micropython needs a defined size of deque
# self._incoming_bytes_buffer = deque((), 300) # List/Deque of integers

# Also need a generic timeout function as micropython and python return different
# results for time.time(). Python returns a float. Micropython only returns an integer.
# https://docs.micropython.org/en/latest/library/utime.html
class TimeoutHelper:

    def get_timeout_time(self, offset_in_seconds: float):
        """Get a platform specific timeout time based on the provided offset in float seconds."""
        # micropython use the ms integers
        offset_in_milliseconds = int(offset_in_seconds * 1000.0)
        timeout_time = time.ticks_add(time.ticks_ms(), offset_in_milliseconds)
        return timeout_time

    def is_timedout(self, timeout_time):
        """A platform specific test of the timeout time."""
        # micropython ticks_diff(ticks1, ticks2) has the same meaning as ticks1 - ticks2
        return time.ticks_diff(time.ticks_ms(), timeout_time) >= 0


class Nm3:
    """NM3 Driver over input/output binary streams with non-blocking Read() and Write() functions."""

    RESPONSE_TIMEOUT = 0.5

    def __init__(self, input_stream, output_stream):
        """Constructor. input_stream and output_stream need to be (bytes) IO with
        non-blocking Read() and Write() binary functions."""

        self._input_stream = input_stream
        self._output_stream = output_stream
        # Micropython needs a defined size of deque
        self._incoming_bytes_buffer = deque((), 300) # List/Deque of integers
        self._received_packet_parser = MessagePacketParser()

    def __call__(self):
        return self

    def get_address(self) -> int:
        """Gets the NM3 Address (000-255)."""

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()
        timeout_helper = TimeoutHelper()

        # Write the command to the serial port
        cmd_string = '$?'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = timeout_helper.get_timeout_time(Nm3.RESPONSE_TIMEOUT)
        while awaiting_response and not timeout_helper.is_timedout(timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '#A255V21941\r\n'
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 5:
            return -1

        addr_string = resp_string[2:5]
        addr_int = int(addr_string)

        return addr_int

    def set_address(self,
                    address: int) -> int:
        """Sets the NM3 Address (000-255)."""

        # Checks on parameters
        if address < 0 or address > 255:
            print('Invalid address (0-255): ' + str(address))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()
        timeout_helper = TimeoutHelper()

        # Write the command to the serial port
        cmd_string = '$A' + '{:03d}'.format(address)
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = timeout_helper.get_timeout_time(Nm3.RESPONSE_TIMEOUT)
        while awaiting_response and not timeout_helper.is_timedout(timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '#A255\r\n' 7 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 5:
            return -1

        addr_string = resp_string[2:5]
        addr_int = int(addr_string)

        return addr_int

    def get_battery_voltage(self) -> float:
        """Gets the NM3 Battery Voltage."""

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()
        timeout_helper = TimeoutHelper()

        # Write the command to the serial port
        cmd_string = '$?'
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = timeout_helper.get_timeout_time(Nm3.RESPONSE_TIMEOUT)
        while awaiting_response and not timeout_helper.is_timedout(timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '#A255V21941\r\n'
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 11:
            return -1

        adc_string = resp_string[6:11]
        adc_int = int(adc_string)

        # Convert the ADC value to a float voltage. V = adc_int * 15 / 65536.
        voltage = float(adc_int) * 15.0 / 65536.0

        return voltage

    def send_ping(self,
                  address: int,
                  timeout: float = 5.0) -> float:
        """Sends a ping to the addressed node and returns the one way time of flight in seconds
           from this device to the node address provided.
        """

        # Checks on parameters
        if address < 0 or address > 255:
            print('Invalid address (0-255): ' + str(address))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()
        timeout_helper = TimeoutHelper()

        # Write the command to the serial port
        cmd_string = '$P' + '{:03d}'.format(address)
        cmd_bytes = cmd_string.encode('utf-8')
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = timeout_helper.get_timeout_time(Nm3.RESPONSE_TIMEOUT)
        while awaiting_response and not timeout_helper.is_timedout(timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '$P255\r\n' 7 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 5:  # E
            return -1

        # Now await the range or TO after 4 seconds
        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = timeout_helper.get_timeout_time(timeout)
        while awaiting_response and not timeout_helper.is_timedout(timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '#R255T12345\r\n' or '#TO\r\n' 13 or 5 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 11: # TO
            return -1

        time_string = resp_string[6:11]
        time_int = int(time_string)

        # Convert the time value to a float seconds. T = time_int * 31.25E-6.
        time = float(time_int) * 31.25E-6

        return time

    def send_broadcast_message(self,
                               message_bytes: bytes) -> int:
        """Sends a broadcast message of message_bytes. Maximum of 64 bytes.
        """

        # Checks on parameters
        if len(message_bytes) < 2 or len(message_bytes) > 64:
            print('Invalid length of message_bytes (2-64): ' + str(len(message_bytes)))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()
        timeout_helper = TimeoutHelper()

        # Write the command to the serial port
        cmd_string = '$B' + '{:02d}'.format(len(message_bytes))
        cmd_bytes = cmd_string.encode('utf-8') + message_bytes
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = timeout_helper.get_timeout_time(Nm3.RESPONSE_TIMEOUT)
        while awaiting_response and not timeout_helper.is_timedout(timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '$B00\r\n' 6 bytes

        # Check that it has received all the expected bytes. Return error if not.
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 4:
            return -1

        return len(message_bytes)

    def send_unicast_message(self,
                             address: int,
                             message_bytes: bytes) -> int:
        """Sends a unicast message of message_bytes to address. Maximum of 64 bytes.
        """

        # Checks on parameters
        if address < 0 or address > 255:
            print('Invalid address (0-255): ' + str(address))
            return -1

        if len(message_bytes) < 2 or len(message_bytes) > 64:
            print('Invalid length of message_bytes (2-64): ' + str(len(message_bytes)))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()
        timeout_helper = TimeoutHelper()

        # Write the command to the serial port
        cmd_string = '$U' + '{:03d}'.format(address) + '{:02d}'.format(len(message_bytes))
        cmd_bytes = cmd_string.encode('utf-8') + message_bytes
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = timeout_helper.get_timeout_time(Nm3.RESPONSE_TIMEOUT)
        while awaiting_response and not timeout_helper.is_timedout(timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '$U12300\r\n' 9 bytes

        # Check that it has received all the expected bytes. Return error if not.
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 7:
            return -1

        # If the address is invalid then returns 'E\r\n'

        return len(message_bytes)

    def send_unicast_message_with_ack(self,
                                      address: int,
                                      message_bytes: bytes,
                                      timeout: float = 5.0) -> float:
        """Sends a unicast message of message_bytes to address. Maximum of 64 bytes.
           Waits for Ack from the remote node and returns the time of flight in seconds.
        """

        # Checks on parameters
        if address < 0 or address > 255:
            print('Invalid address (0-255): ' + str(address))
            return -1

        if len(message_bytes) < 2 or len(message_bytes) > 64:
            print('Invalid length of message_bytes (2-64): ' + str(len(message_bytes)))
            return -1

        # Absorb any incoming bytes into the receive buffers to process later
        self.poll_receiver()

        response_parser = Nm3ResponseParser()
        timeout_helper = TimeoutHelper()

        # Write the command to the serial port
        cmd_string = '$M' + '{:03d}'.format(address) + '{:02d}'.format(len(message_bytes))
        cmd_bytes = cmd_string.encode('utf-8') + message_bytes
        # Check that it has written all the bytes. Return error if not.
        if self._output_stream.write(cmd_bytes) != len(cmd_bytes):
            print('Error writing command')
            return -1

        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = timeout_helper.get_timeout_time(Nm3.RESPONSE_TIMEOUT)
        while awaiting_response and not timeout_helper.is_timedout(timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '$M12300\r\n' 9 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 7:  # E
            return -1

        # Now await the range or TO after 4 seconds
        # Await the response
        response_parser.reset()
        awaiting_response = True
        timeout_time = timeout_helper.get_timeout_time(timeout)
        while awaiting_response and not timeout_helper.is_timedout(timeout_time):
            resp_bytes = self._input_stream.read()
            for b in resp_bytes:
                if response_parser.process(b):
                    # Got a response
                    awaiting_response = False
                    break

        if not response_parser.has_response():
            return -1

        # Expecting '#R255T12345\r\n' or '#TO\r\n' 13 or 5 bytes
        resp_string = response_parser.get_last_response_string()
        if not resp_string or len(resp_string) < 11: # TO
            return -1

        time_string = resp_string[6:11]
        time_int = int(time_string)

        # Convert the time value to a float seconds. T = time_int * 31.25E-6.
        time = float(time_int) * 31.25E-6

        return time

    def poll_receiver(self):
        """Check the input_stream (non-blocking) and place bytes into incoming buffer for processing.
        """

        resp_bytes = self._input_stream.read()
        while resp_bytes:
            for a_byte in resp_bytes:
                self._incoming_bytes_buffer.append(a_byte)

            resp_bytes = self._input_stream.read()

    def process_incoming_buffer(self,
                                max_bytes_count: int = 0) -> int:
        """Process the bytes stored in the incoming buffer up to the max_bytes_count (0=ignore).
           Returns the number of bytes still awaiting processing.
        """

        byte_count = 0
        while self._incoming_bytes_buffer:
            self._received_packet_parser.process(self._incoming_bytes_buffer.popleft())
            byte_count = byte_count + 1

            # Check for max_bytes_count
            if 0 < max_bytes_count <= byte_count:
                return len(self._incoming_bytes_buffer)

        return len(self._incoming_bytes_buffer)

    def has_received_packet(self) -> bool:
        """Has received packet in the queue."""

        return self._received_packet_parser.has_packet()

    def get_received_packet(self) -> MessagePacket:
        """Gets the next received packet or None if the queue is empty.
        """

        return self._received_packet_parser.get_packet()


def test_message_packet():
    """Tests on the MessagePacket."""
    import json

    message_packet = MessagePacket()

    # Empty Packet
    jason = message_packet.json()
    print(json.dumps(jason))

    # Populated Packet
    message_packet.source_address = 7
    message_packet.destination_address = 255
    message_packet.packet_type = MessagePacket.PACKETTYPE_UNICAST
    message_packet.packet_payload = [0, 1, 2, 3, 4, 5, 6, 7]
    message_packet.timestamp = time.localtime()
    jason = message_packet.json()
    print(json.dumps(jason))


def main():
    """Run tests on the driver code."""
    test_message_packet()


if __name__ == '__main__':
    main()
