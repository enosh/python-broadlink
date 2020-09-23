from typing import List

from .device import device
from .exceptions import check_error
from .helpers import calculate_crc16


class hysen(device):
    """Controls a Hysen HVAC."""

    def __init__(self, *args, **kwargs) -> None:
        """Initialize the controller."""
        device.__init__(self, *args, **kwargs)
        self.type = "Hysen heating controller"

    # Send a request
    # input_payload should be a bytearray, usually 6 bytes, e.g. bytearray([0x01,0x06,0x00,0x02,0x10,0x00])
    # Returns decrypted payload
    # New behaviour: raises a ValueError if the device response indicates an error or CRC check fails
    # The function prepends length (2 bytes) and appends CRC

    def send_request(self, input_payload: bytes) -> bytes:
        """Send a request to the device."""
        crc = calculate_crc16(input_payload)

        # first byte is length, +2 for CRC16
        request_payload = bytearray([len(input_payload) + 2, 0x00])
        request_payload.extend(input_payload)

        # append CRC
        request_payload.append(crc & 0xFF)
        request_payload.append((crc >> 8) & 0xFF)

        # send to device
        response = self.send_packet(0x6a, request_payload)
        check_error(response[0x22:0x24])
        response_payload = self.decrypt(response[0x38:])

        # experimental check on CRC in response (first 2 bytes are len, and trailing bytes are crc)
        response_payload_len = response_payload[0]
        if response_payload_len + 2 > len(response_payload):
            raise ValueError('hysen_response_error', 'first byte of response is not length')
        crc = calculate_crc16(response_payload[2:response_payload_len])
        if (response_payload[response_payload_len] == crc & 0xFF) and (
                response_payload[response_payload_len + 1] == (crc >> 8) & 0xFF):
            return response_payload[2:response_payload_len]
        raise ValueError('hysen_response_error', 'CRC check on response failed')

    def get_temp(self) -> int:
        """Return the room temperature in degrees celsius."""
        payload = self.send_request(bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x08]))
        return payload[0x05] / 2.0

    def get_external_temp(self) -> int:
        """Return the external temperature in degrees celsius."""
        payload = self.send_request(bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x08]))
        return payload[18] / 2.0

    def get_full_status(self) -> dict:
        """Return the state of the device.

        Timer schedule included.
        """
        payload = self.send_request(bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x16]))
        data = {}
        data['remote_lock'] = payload[3] & 1
        data['power'] = payload[4] & 1
        data['active'] = (payload[4] >> 4) & 1
        data['temp_manual'] = (payload[4] >> 6) & 1
        data['room_temp'] = (payload[5] & 255) / 2.0
        data['thermostat_temp'] = (payload[6] & 255) / 2.0
        data['auto_mode'] = payload[7] & 15
        data['loop_mode'] = (payload[7] >> 4) & 15
        data['sensor'] = payload[8]
        data['osv'] = payload[9]
        data['dif'] = payload[10]
        data['svh'] = payload[11]
        data['svl'] = payload[12]
        data['room_temp_adj'] = ((payload[13] << 8) + payload[14]) / 2.0
        if data['room_temp_adj'] > 32767:
            data['room_temp_adj'] = 32767 - data['room_temp_adj']
        data['fre'] = payload[15]
        data['poweron'] = payload[16]
        data['unknown'] = payload[17]
        data['external_temp'] = (payload[18] & 255) / 2.0
        data['hour'] = payload[19]
        data['min'] = payload[20]
        data['sec'] = payload[21]
        data['dayofweek'] = payload[22]

        weekday = []
        for i in range(0, 6):
            weekday.append(
                {'start_hour': payload[2 * i + 23], 'start_minute': payload[2 * i + 24], 'temp': payload[i + 39] / 2.0})

        data['weekday'] = weekday
        weekend = []
        for i in range(6, 8):
            weekend.append(
                {'start_hour': payload[2 * i + 23], 'start_minute': payload[2 * i + 24], 'temp': payload[i + 39] / 2.0})

        data['weekend'] = weekend
        return data

    # Change controller mode
    # auto_mode = 1 for auto (scheduled/timed) mode, 0 for manual mode.
    # Manual mode will activate last used temperature.
    # In typical usage call set_temp to activate manual control and set temp.
    # loop_mode refers to index in [ "12345,67", "123456,7", "1234567" ]
    # E.g. loop_mode = 0 ("12345,67") means Saturday and Sunday follow the "weekend" schedule
    # loop_mode = 2 ("1234567") means every day (including Saturday and Sunday) follows the "weekday" schedule
    # The sensor command is currently experimental
    def set_mode(self, auto_mode: int, loop_mode: int, sensor: int = 0) -> None:
        """Set the mode of the device."""
        mode_byte = ((loop_mode + 1) << 4) + auto_mode
        self.send_request(bytearray([0x01, 0x06, 0x00, 0x02, mode_byte, sensor]))

    # Advanced settings
    # Sensor mode (SEN) sensor = 0 for internal sensor, 1 for external sensor,
    # 2 for internal control temperature, external limit temperature. Factory default: 0.
    # Set temperature range for external sensor (OSV) osv = 5..99. Factory default: 42C
    # Deadzone for floor temprature (dIF) dif = 1..9. Factory default: 2C
    # Upper temperature limit for internal sensor (SVH) svh = 5..99. Factory default: 35C
    # Lower temperature limit for internal sensor (SVL) svl = 5..99. Factory default: 5C
    # Actual temperature calibration (AdJ) adj = -0.5. Prescision 0.1C
    # Anti-freezing function (FrE) fre = 0 for anti-freezing function shut down,
    #  1 for anti-freezing function open. Factory default: 0
    # Power on memory (POn) poweron = 0 for power on memory off, 1 for power on memory on. Factory default: 0
    def set_advanced(
        self,
        loop_mode: int,
        sensor: int,
        osv: int,
        dif: int,
        svh: int,
        svl: int,
        adj: float,
        fre: int,
        poweron: int,
    ) -> None:
        """Set advanced options."""
        input_payload = bytearray([0x01, 0x10, 0x00, 0x02, 0x00, 0x05, 0x0a, loop_mode, sensor, osv, dif, svh, svl,
                                   (int(adj * 2) >> 8 & 0xff), (int(adj * 2) & 0xff), fre, poweron])
        self.send_request(input_payload)

    # For backwards compatibility only.  Prefer calling set_mode directly.
    # Note this function invokes loop_mode=0 and sensor=0.
    def switch_to_auto(self) -> None:
        """Switch mode to auto."""
        self.set_mode(auto_mode=1, loop_mode=0)

    def switch_to_manual(self) -> None:
        """Switch mode to manual."""
        self.set_mode(auto_mode=0, loop_mode=0)

    # Set temperature for manual mode (also activates manual mode if currently in automatic)
    def set_temp(self, temp: float) -> None:
        """Set the target temperature."""
        self.send_request(bytearray([0x01, 0x06, 0x00, 0x01, 0x00, int(temp * 2)]))

    # Set device on(1) or off(0), does not deactivate Wifi connectivity.
    # Remote lock disables control by buttons on thermostat.
    def set_power(self, power: int = 1, remote_lock: int = 0) -> None:
        """Set the power state of the device."""
        self.send_request(bytearray([0x01, 0x06, 0x00, 0x00, remote_lock, power]))

    # set time on device
    # n.b. day=1 is Monday, ..., day=7 is Sunday
    def set_time(self, hour: int, minute: int, second: int, day: int) -> None:
        """Set the time."""
        self.send_request(bytearray([0x01, 0x10, 0x00, 0x08, 0x00, 0x02, 0x04, hour, minute, second, day]))

    # Set timer schedule
    # Format is the same as you get from get_full_status.
    # weekday is a list (ordered) of 6 dicts like:
    # {'start_hour':17, 'start_minute':30, 'temp': 22 }
    # Each one specifies the thermostat temp that will become effective at start_hour:start_minute
    # weekend is similar but only has 2 (e.g. switch on in morning and off in afternoon)
    def set_schedule(self, weekday: List[dict], weekend: List[dict]) -> None:
        """Set timer schedule."""
        # Begin with some magic values ...
        input_payload = bytearray([0x01, 0x10, 0x00, 0x0a, 0x00, 0x0c, 0x18])

        # Now simply append times/temps
        # weekday times
        for i in range(0, 6):
            input_payload.append(weekday[i]['start_hour'])
            input_payload.append(weekday[i]['start_minute'])

        # weekend times
        for i in range(0, 2):
            input_payload.append(weekend[i]['start_hour'])
            input_payload.append(weekend[i]['start_minute'])

        # weekday temperatures
        for i in range(0, 6):
            input_payload.append(int(weekday[i]['temp'] * 2))

        # weekend temperatures
        for i in range(0, 2):
            input_payload.append(int(weekend[i]['temp'] * 2))

        self.send_request(input_payload)

class tornado(device):
    """Controls a Tornado 16X SQ air conditioner."""
    def __init__(self, *args, **kwargs):
        device.__init__(self, *args, **kwargs)
        self.type = "Tornado air conditioner"
    
    def _decode(self, response) -> bytes:
        payload = self.decrypt(bytes(response[0x38:]))
        return payload
    
    def _calculate_checksum(self, packet:list, target:int=0x20017) -> tuple(int, int):
        """Calculate checksum of given array,
        by adding little endian words and subtracting from target.
        
        Args:
            packet (list/bytearray/bytes): 
        """
        result = target - (sum([v if i % 2 == 0 else v << 8 for i, v in enumerate(packet)]) & 0xffff)
        return (result & 0xff, (result >> 8) & 0xff)

    def _send_short_payload(self, payload:int) -> bytes:
        """Send a request for info from A/C unit and returns the response.
        0 = GET_AC_INFO, 1 = GET_STATES, 2 = GET_SLEEP_INFO, 3 = unknown function
        """
        
        if (payload == 0):
            packet = bytearray([0x0c, 0x00, 0xbb, 0x00, 0x06, 0x80, 0x00, 0x00, 0x02, 0x00, 0x21, 0x01, 0x1b, 0x7e])
        elif (payload == 1):
            packet = bytearray([0x0c, 0x00, 0xbb, 0x00, 0x06, 0x80, 0x00, 0x00, 0x02, 0x00, 0x11, 0x01, 0x2b, 0x7e])
        elif (payload == 2):
            packet = bytearray([0x0c, 0x00, 0xbb, 0x00, 0x06, 0x80, 0x00, 0x00, 0x02, 0x00, 0x41, 0x01, 0xfb, 0x7d])
        elif (payload == 3):
            packet = bytearray(16)
            packet[0x00] = 0xd0
            packet[0x01] = 0x07
        else:
            pass

        response = self.send_packet(0x6a, packet)
        check_error(response[0x22:0x24])
        return (self._decode(response))

    def get_state(self, payload:bool=None) -> dict:
        """Returns a dictionary with the unit's parameters.
        
        Args:
            payload (Optional[bool]): add the received payload for debugging
        
        Returns:
            dict:
                state (bool): power
                set_temp (float): temperature set point 16<n<32
                mode (str): Cooling, Heating, Fan, Dry, Auto (use first letter)
                speed (str): Mute, Low, Mid, High, Turbo (available only in cooling) (first letter and Mu for Mute)
                swing_h (str): ON, OFF
                swing_v (str): ON, OFF, 1, 2, 3, 4, 5 (fixed positions)
                sleep (bool): 
                display (bool):
                health (bool):
        """
        payload = self._send_short_payload(1)
        assert(len(payload) == 32)
        data = {}
        data['state'] = True if (payload[0x14] & 0x20) == 0x20 else False
        data['set_temp'] = (payload[0x0c] >> 3) + 8 + (0.0 if ((payload[0xe] & 0b10000000) == 0) else 0.5)

        swing_v = payload[0x0c] & 0b111
        swing_h = (payload[0x0d] & 0b11100000) >> 5
        if (swing_h == 0b111):
            data['swing_h'] = 'OFF'
        elif (swing_h == 0b111):
            data['swing_h'] = 'OFF'
        else:
            data['swing_h'] = 'unrecognized value'
        
        if (swing_v == 0b111):
            data['swing_v'] = 'OFF'
        elif (swing_v == 0b000):
            data['swing_v'] = 'ON'
        elif (swing_v >= 0 and swing_v <=5):
            data['swing_v'] = str(swing_v)
        else:
            data['swing_v'] = 'unrecognized value'

        mode = payload[0x11] >> 3 << 3
        if mode == 0x00:
            data['mode'] = 'A'
        elif mode == 0x20:
            data['mode'] = 'C'
        elif mode == 0x40:
            data['mode'] = 'D'
        elif mode == 0x80:
            data['mode'] = 'H'
        elif mode == 0xc0:
            data['mode'] = 'F'
        else:
            data['mode'] = 'unrecognized value'

        speed_L = payload[0x0f]
        speed_R = payload[0x10] 
        if speed_L == 0x60 and speed_R == 0x00:
            data['speed'] = 'L'
        elif speed_L == 0x40 and speed_R == 0x00:
            data['speed'] = 'M'
        elif speed_L == 0x20 and speed_R == 0x00:
            data['speed'] = 'H'
        elif speed_L == 0x40 and speed_R == 0x80:
            data['speed'] = 'Mu'
        elif speed_R == 0x40:
            data['speed'] = 'T'
        elif speed_L == 0xa0 and speed_R == 0x00:
            data['speed'] = 'A'
        else:
            data['speed'] = 'unrecognized value'

        data['sleep'] = True if (payload[0x1a] & 0b100 == 0b000) else False
        data['display'] = (payload[0x16] & 0x10 == 0x10)
        data['health'] = (payload[0x14] & 0b11 == 0b11)

        checksum = self._calculate_checksum(payload[:0x19]) # checksum=(payload[0x1a] << 8) + payload[0x19]

        if (payload[0x19] == checksum[0] and payload[0x1a] == checksum[1]):
            pass # success
        else:
            print('checksum fail', ['{:02x}'.format(x) for x in checksum])

        if (payload):
            data['received_payload'] = payload

        return data

    def get_ac_info(self, payload:bool=None) -> dict:
        """Returns dictionary with A/C info...
        Not implemented yet, except power state.

        Args:
            payload (Optional[bool]): add the received payload for debugging
        """
        payload = self._send_short_payload(0)

        # first 13 bytes are the same: 22 00 bb 00 07 00 00 00 18 00 01 21 c0
        data = {}
        data['state'] = True if (payload[0x0d] & 0b1 == 0b1) else False

        if (payload):
            data['received_payload'] = payload

        return data

    def set_advanced(self, state:bool=None, mode:str=None, set_temp:float=None, speed:str=None, swing_v:str=None, swing_h:str=None,
                        sleep:bool=None, display:bool=None, health:bool=None) -> bytes:
        """Set paramaters of unit and return response.
        If not all parameters are specificed, will try to derive from the unit's current state. Doesn't work when powered down.
        Args:
            state (bool): power
            set_temp (float): temperature set point 16<n<32
            mode (str): Cooling, Heating, Fan, Dry, Auto (use first letter)
            speed (str): Mute, Low, Mid, High, Turbo (available only in cooling) (use first letter and Mu for Mute)
            swing_h (str): ON, OFF
            swing_v (str): ON, OFF, 1, 2, 3, 4, 5 (fixed positions)
            sleep (bool): 
            display (bool):
            health (bool):
        """

        args = locals()
        args.pop('self')
        received_state = self.get_state()
        for k, v in args.items():
            if v == None:
                args[k] = received_state[k]
        
        PREFIX = [0x19, 0x00, 0xbb, 0x00, 0x06, 0x80, 0x00, 0x00, 0x0f, 0x00, 0x01, 0x01] # 12B
        MIDDLE = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # 13B + 2B checksum
        SUFFIX = [0, 0, 0, 0, 0] # 5B
        payload = PREFIX + MIDDLE + SUFFIX

        assert ((args['set_temp'] >= 16) and (args['set_temp'] <= 32) and ((args['set_temp'] * 2) % 1 == 0))

        if (args['swing_H'] == 'OFF'):
            swing_h = 0b111
        elif (args['swing_H'] == 'ON'):
            swing_h = 0b000
        else:
            raise ValueError('unrecognized swing horizontal value {}'.format(args['swing_H']))

        if (args['swing_V'] == 'OFF'):
            swing_h = 0b111
        elif (args['swing_V'] == 'ON'):
            swing_h = 0b000
        elif (args['swing_V'] >= 0 and args['swing_V'] <= 5):
            swing_v = str(args['swing_V'])
        else:
            raise ValueError('unrecognized swing vertical value {}'.format(args['swing_H']))

        if (speed == 'L'):
            speed_L, speed_R = 0x60, 0x00
        elif (speed == 'M'):
            speed_L, speed_R = 0x40, 0x00
        elif (speed  == 'H'):
            speed_L, speed_R = 0x20, 0x00
        elif (speed == 'Mu'):
            speed_L, speed_R = 0x40, 0x80
            assert (mode == 'F')
        elif (speed == 'T'):
            speed_R = 0x40
            speed_L = 0x20 # doesn't matter
        elif (speed == 'A'):
            speed_L, speed_R = 0xa0, 0x00
        else:
            raise ValueError('unrecognized speed value: {}'.format(speed))

        if (args['mode'] == 'A'):
            mode_1 = 0x00
        elif (args['mode'] == 'C'):
            mode_1 = 0x20
        elif (args['mode'] == 'D'):
            mode_1 = 0x40
        elif (args['mode'] == 'H'):
            mode_1 = 0x80
        elif (args['mode'] == 'F'):
            mode_1 = 0xc0
        else:
            raise ValueError('unrecognized mode value: {}'.format(mode))

        payload[0x0c] = ((int(args['set_temp']) - 8 << 3) | swing_h)
        payload[0x0d] = (swing_v << 5 | 0b100)
        payload[0x0e] = (0b10000000 if (args['set_temp'] % 1 == 0.5) else 0b0) | 0x2d
        payload[0x0f] = speed_L
        payload[0x10] = speed_R
        payload[0x11] = mode_1 | (0b100 if sleep else 0x0)
        # payload[0x12] = always 0x00
        # payload[0x13] = always 0x00
        payload[0x14] = (0b11 if args['health'] else 0b00) | (0b100000 if args['state'] else 0b000000)
        # payload[0x15] = always 0x00
        payload[0x16] = 0b10000 if args['display'] else 0b00000 # 0b_00 also changes
        # payload[0x17] = always 0x00
        payload[0x18] = 0b101 # either 0x00 or 0x05 - unclear on what it does
        
        # 0x19-0x1a - checksum
        checksum = self._calculate_checksum(payload[:0x19]) # checksum=(payload[0x1a] << 8) + payload[0x19]
        payload[0x19] = checksum[0]
        payload[0x1a] = checksum[1]

        response = self.send_packet(0x6a, bytearray(payload))
        check_error(response[0x22:0x24])
        response_payload = self._decode(response)
        return response_payload
