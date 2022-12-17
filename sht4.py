import logging
from . import bus
from struct import unpack_from


REPORT_TIME = .8

DEFAULT_ADDR = 0x44  # Factory SHT4 I2C Address
RESET = 0x94  # SOFTRESET
HIGH_PRECISION_MODE = 0xFD
REQUEST_CHIPID = 0x89


class SHT4:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=DEFAULT_ADDR, default_speed=100000
        )
        self.mcu = self.i2c.get_mcu()
        self.min_temp = 0
        self.max_temp = 0
        self.temp = 0
        self.humidity = 0
        self.sample_timer = None
        self.max_sample_time = None
        self.printer.add_object("sht4 " + self.name, self)
        self.printer.register_event_handler(
            "klippy:connect", self.handle_connect)

    def handle_connect(self):
        self.chipID = self.get_chipID()
        # each chip is calibrated and unique id, check datasheet, need more research on this
        # if chip_id != CHIP_ID:
        #    logging.info("SHT4: Unknown Chip ID received %#x" % chip_id)
        # else:
        #    logging.info("SHT4: Found SHT4  at %#x" % (self.i2c.i2c_address))
        # if it outputs zero, it means there is a crc error
        
        #fix below
        if self.chipID != 0:
            logging.info("SHT4: Found SHT4  at %#x, chipID %#x" %
                         (self.i2c.i2c_address, self.chipID))
        self.reset()
        self.sample_timer = self.reactor.register_timer(self.sample_sensor)
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def sample_sensor(self, eventtime):
        #need to test timing!
        #self.reactor.pause(self.reactor.monotonic() + .1)
        self.recv = self.get_measurements()
        self.temp_data = self.recv[0:2]
        self.temp_crc = self.recv[2]
        self.humidity_data = self.recv[3:5]
        self.humidity_crc = self.recv[5]
        if self.temp_crc == self._crc8(self.temp_data) and self.humidity_crc == self._crc8(self.humidity_data):
            self.raw_temp = unpack_from(">H", self.temp_data)[0]
            self.temp = -45.0 + 175.0 * self.raw_temp / 65535.0
            self.raw_humidity = unpack_from(">H", self.humidity_data)[0]
            self.raw_humidity = -6.0 + 125.0 * self.raw_humidity / 65535.0
            self.humidity = max(min(self.raw_humidity, 100), 0)
        else:
            logging.info("SHT4: crc error SHT4 at %#x" % (self.i2c.i2c_address))
        #self.reactor.pause(self.reactor.monotonic() + 0.1)
        measured_time = self.reactor.monotonic()
        self._callback(self.mcu.estimated_print_time(measured_time), self.temp)
        return measured_time + REPORT_TIME

    def get_chipID(self):
        data = [REQUEST_CHIPID]
        self.reactor.pause(self.reactor.monotonic() + .1)
        self.i2c.i2c_write(data)
        self.reactor.pause(self.reactor.monotonic() + .1)
        data = []
        recv = self.i2c.i2c_read(data, 6)
        ser1 = bytearray(recv['response'])[0:2]
        ser1_crc = bytearray(recv['response'])[2]
        ser2 = bytearray(recv['response'])[3:5]
        ser2_crc = bytearray(recv['response'])[5]
        if ser1_crc != self._crc8(ser1) or ser2_crc != self._crc8(ser2):
            serial = 0
        else:
            serial = (ser1[0] << 24) + (ser1[1] << 16) + \
                (ser2[0] << 8) + ser2[1]
        return serial

    def get_measurements(self):
        data = [HIGH_PRECISION_MODE]  # first mode - High precision
        # testing required here
        self.reactor.pause(self.reactor.monotonic() + .1)
        self.i2c.i2c_write(data)
        self.reactor.pause(self.reactor.monotonic() + .1)
        data = []
        recv = self.i2c.i2c_read(data, 6)
        return bytearray(recv['response'])

    def reset(self):
        self.reactor.pause(self.reactor.monotonic() + .1)
        data = [RESET]  # softreset
        logging.info("SHT4: sending_reset")
        self.i2c.i2c_write(data)
        self.reactor.pause(self.reactor.monotonic() + .1)

    def get_status(self, eventtime):
        data = {'temperature': round(self.temp, 2)}
        data['humidity'] = self.humidity
        return data

    def _crc8(self, buffer): #verify crc8, check datasheet
        crc = 0xFF
        for byte in buffer:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        return crc & 0xFF  # return the bottom 8 bits


def load_config(config):
    # Register sensor
    logging.info("SHT4: init")
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("SHT4", SHT4)
