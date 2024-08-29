#!/usr/bin/env python
from time import sleep
from collections import namedtuple
import subprocess
import re
import platform
import os

from .stusb4500_types import (
    Version,
    PortStatus,
    PDO_Contract,
    PdoSinkFix,
    PdoSinkVar,
    PdoSinkBat,
    Rdo,
    Vbus,
)


class STUSB4500:
    """Check the STUSB4500
    :param address: STUSB4500 I2C Address
    :type address: int
    """

    def __init__(self, bus=None, address=0x28, reset_pin=4):
        self.addr = address
        self.bus = bus
        self.reset_pin = reset_pin
        self.where = platform.system()
        self.config = None

        self._initialize_bus()
        self._initialize_gpio()

    def _initialize_bus(self):
        """Инициализация I2C шины в зависимости от платформы."""
        try:
            if self.where == "Linux":
                self.bus = self._initialize_linux_bus()
            elif self.where == "Windows":
                from i2c_mp_usb import I2C_MP_USB as SMBus

                self.bus = SMBus()
            else:
                raise EnvironmentError("Платформа не поддерживается.")
        except Exception as e:
            print(f"Ошибка инициализации I2C шины: {e}")

    def _initialize_linux_bus(self):
        import smbus

        """Инициализация I2C шины для Linux."""
        if "raspberrypi" in platform.uname().machine.lower():
            return smbus.SMBus(
                self.bus or 1
            )  # I2C bus номер 1 для Raspberry Pi по умолчанию
        return smbus.SMBus(self._detect_i2c_bus())

    def _detect_i2c_bus(self):
        """Определение I2C шины для устройств на Linux."""
        try:
            p = subprocess.Popen(["i2cdetect", "-l"], stdout=subprocess.PIPE)
            for line in p.stdout:
                line = line.decode("utf-8")
                if "i2c-tiny-usb" in line:
                    match = re.search(r"i2c-(\d+)", line)
                    if match:
                        return int(match.group(1))
        except subprocess.SubprocessError as e:
            print(f"Ошибка определения I2C шины: {e}")
        return None

    def _initialize_gpio(self):
        """Инициализация GPIO для Raspberry Pi."""
        if self.is_raspberry_pi():
            try:
                import RPi.GPIO as gpio

                gpio.setwarnings(False)
                gpio.setmode(gpio.BCM)
                gpio.setup(self.reset_pin, gpio.OUT)
                self.gpio = gpio
            except ImportError:
                print("Модуль RPi.GPIO не установлен.")

    def is_raspberry_pi(self):
        """Проверка, является ли текущая платформа Raspberry Pi."""
        return os.path.exists("/proc/device-tree/model")

    def __read_byte(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def __write_byte(self, reg, data):
        return self.bus.write_byte_data(self.addr, reg, data)

    def hard_reset(self):
        self.gpio.output(self.reset_pin, self.gpio.HIGH)
        sleep(0.2)
        self.gpio.output(self.reset_pin, self.gpio.LOW)

    # 	\brief Reads typec revision and usbpd revision of the chip
    def version(self):
        BCD_TYPEC_REV_LOW = self.__read_byte(0x06)
        BCD_TYPEC_REV_HIGH = self.__read_byte(0x07)
        typec_rev = BCD_TYPEC_REV_HIGH << 8 | BCD_TYPEC_REV_LOW
        BCD_USBPD_REV_LOW = self.__read_byte(0x08)
        BCD_USBPD_REV_HIGH = self.__read_byte(0x09)
        usbpd_rev = BCD_USBPD_REV_HIGH << 8 | BCD_USBPD_REV_LOW
        Version = namedtuple("Version", "typec_rev, usbpd_rev")
        Version.__str__ = (
            lambda v: "Version(typec_rev="
            + hex(v.typec_rev)
            + ", usbpd_rev="
            + hex(v.usbpd_rev)
            + ")"
        )
        Version.__repr__ = Version.__str__

        return Version(typec_rev=typec_rev, usbpd_rev=usbpd_rev)

    # 	\brief Reads the current Status of the port (Sink-Source connection)
    def port_status(self):
        PORT_STATUS_0 = self.__read_byte(0x0D)
        PORT_STATUS_1 = self.__read_byte(0x0E)

        attachedDeviceAsString = [
            "None",
            "Sink",
            "Source",
            "Debug Accessory",
            "Audio Accessory",
            "Power Accessory",
        ]

        PortStatus = namedtuple(
            "PortStatus",
            "stateChanged, attachedDevice, lowPowerStandby, powerMode, dataMode, attached",
        )
        PortStatus.__str__ = (
            lambda ps: "PortStatus(stateChanged="
            + ("True" if ps.stateChanged == 1 else "False")
            + ", attachedDevice="
            + (
                attachedDeviceAsString[ps.attachedDevice]
                if (ps.attachedDevice >= 0 and ps.attachedDevice <= 5)
                else "undefined(" + str(ps.attachedDevice) + ")"
            )
            + ", lowPowerStandby="
            + ("standby mode" if ps.lowPowerStandby == 1 else "normal mode")
            + ", powerMode="
            + ("Source" if ps.powerMode == 1 else "Sink")
            + ", dataMode="
            + ("DFP" if ps.dataMode == 1 else "UFP")
            + ", attached="
            + ("True" if ps.attached == 1 else "False")
            + ")"
        )
        PortStatus.__repr__ = PortStatus.__str__
        return PortStatus(
            stateChanged=PORT_STATUS_0 & 0x01,
            attachedDevice=PORT_STATUS_1 >> 5 & 0x07,
            lowPowerStandby=PORT_STATUS_1 >> 4 & 0x01,
            powerMode=PORT_STATUS_1 >> 3 & 0x01,
            dataMode=PORT_STATUS_1 >> 2 & 0x01,
            attached=PORT_STATUS_1 & 0x01,
        )

    # 	\brief Reads the currently active PDO contract
    def active_contract(self):
        DPM_PDO_NUMB = self.__read_byte(0x70)
        PDO_Contract = namedtuple("PDO_Contract", "num")
        return PDO_Contract(num=DPM_PDO_NUMB & 3)

    # 	\brief Forces to use another PDO contract
    # 	The STUSB4500 offers 3 PDO contracts and take effect after a soft reset
    # 	\param newValue: New contract PDO1, PDO2 or PDO3
    def set_active_contract(self, newValue):
        # this is the active pdo contract. This takes effect after a sw reset (if source has the capability of the configured pdo)
        if newValue >= 0 and newValue < 4:
            return self.__write_byte(0x70, newValue)

    # 	\brief Reads all currently configured PDOs from the chip
    # 	REQ_SRC_CURRENT == unconstrainedPower
    def read_pdo(self):
        base_reg = 0x85
        bvalues = []  # byte values
        for reg in range(0x85, 0x91):
            bvalues.append(self.__read_byte(reg))

        supplyStr = ["Fixed", "Variable", "Battery"]

        PdoSinkFix = namedtuple(
            "PdoSinkFix",
            "current, voltage, fastRoleReqCur, dualRoleData, usbCommunicationsCapable, unconstrainedPower, higherCapability, dualRolePower, supply, raw",
        )
        PdoSinkFix.__str__ = (
            lambda ps: "PdoSink("
            + "voltage="
            + str(ps.voltage / 20.0)
            + "V"
            + ", current="
            + str(ps.current / 100.0)
            + "A"
            + ", fastRoleReqCur="
            + str(ps.fastRoleReqCur)
            + ", dualRoleData="
            + str(ps.dualRoleData)
            + ", usbCommunicationsCapable="
            + str(ps.usbCommunicationsCapable)
            + ", unconstrainedPower="
            + str(ps.unconstrainedPower)
            + ", higherCapability="
            + str(ps.higherCapability)
            + ", dualRolePower="
            + str(ps.dualRolePower)
            + ", supply="
            + (
                supplyStr[ps.supply]
                if ps.supply >= 0 and ps.supply < 3
                else "Undefined"
            )
            + ", raw=0x"
            + format(ps.raw, "08x")
            + ")"
        )
        PdoSinkFix.__repr__ = PdoSinkFix.__str__

        PdoSinkVar = namedtuple(
            "PdoSinkVar", "min_voltage, max_voltage, current, supply, raw"
        )
        PdoSinkVar.__str__ = (
            lambda ps: "PdoSink("
            + "voltage=["
            + str(ps.min_voltage / 20.0)
            + "V-"
            + str(ps.max_voltage / 20.0)
            + "V]"
            + ", current="
            + str(ps.current / 100.0)
            + "A"
            + ", supply="
            + (
                supplyStr[ps.supply]
                if ps.supply >= 0 and ps.supply < 3
                else "Undefined"
            )
            + ", raw=0x"
            + format(ps.raw, "08x")
            + ")"
        )
        PdoSinkVar.__repr__ = PdoSinkVar.__str__

        PdoSinkBat = namedtuple(
            "PdoSinkBat", "min_voltage, max_voltage, power, supply, raw"
        )
        PdoSinkBat.__str__ = (
            lambda ps: "PdoSink("
            + "voltage=["
            + str(ps.min_voltage / 20.0)
            + "V-"
            + str(ps.max_voltage / 20.0)
            + "V]"
            + ", power="
            + str(ps.power)
            + "W"
            + ", supply="
            + (
                supplyStr[ps.supply]
                if ps.supply >= 0 and ps.supply < 3
                else "Undefined"
            )
            + ", raw=0x"
            + format(ps.raw, "08x")
            + ")"
        )
        PdoSinkBat.__repr__ = PdoSinkBat.__str__

        pdo = {}
        for i in range(0, 3):
            reg = (
                bvalues[i * 4 + 3] << 24
                | bvalues[i * 4 + 2] << 16
                | bvalues[i * 4 + 1] << 8
                | bvalues[i * 4]
            )
            supply = reg >> 30 & 0x3
            if supply == 0:  #  fixed
                pdo[i + 1] = PdoSinkFix(
                    supply=supply,
                    dualRolePower=reg >> 29 & 0x1,
                    higherCapability=reg >> 28 & 0x1,
                    unconstrainedPower=reg >> 27 & 0x1,
                    usbCommunicationsCapable=reg >> 26 & 0x1,
                    dualRoleData=reg >> 25 & 0x1,
                    fastRoleReqCur=reg >> 23 & 0x3,
                    voltage=reg >> 10 & 0x3FF,
                    current=reg & 0x3FF,
                    raw=reg,
                )
            elif supply == 1:  # variable
                pdo[i + 1] = PdoSinkVar(
                    supply=supply,
                    max_voltage=reg >> 20 & 0x3FF,
                    min_voltage=reg >> 10 & 0x3FF,
                    current=reg & 0x3FF,
                    raw=reg,
                )
            elif supply == 2:  # battery
                pdo[i + 1] = PdoSinkBat(
                    supply=supply,
                    max_voltage=reg >> 20 & 0x3FF,
                    min_voltage=reg >> 10 & 0x3FF,
                    power=reg & 0x3FF,
                    raw=reg,
                )
        return pdo

    # 	\brief Reads and then prints the Power Data Object
    def print_pdo(self):
        for k, v in self.read_pdo().items():
            print("PDO#" + str(k) + ": ", v)

    # 	\brief Read out the Requested Data Object (RDO)
    def read_rdo(self):
        base_reg = 0x91
        bvalues = []  # byte values
        for reg in range(0x91, 0x95):
            bvalues.append(self.__read_byte(reg))

        requested_voltage = self.__read_byte(0x21)  # *100mV
        requested_voltage /= 10.0  # I want it in Volt not milli volt

        reg = bvalues[3] << 24 | bvalues[2] << 16 | bvalues[1] << 8 | bvalues[0]
        Rdo = namedtuple(
            "RDO",
            "voltage, current, maxCurrent, unchunkedMess_sup, usbSuspend, usbComCap, capaMismatch, giveBack, objectPos, raw",
        )
        Rdo.__str__ = (
            lambda ps: "RDO("
            + "voltage="
            + str(requested_voltage)
            + "V"
            + ", current="
            + str(ps.current / 100.0)
            + "A"
            + ", maxCurrent="
            + str(ps.maxCurrent / 100.0)
            + "A"
            + ", unchunkedMess_sup="
            + str(ps.unchunkedMess_sup)
            + ", usbSuspend="
            + str(ps.usbSuspend)
            + ", usbComCap="
            + str(ps.usbComCap)
            + ", capaMismatch="
            + str(ps.capaMismatch)
            + ", giveBack="
            + str(ps.giveBack)
            + ", objectPos="
            + str(ps.objectPos)
            + ", raw=0x"
            + format(ps.raw, "08x")
            + ")"
        )
        Rdo.__repr__ = Rdo.__str__
        return Rdo(
            voltage=requested_voltage,
            objectPos=reg >> 28 & 0x7,
            giveBack=reg >> 27 & 0x1,
            capaMismatch=reg >> 26 & 0x1,
            usbComCap=reg >> 25 & 0x1,
            usbSuspend=reg >> 24 & 0x1,
            unchunkedMess_sup=reg >> 23 & 0x1,
            current=reg >> 10 & 0x3FF,
            maxCurrent=reg & 0x3FF,
            raw=reg,
        )

    # 	\brief Perform a software reset
    # 	RESET_CTRL Register @0x23 bit 0 = {1 := reset, 0 := no reset}
    def reset(self):
        RESET_CTRL = self.__read_byte(0x23)
        self.__write_byte(0x23, RESET_CTRL | 0x01)
        sleep(0.25)
        self.__write_byte(0x23, RESET_CTRL & ~0x01)

    # 	\param num: PDO to be changed (1..3)
    # 	\param volt: Desired voltage in mV
    # 	\param current: Desired current in mA
    def set_pdo(self, num, volt, current):
        if num > 0 and num < 4:
            reg32 = (int(current / 10) & 0x3FF) | int(volt / 50) << 10  # | (1 << 29)
            print(reg32)
            self.__write_byte(0x85 + (num - 1) * 4, reg32 & 0xFF)
            self.__write_byte(0x86 + (num - 1) * 4, (reg32 >> 8) & 0xFF)
            # self.__write_byte( 0x87 + (num - 1) * 4, (reg32 >> 16) & 0xff)
            # self.__write_byte( 0x88 + (num - 1) * 4, (reg32 >> 24) & 0xff)
        else:
            print(num, " is no valid pdo!")

    # 	\brief Configures a PDO with a variable Voltage/Current
    # 	\param pdo_num: PDO to be configured
    # 	\param current: Desired Current in mA
    # 	\param min_voltage: Min Voltage in mV
    # 	\param max_voltage: Max Voltage in mV
    def set_pdo_variable(self, pdo_num, current, min_voltage, max_voltage):
        if pdo_num > 1 and pdo_num <= 3:
            if (
                min_voltage >= 5000
                and min_voltage <= 20000
                and max_voltage >= 5000
                and max_voltage <= 20000
                and min_voltage <= max_voltage
            ):
                reg32 = 1 << 30  # variable supply
                reg32 |= int(current / 10)
                reg32 |= int(min_voltage / 50) << 10  # min voltage
                reg32 |= int(min_voltage / 50) << 20  # max voltage

                self.__write_byte(0x85 + (pdo_num - 1) * 4, reg32 & 0xFF)
                self.__write_byte(0x86 + (pdo_num - 1) * 4, (reg32 >> 8) & 0xFF)
                self.__write_byte(0x87 + (pdo_num - 1) * 4, (reg32 >> 16) & 0xFF)
                self.__write_byte(0x88 + (pdo_num - 1) * 4, (reg32 >> 24) & 0xFF)
        elif pdo_num == 1:
            print("PDO#1 cannot have a variable supply")

    # 	\brief Unlocks the internal NVM registers
    # 	\param lock: Unlocks if False, locks if True
    def nvm_lock(self, lock):
        if lock is False:
            self.__write_byte(0x95, 0x47)
        else:
            self.__write_byte(0x95, 0x00)

    # 	\brief Dumps the NVM
    # 	\info factory nvm dump
    # 		00 00 b0 aa 00 45 00 00 (0xc0-0xc7 hidden)
    # 		10 40 9c 1c ff 01 3c df (0xc8-0xcf)
    # 		02 40 0f 00 32 00 fc f1 (0xd0-0xd7)
    # 		00 19 56 af f5 35 5f 00 (0xd8-0xdf)
    # 		00 4b 90 21 43 00 40 fb (0xe0-0xe7)
    def nvm_dump(self):
        self.nvm_lock(False)  # unlock NVM

        def nvm_wait_for_execution():
            while True:
                reg8 = self.__read_byte(0x96)
                if reg8 & 0x10 == 0x00:
                    break

        self.config = []
        for num_sector in range(0, 5):
            # send command opcode READ(0x00) to FTP_CTRL_1(0x97)
            self.__write_byte(0x97, 0 & 0x07)
            # execute command
            self.__write_byte(0x96, (num_sector & 0x07) | 0x80 | 0x40 | 0x10)
            nvm_wait_for_execution()
            # read 8 bytes that are copied from nvm to 0x53-0x5a
            sector = []
            for i in range(0, 8):
                sector.append(self.__read_byte(0x53 + i))
            self.config.append(sector)
        self.nvm_lock(True)  # lock NVM

        # nicely print out the values
        sec = 0
        for sector in self.config:
            line = "%d: [" % sec
            sec += 1
            for byte in sector:
                line += "0x" + format(byte, "02x") + ", "
            line = line[:-2]  # remove trailing comma
            line += "]"
            # print(line)
        return

    # 	Write procedure
    # 	Enter Write mode:
    # 	1. PASSWORD_REG(0x95) <= PASSWORD(0x47) to unlock flash
    # 	2. RW_BUFFER(0x53) <= 0 if partial erasing sectors
    # 	3. CTRL_0(0x96) <= PWR | RST_N to soft reset chip and power on
    # 	4. CTRL_1(0x97) <= SECTORS_TOBE_ERASED_MASK << 3 | WRITE_SER to send erase command for the specified sectors (1 hot encoded)
    # 	5. CTRL_0(0x96) <= PWR | RST_N | REQ to commit command in CTRL_1
    # 	6. Wait until REQ bit in CTRL_0 is cleared
    # 	7. CTRL_1(0x97) <= SOFT_PROG_SECTOR
    # 	8. CTRL_0(0x96) <= PWR | RST_N | REQ to commit command in CTRL_1
    # 	9. Wait until REQ bit in CTRL_0 is cleared
    # 	10. CTRL_1(0x97) <= ERASE_SECTOR
    # 	11. CTRL_0(0x96) <= PWR | RST_N | REQ to commit command in CTRL_1
    # 	12. Wait until REQ bit in CTRL_0 is cleared
    # 	Write Sector:
    # 	1. Write sector data into RW_BUFFER(0x53-0x5A)
    # 	2. CTRL_0(0x96) <= PWR | RST_N
    # 	3. CTRL_1(0x97) <= WRITE_PLR
    # 	4. CTRL_0(0x96) <= PWR | RST_N | REQ to commit command in CTRL_1
    # 	5. Wait until REQ bit in CTRL_0 is cleared
    # 	6. CTRL_1(0x97) <= PROG_SECTOR
    # 	7. CTRL_0(0x96) <= PWR | RST_N | REQ | SECTOR to commit command in CTRL_1
    # 	8. Wait until REQ bit in CTRL_0 is cleared
    # 	Exit Programming mode:
    # 	1. CTRL_0(0x96) <= RST_N
    # 	2. CTRL_1(0x97) <= 0
    # 	3. PASSWD(0x95) <= 0

    # 	\brief writes data into nvm
    # 	\param sector_data: dictionary where key is sector {0..4} and value is 8 bytes
    # 	\info: Requires a hard reset to take effect
    def nvm_write(self):
        pwr = 0x80
        rst_n = 0x40
        req = 0x10

        def nvm_wait_for_execution():
            while True:
                reg8 = self.__read_byte(0x96)
                if reg8 & 0x10 == 0x00:
                    break

        section_mask = 0
        # for k, v in sector_data.items():
        for k, v in enumerate(self.config):
            if k >= 0 and k <= 4:
                if len(v) == 8:
                    section_mask |= 1 << k
                else:
                    print("New sector data has to many bytes (sector: %d)" % k)
                    return
            else:
                print("Invalid sector %d" % k)
                return

        self.nvm_lock(False)
        # 	Erase specified sectors to be able to program them
        # self.__write_byte( 0x53, 0x00)
        # self.__write_byte( 0x96, pwr | rst_n)
        self.__write_byte(0x97, section_mask << 3 | 0x02)  # WRITE_SER opcode
        self.__write_byte(0x96, pwr | rst_n | req)
        nvm_wait_for_execution()
        self.__write_byte(0x97, 0x07)  # Soft_prog_sector opcode
        self.__write_byte(0x96, pwr | rst_n | req)
        nvm_wait_for_execution()
        self.__write_byte(0x97, 0x05)  # erase_sector opcode
        self.__write_byte(0x96, pwr | rst_n | req)
        nvm_wait_for_execution()
        # 	Write data to sectors
        # for k, v in sector_data.items():
        for k, v in enumerate(self.config):
            # write new data into rw_bufer@0x53
            rw_buffer = 0x53
            for byte in v:
                self.__write_byte(rw_buffer, byte)
                rw_buffer += 1
            self.__write_byte(0x97, 0x01)  # WRITE_PLR opcode
            self.__write_byte(0x96, pwr | rst_n | req)
            nvm_wait_for_execution()
            self.__write_byte(0x97, 0x06)  # PROG_SECTOR opcode
            self.__write_byte(0x96, pwr | rst_n | req | k)
            nvm_wait_for_execution()
        # 	Exit programming mode
        self.__write_byte(0x96, rst_n)
        self.__write_byte(0x97, 0)
        self.nvm_lock(True)

    # 	\brief These are the default values programmed by factory
    nvm_factory_defaults = {
        0: [0x00, 0x00, 0xB0, 0xAA, 0x00, 0x45, 0x00, 0x00],
        1: [0x10, 0x40, 0x9C, 0x1C, 0xFF, 0x01, 0x3C, 0xDF],
        2: [0x02, 0x40, 0x0F, 0x00, 0x32, 0x00, 0xFC, 0xF1],
        3: [0x00, 0x19, 0x56, 0xAF, 0xF5, 0x35, 0x5F, 0x00],
        4: [0x00, 0x4B, 0x90, 0x21, 0x43, 0x00, 0x40, 0xFB],
    }
    nvm_12v = {
        0: [0x00, 0x00, 0xB0, 0xAA, 0x00, 0x45, 0x00, 0x00],
        1: [0x00, 0x40, 0x9D, 0x1C, 0xFF, 0x01, 0x3C, 0xDF],
        2: [0x02, 0x40, 0x0F, 0x00, 0x32, 0x00, 0xFC, 0xF1],
        3: [0x00, 0x19, 0xBF, 0x55, 0x57, 0x55, 0x55, 0x00],
        4: [0x00, 0x2D, 0xF0, 0x20, 0x43, 0x00, 0x00, 0xFB],
    }

    def vbus_ctrl(self):
        # 	Defining types
        Vbus = namedtuple(
            "VBUS",
            "discharge_0v, discharge_trans, vbus_discharge, vsrc_discharge, sink_vbus_en",
        )
        Vbus.__str__ = (
            lambda v: "VBUS(discharge_time_transition="
            + str(v.discharge_trans * 24)
            + "ms"
            + ", discharge_to_0V="
            + str(v.discharge_0v * 84)
            + "ms"
            + ", vbus_discharge="
            + ("Enabled" if v.vbus_discharge else "Disabled")
            + ", vsrc_discharge="
            + ("Enabled" if v.vsrc_discharge else "Disabled")
            + ", sink_vbus_en="
            + ("Enabled" if v.sink_vbus_en else "Disabled")
            + ")"
        )
        Vbus.__repr__ = Vbus.__str__

        # 	Read out relevant registers
        VBUS_DISCHARGE_TIME_CTRL = self.__read_byte(0x25)
        VBUS_DISCHARGE_CTRL = self.__read_byte(0x26)
        VBUS_CTRL = self.__read_byte(0x27)

        # 	Map data to the type
        return Vbus(
            discharge_0v=VBUS_DISCHARGE_TIME_CTRL >> 4 & 0x0F,
            discharge_trans=VBUS_DISCHARGE_TIME_CTRL & 0x0F,
            vbus_discharge=True if VBUS_DISCHARGE_CTRL >> 7 & 0x01 else False,
            vsrc_discharge=True if VBUS_DISCHARGE_CTRL >> 6 & 0x01 else False,
            sink_vbus_en=True if VBUS_CTRL >> 1 & 0x01 else False,
        )

    def get_voltage(self, pdo):
        """
        Get output voltage for the specified PDO channel
        :param pdo: PDO Channel to retrieve voltage for
        :type pdo: int
        :return: Voltage in V
        :rtype: float
        """
        assert 1 <= pdo <= 3, "pdo channel not supported"

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        if pdo == 1:
            return 5
        elif pdo == 2:
            return self.config[4][1] * 0.2
        else:
            return (((self.config[4][3] & 0x03) << 8) + self.config[4][2]) * 0.05

    def get_current(self, pdo):
        """
        Get output current for the specified PDO channel
        :param pdo: PDO Channel to retrieve voltage for
        :type pdo: int
        :return: Current in A
        :rtype: float
        """
        assert 1 <= pdo <= 3, "pdo channel not supported"

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        if pdo == 1:
            cur_setting = (self.config[3][2] & 0xF0) >> 4
        elif pdo == 2:
            cur_setting = self.config[3][4] & 0x0F
        else:
            cur_setting = (self.config[3][5] & 0xF0) >> 4

        if cur_setting == 0:
            return 0
        elif cur_setting < 11:
            return cur_setting * 0.25 + 0.25
        else:
            return cur_setting * 0.50 - 2.50

    def get_lower_voltage_limit(self, pdo):
        """
        Get lower voltage lockout limit (5-20%)
        :param pdo: PDO Channel to retrieve voltage for
        :type pdo: int
        :return: Lower voltage limit in V
        :rtype: float
        """
        assert 1 <= pdo <= 3, "pdo channel not supported"

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        if pdo == 1:
            return 0
        elif pdo == 2:
            return (self.config[3][4] >> 4) + 5
        else:
            return (self.config[3][6] & 0x0F) + 5

    def get_upper_voltage_limit(self, pdo):
        """
        Get over voltage lockout limit (5-20%)
        :param pdo: PDO Channel to retrieve voltage for
        :type pdo: int
        :return: Upper voltage limit in V
        :rtype: float
        """
        assert 1 <= pdo <= 3, "pdo channel not supported"

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        if pdo == 1:
            return (self.config[3][3] >> 4) + 5
        elif pdo == 2:
            return (self.config[3][5] & 0x0F) + 5
        else:
            return (self.config[3][6] >> 4) + 5

    def get_flex_current(self):
        """
        Get the global current value common to all PDO numbers
        :return: Flex current in A
        :rtype: float
        """
        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        return (
            ((self.config[4][4] & 0x0F) << 6) + ((self.config[4][3] & 0xFC) >> 2)
        ) / 100.0

    def get_pdo_number(self):
        """
        Get current PDO number in use
        :return: PDO channel in use
        :rtype: int
        """
        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        return (self.config[3][2] & 0x06) >> 1

    def get_external_power(self):
        """
        Return the SNK_UNCONS_POWER parameter value
        0 - No external source of power
        1 - An external power source is available and is sufficient to power
                the system.
        :return: External power available
        :rtype: int
        """
        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        return (self.config[3][2] & 0x08) >> 3

    def get_usb_comm_capable(self):
        """
        Returns the USB_COMM_CAPABLE parameter value.
        0 - Sink does not support data communication
        1 - Sink does support data communication
        :return USB_COMM_CAPABLE parameter value
        :rtype: int
        """
        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        return self.config[3][2] & 0x01

    def get_config_ok_gpio(self):
        """
        Return the POWER_OK_CFG parameter value
        0 - Configuration 1
        1 - N/A
        2 - Configuration 2 (default)
        3 - Configuration 3

        Configuration 1:
        - VBUS_EN_SNK: Hi-Z - No source attached
                                          0 - Source attached
        - POWER_OK2:   Hi-Z - No functionality
        - POWER_OK3:   Hi-Z - No functionality

        Configuration 2 (defualt):
        - VBUS_EN_SNK: Hi-Z - No source attached
                                          0 - Source attached
        - POWER_OK2:   Hi-Z - No PD explicit contract
                                          0 - PD explicit contract with PDO2
        - POWER_OK3:   Hi-Z - No PD explicit contract
                                          0 - PD explicit contract with PDO3

        Configuration 3:
        - VBUS_EN_SNK: Hi-Z - No source attached
                                          0 - source attached
        - POWER_OK2:   Hi-Z - No source attached or source supplies default
                                                  USB Type-C current at 5V when source attached.
                                          0 - Source supplies 3.0A USB Type-C current at 5V
                                                  when source is attached.
        - POWER_OK3:   Hi-Z - No source attached or source supplies default
                                                  USB Type-C current at 5V when source attached.
                                          0 - Source supplies 1.5A USB Type-C current at 5V
                                                  when source is attached.

        :return: POWER_OK_CFG value
        :rtype: int
        """
        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        return (self.config[4][4] & 0x60) >> 5

    def get_gpio_ctrl(self):
        """
        Returns the GPIO pin configuration.

        0 - SW_CTRL_GPIO
        1 - ERROR_RECOVERY
        2 - DEBUG
        3 - SINK_POWER

        SW_CTRL_GPIO:
        - Software controlled GPIO. The output state is defined by the value
          of I2C register bit-0 at address 0x2D.

          Hi-Z - When bit-0 value is 0 (at start-up)
                 0 - When bit-0 value is 1

        ERROR_RECOVERY:
        - Hardware fault detection (i.e. overtemperature is detected, overvoltage is
          detected on the CC pins, or after a hard reset the power delivery communication
          with the source is broken).

          Hi-Z - No hardware fault detected
                 0 - Hardware fault detected

        DEBUG:
        - Debug accessory detection

          Hi-Z - No debug accessory detected
                 0 - debug accessory detected

        SINK_POWER:
        - Indicates USB Type-C current capability advertised by the source.

          Hi-Z - Source supplies defualt or 1.5A USB Type-C current at 5V
                 0 - Source supplies 3.0A USB Type-C current at 5V

        :return: GPIO pin configuration
        :rtype: int
        """
        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        return (self.config[1][0] & 0x30) >> 4

    def get_power_above_5v_only(self):
        """
        Returns the POWER_ONLY_ABOVE_5V parameter configuration.

        0 - VBUS_EN_SNK pin enabled when source is attached whatever VBUS_EN_SNK
                voltage (5V or any PDO voltage)
        1 - VBUS_EN_SNK pin enabled only when source attached and VBUS voltage
                negotiated to PDO2 or PDO3 voltage

        :return: POWER_ONLY_ABOVE_5V configuration
        :rtype: int
        """
        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        return (self.config[4][6] & 0x08) >> 3

    def get_req_src_current(self):
        """
        Return the REQ_SRC_CURRENT parameter configuration. In case of match, selects
        which operation current from the sink or the source is to be requested in the
        RDO message.

        0 - Request I(SNK_PDO) as operating current in RDO message
        1 - Request I(SRC_PDO) as operating current in RDO message

        :return: REQ_SRC_CURRENT parameter configuration
        :rtype: int
        """
        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        return (self.config[4][6] & 0x10) >> 4

    def set_voltage(self, pdo, voltage):
        """
        Set the voltage for the given PDO channel

        Note: PDO1 - Fixed at 5V
                  PDO2 - 5-20V, 20mV resolution
                  PDO3 - 5-20V, 20mV resolution

        :param pdo: PDO channel to set voltage for
        :type pdo: int
        :param voltage: Voltage to set
        :type voltage: float
        """
        # Voltage can only be in range of 5-20V
        if voltage < 5:
            voltage = 5
        elif voltage > 20:
            voltage = 20

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        if pdo == 1:
            # PDO1 is fixed at 5V, no change needed
            return
        elif pdo == 2:
            self.config[4][1] = int(voltage / 0.2)
        else:
            set_voltage = int(voltage / 0.05)
            self.config[4][2] = 0xFF & set_voltage
            self.config[4][3] &= 0xFC
            self.config[4][3] |= set_voltage << 8

    def set_current(self, pdo, current):
        """
        Set the current limit for the given PDO channel

        Note: Valid current values are:
        0.00*, 0.50, 0.75, 1.00, 1.25, 1.50, 1.75, 2.00,
        2.25, 2.50, 2.75, 3.00, 3.50, 4.00, 4.50, 5.00

        :param pdo: PDO channel to set current for
        :type pdo: int
        :param current: Current limit to set in A
        :type current: float
        """
        # Current from 0.5A-3.0A set in 0.25A steps
        # Current from 3.0A-5.0A set in 0.50A steps
        if current < 0.5:
            current = 0
        elif current <= 3:
            current = (4 * current) - 1
        else:
            current = (2 * current) + 5

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        if pdo == 1:
            self.config[3][2] &= 0x0F
            self.config[3][2] |= int(current) << 4
        elif pdo == 2:
            self.config[3][4] &= 0xF0
            self.config[3][4] |= int(current)
        else:
            self.config[3][5] &= 0x0F
            self.config[3][5] |= int(current) << 4

    def set_lower_voltage_limit(self, pdo, value):
        """
        Sets the under voltage lock out parameter for the PDO channels

        :param pdo: PDO channel to set
        :type pdo: int
        :param value: Under voltage coefficient (5-20%)
        :type value: int
        """
        if value < 5:
            value = 5
        elif value > 20:
            value = 20

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        if pdo == 1:
            # UVLO1 fixed
            return
        elif pdo == 2:
            self.config[3][4] &= 0x0F
            self.config[3][4] |= (value - 5) << 4
        else:
            self.config[3][4] &= 0xF0
            self.config[3][4] |= value - 5

    def set_upper_voltage_limit(self, pdo, value):
        """
        Sets the over voltage lock out parameter for each of the PDO channels

        :param pdo: PDO channel to set
        :type pdo: int
        :param value: Over voltage coefficient (5-20%)
        ":type value: int
        """
        if value < 5:
            value = 5
        elif value > 20:
            value = 20

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        if pdo == 1:
            self.config[3][3] &= 0x0F
            self.config[3][3] |= (value - 5) << 4
        elif pdo == 2:
            self.config[3][5] &= 0xF0
            self.config[3][5] |= value - 5
        elif pdo == 3:
            self.config[3][6] &= 0x0F
            self.config[3][6] |= (value - 5) << 4

    def set_flex_current(self, value):
        """
        Set the flexible current value common to all PDO channels

        :param value: Current value to set the FLEX_I parameter (0-5A)
        :type value: float
        """
        if value < 0:
            value = 0
        elif value > 5:
            value = 5

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        flex_value = value * 100

        self.config[4][3] &= 0x03
        self.config[4][3] |= (flex_value & 0x3F) << 2

        self.config[4][4] &= 0xF0
        self.config[4][4] |= (flex_value & 0x3C0) >> 6

    def set_pdo_number(self, value):
        """
        Set the number of sink PDOs

        0 - 1 PDO (5V only)
        1 - 1 PDO (5V only)
        2 - 2 PDOs (PDO2 has the highest priority, followed by PDO1)
        3 - 3 PDOs (PDO3 has the highest priority, followed by PDO2, and then PDO1).

        :param value: Number of sink PDOs
        :type value: int
        """
        assert 1 <= value <= 3

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        self.config[3][2] &= 0xF9
        self.config[3][2] |= value << 1

    def set_external_power(self, value):
        """
        Sets the SNK_UNCONS_POWER parameter value

        0 - No external source of power
        1 - An external power source is available and is sufficient to

        :param value: Value to set to SNK_UNCONS_POWER
        :type value: int
        """
        if value != 0:
            value = 1

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        self.config[3][2] &= 0xF7
        self.config[3][2] |= value

    def set_usb_comm_capable(self, value):
        """
        Sets the USB_COMM_CAPABLE parameter value

        0 - Sink does not support data communication
        1 - Sink does support data communication

        :param value: Value to set USB_COMM_CAPABLE
        :type value: int
        """
        if value != 0:
            value = 1

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        self.config[3][2] &= 0xFE
        self.config[3][2] |= value

    def set_config_ok_gpio(self, value):
        """
        Sets the POWER_OK_CFG parameter value.

        Parameter: value - Value to set to POWER_OK_CFG
        0 - Configuration 1
        1 - No applicable
        2 - Configuration 2 (default)
        3 - Configuration 3

        Configuration 1:
        - VBUS_EN_SNK: Hi-Z - No source attached
                                          0 - Source attached
        - POWER_OK2:   Hi-Z - No functionality
        - POWER_OK3:   Hi-Z - No functionality

        Configuration 2 (defualt):
        - VBUS_EN_SNK: Hi-Z - No source attached
                                          0 - Source attached
        - POWER_OK2:   Hi-Z - No PD explicit contract
                                          0 - PD explicit contract with PDO2
        - POWER_OK3:   Hi-Z - No PD explicit contract
                                          0 - PD explicit contract with PDO3

        Configuration 3:
        - VBUS_EN_SNK: Hi-Z - No source attached
                                          0 - source attached
        - POWER_OK2:   Hi-Z - No source attached or source supplies default
                                                  USB Type-C current at 5V when source attached.
                                          0 - Source supplies 3.0A USB Type-C current at 5V
                                                  when source is attached.
        - POWER_OK3:   Hi-Z - No source attached or source supplies default
                                                  USB Type-C current at 5V when source attached.
                                          0 - Source supplies 1.5A USB Type-C current at 5V
                                                  when source is attached.
        :param value: Configuration to set
        :type value: int
        """
        if value < 2:
            value = 0
        elif value > 3:
            value = 3

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        self.config[4][4] &= 0xCF
        self.config[4][4] |= value << 5

    def set_gpio_ctrl(self, value):
        """
        Sets the GPIO pin configuration.

        Paramter: value - Value to set to GPIO_CFG
        0 - SW_CTRL_GPIO
        1 - ERROR_RECOVERY
        2 - DEBUG
        3 - SINK_POWER

        SW_CTRL_GPIO:
        - Software controlled GPIO. The output state is defined by the value
          of I2C register bit-0 at address 0x2D.

          Hi-Z - When bit-0 value is 0 (at start-up)
                 0 - When bit-0 value is 1

        ERROR_RECOVERY:
        - Hardware fault detection (i.e. overtemperature is detected, overvoltage is
          detected on the CC pins, or after a hard reset the power delivery communication
          with the source is broken).

          Hi-Z - No hardware fault detected
                 0 - Hardware fault detected

        DEBUG:
        - Debug accessory detection

          Hi-Z - No debug accessory detected
                 0 - debug accessory detected

        SINK_POWER:
        - Indicates USB Type-C current capability advertised by the source.

          Hi-Z - Source supplies defualt or 1.5A USB Type-C current at 5V
                 0 - Source supplies 3.0A USB Type-C current at 5V

        :param value: GPIO configuration to set
        :type value: int
        """
        assert 0 <= value <= 3, "configuration not supported"

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        self.config[1][0] &= 0xCF
        self.config[1][0] |= value << 4

    def set_power_above_5v_only(self, value):
        """
        Sets the POWER_ONLY_ABOVE_5V parameter configuration.

        0 - VBUS_EN_SNK pin enabled when source is attached whatever VBUS_EN_SNK
                voltage (5V or any PDO voltage)
        1 - VBUS_EN_SNK pin enabled only when source attached and VBUS voltage
                negotiated to PDO2 or PDO3 voltage

        :param value: Value to select VBUS_EN_SNK pin configuration
        :type value: int
        """
        if value != 0:
            value = 1

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        self.config[4][6] &= 0xF7
        self.config[4][6] |= value << 3

    def set_req_src_current(self, value):
        """
        Sets the REQ_SRC_CURRENT parameter configuration. In case of match, selects
        which operation current from the sink or the source is to be requested in the
        RDO message.

        0 - Request I(SNK_PDO) as operating current in RDO message
        1 - Request I(SRC_PDO) as operating current in RDO message

        :param value: Value to set to REQ_SRC_CURRENT
        :type value: int
        """
        if value != 0:
            value = 1

        # Read the configuration if it is empty
        if self.config is None:
            self.nvm_dump()

        self.config[4][6] &= 0xEF
        self.config[4][6] |= value << 4


def main():
    addr = 0x28
    reset_pin = 4
    PD = STUSB4500()
    print(PD.read_rdo())
    PD.print_pdo()
    print(PD.active_contract())
    PD.nvm_dump()
    PD.set_voltage(2, 9.0)
    PD.set_current(2, 1.5)
    PD.set_voltage(3, 9.0)
    PD.set_current(3, 1.5)
    # PD.set_active_contract(2)
    # PD.set_pdo_number(2)
    PD.nvm_write()
    print(PD.read_rdo())

    # PDO#1:  PdoSink(voltage=5.0V, current=3.0A, fastRoleReqCur=0, dualRoleData=0, usbCommunicationsCapable=1, unconstrainedPower=1, higherCapability=1, dualRolePower=0, supply=Fixed)
    # PDO#2:  PdoSink(voltage=9.0V, current=2.0A, fastRoleReqCur=0, dualRoleData=0, usbCommunicationsCapable=0, unconstrainedPower=0, higherCapability=0, dualRolePower=0, supply=Fixed)
    # PDO#3:  PdoSink(voltage=12.0V, current=1.5A, fastRoleReqCur=0, dualRoleData=0, usbCommunicationsCapable=0, unconstrainedPower=0, higherCapability=0, dualRolePower=0, supply=Fixed)


if __name__ == "__main__":
    main()
