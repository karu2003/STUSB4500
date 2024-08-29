from collections import namedtuple

Version = namedtuple("Version", "typec_rev, usbpd_rev")
PortStatus = namedtuple(
    "PortStatus",
    "stateChanged, attachedDevice, lowPowerStandby, powerMode, dataMode, attached",
)
PDO_Contract = namedtuple("PDO_Contract", "num")
PdoSinkFix = namedtuple(
    "PdoSinkFix",
    "current, voltage, fastRoleReqCur, dualRoleData, usbCommunicationsCapable, unconstrainedPower, higherCapability, dualRolePower, supply, raw)"
)
PdoSinkVar = namedtuple("PdoSinkVar", "min_voltage, max_voltage, current, supply, raw")

PdoSinkBat = namedtuple("PdoSinkBat", "min_voltage, max_voltage, power, supply, raw")

Rdo = namedtuple(
    "RDO",
    "voltage, current, maxCurrent, unchunkedMess_sup, usbSuspend, usbComCap, capaMismatch, giveBack, objectPos, raw",
)
Vbus = namedtuple(
    "VBUS",
    "discharge_0v, discharge_trans, vbus_discharge, vsrc_discharge, sink_vbus_en",
)


def _format_version(v):
    return (
        "Version(typec_rev="
        + hex(v.typec_rev)
        + ", usbpd_rev="
        + hex(v.usbpd_rev)
        + ")"
    )


Version.__str__ = _format_version
Version.__repr__ = _format_version


def _format_port_status(ps):
    attachedDeviceAsString = [
        "None",
        "Sink",
        "Source",
        "Debug Accessory",
        "Audio Accessory",
        "Power Accessory",
    ]
    return (
        "PortStatus(stateChanged="
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


PortStatus.__str__ = _format_port_status
PortStatus.__repr__ = _format_port_status


def _format_pdo_sink_fix(ps):
    supplyStr = ["Fixed", "Variable", "Battery"]
    return (
        "PdoSink("
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
        + (supplyStr[ps.supply] if ps.supply >= 0 and ps.supply < 3 else "Undefined")
        + ", raw=0x"
        + format(ps.raw, "08x")
        + ")"
    )


PdoSinkFix.__str__ = _format_pdo_sink_fix
PdoSinkFix.__repr__ = _format_pdo_sink_fix


def _format_pdo_sink_var(ps):
    supplyStr = ["Fixed", "Variable", "Battery"]
    return (
        "PdoSink("
        + "voltage=["
        + str(ps.min_voltage / 20.0)
        + "V-"
        + str(ps.max_voltage / 20.0)
        + "V]"
        + ", current="
        + str(ps.current / 100.0)
        + "A"
        + ", supply="
        + (supplyStr[ps.supply] if ps.supply >= 0 and ps.supply < 3 else "Undefined")
        + ", raw=0x"
        + format(ps.raw, "08x")
        + ")"
    )


PdoSinkVar.__str__ = _format_pdo_sink_var
PdoSinkVar.__repr__ = _format_pdo_sink_var


def _format_pdo_sink_bat(ps):
    supplyStr = ["Fixed", "Variable", "Battery"]
    return (
        "PdoSink("
        + "voltage=["
        + str(ps.min_voltage / 20.0)
        + "V-"
        + str(ps.max_voltage / 20.0)
        + "V]"
        + ", power="
        + str(ps.power)
        + "W"
        + ", supply="
        + (supplyStr[ps.supply] if ps.supply >= 0 and ps.supply < 3 else "Undefined")
        + ", raw=0x"
        + format(ps.raw, "08x")
        + ")"
    )


PdoSinkBat.__str__ = _format_pdo_sink_bat
PdoSinkBat.__repr__ = _format_pdo_sink_bat


def _format_rdo(ps):
    return (
        "RDO("
        + "voltage="
        + str(ps.voltage)
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


Rdo.__str__ = _format_rdo
Rdo.__repr__ = _format_rdo


def _format_vbus(v):
    return (
        "VBUS(discharge_time_transition="
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


Vbus.__str__ = _format_vbus
Vbus.__repr__ = _format_vbus
