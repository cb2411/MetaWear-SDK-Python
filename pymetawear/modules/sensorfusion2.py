#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

.. moduleauthor:: mgeorgi <marcus.georgi@kinemic.de>

Created: 2016-02-01

"""

from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import re
import time
import logging
from functools import wraps
from ctypes import cast, POINTER

from pymetawear import libmetawear
from pymetawear.exceptions import PyMetaWearException
from pymetawear.mbientlab.metawear.cbindings import * 
from pymetawear.modules.base import PyMetaWearModule, Modules

log = logging.getLogger(__name__)
current_processor = None
waiting_for_processor = False
max_processor_wait_time = 5


def require_fusion_module(f):
    def wrapper(*args, **kwargs):
        if getattr(args[0], 'available', False) is False:
            raise PyMetaWearException("There is no Sensor Fusion "
                                      "module on your MetaWear board!")
        return f(*args, **kwargs)

    return wrapper


def require_bmi160(f):
    def wrapper(*args, **kwargs):
        if getattr(args[0], 'gyro_r_class', None) is None:
            raise PyMetaWearException("There is not Gyroscope "
                                      "module of your MetaWear board!")
        return f(*args, **kwargs)

    return wrapper


def require_bmm150(f):
    def wrapper(*args, **kwargs):
        if getattr(args[0], 'mag_p_class', None) is None:
            raise PyMetaWearException("There is not Magnetometer "
                                      "module on your MetaWear board!")
        return f(*args, **kwargs)

    return wrapper


class SensorFusionModule(PyMetaWearModule):
    """MetaWear accelerometer module implementation.

    :param ctypes.c_long board: The MetaWear board pointer value.
    :param int module_id: The module id of the sensorfusion
        component, obtained from ``libmetawear``.
    :param bool debug: If ``True``, module prints out debug information.

    """

    def __init__(self, board, module_id, debug=False):
        super(SensorFusionModule, self).__init__(board, debug)
        self.module_id = module_id

        acc_sensors = [
            Const.MODULE_ACC_TYPE_BMA255,  #3
            Const.MODULE_ACC_TYPE_BMI160,  #1
            Const.MODULE_ACC_TYPE_MMA8452Q #0
        ]
        acc_sensor_odr = [
            AccBma255Odr,
            AccBmi160Odr,
            AccMma8452qOdr
        ]
        acc_sensor_fsr = [
            AccMma8452qRange,
            AccBoschRange,
            AccBoschRange
        ]

        if self.module_id == Modules.MBL_MW_MODULE_NA:
            # No sensor fusion present!
            self.available = False
            # No gyroscope present! TODO: How to tell which is missing?
            self.gyro_r_class = None
            self.gyro_o_class = None
            # No magnetometer present!
            self.mag_o_class = None
            self.mag_p_class = None
        else:
            self.available = True
            self.gyro_r_class = GyroBmi160Range
            self.gyro_o_class = GyroBmi160Odr
            self.mag_o_class = MagBmm150Odr
            self.mag_p_class = MagBmm150Preset
            for count, a in enumerate(acc_sensors):
                if module_id == a:
                    self.acc_o_class = acc_sensor_odr[count]
                    self.acc_r_class = acc_sensor_fsr[count]

        self.board = board

        self.current_active_signal = None

        self._streams_to_enable = {
            SensorFusionData.CORRECTED_ACC: False,
            SensorFusionData.CORRECTED_GYRO: False,
            SensorFusionData.CORRECTED_MAG: False,
            SensorFusionData.QUATERION: False,
            SensorFusionData.EULER_ANGLE: False,
            SensorFusionData.GRAVITY_VECTOR: False,
            SensorFusionData.LINEAR_ACC: False,
        }

        self._data_source_signals = {
            SensorFusionData.CORRECTED_ACC: None,
            SensorFusionData.CORRECTED_GYRO: None,
            SensorFusionData.CORRECTED_MAG: None,
            SensorFusionData.QUATERION: None,
            SensorFusionData.EULER_ANGLE: None,
            SensorFusionData.GRAVITY_VECTOR: None,
            SensorFusionData.LINEAR_ACC: None,
        }

        self._callbacks = {}
        self._waiting_for_processor = False

        self.acc_odr = {}
        self.gyr_odr = {}
        self.mag_odr = {}
        self.acc_fsr = {}
        self.gyr_fsr = {}
        self.mag_power_presets = {}

        self.acc_current_odr = 0
        self.acc_current_fsr = 0

        acc_odr_class = None
        acc_fsr_class = None

        self.high_frequency_stream = False # TODO: Is this just for acc?

        # FOR ACCELEROMETER
        if self.acc_o_class is not None:
            # Parse possible output data rates for this accelerometer.
            for key, value in vars(self.acc_o_class).items():
                if re.search('^_([0-9]+)\_*([0-9]*)Hz', key) and key is not None:
                    self.acc_odr.update({key[1:-2].replace("_", "."):value})

        if self.acc_r_class is not None:
            # Parse possible output data ranges for this accelerometer.
            for key, value in vars(self.acc_r_class).items():
                if re.search('^_([0-9]+)G', key) and key is not None:
                    self.acc_fsr.update({key[1:-1]:value})

        # FOR GYROSCOPE
        if self.gyro_o_class is not None:
            # Parse possible output data rates for this gyroscope.
            for key, value in vars(self.gyro_o_class).items():
                if re.search('_([0-9]+)Hz', key) and key is not None:
                    self.gyr_odr.update({key[1:-2]: value})

        if self.gyro_r_class is not None:
            # Parse possible ranges for this gyroscope.
            for key, value in vars(self.gyro_r_class).items():
                if re.search('_([0-9]+)dps', key) and key is not None:
                    self.gyr_fsr.update({key[1:-3]: value})

        # FOR MAGNETOMETER
        if self.mag_p_class is not None:
            # Parse possible presets for this magnetometer.
            for key, value in vars(self.mag_p_class).items():
                if re.search('^([A-Z\_]*)', key) and isinstance(value,int):
                    self.mag_power_presets.update({key.lower():value})

        if self.mag_o_class is not None:
            # Parse possible data rates for this magnetometer.
            for key, value in vars(self.mag_o_class).items():
                if re.search('^_([0-9\_]*Hz)', key) and isinstance(value,int):
                    self.mag_odr.update({key:value})

        if debug:
            log.setLevel(logging.DEBUG)

    def __str__(self):
        return "{0}".format(self.module_name)

    def __repr__(self):
        return str(self)

    @require_fusion_module
    def set_sample_delay(self, data_source, delay=None, differential=False):
        """
        Change the delay between samples using the onboard time processor
        module to change the effective sampling rate of a specific data source.

        :param data_source: A data source from sensor.SensorFusion
        :param delay: The delay in ms between samples,
            or None to reset to default
        :param differential: Set Time Preprocessor mode to differential,
            instead of the default, absolute
        """

        global current_processor
        global waiting_for_processor

        if self._data_source_signals[data_source] is None:
            log.debug("Getting data signal for data source {0}".format(
                data_source
            ))
            self._data_source_signals[data_source] = \
                libmetawear.mbl_mw_sensor_fusion_get_data_signal(
                    self.board, data_source
                )

        if delay is not None:
            mode = TimeMode.DIFFERENTIAL if differential else \
                TimeMode.ABSOLUTE
            waiting_for_processor = True
            log.debug("Creating time dataprocessor for signal {0}".format(
                self._data_source_signals[data_source]
            ))
            libmetawear.mbl_mw_dataprocessor_time_create(
                self._data_source_signals[data_source],
                mode,
                delay,
                FnVoid_VoidP(processor_set))

            wait_time = 0
            while waiting_for_processor and wait_time < max_processor_wait_time:
                sleeptime = 0.1
                time.sleep(sleeptime)
                wait_time += sleeptime
            if current_processor is not None:
                self._data_source_signals[data_source] = current_processor
                current_processor = None
            else:
                raise PyMetaWearException("Can't set data processor!")

        else:
            data_signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(
                    self.board, data_source)
            if self._data_source_signals[data_source] != data_signal:
                libmetawear.mbl_mw_dataprocessor_remove(
                    self._data_source_signals[data_source]
                )
                self._data_source_signals[data_source] = data_signal

    def _delay_set(self, processor):
        self._current_processor = processor
        self._waiting_for_processor = False

    @require_fusion_module
    def set_mode(self, mode):
        libmetawear.mbl_mw_sensor_fusion_set_mode(self.board,
                                                  mode)
        libmetawear.mbl_mw_sensor_fusion_write_config(self.board)

    @require_fusion_module
    def set_acc_range(self, acc_range):
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(self.board,
                                                       acc_range)
        libmetawear.mbl_mw_sensor_fusion_write_config(self.board)

    def _get_acc_odr(self, value):
        sorted_ord_keys = sorted(self.acc_odr.keys(), key=lambda x: (float(x)))
        diffs = [abs(value - float(k)) for k in sorted_ord_keys]
        min_diffs = min(diffs)
        if min_diffs > 0.5:
            raise ValueError(
                "Requested ODR ({0}) was not part of possible values: {1}".format(
                    value, [float(x) for x in sorted_ord_keys]))
        return float(value)

    def _get_acc_fsr(self, value):
        sorted_ord_keys = sorted(self.acc_fsr.keys(), key=lambda x: (float(x)))
        diffs = [abs(value - float(k)) for k in sorted_ord_keys]
        min_diffs = min(diffs)
        if min_diffs > 0.1:
            raise ValueError(
                "Requested FSR ({0}) was not part of possible values: {1}".format(
                    value, [x for x in sorted_ord_keys]))
        return float(value)

    def set_acc_settings(self, data_range, data_rate):
        """Set accelerometer settings.
         Can be called with two or only one setting:
         .. code-block:: python
            mwclient.set_accelerometer_settings(data_rate=200.0, data_range=8.0)
        will give the same result as
        .. code-block:: python
            mwclient.set_accelerometer_settings(data_rate=200.0)
            mwclient.set_accelerometer_settings(data_range=8.0)
        albeit that the latter example makes two writes to the board.
        Call :meth:`~get_possible_settings` to see which values
        that can be set for this sensor.
        :param float data_rate: The frequency of accelerometer updates in Hz.
        :param float data_range: The measurement range in the unit ``g``.
        """

        if data_rate is not None:
            self.acc_current_odr = data_rate
            odr = self._get_acc_odr(data_rate)
            if self._debug:
                log.debug("Setting Accelerometer ODR to {0}".format(odr))
            libmetawear.mbl_mw_acc_set_odr(self.board, c_float(odr))

        if data_range is not None:
            self.acc_current_fsr = data_range
            fsr = self._get_acc_fsr(data_range)
            if self._debug:
                log.debug("Setting Accelerometer FSR to {0}".format(fsr))
            libmetawear.mbl_mw_acc_set_range(self.board, c_float(fsr))

        if (data_rate is not None) or (data_range is not None):
            self.acc_current_odr = data_rate
            self.acc_current_fsr = data_range
            libmetawear.mbl_mw_acc_write_acceleration_config(self.board)

    @require_fusion_module
    def set_gyro_range(self, gyro_range):
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(self.board,
                                                        gyro_range)
        libmetawear.mbl_mw_sensor_fusion_write_config(self.board)

    def _get_gyr_odr(self, value):
        sorted_ord_keys = sorted(self.gyr_odr.keys(), key=lambda x: (float(x)))
        diffs = [abs(value - float(k)) for k in sorted_ord_keys]
        min_diffs = min(diffs)
        if min_diffs > 0.5:
            raise ValueError(
                "Requested ODR ({0}) was not part of possible values: {1}".format(
                    value, [float(x) for x in sorted_ord_keys]))
        k = sorted_ord_keys[diffs.index(min_diffs)]
        return self.gyr_odr.get(k)

    def _get_gyr_fsr(self, value):
        sorted_ord_keys = sorted(self.gyr_fsr.keys(), key=lambda x: (float(x)))
        diffs = [abs(value - float(k)) for k in sorted_ord_keys]
        min_diffs = min(diffs)
        if min_diffs > 0.1:
            raise ValueError(
                "Requested FSR ({0}) was not part of possible values: {1}".format(
                    value, [float(x) for x in sorted_ord_keys]))
        k = sorted_ord_keys[diffs.index(min_diffs)]
        return self.gyr_fsr.get(k)

    @require_bmi160
    def set_gyro_settings(self, data_rate=None, data_range=None):
        """Set gyroscope settings.
         Can be called with two or only one setting:
         .. code-block:: python
            mwclient.gyroscope.set_settings(data_rate=200.0, data_range=8.0)
        will give the same result as
        .. code-block:: python
            mwclient.gyroscope.set_settings(data_rate=200.0)
            mwclient.gyroscope.set_settings(data_range=1000.0)
        albeit that the latter example makes two writes to the board.
        Call :meth:`~get_possible_settings` to see which values
        that can be set for this sensor.
        :param float data_rate: The frequency of gyroscope updates in Hz.
        :param float data_range: The measurement range in the unit ``dps``,
            degrees per second.
        """
        if data_rate is not None:
            odr = self._get_gyr_odr(data_rate)
            if self._debug:
                log.debug("Setting Gyroscope ODR to {0}".format(odr))
            libmetawear.mbl_mw_gyro_bmi160_set_odr(self.board, odr)
        if data_range is not None:
            fsr = self._get_gyr_fsr(data_range)
            if self._debug:
                log.debug("Setting Gyroscope FSR to {0}".format(fsr))
            libmetawear.mbl_mw_gyro_bmi160_set_range(self.board, fsr)

        if (data_rate is not None) or (data_range is not None):
            libmetawear.mbl_mw_gyro_bmi160_write_config(self.board)

    def _get_mag_power_preset(self, value):
        if value.lower() in self.mag_power_presets:
            return self.mag_power_presets.get(value.lower())
        else:
            raise ValueError("Requested power preset ({0}) was not part of "
                             "possible values: {1}".format(value.lower(), [self.mag_power_presets.keys()]))

    @require_bmm150
    def set_mag_settings(self, power_preset):
        """Set magnetometer settings.
         .code-block:: python
            mwclient.magnetometer.set_settings(power_preset="LOW_POWER")
        Call :meth:`~get_possible_settings` to see which values
        that can be set for this sensor.
        :param str power_preset: The power preset, influencing the data rate,
            accuracy and power consumption
        """
        pp = self._get_mag_power_preset(power_preset)
        if self._debug:
            log.debug("Setting Magnetometer power preset to {0}".format(pp))
        libmetawear.mbl_mw_mag_bmm150_set_preset(self.board, pp)

    @require_fusion_module
    def get_data_signal(self, data_source):
        if self._data_source_signals[data_source] is None:
            self._data_source_signals[data_source] = \
                libmetawear.mbl_mw_sensor_fusion_get_data_signal(
                    self.board, data_source
                )
        return self._data_source_signals[data_source]

    @property
    def data_signal(self):
        return self.current_active_signal

    @property
    def module_name(self):
        return "Sensor Fusion"

    def get_current_settings(self):
        raise NotImplementedError()

    def get_possible_settings(self):
        raise NotImplementedError()

    def set_settings(self):
        raise NotImplementedError()

    @require_fusion_module
    def notifications(self,
                      corrected_acc_callback=None,
                      corrected_gyro_callback=None,
                      corrected_mag_callback=None,
                      quaternion_callback=None,
                      euler_angle_callback=None,
                      gravity_callback=None,
                      linear_acc_callback=None):
        """Subscribe or unsubscribe to sensor fusion notifications.

        Convenience method for handling sensor fusion usage.

        Example:

        .. code-block:: python

            def handle_notification(data):
                # Handle a (epoch_time, (x,y,z,accuracy)) corrected acc tuple.
                epoch = data[0]
                xyzaccu = data[1]
                print("[{0}] X: {1}, Y: {2}, Z: {3}".format(
                    epoch, *xyzaccu[:-1]))

            mwclient.sensorfusion.notifications(
                corrected_acc_callback=handle_notification)

        :param callable corrected_acc_callback: Acceleration notification
            callback function.
            If `None`, unsubscription to acceleration notifications is
            registered.
        :param callable corrected_gyro_callback: Gyroscope notification
            callback function.
            If `None`, unsubscription to gyroscope notifications is registered.
        :param callable corrected_mag_callback: Magnetometer notification
            callback function.
            If `None`, unsubscription to magnetometer notifications is
            registered.
        :param callable quaternion_callback: Quaternion notification callback
            function.
            If `None`, unsubscription to quaternion notifications is registered.
        :param callable euler_angle_callback: Euler angle notification callback
            function.
            If `None`, unsubscription to euler angle notifications is
            registered.
        :param callable gravity_callback: Gravity vector notification callback
            function.
            If `None`, unsubscription to gravity notifications is registered.
        :param callable linear_acc_callback: Linear acceleration notification
            callback function.
            If `None`, unsubscription to linear acceleration notifications is
            registered.

        """
        callback_data_source_map = {
            SensorFusionData.CORRECTED_ACC: corrected_acc_callback,
            SensorFusionData.CORRECTED_GYRO: corrected_gyro_callback,
            SensorFusionData.CORRECTED_MAG: corrected_mag_callback,
            SensorFusionData.QUATERION: quaternion_callback,
            SensorFusionData.EULER_ANGLE: euler_angle_callback,
            SensorFusionData.GRAVITY_VECTOR: gravity_callback,
            SensorFusionData.LINEAR_ACC: linear_acc_callback
        }

        for data_source in callback_data_source_map:
            if callback_data_source_map[data_source] is not None:
                self._streams_to_enable[data_source] = True
            else:
                self._streams_to_enable[data_source] = False

        enable = False in [x is None for x in callback_data_source_map.values()]
        log.debug("Enable: %s" % enable)

        if not enable:
            self.stop()
            self.toggle_sampling(False)

        for data_source in callback_data_source_map:
            self.current_active_signal = self.get_data_signal(data_source)
            callback = callback_data_source_map[data_source]
            if callback is not None:
                self.check_and_change_callback(
                    self.get_data_signal(data_source),
                    sensor_data(callback)
                )
            else:
                self.check_and_change_callback(
                    self.get_data_signal(data_source),
                    None
                )

        if enable:
            self.toggle_sampling(True)
            self.start()

    def check_and_change_callback(self, data_signal, callback):
        if callback is not None:
            if self._debug:
                log.debug("Subscribing to {0} changes. (Sig#: {1})".format(
                    self.module_name, data_signal))
            if data_signal in self._callbacks:
                raise PyMetaWearException(
                    "Subscription to {0} signal already in place!")
            self._callbacks[data_signal] = (callback, FnVoid_DataP(callback))
            libmetawear.mbl_mw_datasignal_subscribe(
                data_signal, self._callbacks[data_signal][1])
        else:
            if data_signal not in self._callbacks:
                return
            if self._debug:
                log.debug("Unsubscribing to {0} changes. (Sig#: {1})".format(
                    self.module_name, data_signal))
            libmetawear.mbl_mw_datasignal_unsubscribe(data_signal)
            del self._callbacks[data_signal]

    @require_fusion_module
    def start(self):
        """Switches the sensorfusion to active mode."""
        libmetawear.mbl_mw_sensor_fusion_start(self.board)

    @require_fusion_module
    def stop(self):
        """Switches the sensorfusion to standby mode."""
        libmetawear.mbl_mw_sensor_fusion_stop(self.board)

    @require_fusion_module
    def toggle_sampling(self, enabled=True):
        """Enables or disables sensor fusion sampling.

        :param bool enabled: Desired state of the sensor fusion module.

        """
        if enabled:
            for data_source in self._streams_to_enable:
                if self._streams_to_enable[data_source]:
                    libmetawear.mbl_mw_sensor_fusion_enable_data(
                        self.board, data_source)
        else:
            libmetawear.mbl_mw_sensor_fusion_clear_enabled_mask(
                self.board)


def processor_set(processor):
    """
    Set global variables as the libmetawear callback can't handle the self
    parameter of instance methods.

    :param processor: The processor thas was created
    """
    global current_processor
    global waiting_for_processor
    current_processor = processor
    waiting_for_processor = False


def sensor_data(func):
    @wraps(func)
    def wrapper(data):
        if data.contents.type_id == DataTypeId.CARTESIAN_FLOAT:
            epoch = int(data.contents.epoch)
            data_ptr = cast(data.contents.value, POINTER(CartesianFloat))
            func((epoch, (data_ptr.contents.x,
                          data_ptr.contents.y,
                          data_ptr.contents.z)))
        elif data.contents.type_id == DataTypeId.QUATERNION:
            epoch = int(data.contents.epoch)
            data_ptr = cast(data.contents.value, POINTER(Quaternion))
            func((epoch, (data_ptr.contents.w,
                          data_ptr.contents.x,
                          data_ptr.contents.y,
                          data_ptr.contents.z)))
        elif data.contents.type_id == DataTypeId.CORRECTED_CARTESIAN_FLOAT:
            epoch = int(data.contents.epoch)
            data_ptr = cast(data.contents.value,
                            POINTER(CorrectedCartesianFloat))
            func((epoch, (data_ptr.contents.x,
                          data_ptr.contents.y,
                          data_ptr.contents.z,
                          data_ptr.contents.accuracy)))
        elif data.contents.type_id == DataTypeId.EULER_ANGLE:
            epoch = int(data.contents.epoch)
            data_ptr = cast(data.contents.value, POINTER(EulerAngles))
            func((epoch, (data_ptr.contents.heading,
                          data_ptr.contents.pitch,
                          data_ptr.contents.roll,
                          data_ptr.contents.yaw)))
        else:
            raise PyMetaWearException('Incorrect data type id: {0}'.format(
                data.contents.type_id))

    return wrapper
