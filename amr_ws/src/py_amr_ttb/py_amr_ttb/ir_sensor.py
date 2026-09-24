"""Interpret external TurtleBot IR messages for autonomous wandering."""

import math


class IrSensorState:
    """Store the latest usable IR readings and detect nearby obstacles."""

    def __init__(self, config):
        self.config = config
        self.values = []
        self.frame_names = []
        self.last_message_time = None

    def update(self, message, timestamp):
        """Store one IR message, returning an error string when invalid."""
        readings = getattr(message, 'readings', None)
        if readings is None:
            self._invalidate()
            return 'IR message does not match IrIntensityVector: no readings.'

        values = []
        frame_names = []
        for reading in readings:
            raw_value = getattr(reading, 'value', None)
            if raw_value is None:
                self._invalidate()
                return 'IR reading does not match IrIntensity: no value.'

            try:
                value = float(raw_value)
            except (TypeError, ValueError):
                self._invalidate()
                return 'IR reading value is not numeric.'

            if not math.isfinite(value):
                self._invalidate()
                return 'IR reading value is not finite.'

            values.append(value)
            header = getattr(reading, 'header', None)
            frame_names.append(getattr(header, 'frame_id', ''))

        if not values:
            self._invalidate()
            return 'IR message contains no readings.'

        self.values = values
        self.frame_names = frame_names
        self.last_message_time = float(timestamp)
        return None

    def is_fresh(self, now):
        """Return whether a valid reading arrived within the timeout."""
        if self.last_message_time is None:
            return False

        age = float(now) - self.last_message_time
        return 0.0 <= age <= self.config.SENSOR_TIMEOUT

    def validation_error(self):
        """Explain why the current readings/configuration cannot be used."""
        threshold = self.config.IR_OBSTACLE_THRESHOLD
        if threshold is None:
            return 'WANDER disabled: calibrate the IR threshold.'

        try:
            threshold = float(threshold)
        except (TypeError, ValueError):
            return 'IR obstacle threshold must be numeric.'
        if not math.isfinite(threshold):
            return 'IR obstacle threshold must be finite.'

        if not self.values:
            return 'IR data is missing.'

        indices = self.config.IR_FRONT_SENSOR_INDICES
        if indices is None:
            return None
        if not indices:
            return 'IR front-sensor index list is empty.'

        for index in indices:
            if not isinstance(index, int):
                return 'IR front-sensor indices must be integers.'
            if index < 0 or index >= len(self.values):
                return 'IR front-sensor index is outside the message.'
        return None

    def obstacle_detected(self):
        """Apply the calibrated sensor selection and threshold rule.

        Invalid configuration fails closed by reporting an obstacle. The
        wander behavior separately reports the detailed validation error and
        returns a zero command, so it will not turn on bad sensor data.
        """
        if self.validation_error() is not None:
            return True

        values = self.values
        indices = self.config.IR_FRONT_SENSOR_INDICES
        if indices is not None:
            values = [self.values[index] for index in indices]

        threshold = float(self.config.IR_OBSTACLE_THRESHOLD)
        if self.config.IR_OBSTACLE_WHEN_ABOVE_THRESHOLD:
            return any(value >= threshold for value in values)
        return any(value <= threshold for value in values)

    def _invalidate(self):
        """Discard old data immediately after a malformed sensor message."""
        self.values = []
        self.frame_names = []
        self.last_message_time = None
