import csv
import os
from datetime import datetime, timezone

from platformio.device.monitor.filters.base import DeviceMonitorFilterBase


class SensorCsvLogger(DeviceMonitorFilterBase):
    NAME = "sensor_csv"

    HEADER = [
        "host_timestamp_utc",
        "device_uptime_ms",
        "sensor",
        "ph",
        "orp_mv",
        "ec_us_cm",
        "temperature_c",
        "salinity_ppm",
        "tds_ppm",
        "depth_m",
        "status",
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._line_buffer = ""
        self._file = None
        self._writer = None

    def __call__(self):
        log_dir = os.path.join(self.project_dir or os.getcwd(), "logs")
        os.makedirs(log_dir, exist_ok=True)
        filename = "sensor-data-%s.csv" % datetime.now().strftime("%Y%m%d-%H%M%S")
        path = os.path.join(log_dir, filename)
        self._file = open(path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._file.flush()
        print("--- Sensor CSV logging to %s" % path)
        return self

    def __del__(self):
        if self._file:
            self._file.close()

    def rx(self, text):
        self._line_buffer += text
        while "\n" in self._line_buffer:
            line, self._line_buffer = self._line_buffer.split("\n", 1)
            self._log_data_line(line.rstrip("\r"))
        return text

    def _log_data_line(self, line):
        if not line.startswith("DATA,"):
            return

        fields = next(csv.reader([line]))
        if len(fields) != 11:
            return

        timestamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        self._writer.writerow([timestamp, *fields[1:]])
        self._file.flush()
