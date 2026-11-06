# Bosch BME280 / BMP280 for MicroPython (and CPython, for the tests).
# Integer compensation formulas from the Bosch datasheets (BME280 4.2.3, BMP280 3.11.3),
# written so Python's unlimited ints give exactly the C results.

CHIP_IDS = {0x60: "BME280", 0x58: "BMP280", 0x57: "BMP280", 0x56: "BMP280"}


def _u16(b, i):
    return b[i] | (b[i + 1] << 8)


def _s16(b, i):
    v = _u16(b, i)
    return v - 65536 if v > 32767 else v


def _s8(v):
    return v - 256 if v > 127 else v


def _tdiv(a, b):
    """C integer division (truncates towards zero)."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b > 0) else -q


def parse_calib(tp, h1=None, h=None):
    """tp = 24 bytes from 0x88; h1 = byte at 0xA1; h = 7 bytes from 0xE1 (BME280 only)."""
    c = {"T1": _u16(tp, 0), "T2": _s16(tp, 2), "T3": _s16(tp, 4),
         "P1": _u16(tp, 6), "P2": _s16(tp, 8), "P3": _s16(tp, 10), "P4": _s16(tp, 12),
         "P5": _s16(tp, 14), "P6": _s16(tp, 16), "P7": _s16(tp, 18), "P8": _s16(tp, 20),
         "P9": _s16(tp, 22), "H1": 0, "H2": 0, "H3": 0, "H4": 0, "H5": 0, "H6": 0}
    if h is not None:
        c["H1"] = h1
        c["H2"] = _s16(h, 0)
        c["H3"] = h[2]
        c["H4"] = (_s8(h[3]) * 16) | (h[4] & 0x0F)
        c["H5"] = (_s8(h[5]) * 16) | (h[4] >> 4)
        c["H6"] = _s8(h[6])
    return c


def t_fine(c, adc_t):
    var1 = (((adc_t >> 3) - (c["T1"] << 1)) * c["T2"]) >> 11
    d = (adc_t >> 4) - c["T1"]
    var2 = (((d * d) >> 12) * c["T3"]) >> 14
    return var1 + var2


def temp_centi(tf):
    return (tf * 5 + 128) >> 8


def press_q24_8(c, adc_p, tf):
    var1 = tf - 128000
    var2 = var1 * var1 * c["P6"]
    var2 = var2 + ((var1 * c["P5"]) << 17)
    var2 = var2 + (c["P4"] << 35)
    var1 = ((var1 * var1 * c["P3"]) >> 8) + ((var1 * c["P2"]) << 12)
    var1 = (((1 << 47) + var1) * c["P1"]) >> 33
    if var1 == 0:
        return 0
    p = 1048576 - adc_p
    p = _tdiv(((p << 31) - var2) * 3125, var1)
    var1 = (c["P9"] * (p >> 13) * (p >> 13)) >> 25
    var2 = (c["P8"] * p) >> 19
    return ((p + var1 + var2) >> 8) + (c["P7"] << 4)


def hum_q22_10(c, adc_h, tf):
    v = tf - 76800
    v = ((((adc_h << 14) - (c["H4"] << 20) - (c["H5"] * v)) + 16384) >> 15) * \
        (((((((v * c["H6"]) >> 10) * (((v * c["H3"]) >> 11) + 32768)) >> 10) + 2097152) * c["H2"] + 8192) >> 14)
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * c["H1"]) >> 4)
    v = 0 if v < 0 else v
    v = 419430400 if v > 419430400 else v
    return v >> 12


class Bmx280:
    """Tiny driver: finds the sensor at 0x76 or 0x77, reads t_c, hpa and (BME280 only) rh."""

    def __init__(self, i2c):
        self.i2c = i2c
        self.addr = None
        self.kind = "none"
        self.cal = None

    def probe(self):
        self.addr = None
        self.kind = "none"
        for a in (0x76, 0x77):
            try:
                cid = self.i2c.readfrom_mem(a, 0xD0, 1)[0]
            except OSError:
                continue
            if cid not in CHIP_IDS:
                continue
            try:
                self.i2c.writeto_mem(a, 0xE0, b"\xb6")          # soft reset
                _sleep_ms(10)
                tp = self.i2c.readfrom_mem(a, 0x88, 24)
                if cid == 0x60:
                    h1 = self.i2c.readfrom_mem(a, 0xA1, 1)[0]
                    h = self.i2c.readfrom_mem(a, 0xE1, 7)
                    self.cal = parse_calib(tp, h1, h)
                    self.i2c.writeto_mem(a, 0xF2, b"\x01")      # humidity x1 (before 0xF4)
                else:
                    self.cal = parse_calib(tp)
                self.i2c.writeto_mem(a, 0xF5, b"\xa0")          # 1 s standby, no filter
                self.i2c.writeto_mem(a, 0xF4, b"\x27")          # temp x1, pressure x1, normal mode
            except OSError:
                continue
            self.addr, self.kind = a, CHIP_IDS[cid]
            return True
        return False

    def read(self):
        """dict with t_c, hpa and rh (BME280), or None."""
        if self.addr is None:
            return None
        try:
            d = self.i2c.readfrom_mem(self.addr, 0xF7, 8 if self.kind == "BME280" else 6)
        except OSError:
            self.addr, self.kind = None, "none"
            return None
        adc_p = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4)
        adc_t = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4)
        if adc_t == 0x80000:                                   # no measurement yet
            return None
        tf = t_fine(self.cal, adc_t)
        out = {"t_c": temp_centi(tf) / 100, "hpa": round(press_q24_8(self.cal, adc_p, tf) / 25600, 1)}
        if self.kind == "BME280":
            out["rh"] = round(hum_q22_10(self.cal, (d[6] << 8) | d[7], tf) / 1024, 1)
        return out


def _sleep_ms(ms):
    try:
        import time
        time.sleep_ms(ms)
    except AttributeError:                                     # CPython (tests)
        import time
        time.sleep(ms / 1000)
