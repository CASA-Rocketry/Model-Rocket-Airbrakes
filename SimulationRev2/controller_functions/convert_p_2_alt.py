def find_altitude(p, p0):
    a = 44330.0 * (1 - (p / p0) ** (1 / 5.255))
    return a