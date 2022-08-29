def read_line_from_file(filename, line_num=1):
    target_line = ''
    with open(filename) as file:
        for i in range(line_num - 1):
            file.readline()
        target_line = file.readline()

    return target_line


def parse_lat_lon_alt(csv_line):
    coords = csv_line.split(', ')
    lat = float(coords[0].split(' ')[1])
    lon = float(coords[1].split(' ')[1])

    return lon, lat, 0.0
