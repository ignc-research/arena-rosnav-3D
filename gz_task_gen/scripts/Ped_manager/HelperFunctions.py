def get_ros_package_path(package_name):
    # type: (str) -> str
    try:
        import rospkg

        rospack = rospkg.RosPack()
        return rospack.get_path(package_name)
    except:
        return ""


def get_nth_decimal_part(x, n):
    # type: (float, int) -> int
    """
    Get the n'th decimal part of a decimal number.
    Example:
        get_nth_decimal_part(1.234, 2) == 3
    """
    x *= 10 ** n  # push relevant part directly in front of decimal point
    x %= 10  # remove parts left of the relevant part
    return int(x)  # remove decimal places


def round_to_closest_20th(x):
    # type: (float) -> float
    """
    Round to X.X0 or X.X5.
    Example:
        round_one_and_half_decimal_places(1.234) == 1.25
    """
    return round(x * 20) / 20

def rad_to_deg(angle):
    # type: (float) -> float
    import math
    angle = normalize_angle_rad(angle)
    angle = 360.0 * angle / (2.0 * math.pi)
    return angle

def deg_to_rad(angle):
    # type: (float) -> float
    import math
    angle = normalize_angle_deg(angle)
    angle = 2 * math.pi * angle / 360.0
    return angle

def normalize_angle_deg(angle):
    # type: (float) -> float
    import math

    # make sure angle is positive
    while angle < 0:
        angle += 360

    # make sure angle is between 0 and 360
    angle = math.fmod(angle, 360.0)
    return angle

def normalize_angle_rad(angle):
    # type: (float) -> float
    import math

    # make sure angle is positive
    while angle < 0:
        angle += 2 * math.pi
    # make sure angle is between 0 and 2 * pi
    angle = math.fmod(angle, 2 * math.pi)
    return angle

def normalize_angle(angle , rad = True):
    # type: (float, bool) -> float
    if rad:
        return normalize_angle_rad(angle)
    else:
        return normalize_angle_deg(angle)

def get_current_user_path(path_in):
    # type (str) -> str
    """
    Convert a path from another user to the current user, for example:
    "/home/alice/catkin_ws" -> "/home/bob/catkin_ws"
    """
    if path_in == "":
        return ""
    # from pathlib import Path

    # path = Path(path_in)
    # c = os.path.split(path)
    # new_path = Path.home().joinpath(*path.parts[3:])

    from os.path import expanduser
    path = path_in.split('/')
    new_path = expanduser("~") + '/' + '/'.join(path[3:])
    print new_path
    return str(new_path)
