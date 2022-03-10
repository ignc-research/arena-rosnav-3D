import rospy
from functools import wraps


def timeit(f):
    @wraps(f)
    def timed(*args, **kw):

        ts = rospy.get_time()
        result = f(*args, **kw)
        te = rospy.get_time()

        print("func:%r args:[%r, %r] took: %2.4f sec" % (f.__name__, args, kw, te - ts))
        return result

    return timed
