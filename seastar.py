from ctypes import *

import array
import itertools

libseastar = CDLL("./seastar/libseastar.so")
libseastar.init_astar.argtypes = [c_long, c_long]
libseastar.astar.argtypes = [POINTER(c_long), c_long, POINTER(c_long), c_long,
    POINTER(c_long), c_long, c_long]
libseastar.astar.restype = POINTER(c_long)
libseastar.reset_obstacles.argtype = [POINTER(c_long), c_long, c_long]
libseastar.reset_obstacles.restype = c_void_p
libseastar.set_layer_distance.argtype = [c_long, c_long]
libseastar.set_layer_distance.restype = c_void_p
libseastar.reset_layer_distance.argtype = [c_long]
libseastar.reset_layer_distance.restype = c_void_p

def xytuples_to_clist(lst):
    """
    Given a list of tuples, this function converts it to the flat c_type
    int array used by the c based astar
    """
    clength = len(lst) * 2
    cltype = c_long * clength
    outlst = []
    for (x,y) in lst:
        outlst.append(x)
        outlst.append(y)
    return cltype(*outlst)

def clist_to_xytuples(lst, length=None):
    """
    Given a flat c_type int array of points, convert it to a list of tuples
    with the method based on if you know how long the array is.
    """
    if length != None:
        # Apply the length to the clist, which may or may not already have a
        # size:
        resize(lst,length * sizeof(c_long))
        lstpy = itertools.imap(int, lst)
    else:     
        # We don't know what the length of the list is. We have to expand
        # The list and then search through the memory, wee!
        lstpy = itertools.takewhile(lambda x: x != -1, itertools.imap(lambda y: int(lst[y]), itertools.count()))
    return itertools.izip(lstpy,lstpy)


class Seastar(object):
    """
    Python code to interface with my C based A*
    """
    def __init__(self, map_width, map_height):
        """
        Calls the libseastar's init astar function that prepares the adjacency
        table. You can call it a bunch of times, but it just wastes CPU.
        
        Also initializes the blocking and obstacles to some default settings.
        """
        libseastar.init_astar(c_long(map_width), c_long(map_height))
        self.map_width = map_width
        self.map_height = map_height

        # Init: no obstacles
        self.obstacles = array.array('l',[ 0 for _ in range(map_width * map_height) ])

        # Init: Everything blocks a path.
        self.set_blocking(0xffffffff) # Block on any obstacle
    
    def xy(self,x,y):
        return  y * self.map_width + x

    def set_blocking(self,blk):
        """
        Changes the blocking mask that will be used in the next pathing.
        """
        self.blocking = blk
        self.c_blocking = c_long(blk)

    def reset_obstacles(self,layer=0xffffffff):
        """
        Given a layer mask, remove all the obstacles from that layer.
        """
        addr, count = self.obstacles.buffer_info()
        c_obstacles = cast(addr, POINTER(c_long))
        c_obstacles_c = c_long(count)
        c_layer = c_long(layer)
        libseastar.reset_obstacles(c_obstacles, c_obstacles_c, c_layer)

    def remove_obstacle(self, pt, layer):
        index = self.xy(pt)
        self.obstacles[index] &= ~layer

    def add_obstacle(self, pt, layer):
        index = self.xy(pt)
        self.obstacles[index] |= layer
        
    def add_obstacles(self, lst, layer):
        """
        Given a list of x,y tuples and a layer mask, add obstacles to those
        layers.
        """
        for (x,y) in lst:
            index = self.xy(x,y)
            self.obstacles[index] |= layer
    
    def load_obstacles(self, iterable):
        self.obstacles = array.array('l')
        self.obstacles.extend(iterable)

    def set_layer_distance(self, layer, distance):
        libseastar.set_layer_distance(c_long(layer), c_long(distance))

    def reset_layer_distance(layer):
        libseastar.reset_layer_distance(c_long(layer))

    def get_path(self, starts, ends):
        """
        Tries to find a path between one of the starts and ends. Returns the
        shortest path between one of the starts and one of the ends. Returns
        an empty list if no path is found. Otherwise, returns a list of x,y
        tuples which are the path.

        C function args:
        -list of start points
        -count of start points
        -list of end points
        -count of end pints
        -list of obstacles
        -count of obstacles
        -blocking mask.
        """
        for start in starts:
            if start in ends:
                return [start]
        c_starts = xytuples_to_clist(starts)
        c_starts_c = c_long(len(starts)*2)
        c_ends = xytuples_to_clist(ends)
        c_ends_c = c_long(len(ends)*2)
        addr, count = self.obstacles.buffer_info()
        c_obstacles = cast(addr, POINTER(c_long))
        c_obstacles_c = c_long(count)
        rslt = libseastar.astar(c_starts, c_starts_c, c_ends, c_ends_c,
            c_obstacles, c_obstacles_c, self.c_blocking)
        if not rslt:
            return []
        return clist_to_xytuples(rslt)
