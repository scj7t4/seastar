#ifndef UTIL_H
#define UTIL_H

#ifdef _WIN32
#define DLLEXPORT extern "C" __declspec(dllexport)
#else
#define DLLEXPORT
#endif
struct OpenPt
{
    long est;
    long sofar;
    long index;
};
bool operator<(const OpenPt &lhs, const OpenPt& rhs);

inline long position_to_index(const long &x, const long &y);
inline bool check_coord(const long &x, const long &y, const long &max_x, const long &max_y);
inline long distance(const long &x1, const long &y1, const long &x2, const long &y2);
inline long indextox(const long &p);
inline long indextoy(const long &p);
inline long indexdistance(const long &p1, const long &p2);

#ifdef __cplusplus
extern "C"
{
#endif


/// Create the adjacency list.
DLLEXPORT void init_astar(const long map_width, const long map_height);
DLLEXPORT long* astar(long *startv, const long startc, long *endv, const long endc, long* obstaclev,
             const long obstaclec, const long blocking);
DLLEXPORT void reset_obstacles(long * map, const long len, const long layer);
DLLEXPORT void set_layer_distance(long layer, long distance);
DLLEXPORT void reset_layer_distance(long layer);

#ifdef __cplusplus
}
#endif

#endif
