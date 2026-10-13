# CORS
Continuously Operating Reference Stations (CORS) provide network RTK differential services.

# Overview
The main features of CORS include:
1. Support for multi-frequency, multi-system GNSS, including BeiDou, GPS, GLONASS, Galileo, and QZSS
2. Support for multi-frequency network RTK baseline solutions (dual-frequency, triple-frequency, etc.)
3. Support for medium- and long-baseline network RTK solutions (within 300 km)
4. Fast and robust multi-strategy ambiguity fixing (within 300 km, ambiguity fixing rate of 97%; within 200 km, ambiguity fixing rate of 99%; including low-elevation satellite ambiguity fixing)
5. Virtual Reference Station (VRS) technology
6. Time synchronization technology for reference station observation data
7. Support for large-scale reference station connectivity
8. Reference station networking based on Delaunay triangulation
9. Atmospheric delay error estimation for reference stations, using multi-frequency / multi-combination joint least squares to estimate ionospheric and tropospheric differential correction terms
10. Integrity monitoring for reference stations and virtual stations
11. Support for broadcasting VRS data and providing network RTK differential positioning services
12. Optimization of network RTK computation efficiency
13. High concurrency capabilities: atomic locks + event notifications, lock-free queues, multi-process + multi-thread processing, shared memory, high-performance hash tables, etc.
14. Support for precise GNSS orbit determination with centimeter-level accuracy
15. Support for SSR correction generation

# References
[1] http://www.rtklib.com

[2] http://www.rtcm.org

[3] http://software.rtcm-ntrip.org

# Contact

QQ: 1971129844, Su Jinglan

Email: 1971129844@qq.com

# Note
If you need the professional version of CORS, please contact the author for details.
