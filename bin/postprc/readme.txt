# ============================================================
# cors-postprc - CORS post-processing replay tool
# ============================================================
#
# Purpose
#   Replays recorded reference station data (a recorded NTRIP stream
#   file) through the engine, reproducing the full processing chain
#   offline: network RTK baseline solving, atmospheric delay modelling,
#   VRS observation generation and monitoring. No live stations needed.
#
# Run
#   Open a command prompt (cmd) in this directory and run:
#     .\cors-postprc.exe -o .\conf\cors_postprc.conf -s 2025:4:20:0:0:0
#       -o  configuration file (relative paths resolve against the
#           working directory at start-up)
#       -s  replay start time, format: year:month:day:hour:minute:second
#
# Configuration: conf\cors_postprc.conf
#   Same option set as cors.conf. Key entries:
#     ntrip-record-file = .\ntrip_record_20250420.dat   recorded data file
#     monitor-port      = 7999                          monitoring service port
#
# Sample data
#   ntrip_record_20250420.dat is a raw observation data set of CORS
#   reference stations, about three hours long, sampled at 1 Hz.
#   Download link: https://pan.baidu.com/s/1vVVAsQUiu6s38APWQrCSCA  code: ctxu
#   After download, place the file in this directory (next to cors-postprc.exe).
#
# Directory layout
#   cors-postprc.exe   the replay tool
#   conf\              configuration files
#   lib*.dll           MinGW runtime libraries - keep them next to the
#                      executable, do not delete
#   readme.txt         this file