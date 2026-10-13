mcors 0.1.0 (Windows x64)
==========================

Real-time CORS reference station network monitor and solution engine,
as a ready-to-run release package.

Quick start (3 steps)
---------------------
1. Double-click start.cmd
   (or run it from a command line: start.cmd)
   - Starts the engine automatically (its own console window, port 9000)
   - Starts the dashboard collector automatically (uses the bundled
     runtime\node.exe, so no Node.js installation is needed)
   - Opens the browser at http://localhost:8181/

2. Once the dashboard opens you should see the station network map
   (Delaunay triangulation) and the baseline solution table.
   Type help in the engine console to list the available commands;
   start-gui reopens the dashboard at any time.

3. To stop: double-click stop.cmd (or close both mcors-* windows).

Starting only one part
----------------------
  start.cmd engine      engine only
  start.cmd dash        dashboard only (the engine must already be running)

Configuring your data sources (optional)
----------------------------------------
  By default conf\ntripsources holds redacted examples only
  (STATION001/002 enabled, the rest commented out).
  Add your real NTRIP stations in the same format:

    name,NTRIPhost,port,user,password,mountpoint,lat,lon,height,type#

  Save, then restart the engine (stop.cmd followed by start.cmd). While a
  station is not connected you can also add it live, either from the
  dashboard's Station management panel or with the console addsource
  command (runtime only).

Important notes
---------------
- The engine must own a real console window; it cannot run as a
  windowless background process.
- The engine listens on one of this machine's NIC IPs (not 127.0.0.1 or
  0.0.0.0). The collector detects it automatically, so no manual
  configuration is needed.
- Ports: dashboard 8181, solution stream 8080, console 9000. If one is
  already taken, stop the process holding it first.
- agentuser contains sample credentials (test1/password) for NTRIP client
  authentication. Change them before deploying.
- For day-to-day operation and troubleshooting, see doc\manual.html
  (English); the Chinese edition is doc\manual.zh.html.
- The dashboard defaults to English. The language toggle in the top-right
  corner switches to Chinese, and the choice is remembered.

Directory layout
----------------
  cors-engine.exe      Solution engine (plus libuv and 3 other runtime DLLs)
  runtime\node.exe     Bundled Node.js runtime for the dashboard collector;
                       nothing to install
  conf\                Configuration (ntripsources is a redacted template;
                       fill in your real stations)
  viz\collector\       Dashboard collector (Node.js, zero third-party deps)
  viz\web\             Dashboard front end (static, English/Chinese)
  doc\manual.html      Full manual - English (build, config, API, FAQ)
  doc\manual.zh.html   Full manual - Chinese (same content, separate file)
  README.zh.txt        This readme in Chinese

Known limitations
-----------------
- This package is Windows x64 only; for Linux builds see BUILD.md in the
  source repository.
- Stations added at runtime (marked "runtime") disappear after a restart.
  To make one permanent, write it into conf\ntripsources.
- A removed baseline's row stays on the dashboard for 30 seconds (buffer
  period) before fading out. This is expected behaviour.
