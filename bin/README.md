# CORS Engine Quick Reference

## Start the engine

```bash
cors-engine -o options-file -t trace-level -p server-port -m advanced-mode
```

Example:

```bash
./cors-engine -o cors.conf -t 1 -m 0 -p 9000
```

## Engine commands

1. Start the engine
   ```bash
   start
   ```

2. Stop the engine
   ```bash
   shutdown
   ```

3. Add a source
   ```bash
   addsource sourcename IP port ntripuser ntripuserpassword ntripsourcemountpoint latitude(deg) longitude(deg) height(m) sourceType(M:eph+obs,E:eph,O:obs)
   ```

   Examples:
   ```bash
   addsource A001 127.0.0.1 8002 ntripuser,ntrip@test A001 37.64052108 116.92379211 63.876 M
   addsource A001 127.0.0.1 8002 ntripuser,ntrip@test A001 37.64052108 116.92379211 63.876 O
   ```

4. Delete a source
   ```bash
   delsource sourcename
   ```

5. Show source information
   ```bash
   sourceinfo all
   ```

6. Show navigation data
   ```bash
   navidata
   ```

7. Show baseline solution
   ```bash
   showbls
   ```

8. Show Delaunay triangulation of CORS reference stations
   ```bash
   showdtrigs
   ```

9. Show the CORS base station subnet
   ```bash
   showdtrigs
   ```

10. Add an NtripAgent user
    ```bash
    adduser username userpassword
    ```

11. Delete an NtripAgent user
    ```bash
    deluser username
    ```

12. Show NtripAgent users
    ```bash
    showusers
    ```

13. Add a VRS station
    ```bash
    addvsta name ECEF-X ECEF-Y ECEF-Z
    ```

    Example:
    ```bash
    addvsta VRS001 -2149724.236 4414605.355 4061567.167
    ```

14. Delete a VRS station
    ```bash
    delvsta name
    ```

15. Show VRS stations
    ```bash
    showvsta
    ```
