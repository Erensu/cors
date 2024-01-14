run cors-engine:
> cors-engine -o options-file -t trace-level -p server-port -m advanced-mode
>> eg: ./cors-engine -o cors.conf -t 1 -m 0 -p 9000

cors-engine commands:
1. run engine 
> start

2. stop engine
> shutdown

3. add source 
> addsource sourcename IP port ntripuser ntripuserpassword ntripsourcemountpoint sourcelatitude(deg) sourcelongitude(deg) sourceheight(m) sourcetype(M:eph+obs,E:eph,O:obs)
>> eg:  A001 127.0.0.1 8002 ntripuser,ntrip@test A001 37.64052108 116.92379211 63.876 M
>> eg:  A001 127.0.0.1 8002 ntripuser,ntrip@test A001 37.64052108 116.92379211 63.876 O

4. delete source
> delsource sourcename

5. show source's information
> sourceinfo all

6. show navigation data
> navidata

7. show baseline solution
> showbls

8. show cors base stations delaunay triangulate
> showdtrigs

9. show cors base station subnet
> showdtrigs

10. add ntripagnent user
> adduser username userpassword

11. delete ntripagent user
> deluser username

12. show ntripagent users
> showusers

13. add VRS station
> addvsta name ECEF-X ECEF-Y ECEF-Z
>> eg: addvsta VRS001 -2149724.236 4414605.355 4061567.167

14. delete VRS station
> delvsta name

15. show VRS station
> showvsta



