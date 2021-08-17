# Gimbal_DataLogger
Use Raspberry Pi 4 logging gimbal debugs data to Firebase real time database

## Raspberry login info
 1. user: pi
 2. password: datalogger
 3. static ip: 192.168.11.8

## Sofware install
 1. install firabase  
   $ sudo pip3 install pyrebase  
   $ sudo pip3 install --upgrade google-auth-oauthlib
 2. install pymavlink  
   Use pymavlink library included

## Buzzer and Led Status
```
  1. Single beep: push data to server
  2. Double beep: no internet connection
  3. Triple beep: no mavlink connection
```
## Ref
 1. https://funprojects.blog/2018/10/09/iot-with-google-firebase/
