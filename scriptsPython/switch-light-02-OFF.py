#!/usr/bin/python

import serial
import time
import re
import requests

domoticz_ip='127.0.0.1'
domoticz_port='8080'
user=''
password=''

idWidgetTemperature = 2


def majWidget(widgetId, temperature,humidity ):
    url='/json.htm?type=command&param=udevice&idx='+str(widgetId)
    url+='&nvalue=0&svalue='
    #url+= str(val)
    url+= str(temperature)+';'+str(humidity)+';2'

    requete='http://'+domoticz_ip+':'+domoticz_port+url
    print requete
    #print requete
    #r=requests.get(requete,auth=HTTPBasicAuth(user,password))
    r=requests.get(requete)
    if  r.status_code != 200:
          print "Erreur API Domoticz"

ser = serial.Serial('/dev/ttyUSB0', 9600)


def readResponse():
	response = []
	line = ''
	while (not re.match('WAITING', line)):
		line = ser.readline()
		print 'readResponse : ' + line
		if re.match('WAITING', line):
			break
		else:
			response.append(line)
	return response
		

def sendRequest(command, arg):
	ser.write(command)
	return readResponse()

def getTempAndHumidity():
	response = sendRequest('Humidity', '')
	humidity = False
	temp = False
	for line in response:
		matchObj = re.match( r'Humidite \(%\): ([0-9]+\.[0-9]+)', line)
		if matchObj :
			humidity = matchObj.group(1)
			print 'humidity read : ' + humidity
		matchObj = re.match( r'Temperature \(\^C\): ([0-9]+\.[0-9]+)', line)
		if matchObj :
			temp = matchObj.group(1)
			print 'Temperature read : ' + temp
			
	if humidity and temp :
		majWidget(idWidgetTemperature, temp, humidity)
	
	
	


# Wait arduino waiting message :
readResponse()

print 'Send Request light 2 OFF'
sendRequest('Send:5264844', '')

print 'request sended'


