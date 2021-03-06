# import dependencies
import os
import json
import struct
import time
import numpy as np
import datetime as dt
from flask import Flask, Response, request, redirect, url_for, escape, jsonify, make_response
from flask_mongoengine import MongoEngine
from itertools import chain

app = Flask(__name__)
TIME_FORMAT = "%Y-%m-%d_%H:%M:%S"
TIME_FORMAT_DEL = "%Y-%m-%dT%H:%M:%S"

dev_euis = ['78AF580300000485','78AF580300000506']

# check if running in the cloud and set MongoDB settings accordingly
if 'VCAP_SERVICES' in os.environ:
	vcap_services = json.loads(os.environ['VCAP_SERVICES'])
	mongo_credentials = vcap_services['mongodb-2'][0]['credentials']
	mongo_uri = mongo_credentials['uri']
else:
	mongo_uri = 'mongodb://localhost/db'


app.config['MONGODB_SETTINGS'] = [
	{
		'host': mongo_uri,
		'alias': 'soil_params'
	}
]

# bootstrap our app
db = MongoEngine(app)

class DataPoint(db.Document):
	devEUI = db.StringField(required=True)
	timestamp = db.DateTimeField()
	time = db.StringField()
	temperature = db.IntField();
	illuminance = db.IntField();
	humidity = db.IntField();
	counter = db.IntField();
	debit = db.FloatField();
	voltage = db.IntField();
	#work in a specific mongoDB collection:
	meta = {'db_alias': 'soil_params'}

# set the port dynamically with a default of 3000 for local development
port = int(os.getenv('PORT', '3000'))

# functions for decoding payload
def bitshift (payload,lastbyte):
	return 8*(payload-lastbyte-1)

# our base route which just returns a string
@app.route('/')
def hello_world():
	return "<b>Congratulations! Welcome to Soil Parameter!</b>"

#some functions for the freeboard interface
@app.route('/devices',methods=['GET'])
def devices():
	query = request.args
	if 'dev' in query:
		for i, dev in enumerate(dev_euis):
			if dev == query['dev']:
				return json.dumps(latest_values[i],indent=4)
	return json.dumps({})


#output JSON
@app.route('/json', methods=['GET'])
def print_json():
	query = request.args
	response = DataPoint.objects().to_json()
	return Response(response,mimetype='application/json', headers={'Content-Disposition':'attachment;filename=database.json'})

#querying the database and giving back a JSON file
@app.route('/query', methods=['GET'])
def db_query():
	start = dt.datetime.now() - dt.timedelta(days=365)
	end = dt.datetime.now() + dt.timedelta(hours=2)

	#enable for deleting objects. Attention, deletes parts of the database! 
	if 'delete' in query and 'start' in query and 'end' in query:
		end = dt.datetime.strptime(query['end'], TIME_FORMAT)
		start = dt.datetime.strptime(query['start'], TIME_FORMAT)
		#DataPoint.objects(track_ID=query['delete'],timestamp__lt=end,timestamp__gt=start).delete()
		#return 'objects deleted'
		return 'delete feature disabled for security reasons'

	if 'delpoint' in query:
		print('query for deleting point received')
		deltime_start = dt.datetime.strptime(query['delpoint'], TIME_FORMAT_DEL) - dt.timedelta(seconds=2)
		deltime_end = dt.datetime.strptime(query['delpoint'], TIME_FORMAT_DEL) + dt.timedelta(seconds=2)
		n_points = DataPoint.objects(timestamp__lt=deltime_end, timestamp__gt=deltime_start).count()
		DataPoint.objects(timestamp__lt=deltime_end, timestamp__gt=deltime_start).delete()
		return '{} points deleted'.format(n_points)

	if 'start' in query:
		start = dt.datetime.strptime(query['start'], TIME_FORMAT)

	if 'end' in query:
		end = dt.datetime.strptime(query['end'], TIME_FORMAT)

	return datapoints


# Swisscom LPN listener to POST from actility
@app.route('/sc_lpn', methods=['POST'])
def sc_lpn():
	"""
	This method handles every message sent by the LORA sensors
	:return:
	"""
	print("Data received from ThingPark...")
	j = []
	try:
		j = request.json
	except:
		print("Unable to read information or json from sensor...")
	
	print("JSON received:")
	print(j)

	tuino_list = ['78AF580300000485','78AF580300000506']
	r_deveui = j['DevEUI_uplink']['DevEUI']
	#Parse JSON from ThingPark	
	payload = j['DevEUI_uplink']['payload_hex']
	payload_int = int(j['DevEUI_uplink']['payload_hex'],16)
	bytes = bytearray.fromhex(payload)
	r_time = j['DevEUI_uplink']['Time']
	r_timestamp = dt.datetime.strptime(r_time,"%Y-%m-%dT%H:%M:%S.%f+02:00")
	if sys.getsizeof(bytes)) == 1:
		if(bytes[0]=='t')	##send time
			headers_POST = "Content-type:application/x-www-form-urlencoded"
			time=int(time.time())
			time_bytes = time.to_bytes(4, 'big')
			print('sending to LoRa');
			params = {'DevEUI' = r_deveui,
			'FPORT' = '1',
			'Payload' = time_bytes			
			}
			url="https://proxy1.lpn.swisscom.ch/thingpark/lrc/rest/downlink/"
			request.post(url, headers=headers_POST, params=params)
		elif
			if(bytes[0]=='U')	##Unexpected Flow
			print('unexpected flow')
		
		elif
			if(bytes[0]=='B')	##Unexpected Flow
			print('Battery Low')
	else
		if(r_deveui in tuino_list):
			r_temperature = (bytes[0]<<8)+bytes[1]
			r_iluminance = bytes[2]
			r_humidity = bytes[3]
			r_counter = (bytes[4]>>8)+bytes[5]
			r_debit = ((bytes[6]>>8)+bytes[7])/100
			r_voltage = ((bytes[8]>>8)+bytes[9])
			
			print('Temperature = ' + str(r_temperature) + ' deg C')
			print('Iluminance = ' + str(r_iluminance) + '%')
			print('Humidity = ' + str(r_humidity) + '%')
			print('Counter = ' + str(r_counter) + ' pulses')
			print('Debit = ' + str(r_debit) + ' l')
			print('Voltage = ' + str(r_voltage) + ' mV')
		else:
			return "device type not recognised"

		datapoint = DataPoint(devEUI=r_deveui, time= r_time, timestamp = r_timestamp, temperature=r_temperature, iluminance=r_iluminance, humidity = r_humidity, counter=r_counter, debit=r_debit, voltage=r_voltage)
		print(datapoint)
		datapoint.save()
		print('Datapoint saved to database')
		return 'Datapoint DevEUI %s saved' %(r_deveui)


# start the app
if __name__ == '__main__':
	app.run(host='0.0.0.0', port=port)
