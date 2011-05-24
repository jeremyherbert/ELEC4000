import urllib2, urllib, pdb, time, sqlite3
from datetime import datetime
from datetime import timedelta
from django.core.management import setup_environ
import settings
setup_environ(settings)

from ecg.models import Node, NodeRecording

global time1, time2
time1 = 0
time2 = 0
##############

def get_url(url):
	while (1):
		try:
			#pdb.set_trace()
			time.sleep(2)
			opener = urllib2.urlopen(url)
			time.sleep(2)
			return opener.read()
		except urllib2.URLError:
			print "URL get %s failed" % url
			pass

def get_latest_beats():

	print "beat"
	beats = get_url('http://[fec0::0]/get/beat').replace('/','').split('\n')
	time.sleep(1)
	#pdb.set_trace()
	return beats

def send_emergency():
	get_url('http://[fec0::0]/send')
	
def get_time():
	return get_url('http://[fec0::0]/get/time').rstrip()
	
def convert_local_to_timestamp(local):
	global time1, time2
	
	dt = int(time2)-int(time1); # equiv to 1 second

	number_of_seconds = (int(time2)-int(local))/dt

	return datetime.now() - timedelta(seconds=number_of_seconds)
	
def get_data(node_id):
	while(1):
		data = get_url('http://[fec0::0]/get/data/%s' % str(node_id).zfill(3))
		if data.find('started') > -1 or data.find('wait') > -1:
			print "waiting...."
			continue
		else:
			break
			
	data_finished = False
	while(not data_finished):
		data2 = data.replace('//', '')
		
		for line in data2.split('\n'):
			print line
			if line.find('done') > -1:
				data_finished = True
				break
				
			try:
				#pdb.set_trace()
				rx_id, hr, d2, d3, d4, d5, d6, d7, local_timestamp = line.split('/')
				print "got hr: %s" % hr
				
				if hr < 10:
					continue
				
			except:
				continue
			try:
				node = Node.objects.get(node_id=rx_id)
			except:
				node = Node(node_id=rx_id, beat_interval=15000, read_interval=1500, max_heart_rate=120, min_heart_rate=50)
				node.save()
			
			rec = NodeRecording(node=node, heart_rate=hr, D2=d2, D3=d3, D4=d4, D5=d5, D6=d6, D7=d7, local_timestamp=local_timestamp, timestamp=convert_local_to_timestamp(local_timestamp))
			rec.save()
		
		if (not data_finished):
			data = get_url('http://[fec0::0]/get/data/%s' % str(node_id).zfill(3))
	
def get_emergency():
	data = get_url('http://[fec0::0]/get/emer').replace('\n', '').replace('//','')
	if data:
	#	pdb.set_trace()
		node_id, emer_type, time = data.split('/')
		handle = open('log.txt', 'w')
		handle.write('Emergency at node %s, type %s, time %s' % (node_id, emer_type, convert_local_to_timestamp(time)))
		handle.close()
		pdb.set_trace()

beat_table = {}

if __name__ == "__main__":
	global time1, time2
	time1 = get_time();
	time.sleep(1)
	time2 = get_time();
	
	while (1):
	
		##############
	
		# build the beat table
		for node_id in get_latest_beats():
			if node_id == '':
				continue
			
			beat_table[node_id] = datetime.now()
				
		for node_id in beat_table.keys():
			if datetime.now() - beat_table[node_id] > timedelta(seconds=30):
				send_emergency()
		
		get_data(7)
		get_emergency()
		##############
	
		#pdb.set_trace()
	
		# wait for a bit
		time.sleep(5)
