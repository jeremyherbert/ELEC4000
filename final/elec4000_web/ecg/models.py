from django.db import models

# Create your models here.
class Node(models.Model):
	#id, 3 digit
	#read interval
	#beat interval
	#max hr
	#min hr
	#is live
	
	node_id = models.IntegerField()
	read_interval = models.IntegerField()
	beat_interval = models.IntegerField()
	max_heart_rate = models.IntegerField()
	min_heart_rate = models.IntegerField()
	
	
class NodeRecording(models.Model):
	# node data
	node = models.ForeignKey(Node)
	heart_rate = models.IntegerField()
	D2 = models.IntegerField()
	D3 = models.IntegerField()
	D4 = models.IntegerField()
	D5 = models.IntegerField()
	D6 = models.IntegerField()
	D7 = models.IntegerField()
	local_timestamp = models.IntegerField()
	timestamp = models.DateTimeField(auto_now=False)
	