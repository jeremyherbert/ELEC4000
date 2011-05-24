from django.shortcuts import render_to_response, get_object_or_404
from django.http import HttpResponseRedirect
from models import Node, NodeRecording
import pdb, time

import urllib2

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

def index(request):
	return HttpResponseRedirect('/get/1')

def set(request, node_pk):
	node = get_object_or_404(Node, pk=node_pk)
	node.is_live = not node.is_live
	get_url('http://[fec0::0]/set/%s/%s/%s/%i' % (str(node.node_id).zfill(3), str(node.beat_interval).zfill(5), str(node.read_interval).zfill(5), node.is_live))
	node.save();
	
	return HttpResponseRedirect('/')

def get_node(request, node_pk):
	node = get_object_or_404(Node, pk=node_pk)
	recs = node.noderecording_set.all()
	return render_to_response('index.html', {'selected_node': node, 'node_list': Node.objects.all(), 'recs': recs, 'recs_count': len(recs), 'logfile':open('log.txt','r').read()})