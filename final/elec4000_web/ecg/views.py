from django.shortcuts import render_to_response, get_object_or_404
from django.http import HttpResponseRedirect
from models import Node, NodeRecording
import pdb

def index(request):
	return HttpResponseRedirect('/get/1')

def get_node(request, node_pk):
	node = get_object_or_404(Node, pk=node_pk)
	recs = node.noderecording_set.all()
	return render_to_response('index.html', {'selected_node': node, 'node_list': Node.objects.all(), 'recs': recs, 'recs_count': len(recs)})