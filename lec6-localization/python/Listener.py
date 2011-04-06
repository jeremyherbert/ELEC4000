#!/usr/bin/python

import sys
import struct
import SFClient
from Tkinter import *
from threading import *
import math

DBM_THRESHOLD=6
NUM_NODES=5
AM_TYPE=12

node_coords = []
def init_node_coords():
  angle = 2*math.pi/NUM_NODES
  for i in range(0, NUM_NODES):
    node_coords.append((150+100*math.cos(i*angle), 150+100*math.sin(i*angle)))

def learn_matrix():
  print "foo"

prev_text = []
def plot_matrix(canvas, x, y, matrix, changed):
    for i in range(0,len(prev_text)):
      canvas.delete(prev_text[i])
    for i in range(0,NUM_NODES):
      for j in range(0,NUM_NODES):
        s = '%d' % matrix[i+NUM_NODES*j]
        c = "black"
        if matrix[i+NUM_NODES*j]==127:
          s = "."
        if changed[i+NUM_NODES*j]:
          c = "red"
        prev_text.append(canvas.create_text(x+30*i, y+20*j, text=s, fill=c))

def plot_points(canvas):
  for i in range(0, NUM_NODES):
    canvas.create_oval(node_coords[i][0]-5, node_coords[i][1]-5, node_coords[i][0]+5, node_coords[i][1]+5, fill="black")
    canvas.create_text(node_coords[i][0]+15, node_coords[i][1]+15, text="%d" % i)

prev_matrix=[]
class DataThread(Thread):
  def __init__(self, canvas):
    Thread.__init__(self)
    self.canvas = canvas

  def run(self):
    global prev_matrix
    sfc = SFClient.SFClient(sys.argv[1], int(sys.argv[2]))
    changed = []
    while True:
      data = sfc.read().data
      #check length and AM_TYPE
      if (len(data) > 6) and (data[6]==AM_TYPE):
        data = data[7:]
        matrix = [struct.unpack(">b",chr(c))[0] for c in data]

        if len(prev_matrix)>0:
          changed=[abs(a-b)>DBM_THRESHOLD for a,b in zip(matrix,prev_matrix)]
        else:
          changed=[False for j in range(0,len(matrix))]
        prev_matrix=matrix
        plot_matrix(self.canvas, 325, 100, matrix, changed)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "usage: Listener.py <host> <port>"
        sys.exit(1)
    init_node_coords()

    tk = Tk()
    tk.wm_geometry("500x300+20+40")
    frame = Frame(tk)
    frame.pack()
    canvas = Canvas(tk, width=500, height=300)
    canvas.pack()
    
    button=Button(frame, text="Learn Matrix", command=learn_matrix)
    button.pack()

    text_window = canvas.create_rectangle(300,10,485,255,fill="gray", outline="black")
    plot_points(canvas)
    
    dataThread = DataThread(canvas)
    dataThread.setDaemon(True)
    dataThread.start()
    tk.mainloop()

