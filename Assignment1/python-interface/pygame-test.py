import sys
#import and init pygame
import pygame

from socket import *

sock = socket(AF_INET6, SOCK_DGRAM)
sock.bind(("fec0::64",7001))

pygame.init() 

#create the screen
window = pygame.display.set_mode((640, 480)) 

strength33 = 50
strength66 = 50

#draw a line - see http://www.pygame.org/docs/ref/draw.html for more 
#pygame.draw.line(window, (255, 255, 255), (0, 0), (30, 50))
sock.sendto("\r\n", ("fec0::1", 7001))
#input handling (somewhat boilerplate code):
while True: 
    data,addr = sock.recvfrom(1024)
    sp = data.split('\n')

    node = ""
    seq = ""
    strength = ""

    try:
        node = sp[1].split(" ")[2].replace('\r','')
        seq = sp[2].split(" ")[2].replace('\r', '')
        strength = sp[3].split(" ")[1].replace('\r', '')
    except:
        pass
    
    if node == "33":
        strength33 = int(strength)+50
    else:
        strength66 = int(strength)+50
    pygame.draw.circle(window, (0,0,0), (320,240), 320)
    pygame.draw.circle(window, (255,0,0), (320,240), 200-2*strength33, 2)
    pygame.draw.circle(window, pygame.color.Color(0,255,0), (320,240), 200-2*strength66, 2) 

    #draw it to the screen
    pygame.display.flip()

    for event in pygame.event.get(): 
        if event.type == pygame.QUIT: 
            sys.exit(0) 
        else: 
            print event 
