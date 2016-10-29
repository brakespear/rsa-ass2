#!/usr/bin/python2.7

import sys, turtle, re, time
t = turtle.Pen()
t.speed('fastest')


#draw axis for the first frame
t.up()
t.goto(-600,-1000)
t.down()
t.goto(-600,1000)
t.up()
t.goto(-1000,0)
t.down()
t.goto(1000,0)

frameNo = 1
# Open text file supplied as command line argument
file_in = open(sys.argv[1],'r').readlines()
for line in file_in:
    line = line.strip('\n')
    if "Legs centre" in line:
        rubbish,coords = line.split('|')
        centerx,centery = coords.split(',')
        print "Legs center at ",centerx,centery
        t.up()
        t.goto(80*float(centerx)-600,80*float(centery))
        t.down()
		# Display the legs centre as a red dot
        t.dot(5,'red')
        t.up()
    elif "&" in line:
		# Freeze the simulation momentarily
        t.up()
        t.goto(-1000,-1000)
        turtle.update()
       	time.sleep(0.05) #Quick simulation of multiple frames
		#time.sleep(15) used to view a single frame
		# Display the frame number so individual frames can be analysed
		# As part of debugging the algorithm
		print "Frame #=",frameNo
		frameNo+=1
        t.clear()
		# Set animation mode, drawing happens instantly
        t.tracer(False)
    else:
		# Extract data using regex
        cartesianCoords = re.search(r"\[(.+)]",line)
        x,y = cartesianCoords.group(0).strip('[]').split(',')
        polarCoords = re.search(r"{(.+)}",line)
        r, theta = polarCoords.group(0).strip('{}').split(',')
		# Avoid invalid points
        if "nan" in x or "nan" in y or "inf" in x or "inf" in y:
            continue
        t.up()
        t.goto(80*float(x)-600,80*float(y))
        t.down()
		# Display detected laser scan points
        t.dot(2,'black')

t.up()
t.goto(-1000,-1000)
turtle.done()