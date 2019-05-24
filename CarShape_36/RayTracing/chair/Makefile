CFLAGS = 
LDFLAGS = -lm -lGL -lGLU -lglut -lX11

all: keypoint_visibility drawCAD

keypoint_visibility: keypoint_visibility.o
	gcc -o keypoint_visibility keypoint_visibility.c $(LDFLAGS)

drawCAD: drawCAD.o
	gcc -o drawCAD drawCAD.c $(LDFLAGS)

clean:
	rm keypoint_visibility
	rm keypoint_visibility.o
	rm drawCAD
	rm drawCAD.o
