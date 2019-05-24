CFLAGS = 
LDFLAGS = -lm -lGL -lGLU -lglut -lX11

all: drawCAD

drawCAD: drawCAD.o
	gcc -o drawCAD drawCAD.c $(LDFLAGS)

clean:
	rm keypoint_visibility
	rm keypoint_visibility.o
	rm drawCAD
	rm drawCAD.o
