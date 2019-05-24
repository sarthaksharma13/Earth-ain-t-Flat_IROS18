/*
Originally provided along with the PASCAL3D dataset.
Adapted by KM for use in visibility checking of car keypoints.
*/

#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define WIDTH 500
#define HEIGHT 500

/* global variables */
int Nvertice, Nface, Nanchor, Flag;
float azimuth;
int *IsExist;
GLfloat *Vertices;
GLfloat *Anchors;
GLuint *Faces;
FILE *Fp;


// Load a CAD model from a .off file
int load_off_file(int* pnv, GLfloat** pvertices, int* pnf, GLuint** pfaces, char* filename) {

  FILE* fp;
  char buffer[256];
  int nv, nf, aux, i;
  GLfloat *vertices;
  GLuint *faces;

  // Open .off file
  fp = fopen(filename, "r");
  if(fp == NULL){
    printf("Can not open file %s\n", filename);
    *pnv = 0;
    *pvertices = NULL;
    *pnf = 0;
    *pfaces = NULL;
    return 1;
  }

  // Read file header
  fgets(buffer, 256, fp);
  if(strncmp(buffer, "OFF", 3) != 0){
    printf("Wrong format .off file %s\n", filename);
    return 1;
  }

  // Read numbers
  fscanf(fp, "%d", &nv);
  fscanf(fp, "%d", &nf);
  fscanf(fp, "%d", &aux);

  // Allocate memory
  vertices = (GLfloat*)malloc(sizeof(GLfloat)*nv*3);
  if(vertices == NULL){
    printf("Out of memory!\n");
    return 1;
  }

  // Read vertices
  for(i = 0; i < 3*nv; i++)
    fscanf(fp, "%f", vertices+i);

  // Allocate memory
  if(nf != 0){
    faces = (GLuint*)malloc(sizeof(GLuint)*nf*3);
    if(faces == NULL){
      printf("Out of memory\n");
      return 1;
    }

    // Read faces
    for(i = 0; i < nf; i++){
      fscanf(fp, "%d", &aux);
      if(aux != 3){
        printf("Face contains more than 3 vertices!\n");
        return 1;
      }
      fscanf(fp, "%d", faces + 3*i);
      fscanf(fp, "%d", faces + 3*i+1);
      fscanf(fp, "%d", faces + 3*i+2);
    }
  }
  else
    faces = NULL;

  fclose(fp);
  *pnv = nv;
  *pvertices = vertices;
  *pnf = nf;
  *pfaces = faces;
  return 0;
}


// Load anchor points from file
void load_anchor_points(int cad_index) {

  // Loop indices, number of vertices, number of faces
  int i, j, nv, nf;
  // Filename (for each anchor point location)
  char filename[256];
     
  // Names of each anchor point
  // char *names_car[] = {"left_front_wheel", "left_back_wheel",
  //           "right_front_wheel", "right_back_wheel",
  //           "upper_left_windshield", "upper_right_windshield",
  //           "upper_left_rearwindow", "upper_right_rearwindow",
  //           "left_front_light", "right_front_light",
  //           "left_back_trunk", "right_back_trunk"};
  // char *names_car[] = {"right_back_trunk"};
  char *names_car[] = {"back_upper_left","back_upper_right",
            "leg_lower_left","leg_lower_right",
            "leg_upper_left", "leg_upper_right",
            "seat_upper_left", "seat_upper_right",
            "seat_lower_left", "seat_lower_right"};

  char **names;
  Nanchor = 10;
  names = names_car;

  GLfloat *vertices;
  GLuint *faces;

  Anchors = (GLfloat*)malloc(sizeof(GLfloat)*Nanchor*3);
  if(Anchors == NULL){
    printf("Out of memory!\n");
    exit(1);
  }
  memset(Anchors, 0, sizeof(GLfloat)*Nanchor*3);

  // Flag to hold visibility stats for keypoints
  IsExist = (int*)malloc(sizeof(int)*Nanchor);
  if(IsExist == NULL){
    printf("Out of memory!\n");
    exit(1);
  }

  // Load coordinates for each keypoint
  for(i = 0; i < Nanchor; i++) {
    sprintf(filename, "/home/sarthak/Documents/RayTracing/chair/%02d_%s.off", cad_index, names[i]);
    load_off_file(&nv, &vertices, &nf, &faces, filename);
    if(nv == 0){
      // printf("%s does not exist\n", names[i]);
      IsExist[i] = 0;
    }
    else{
      IsExist[i] = 1;
      for(j = 0; j < nv; j++)
      {
        Anchors[3*i] += vertices[3*j];
        Anchors[3*i+1] += vertices[3*j+1];
        Anchors[3*i+2] += vertices[3*j+2];
      }
      Anchors[3*i] /= nv;
      Anchors[3*i+1] /= nv;
      Anchors[3*i+2] /= nv;
      // printf("%s: %.4f, %.4f, %.4f\n", names[i], Anchors[3*i], Anchors[3*i+1], Anchors[3*i+2]);
      free(vertices);
      if(faces != NULL)
        free(faces);
    }
  }
}


// Drawing function
void display(void) {

  int i, aind, eind;
  int *visibility;
  GLint viewport[4];
  GLdouble mvmatrix[16], projmatrix[16];
  GLdouble x, y, z;
  GLdouble a = 0, e = 0, d = 3;
  GLfloat depth;

  glEnable(GL_DEPTH_TEST);

  // Vertices array
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 0, Vertices);

  // Set the azimuth and elevation value for which visibility is to be tested
  a = -90 + azimuth;
  e = -90 ;

  // Check visibility
  glClearColor(1.0, 1.0, 1.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(30.0, 1.0, d-0.5, d+0.5);
  glViewport(0, 0, WIDTH, HEIGHT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -d);
  glRotatef(e, 1.0, 0.0, 0.0);
  glRotatef(-a, 0.0, 0.0, 1.0);

  // Draw lines
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glColor3f(0.0, 0.0, 1.0);
  glDrawElements(GL_TRIANGLES, 3*Nface, GL_UNSIGNED_INT, Faces);

  // Hidden-line removal
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0, 1.0);
  glColor3f(1.0, 1.0, 1.0);
  glDrawElements(GL_TRIANGLES, 3*Nface, GL_UNSIGNED_INT, Faces);
  glDisable(GL_POLYGON_OFFSET_FILL);

  // Get the matrices
  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);

  // Check visibility for each keypoint
  glColor3f(1.0, 0.0, 0.0);
  glPointSize(10.0);
  visibility = (int*)malloc(sizeof(int)*Nanchor);
  if(visibility == NULL){
    printf("out of memory\n");
    exit(1);
  }
  for(i = 0; i < Nanchor; i++)
  {
  if(IsExist[i] == 0){
    visibility[i] = 0;
    continue;
  }
  gluProject(Anchors[3*i], Anchors[3*i+1], Anchors[3*i+2], mvmatrix, projmatrix, viewport, &x, &y, &z);
  glReadPixels((GLint)x, (GLint)y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

  if(z <= depth+0.05){
    visibility[i] = 1;
    glBegin(GL_POINTS);
    glVertex3f(Anchors[3*i], Anchors[3*i+1], Anchors[3*i+2]);
    glEnd();
  }
  else
    visibility[i] = 0;
  }

  // Write to file
  if(Flag == 1){
    // fprintf(Fp, "%f %f\n", a, e);
    // printf("%f %f\n", a, e);
    for(i = 0; i < Nanchor; i++){
      fprintf(Fp, "%d ", visibility[i]);
      // printf("%d ", visibility[i]);
    }
    fprintf(Fp, "\n");
    // printf("\n");
    Flag = 0;
    fclose(Fp);
  }

  free(visibility);
  glFlush();

  // glutLeaveMainLoop();
  return;
}


// Reshape the viewpoint
void reshape(int w, int h) {
  glViewport(0, 0, (GLsizei)w, (GLsizei)h);
  // glutLeaveMainLoop();
  return;
}


// Main function
int main(int argc, char** argv) {

  // Name of the file containing the CAD model (.off file)
  char filename[256];

  // Verify that the requisite number of command-line arguments have been passed
  if(argc != 3){
    printf("Usage: ./keypoint_visibility cad_index(usually 1) azimuth(in degrees)\n");
    return 1;
  }

  // Initialize display parameters
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(WIDTH, HEIGHT);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("anchor_depth_test");

  // Filename of the .off file
  // sprintf(filename, "%s/%02d.off", argv[1], atoi(argv[2]));
  int cad_index = atoi(argv[1]);
  azimuth = atof(argv[2]);

  sprintf(filename, "/home/sarthak/Documents/RayTracing/chair/%02d.off", cad_index);
  
  // Load the CAD model from the .off file
  load_off_file(&Nvertice, &Vertices, &Nface, &Faces, filename);
  // printf(".off file loaded!\n");
  
  // Read anchor points (keypoints) from the .off file
  load_anchor_points(cad_index);
  // printf("Keypoints read\n");
  
  // Open output file
  sprintf(filename, "/home/sarthak/Documents/RayTracing/chair/visibility_output.txt");
  Fp = fopen(filename, "w");
  Flag = 1;

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutMainLoop();
  display();

  return 0;
}
