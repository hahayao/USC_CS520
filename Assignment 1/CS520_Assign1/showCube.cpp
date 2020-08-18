/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/
#define GL_SILENCE_DEPRECATION
#include "jello.h"
#include "showCube.h"
#include "SOIL2.h"
#include <iostream>

using namespace std;

//absolute path for all textures
string texturePath[6] = {
                            "/Users/haha/Documents/Prac_C++/CS520_Assign1/texture/dice1.jpg",
                            "/Users/haha/Documents/Prac_C++/CS520_Assign1/texture/dice2.png",
                            "/Users/haha/Documents/Prac_C++/CS520_Assign1/texture/dice3.png",
                            "/Users/haha/Documents/Prac_C++/CS520_Assign1/texture/dice4.png",
                            "/Users/haha/Documents/Prac_C++/CS520_Assign1/texture/dice5.png",
                            "/Users/haha/Documents/Prac_C++/CS520_Assign1/texture/dice6.png",
                        };

//store texure integer
unsigned int texName[6];

point callFacesPara[6][4];



int pointMap(int side, int i, int j)
{
  int r;

  switch (side)
  {
  case 1: //[i][j][0] bottom face
    r = 64 * i + 8 * j;
    break;
  case 6: //[i][j][7] top face
    r = 64 * i + 8 * j + 7;
    break;
  case 2: //[i][0][j] front face
    r = 64 * i + j;
    break;
  case 5: //[i][7][j] back face
    r = 64 * i + 56 + j;
    break;
  case 3: //[0][i][j] left face
    r = 8 * i + j;
    break;
  case 4: //[7][i][j] right face
    r = 448 + 8 * i + j;
    break;
  }

  return r;
}

void showCube(struct world * jello)
{
  int i,j,k,ip,jp,kp;
  point r1,r2,r3; // aux variables
  
  /* normals buffer and counter for Gourad shading*/
  struct point normal[8][8];
  int counter[8][8];

  int face;
  double faceFactor, length;

  if (fabs(jello->p[0][0][0].x) > 10)
  {
    printf ("Your cube somehow escaped way out of the box.\n");
    exit(0);
  }

  
  #define NODE(face,i,j) (*((struct point * )(jello->p) + pointMap((face),(i),(j))))

  
  #define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
       && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
    {\
      glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
      glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
    }\


    //draw incline plane
    if(jello->incPlanePresent != 0)
    {
        //ax + by + cz + d = 0
        //x = 2, z = 2
        double pointInc[3][3];

        
        pointInc[0][1] = 2;
        pointInc[0][2] = -3;
        pointInc[0][0] = (-jello->d - jello->b * pointInc[0][1] - jello->c * pointInc[0][2]) * 1.0 / jello->a;
        
        
        pointInc[1][1] = -2;
        pointInc[1][2] = 3;
        pointInc[1][0] = (-jello->d - jello->b * pointInc[1][1] - jello->c * pointInc[1][2]) * 1.0 / jello->a;
        
        pointInc[2][0] = 2;
        
        pointInc[2][2] = -2;
        pointInc[2][1] = (-jello->d - jello->c * pointInc[2][2] - jello->a * pointInc[2][0]) * 1.0 / jello->b;

        glPolygonMode(GL_FRONT, GL_FILL);
        glPolygonMode(GL_BACK, GL_FILL);
        glBegin(GL_TRIANGLES); // draw point
        glVertex3f(pointInc[0][0],pointInc[0][1],pointInc[0][2]);
        glVertex3f(pointInc[1][0],pointInc[1][1],pointInc[1][2]);
        glVertex3f(pointInc[2][0],pointInc[2][1],pointInc[2][2]);
        glVertex3f(pointInc[0][0],pointInc[0][1],pointInc[0][2]);
        glVertex3f(pointInc[2][0],pointInc[2][1],pointInc[2][2]);
        glVertex3f(pointInc[1][0],pointInc[1][1],pointInc[1][2]);
        glEnd();

    }
    
  if (viewingMode==0) // render wireframe
  {
    glLineWidth(1);
    glPointSize(5);
    glDisable(GL_LIGHTING);
    for (i=0; i<=7; i++)
      for (j=0; j<=7; j++)
        for (k=0; k<=7; k++)
        {
          if (i*j*k*(7-i)*(7-j)*(7-k) != 0) // not surface point
            continue;

          glBegin(GL_POINTS); // draw point
            glColor4f(0,0,0,0);  
            glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);        
          glEnd();

          //
          //if ((i!=7) || (j!=7) || (k!=7))
          //  continue;

          glBegin(GL_LINES);      
          // structural
          if (structural == 1)
          {
              switch(selectedSurface){
                      //front
                  case 0:
                      if(k == 0)
                          glColor4f(1,1,0,1);
                      break;
                      //left
                  case 1:
                      if(i == 0)
                          glColor4f(1,1,0,1);
                      break;
                  case 2:
                      if(i == 7)
                          glColor4f(1,1,0,1);
                      break;
                  case 3:
                      if(k == 7)
                          glColor4f(1,1,0,1);
                      break;
                  case 4:
                      if(j == 7)
                          glColor4f(1,1,0,1);
                      break;
                  case 5:
                      if(j == 0)
                          glColor4f(1,1,0,1);
                      break;
                  default:
                      glColor4f(0,0,1,1);
                      break;
              }
            //glColor4f(0,0,1,1);
            PROCESS_NEIGHBOUR(1,0,0);
            PROCESS_NEIGHBOUR(0,1,0);
            PROCESS_NEIGHBOUR(0,0,1);
            PROCESS_NEIGHBOUR(-1,0,0);
            PROCESS_NEIGHBOUR(0,-1,0);
            PROCESS_NEIGHBOUR(0,0,-1);
          }
          
          // shear
          if (shear == 1)
          {
            glColor4f(0,1,0,1);
            PROCESS_NEIGHBOUR(1,1,0);
            PROCESS_NEIGHBOUR(-1,1,0);
            PROCESS_NEIGHBOUR(-1,-1,0);
            PROCESS_NEIGHBOUR(1,-1,0);
            PROCESS_NEIGHBOUR(0,1,1);
            PROCESS_NEIGHBOUR(0,-1,1);
            PROCESS_NEIGHBOUR(0,-1,-1);
            PROCESS_NEIGHBOUR(0,1,-1);
            PROCESS_NEIGHBOUR(1,0,1);
            PROCESS_NEIGHBOUR(-1,0,1);
            PROCESS_NEIGHBOUR(-1,0,-1);
            PROCESS_NEIGHBOUR(1,0,-1);

            PROCESS_NEIGHBOUR(1,1,1)
            PROCESS_NEIGHBOUR(-1,1,1)
            PROCESS_NEIGHBOUR(-1,-1,1)
            PROCESS_NEIGHBOUR(1,-1,1)
            PROCESS_NEIGHBOUR(1,1,-1)
            PROCESS_NEIGHBOUR(-1,1,-1)
            PROCESS_NEIGHBOUR(-1,-1,-1)
            PROCESS_NEIGHBOUR(1,-1,-1)
          }
          
          // bend
          if (bend == 1)
          {
            glColor4f(1,0,0,1);
            PROCESS_NEIGHBOUR(2,0,0);
            PROCESS_NEIGHBOUR(0,2,0);
            PROCESS_NEIGHBOUR(0,0,2);
            PROCESS_NEIGHBOUR(-2,0,0);
            PROCESS_NEIGHBOUR(0,-2,0);
            PROCESS_NEIGHBOUR(0,0,-2);
          }           
          glEnd();
        }
    glEnable(GL_LIGHTING);
  }
  
    else if(viewingMode == 2)
    {
        defineValue(jello);
        for(int i = 0; i < 6; i++)
        {
            drawJelloTexture(i);
        }
        glEnable(GL_LIGHTING);
    }
    
  else
  {
    glPolygonMode(GL_FRONT, GL_FILL); 
    
    for (face=1; face <= 6; face++) 
      // face == face of a cube
      // 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
    {
      
      if ((face==1) || (face==3) || (face==5))
        faceFactor=-1; // flip orientation
      else
        faceFactor=1;
      

      for (i=0; i <= 7; i++) // reset buffers
        for (j=0; j <= 7; j++)
        {
          normal[i][j].x=0;normal[i][j].y=0;normal[i][j].z=0;
          counter[i][j]=0;
        }

      /* process triangles, accumulate normals for Gourad shading */
  
      for (i=0; i <= 6; i++)
        for (j=0; j <= 6; j++) // process block (i,j)
        {
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i,j),r1); // first triangle
          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i][j],r3,normal[i][j]);
          counter[i][j]++;

          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
          counter[i+1][j+1]++;
        }

      
        /* the actual rendering */
        for (j=1; j<=7; j++) 
        {

          if (faceFactor  > 0)
            glFrontFace(GL_CCW); // the usual definition of front face
          else
            glFrontFace(GL_CW); // flip definition of orientation
         
          glBegin(GL_TRIANGLE_STRIP);
          for (i=0; i<=7; i++)
          {
            glNormal3f(normal[i][j].x / counter[i][j],normal[i][j].y / counter[i][j],
              normal[i][j].z / counter[i][j]);
            glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);
            glNormal3f(normal[i][j-1].x / counter[i][j-1],normal[i][j-1].y/ counter[i][j-1],
              normal[i][j-1].z / counter[i][j-1]);
            glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);
          }
          glEnd();
        }
        
        
    }  
  } // end for loop over faces
  glFrontFace(GL_CCW);
    
}

void showBoundingBox()
{
  int i,j;

  glColor4f(0.6,0.6,0.6,0);

  glBegin(GL_LINES);

  // front face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,-2,-2);
    glVertex3f(i,-2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(2,-2,j);
  }

  // back face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,2,-2);
    glVertex3f(i,2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,2,j);
    glVertex3f(2,2,j);
  }

  // left face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(-2,i,-2);
    glVertex3f(-2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(-2,2,j);
  }

  // right face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(2,i,-2);
    glVertex3f(2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(2,-2,j);
    glVertex3f(2,2,j);
  }
  
  glEnd();

  return;
}

void initTextMain()
{
    glGenTextures(6, texName);
    for(int k = 0; k < 6; k++)
    {
        initTexture(k);

    }
}

void initTexture(int m)
{
    int width, height;
    
    char fileName[60];
    strcpy(fileName, texturePath[m].c_str());

    unsigned char *ht_map = SOIL_load_image
    (
        fileName,
        &width, &height, 0,
        SOIL_LOAD_RGBA
    );
    cout<<SOIL_last_result()<<endl;
        
    glBindTexture(GL_TEXTURE_2D, texName[m]); // make texture
    cout<<texName[m]<<endl;

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        
    // use linear filter both for magnification and minification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    
    // load image data stored at pointer “pointerToImage” into the currently active texture (“texName”)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, ht_map);
    cout<<width<<","<<height<<endl;
    SOIL_free_image_data(ht_map);
}

void face(point a, point b, point c, point d)
{
    glBegin(GL_QUADS);
    glTexCoord2f(0.0,1.0);glVertex3d(a.x, a.y, a.z);
    
    //cout<<a.x<<b.x<<c.x<<endl;
      
    glTexCoord2f(1.0,1.0);glVertex3f(b.x, b.y, b.z);
    
    glTexCoord2f(1.0,0.0);glVertex3f(c.x, c.y, c.z);
      
    glTexCoord2f(0.0,0.0);glVertex3f(d.x, d.y, d.z);
    glEnd();
}

//6 faces for dice
//faces for box

void drawJelloTexture(int m)
{
      // turn on texture mapping (this disables standard OpenGL lighting, unless in GL_MODULATE mode)
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texName[m]);
    //cout<<texName[0]<<endl;
    // no modulation of texture color with lighting; use texture color directly
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_BLEND);
    face(callFacesPara[m][0], callFacesPara[m][1],callFacesPara[m][2], callFacesPara[m][3]);
    
    //cout<<callFacesPara[0][0].x<<","<<callFacesPara[0][0].y<<endl;
      // turn off texture mapping
    glDisable(GL_TEXTURE_2D);
}

void defineValue(struct world * jello)
{

    //front
    callFacesPara[0][0] = jello->p[0][0][0];
    callFacesPara[0][1] = jello->p[7][0][0];
    callFacesPara[0][2] = jello->p[7][0][7];
    callFacesPara[0][3] = jello->p[0][0][7];
    //right
    callFacesPara[1][0] = jello->p[7][0][0];
    callFacesPara[1][1] = jello->p[7][7][0];
    callFacesPara[1][2] = jello->p[7][7][7];
    callFacesPara[1][3] = jello->p[7][0][7];
    //back
    callFacesPara[2][0] = jello->p[7][7][0];
    callFacesPara[2][1] = jello->p[0][7][0];
    callFacesPara[2][2] = jello->p[0][7][7];
    callFacesPara[2][3] = jello->p[7][7][7];
    //left
    callFacesPara[3][0] = jello->p[0][7][0];
    callFacesPara[3][1] = jello->p[0][0][0];
    callFacesPara[3][2] = jello->p[0][0][7];
    callFacesPara[3][3] = jello->p[0][7][7];
    //top
    callFacesPara[4][0] = jello->p[0][0][7];
    callFacesPara[4][1] = jello->p[7][0][7];
    callFacesPara[4][2] = jello->p[7][7][7];
    callFacesPara[4][3] = jello->p[0][7][7];
    //bottom
    callFacesPara[5][0] = jello->p[0][7][0];
    callFacesPara[5][1] = jello->p[7][7][0];
    callFacesPara[5][2] = jello->p[7][0][0];
    callFacesPara[5][3] = jello->p[0][0][0];
}

