/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"
#include <vector>
#include <iostream>

using namespace std;


struct SPringInfo
{
    int point1[3];
    int point2[3];
    double distance;
};

struct Diagonal
{
    int point1[3];
    int point2[3];
    double distance;
};

vector<SPringInfo> allSpring;

struct EquationCoe
{
    double coe[4];
    //if multiplication > 0, initial =  1; else initial = -1;
    int initial;
};

vector<EquationCoe> allEquation;

//check the plane side of each side
int InitialPlaneSide(double array[4], double pt[3])
{
    double result = array[0] * pt[0] + array[1] * pt[1] + array[2] * pt[2] + array[3];
    if(result > 0)
        return 1;
    else
        return -1;
}

void NormalizeVector(double vec[3])
{
    double originalPoint[3] = {0};
    double length = CalRestLength(vec, originalPoint);
    for(int i = 0; i < 3; i++)
    {
        vec[i] /= length;
    }
}

void CreateBoundaryEquation(struct world * jello)
{
    allEquation.resize(0);
    EquationCoe temp;
    double pt[3];
    pt[0] = jello->p[0][0][0].x;
    pt[1] = jello->p[0][0][0].y;
    pt[2] = jello->p[0][0][0].z;
    
    //x = -2, 2;
    temp.coe[0] = 0.5; temp.coe[1] = 0;  temp.coe[2] = 0; temp.coe[3] = 1;
    temp.initial = InitialPlaneSide(temp.coe, pt);
    allEquation.push_back(temp);
    
    temp.coe[0] = 0.5; temp.coe[1] = 0;  temp.coe[2] = 0; temp.coe[3] = -1;
    temp.initial = InitialPlaneSide(temp.coe, pt);
    allEquation.push_back(temp);
    
    //y = -2, 2;
    temp.coe[0] = 0; temp.coe[1] = 0.5;  temp.coe[2] = 0; temp.coe[3] = 1;
    temp.initial = InitialPlaneSide(temp.coe, pt);
    allEquation.push_back(temp);
    
    temp.coe[0] = 0; temp.coe[1] = 0.5;  temp.coe[2] = 0; temp.coe[3] = -1;
    temp.initial = InitialPlaneSide(temp.coe, pt);
    allEquation.push_back(temp);
    
    //z = -2, 2;
    temp.coe[0] = 0; temp.coe[1] = 0;  temp.coe[2] = 0.5; temp.coe[3] = 1;
    temp.initial = InitialPlaneSide(temp.coe, pt);
    allEquation.push_back(temp);
    
    temp.coe[0] = 0; temp.coe[1] = 0;  temp.coe[2] = 0.5; temp.coe[3] = -1;
    temp.initial = InitialPlaneSide(temp.coe, pt);
    allEquation.push_back(temp);
    //add additional coe
    if(jello->incPlanePresent != 0)
    {
        temp.coe[0] = jello->a; temp.coe[1] = jello->b;  temp.coe[2] = jello->c; temp.coe[3] = jello->d;
        temp.initial = InitialPlaneSide(temp.coe, pt);
        allEquation.push_back(temp);
    }
}

int CheckPlaneInOut(double pt[3])
{
    for(int i = 0; i < 6; i++)
    {
        double result = allEquation[i].coe[0] * pt[0] + allEquation[i].coe[1] * pt[1] + allEquation[i].coe[2] * pt[2] + allEquation[i].coe[3];
        if(result * allEquation[i].initial > 0)
        {
            continue;
        }
        else
        {
//            if(abs(result) < 0.000001)
//            {
//                continue;
//            }
            return i;
        }
    }
    //means the point has no collision
    return -1;
}

int CheckInclineInOut(double pt[3])
{
    double result = allEquation[6].coe[0] * pt[0] + allEquation[6].coe[1] * pt[1] + allEquation[6].coe[2] * pt[2] + allEquation[6].coe[3];
    if(result * allEquation[6].initial > 0)
    {
        return -1;
    }
    return 6;
}


//create Structural Spring System
void CreateStructuralSpring()
{
    allSpring.resize(0);
    SPringInfo temp;
    
    for(int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 8; j++)
        {
            for(int k = 0; k < 8; k++)
            {
                temp.distance = 1.0 / 7;
                temp.point1[0] = i; temp.point1[1] = j; temp.point1[2] = k;
                temp.point2[0] = i; temp.point2[1] = j; temp.point2[2] = k;
                
                //find right point
                if(i != 7)
                {
                    temp.point2[0] = i + 1;
                    allSpring.push_back(temp);
                }
                temp.point2[0] = i; temp.point2[1] = j; temp.point2[2] = k;
                
                //find downward point
                if(j != 7)
                {
                    temp.point2[1] = j + 1;
                    allSpring.push_back(temp);
                }
                temp.point2[0] = i; temp.point2[1] = j; temp.point2[2] = k;
                
                //find back point
                if(k != 7)
                {
                    temp.point2[2] = k + 1;
                    allSpring.push_back(temp);
                }
            }
        }
    }
}

void CreateBendSpring()
{
    SPringInfo temp;
    for(int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 8; j++)
        {
            for(int k = 0; k < 8; k++)
            {
                temp.distance = 1.0 / 7 * 2;
                temp.point1[0] = i; temp.point1[1] = j; temp.point1[2] = k;
                temp.point2[0] = i; temp.point2[1] = j; temp.point2[2] = k;
                
                //find right point
                if(i < 6)
                {
                    temp.point2[0] = i + 2;
                    allSpring.push_back(temp);
                }
                temp.point2[0] = i; temp.point2[1] = j; temp.point2[2] = k;
                
                //find downward point
                if(j < 6)
                {
                    temp.point2[1] = j + 2;
                    allSpring.push_back(temp);
                }
                temp.point2[0] = i; temp.point2[1] = j; temp.point2[2] = k;
                
                //find back point
                if(k < 6)
                {
                    temp.point2[2] = k + 2;
                    allSpring.push_back(temp);
                }
            }
        }
    }
}

void CreateDiagonalSpring()
{
    SPringInfo temp;
    
    //calculate sqrt(2) diagonals, check 6 directions for each point
    for(int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 8; j++)
        {
            for(int k = 0; k < 8; k++)
            {
                temp.point1[0] = i; temp.point1[1] = j; temp.point1[2] = k;
                temp.point2[0] = i; temp.point2[1] = j; temp.point2[2] = k;
                temp.distance = sqrt(2.0) / 7;
                //[i+1][j+/-1][k]
                if((j + 1 < 8) && (i + 1 < 8))
                {
                    temp.point2[0] = i + 1; temp.point2[1] = j + 1; temp.point2[2] = k;
                    allSpring.push_back(temp);
                }
                if((j - 1 >= 0) && (i + 1 < 8))
                {
                    temp.point2[0] = i + 1; temp.point2[1] = j - 1; temp.point2[2] = k;
                    allSpring.push_back(temp);
                }
                //[i][j+/-1][k+1]
                if((j + 1 < 8) && (k + 1 < 8))
                {
                    temp.point2[0] = i; temp.point2[1] = j + 1; temp.point2[2] = k + 1;
                    allSpring.push_back(temp);
                }
                if((j - 1 >= 0) && (k + 1 < 8))
                {
                    temp.point2[0] = i; temp.point2[1] = j - 1; temp.point2[2] = k + 1;
                    allSpring.push_back(temp);
                }
                //[i+1][j][k+/-1]
                if((i + 1 < 8) && (k + 1 < 8))
                {
                    temp.point2[0] = i + 1; temp.point2[1] = j; temp.point2[2] = k + 1;
                    allSpring.push_back(temp);
                }
                if((i + 1 < 8) && (k - 1 >= 0))
                {
                    temp.point2[0] = i + 1; temp.point2[1] = j; temp.point2[2] = k - 1;
                    allSpring.push_back(temp);
                }
            }
        }
    }
    //calculate sqrt(3)
    for(int i = 0; i < 7; i++)
    {
        for(int j = 0; j < 7; j++)
        {
            for(int k = 0; k < 7; k++)
            {
                temp.distance = sqrt(3.0) / 7;
                
                //[0][0][0]->[1][1][1]
                temp.point1[0] = i; temp.point1[1] = j; temp.point1[2] = k;
                temp.point2[0] = i + 1; temp.point2[1] = j + 1; temp.point2[2] = k + 1;
                allSpring.push_back(temp);
                //[1][0][0]->[0][1][1]
                temp.point1[0] = i + 1; temp.point1[1] = j; temp.point1[2] = k;
                temp.point2[0] = i; temp.point2[1] = j + 1; temp.point2[2] = k + 1;
                allSpring.push_back(temp);
                //[1][0][1]->[0][1][0]
                temp.point1[0] = i + 1; temp.point1[1] = j; temp.point1[2] = k + 1;
                temp.point2[0] = i; temp.point2[1] = j + 1; temp.point2[2] = k;
                allSpring.push_back(temp);
                //[0][0][1]->[1][1][0]
                temp.point1[0] = i; temp.point1[1] = j; temp.point1[2] = k + 1;
                temp.point2[0] = i + 1; temp.point2[1] = j + 1; temp.point2[2] = k;
                allSpring.push_back(temp);
            }
        }
    }
    //cout<<allSpring.size()<<endl;
}

//know the index no. of two points, calculate the distance within two points
double CalRestLength(double left[3], double right[3])
{
    double length = 0;
    for(int i = 0; i < 3; i++)
    {
        length += pow((left[i] - right[i]), 2);
    }
    length = sqrt(length);
    return length;
}

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  /* for you to implement ... */
    //F(total) = F(Hook) + F(Damping) + F(Forcefield)
    //clear Force
    double Force[8][8][8][3];
    for(int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 8; j++)
        {
            for(int k = 0; k < 8; k++)
            {
                for(int m = 0; m < 3; m++)
                {
                    Force[i][j][k][m] = 0;
                }
            }
        }
    }
    
    for(int i = 0; i < allSpring.size(); i++)
    {
        double leftPoint[3];
        double rightPoint[3];
        int leftIndex[3];
        int rightIndex[3];

        for(int n = 0; n < 3; n++)
        {
            leftIndex[n] = allSpring[i].point1[n];
            rightIndex[n] = allSpring[i].point2[n];
        }

        leftPoint[0] = jello->p[leftIndex[0]][leftIndex[1]][leftIndex[2]].x;
        leftPoint[1] = jello->p[leftIndex[0]][leftIndex[1]][leftIndex[2]].y;
        leftPoint[2] = jello->p[leftIndex[0]][leftIndex[1]][leftIndex[2]].z;
        
        rightPoint[0] = jello->p[rightIndex[0]][rightIndex[1]][rightIndex[2]].x;
        rightPoint[1] = jello->p[rightIndex[0]][rightIndex[1]][rightIndex[2]].y;
        rightPoint[2] = jello->p[rightIndex[0]][rightIndex[1]][rightIndex[2]].z;
        
        double distance = CalRestLength(leftPoint, rightPoint);
        //vector from leftPoint - rightPoint
        double vectorL[3] = {0, 0, 0};
        for(int m = 0; m < 3; m++)
        {
            vectorL[m] = leftPoint[m] - rightPoint[m];
        }
        
        //F(Hook)
        //coeffecient of F(Hook)
        double coHook = - jello->kElastic * (distance - allSpring[i].distance) / distance;
        double hookForce[3] = {0, 0, 0};
        //double vectorL[3] = {0, 0, 0};
        for(int n = 0; n < 3; n++)
        {
            hookForce[n] = vectorL[n] * coHook;
            Force[leftIndex[0]][leftIndex[1]][leftIndex[2]][n] += hookForce[n];
            Force[rightIndex[0]][rightIndex[1]][rightIndex[2]][n] -= hookForce[n];
        }

        //F(Damping)
        double coDamping = -jello->dElastic / pow(distance, 2);
        double Vsub[3] = {0, 0, 0};

        //left - right
        Vsub[0] = jello->v[leftIndex[0]][leftIndex[1]][leftIndex[2]].x - jello->v[rightIndex[0]][rightIndex[1]][rightIndex[2]].x;
        Vsub[1] = jello->v[leftIndex[0]][leftIndex[1]][leftIndex[2]].y - jello->v[rightIndex[0]][rightIndex[1]][rightIndex[2]].y;
        Vsub[2] = jello->v[leftIndex[0]][leftIndex[1]][leftIndex[2]].z - jello->v[rightIndex[0]][rightIndex[1]][rightIndex[2]].z;
        
        
        coDamping = coDamping * dotProduct(Vsub, vectorL);
        
        double dampForce[3] = {0, 0, 0};
        for(int n = 0; n < 3; n++)
        {
            dampForce[n] = coDamping * vectorL[n];
            Force[leftIndex[0]][leftIndex[1]][leftIndex[2]][n] += dampForce[n];
            Force[rightIndex[0]][rightIndex[1]][rightIndex[2]][n] -= dampForce[n];
        }
    }
    //check if collid with boundary
    for(int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 8; j++)
        {
            for(int k = 0; k < 8; k++)
            {
                double pt[3];
                pt[0] = jello->p[i][j][k].x;
                pt[1] = jello->p[i][j][k].y;
                pt[2] = jello->p[i][j][k].z;
                
                double ptv[3];
                ptv[0] = jello->v[i][j][k].x;
                ptv[1] = jello->v[i][j][k].y;
                ptv[2] = jello->v[i][j][k].z;
                
                int check = CheckPlaneInOut(pt);
                if(check == -1)
                {
                    //continue;
                }
                else
                {
                    //cout<<"CHECK: "<<check<<endl;
                    //Hook
                    //calculate the distance
                    double vectorL[3];
                    //double length = allEquation[check].coe[3];
                    double length = 0;
                    for(int m = 0; m < 3; m++)
                    {
                        length += pt[m] * allEquation[check].coe[m];
                    }
                    length += allEquation[check].coe[3];
                    //set the position of this point
                    length = abs(length);
                    
                    double divide = sqrt(pow(allEquation[check].coe[0], 2) + pow(allEquation[check].coe[1], 2) + pow(allEquation[check].coe[2], 2));
                    length /= divide;
                    
                    //find the corret normal vector
                   double checkDirection = 0;
                   for(int m = 0; m < 3; m++)
                   {
                       checkDirection += allEquation[check].coe[m] * ptv[m];
                       vectorL[m] = allEquation[check].coe[m] / divide * length;
                   }
                   //cos(theta) < 0, need to change normal vector's direction
                   if(checkDirection < 0)
                   {
                       for(int m = 0; m < 3; m++)
                       {
                           vectorL[m] = -vectorL[m];
                           
                       }
                   }
                    
                    switch (check) {
                        case 0:
                            jello->p[i][j][k].x = -2;
                            break;
                        case 1:
                            jello->p[i][j][k].x = 2;
                            break;
                        case 2:
                            jello->p[i][j][k].y = -2;
                            break;
                        case 3:
                            jello->p[i][j][k].y = 2;
                            break;
                        case 4:
                            jello->p[i][j][k].z = -2;
                            break;
                        case 5:
                            jello->p[i][j][k].z = 2;
                            //cout<<"updated Z"<<vectorL[2]<<endl;
                            break;
//
//                        case 6:
//                            if(abs(vectorL[0]) < 0.8 && abs(pt[0]) < 8)
//                            {
//                                jello->p[i][j][k].x -= vectorL[0];
//                            }
//                            if(abs(vectorL[1]) < 0.8 && abs(pt[1]) < 8)
//                            {
//                                jello->p[i][j][k].y -= vectorL[1];
//                            }
//                            if(abs(vectorL[2]) < 0.8 && abs(pt[2]) < 8)
//                            {
//                                jello->p[i][j][k].z -= vectorL[2];
//                            }
//
//                            break;
                    }
//                    cout<<jello->p[i][j][k].x<<", "<<jello->p[i][j][k].y<<", "<<jello->p[i][j][k].z<<endl;
//                    if(vectorL[0] > 1 || vectorL[1] > 1 || vectorL[2] > 1)
//                    {
//                        cout<<i<<","<<j<<","<<k<<endl;
//                        cout<<"vector"<<vectorL[0]<<","<<vectorL[1]<<","<<vectorL[2]<<endl;
//                        cout<<divide<<endl;
//                        cout<<length<<endl;
//                        cout<<jello->p[i][j][k].x<<", "<<jello->p[i][j][k].y<<", "<<jello->p[i][j][k].z<<endl;
//                        cout<<- jello->kCollision * vectorL[0]<<", "<<- jello->kCollision * vectorL[1]<<endl;
//                    }
                    
                    double collisionHookForce[3];
                    //hook
                    for(int m = 0; m < 3; m++)
                    {
                        collisionHookForce[m] = - jello->kCollision * vectorL[m];
                        Force[i][j][k][m] += collisionHookForce[m];
                        
                    }
                    
                    double collisionDampForce[3];
                    //damping
                    double coCOllisionDamp = - jello->dCollision * dotProduct(ptv, vectorL) / pow(length, 2);
                    for(int m = 0; m < 3; m++)
                    {
                        collisionDampForce[m] = coCOllisionDamp * vectorL[m];
                        Force[i][j][k][m] += collisionDampForce[m];
                    }
                    
//                    cout<<collisionDampForce[0]<<", "<<collisionHookForce[0]<<","<<Force[i][j][k][0]<<endl;
//                    cout<<collisionDampForce[1]<<", "<<collisionHookForce[1]<<","<<Force[i][j][k][1]<<endl;
//                    cout<<collisionDampForce[2]<<", "<<collisionHookForce[2]<<","<<Force[i][j][k][2]<<endl;
                }
                
                //check incline
                if(jello->incPlanePresent != 0)
                {
                    pt[0] = jello->p[i][j][k].x;
                    pt[1] = jello->p[i][j][k].y;
                    pt[2] = jello->p[i][j][k].z;


                    check = CheckInclineInOut(pt);
                    if(check == -1)
                    {
                        //
                    }
                    else
                    {
                        //Hook
                        //calculate the distance
                        double vectorL[3];
                        //double length = allEquation[check].coe[3];
                        double length = 0;
                        for(int m = 0; m < 3; m++)
                        {
                            length += pt[m] * allEquation[check].coe[m];
                        }
                        length += allEquation[check].coe[3];
                        //set the position of this point
                        length = abs(length);

                        double divide = sqrt(pow(allEquation[check].coe[0], 2) + pow(allEquation[check].coe[1], 2) + pow(allEquation[check].coe[2], 2));
                        length = 1.0 * length / divide;

                        //find the corret normal vector
                       double checkDirection = 0;
                       for(int m = 0; m < 3; m++)
                       {
                           checkDirection += allEquation[check].coe[m] * ptv[m];
                           vectorL[m] = allEquation[check].coe[m] / divide * length;
                       }

                       //cos(theta) < 0, need to change normal vector's direction
                       if(checkDirection < 0)
                       {
                           for(int m = 0; m < 3; m++)
                           {
                               vectorL[m] = -vectorL[m];
                           }
                       }
    //                    if(abs(vectorL[0]) < 3 && abs(pt[0]) < 8)
    //                    {
                            jello->p[i][j][k].x -= vectorL[0];
    //                    }
    //                    if(abs(vectorL[1]) < 3 && abs(pt[1]) < 8)
    //                    {
                            jello->p[i][j][k].y -= vectorL[1];
    //                    }
    //                    if(abs(vectorL[2]) < 3 && abs(pt[2]) < 8)
    //                    {
                            jello->p[i][j][k].z -= vectorL[2];
    //                    }

                        double collisionHookForce[3];
                        //hook
                        for(int m = 0; m < 3; m++)
                        {
                            collisionHookForce[m] = - jello->kCollision * vectorL[m];
                            Force[i][j][k][m] += collisionHookForce[m];

                        }

                        double collisionDampForce[3];
                        //damping
                        double coCOllisionDamp = - jello->dCollision * dotProduct(ptv, vectorL) / pow(length, 2);
                        for(int m = 0; m < 3; m++)
                        {
                            collisionDampForce[m] = coCOllisionDamp * vectorL[m];
                            Force[i][j][k][m] += collisionDampForce[m];
                        }
                    }
                }
                
            }
        }
    }

    //check force Field
    if(jello->resolution != 0)
    {
        int resol = jello->resolution;
        //
        for(int i = 0; i < 8; i++)
        {
            for(int j = 0; j < 8; j++)
            {
                for(int k = 0; k < 8; k++)
                {
                    //current point position
                    double point[3] = {jello->p[i][j][k].x, jello->p[i][j][k].y, jello->p[i][j][k].z};
                    int pointGridIndex[3];
                    double posInGrid[3];
                    //find grid index no.
                    for(int m = 0; m < 3; m++)
                    {
                        pointGridIndex[m] = (int) ((point[m] + 2) / 4 * (resol - 1));
                        if(pointGridIndex[m] < 0)
                        {
                            pointGridIndex[m] = 0;
                        }
                        if(pointGridIndex[m] > resol - 1)
                        {
                            pointGridIndex[m] = resol - 1;
                        }
                        
                        //calculate the relative position in grid
                        posInGrid[m] = ((point[m] + 2) - pointGridIndex[m] * 4.0 / (resol - 1)) / (4.0 / (resol - 1));
                        if(posInGrid[m] < 0)
                        {
                            posInGrid[m] = 0;
                        }
                        if(posInGrid[m] > 1)
                        {
                            posInGrid[m] = 1;
                        }
                    }

                    //interpolate
                    //double fieldForce[3];
                    //store index for 8 vertices
                    //double field[8];
                    //front face
                    int field000 = pointGridIndex[0] * resol * resol + pointGridIndex[1] * resol + pointGridIndex[2];
                    int field100 = (pointGridIndex[0] + 1) * resol * resol + pointGridIndex[1] * resol + pointGridIndex[2];
                    int field010 = pointGridIndex[0] * resol * resol + (pointGridIndex[1] + 1) * resol + pointGridIndex[2];
                    int field110 = (pointGridIndex[0] + 1) * resol * resol + (pointGridIndex[1] + 1) * resol + pointGridIndex[2];
                    //back face
                    int field001 = pointGridIndex[0] * resol * resol + pointGridIndex[1] * resol + pointGridIndex[2] + 1;
                    int field101 = (pointGridIndex[0] + 1) * resol * resol + pointGridIndex[1] * resol + pointGridIndex[2] + 1;
                    int field011 = pointGridIndex[0] * resol * resol + (pointGridIndex[1] + 1) * resol + pointGridIndex[2] + 1;
                    int field111 = (pointGridIndex[0] + 1) * resol * resol + (pointGridIndex[1] + 1) * resol + pointGridIndex[2] + 1;
                    
                    struct point force000, force001, force010, force011, force100, force101, force110, force111;
                    pCPY(jello->forceField[field000] ,force000);
                    pCPY(jello->forceField[field001] ,force001);
                    pCPY(jello->forceField[field010] ,force010);
                    pCPY(jello->forceField[field011] ,force011);
                    pCPY(jello->forceField[field100] ,force100);
                    pCPY(jello->forceField[field101] ,force101);
                    pCPY(jello->forceField[field110] ,force110);
                    pCPY(jello->forceField[field111] ,force111);
                    //front face interpolation
                    struct point frontForce;
                    struct point coe0, coe1, coe2;
                    pSUB(force100, force000, coe0);
                    pMULTIPLY(coe1, posInGrid[0], coe0);
                    
                    pSUB(force010, force000, coe1);
                    pMULTIPLY(coe2, posInGrid[1], coe1);
                    
                    pSUM(force110, force000, coe2);
                    pSUB(coe2, force010, coe2);
                    pSUB(coe2, force100, coe2);
                    pMULTIPLY(coe2, posInGrid[0] * posInGrid[1], coe2);
                    
                    pSUM(coe0, coe1, frontForce);
                    pSUM(frontForce, coe2, frontForce);
                    pSUM(frontForce, force000, frontForce);
                    
                    //back face interpolation
                    struct point backForce;
                    pSUB(force101, force001, coe0);
                    pMULTIPLY(coe1, posInGrid[0], coe0);
                    
                    pSUB(force011, force001, coe1);
                    pMULTIPLY(coe2, posInGrid[1], coe1);
                    
                    pSUM(force111, force001, coe2);
                    pSUB(coe2, force011, coe2);
                    pSUB(coe2, force101, coe2);
                    pMULTIPLY(coe2, posInGrid[0] * posInGrid[1], coe2);
                    
                    pSUM(coe0, coe1, backForce);
                    pSUM(backForce, coe2, backForce);
                    pSUM(backForce, force001, backForce);
                    
                    struct point finalForce;
                    //finalForce = frontForce + z * (backForce - frontForce)
                    struct point temp;
                    pSUB(backForce, frontForce, temp);
                    pMULTIPLY(temp, posInGrid[2], temp);
                    pSUM(frontForce, temp, finalForce);
                    
                    Force[i][j][k][0] += finalForce.x;
                    Force[i][j][k][1] += finalForce.y;
                    Force[i][j][k][2] += finalForce.z;
                    }
                }
            }
        }
    
    //add userforce
    if(!(UserForce[0] == 0 && UserForce[1] == 0))
    {
        //selectedSurface
        //cout<<"ALLForce"<<Force[0][0][0][0] / jello->mass<<", "<<Force[0][0][0][1] / jello->mass<<endl;
        //magnify this force
        double magnify = 0.3;
        double userForcex = (double)UserForce[0];
        double userForcez = (double)UserForce[1];
        double distance = sqrt(pow(userForcex, 2) + pow(userForcez, 2));
        userForcex = userForcex / distance * magnify;
        userForcez = userForcez / distance * magnify;

        //cout<<"Reach"<<userForcex<<", "<<userForcey<<endl;
        //cout<<"ALLForce"<<Force[0][0][0][0]<<", "<<Force[0][0][0][1]<<endl;
        UserForce[0] = 0;
        UserForce[1] = 0;
        for(int i = 0; i < 8; i++)
        {
            for(int j = 0; j < 8; j++)
            {
                for(int k = 0; k < 8; k++)
                {
                    if(i * j * k * (i - 7) * (j - 7) * (k - 7) != 0)
                    {
                        continue;
                    }
                    
//                    jello->p[i][j][k].x += userForcex;
//                    jello->p[i][j][k].y += userForcey;
//                    Force[i][j][k][0] += userForcex;
//                    Force[i][j][k][1] += userForcey;
                    switch(selectedSurface)
                    {
                            //front
                        case 0:
                            if(k == 0)
                            {
                                jello->p[i][j][k].x += userForcex;
                                jello->p[i][j][k].z += userForcez;
                                Force[i][j][k][0] += userForcex;
                                Force[i][j][k][2] += userForcez;
                            }
                            break;
                            //left
                        case 1:
                            if(i == 0)
                            {
                                jello->p[i][j][k].x += userForcex;
                                jello->p[i][j][k].z += userForcez;
                                Force[i][j][k][0] += userForcex;
                                Force[i][j][k][2] += userForcez;
                            }
                            break;
                        case 2:
                            if(i == 7)
                                {
                                    jello->p[i][j][k].x += userForcex;
                                    jello->p[i][j][k].z += userForcez;
                                    Force[i][j][k][0] += userForcex;
                                    Force[i][j][k][2] += userForcez;
                                }
                            break;
                        case 3:
                            if(k == 7)
                                {
                                    jello->p[i][j][k].x += userForcex;
                                    jello->p[i][j][k].z += userForcez;
                                    Force[i][j][k][0] += userForcex;
                                    Force[i][j][k][2] += userForcez;
                                }
                            break;
                        case 4:
                            if(j == 7)
                                {
                                    jello->p[i][j][k].x += userForcex;
                                    jello->p[i][j][k].z += userForcez;
                                    Force[i][j][k][0] += userForcex;
                                    Force[i][j][k][2] += userForcez;
                                }
                            break;
                        case 5:
                            if(j == 0)
                                {
                                    jello->p[i][j][k].x += userForcex;
                                    jello->p[i][j][k].z += userForcez;
                                    Force[i][j][k][0] += userForcex;
                                    Force[i][j][k][2] += userForcez;
                                }
                            break;
                        default:
                            break;
                    }
                }
            }
        }
        
        UserForce[0] = 0;
        UserForce[1] = 0;
        //cout<<"ALLForce"<<Force[0][0][0][0] / jello->mass<<", "<<Force[0][0][0][1] / jello->mass<<endl;
    }
    
    //cout<<"Acceleration:"<<a[0][0][0].x<<endl;
    //acceleration
    for(int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 8; j++)
        {
            for(int k = 0; k < 8; k++)
            {
                a[i][j][k].x = Force[i][j][k][0] / jello->mass;
                a[i][j][k].y = Force[i][j][k][1] / jello->mass;
                a[i][j][k].z = Force[i][j][k][2] / jello->mass;
                //cout<<"a:";
                //cout<<a[i][j][k].x<<", "<<a[i][j][k].y<<", "<<a[i][j][k].z<<endl;;
            }
        }
    }
}

double dotProduct(double a[3], double b[3])
{
    double sum = 0;
    for(int i = 0; i < 3; i++)
    {
        sum += a[i] * b[i];
    }
    return sum;
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
