/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

void computeAcceleration(struct world * jello, struct point a[8][8][8]);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);
void CreateStructuralSpring();
void CreateBendSpring();
void CreateDiagonalSpring();
double dotProduct(double a[3], double b[3]);
void computeAcceleration(struct world * jello, struct point a[8][8][8]);
void CalPointPlane(double t[3], double co[4], double &length, double vec[3]);
double CalRestLength(double left[3], double right[3]);
void CreateBoundaryEquation(struct world * jello);
#endif

