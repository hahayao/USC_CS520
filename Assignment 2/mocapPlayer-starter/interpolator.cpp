#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include "performanceCounter.h"

using namespace std;

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    
    if(N != 0)
    {
        PerformanceCounter timer;
        timer.StartCounter();
        int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

        int startKeyframe = 0;
        while (startKeyframe + N + 1 < inputLength)
        {
          int endKeyframe = startKeyframe + N + 1;

          Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
          Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

          // copy start and end keyframe
          pOutputMotion->SetPosture(startKeyframe, *startPosture);
          pOutputMotion->SetPosture(endKeyframe, *endPosture);

          // interpolate in between
          for(int frame=1; frame<=N; frame++)
          {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
              interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
          }

          startKeyframe = endKeyframe;
        }

        for(int frame=startKeyframe+1; frame<inputLength; frame++)
          pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
        
        timer.StopCounter();
        printf("Computation time of Linear Euler Interpolation Method: %lf\n", timer.GetElapsedTime());
    }
  //if N = -1, it will generate random N.
    else
    {
        int currentN = 0;
        int nextN = 0;
        
        PerformanceCounter timer;
        timer.StartCounter();
        int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

        int startKeyframe = 0;
        
        if(startKeyframe == 0)
        {
            currentN = rand()%10 + 5;
        }
        nextN = rand()%10 + 5;
        
        while (startKeyframe + currentN + nextN < inputLength - 5)
        {
          int endKeyframe = startKeyframe + currentN + 1;

          Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
          Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

          // copy start and end keyframe
          pOutputMotion->SetPosture(startKeyframe, *startPosture);
          pOutputMotion->SetPosture(endKeyframe, *endPosture);

          // interpolate in between
          for(int frame=1; frame<=currentN; frame++)
          {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (currentN+1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
              interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
          }
            printf("Random Interpolate Frame Num: %d\n", currentN);

            startKeyframe = endKeyframe;
            currentN = nextN;
            nextN = rand()%10 + 5;
        }

        for(int frame=startKeyframe+1; frame<inputLength; frame++)
          pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
        
        timer.StopCounter();
        printf("Computation time of Linear Euler Interpolation Method: %lf\n", timer.GetElapsedTime());
    }
}

void Interpolator::Rotation2Euler(double R[9], double (&angles)[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::MulMatrix3By3(double a[3][3], double b[3][3], double (&c)[3][3])
{
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            c[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
}

void Interpolator::Euler2Rotation(double angles[3], double (&R)[9])
{
  // students should implement this
    //apply angles[3] to rotation matrix
    for(int i = 0; i < 3; i++)
    {
        angles[i] = angles[i] / 180.0 * M_PI;
    }
    //matrix for theta3
    double mat3[3][3] = {{cos(angles[2]), -sin(angles[2]), 0},
                        {sin(angles[2]), cos(angles[2]), 0},
                        {0, 0, 1}};
    //matrix for theta2
    double mat2[3][3] = {{cos(angles[1]), 0, sin(angles[1])},
                        {0, 1, 0},
                        {-sin(angles[1]), 0, cos(angles[1])}};
    //matrix for theta1
    double mat1[3][3] = {{1, 0, 0},
                        {0, cos(angles[0]), -sin(angles[0])},
                        {0, sin(angles[0]), cos(angles[0])}};
    
    double matTemp[3][3];
    MulMatrix3By3(mat3, mat2, matTemp);
    double matResult[3][3];
    MulMatrix3By3(matTemp, mat1, matResult);
    
    //convert to 1D array
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            R[i*3 + j] = matResult[i][j];
        }
    }
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
    PerformanceCounter timer;
    timer.StartCounter();
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
      int endKeyframe = startKeyframe + N + 1;

      Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
      Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

      // copy start and end keyframe
      pOutputMotion->SetPosture(startKeyframe, *startPosture);
      pOutputMotion->SetPosture(endKeyframe, *endPosture);
        
        //variables for root position interpolation
        Posture * q1, * q2, * q3, * q4;
        vector q1RootP, q2RootP, q3RootP, q4RootP;
        vector root_a1, root_aba, root_b2;
        vector q1OnCurve, q2OnCurve;
        //variables for bones rotation interpolation
        vector q1RotationA, q2RotationA, q3RotationA, q4RotationA;
        vector bone_a1, bone_aba, bone_b2;
        vector EulerAng[MAX_BONES_IN_ASF_FILE][4];

        //the first time to interpolate, cannot get a1 in general formula
        if(startKeyframe == 0)
        {
            q1 = pInputMotion->GetPosture(startKeyframe);
            q2 = pInputMotion->GetPosture(startKeyframe + N + 1);
            q3 = pInputMotion->GetPosture(startKeyframe + 2 * N + 2);
            
            q1RootP = q1->root_pos;
            q2RootP = q2->root_pos;
            q3RootP = q3->root_pos;
            //q1, a1, b2, q2
            //a_n
            root_a1 = LinearInterpMethod(q1RootP, LinearInterpMethod(q3RootP, q2RootP, 2.0), 1.0/3);
            //b_n+1
            root_aba = LinearInterpMethod(LinearInterpMethod(q1RootP, q2RootP, 2.0), q3RootP, 0.5);
            root_b2 = LinearInterpMethod(q2RootP, root_aba, -1.0/3);
            //define two points on curve
            q1OnCurve = q1RootP;
            q2OnCurve = q2RootP;
            
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                q1RotationA = q1->bone_rotation[bone];
                q2RotationA = q2->bone_rotation[bone];
                q3RotationA = q3->bone_rotation[bone];
                
                //q1, a1, b2, q2
                //a_n
                bone_a1 = LinearInterpMethod(q1RotationA, LinearInterpMethod(q3RotationA, q2RotationA, 2.0), 1.0/3);
                //b_n+1
                bone_aba = LinearInterpMethod(LinearInterpMethod(q1RotationA, q2RotationA, 2.0), q3RotationA, 0.5);
                bone_b2 = LinearInterpMethod(q2RotationA, bone_aba, -1.0/3);
                //store value in 2D array
                EulerAng[bone][0] = q1RotationA;
                EulerAng[bone][1] = bone_a1;
                EulerAng[bone][2] = bone_b2;
                EulerAng[bone][3] = q2RotationA;
            }
            
        }
        //the ending frame, q_n-1 to q_n, cannot get b_n in general formula
        else if(endKeyframe + N + 1 > inputLength - 1)
        {
            q1 = pInputMotion->GetPosture(startKeyframe - N -1);
            q2 = pInputMotion->GetPosture(startKeyframe);
            q3 = pInputMotion->GetPosture(startKeyframe + N + 1);
            
            q1RootP = q1->root_pos;
            q2RootP = q2->root_pos;
            q3RootP = q3->root_pos;
            //q_n-1, a_n-1, b_n, q_n
            //a_n-1
            root_aba = LinearInterpMethod(LinearInterpMethod(q1RootP, q2RootP, 2.0), q3RootP, 0.5);
            root_a1 = LinearInterpMethod(q2RootP, root_aba, 1.0/3);
            root_b2 = LinearInterpMethod(q3RootP, LinearInterpMethod(q1RootP, q2RootP, 2.0), 1.0/3);
            //define two points on curve
            q1OnCurve = q2RootP;
            q2OnCurve = q3RootP;
            
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                q1RotationA = q1->bone_rotation[bone];
                q2RotationA = q2->bone_rotation[bone];
                q3RotationA = q3->bone_rotation[bone];
                
                //q_n-1, a_n-1, b_n, q_n
                //a_n-1
                bone_aba = LinearInterpMethod(LinearInterpMethod(q1RotationA, q2RotationA, 2.0), q3RotationA, 0.5);
                bone_a1 = LinearInterpMethod(q2RotationA, bone_aba, 1.0/3);
                bone_b2 = LinearInterpMethod(q3RotationA, LinearInterpMethod(q1RotationA, q2RotationA, 2.0), 1.0/3);
                //store value in 2D array
                EulerAng[bone][0] = q2RotationA;
                EulerAng[bone][1] = bone_a1;
                EulerAng[bone][2] = bone_b2;
                EulerAng[bone][3] = q3RotationA;
            }
        }
        //normal formula
        else
        {
            q1 = pInputMotion->GetPosture(startKeyframe - N -1);
            q2 = pInputMotion->GetPosture(startKeyframe);
            q3 = pInputMotion->GetPosture(startKeyframe + N + 1);
            q4 = pInputMotion->GetPosture(startKeyframe + 2 * N + 2);
           
            q1RootP = q1->root_pos;
            q2RootP = q2->root_pos;
            q3RootP = q3->root_pos;
            q4RootP = q4->root_pos;
            
            root_aba = LinearInterpMethod(LinearInterpMethod(q1RootP, q2RootP, 2.0), q3RootP, 0.5);
            root_a1 = LinearInterpMethod(q2RootP, root_aba, 1.0/3);
            
            root_aba = LinearInterpMethod(LinearInterpMethod(q2RootP, q3RootP, 2.0), q4RootP, 0.5);
            root_b2 = LinearInterpMethod(q3RootP, root_aba, -1.0/3);
            //define two points on curve
            q1OnCurve = q2RootP;
            q2OnCurve = q3RootP;
            
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                q1RotationA = q1->bone_rotation[bone];
                q2RotationA = q2->bone_rotation[bone];
                q3RotationA = q3->bone_rotation[bone];
                q4RotationA = q4->bone_rotation[bone];
                
                bone_aba = LinearInterpMethod(LinearInterpMethod(q1RotationA, q2RotationA, 2.0), q3RotationA, 0.5);
                bone_a1 = LinearInterpMethod(q2RotationA, bone_aba, 1.0/3);
                
                bone_aba = LinearInterpMethod(LinearInterpMethod(q2RotationA, q3RotationA, 2.0), q4RotationA, 0.5);
                bone_b2 = LinearInterpMethod(q3RotationA, bone_aba, -1.0/3);
                //store value in 2D array
                EulerAng[bone][0] = q2RotationA;
                EulerAng[bone][1] = bone_a1;
                EulerAng[bone][2] = bone_b2;
                EulerAng[bone][3] = q3RotationA;
            }
        }
        
        
      // interpolate in between
      for(int frame=1; frame<=N; frame++)
      {
          Posture interpolatedPosture;
          double t = 1.0 * frame / (N+1);

          //store root position
          interpolatedPosture.root_pos = DeCasteljauEuler(t, q1OnCurve, root_a1, root_b2, q2OnCurve);

          // interpolate bone rotations - d, Eu
          for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
          {
              interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, EulerAng[bone][0], EulerAng[bone][1], EulerAng[bone][2], EulerAng[bone][3]);
          }

          pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
      }
        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    timer.StopCounter();
    printf("Computation time of Bezier Euler Interpolation Method: %lf\n", timer.GetElapsedTime());
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
    if(N != 0)
    {
        PerformanceCounter timer;
        timer.StartCounter();
        //s1: transfer bone from Euler to Quaternion
        //s2: apply SLERP to interpolation
        int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

        int startKeyframe = 0;
        while (startKeyframe + N + 1 < inputLength)
        {
          int endKeyframe = startKeyframe + N + 1;

          Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
          Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

          // copy start and end keyframe
          pOutputMotion->SetPosture(startKeyframe, *startPosture);
          pOutputMotion->SetPosture(endKeyframe, *endPosture);
            
            // 0 for start, 1 for end
            Quaternion<double> startQuaternion[MAX_BONES_IN_ASF_FILE], endQuaternion[MAX_BONES_IN_ASF_FILE];
            
            //Euler->Quaternion
            double temp[3];
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                startPosture->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, startQuaternion[bone]);
                
                endPosture->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, endQuaternion[bone]);
            }
            
          // interpolate in between
          for(int frame=1; frame<=N; frame++)
          {
              Posture interpolatedPosture;
              double t = 1.0 * frame / (N+1);

              //store root position
              interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;
              
              // interpolate bone rotations - SlERP, Q
              for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
              {
                  double tempAngle[3];
                  Quaternion<double> tempQuat = Slerp(startQuaternion[bone], endQuaternion[bone], t);
                  Quaternion2Euler(tempQuat, tempAngle);
                  interpolatedPosture.bone_rotation[bone].setValue(tempAngle[0], tempAngle[1], tempAngle[2]);
                  
              }

              pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
          }
            startKeyframe = endKeyframe;
        }

        for(int frame=startKeyframe+1; frame<inputLength; frame++)
            pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
        timer.StopCounter();
        printf("Computation time of Linear Quaternion Interpolation Method: %lf\n", timer.GetElapsedTime());
    }
    else
    {
        int currentN = 0;
        int nextN = 0;
        
        PerformanceCounter timer;
        timer.StartCounter();
        int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

        int startKeyframe = 0;
        
        if(startKeyframe == 0)
        {
            currentN = rand()%10 + 5;
        }
        nextN = rand()%10 + 5;
        
        while (startKeyframe + currentN + nextN < inputLength - 5)
        {
            int endKeyframe = startKeyframe + currentN + 1;
            Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
            Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

            // copy start and end keyframe
            pOutputMotion->SetPosture(startKeyframe, *startPosture);
            pOutputMotion->SetPosture(endKeyframe, *endPosture);
            
            // 0 for start, 1 for end
            Quaternion<double> startQuaternion[MAX_BONES_IN_ASF_FILE], endQuaternion[MAX_BONES_IN_ASF_FILE];
            
            //Euler->Quaternion
            double temp[3];
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                startPosture->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, startQuaternion[bone]);
                
                endPosture->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, endQuaternion[bone]);
            }
            
          // interpolate in between
          for(int frame=1; frame<=currentN; frame++)
          {
              Posture interpolatedPosture;
              double t = 1.0 * frame / (currentN+1);

              //store root position
              interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;
              
              // interpolate bone rotations - SlERP, Q
              for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
              {
                  double tempAngle[3];
                  Quaternion<double> tempQuat = Slerp(startQuaternion[bone], endQuaternion[bone], t);
                  Quaternion2Euler(tempQuat, tempAngle);
                  interpolatedPosture.bone_rotation[bone].setValue(tempAngle[0], tempAngle[1], tempAngle[2]);
                  
              }

              pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
          }
            printf("Random Interpolate Frame Num: %d\n", currentN);

            startKeyframe = endKeyframe;
            currentN = nextN;
            nextN = rand()%10 + 5;
        }

        for(int frame=startKeyframe+1; frame<inputLength; frame++)
            pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
        timer.StopCounter();
        printf("Computation time of Linear Quaternion Interpolation Method: %lf\n", timer.GetElapsedTime());
        }
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
    PerformanceCounter timer;
    timer.StartCounter();
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
      int endKeyframe = startKeyframe + N + 1;

      Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
      Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

      // copy start and end keyframe
      pOutputMotion->SetPosture(startKeyframe, *startPosture);
      pOutputMotion->SetPosture(endKeyframe, *endPosture);
        
        //variables for root position interpolation
        Posture * q1, * q2, * q3, * q4;
        vector q1RootP, q2RootP, q3RootP, q4RootP;
        vector root_a1, root_aba, root_b2;
        vector q1OnCurve, q2OnCurve;
        //variables for bones rotation interpolation
        Quaternion<double> q1RotationA, q2RotationA, q3RotationA, q4RotationA;
        Quaternion<double> bone_a1, bone_aba, bone_b2;
        Quaternion<double> EulerAng[MAX_BONES_IN_ASF_FILE][4];
        double temp[3];
        Quaternion<double> temp_qua;

        //the first time to interpolate, cannot get a1 in general formula
        if(startKeyframe == 0)
        {
            q1 = pInputMotion->GetPosture(startKeyframe);
            q2 = pInputMotion->GetPosture(startKeyframe + N + 1);
            q3 = pInputMotion->GetPosture(startKeyframe + 2 * N + 2);
            
            q1RootP = q1->root_pos;
            q2RootP = q2->root_pos;
            q3RootP = q3->root_pos;
            //q1, a1, b2, q2
            //a_n
            root_a1 = LinearInterpMethod(q1RootP, LinearInterpMethod(q3RootP, q2RootP, 2.0), 1.0/3);
            //b_n+1
            root_aba = LinearInterpMethod(LinearInterpMethod(q1RootP, q2RootP, 2.0), q3RootP, 0.5);
            root_b2 = LinearInterpMethod(q2RootP, root_aba, -1.0/3);
            //define two points on curve
            q1OnCurve = q1RootP;
            q2OnCurve = q2RootP;
            
            //Euler->Quaternion
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                //q1
                q1->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, q1RotationA);
                //q2
                q2->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, q2RotationA);
                //q3
                q3->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, q3RotationA);
                
                //q1, a1, b2, q2
                //a_n
                temp_qua = Slerp(q3RotationA, q2RotationA, 2.0);
                bone_a1 = Slerp(q1RotationA, temp_qua, 1.0/3);
                //b_n+1
                temp_qua = Slerp(q1RotationA, q2RotationA, 2.0);
                bone_aba = Slerp(temp_qua, q3RotationA, 0.5);
                bone_b2 = Slerp(q2RotationA, bone_aba, -1.0/3);
                //store value in 2D array
                EulerAng[bone][0] = q1RotationA;
                EulerAng[bone][1] = bone_a1;
                EulerAng[bone][2] = bone_b2;
                EulerAng[bone][3] = q2RotationA;
            }
            
        }
        //the ending frame, q_n-1 to q_n, cannot get b_n in general formula
        else if(endKeyframe + N + 1 > inputLength - 1)
        {
            q1 = pInputMotion->GetPosture(startKeyframe - N -1);
            q2 = pInputMotion->GetPosture(startKeyframe);
            q3 = pInputMotion->GetPosture(startKeyframe + N + 1);
            
            q1RootP = q1->root_pos;
            q2RootP = q2->root_pos;
            q3RootP = q3->root_pos;
            //q_n-1, a_n-1, b_n, q_n
            //a_n-1
            root_aba = LinearInterpMethod(LinearInterpMethod(q1RootP, q2RootP, 2.0), q3RootP, 0.5);
            root_a1 = LinearInterpMethod(q2RootP, root_aba, 1.0/3);
            root_b2 = LinearInterpMethod(q3RootP, LinearInterpMethod(q1RootP, q2RootP, 2.0), 1.0/3);
            //define two points on curve
            q1OnCurve = q2RootP;
            q2OnCurve = q3RootP;
            
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                //q1
                q1->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, q1RotationA);
                //q2
                q2->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, q2RotationA);
                //q3
                q3->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, q3RotationA);
                
                //q_n-1, a_n-1, b_n, q_n
                //a_n-1
                temp_qua = Slerp(q1RotationA, q2RotationA, 2.0);
                bone_aba = Slerp(temp_qua, q3RotationA, 0.5);
                bone_a1 = Slerp(q2RotationA, bone_aba, 1.0/3);
                temp_qua = Slerp(q1RotationA, q2RotationA, 2.0);
                bone_b2 = Slerp(q3RotationA, temp_qua, 1.0/3);
                //store value in 2D array
                EulerAng[bone][0] = q2RotationA;
                EulerAng[bone][1] = bone_a1;
                EulerAng[bone][2] = bone_b2;
                EulerAng[bone][3] = q3RotationA;
            }
        }
        //normal formula
        else
        {
            q1 = pInputMotion->GetPosture(startKeyframe - N -1);
            q2 = pInputMotion->GetPosture(startKeyframe);
            q3 = pInputMotion->GetPosture(startKeyframe + N + 1);
            q4 = pInputMotion->GetPosture(startKeyframe + 2 * N + 2);
           
            q1RootP = q1->root_pos;
            q2RootP = q2->root_pos;
            q3RootP = q3->root_pos;
            q4RootP = q4->root_pos;
            
            root_aba = LinearInterpMethod(LinearInterpMethod(q1RootP, q2RootP, 2.0), q3RootP, 0.5);
            root_a1 = LinearInterpMethod(q2RootP, root_aba, 1.0/3);
            
            root_aba = LinearInterpMethod(LinearInterpMethod(q2RootP, q3RootP, 2.0), q4RootP, 0.5);
            root_b2 = LinearInterpMethod(q3RootP, root_aba, -1.0/3);
            //define two points on curve
            q1OnCurve = q2RootP;
            q2OnCurve = q3RootP;
            
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                //q1
                q1->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, q1RotationA);
                //q2
                q2->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, q2RotationA);
                //q3
                q3->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, q3RotationA);
                //q4
                q4->bone_rotation[bone].getValue(temp);
                Euler2Quaternion(temp, q4RotationA);
                
                temp_qua = Slerp(q1RotationA, q2RotationA, 2.0);
                bone_aba = Slerp(temp_qua, q3RotationA, 0.5);
                bone_a1 = Slerp(q2RotationA, bone_aba, 1.0/3);
                
                temp_qua = Slerp(q2RotationA, q3RotationA, 2.0);
                bone_aba = Slerp(temp_qua, q4RotationA, 0.5);
                bone_b2 = Slerp(q3RotationA, bone_aba, -1.0/3);
                //store value in 2D array
                EulerAng[bone][0] = q2RotationA;
                EulerAng[bone][1] = bone_a1;
                EulerAng[bone][2] = bone_b2;
                EulerAng[bone][3] = q3RotationA;
            }
        }
        
        
      // interpolate in between
      for(int frame=1; frame<=N; frame++)
      {
          Posture interpolatedPosture;
          double t = 1.0 * frame / (N+1);

          //store root position
          interpolatedPosture.root_pos = DeCasteljauEuler(t, q1OnCurve, root_a1, root_b2, q2OnCurve);

          // interpolate bone rotations - d, Eu
          for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
          {
              double tempAngle[3];
              Quaternion<double> tempQuat = DeCasteljauQuaternion(t, EulerAng[bone][0], EulerAng[bone][1], EulerAng[bone][2], EulerAng[bone][3]);
              Quaternion2Euler(tempQuat, tempAngle);
              interpolatedPosture.bone_rotation[bone].setValue(tempAngle[0], tempAngle[1], tempAngle[2]);
          }

          pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
      }
        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    timer.StopCounter();
    printf("Computation time of Bezier Quaternion Interpolation Method: %lf\n", timer.GetElapsedTime());
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q)
{
//    for(int i = 0; i < 3; i++)
//    {
//        angles[i] = angles[i] / 180.0 * M_PI;
//        angles[i] /= 2.0;
//    }
//  // students should implement this
//    double w = cos(angles[0]) * cos(angles[1]) * cos(angles[2]) + sin(angles[0]) * sin(angles[1]) * sin(angles[2]);
//    double x = sin(angles[0]) * cos(angles[1]) * cos(angles[2]) - cos(angles[0]) * sin(angles[1]) * sin(angles[2]);
//    double y = cos(angles[0]) * sin(angles[1]) * cos(angles[2]) + sin(angles[0]) * cos(angles[1]) * sin(angles[2]);
//    double z = cos(angles[0]) * cos(angles[1]) * sin(angles[2]) - sin(angles[0]) * sin(angles[1]) * cos(angles[2]);
//    q.Set(w, x, y, z);
    //Euler to rotation matrix
    double rotationMatrix[9];
    Euler2Rotation(angles, rotationMatrix);
    //rotation matrix to quaternion(double)
    q = Quaternion<double>::Matrix2Quaternion(rotationMatrix);
    q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double (&angles)[3])
{
  // students should implement this
    //convert Quaternion to rotation matrix
//    double w = q.Gets();
//    double x = q.Getx();
//    double y = q.Gety();
//    double z = q.Getz();
//    double rotationMatrix[9] = {1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y,
//                                2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x,
//                                2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y
//    };
    double rotationMatrix[9];
    q.Quaternion2Matrix(rotationMatrix);
    
    //convert rotation matrix to euler
    Rotation2Euler(rotationMatrix, angles);
}

Quaternion<double> Interpolator::Slerp(Quaternion<double> & qStart, Quaternion<double> & qEnd_, double t)
{
  // students should implement this
    if(qStart == qEnd_)
    {
        return qEnd_;
    }
    
    double dotProduct = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();
    //check if find the nearest one
    if(dotProduct < 0)
    {
        dotProduct = - dotProduct;
        qEnd_.Set(-qEnd_.Gets(), -qEnd_.Getx(), -qEnd_.Gety(), -qEnd_.Getz());
    }
    double theta = acos(dotProduct);
    
    //avoid overflow, divider cannot be 0
    if(theta == 0)
    {
        return qEnd_;
    }
    double coeffecient0 = sin((1 - t) * theta) / sin(theta);
    double coeffecient1 = sin(t * theta) / sin(theta);
    Quaternion<double> result;
    result = coeffecient0 * qStart + coeffecient1 * qEnd_;
    
    return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // Using BezierInterMethod to replace this method
  Quaternion<double> result;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
    //first
    vector q0 = LinearInterpMethod(p0, p1, t);
    vector q1 = LinearInterpMethod(p1, p2, t);
    vector q2 = LinearInterpMethod(p2, p3, t);
    //second
    vector r0 = LinearInterpMethod(q0, q1, t);
    vector r1 = LinearInterpMethod(q1, q2, t);
    //third
    vector result = LinearInterpMethod(r0, r1, t);
    
    return result;
}

inline vector Interpolator::LinearInterpMethod(vector startPoint, vector endPoint, double t)
{
    vector r = startPoint * (1-t) + endPoint * t;
    return r;
}

inline Quaternion<double> Interpolator::BezierInterMethod(Quaternion<double> startPoint, Quaternion<double> endPoint, double t)
{
    Quaternion<double> r = startPoint * (1 - t) + endPoint * t;
    return r;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
    //first
    Quaternion<double> q0 = Slerp(p0, p1, t);
    Quaternion<double> q1 = Slerp(p1, p2, t);
    Quaternion<double> q2 = Slerp(p2, p3, t);
    //second
    Quaternion<double> r0 = Slerp(q0, q1, t);
    Quaternion<double> r1 = Slerp(q1, q2, t);
    //third
    Quaternion<double> result = Slerp(r0, r1, t);
    
    return result;
}
