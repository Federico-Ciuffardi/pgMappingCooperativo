#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include "GVD/src/utils.h"
using namespace std;

// modified version of: https://github.com/jincheng9/AffinityPropagation 

//N is the number of two-dimension data points
//S is the similarity matrix
//R is the responsibility matrix
//A is the availabiltiy matrix
//iter is the maximum number of iterations
//lambda is the damping factor

inline vector<Pos> affinityPropagation(vector<Pos> &dataPos, Float radius) {
  // pre computations
  int N = dataPos.size();

  if(N <= radius){
    vector<Pos> res;
    res.push_back(dataPos[N/2]);
    return res;
  }

  vector<vector<Int>> dataPoint(N,vector<Int>(2));
  for (int i = 0; i < dataPos.size(); i++){
    Pos p = dataPos[i];
    dataPoint[i][0] = p.x;
    dataPoint[i][1] = p.y;
  }

  // affinityPropagation
  vector<vector<Float>> S(N,vector<Float>(N));
  vector<vector<Float>> R(N,vector<Float>(N));
  vector<vector<Float>> A(N,vector<Float>(N));
  int iter = 100;
  Float lambda = 0.9;

  Float size = N*(N-1)/2.0;
  Float sum = 0;
  vector<Float> tmpS;
  //compute similarity between data point i and j (i is not equal to j)
  for(int i=0; i<N-1; i++) {
    for(int j=i+1; j<N; j++) {
      S[i][j] = -((dataPoint[i][0]-dataPoint[j][0])*(dataPoint[i][0]-dataPoint[j][0])+(dataPoint[i][1]-dataPoint[j][1])*(dataPoint[i][1]-dataPoint[j][1]));
      S[j][i] = S[i][j];
      sum += S[j][i];
    }
  }

  Float mean = (sum/size);

  //compute preferences for all data points
  for(int i=0; i<N; i++) S[i][i] = -pow(-mean,0.5)*radius*2;


  for(int m=0; m<iter; m++) {
    //update responsibility
    for(int i=0; i<N; i++) {
      for(int k=0; k<N; k++) {
        Float max = -1e100;
        for(int kk=0; kk<k; kk++) {
          if(S[i][kk]+A[i][kk]>max) 
            max = S[i][kk]+A[i][kk];
        }
        for(int kk=k+1; kk<N; kk++) {
          if(S[i][kk]+A[i][kk]>max) 
            max = S[i][kk]+A[i][kk];
        }
        R[i][k] = (1-lambda)*(S[i][k] - max) + lambda*R[i][k];
      }
    }
    //update availability
    for(int i=0; i<N; i++) {
      for(int k=0; k<N; k++) {
        if(i==k) {
          Float sum = 0.0;
          for(int ii=0; ii<i; ii++) {
            sum += max(0.0f, R[ii][k]);
          }
          for(int ii=i+1; ii<N; ii++) {
            sum += max(0.0f, R[ii][k]);
          }
          A[i][k] = (1-lambda)*sum + lambda*A[i][k];
        } else {
          Float sum = 0.0;
          int maxik = max(i, k);
          int minik = min(i, k);
          for(int ii=0; ii<minik; ii++) {
            sum += max(0.0f, R[ii][k]);
          }
          for(int ii=minik+1; ii<maxik; ii++) {
            sum += max(0.0f, R[ii][k]);
          }
          for(int ii=maxik+1; ii<N; ii++) {
            sum += max(0.0f, R[ii][k]);
          }
          A[i][k] = (1-lambda)*min(0.0f, R[k][k]+sum) + lambda*A[i][k];
        }
      }
    }
  }

  //find the exemplar
  vector<vector<Float>> E(N,vector<Float>(N));
  vector<Pos> center;
  for(int i=0; i<N; i++) {
    E[i][i] = R[i][i] + A[i][i];
    if(E[i][i]>0) {
      center.push_back(Pos(dataPos[i]));
    }
  }
  return center;
}
