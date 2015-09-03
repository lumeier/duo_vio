//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: any.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 03-Sep-2015 21:38:20
//

// Include Files
#include "rt_nonfinite.h"
#include "SLAM.h"
#include "any.h"
#include <ros/console.h>
#include <stdio.h>

// Function Definitions

//
// Arguments    : const boolean_T x[24]
// Return Type  : boolean_T
//
boolean_T any(const boolean_T x[24])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 24)) {
    if (!!x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

//
// Arguments    : const emxArray_boolean_T *x
//                boolean_T y[24]
// Return Type  : void
//
void b_any(const emxArray_boolean_T *x, boolean_T y[24])
{
  int iy;
  int i1;
  int i2;
  int j;
  int ix;
  boolean_T exitg1;
  boolean_T b0;
  for (iy = 0; iy < 24; iy++) {
    y[iy] = false;
  }

  iy = -1;
  i1 = 0;
  i2 = (x->size[1] - 1) * 24;
  for (j = 0; j < 24; j++) {
    i1++;
    i2++;
    iy++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && (ix <= i2)) {
      b0 = !x->data[ix - 1];
      if (!b0) {
        y[iy] = true;
        exitg1 = true;
      } else {
        ix += 24;
      }
    }
  }
}

//
// Arguments    : const boolean_T x[3]
// Return Type  : boolean_T
//
boolean_T c_any(const boolean_T x[3])
{
  boolean_T y;
  int k;
  boolean_T exitg1;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!!x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

//
// Arguments    : const emxArray_boolean_T *x
// Return Type  : boolean_T
//
boolean_T d_any(const emxArray_boolean_T *x)
{
  boolean_T y;
  int ix;
  boolean_T exitg1;
  boolean_T b1;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[1])) {
    b1 = !x->data[ix - 1];
    if (!b1) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

//
// File trailer for any.cpp
//
// [EOF]
//
