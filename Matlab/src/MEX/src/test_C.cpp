#include <cmath>
#include "mex.h"
#include "math.h"
#include <Eigen>

void square(Eigen::MatrixXd & in, Eigen::MatrixXd & out){
    out.resize(2,2);
    out = in*in;
}
// void tripProd(double & a, double & b, double & c, double & out){
//     out = a*b*c;
// }

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    Eigen::MatrixXd mat(2,2),out;
    Eigen::Map<Eigen::MatrixXd> map(mxGetPr(mxGetField(prhs[0],0,"a")),2,2);
    mat = map;

    // mat = mat*2.0;
    
    // double *test1 = mxGetPr(prhs[0]);
    // double *test2 = mxGetPr(prhs[1]);
    // double *test3 = mxGetPr(prhs[2]);
    // double out;
    plhs[0] = mxCreateDoubleMatrix(2,2,mxREAL);
    Eigen::Map<Eigen::MatrixXd> mapOut(mxGetPr(plhs[0]),2,2);
    // mapOut = mat;
    // plhs[1] = mxCreateDoubleMatrix(1,1,mxREAL);
    // double *outptr2 = mxGetPr(plhs[1]);

    
    // tripProd(*test1,*test2,*test3,*outptr2);
    square(mat,out);
    mapOut = out;
    // mexPrintf("out = %f",*outptr);
 }