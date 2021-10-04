#include <cmath>
#include "mex.h"
#include "math.h"
#include <Eigen>

void square(Eigen::MatrixXd & in1, int & i, Eigen::MatrixXd & out){
    // out.resize(4,6);
    // out = (in1.array() < 3.0).select(1,out);
    // out = (in1);
    // out << in1*in2;
    out.col(i) = in1;
}
// void tripProd(double & a, double & b, double & c, double & out){
//     out = a*b*c;
// }

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    Eigen::MatrixXd mat1, mat2,out;
    mexPrintf("un-init rows = %f",mat1.cols());
    Eigen::Map<Eigen::MatrixXd> map(mxGetPr(prhs[0]),4,1);
    mat1 = map;

    // Eigen::Map<Eigen::MatrixXd> map2(mxGetPr(prhs[1]),1,6);
    // mat2 = map2;

    // mat = mat*2.0;
    
    // double *test1 = mxGetPr(prhs[0]);
    // double *test2 = mxGetPr(prhs[1]);
    // double *test3 = mxGetPr(prhs[2]);
    // double out;
    plhs[0] = mxCreateDoubleMatrix(4,5,mxREAL);
    Eigen::Map<Eigen::MatrixXd> mapOut(mxGetPr(plhs[0]),4,5);
    // mapOut = mat;
    // plhs[1] = mxCreateDoubleMatrix(1,1,mxREAL);
    // double *outptr2 = mxGetPr(plhs[1]);

    
    // tripProd(*test1,*test2,*test3,*outptr2);
    // Eigen::MatrixXd out;
    out.resize(4,5);
    out.setZero();
    
    for(int i=0;i<5;i++){
        square(mat1,i,out);
    }
    
    mapOut = out;
    // mexPrintf("out = %f",*outptr);
 }