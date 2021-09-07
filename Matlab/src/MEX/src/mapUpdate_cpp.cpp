#include <cmath>
#include "mex.h"
#include "matrix.h"
#include "math.h"
#include <Eigen>

struct Map{
    Eigen::VectorXd N_Line;
    Eigen::VectorXd E_Line;
    Eigen::MatrixXd logodds;

};

struct Paramaters{
    Eigen::VectorXd theta;
    double res;
    double hitDepth;
    double hitChange;
    double passChange;
    double maxRange;

};

void mapUpdate(Eigen::VectorXd & y, Eigen::VectorXd & x, Map & map, Paramaters & param){
    // Declare Variables
    double maxDist = hypot(map.N_Line(map.N_Line.rows()-1)-map.N_Line(0),map.N_Line(map.N_Line.rows()-1)-map.N_Line(0));
    int measRes = floor(1 + maxDist/param.res);
    Eigen::VectorXd measInt(param.theta.rows());
    Eigen::Matrix<int,Eigen::Dynamic,1> isUseable;    
    Eigen::MatrixXd N_meas(param.theta.rows(),measRes), E_meas(param.theta.rows(),measRes),N_idx(param.theta.rows(),measRes), E_idx(param.theta.rows(),measRes);


    // Calculate matrix of points at which to change logodds
    measInt.setLinSpaced(measRes,0.0,maxDist);

    N_meas << (x(0) + (measInt * param.theta.array().cos().matrix()).array()).matrix();
    E_meas << (x(1) + (measInt * param.theta.array().sin().matrix()).array()).matrix();

    N_idx = (1+(map.N_Line.rows()-1)*(N_meas.array()-map.N_Line(0))/(map.N_Line(map.N_Line.rows()-1)-map.N_Line(0))).matrix();
    E_idx = (1+(map.E_Line.rows()-1)*(E_meas.array()-map.E_Line(0))/(map.E_Line(map.E_Line.rows()-1)-map.E_Line(0))).matrix();
    
    // Check which measurments are useable 
    // Not used right now
    for(int i=0; i<10;i++){
        if(y(i) < param.maxRange){
            isUseable.resize(isUseable.size()+1);
            isUseable(i) = i;
            mexPrintf("Finding Crash %f ", y.size());
        }     
    }
    if(isUseable.rows()==0){
        mexPrintf("No Useable Measurments");
        // return;
    }

    // Loop through useable measurments and measrument intervals
    for(int i=0;i<isUseable.rows();i++){
        int index = isUseable(i);
        for(int k = 0; k<measInt.rows(); k++){
            
            if(N_meas(index,k) < map.N_Line(map.N_Line.rows()-1) & N_meas(index,k) > map.N_Line(0) & E_meas(index,k) < map.E_Line(map.E_Line.rows()-1) & E_meas(index,k) > map.E_Line(0)){
                //Assume pass if 0 measurment returned
                  //  ! Could result in close objects being considered not there however a max range measurment is far more likely
                if(y(index == 0)){
                    map.logodds(N_idx(index,k),E_idx(index,k)) -= param.passChange/5;
                    
                }
                // Decrease Logodds if ray passed through grid square
                else if(y(index > measInt(index))){
                   map.logodds(N_idx(index,k),E_idx(index,k)) -= param.passChange;
                }

                // Increase LogOdds if ray hit in gridsquare
                else if(y(index < measInt(index))){
                   map.logodds(N_idx(index,k),E_idx(index,k)) += param.passChange;
                }
                
                // Should never hit this else
                else{
                    mexErrMsgTxt("Somthing fucked broke!!");
                    // break;
                }
            }
            // Do nothing if outside the map
            else{
                // break;
            }
        }
    }
    
    // Saturate logodds to ensure their magnitude does not get too large
    // map.logodds = (map.logodds.array() > 50).select(50, map.logodds);
    // map.logodds = (map.logodds.array() > -50).select(-50, map.logodds);

}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    // Declare Variables
    Map map;
    Paramaters param;
    Eigen::VectorXd y, x;  

    mxArray *temp;

    // Check correct number of outputs are given
    if(nlhs < 1){
        mexErrMsgTxt("Need 1 output argumments, to few were given");
        return;
    }
    else if(nlhs > 1){
        mexErrMsgTxt("Need 1 output argumments, to many were given");
        return;
    }
    
    // Check correct number of inputs were given
    if(nrhs < 4){
        mexErrMsgTxt("Need 2 output argumments, to few were given");
        return;
    }
    else if(nrhs > 4){
        mexErrMsgTxt("Need 2 output argumments, to many were given");
        return;
    }

    // Extract variables from inputs and assign to structures and eigen matricies as needed
    if(mxGetPr(prhs[0]) != NULL){
        size_t length = mxGetM(prhs[0]);
        y.resize(length);
        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(prhs[0]),length,1);
        y = temp_map;
    }
    else{
        mexErrMsgTxt("y input not detected");
        return;
    }

    if(mxGetPr(prhs[1]) != NULL){
        size_t length = mxGetM(prhs[1]);

        x.resize(length);
        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(prhs[1]),length,1);
        x = temp_map;
    }
    else{
        mexErrMsgTxt("x input not detected");
        return;
    }    

    temp = mxGetField(prhs[2],0,"y");
    if(temp != NULL){
        size_t length = mxGetM(temp);

        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(temp),length,1);
        map.N_Line.resize(length);
        map.N_Line = temp_map;
    }
    else{
        mexErrMsgTxt("map.y not detected");
        return;
    }

    temp = mxGetField(prhs[2],0,"x");
    if(temp != NULL){
        size_t length = mxGetM(temp);
        
        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(temp),length,1);

        map.E_Line.resize(length,1);
        map.E_Line = temp_map;
    }
    else{
        mexErrMsgTxt("map.x not detected");
        return;
    }

    temp = mxGetField(prhs[2],0,"logodds");
    if(temp != NULL){
        size_t rows = mxGetM(temp);
        size_t cols = mxGetN(temp);
        
        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(temp),rows,cols);
        map.logodds.resize(rows,cols);
        map.logodds = temp_map;
    }
    else{
        mexErrMsgTxt("map.logodds not detected");
        return;
    }

    temp = mxGetField(prhs[3],0,"maxRange");
    if(temp != NULL){
        param.maxRange = mxGetPr(temp)[0];
    }
    else{
        mexErrMsgTxt("param.maxRange not detected");
        return;
    }

    temp = mxGetField(prhs[3],0,"res");
    if(temp != NULL){
        param.res = mxGetPr(temp)[0];
    }
    else{
        mexErrMsgTxt("param.res not detected");
        return;
    }

    temp = mxGetField(prhs[3],0,"hitDepth");
    if(temp != NULL){
        param.hitDepth = mxGetPr(temp)[0];
    }
    else{
        mexErrMsgTxt("param.hitDepth not detected");
        return;
    }

    temp = mxGetField(prhs[3],0,"hitChange");
    if(temp != NULL){
        param.hitChange = mxGetPr(temp)[0];
    }
    else{
        mexErrMsgTxt("param.hitChange not detected");
        return;
    }

    temp = mxGetField(prhs[3],0,"theta");
    if(temp != NULL){
        size_t length = mxGetM(temp);
        
        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(temp),length,1);
        param.theta.resize(length,1);

        param.theta = temp_map;
    }
    else{
        mexErrMsgTxt("param.theta not detected");
        return;
    }

    // Call update function
    mapUpdate(y,x,map,param);

    // Map Eigen to matlab mex array and output
    plhs[0] = mxCreateDoubleMatrix(map.logodds.rows(),map.logodds.cols(),mxREAL);
    Eigen::Map<Eigen::MatrixXd> tempOut(mxGetPr(plhs[0]),map.logodds.rows(),map.logodds.cols());
    tempOut = map.logodds;
}