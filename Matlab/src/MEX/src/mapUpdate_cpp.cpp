#include <cmath>
#include "mex.h"
#include "matrix.h"
#include "math.h"
#include <Eigen>

struct Map{
    Eigen::MatrixXd N_Line;
    Eigen::MatrixXd E_Line;
    Eigen::MatrixXd logodds;

};

struct Paramaters{
    Eigen::MatrixXd theta;
    double res;
    double hitDepth;
    double hitChange;
    double passChange;
    double maxRange;

};

void mapUpdate(Eigen::MatrixXd & y, Eigen::MatrixXd & x, Map & map, Paramaters & param){
    // Declare Variables
    double maxDist = hypot(map.N_Line(map.N_Line.rows()-1)-map.N_Line(0),map.E_Line(map.E_Line.rows()-1)-map.E_Line(0));
    int measRes = floor(1 + maxDist/param.res);
    Eigen::VectorXd measInt(measRes);
    Eigen::Matrix<int,Eigen::Dynamic,1> isUseable;    
    Eigen::MatrixXd N_meas(param.theta.rows(),measRes), E_meas(param.theta.rows(),measRes);
    Eigen::MatrixXd N_idx(param.theta.cols(),measRes), E_idx(param.theta.cols(),measRes);
    
    // mexPrintf("N_l %f \n",map.N_Line(600,0));
    
    // mexPrintf("x %d     %d\n",x.rows(),x.cols());

    // Calculate matrix of points at which to change logodds
    measInt.setLinSpaced(measRes,0.0,maxDist);
    // mexPrintf("meas %f \n",param.hitChange);
    // mexPrintf("y %d \n",measRes);
    // mexPrintf("theta %d %d\n",param.theta.array().cos().matrix().rows(), param.theta.array().cos().matrix().cols());
    // mexPrintf("theta %d %d\n",measInt.transpose().rows(), measInt.transpose().cols());
    

    // N_meas << measInt * param.theta.array().cos().matrix();
    N_meas << (x(0) + (param.theta.array().cos().matrix() * measInt.transpose()).array()).matrix();

    E_meas << (x(1) + (param.theta.array().sin().matrix() * measInt.transpose()).array()).matrix();

    N_idx = ((map.N_Line.rows()-1)*(N_meas.array()-map.N_Line(0))/(map.N_Line(map.N_Line.rows()-1)-map.N_Line(0))).round().matrix();
    E_idx = ((map.E_Line.rows()-1)*(E_meas.array()-map.E_Line(0))/(map.E_Line(map.E_Line.rows()-1)-map.E_Line(0))).round().matrix();
    
    // Check which measurments are useable 
    // Not used right now
    for(int i=0; i<y.rows();i++){
        if(y(i) < param.maxRange){
            isUseable.resize(isUseable.size()+1);
            isUseable(i) = i;
            // mexPrintf("Finding Crash %d   %d\n", i, isUseable(i));
        }     
    }
    
    if(isUseable.rows()==0){
        mexErrMsgTxt("No Useable Measurments");
        return;
    }
    
        // mexPrintf("meas %d %d\n",measInt.rows(), measInt.cols());
        // mexPrintf("idx %f %f\n",N_meas(0,0), N_meas(0,6));
        // mexPrintf("log %f\n",map.logodds(N_idx(0,0),E_idx(0,0)));
    // mexErrMsgTxt("No Useable Measurments");
    // return;

    // Loop through useable measurments and measrument intervals
    for(int i=0;i<y.rows();i++){
        int index = i;
        // mexPrintf("Finding Crash %d   %d\n", N_meas.rows(),N_meas.cols());
        // mexPrintf("N_las %f \n",map.E_Line(2000,0));

        for(int k = 0; k<measInt.rows(); k++){
            
            if(N_meas(index,k) < map.N_Line(map.N_Line.rows()-1) & N_meas(index,k) > map.N_Line(0) & E_meas(index,k) < map.E_Line(map.E_Line.rows()-1) & E_meas(index,k) > map.E_Line(0)){
                
                //Assume pass if 0 measurment returned
                  //  ! Could result in close objects being considered not there however a max range measurment is far more likely
                if(y(index) == 0){
                    // mexErrMsgTxt("false miss");
                    // return;
                    // map.logodds(N_idx(index,k),E_idx(index,k)) -= param.passChange/5.0;
                    map.logodds(N_idx(index,k),E_idx(index,k)) -= param.passChange/5.0;
                    // mexPrintf("New Log Odds %f\n",map.logodds(N_idx(index,k),E_idx(index,k)));
                }
                // Decrease Logodds if ray passed through grid square
                else if(y(index) > measInt(k)){
                    // mexErrMsgTxt("true miss");
                    // return;
                    map.logodds(N_idx(index,k),E_idx(index,k)) -= param.passChange;
                }
                // mexErrMsgTxt("asdkguakjsdbglkabsd");
                // return;
                // Increase LogOdds if ray hit in gridsquare
                else if((y(index)+ param.hitDepth) > measInt(k)){
                    // mexErrMsgTxt("hit");
                    // return;
                    map.logodds(N_idx(index,k),E_idx(index,k)) += param.hitChange;
                    
                }
                
                // Should never hit this else
                else{
                    // mexErrMsgTxt("Somthing fucked broke!!");
                    // break;
                }
                    // mexPrintf("passed to inside mapUpdate %f\n", map.logodds(N_idx(index,k),E_idx(index,k)));

            }
            // Do nothing if outside the map
            else{
                // mexErrMsgTxt("Somthing fucked broke!!!!");
                // return;
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
    Eigen::MatrixXd y, x;  

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
        int length = mxGetN(prhs[0]);
        // y.resize(5,1);
        // mexPrintf("%d\n",length);
        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(prhs[0]),length,1);
        y = temp_map;
        // mexPrintf("%d   %d\n",y.rows(),y.cols());
    }
    else{
        mexErrMsgTxt("y input not detected");
        return;
    }

    if(mxGetPr(prhs[1]) != NULL){
        size_t length = mxGetM(prhs[1]);

        // x.resize(length);
        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(prhs[1]),length,1);
        x = temp_map;
    }
    else{
        mexErrMsgTxt("x input not detected");
        return;
    }    

    temp = mxGetField(prhs[2],0,"y");
    if(temp != NULL){
        // mexPrintf("reading Y");
        size_t length = mxGetN(temp);
        
        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(temp),length,1);
        // map.N_Line.resize(length);
        map.N_Line = temp_map;
        
    }
    else{
        mexErrMsgTxt("map.y not detected");
        return;
    }

    temp = mxGetField(prhs[2],0,"x");
    if(temp != NULL){
        size_t length = mxGetN(temp);
        
        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(temp),length,1);

        // map.E_Line.resize(length,1);
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
        // map.logodds.resize(rows,cols);
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

    temp = mxGetField(prhs[3],0,"passChange");
    if(temp != NULL){
        param.passChange = mxGetPr(temp)[0];
    }
    else{
        mexErrMsgTxt("param.passChange not detected");
        return;
    }

    temp = mxGetField(prhs[3],0,"theta");
    if(temp != NULL){
        size_t length = mxGetN(temp);
        
        Eigen::Map<Eigen::MatrixXd> temp_map(mxGetPr(temp),length,1);
        // param.theta.resize(length,1);

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