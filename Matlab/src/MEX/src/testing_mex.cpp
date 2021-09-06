#include <cmath>
#include "mex.hpp"
#include "mexAdapter.hpp"
#include "mex.h"
#include <Eigen>

using namespace matlab::data;
using matlab::mex::ArgumentList;
using matlab::engine::convertUTF8StringToUTF16String;
struct paramaters{
    double a;
    double b;
};

void multiply(double & a, double & b, double & out){
    out = a*b;
}

class MexFunction : public matlab::mex::Function {
    matlab::data::ArrayFactory f;
public:
    void operator()(ArgumentList outputs, ArgumentList inputs){
        // // Implement function
        // TypedArray<double> *p;
        TypedArray<double> *doubleArray = mxGetPr(inputs[0]);
        Eigen::Map<Eigen::MatrixXd> mapped(mxGetPr(inputs[0]),2,2);
        // paramaters params;
        // double out, in1, in2;
        // in1 = inputs[0][0];
        // in2 = inputs[1][0];
        
        // tmp = mxGetField(inputs[0], 0, "a");
        // int test = (int)(mxGetPr(tmp)[0]);

        // StructArray p(inputs[0]);
        // TypedArray<double> data = p[0]["a","b"];
        // // TypedArray<double> data[] = p[0]["b"];
        // matlab::data::Reference<Struct> param = p[0];

        // // double in = inputs[1][0];
        // // for (auto& elem : doubleArray) {
        // //     elem *= in;
        // // }
        // // TypedArray<double> doubleArray_2 = std::move(inputs[1]);
        // // for (auto& elem_2 : doubleArray_2) {
        // //     elem_2 *= 3;
        // // };
        // // Assign outputs
        // matlab::data::Reference<Array> val = p[0]["a"];
        // std::cout << val << std::endl;
        // params.a = data[0];
        // params.b = data[1];

        // multiply(params.a,params.b,out);
        // params.a = factory.createScalar(param["a"]);
        
        // params.a = p[0]["a"];
        // auto a = mxGetPr(inputs[1])
        TypedArray<double> test = f.createArray({1,4},{1.0,2.0,3.0,4.0});
        outputs[0] = test;
        // outputs[0] = p[0]["a"];
    }
};
