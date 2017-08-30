
#ifndef CANOPEN_MOTOR_NODE_UNIT_CONVERTER_H_
#define CANOPEN_MOTOR_NODE_UNIT_CONVERTER_H_

#include <string>
#include <list>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include "muParser.h"
#include <boost/math/special_functions/fpclassify.hpp> // for isnan

namespace canopen {
class UnitConverter{
public:
    typedef boost::function<double * (const std::string &) > get_var_func_type;

    UnitConverter(const std::string &expression, get_var_func_type var_func)
    : var_func_(var_func)
    {
        parser_.SetVarFactory(UnitConverter::createVariable, this);

        parser_.DefineConst("pi", M_PI);
        parser_.DefineConst("nan", std::numeric_limits<double>::quiet_NaN());

        parser_.DefineFun("rad2deg", UnitConverter::rad2deg);
        parser_.DefineFun("deg2rad", UnitConverter::deg2rad);
        parser_.DefineFun("norm", UnitConverter::norm);
        parser_.DefineFun("smooth", UnitConverter::smooth);
        parser_.DefineFun("avg", UnitConverter::avg);

        parser_.SetExpr(expression);
    }

    void reset(){
        for(variable_ptr_list::iterator it = var_list_.begin(); it != var_list_.end(); ++it){
            **it = std::numeric_limits<double>::quiet_NaN();
        }
    }
    double evaluate() { int num; return parser_.Eval(num)[0]; }
private:
    typedef boost::shared_ptr<double> variable_ptr;
    typedef std::list<variable_ptr> variable_ptr_list;

    static double* createVariable(const char *name, void * userdata) {
        UnitConverter * uc = static_cast<UnitConverter*>(userdata);
        double *p = uc->var_func_ ? uc->var_func_(name) : 0;
        if(!p){
            p = new double(std::numeric_limits<double>::quiet_NaN());
            uc->var_list_.push_back(variable_ptr(p));
        }
        return p;
    }
    variable_ptr_list var_list_;
    get_var_func_type var_func_;

    mu::Parser parser_;

    static double rad2deg(double r){
        return r*180.0/M_PI;
    }
    static double deg2rad(double d){
        return d*M_PI/180.0;
    }
    static double norm(double val, double min, double max){
        while(val >= max) val -= (max-min);
        while(val < min) val += (max-min);
        return val;
    }
    static double smooth(double val, double old_val, double alpha){
        if(boost::math::isnan(val)) return 0;
        if(boost::math::isnan(old_val)) return val;
        return alpha*val + (1.0-alpha)*old_val;
    }
    static double avg(const double *vals, int num)
    {
        double s = 0.0;
        int i=0;
        for (; i<num; ++i){
            const double &val = vals[i];
            if(boost::math::isnan(val)) break;
            s += val;
        }
        return s / double(i+1);
    }
};

}

#endif /* CANOPEN_MOTOR_NODE_UNIT_CONVERTER_H_ */
