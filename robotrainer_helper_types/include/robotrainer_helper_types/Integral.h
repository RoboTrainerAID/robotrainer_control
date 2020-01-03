#ifndef _ROBOTRAINER_HELPER_TYPES_INTEGRAL_H_
#define _ROBOTRAINER_HELPER_TYPES_INTEGRAL_H_

#define DIM 3
#define WINDOWSIZE_DEFAULT 128
#define USE_RESET_DEFAULT false

#include "robotrainer_helper_types/wrench_twist_datatype.h"

#include <ros/ros.h>

#include <array>
#include <list>

namespace robotrainer_helper_types{

class IntegralBase {
public:
        virtual double calculateIntegral(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) = 0;
        //virtual double calculateIntegral(wrench_twist_ input) = 0;
    
        double getIntegralValue() {return integralValue_;}
        void resetIntegralValue() {integralValue_ = 0.0;}
    
protected:
        ros::Time last_time_ = ros::Time::now(); 
        double integralValue_;
};
   
class IntegralMultiDimBase  {
public :
        virtual std::array<double, DIM> calculateIntegralMultiDim(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) = 0;
        
        std::array<double, DIM> getIntegralValues(){return integralValues_;}

        void resetIntegralValues() {
                for(auto i = 0; i < DIM; ++i){
                        integralValues_[i] = 0.0;
                }
        }
  
protected:
        ros::Time last_time_;
        std::array<double, DIM> integralValues_;
};
    
  
/**
 * Derived classes of IntegralBase
 * 
 * 
 */
class RAWIntegral : public IntegralBase {
public:
        RAWIntegral() {
                resetIntegralValue();
        }

        double calculateIntegral(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) {
                for(auto i = 0; i < DIM; ++i) {
                        integralValue_ =  x[i]*y[i];
                }             
                return integralValue_;
        }
};

class STDIntegral : public IntegralBase {
public:
        STDIntegral(){
                resetIntegralValue();
        }

        ~STDIntegral(){}
    
        double calculateIntegral(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) {
                double time_delta = (time - last_time_).toSec();
                last_time_ = time;
                for(auto i = 0; i < DIM; ++i) {
                        integralValue_ +=  x[i]*y[i]*time_delta;
                }             
                return integralValue_;
        }
};

class ChuyIntegral : public IntegralBase {
public:
        ChuyIntegral(){
                resetIntegralValue();
        }

        ~ChuyIntegral(){}
        
        double calculateIntegral(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) {   
                double power;
                double time_delta = (time - last_time_).toSec();
                last_time_ = time;
                //ROS_INFO("time delta = [%.2f]",time_delta); 
                for(auto i = 0; i < DIM; ++i) {
                        power = x[i]*y[i];
                        if(integralValue_ < 0.0 && power > 0.0){
                                resetIntegralValue();
                        }
                        integralValue_ += power*time_delta;
                }             
                return integralValue_;
        }
    
};

class SlidingIntegral : public IntegralBase {
public:
        SlidingIntegral() : windowSize_(WINDOWSIZE_DEFAULT), use_reset_(USE_RESET_DEFAULT){
                resetIntegralValue();
                resetWindow();
        }
        SlidingIntegral(int windowSize) : windowSize_(windowSize), use_reset_(USE_RESET_DEFAULT){
                resetIntegralValue();
                resetWindow();
        }
                        SlidingIntegral(int windowSize, bool use_reset) : windowSize_(windowSize), use_reset_(use_reset){
                resetIntegralValue();
                resetWindow();
        }
    
        SlidingIntegral(bool use_reset) : windowSize_(WINDOWSIZE_DEFAULT), use_reset_(use_reset){
                resetIntegralValue();
                resetWindow();
        }
    
        ~SlidingIntegral(){}
    
        const int getWindowSize(){return windowSize_;}
    
        void resetWindow() {
                integralWindow_.assign(windowSize_, 0.0);
        }
    
        virtual double calculateIntegral(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) {
                double time_delta = (time - last_time_).toSec();
                last_time_ = time;
                double power = 0.0;
                for(auto i = 0; i < DIM; ++i){
                        power +=  x[i]*y[i];
                }
                                        
                double current_value = power*time_delta;
                                        
                if(use_reset_ &&  integralValue_ < 0.0 && power > 0.0){        
                        resetIntegralValue();
                        resetWindow();					
                        integralValue_ += current_value;
                        integralWindow_.push_back(current_value);
                        
                        double firstVal = integralWindow_.front();
                        integralWindow_.pop_front();
                        integralValue_ -= firstVal;
                } else {
                        integralValue_ += current_value;
                        integralWindow_.push_back(current_value);
                                
                        double firstVal = integralWindow_.front();
                        integralWindow_.pop_front();
                        integralValue_ -= firstVal;
                }
                return integralValue_;
        }
    
        virtual double calculateIntegral(const wrench_twist& input, const ros::Time& time) {
                double time_delta = ( time - last_time_).toSec();
                last_time_ = time;
                double power = input.wrench_.force.x*input.twist_.linear.x + input.wrench_.force.y*input.twist_.linear.y +input.wrench_.torque.z*input.twist_.angular.z;
                double current_value = power*time_delta; 
                                        
                if(use_reset_ && ( integralValue_ < 0.0 && power > 0.0)){        
                        resetIntegralValue();
                        resetWindow();
                        integralValue_ += current_value;
                        integralWindow_.push_back(current_value);
                        
                        double firstVal = integralWindow_.front();
                        integralWindow_.pop_front();
                        integralValue_ -= firstVal;
                } else {
                        integralValue_ += current_value;
                        integralWindow_.push_back(current_value);
                                
                        double firstVal = integralWindow_.front();
                        integralWindow_.pop_front();
                        integralValue_ -= firstVal;
                }
                return integralValue_;
        }
    
        void resetIntegral() {
                resetWindow();
                resetIntegralValue();
        }
    
protected:
        // if used, the integral Value and Window is reset if (integralValue_ is < 0 && powerofSystem > 0)
        bool use_reset_;
        const int windowSize_;
        std::list<double> integralWindow_;
        
};

class SlidingIntegralLineraWeigth : public SlidingIntegral {
public:
        SlidingIntegralLineraWeigth(){};
        SlidingIntegralLineraWeigth(int windowSize) : SlidingIntegral(windowSize){};
        SlidingIntegralLineraWeigth(int windowSize, bool use_reset) : SlidingIntegral(windowSize, use_reset){};
    
        double calculateIntegral(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) {
                SlidingIntegral::calculateIntegral(x,y, time);
                resetIntegralValue();
                                                
                int counter = 0;
                std::list<double>::iterator it= integralWindow_.begin();
                for (it; it != integralWindow_.end(); ++it){
                        if(counter == 0){
                        ++counter;
                        continue;
                        }
                        integralValue_ += ((double)counter/(double)(windowSize_-1))* (*it);
                        ++counter;
                }
                return integralValue_;
        }
};
/**
 * 
 *  Derived classes of IntegralMultiDimBase
 * 
 */

class RAWIntegralMultiDim : public IntegralMultiDimBase {
public:
        RAWIntegralMultiDim() {
                resetIntegralValues();
        }
        
        std::array<double, DIM> calculateIntegralMultiDim(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) {
                for(auto i = 0; i < DIM; ++i){
                        integralValues_[i] = x[i]*y[i];
                }
                return integralValues_;
        }
};

class STDIntegralMultiDim : public IntegralMultiDimBase {
public:
        STDIntegralMultiDim() {
                resetIntegralValues();
        }
        
        std::array<double, DIM> calculateIntegralMultiDim(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) {
                double time_delta = (time - last_time_).toSec();
                        last_time_ = time;
                for(auto i = 0; i < DIM; ++i){
                        integralValues_[i] += x[i]*y[i]*time_delta;
                }
                return integralValues_;
        }
};

class ChuyIntegralMultiDim : public IntegralMultiDimBase {
public:
        ChuyIntegralMultiDim() {
            resetIntegralValues();
        }
        
        std::array<double, DIM> calculateIntegralMultiDim(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) {
                double time_delta = (time - last_time_).toSec();
                last_time_ = time;
                double power;
                for(auto i = 0; i < DIM; ++i){
                        power = x[i] * y[i];
                        if(integralValues_[i] < 0.0 && power > 0.0){
                                //reset integralValue for dimension
                                integralValues_[i] = 0.0;
                        }
                        integralValues_[i] += power*time_delta;
                }
                return integralValues_;
        }

};

class SlidingIntegralMultiDim : public IntegralMultiDimBase {
public:
        SlidingIntegralMultiDim() : windowSize_(WINDOWSIZE_DEFAULT), use_reset_(USE_RESET_DEFAULT) {
                resetIntegral();
        }
        SlidingIntegralMultiDim(int windowSize) : windowSize_(windowSize), use_reset_(USE_RESET_DEFAULT) {
                resetIntegral();
        }
        SlidingIntegralMultiDim(int windowSize, bool use_reset) : windowSize_(windowSize), use_reset_(use_reset){
                resetIntegral();
        }
        
        ~SlidingIntegralMultiDim(){}
        
        
        const int getWindowSize(){return windowSize_;}
        
        void resetIntegralValue(const int index_of_reset){ integralValues_[index_of_reset] = 0.0;}
        
        void resetWindowMultiDim() {
                for(auto i = 0; i < DIM; ++i){
                        resetWindowMultiDim(i);
                }
        }
        
        void resetWindowMultiDim(const int index_of_reset) {
                integralWindow3d_[index_of_reset].clear();
                integralWindow3d_[index_of_reset].assign(windowSize_, 0.0);
        }
        
        void resetIntegral() {
                resetWindowMultiDim();
                resetIntegralValues();
        }

        void resetIntegral(const int index) {
                resetWindowMultiDim(index);
                resetIntegralValue(index);
        }

        virtual std::array<double, DIM> calculateIntegralMultiDim(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time) {
                double time_delta = (time - last_time_).toSec();
                last_time_ = time;
                double power;
                for(auto i = 0; i < DIM; ++i){
                        //add value of dim i
                        power = x[i]*y[i];
                        double current_value = power * time_delta;
                        if(use_reset_ && (integralValues_[i] < 0.0 && power >= 0.0)){
                                resetIntegral(i);
                                integralValues_[i] += current_value;
                                integralWindow3d_[i].push_back(current_value);
                                //move window
                                double firstVal = integralWindow3d_[i].front();
                                integralWindow3d_[i].pop_front();
                                integralValues_[i] -= firstVal;
                        } else {
                                integralValues_[i] += current_value;
                                integralWindow3d_[i].push_back(current_value);
                                //move window
                                double firstVal = integralWindow3d_[i].front();
                                integralWindow3d_[i].pop_front();
                                integralValues_[i] -= firstVal;
                        }
                        if (std::fabs(integralValues_[i]) < 0.00000001) {
                                integralValues_[i] = 0.0;
                        }
                }
                return integralValues_;
        }
    
    
protected:
        // if used, the integral Value and Window is reset if (integralValue_ is < 0 && powerofSystem > 0)
        bool use_reset_;
        const int windowSize_;
        std::array<std::list<double>, DIM> integralWindow3d_;
};


class SlidingIntegralLineraWeigthMultiDim : public SlidingIntegralMultiDim {
public:
        SlidingIntegralLineraWeigthMultiDim(){};
        SlidingIntegralLineraWeigthMultiDim(int windowSize) : SlidingIntegralMultiDim(windowSize){};
        SlidingIntegralLineraWeigthMultiDim(int windowSize, bool use_reset) : SlidingIntegralMultiDim(windowSize,use_reset){};
        
        std::array<double, DIM> calculateIntegralMultiDim(std::array<double,DIM> x, std::array<double,DIM> y, const ros::Time& time)
        {
                SlidingIntegralMultiDim::calculateIntegralMultiDim(x,y, time);
                resetIntegralValues();
                
                for(auto i = 0; i <DIM;  ++i) {
                        int counter = 0;
                        std::list<double>::iterator it= integralWindow3d_[i].begin();
                        for (it; it != integralWindow3d_[i].end(); ++it) {
                                if(counter == 0){
                                        ++counter;
                                        continue;
                                }
                                integralValues_[i] += ((double)counter/(double)(windowSize_-1))* (*it);
                                ++counter;
                        }
                }
                return integralValues_;
        }
        
};

    
}//namespace

#endif
