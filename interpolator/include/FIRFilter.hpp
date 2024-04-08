/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Filter.h
 * Author: Davide Chiaravalli, Federico Califano
 *
 * Created on Sep 8, 2016, 2:05 PM
 */

#ifndef FIRFILTER_H
#define FIRFILTER_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <math.h>
#include <iostream>

using Vector6d = Eigen::Matrix<double,6,1>;


namespace fir_filter
{

    class FIRFilter 
    {
        public:

            FIRFilter(double period=1,double Ts=0.001);			//Class Constructor #1
            virtual ~FIRFilter();			//Class Destructor


            void initialiseFilter(Vector6d initial_value);
            void setFilterParameters(double period, double Ts);
            void addNewInput(Vector6d input);
            Vector6d evaluateFilterOutput();
            void resetInputPosition();
            int getFilterCellNumber();
        
        
        // private:

            double period;							//stores the time period of the spline
            double sampling_time;						//stores the sampling time of the filters used to evaluate the spline
            int cell_number;						//stores the numbers of state memory cells of the filters
            int next_input_position;                //next cell of the filter receiveing the input
            std::vector<Vector6d> state;	            //stores the filters states
            Vector6d sum;                              //stores the sum of the filter output

    };

}

#endif /* FIRFILTER_H */
