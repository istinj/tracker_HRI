/*
 * colormod.h
 *
 *  Created on: 10/dic/2016
 *      Author: istin
 */
#pragma once
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <cmath>
//EIGEN
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

//OpenCV
#include <opencv2/opencv.hpp>

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#define FPS (int)1000/29


//! STATO = (x,y,Vx,Vy) -> laserscan space
struct State
{
	Eigen::Vector4f mean; // (x,y,vx,vy)
	Eigen::Matrix4f cov;
};

//! OBSERVATION = (x,y) of the laserscan detection's center.
struct Observation
{
	Eigen::Vector2f mean; // (x,y)
	Eigen::Matrix2f cov;
};


float roundPI2(float a);
void displayImage(const cv::Mat& image_, const std::string name_);


