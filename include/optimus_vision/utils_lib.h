#ifndef UTILS_LIB_H
#define UTILS_LIB_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cstdint>

uint16_t getPanelDistance(cv::Mat depthValues);
uint16_t getGroundDistance(cv::Mat depthValues);
uint16_t calculateClippingDistance(uint16_t groundDistance, uint16_t panelDistance);

cv::Mat findDrawContours(cv::Mat close,cv::Mat depthColormap);
std::vector<cv::Point> polyContour(cv::Mat depthColormap, std::vector<cv::Point> contour);
std::vector<cv::Point> simplifyContour(std::vector<cv::Point> contour);
std::vector<cv::Point> filterPolyLines(std::vector<cv::Point> contour);

cv::Mat filterDepth(cv::Mat depthValues, uint16_t clippingDistance);


cv::Mat processDepth(cv::Mat depthImage, uint16_t imageHeight, uint16_t imageWidth);

#endif  // UTILS_LIB_H