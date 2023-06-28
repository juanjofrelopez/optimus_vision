#include "optimus_vision/utils_lib.h"

// Global filter Variables
const uint16_t poly_buffer_size = 15;
uint16_t poly_buffer_indx = 0;
uint16_t poly_buffer[4][2][poly_buffer_size] = {{{0}}};

uint16_t getPanelDistance(cv::Mat depthValues){
    std::vector<int> panelValues;
    for (int c = 292; c < 348; c++){
        for (int r = 15; r < 465; r++){
            panelValues.push_back(static_cast<int>(depthValues.at<int>(r, c)));
        }
    }
    // calculate median
    std::sort(panelValues.begin(), panelValues.end());
    int n = panelValues.size();
    int median = 0;
    if (n % 2 == 0)
        median = (panelValues[n / 2 - 1] + panelValues[n / 2]) / 2.0;
    else
        median = panelValues[n / 2];
    return median;
}

uint16_t getGroundDistance(cv::Mat depthValues){
    std::vector<uint16_t> groundValues;
    for (int c = 15; c < 70; c++){
        for (int r = 15; r < 465; r++){
            groundValues.push_back(static_cast<uint16_t>(depthValues.at<int>(r,c)));
        }
    }
    for (int c = 570; c< 625; c++){
        for (int r = 15; r < 465; r++){
            groundValues.push_back(static_cast<uint16_t>(depthValues.at<int>(r,c)));
        }
    }
    // calculate median
    std::sort(groundValues.begin(), groundValues.end());
    uint16_t n = groundValues.size();
    uint16_t median = 0;
    if (n % 2 == 0)
        median = (groundValues[n / 2 - 1] + groundValues[n / 2]) / 2.0;
    else
        median = groundValues[n / 2];
    return median;
}

uint16_t calculateClippingDistance(uint16_t groundDistance, uint16_t panelDistance){
    uint16_t  ratio = groundDistance - panelDistance;
    if (ratio < 1000 || ratio > 3000 || panelDistance < 3000){
        return 5000;
    }
    uint16_t clippingDistance = (panelDistance+groundDistance)/2;
    return clippingDistance;
}

cv::Mat filterDepth(cv::Mat depthValues, uint16_t clippingDistance){
    for (uint16_t r = 0; r < 480; r++){
        for (uint16_t c = 0; c < 640; c++){
            uint16_t tempValue = static_cast<uint16_t>(depthValues.at<uint32_t>(r,c));
            if(tempValue > clippingDistance){
                // asignarle un cero en la misma posicion
                depthValues.at<uint32_t>(r,c) = 0;
            }
        }
    }

    cv::Mat grayScaleDepth;
    convertScaleAbs(depthValues, grayScaleDepth);
    
    // uint16_t thresh = 255;
    // uint16_t max_value = 255;
    // cv::Mat thresholded_image;
    // cv::threshold(grayScaleDepth, thresholded_image, thresh, max_value, cv::THRESH_TRUNC);

    // cv::Mat denoise;
    // cv::medianBlur(thresholded_image, denoise, 7);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(27, 27));
    cv::Mat close;
    //cv::morphologyEx(thresholded_image, close, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(grayScaleDepth, close, cv::MORPH_CLOSE, kernel);

    return close;
}

std::vector<cv::Point> simplifyContour(std::vector<cv::Point> contour){
    int n_corners=4;
    int n_iter = 0;
    int max_iter = 100;

    float lb = 0.0;
    float ub = 1.0;
    
    while(true){
        n_iter = n_iter+1;
        if(n_iter > max_iter){
            return contour; 
        }
        
        float k = (lb + ub)/2.0;
        float eps = k*cv::arcLength(contour, true);
        
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, eps, true);
        
        if (approx.size() > n_corners){
            lb = (lb + ub)/2.0;
        }else if (approx.size() < n_corners){
            ub = (lb + ub)/2.0;
        }else{
            return approx;
        }
    }
}

std::vector<cv::Point> filterPolyLines(std::vector<cv::Point> contour){
    //std::vector<cv::Point> filteredPoly;
    int filteredPoly[4][2];

    // anexar los puntos actuales a la posicion actual del array del filtro
    for(int i = 0 ; i<contour.size();i++){
        poly_buffer[i][0][poly_buffer_indx] = contour[i].x;
        poly_buffer[i][1][poly_buffer_indx] = contour[i].y;
    }
    poly_buffer_indx = (poly_buffer_indx+1)%poly_buffer_size;

    for(int r = 0 ; r<4;r++){
        for(int c = 0 ; c<2; c++){
            uint16_t temporal[poly_buffer_size] = {0};

            for(int d = 0 ; d<poly_buffer_size; d++){
                temporal[d] = poly_buffer[r][c][d];
            }
            
            // calculate median
            int endSize = sizeof(temporal) / sizeof(temporal[0]);
            std::sort(temporal, temporal + endSize);
            int n = sizeof(temporal) / sizeof(uint16_t);
            int median = 0;
            if (n % 2 == 0)
                median = (temporal[n / 2 - 1] + temporal[n / 2]) / 2.0;
            else
                median = temporal[n / 2];
            
            // prepare output 
            filteredPoly[r][c] = median;
        }
    }

    std::vector<cv::Point> gilada;
    for(int i = 0; i<4;i++){
        cv::Point temporalPoint(filteredPoly[i][0],filteredPoly[i][1]);
        gilada.push_back(temporalPoint);
    }
    return gilada;
}

std::vector<cv::Point> polyContour(std::vector<cv::Point> contour){
    std::vector<cv::Point> approximatedContour = simplifyContour(contour);
    std::vector<cv::Point> filteredPoly = filterPolyLines(approximatedContour);
    return filteredPoly;
}

cv::Mat findDrawContours(cv::Mat close,cv::Mat depthColormap){
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(close, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    
    for (size_t i = 0; i< contours.size(); i++){
        //drawContours( depthColormap, contours, (int)i, (0,255,0), 2, cv::LINE_8, hierarchy, 0 );
        double area = cv::contourArea(contours[i]);
        if (!(area < 150000 || area > 200000)){
            std::vector<cv::Point> approximatedContour = polyContour(contours[i]);
            cv::drawContours(depthColormap, std::vector<std::vector<cv::Point> >(1,approximatedContour), -1, CV_RGB(0,255,0), 2, 8);
        }
    }
    return depthColormap;   
}

cv::Mat processDepth(cv::Mat depthValues){
    uint16_t panelDistance = getPanelDistance(depthValues);
    uint16_t groundDistance = getGroundDistance(depthValues);
    uint16_t clippingDistance = calculateClippingDistance(groundDistance,panelDistance);
    
    // std::cout<<panelDistance<<" "<<groundDistance<<" "<<clippingDistance<<std::endl;
    // std::cout<<"\x1b[1A"<<"\x1b[2K"<<"\r";
    
    cv::Mat close = filterDepth(depthValues,clippingDistance);

    //cv::Mat depthColormap;
    cv::Mat depthMask = cv::Mat::zeros(close.size(), CV_8UC3);
    // cv::convertScaleAbs(depthValues, depthColormap);
    // cv::applyColorMap(depthColormap, depthColormap, cv::COLORMAP_JET);
    depthMask = findDrawContours(close,depthMask);

    return depthMask;
}