#include "simplecollisionavoidance.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
using namespace cv;
using namespace std;
const float minAreaRatio = 0.01;
const float maxAreaRatio = 0.85;
const float overlap_ratio = 0.99;


static inline float distance(const Point& a, const Point& b){
  //euclide distance
  return pow(pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0), 0.5);
}

static float getMinDepthDistance(const Mat& depth_img, const vector<Point>& points){
  float min_depth = std::numeric_limits<float>::max();
  for(unsigned int i = 0; i < points.size(); i++){
    float depth =  (depth_img.at<short>(points[i].y,points[i].x))/128.0; //in real life, src is a 16bit depth map image
    if(min_depth > depth){
      min_depth = depth;
    }
  }
  return min_depth;
}

bool SimpleCollisionAvoidance::isInsideContours(const vector<Point>& contour, const Rect& rect, const float overlap_ratio, vector<Point>& points_outside){
  int insideNum = 0;
  for(int i = rect.x; i <  rect.x + rect.width; i++){
    for(int j = rect.y; j < rect.y + rect.height; j++){
      if(pointPolygonTest(contour, Point(i, j), false) >= 0){ //Check a certain point is inside (>= 0) contours
        insideNum++;
        vector<Point>::iterator it = std::find(points_outside.begin(), points_outside.end(), Point(i,j));
        if(it != points_outside.end()){
          points_outside.erase(it);
        }
      }
    }
  }
  if(insideNum > overlap_ratio * (_win_size_width * _win_size_height)){
    return true;
  }
  return false;
}


void SimpleCollisionAvoidance::point2FOV(const Point &p, float &horiz_ang, float &vert_ang)
{
  horiz_ang = p.x * _horiz_fov / _width;
  vert_ang = p.y * _vert_fov / _height;
}


int SimpleCollisionAvoidance::startDetection(const cv::Mat &depth_img, const float thresh, const cv::Rect& ref_rect)
{
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  //Convert the depth_img to binary data based on a given threshold
  threshold( depth_img, _threshold_img, thresh, 255, THRESH_BINARY );
  //bitwise_not(_threshold_img, _threshold_img);
  //Remove unwanted black and white noise
  const int dilation_size = 10;
  Mat element = getStructuringElement( MORPH_RECT,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  morphologyEx(_threshold_img, _filtered_img, MORPH_OPEN, element );

  //Find contours on filtered binary image
  findContours( _filtered_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
   /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  /// Draw polygonal contour + bonding rects + circles
  //_final_img = Mat::zeros( depth_img.size(), CV_8UC3 );
  cvtColor(depth_img, _final_img, CV_GRAY2RGB);
  //depth_img.copyTo(_final_img);
  vector<RotatedRect> minRect (contours.size());
  vector<int> validContourIndexes;
  //Remove invalid contours (not in range)
  for(unsigned int i = 0; i < contours.size(); i++ )
  {
    approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
    boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    minRect[i] = minAreaRect(Mat(contours_poly[i]));
    Point2f rect_points[4]; minRect[i].points( rect_points );
    //Calculate area of polygon with shoelace formula
    float area = 0.0;
    int k = 3;
    float minY, maxY;
    minY = maxY = rect_points[0].y;
    for( int j = 0; j < 4; j++ ){
      area += (rect_points[k].x + rect_points[j].x) * (rect_points[k].y - rect_points[j].y);
      k = j;
      if(minY > rect_points[j].y){
        minY = rect_points[j].y;
      }
      if(maxY < rect_points[j].y){
        maxY = rect_points[j].y;
      }
    }
    area = abs(area/2.0);
    float minArea = minAreaRatio * _width * _height;
    float maxArea = maxAreaRatio * _width * _height;
    if(area > minArea && area < maxArea){
        drawContours( _final_img, contours_poly, i, Scalar(0, 255, 0), 1, 8, vector<Vec4i>(), 0, Point() );
        validContourIndexes.push_back(i);
    }
  }

  /// Measure obstacle distance at center with WIN_SIZE
//  Point tl(_width/2 - _win_size_width/2, _height/2 - _win_size_height/2);
//  Rect rectCenter(tl, Size(_win_size_width, _win_size_height));
  Rect rectCenter = ref_rect;
  /// Checking whether current rectangle overlapped with countours.
  /// If it is overlapped 100% then go forward
  bool insideContour = true;
  vector<Point> points_outside;
  for(int i = rectCenter.x; i < rectCenter.x + rectCenter.width; i++){
    for(int j = rectCenter.y; j < rectCenter.y + rectCenter.height; j++){
      points_outside.push_back(Point(i, j));
    }
  }
  for(unsigned int i = 0; i < validContourIndexes.size(); i++){
    if(!isInsideContours(contours_poly[validContourIndexes[i]], rectCenter, overlap_ratio, points_outside)){
      insideContour = false;
    }
  }
  float distance_at_center = 0;
  if(points_outside.size() > 0){
    distance_at_center = getMinDepthDistance(depth_img, points_outside);
  }
  int direction = GO_STRAIGHT;
  if(!insideContour && distance_at_center < _safety_distance && points_outside.size() > 0){
    //Find window inside contours which is closest to center
    //Find center of contours
    float minDist = std::numeric_limits<float>::max();
    //Rect targetRect;
    bool foundPath = false;
    for(unsigned int i = 0; i < validContourIndexes.size(); i++){
      Rect boundRect = boundingRect(contours_poly[validContourIndexes[i]]);
      for(int w = boundRect.x; w < boundRect.x + (boundRect.width - _win_size_width); w+=_win_size_width/3){
        for(int h = boundRect.y; h < boundRect.y + (boundRect.height - _win_size_height); h+=_win_size_height/3){
          Rect rect(Point(w, h), Size(_win_size_width, _win_size_height));
          vector<Point> p_outs;
          if(isInsideContours(contours_poly[validContourIndexes[i]], rect, overlap_ratio, p_outs)){
            float dist = distance(Point(rectCenter.x + rectCenter.width/2, rectCenter.y + rectCenter.height/2),
                                   Point(rect.x + rect.width/2, rect.y + rect.height/2));
            if(minDist > dist){
              minDist = dist;
              _targetRect = rect;
              foundPath = true;
            }
          }
        }
      }
    }
    if(foundPath){
      //Transform pixel offset to real coordination
      rectangle(_final_img, rectCenter, Scalar(0, 0, 255), 1);
      rectangle(_final_img, _targetRect, Scalar(0, 255, 0), 1);
      Point center(rectCenter.x + rectCenter.width/2, rectCenter.y + rectCenter.height/2);
      Point target(_targetRect.x + _targetRect.width/2, _targetRect.y + _targetRect.height/2);
      line(_final_img, center, target, Scalar(255, 0, 0), 1);
      float horiz_center, vert_center, horiz_target, vert_target;
      horiz_center = vert_center = horiz_target = vert_target = 0;
      point2FOV(center, horiz_center, vert_center);
      point2FOV(target, horiz_target, vert_target);
      float delta_horiz = horiz_center - horiz_target;
      float delta_vert = vert_center - vert_target;
      float horiz_dist = distance_at_center * tan(delta_horiz);
      float vert_dist = distance_at_center * tan(delta_vert);
      float x_center = 0;
      float y_center = 0;
      //float z_center = distance_at_center;
      _x = x_center + horiz_dist;
      _y = y_center + vert_dist;
      _z = distance_at_center;
      _yaw = delta_horiz;
//      char text[100] = {0};
//      sprintf(text, "center(%.2f, %.2f, %.2f) target(%.2f, %.2f, %.2f)",x_center, y_center, z_center, _x, _y, _z);
//      _info1 = text;
//      text[0] = '\0';
//      sprintf(text, "min_depth=%.2f horiz_angl=%.2f vert_angl_=%.2f", distance_at_center, delta_horiz, delta_vert);
//      _info2 = text;
      direction = TURN;
    }else{
      direction = STOP;
      _targetRect = ref_rect;
      rectangle(_final_img, rectCenter, Scalar(0, 0, 255), 1);
    }
  }else{  //Keep moving
    _targetRect = ref_rect;
    rectangle(_final_img, _targetRect, Scalar(0, 255, 0), 1);
  }

//  if(direction != STOP){
//    putText(_final_img, _info1.c_str(), Point2f(10, _height - 20), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 0, 255));
//    putText(_final_img, _info2.c_str(), Point2f(10, _height - 10), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 0, 255));
//  }

  return direction;

}

void SimpleCollisionAvoidance::visualizeDirection(const int visualType)
{
  switch (visualType) {
  case FINAL_RESULT_ONLY:
    namedWindow( "Result", CV_WINDOW_AUTOSIZE );
    imshow( "Result", _final_img );
    break;
  case FILTERED_ONLY:
    namedWindow( "Filtered", CV_WINDOW_AUTOSIZE );
    imshow( "Filtered", _filtered_img );
    break;
  case ALL:
    namedWindow( "Result", CV_WINDOW_AUTOSIZE );
    imshow( "Result", _final_img );
    namedWindow( "Filtered", CV_WINDOW_AUTOSIZE );
    imshow( "Filtered", _filtered_img );
    break;
  default:
    break;
  }
}
