#ifndef SIMPLECOLLISIONAVOIDANCE_H
#define SIMPLECOLLISIONAVOIDANCE_H
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include<opencv2/opencv.hpp>
#include<vector>
#include<string>
enum AddressDirection{
  GO_STRAIGHT = 0,
  STOP,
  TURN
};

enum VisualizationType{
  FINAL_RESULT_ONLY,
  FILTERED_ONLY,
  ALL
};

class SimpleCollisionAvoidance
{
public:
  SimpleCollisionAvoidance(int w, int h, int win_size_w, int win_size_h, float h_f, float v_f, float safety_distance):
    _width(w), _height(h), _win_size_width(win_size_w), _win_size_height(win_size_h),
    _safety_distance(safety_distance),
    _horiz_fov(h_f), _vert_fov(v_f){
  }
  int startDetection(const cv::Mat& depth_img, const float threshold, const cv::Rect& refRect);
  void getTargetCoordinate(float& x, float& y, float& z){x = _x; y = _y; z = _z;}
  void visualizeDirection(const int visualType = ALL);
  cv::Rect& getTargetRect(){return _targetRect;}
private:
  int _width, _height, _win_size_width, _win_size_height;
  float _safety_distance;
  float _horiz_fov, _vert_fov;
  cv::Rect _targetRect;
  float _x, _y, _z;
//  std::string _info1, _info2;
  cv::Mat _threshold_img, _filtered_img, _final_img;
  void point2FOV(const cv::Point& p, float& horiz_ang, float& vert_ang);
  bool isInsideContours(const std::vector<cv::Point>& contour, const cv::Rect& rect, const float overlap_ratio, std::vector<cv::Point>& points_outside);
};

#endif // SIMPLECOLLISIONAVOIDANCE_H
