#include "Src/opencv4/opencv2/opencv.hpp"
#include "Src/opencv4/opencv2/opencv_modules.hpp"
#include "Src/opencv4/opencv2/videoio.hpp"
#include <iomanip>
#include <iostream>
#include <vector>
#define APRILGRIDARRAYSIZE 36
#define DEBUG
struct marker_bounds {
  /* data */
  float bound_1x = 0;
  float bound_2x = 0;
  float bound_3x = 0;
  float bound_4x = 0;
  float bound_1y = 0;
  float bound_2y = 0;
  float bound_3y = 0;
  float bound_4y = 0;
  int id = 0;
};
extern "C" {
#include "Src/apriltag.h"
#include "Src/tag25h9.h"
#include "Src/tag36h11.h"
// #include "Tag25h7.h"
#include "Src/common/getopt.h"
#include "Src/tag16h5.h"
#include "Src/tagCircle21h7.h"
#include "Src/tagCircle49h12.h"
#include "Src/tagCustom48h12.h"
#include "Src/tagStandard41h12.h"
#include "Src/tagStandard52h13.h"
}

using namespace std;
using namespace cv;
class marker_pointcloud {
private:
  std::vector<marker_bounds> *vector_point_cloud;
  int* set_id;

  /* data */
public:
  marker_pointcloud(/* args */);
  ~marker_pointcloud();
};

marker_pointcloud::marker_pointcloud(/* args */) 
{
  set_id= nullptr;
}

marker_pointcloud::~marker_pointcloud() {}

class opencv_demo {
private:
  apriltag_family_t *tf = nullptr;
  apriltag_detector_t *td = nullptr;
  apriltag_detection_t *det = nullptr;
  //  this->td = apriltag_detector_create();

public:
  opencv_demo(/* args */);
  ~opencv_demo();
  Mat draw_marker(Mat input_frame, Mat Colour);
  /// @brief generates data for the april grid point cloud
  /// @return labelled image or return a matrix of zeros if not available
  marker_pointcloud get_bounds();
  marker_pointcloud set_bounds(marker_bounds input_tagset);
};
Mat opencv_demo::draw_marker(Mat input_frame, Mat Colour) {
  auto gray = input_frame;
  // Make an image_u8_t header for the Mat data
  image_u8_t im = {.width = gray.cols,
                   .height = gray.rows,
                   .stride = gray.cols,
                   .buf = gray.data};

  apriltag_detector_detect(td, &im);
  zarray_t *detections = apriltag_detector_detect(td, &im);
  if (zarray_size(detections) == APRILGRIDARRAYSIZE) {
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      line(Colour, Point(det->p[0][0], det->p[0][1]),
           Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
      line(Colour, Point(det->p[0][0], det->p[0][1]),
           Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
      marker_bounds *marker;
      marker->id= det->id;
      marker->bound_1x=det->p[0][0];
      marker->bound_2x=det->p[1][0];
      marker->bound_3x=det->p[2][0];
      marker->bound_4x=det->p[3][0];
      marker->bound_1y=det->p[0][1];
      marker->bound_2y=det->p[1][1];
      marker->bound_3y=det->p[2][1];
      marker->bound_4y=det->p[3][1];

      if (det->id == 23) {

        cout << det->p[0][0] << "\t" << det->p[1][0] << "\t" << det->p[2][0]
             << "\t" << det->p[3][0] << "\t";
        cout << det->p[0][1] << "\t" << det->p[1][1] << "\t" << det->p[2][1]
             << "\t" << det->p[3][1] << "\n";
      }
      // line(Colour, Point(det->p[1][0], det->p[1][1]),
      //          Point(det->p[2][0], det->p[2][1]),
      //          Scalar(0xff, 0, 0), 2);
      // line(Colour, Point(det->p[2][0], det->p[2][1]),
      //          Point(det->p[3][0], det->p[3][1]),
      //          Scalar(0xff, 0, 0), 2);

      stringstream ss;
      ss << det->id;
      String text = ss.str();
      int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
      double fontscale = 1.0;
      int baseline;
      Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
      putText(Colour, text,
              Point(det->c[0] - textsize.width / 2,
                    det->c[1] + textsize.height / 2),
              fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
    }
    apriltag_detections_destroy(detections);
    return Colour;

  }

  else
  {
#ifndef DEBUG
    return cv::Mat::zeros(Colour.size(), Colour.type());
#else
    return Colour;
#endif
}}
opencv_demo::opencv_demo(/* args */) {
  this->tf = tag36h11_create();
  tf->width_at_border = 8;
  tf->total_width = 10;
  tf->reversed_border = false;
  this->td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
}

opencv_demo::~opencv_demo() { tag36h11_destroy(tf); }

#include <time.h>

using namespace cv;
using namespace std;
opencv_demo aprilgrid;
int main(int argc, char **argv) {

  // Start default camera
  cv::VideoCapture cap(0);

  // cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps <<
  // endl;

  // Check if camera opened successfully
  if (!cap.isOpened()) {
    cout << "Error opening video stream" << endl;
    return -1;
  }

  // Default resolutions of the frame are obtained.The default resolutions are
  // system dependent. int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH); int
  // frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  double frame_counter;
  double fps = cap.get(CAP_PROP_FPS);
  cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps << endl;

  //   // Define the codec and create VideoWriter object.The output is stored in
  //   'outcpp.avi' file. VideoWriter video("outcpp.avi",
  //   cv::VideoWriter::fourcc('M','J','P','G'), 10,
  //   Size(frame_width,frame_height));
  time_t start, end;
  time(&start);
  while (1) {

    Mat frame, gray;

    // Capture frame-by-frame
    cap >> frame;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    frame_counter++;
    // If the frame is empty, break immediately
    if (frame.empty())
      break;

    // Write the frame into the file 'outcpp.avi'
    // cap.write(frame);
    frame = aprilgrid.draw_marker(gray, frame);
    // Display the resulting frame

    time(&end);

    imshow("Tag Detections", frame);
    // Press  ESC on keyboard to  exit
    char c = (char)waitKey(1);
    if (c == 27)
      break;
  }

  // When everything done, release the video capture and write object
  cap.release();
  //   video.release();

  // Closes all the frames
  destroyAllWindows();
  return 0;
}