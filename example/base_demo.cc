#include "Src/opencv4/opencv2/opencv_modules.hpp"
#include "Src/opencv4/opencv2/videoio.hpp"
#include <Src/opencv4/opencv2/opencv.hpp>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>
#define APRILGRIDARRAYSIZE 36
#define DEBUG

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

struct point {
  float x = 0;
  float y = 0;
};

struct marker_bounds {
  /* data */
  point bound_1;
  point bound_2;
  point bound_3;
  point bound_4;

  int id = 0;
};

using namespace std;

using namespace cv;
class marker_pointcloud {
private:
  int set_id;
  std::vector<marker_bounds> instance_vector;
  struct acquisition_instance_Str
  {
    vector<marker_bounds> vectore_acq;
    int id;

    /* data */
  };
  
  std::vector<marker_bounds> vector_detection_set;
  /* data */
public:
  marker_pointcloud();
  ~marker_pointcloud();
  void create_detect_instance(vector<marker_bounds> input_vector);
  void create_acquisition_instance(vector<marker_bounds> input_vector);
  int get_set_id() { return set_id; }
};
void marker_pointcloud::create_acquisition_instance(
    vector<marker_bounds> input_vector)

{
  this->instance_vector = input_vector;
  this->set_id++;
  // TODO:  you may add filters here to determine stuff currently left blank
  acquisition_instance_Str acq;
  acq.id=this->set_id;
  acq.vectore_acq=instance_vector;
  #ifdef DEBUGV
    cout << "hello";
    cout << "opset" << this->instance_vector.size();
  #endif
  this->create_acquisition_instance(acq);
}
void marker_pointcloud::create_acquisition_instance(acquisition_instance_Str acq)
{
  this->vector_detection_set
 
}
marker_pointcloud::marker_pointcloud() {
  cout << "ctor created";
  this->set_id = 0;
}

marker_pointcloud::~marker_pointcloud() {}

class opencv_demo {
private:
  apriltag_family_t *tf = nullptr;
  apriltag_detector_t *td = nullptr;
  apriltag_detection_t *det = nullptr;
  //  this->td = apriltag_detector_create();
  marker_pointcloud detection_input;

public:
  opencv_demo(/* args */);
  ~opencv_demo();
  Mat draw_marker(Mat input_frame, Mat Colour);
  bool set_complete = false;
  /// @brief generates data for the april grid point cloud
  /// @return labelled image or return a matrix of zeros if not available
};
Mat opencv_demo::draw_marker(Mat input_frame, Mat Colour) {
  auto gray = input_frame;
  vector<marker_bounds> local_store;
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
      marker_bounds marker;
      marker.id = det->id;
      marker.bound_1.x = det->p[1][0];
      marker.bound_2.x = det->p[2][0];
      marker.bound_3.x = det->p[3][0];
      marker.bound_4.x = det->p[4][0];
      marker.bound_1.y = det->p[1][1];
      marker.bound_2.y = det->p[2][1];
      marker.bound_3.y = det->p[3][1];
      marker.bound_4.y = det->p[4][1];
      local_store.push_back(marker);
#ifdef DEBUGV

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
#endif
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
    this->detection_input.create_acquisition_instance(local_store);
    apriltag_detections_destroy(detections);
   
    int set_num = this->detection_input.get_set_id();
    cout<<"\n"<<set_num;
    if (set_num>= 100)
      this->set_complete= true;

   return Colour;
  }

  else {
#ifndef DEBUG
    return cv::Mat::zeros(Colour.size(), Colour.type());
#else
    return Colour;
#endif
  }
}
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
    if (aprilgrid.set_complete)
      break;
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