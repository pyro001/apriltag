#include <opencv4/opencv2/opencv_modules.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/opencv.hpp>

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
using namespace std;

// ===================================================
//  opencv aprilgrid extraction support structures
//====================================================
struct point
{
  float x = 0;
  float y = 0;
  bool operator==(const point &comparator)

  {
#ifdef DEBUGV
    cout << "\n operator overload::" << int(this->x / 10) << "\t" << int(comparator.x / 10);
#endif
    return (int(this->x / RESOLUTIONMOD) == int(comparator.x / RESOLUTIONMOD) || int(this->y / RESOLUTIONMOD) == int(comparator.y / RESOLUTIONMOD));
  }
};

struct marker_bounds
{
  /* data */
  point bound_1;
  point bound_2;
  point bound_3;
  point bound_4;

  int id = 0;
  bool operator==(const marker_bounds &comparator)
  {
    return this->bound_1 == comparator.bound_1 ||
           bound_2 == comparator.bound_2 ||
           bound_3 == comparator.bound_3 ||
           bound_4 == comparator.bound_4;
  }
};

struct acquisition_instance_Str
{
  std::vector<marker_bounds> vector_acq;
  int id;

  /* data */
};

using namespace cv;

// ===================================================
//  point cloud selection
//====================================================
class marker_pointcloud
{
private:
  int set_id;
  std::vector<marker_bounds> instance_vector;

  const int max_iter = 3;
  std::vector<acquisition_instance_Str> vector_detection_set;
  /* data */
public:
  marker_pointcloud();
  ~marker_pointcloud();
  /// @brief
  /// @return
  bool motion_filter(marker_bounds input);
  void create_acquisition_instance(acquisition_instance_Str input_vector);
  void create_detect_instance(std::vector<marker_bounds> input_vector);
  int get_set_id() { return set_id; }
};
// ===================================================
// fuctions in marker pointcloud class
//====================================================
void marker_pointcloud::create_detect_instance(
    std::vector<marker_bounds> input_vector)

{
  this->instance_vector = input_vector;
  marker_bounds manifold = input_vector.front(); // this is sorted by ids get id 2 bound1 and pass to motion filter
  point ptmanifold = manifold.bound_1;
#ifdef DEBUGV
  cout << "input control motion filter:" << ptmanifold.x << "\t::\t" << ptmanifold.y;
#endif
  // TODO:  you may add filters here to determine stuff currently left blank
  bool new_image_found = this->motion_filter(manifold);
  if (new_image_found)
  {
    this->set_id++;
    acquisition_instance_Str acq;
    acq.id = set_id;
    acq.vector_acq = instance_vector;
#ifdef DEBUGV
    cout << "hello";
    cout << "opset" << this->instance_vector.size();
#endif
    create_acquisition_instance(acq);
  }
}
void marker_pointcloud::create_acquisition_instance(
    acquisition_instance_Str input_vector)
{
  this->vector_detection_set.emplace_back(input_vector);
}
marker_pointcloud::marker_pointcloud()
{
  cout << "ctor created";
  this->set_id = 0;
}

marker_pointcloud::~marker_pointcloud() {}

// ===================================================
//  motion detection functions
//====================================================
bool marker_pointcloud::motion_filter(marker_bounds input)
{

  // step 1 from the last value of the marker 2 bound 1 get values

  // step 2 from the bound 1 marker 2 verify if the values are in range of the other stored values
  //  if not in range add as a valid data set.

  if (this->vector_detection_set.empty())
  {
    cout << "empty vector: pass through \n";
    return (true);
  }
  for (std::vector<acquisition_instance_Str>::iterator
           it = vector_detection_set.begin();
       it != vector_detection_set.end(); ++it)
  {

    vector<marker_bounds> detectionstub = it->vector_acq;

    marker_bounds mkb = detectionstub.front();
#ifdef DEBUGV
    cout << "\ncomapring id" << mkb.id << "\t::\t" << input.id << "\n";
#endif
    if (mkb == input)
      return (false);
  }
  return (true);
}
// ===================================================
//  opencv aprilgrid extraction
//====================================================
class opencv_demo
{
private:
  apriltag_family_t *tf = nullptr;
  apriltag_detector_t *td = nullptr;
  apriltag_detection_t *det = nullptr;
  //  this->td = apriltag_detector_create();
  marker_pointcloud detection_input;
  int max_iter;

public:
  opencv_demo(/* args */);
  ~opencv_demo();
  opencv_demo(int max_iterations);
  Mat draw_marker(Mat input_frame, Mat Colour);
  bool set_complete = false;

  /// @brief generates data for the april grid point cloud
  /// @return labelled image or return a matrix of zeros if not available
};

// ===================================================
//  opencv aprilgrid extraction functions
//====================================================
Mat opencv_demo::draw_marker(Mat input_frame, Mat Colour)
{
  auto gray = input_frame;
  vector<marker_bounds> local_store;
  // Make an image_u8_t header for the Mat data
  image_u8_t im = {.width = gray.cols,
                   .height = gray.rows,
                   .stride = gray.cols,
                   .buf = gray.data};
#ifdef DEBUGV
  apriltag_detector_detect(td, &im);
#endif
  zarray_t *detections = apriltag_detector_detect(td, &im);
#ifdef DEBUGV
  cout << "/\ndetected\t\t:::\t\t" << zarray_size(detections);
#endif

  if (zarray_size(detections) >= APRILGRIDARRAYSIZE)
  {
    for (int i = 0; i < zarray_size(detections); i++)
    {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      line(Colour, Point(det->p[0][0], det->p[0][1]),
           Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
      line(Colour, Point(det->p[0][0], det->p[0][1]),
           Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
      marker_bounds marker;
      marker.id = det->id;
      marker.bound_1.x = det->p[0][0];
      marker.bound_2.x = det->p[1][0];
      marker.bound_3.x = det->p[2][0];
      marker.bound_4.x = det->p[3][0];
      marker.bound_1.y = det->p[0][1];
      marker.bound_2.y = det->p[1][1];
      marker.bound_3.y = det->p[2][1];
      marker.bound_4.y = det->p[3][1];
      local_store.push_back(marker);
#ifdef DEBUGV

      if (det->id == 2)
      {

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
    this->detection_input.create_detect_instance(local_store);
    apriltag_detections_destroy(detections);

    int set_num = this->detection_input.get_set_id();
    cout << "\n"
         << set_num;
    if (set_num >= 10)
      this->set_complete = true;

    return Colour;
  }

  else
  {
#ifndef DEBUG
    return cv::Mat::zeros(Colour.size(), Colour.type());
#else
    return Colour;
#endif
  }
}
opencv_demo::opencv_demo(/* args */)
{
  this->max_iter = 10;
  this->tf = tag36h11_create();
  tf->width_at_border = 8;
  tf->total_width = max_iter;
  tf->reversed_border = false;
  this->td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
}
opencv_demo::opencv_demo(int max_iterations)
{
  this->max_iter = max_iterations;
  this->tf = tag36h11_create();
  tf->width_at_border = 8;
  tf->total_width = max_iter;
  tf->reversed_border = false;
  this->td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
}
opencv_demo::~opencv_demo() { tag36h11_destroy(tf); }
// ===================================================
// MAIN
//====================================================
#include <time.h>

using namespace cv;
using namespace std;
opencv_demo aprilgrid;
int main(int argc, char **argv)
{

  // Start default camera
  cv::VideoCapture cap(0);

  // cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps <<
  // endl;

  // Check if camera opened successfully
  if (!cap.isOpened())
  {
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
  while (1)
  {

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