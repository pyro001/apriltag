#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/opencv_modules.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <sys/stat.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>
#include <semaphore.h>
#define APRILGRIDARRAYSIZE 20 // 36
#define DEBUG
#define RESOLUTIONMOD 1
#define CPPHTTPLIB_OPENSSL_SUPPORT
#include <pthread.h>
extern "C"
{
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
sem_t thread_release, thread_synch;
// ===================================================
//  opencv aprilgrid extraction support structures
//====================================================
struct point
{
  float x = 0;
  float y = 0;
  cv::Point2f pt2D = cv::Point2f(this->x, this->y);
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
  point centre;
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

  float squaresize = (25.55 / 1000);
  float margin = (06.8 / 1000);
  double computeReprojectionErrors(const vector<vector<cv::Point3f>> &objectPoints,
                                   const vector<vector<cv::Point2f>> &imagePoints,
                                   const vector<cv::Mat> &rvecs, const vector<cv::Mat> &tvecs,
                                   const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs);

  const int max_iter = 3;

  std::vector<acquisition_instance_Str> vector_detection_set;
  /* data */
public:
  marker_pointcloud();
  marker_pointcloud(string file);
  ~marker_pointcloud();
  /// @brief
  /// @return
  bool motion_filter(marker_bounds input);
  void create_acquisition_instance(acquisition_instance_Str input_vector);
  void create_detect_instance(std::vector<marker_bounds> input_vector);
  void setup_calibration(float square_size, float space_size, cv::Mat img);
  int get_set_id() { return set_id; }
  std::string filename;
};

// ===================================================
// fuctions in marker pointcloud class
//====================================================
void marker_pointcloud::create_detect_instance(
    std::vector<marker_bounds> input_vector)

{
  this->instance_vector = input_vector;
  marker_bounds manifold = input_vector.front(); // this is sorted by ids get id 2 bound1 and pass to motion filter

#ifdef DEBUGV
  cout << "input control motion filter:" << ptmanifold.x << "\t::\t" << ptmanifold.y;
#endif
  // TODO:  you may add filters here to determine stuff currently left blank
  bool new_image_found =  this->motion_filter(manifold);
  if (new_image_found)
  {
    this->set_id++;
    cout << "\n batch number" << this->set_id;
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
  this->filename = "store_default.yaml";
  cout << "ctor created";
  this->set_id = 0;
}
marker_pointcloud::marker_pointcloud(string file)
{
  this->filename = file;
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
    // cout << "empty vector: pass through \n";
    return (true);
  }
  for (std::vector<acquisition_instance_Str>::iterator
           it = vector_detection_set.begin();
       it != vector_detection_set.end(); ++it)
  {

    vector<marker_bounds> detectionstub = it->vector_acq;
    for (std::vector<marker_bounds>::iterator it2 = detectionstub.begin(); it2 != detectionstub.end(); ++it2)
    {
      marker_bounds mkb = detectionstub.front();
      if (mkb.id == input.id)
      {
#ifdef DEBUGV
        cout << "\ncomapring id" << mkb.id << "\t::\t" << input.id << "\n";
#endif
        if (mkb == input)
          return (false);
      }
    }
  }
  return (true);
}
double marker_pointcloud::computeReprojectionErrors(const vector<vector<cv::Point3f>> &objectPoints,
                                                    const vector<vector<cv::Point2f>> &imagePoints,
                                                    const vector<cv::Mat> &rvecs, const vector<cv::Mat> &tvecs,
                                                    const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)
{
  vector<Point2f> imagePoints2;
  int i;
  double totalErr = 0, err;
  vector<float> perViewErrors;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i)
  {
    projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);

    int n = (int)objectPoints[i].size();

    cout << err * err / n << "\nerror" << err / n << "round id\t" << i << "size of objective" << n << "\n";
    // perViewErrors[i] = (float)std::sqrt(err / n);
    totalErr += (err * err / n);
    // totalPoints += n;
  }
  cout << "total poincould size " << (int)objectPoints.size() << "\tmean error " << sqrt(totalErr / (int)objectPoints.size()) << "\n";

  return (double(sqrt(totalErr / (int)objectPoints.size())));
}
void marker_pointcloud::setup_calibration(float square_size, float space_size, cv::Mat img)
{
  vector<vector<Point3f>> object_points;
  vector<vector<Point2f>> image_points;
  vector<vector<Point3f>> object_points2;
  vector<vector<Point2f>> image_points2;
  cv::Mat K;
  cv::Mat D;
  cv::Mat K2;
  cv::Mat D2;
  vector<Mat> rvecs, tvecs, rvecs2, tvecs2;
  int flag = 0;
  flag |= cv::CALIB_FIX_K4;

  flag |= cv::CALIB_FIX_K5;
  for (std::vector<acquisition_instance_Str>::iterator
           it = this->vector_detection_set.begin();
       it != this->vector_detection_set.end(); ++it)
  {
    vector<cv::Point2f> image_point;
    vector<Point3f> object_point;
    int old_id = 1;

    point pt_ref;
    pt_ref.x = (this->squaresize + this->margin) * 6;
    std::vector<marker_bounds> detection_stub = it->vector_acq;

    for (std::vector<marker_bounds>::iterator
             it2 = detection_stub.begin();
         it2 != detection_stub.end(); ++it2)
    {
      pt_ref.x = pt_ref.x - (this->squaresize + this->margin);
      // if (it2->id - old_id > 1)
      // {
      //   // cout << "\ndifferential on id\t" << it2->id << "::" << old_id << "::" << it2->id - old_id;
      //   pt_ref.x = pt_ref.x - ((it2->id - (old_id)-1) * (this->margin + this->squaresize));
      //   if (double(pt_ref.x) < 0.000001 && double(pt_ref.x) > -0.000001)
      //   {
      //     pt_ref.x = 0.0;
      //   }

      //   if (pt_ref.x < -0.00001 && it2->id - old_id < 6)
      //   {
      //     pt_ref.y = pt_ref.y + (this->margin + this->squaresize);
      //     pt_ref.x = pt_ref.x + (this->squaresize + this->margin) * 6;
      //   }
      //   else if (pt_ref.x < -0.00001 && (it2->id - old_id) == 6)
      //     {
      //       pt_ref.y = pt_ref.y + (this->margin + this->squaresize);
      //     }

      //   // cout << "\n projected next point" << pt_ref.x << ":\t:" << pt_ref.y;
      // }
      if ((it2->id - old_id) > 1)
      { // alternate route just do what you were doing for 36pts but dont record
        // cout << "\ndifferential on id\t" << it2->id << "::" << old_id << "::" << it2->id - old_id;
        for (int i = old_id + 1; i < it2->id; i++)
        {
          // cout << "\nget id " << i;
          // cout << "point at" << pt_ref.x << "::\t" << pt_ref.y;
          if (pt_ref.x <= 0)
          {
            // cout << "row skip" << pt_ref.x << "::\t" << pt_ref.y;

            pt_ref.y = pt_ref.y + this->margin + this->squaresize;

            pt_ref.x = (this->squaresize + this->margin) * 6;
          }
          pt_ref.x = pt_ref.x - (this->squaresize + this->margin);
        }
        // cout << "\n projected next point" << pt_ref.x << "::\t" << pt_ref.y;
      }

      if (pt_ref.x <= 0.00000001)
        pt_ref.x = 0;
      // cout << "\n"
      //      << it2->id << "for id" << pt_ref.x << "::" << pt_ref.y << "\n";

      image_point.push_back(cv::Point2f(it2->bound_2.x, it2->bound_2.y));
      // cout << "\n" << it2->id << "for id" << it2->bound_1.x << "::" << it2->bound_1.y << "\n";
      object_point.push_back(cv::Point3f(pt_ref.x, pt_ref.y, 0));
      // cout << "\n"<< it2->id << "for id" << pt_ref.x << "::" << "0" << "\n";
      image_point.push_back(cv::Point2f(it2->bound_3.x, it2->bound_3.y));
      // cout << "\n"<< it2->id << "for id" << it2->bound_1.x << "::" << it2->bound_1.y << "\n";
      object_point.push_back(cv::Point3f(pt_ref.x, pt_ref.y + this->squaresize, 0));
      // cout << cv::Point3f(pt_ref.x, pt_ref.y + this->squaresize, 0);

      image_point.push_back(cv::Point2f(it2->bound_1.x, it2->bound_1.y));
      // cout << cv::Point2f(it2->bound_3.x, it2->bound_3.y);
      // cout<<cv::Point3f(pt_ref.x + this->squaresize, 0, 0);
      object_point.push_back(cv::Point3f(pt_ref.x + this->squaresize, pt_ref.y, 0));

      image_point.push_back(cv::Point2f(it2->bound_4.x, it2->bound_4.y));

      object_point.push_back(cv::Point3f(pt_ref.x + this->squaresize, pt_ref.y + this->squaresize, 0));
      // cout << "\n"
      //      << it2->id << "for id" << pt_ref.x << "::" << pt_ref.y << "\n";
      // cout << "\n1:::\t" << (cv::Point2f(it2->bound_1.x, it2->bound_1.y)) << (cv::Point3f(pt_ref.x, pt_ref.y, 0)) << "\n4:::\t" << (cv::Point2f(it2->bound_4.x, it2->bound_4.y)) << (cv::Point3f(pt_ref.x, pt_ref.y + this->squaresize, 0));
      // cout << "\n2:::\t" << (cv::Point2f(it2->bound_2.x, it2->bound_2.y)) << (cv::Point3f(pt_ref.x + this->squaresize, pt_ref.y, 0)) << "\n3:::\t" << (cv::Point2f(it2->bound_3.x, it2->bound_3.y)) << cv::Point3f(pt_ref.x + this->squaresize, pt_ref.y + this->squaresize, 0) << "\n";

      // if (int(pt_ref.x/100000)==0)
      // {
      //   pt_ref.x = 0;
      // }

      // cout << "\n"
      //      << it2->id << "for id" << pt_ref.x << "::" << this->margin<< "\n";
      if (pt_ref.x <= 0)
      {

        pt_ref.y = pt_ref.y + this->margin + this->squaresize;
        pt_ref.x = (this->squaresize + this->margin) * 6;
      }
      old_id = it2->id;
    }
    image_points.push_back(image_point);
    object_points.push_back(object_point);
    // cout << "\nshapes og n vectors " << object_point.size() << "\t\t" << image_point.size();

    // cout << "\nshapes " << object_points.size() << "\t\t" << image_points.size();
  }

  cv::calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs);
  double reproj_error = computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D);
  cout << "\nCalibration error: " << reproj_error << endl;

  // for (std::vector<acquisition_instance_Str>::iterator it = this->vector_detection_set.begin(); it != this->vector_detection_set.end(); ++it)
  // {
  //   vector<cv::Point2f> image_point2;
  //   vector<Point3f> object_point2;

  //   point pt_ref;
  //   pt_ref.x = (this->squaresize + this->margin) * 6;
  //   std::vector<marker_bounds> detection_stub = it->vector_acq;
  //   for (std::vector<marker_bounds>::iterator
  //            it2 = detection_stub.begin();
  //        it2 != detection_stub.end(); ++it2)
  //   {
  //     pt_ref.x = pt_ref.x - (this->squaresize + this->margin);
  //     // cout<< "\n"<<it2->id<<"for id"<<pt_ref.x<<"::"<<pt_ref.y<<"\n";
  //     if (pt_ref.x <= 0.00000001)
  //       pt_ref.x = 0;

  //     image_point2.push_back(cv::Point2f(it2->centre.x, it2->bound_1.y));
  //     // cout << "\n" << it2->id << "for id" << it2->bound_1.x << "::" << it2->bound_1.y << "\n";
  //     object_point2.push_back(cv::Point3f(pt_ref.x, pt_ref.y, 0));

  //     if (pt_ref.x <= 0)
  //     {

  //       pt_ref.y = pt_ref.y + this->margin + this->squaresize;
  //       pt_ref.x = (this->squaresize + this->margin) * 6;
  //     }
  //   }
  //   // image_points2.push_back(image_point2);
  //   // object_points2.push_back(object_point2);
  // }

  // double reproj_error2 = cv::calibrateCamera(object_points2, image_points2, img.size(), K2, D2, rvecs2, tvecs2);
  // cout << "\n Calibration error: " << computeReprojectionErrors(object_points2, image_points2, rvecs2, tvecs2, K2, D2) << endl;
  FileStorage fs(this->filename, FileStorage::APPEND);
  fs << "square-size" << this->squaresize;
  fs << "margin" << this->margin;
  fs << "K" << K;
  fs << "D" << D;
  fs << "Reprojection error" << reproj_error;
  // fs << "K2 - centre" << K2;
  // fs << "D2 - centre" << D2;
  // fs << "Reprojection error - centre" << reproj_error2;

  printf("Done Calibration\n");
}
// ===================================================
//  opencv aprilgrid extraction
//====================================================
class opencv_demo
{
private:
  string window_id;
  apriltag_family_t *tf = nullptr;
  apriltag_detector_t *td = nullptr;
  apriltag_detection_t *det = nullptr;
  //  this->td = apriltag_detector_create();
  marker_pointcloud detection_input;
  int max_iter;

public:
  opencv_demo(/* args */);
  ~opencv_demo();
  opencv_demo(int max_iterations, string storage_file);
  Mat draw_marker(Mat input_frame, Mat Colour, opencv_demo *obj);
  bool set_complete = false;
  bool instance_complete = false;

  /// @brief generates data for the april grid point cloud
  /// @return labelled image or return a matrix of zeros if not available
};

// ===================================================
//  opencv aprilgrid extraction functions
//====================================================
Mat opencv_demo::draw_marker(Mat input_frame, Mat Colour, opencv_demo *obj)
{
  auto gray = input_frame;
  // cout<< "\n input_frame size"<< Colour.size();
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
    this->instance_complete = true;

    for (int i = 0; i < zarray_size(detections); i++)
    {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      // line(Colour, Point(det->p[0][0], det->p[0][1]),
      //      Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
      // line(Colour, Point(det->p[0][0], det->p[0][1]),
      //      Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);

      marker_bounds marker;
      marker.id = det->id;
      marker.bound_1.x = det->p[0][0];
      marker.bound_2.x = det->p[1][0];
      marker.bound_3.x = det->p[2][0];
      marker.bound_4.x = det->p[3][0];
      marker.centre.x = det->c[0];
      marker.bound_1.y = det->p[0][1];
      marker.bound_2.y = det->p[1][1];
      marker.bound_3.y = det->p[2][1];
      marker.bound_4.y = det->p[3][1];
      marker.centre.y = det->c[1];
      local_store.push_back(marker);
#ifdef DEBUG
      if (det->id == 2)
      {
        // cout << "\npt1\t" << det->p[0][0] << "\t::\t" << det->p[0][1] << "\npt2\t" << det->p[1][0]
        //      << "\t::\t" << det->p[1][1] << "\t";
        // cout << "\npt3\t" << det->p[2][0] << "\t::\t" << det->p[2][1] << "\npt4\t" << det->p[3][0]
        //      << "\t::\t" << det->p[3][1] << "\n";
        // line(Colour, Point(det->p[0][0], det->p[0][1]),
        //      Point(det->p[1][0], det->p[1][1]),
        //      Scalar(0xff, 0, 0), 2);
        line(Colour, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[3][0], det->p[3][1]),
             Scalar(0x00, 0xff, 0), 2);
        // line(Colour, Point(det->p[2][0], det->p[2][1]),
        //      Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
      }
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
    // sem_trywait(&thread_synch);
    if (obj->instance_complete && ((this->detection_input.get_set_id() <= obj->detection_input.get_set_id())))
    {
      cout << "trigger condition";
      // obj->instance_complete = false;
      this->detection_input.create_detect_instance(local_store);
      // this->instance_complete = false;
      // usleep(333333);
    }
  
    
    else
    {
      // sem_post(&thread_synch);
      this->instance_complete = false;
    }

    apriltag_detections_destroy(detections);

    int set_num = this->detection_input.get_set_id();

    if (set_num >= this->max_iter)
    {
      this->set_complete = true;
      // cout << "\ncalibration criteria met. running calibration...";
      this->detection_input.setup_calibration(2, 2, input_frame);
    }
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
  this->tf->width_at_border = 8;
  this->tf->total_width = 10;
  marker_pointcloud detection_input;
  tf->reversed_border = false;
  this->td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
}
opencv_demo::opencv_demo(int max_iterations, string storage_file)
{
  this->detection_input.filename = storage_file;
  this->max_iter = max_iterations;
  this->tf = tag36h11_create();
  tf->width_at_border = 8;
  tf->total_width = 10;
  tf->reversed_border = false;
  this->td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
}
opencv_demo::~opencv_demo() { tag36h11_destroy(tf); }
// ===================================================
// MAIN
//====================================================


bool killall = false;
using namespace cv;
using namespace std;
opencv_demo aprilgrid2(10, "rootmonitor.yaml"), aprilgrid(10, "externalmonitor.yaml");

void *capturedata_aprilgrid(void *input)
{

  int in = *(int *)input;
  sem_wait(&thread_release);
  cv::VideoCapture cap2(in);
  Mat frame2, gray2;
  sem_post(&thread_release);
  while (!killall)
  {
    sem_wait(&thread_release);
    if (!aprilgrid2.set_complete)
    {
      

      // cout << "cwebcam2";
      // Capture frame-by-frame

      cap2 >> frame2;

      // cv::flip(frame,frame,1);
      cvtColor(frame2, gray2, COLOR_BGR2GRAY);

      // If the frame is empty, break immediately

      if (frame2.empty())
        break;
      // Write the frame into the file 'outcpp.avi'
      // cap.write(frame);
      string name_cam = "CAM2";
      frame2 = aprilgrid2.draw_marker(gray2, frame2, &aprilgrid);
      imshow(name_cam, frame2);
      // Display the resulting frame
    }
    else
    {
      cap2.release();
      return NULL;
    }
    sem_post(&thread_release);
  }
  cap2.release();
  return NULL;
}

int main(int argc, char **argv)
{
  sem_init(&thread_release, 2, 2);
  sem_init(&thread_synch, 2, 2);
  int i = 2;
  // cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps <<
  // endl;

  // Check if camera opened successfully
  // if (!cap.isOpened() || !cap2.isOpened())
  // {
  //   cout << "Error opening video stream" << endl;
  //   return -1;
  // }

  // Default resolutions of the frame are obtained.The default resolutions are
  // system dependent. int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH); int
  // frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  // double fps = cap.get(CAP_PROP_FPS);
  // cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps << endl;
  pthread_t capture_thread1;
  pthread_create(&capture_thread1, NULL, &capturedata_aprilgrid, &i);
  sem_wait(&thread_release);
  // Start default camera
  cv::VideoCapture cap(0);
  //   // Define the codec and create VideoWriter object.The output is stored in
  //   'outcpp.avi' file. VideoWriter video("outcpp.avi",
  //   cv::VideoWriter::fourcc('M','J','P','G'), 10,
  //   Size(frame_width,frame_height));
  time_t start, end;
  sem_post(&thread_release);
  time(&start);
  while (!killall)
  {
    sem_wait(&thread_release);
    Mat frame, gray;

    if (!aprilgrid.set_complete)
    {

      // cout << "main";
      // cout << aprilgrid2.instance_complete;
      cap >> frame;
      // cv::flip(frame,frame,1);
      cvtColor(frame, gray, COLOR_BGR2GRAY);
      if (frame.empty())
        break;

      frame = aprilgrid.draw_marker(gray, frame, &aprilgrid2);
      imshow("CAM0", frame);
    }

    time(&end);

    if (aprilgrid.set_complete && aprilgrid2.set_complete)
    {
      cap.release();
      break;
    }
    sem_post(&thread_release);
    // Press  ESC on keyboard to  exit
    char c = (char)waitKey(1);
    if (c == 27)
    {
      break;
      killall = true;
    }
  }

  // Closes all the frames

  destroyAllWindows();
  pthread_join(capture_thread1, NULL);

  return 0;
}
