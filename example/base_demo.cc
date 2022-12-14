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
#include <errno.h>
#include <ctime>
#include <filesystem>
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

// ======================================================
// ===================================================
//  Static Defines
//====================================================
#define APRILGRIDARRAYSIZE 20 // 36
#define DEBUG
#define RESOLUTIONMOD 10
#define JITTER 15
#define delay_inus 3333
#define ENFORCE_SYNCH true
#define DUAL_CAM true
#define MAX_SETS 10

//=======================================================
using namespace std;
struct timespec ts;
sem_t thread_release, thread_synch;
// ===================================================
//  opencv aprilgrid extraction support structures
//====================================================
// Defining a struct called point. It has two float variables x and y. It also has a cv::Point2f variable pt2D. It also has an operator overload function.
struct point
{
  float x = 0;
  float y = 0;
  cv::Point2f pt2D = cv::Point2f(this->x, this->y);
  /// @brief
  /// @param comparator
  /// @return
  bool operator==(const point &comparator)

  {
#ifdef DEBUGV
    cout << "\n operator overload::" << int(this->x / 10) << "\t" << int(comparator.x / 10);
#endif
    return ((int((this->x + JITTER) / RESOLUTIONMOD) >= int(comparator.x / RESOLUTIONMOD) && int((this->x - JITTER) / RESOLUTIONMOD) <= int(comparator.x / RESOLUTIONMOD)) || (int((this->y + JITTER) / RESOLUTIONMOD) >= int(comparator.y / RESOLUTIONMOD) && int((this->y - JITTER) / RESOLUTIONMOD) <= int(comparator.y / RESOLUTIONMOD)));
  }
};

// The above code is defining a struct called marker_bounds. This struct is used to store the four points that make up the marker.
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
  bool compare_id(const marker_bounds &comparator)
  {
    return (this->id == comparator.id);
  }
};


// *|MARKER_CURSOR|*
struct acquisition_instance_Str
{
  std::vector<marker_bounds> vector_acq;
  int id;

  /* data */
};

using namespace cv;
// =================================================
// The camera model for a set of images
//==================================================

class camera_core
{


public:
  /// @brief
  /// @return
  virtual std::vector<vector<cv::Point2f>> get_camera_image_pts()=0;
  /// @brief
  /// @return
  virtual std::vector<vector<cv::Point3f>> get_camera_object_point()=0;
  /// @brief
  /// @return
  virtual cv::Mat get_cameraMatrix()=0;
  /// @brief
  /// @return
  virtual cv::Mat get_distCoeffs()=0;

  camera_core(/* args */)= default;
  ~camera_core()= default;
};

class camera_model: public camera_core
{
private:
  std::vector<cv::Point3f> ideal_object_point;
  std::vector<std::vector<cv::Point3f>> ideal_object_pointcloud;
  std::vector<vector<cv::Point2f>> image_point;
  vector<cv::Mat> rvecs;
  vector<cv::Mat> tvecs;
  vector<cv::Mat> image_chain;
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

public:
  /// @brief
  /// @return
  std::vector<vector<cv::Point2f>> get_camera_image_pts() override;
  /// @brief
  /// @return
  std::vector<vector<cv::Point3f>> get_camera_object_point() override;
  /// @brief
  /// @return
  cv::Mat get_cameraMatrix() override;
  /// @brief
  /// @return
  cv::Mat get_distCoeffs() override;

  camera_model(vector<cv::Point3f> obj,
               std::vector<vector<cv::Point2f>> img,
               vector<cv::Mat> rot,
               vector<cv::Mat> trans,
               vector<cv::Mat> images,
               cv::Mat cameraMatrix,
               cv::Mat distortion

  );

  camera_model(/* args */);
  ~camera_model();
};



std::vector<vector<cv::Point2f>> camera_model::get_camera_image_pts()
{

  return (this->image_point);
}

std::vector<vector<cv::Point3f>> camera_model::get_camera_object_point()
{
  for (int i = 0; i < int(image_point.size()); i++)
  {
    ideal_object_pointcloud.push_back(this->ideal_object_point);
  }

  return (this->ideal_object_pointcloud);
}

cv::Mat camera_model::get_cameraMatrix() { return (this->cameraMatrix); }

cv::Mat camera_model::get_distCoeffs() { return (cameraMatrix); }

camera_model::camera_model(vector<cv::Point3f> obj,
                           std::vector<vector<cv::Point2f>> img,
                           vector<cv::Mat> rot, vector<cv::Mat> trans, vector<cv::Mat> images,
                           cv::Mat cameraMatrix, cv::Mat distortion)
{
  this->ideal_object_point = obj;
  this->image_point = img;
  this->rvecs = rot;
  this->tvecs = trans;
  this->image_chain = images;
  this->cameraMatrix = cameraMatrix;
  this->distCoeffs = distortion;
}

camera_model::camera_model(/* args */) {}

camera_model::~camera_model()
{
}

// ===========================================================
class callibration_core
{
private:
  // /* data */
  virtual double computeReprojectionErrors(const vector<vector<cv::Point3f>> &objectPoints,
                                           const vector<vector<cv::Point2f>> &imagePoints,
                                           const vector<cv::Mat> &rvecs, const vector<cv::Mat> &tvecs,
                                           const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)=0;
  virtual void recompute_geometry(const vector<vector<cv::Point2f>> &imagePoints,
                                  const vector<cv::Mat> &rvecs, const vector<cv::Mat> &tvecs,
                                  const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)=0;

public:
  callibration_core(/* args */)=default;
  virtual camera_model generate_camera_model_prototype() =0;
  virtual void callibrate(float square_size, float space_size, cv::Mat img)=0;

  ~callibration_core(){};
};

// ===================================================
//  point cloud selection
//====================================================
class callibration_mono : public  callibration_core
{
private:
  int set_id;
  vector<cv::Point3f> ideal_object_point;
  std::vector<marker_bounds> instance_vector;
  std::vector<vector<cv::Point2f>> completed_vector;
  vector<cv::Mat> rvecs;
  vector<cv::Mat> image_chain;
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  vector<cv::Mat> tvecs;
  float squaresize = (25.55 / 1000);
  float margin = (06.8 / 1000);
  double computeReprojectionErrors(const vector<vector<cv::Point3f>> &objectPoints,
                                   const vector<vector<cv::Point2f>> &imagePoints,
                                   const vector<cv::Mat> &rvecs, const vector<cv::Mat> &tvecs,
                                   const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs) override;

  const int max_iter = 3;
  /// @brief
  /// @param imagePoints
  /// @param rvecs
  /// @param tvecs
  /// @param cameraMatrix
  /// @param distCoeffs
  void recompute_geometry(const vector<vector<cv::Point2f>> &imagePoints,
                          const vector<cv::Mat> &rvecs, const vector<cv::Mat> &tvecs,
                          const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs) override;
  std::vector<acquisition_instance_Str> vector_detection_set;
  /* data */
public:
  callibration_mono();
  callibration_mono(string file);
  ~callibration_mono();
  std::vector<std::vector<Point3f>> object_point_cloud;

  bool motion_filter(marker_bounds input);
  /// @brief
  /// @param input_vector
  void create_acquisition_instance(acquisition_instance_Str input_vector);
  /// @brief creates a "capture of the current points of interest in the image"
  /// @param input_vector the current point cloud to be set at the latest instance vector
  /// @param enforce_cap  enforce the capture mechanism to activates ; dependencies with the 
  void create_detect_instance(std::vector<marker_bounds> input_vector, bool enforce_cap);
  void callibrate(float square_size, float space_size, cv::Mat img);
  // override;

  camera_model generate_camera_model_prototype();
  int get_set_id() { return set_id; }

  std::string filename;
};

// ===================================================
// fuctions in marker pointcloud class
//====================================================

/// @brief
/// @param input_vector
/// @param enforce_cap
void callibration_mono::create_detect_instance(
    std::vector<marker_bounds> input_vector, bool enforce_cap)

{
  time_t t;      // t passed as argument in function time()
  struct tm *tt; // decalring variable for localtime()
  time(&t);      // passing argument to time()
  tt = localtime(&t);
  this->instance_vector = input_vector;

  marker_bounds manifold = input_vector.front(); // this is sorted by ids get id 2 bound1 and pass to motion filter

  // cout << "\n"<< this->filename << "semaphore entry success result::\t" <<this->set_id<< endl;

  // TODO:  you may add filters here to determine stuff currently left blank
  bool new_image_found = this->motion_filter(manifold);
  if (new_image_found || enforce_cap)
  {
    sem_post(&thread_synch);
    this->set_id++;

    cout << "\nstoring data" << this->filename << "immediate cap value::\t" << enforce_cap << "batch value::\t" << this->set_id << "\t" << asctime(tt);

    // cout << "\n batch number" << this->set_id;
    acquisition_instance_Str acq;
    acq.id = set_id;
    acq.vector_acq = instance_vector;

    create_acquisition_instance(acq);
    usleep(delay_inus);
  }
  else if (!new_image_found)
  {
    ;
    // cout << "\nfailed data" << this->filename << "immediate cap value::\t" << enforce_cap << "batch value::\t" << this->set_id << "\t" << asctime(tt);

    // cout << "\nfailed data" << this->filename << "batch value::\t" << this->set_id +1<< "\t" << asctime(tt);
  }
}
/// @brief
/// @param input_vector

void callibration_mono::create_acquisition_instance(
    acquisition_instance_Str input_vector)
{
  this->vector_detection_set.emplace_back(input_vector);
}

/// @brief


callibration_mono::callibration_mono()
{

  this->filename = "store_default.yaml";
  // cout << "ctor created";
  // object point matrix
  point pt_ref;
  pt_ref.x = (this->squaresize + this->margin) * 6;
  for (int i = 2; i < 38; i++)
  {
    pt_ref.x = pt_ref.x - (this->squaresize + this->margin);
    if (pt_ref.x <= 0.00000001)
      pt_ref.x = 0;
    this->ideal_object_point.push_back(cv::Point3f(pt_ref.x, pt_ref.y, 0));
    this->ideal_object_point.push_back(cv::Point3f(pt_ref.x, pt_ref.y + this->squaresize, 0));
    this->ideal_object_point.push_back(cv::Point3f(pt_ref.x + this->squaresize, pt_ref.y, 0));
    this->ideal_object_point.push_back(cv::Point3f(pt_ref.x + this->squaresize, pt_ref.y + this->squaresize, 0));
    if (pt_ref.x <= 0)
    {

      pt_ref.y = pt_ref.y + this->margin + this->squaresize;
      pt_ref.x = (this->squaresize + this->margin) * 6;
    }
  }
  this->set_id = 0;
}
/// @brief param constructor
/// @param file this is the name of the file that would want the pointcloud to be stored in
callibration_mono::callibration_mono(string file)
{
  this->filename = file;
  this->set_id = 0;

  point pt_ref;
  pt_ref.x = (this->squaresize + this->margin) * 6;
  for (int i = 2; i < 38; i++)
  {
    pt_ref.x = pt_ref.x - (this->squaresize + this->margin);
    if (pt_ref.x <= 0.00000001)
      pt_ref.x = 0;
    this->ideal_object_point.push_back(cv::Point3f(pt_ref.x, pt_ref.y, 0));
    this->ideal_object_point.push_back(cv::Point3f(pt_ref.x, pt_ref.y + this->squaresize, 0));
    this->ideal_object_point.push_back(cv::Point3f(pt_ref.x + this->squaresize, pt_ref.y, 0));
    this->ideal_object_point.push_back(cv::Point3f(pt_ref.x + this->squaresize, pt_ref.y + this->squaresize, 0));
    if (pt_ref.x <= 0)
    {

      pt_ref.y = pt_ref.y + this->margin + this->squaresize;
      pt_ref.x = (this->squaresize + this->margin) * 6;
    }
  }
}
/// @brief 
callibration_mono::~callibration_mono()
{
  this->ideal_object_point.clear();
  this->instance_vector.clear();
  this->completed_vector.clear();
}

// ===================================================
//  motion detection functions
//====================================================
bool callibration_mono::motion_filter(marker_bounds input)
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
      if (mkb.compare_id(input))
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
/// @brief
/// @param objectPoints
/// @param imagePoints
/// @param rvecs
/// @param tvecs
/// @param cameraMatrix
/// @param distCoeffs
/// @return
double callibration_mono::computeReprojectionErrors(const vector<vector<cv::Point3f>> &objectPoints,
                                                    const vector<vector<cv::Point2f>> &imagePoints,
                                                    const vector<cv::Mat> &rvecs, const vector<cv::Mat> &tvecs,
                                                    const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)
{
  vector<Point2f> imagePoints2;
  int i;
  double totalErr = 0, err = 0;
  vector<float> perViewErrors;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i)
  {
    projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    // cout << "size of objective\t" << objectPoints[i].size() << "size of image\t" << imagePoints[i].size();
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
/// @brief
/// @param imagePoints
/// @param rvecs
/// @param tvecs
/// @param cameraMatrix
/// @param distCoeffs

void callibration_mono::recompute_geometry(
    const vector<vector<cv::Point2f>> &imagePoints,
    const vector<cv::Mat> &rvecs, const vector<cv::Mat> &tvecs,
    const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs)
{
  // step0 generate the ideal  object matrix
  point pt_ref;
  vector<Point2f> imagePoints2;
  cout << "entered the completed set function";
  for (int i = 0; i < (int)imagePoints.size(); i++)
  {
    vector<Point2f> temp_image = cv::Mat(imagePoints[i]);
    // step 1 . find the projected points for the full set
    // cout << cv::Mat(this->ideal_object_point).size();
    projectPoints(cv::Mat(this->ideal_object_point), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
    // step2  . find the missing points in the  current instance of the combined point
    // cloud.
    acquisition_instance_Str detection_instance = this->vector_detection_set[i];

    int id = 2;
    int pt_counter = 0;
    // int start_id = 2;
    std::vector<marker_bounds> detection_stub = detection_instance.vector_acq;
    vector<cv::Point2f> correction;
    for (std::vector<marker_bounds>::iterator
             it2 = detection_stub.begin();
         it2 != detection_stub.end(); ++it2)
    {

      // std::cout << "\nid data\t" << id << "\t" << it2->id;
      if (id != it2->id) // this is missing data indicator if this happens use the projection matrix op
      {

        while (id < it2->id)
        {
          correction.push_back(imagePoints2[pt_counter]);
          pt_counter++;
          correction.push_back(imagePoints2[pt_counter]);
          pt_counter++;
          correction.push_back(imagePoints2[pt_counter]);
          pt_counter++;
          correction.push_back(imagePoints2[pt_counter]);
          pt_counter++;
          id++;
        }
      }
      if (id == it2->id) // id found in the original dataset
      {
        // cout << "\ninsert from main";

        correction.push_back(cv::Point2f(it2->bound_2.x, it2->bound_2.y));
        pt_counter++;
        correction.push_back(cv::Point2f(it2->bound_3.x, it2->bound_3.y));
        pt_counter++;
        correction.push_back(cv::Point2f(it2->bound_1.x, it2->bound_1.y));
        pt_counter++;
        correction.push_back(cv::Point2f(it2->bound_4.x, it2->bound_4.y));
        pt_counter++;
        // correction.push_back(temp_image[id - start_id]);
      }
      id++;
    }
    // if stuff is missing from the end
    if (id <= 37)
    {
      // cout << "\nadjusting the tail";
      while (id <= 37)
      {
        correction.push_back(imagePoints2[pt_counter]);
        pt_counter++;
        correction.push_back(imagePoints2[pt_counter]);
        pt_counter++;
        correction.push_back(imagePoints2[pt_counter]);
        pt_counter++;
        correction.push_back(imagePoints2[pt_counter]);
        pt_counter++;
        id++;
      }
      id++;
    }

    // cout << "\n corrected matrix" << i << "\t" << correction.size() << "\n";
    this->completed_vector.push_back(correction);
  }
  // cout << "\ncompleted_vector" << this->completed_vector.size();
}
/// @brief
/// @param square_size
/// @param space_size
/// @param img
void callibration_mono::callibrate(float square_size, float space_size,
                                          cv::Mat img)
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
  this->object_point_cloud = object_points;
  cv::calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs);
  double reproj_error = computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D);
  cout << "\nCalibration error: " << reproj_error << endl;
  this->recompute_geometry(image_points, rvecs, tvecs, K, D);
  for (int i = 0; i < int(this->completed_vector.size()); i++)
  {

    object_points2.push_back(this->ideal_object_point);
  }
  cout << "object size" << this->ideal_object_point.size() << "\t total size" << object_points2.size() << endl;
  cout << "finalised reprojection " << this->filename << this->computeReprojectionErrors(object_points2, this->completed_vector, rvecs, tvecs, K, D);
  this->cameraMatrix = K;
  this->tvecs = tvecs;
  this->distCoeffs = D;
  FileStorage fs(this->filename, FileStorage::APPEND);
  fs << "square-size" << this->squaresize;
  fs << "margin" << this->margin;
  fs << "K" << K;
  fs << "D" << D;
  fs << "Reprojection error" << reproj_error;
  fs << "rotation" << rvecs;
  fs << "translation" << tvecs;
  // fs << "K2 - centre" << K2;
  // fs << "D2 - centre" << D2;
  // fs << "Reprojection error - centre" << reproj_error2;

  printf("Done Calibration\n");
}
/// @brief generate a camera object from pt cloud params in the pt cloud object
/// @return camera_model object. 
camera_model callibration_mono::generate_camera_model_prototype()
{

  // camera_model(vector<cv::Point3f> obj,
  //              std::vector<vector<cv::Point2f>> img,
  //              vector<cv::Mat> rot,
  //              vector<cv::Mat> trans,
  //              vector<cv::Mat> images,
  //              cv::Mat cameraMatrix,
  //              cv::Mat distortion

  //              )
  camera_model temporary_cam(this->ideal_object_point, this->completed_vector, this->rvecs,
                             this->tvecs, this->image_chain, this->cameraMatrix, this->distCoeffs);
  return (temporary_cam);
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

  int max_iter;

public:
  callibration_mono detection_input;
  /// @brief
  opencv_demo(/* args */);
  ~opencv_demo();
  /// @brief
  /// @param max_iterations
  /// @param storage_file
  opencv_demo(int max_iterations, string storage_file);
  /// @brief
  /// @param input_frame
  /// @param Colour
  /// @param obj
  /// @return
  Mat draw_marker(Mat input_frame, Mat Colour, opencv_demo *obj);
  /// @brief
  bool set_complete = false;
  /// @brief
  bool instance_complete = false;
  camera_model cam;
  /// @brief generates data for the april grid point cloud
  /// @return labelled image or return a matrix of zeros if not available
};

// ===================================================
//  opencv aprilgrid extraction functions
//====================================================

Mat opencv_demo::draw_marker(Mat input_frame, Mat Colour, opencv_demo *obj)
{
  auto gray = input_frame;
  // cout << detection_input.filename << input_frame.size();
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
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec = 300000;
    int s = sem_timedwait(&thread_synch, &ts);
    // cout << "\n semaphore result::\t" << s << "\t" << this->detection_input.filename << endl;
    if (s != -1) //&& ((this->detection_input.get_set_id() <= obj->detection_input.get_set_id())))
    {
      // resource acquired but is the other cam fine?
      if ((obj->instance_complete) && (this->detection_input.get_set_id() <= obj->detection_input.get_set_id()))
      {

        // obj->instance_complete = false;
        // the idea here is that if you have lagged in data the latest input image is the correct one
        // bypass the motion filter if that is true.
        this->detection_input.create_detect_instance(local_store, ((this->detection_input.get_set_id() < obj->detection_input.get_set_id()) & ENFORCE_SYNCH));
      }
      // the other cam doesnt have enough to store aka reset resource
      else
      {
        sem_post(&thread_synch);
      }
    }
    else
    {
      sem_post(&thread_synch);
      // cout << "\n"
      //      << this->detection_input.filename << "semaphore entry fail result::\t" << s << "::" << obj->instance_complete << endl;
      this->instance_complete = false;
    }

    apriltag_detections_destroy(detections);

    int set_num = this->detection_input.get_set_id();

    if (set_num >= this->max_iter)
    {
      this->set_complete = true;
      // cout << "\ncalibration criteria met. running calibration...";
      this->detection_input.callibrate(2, 2, input_frame);
      this->cam = detection_input.generate_camera_model_prototype();
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
  callibration_mono detection_input;
  tf->reversed_border = false;
  this->td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
}
opencv_demo::opencv_demo(int max_iterations, string storage_file)
{
  time_t now;
  char the_date[24];

  the_date[0] = '\0';

  now = time(NULL);

  if (now != -1)
  {
    strftime(the_date, 24, "%d_%m_%Y_%T_", gmtime(&now));
  }
  this->detection_input.filename = std::string(the_date) + storage_file;
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
opencv_demo aprilgrid(MAX_SETS, "rootmonitor.yaml"), aprilgrid2(MAX_SETS, "externalmonitor.yaml");

/// @brief
/// @param input
/// @return
void *capturedata_aprilgrid(void *input)
{

  int in = *(int *)input;
  // sem_wait(&thread_release);
  cv::VideoCapture cap2(in);
  cap2.set(cv::CAP_PROP_FRAME_WIDTH, 1280);

  cap2.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  Mat frame2, gray2;

  while (!killall)
  {
    sem_post(&thread_release);
    // sem_wait(&thread_release);
    if (!aprilgrid2.set_complete)
    {
      // usleep(delay_inus);
      // cout << "\ncwebcam2";
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
    // sem_post(&thread_release);
  }
  cap2.release();
  return NULL;
}

int main(int argc, char **argv)
{

  chdir("/home/pyro/repos/apriltag/example/tests");
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
  Mat frame, gray;
  // Default resolutions of the frame are obtained.The default resolutions are
  // system dependent. int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH); int
  // frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  // double fps = cap.get(CAP_PROP_FPS);
  // cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps << endl;
  pthread_t capture_thread1;
  pthread_create(&capture_thread1, NULL, &capturedata_aprilgrid, &i);
  // sem_wait(&thread_release);
  // Start default camera
  cv::VideoCapture cap(0);
  // cv::VideoCaptureProperties(cap, cv::CAP_PROP_FRAME_WIDTH, 1280);

  // cv::VideoCaptureProperties(cap, cv::CAP_PROP_FRAME_HEIGHT, 720);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);

  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  //   // Define the codec and create VideoWriter object.The output is stored in
  //   'outcpp.avi' file. VideoWriter video("outcpp.avi",
  //   cv::VideoWriter::fourcc('M','J','P','G'), 10,
  //   Size(frame_width,frame_height));
  time_t start, end;
  // sem_post(&thread_release);
  time(&start);
  while (!killall)
  {
    sem_wait(&thread_release);

    if (!aprilgrid.set_complete)
    {
      // usleep(delay_inus);
      // cout << "\nmain";
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

    if (aprilgrid.set_complete && (aprilgrid2.set_complete && DUAL_CAM))
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
  cv::Mat K1, K2, D1, D2, R, F, E;
  cv::Vec3d T;
  auto obj = aprilgrid2.cam.get_camera_object_point();
  auto img1 = aprilgrid.cam.get_camera_image_pts();

  auto img2 = aprilgrid2.cam.get_camera_image_pts();
  K1 = aprilgrid.cam.get_cameraMatrix();

  K2 = aprilgrid2.cam.get_cameraMatrix();
  D1 = aprilgrid.cam.get_distCoeffs();
  D2 = aprilgrid2.cam.get_distCoeffs();
  cv::Mat perViewErrors;
  // auto objectset2 = aprilgrid2.detection_input.object_point_cloud;
  // auto objectset1 = aprilgrid.detection_input.object_point_cloud;
  stereoCalibrate(obj, img1, img2, K1, D1, K2, D2, frame.size(), R, T, E, F, perViewErrors);
  cv::FileStorage fs1("test123.yaml", cv::FileStorage::WRITE);
  fs1 << "K1" << aprilgrid2.cam.get_cameraMatrix();
  fs1 << "K2" << aprilgrid.cam.get_cameraMatrix();
  fs1 << "D1" << aprilgrid2.cam.get_distCoeffs();
  fs1 << "D2" << aprilgrid.cam.get_distCoeffs();
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;
  fs1 << "epipolar errors" << perViewErrors;
  cout << "attempting rectification..\n";
  double cam1error=0, cam2error = 0;
  int iter = 0;
  int nRows = perViewErrors.rows;


  for (iter = 0; iter < nRows; iter++)
  {
    

    cam1error += perViewErrors.at<double>(i, 0) * perViewErrors.at<double>(i, 0);
    cam2error += perViewErrors.at<double>(i, 1) * perViewErrors.at<double>(i, 1);

    // cout << endl<<cam1error << "\t" << cam2error << "\t"<<endl;
  }
  cam1error = double(sqrt(cam1error / iter));
  cam2error = double(sqrt(cam2error / iter));
  cout <<endl <<cam1error << "\t" << cam2error << "\t\n\n";
  cv::Mat R1, R2, P1, P2, Q;
  stereoRectify(K1, D1, K2, D2, frame.size(), R, T, R1, R2, P1, P2, Q);
  
  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;
  return 0;
}
