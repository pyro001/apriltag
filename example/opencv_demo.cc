#include <iostream>
#include <iomanip>
#include <opencv4/opencv2/opencv_modules.hpp>
#include"opencv4/opencv2/opencv.hpp"
#include<opencv4/opencv2/videoio.hpp>


extern "C" 
{
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
// #include "Tag25h7.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
}

using namespace std;
using namespace cv;
class opencv_demo
{
private:
     apriltag_family_t  *tf = NULL;
     apriltag_detector_t *td = NULL;
    
public:
    opencv_demo(/* args */);
    ~opencv_demo();
};

opencv_demo::opencv_demo(/* args */)
{
    tf = tag25h9_create();
    tf->width_at_border = 8;
    tf->total_width = 10;
    tf->reversed_border = false;
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);


    
}

opencv_demo::~opencv_demo()
{
    tag25h9_destroy(tf);
}

#include <time.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

    // Start default camera
    cv::VideoCapture cap(0); 
   
    // cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps << endl;

  // Check if camera opened successfully
  if(!cap.isOpened()){
   	cout << "Error opening video stream" << endl;
        return -1;
  }
  
  // Default resolutions of the frame are obtained.The default resolutions are system dependent.
  int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  double frame_counter;
  double fps = cap.get(CAP_PROP_FPS);
  cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps << endl;

    // Number of frames to capture
  int num_frames = 120;
//   // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
//   VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(frame_width,frame_height));
 time_t start, end;
time(&start);
  while(1){
    
    Mat frame;
   
    // Capture frame-by-frame
    cap >> frame;
    frame_counter++;
    // If the frame is empty, break immediately
    if (frame.empty())
      break;
    
    // Write the frame into the file 'outcpp.avi'
    //cap.write(frame);
   
    // Display the resulting frame    
    imshow( "Frame", frame );
    time(&end);
    double seconds = difftime(end,start);
   // cout<<"difftime"<<seconds<<"\n\a";
    if (seconds>=2)
        {
            cout<<"\nframes"<<frame_counter/seconds;
            frame_counter=0;
            time(&start);
            time(&end);
        }
    // Press  ESC on keyboard to  exit
    char c = (char)waitKey(1);
    if( c == 27 ) 
      break;
    
  }

  // When everything done, release the video capture and write object
  cap.release();
//   video.release();

  // Closes all the frames
  destroyAllWindows();
  return 0;

}