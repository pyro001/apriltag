#include <iostream>
#include <iomanip>
#include <opencv4/opencv2/opencv_modules.hpp>
#include<opencv4/opencv2/opencv.hpp>
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
     apriltag_family_t  *tf = nullptr;
     apriltag_detector_t *td = nullptr;
     apriltag_detection_t *det= nullptr;
    //  this->td = apriltag_detector_create();

public:
    opencv_demo(/* args */);
    ~opencv_demo();
    Mat draw_marker(Mat input_frame, Mat Colour);
    
};
Mat opencv_demo:: draw_marker(Mat input_frame, Mat Colour)
{
    auto gray= input_frame;
        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };
        
        apriltag_detector_detect(td, &im);
         zarray_t* detections = apriltag_detector_detect(td, &im);
     for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(Colour, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(Colour, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(Colour, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(Colour, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(Colour, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        }
        apriltag_detections_destroy(detections);
        return Colour;
}
opencv_demo::opencv_demo(/* args */)
{
    this->tf = tag25h9_create();
    tf->width_at_border = 8;
    tf->total_width = 10;
    tf->reversed_border = false;
    this->td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);


    
}

opencv_demo::~opencv_demo()
{
    tag25h9_destroy(tf);
}

#include <time.h>

using namespace cv;
using namespace std;
opencv_demo aprilgrid;
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

//   // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
//   VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(frame_width,frame_height));
 time_t start, end;
time(&start);
  while(1){
    
     Mat frame, gray;
   
    // Capture frame-by-frame
    cap >> frame;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    frame_counter++;
    // If the frame is empty, break immediately
    if (frame.empty())
      break;
    
    // Write the frame into the file 'outcpp.avi'
    //cap.write(frame);
   frame=aprilgrid.draw_marker(gray,frame );
    // Display the resulting frame    
   
    time(&end);
//     double seconds = difftime(end,start);
//    // cout<<"difftime"<<seconds<<"\n\a";
//     if (seconds>=2)
//         {
//             cout<<"\nframes"<<frame_counter/seconds;
//             frame_counter=0;
//             time(&start);
//             time(&end);
//         }


//       // Make an image_u8_t header for the Mat data
//         image_u8_t im = { .width = gray.cols,
//             .height = gray.rows,
//             .stride = gray.cols,
//             .buf = gray.data
//         };
//         apriltag_family_t *tf = NULL;
//         tf = tag36h11_create();
//         apriltag_detector_t *td = apriltag_detector_create();
//         apriltag_detector_add_family(td, tf);
//         zarray_t *detections = apriltag_detector_detect(td, &im);

//         if (errno == EAGAIN) {
//             printf("Unable to create the %d threads requested.\n",td->nthreads);
//             exit(-1);
//         }

//         // Draw detection outlines
//         for (int i = 0; i < zarray_size(detections); i++) {
//             apriltag_detection_t *det;
//             zarray_get(detections, i, &det);
//             line(frame, Point(det->p[0][0], det->p[0][1]),
//                      Point(det->p[1][0], det->p[1][1]),
//                      Scalar(0, 0xff, 0), 2);
//             line(frame, Point(det->p[0][0], det->p[0][1]),
//                      Point(det->p[3][0], det->p[3][1]),
//                      Scalar(0, 0, 0xff), 2);
//             line(frame, Point(det->p[1][0], det->p[1][1]),
//                      Point(det->p[2][0], det->p[2][1]),
//                      Scalar(0xff, 0, 0), 2);
//             line(frame, Point(det->p[2][0], det->p[2][1]),
//                      Point(det->p[3][0], det->p[3][1]),
//                      Scalar(0xff, 0, 0), 2);

//             stringstream ss;
//             ss << det->id;
//             String text = ss.str();
//             int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
//             double fontscale = 1.0;
//             int baseline;
//             Size textsize = getTextSize(text, fontface, fontscale, 2,
//                                             &baseline);
//             putText(frame, text, Point(det->c[0]-textsize.width/2,
//                                        det->c[1]+textsize.height/2),
//                     fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
//         }
//  apriltag_detections_destroy(detections);

    imshow("Tag Detections", frame);
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