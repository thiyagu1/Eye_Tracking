#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <fstream>

using namespace cv;
using namespace std;



int main(int argc, char** argv){
    Mat src, src_gray;
    ofstream myfile;
    
    string filename = "/Users/thiyaga/Documents/code/opencv-3.2.0/cpp_basic/thesis/UDP2/Eye.mov";
    VideoCapture cap(filename);
    /// Read the video
   // VideoCapture cap(1); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    
    for(;;)
    {
        Mat frame;
        int x,y;
        //myfile.open ("/Users/thiyaga/Desktop/Position.txt", std::ofstream::out | std::ofstream::app);
        myfile.open ("/Users/thiyaga/Documents/code/opencv-3.2.0/cpp_basic/thesis/UDP2/Position.txt", std::ofstream::out);
        
        
        cap >> src; // get a new frame from camera
        /// Convert it to gray
        
        // For Gaze
        cvtColor( src, src_gray, CV_BGR2GRAY );
        
        /// Reduce the noise so we avoid false circle detection
        GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
        
        vector<Vec3f> circles;
        
        /// Apply the Hough Transform to find the circles
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 60, 20, 20, 70 );
        // cout<<"Circle Size ="<<circles.size()<<endl;
        /// Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( src, center, 4, Scalar(255,0,255), -1, 8, 0 );
            x = center.x ;
            y = center.y ;
            //          r = radius;
            
            //          pass(x, y, r);
            cout<<"X="<<x<<"\t"<<"Y="<<y<<endl;//"          "<<"radius="<<radius<<endl;
            // circle outline
            circle( src, center, radius, Scalar(255,255,255), 3, 8, 0 );
            myfile <<"X="<<x<< " " <<"Y=" <<y << endl;
            
            
            
        }
        myfile.close();
        /// Show your results
        namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
        imshow( "Hough Circle Transform Demo", src );
        if (waitKey(1) == 'q') {
            break;
        }
    }
    
    return 0;
}
