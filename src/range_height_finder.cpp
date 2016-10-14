#include <ros/ros.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <math.h>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

// Good Point border criteria
int borderLeft = 120, borderRight = 520, borderLower = 210, borderUpper = 10;

// Camera y pixel vs angle slope equation (Linear Equation) refer to excel file
// [angle = Ay + B]
float aConstant = -0.00305;
float bConstant = 0.678;

// deltaX is how far the camera has moved in X direction, deltaY is in Y direction, deltaPos is the displacement,
// currentPos is where the robot now, cameraHeight is the hight of the camera from the ground
float deltaX;
float deltaY;
float deltaPos;
Point2f currentPos;
float cameraHeight = 0.312;
float xDist, xDistC, height;
vector<Point2f> locationOfInitiation;

// Distance Error Correction (Parabolic Equation) refer to excel file
// [Error Percentage = cConstant y^2 + dConstant y +eConstant]
// Turn this function on or off with the errorCompensation bool variable
float cConstant = 0.0025;
float dConstant = -0.6445;
float eConstant = 45.775;
bool errorCompensation = true;

// Set the desired point grid
// For 640x480
int desiredX[9] = { 160,200,240,280,320,360,400,440,480 };
int desiredY[5] = { 60,100,140,180,200 };


// Declaring some flags
bool pointTrackingFlag = true;
bool calculateTrackpointFlag = false;
bool clearTrackingFlag = false;
bool recenterOffGridPointFlag = false;

Point2f currentPoint;
vector<Point2f> desiredPoint;

vector<int> pointNeedsRecenter;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber subOdom;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    /*image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);*/
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    subOdom = nh_.subscribe("/odom", 1, &ImageConverter::cbOdom,this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  /*void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }*/

  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Odometry data received
  // Print out odometry data
  /*ROS_INFO("vel: %0.3f, ang_vel: %0.3f\n", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
  ROS_INFO("odometry (x, y, yaw): (%0.3f, %0.3f, %0.3f)\n", 
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      (float)tf::getYaw(msg->pose.pose.orientation));*/

  // Assign position from ROS nav msg to global variable
  currentPos = Point2f(1.13*(float)msg->pose.pose.position.x, 1.13*(float)msg->pose.pose.position.y);
}

  void spin()
  {
    // To slow down the processing causing CPU overload
    //ros::Duration wait(2.0);

    // Open camera
  VideoCapture cap(1);

  // Check whether the camera is open yet
  if (!cap.isOpened())
  {
    cerr << "Unable to open the webcam." << endl;
  }

  // Push desired (x,y) in vector of desiredPoint
  for (int i = 0; i < 9; i++)
  {
    for (int j = 0; j < 5; j++)
    {
      desiredPoint.push_back(Point2f(desiredX[i], desiredY[j]));
    }
  }

  TermCriteria terminationCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.02);

  // Matching box size
  Size windowSize(25, 25);

  // Max number of points
  const int maxNumPoints = 45;

  string windowName = "Height and Range finder";
  namedWindow(windowName, 1);

  Mat prevGrayImage, curGrayImage, image, frame;
  // trackingPoints is the current point.
  vector<Point2f> trackingPoints[2];
  // calculatePoints is the previous point data that will be used for calculation
  vector<Point2f> calculatePoints[2];
  vector<int> goodPointsVecTransfer;

  // Image size scaling factor
  float scalingFactor = 1.0;

    while(ros::ok())
    {
      // To slow down the processing causing CPU overload
      //wait.sleep();

      ros::spinOnce();
      // my code here

    cap >> frame;

    if (frame.empty())
      break;

    resize(frame, frame, Size(), scalingFactor, scalingFactor, INTER_AREA);

    frame.copyTo(image);

    cvtColor(image, curGrayImage, COLOR_BGR2GRAY);

    if (!trackingPoints[0].empty())
    {
      vector<uchar> statusVector;
      vector<float> errorVector;

      if (prevGrayImage.empty())
      {
        curGrayImage.copyTo(prevGrayImage);
      }

      calcOpticalFlowPyrLK(prevGrayImage, curGrayImage, trackingPoints[0], trackingPoints[1], statusVector, errorVector, windowSize, 3, terminationCriteria, 0, 0.001);

      int count = 0;
      int minDist = 7;
      int goodPoints = 0;
      vector<int> goodPointsVec;
      // For showing tracking point number
      stringstream bufferstring;
      string gg;

      for (int i = 0; i < trackingPoints[1].size(); i++)
      {
        if (pointTrackingFlag)
        { // Check if new point are too close.
          if (norm(currentPoint - trackingPoints[1][i]) <= minDist)
          {
            pointTrackingFlag = false;
            continue;
          }
        }

        // Check if the status vector is good if not, skip the code below
        if (!statusVector[i])
        {
                    recenterOffGridPointFlag = true;
                    pointNeedsRecenter.push_back(i);
          continue;
                }
        // Remove tracking point that is out of ROI
        if (trackingPoints[1][i].x < borderLeft || trackingPoints[1][i].x > borderRight)
        {
                    recenterOffGridPointFlag = true;
                    pointNeedsRecenter.push_back(i);
          continue;
                }
        if (trackingPoints[1][i].y < borderUpper || trackingPoints[1][i].y > borderLower)
                {
                    recenterOffGridPointFlag = true;
                    pointNeedsRecenter.push_back(i);
          continue;
                }

        // Point optimization (removed)
        //trackingPoints[1][count++] = trackingPoints[1][i];

        // Track point icon
        int radius = 8;
        int thickness = 2;
        int lineType = 3;
        circle(image, trackingPoints[1][i], radius, Scalar(0, 255, 0), thickness, lineType);

        // Show point number in frame
        bufferstring.str("");
        bufferstring << i;
        gg = bufferstring.str();
        cv::putText(image, gg, Point(trackingPoints[1][i].x + 10,trackingPoints[1][i].y + 10), CV_FONT_NORMAL, 0.5, Scalar(0, 0, 255), 1, 1);

        // Add goodPoints count and save the point index in goodPointsVec for calculation
        goodPoints++;
        goodPointsVec.push_back(i);

      }

      // Point optimization (removed)
      //trackingPoints[1].resize(count);

      // Transfer local vector variable to global vector variable
      goodPointsVecTransfer = goodPointsVec;
    }

    // Calculate the distance
    if (calculateTrackpointFlag)
    {
      // Set float point decimal point
      std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
      std::cout.precision(2);

      for (int i = 0; i < goodPointsVecTransfer.size(); i++)
      {
        // Get deltaX and delta Y.
        deltaX = (float)currentPos.x - (float)locationOfInitiation[goodPointsVecTransfer[i]].x;
       	deltaY = (float)currentPos.y - (float)locationOfInitiation[goodPointsVecTransfer[i]].y;

       	// Calculate displacement that the robot makes.
       	deltaPos = sqrt((deltaX*deltaX)+(deltaY*deltaY));

        // Only calculate when 
        if (deltaPos >= 0.5)
        {
          // xDist calculation (How far is it from the object)
          xDist = (tan(aConstant*calculatePoints[0][goodPointsVecTransfer[i]].y + bConstant)*deltaPos)
            / (tan(aConstant*trackingPoints[1][goodPointsVecTransfer[i]].y + bConstant)
              - tan(aConstant*calculatePoints[0][goodPointsVecTransfer[i]].y + bConstant));

          // height calculation (How high is the object)
          height = xDist*tan(aConstant*trackingPoints[1][goodPointsVecTransfer[i]].y + bConstant) + cameraHeight;

          if (errorCompensation)
          {
            // xDist error compensation
            xDistC = xDist - abs((xDist*(((cConstant*calculatePoints[0][goodPointsVecTransfer[i]].y*calculatePoints[0][goodPointsVecTransfer[i]].y)
              + (dConstant*calculatePoints[0][goodPointsVecTransfer[i]].y) + eConstant) / 100)));
          }

          // Print out the distance and height
          if (xDist < 0 || xDist >= 15)
            cout << "Point " << goodPointsVecTransfer[i] << "(" << calculatePoints[0][goodPointsVecTransfer[i]].x << ","
            << calculatePoints[0][goodPointsVecTransfer[i]].y << ") " << " cannot be calculated. deltaPos is " << deltaPos << endl;
          else
          {
            if (errorCompensation)
            {
              cout << "Point " << goodPointsVecTransfer[i] << "(" << calculatePoints[0][goodPointsVecTransfer[i]].x << ","
                << calculatePoints[0][goodPointsVecTransfer[i]].y << ") height is " << height << "m and it is " << xDistC << "m (" << xDist << "m ) away. deltaPos is " << deltaPos << endl;
            }
            else
            {
              cout << "Point " << goodPointsVecTransfer[i] << "(" << calculatePoints[0][goodPointsVecTransfer[i]].x << ","
                << calculatePoints[0][goodPointsVecTransfer[i]].y << ") height is " << height << "m and it is " << xDist << "m  away. deltaPos is " << deltaPos << endl;
            }
          }
        }
      }
      // Add blank line to separate each iteration
      cout << "End of iteration" << endl;


      calculateTrackpointFlag = false;
    }

    // Reset the tracking point
    if (clearTrackingFlag)
    {
            // Turn off recentering otherwise segmentation fault will occur
            recenterOffGridPointFlag = false;

      trackingPoints[0].clear();
      trackingPoints[1].clear();
      calculatePoints[0].clear();
      calculatePoints[1].clear();
      goodPointsVecTransfer.clear();

      clearTrackingFlag = false;
    }

    // Refining the location of the feature points
    if (pointTrackingFlag && trackingPoints[1].size() < maxNumPoints)
    {
      for (int k = 0; k < desiredPoint.size(); k++)
      {
        vector<Point2f> tempPoints;
        tempPoints.push_back(desiredPoint[k]);

        cornerSubPix(curGrayImage, tempPoints, windowSize, cvSize(-1, -1), terminationCriteria);

        // Add point for calculation.
        calculatePoints[0].push_back(tempPoints[0]);
        trackingPoints[1].push_back(tempPoints[0]);
        locationOfInitiation.push_back(currentPos);
      }

      pointTrackingFlag = false;
    }

    // Tracking point is bad or moved away from border, reset that point.
    if (recenterOffGridPointFlag)
    {
      //cout << "Point recenter ";
      for (int k = 0; k < pointNeedsRecenter.size(); k++)
      {
        vector<Point2f> tempPoints;
        tempPoints.push_back(desiredPoint[pointNeedsRecenter[k]]);

        cornerSubPix(curGrayImage, tempPoints, windowSize, cvSize(-1, -1), terminationCriteria);

        // Remove old, bad tracking point from the vector.
        calculatePoints[0].erase(calculatePoints[0].begin() + pointNeedsRecenter[k]);
        trackingPoints[1].erase(trackingPoints[1].begin() + pointNeedsRecenter[k]);
        locationOfInitiation.erase(locationOfInitiation.begin() + pointNeedsRecenter[k]);

        // Insert new tracking point into the vector.
        calculatePoints[0].insert(calculatePoints[0].begin() + pointNeedsRecenter[k], tempPoints[0]);
        trackingPoints[1].insert(trackingPoints[1].begin() + pointNeedsRecenter[k], tempPoints[0]);
        locationOfInitiation.insert(locationOfInitiation.begin() + pointNeedsRecenter[k], currentPos);

        //cout << pointNeedsRecenter[k] << " ";
        // Presumed the point is recentered and can be cleared. If not, it will be fed back by main function.
        pointNeedsRecenter.erase(pointNeedsRecenter.begin() + k);
      }

      //cout << endl;

      if (pointNeedsRecenter.empty())
            recenterOffGridPointFlag = false;
    }


    imshow(windowName, image);

    char ch = waitKey(10);
    // ESC Check
    if (ch == 27)
  	  break;
  	// Start Calaulation by pressing spacebar
  	if (ch == 32)
  	  calculateTrackpointFlag = true;
  	// Clear all trackpoint by pressing 'c'
  	if (ch == 99)
  	  clearTrackingFlag = true;
  	// Deploy new set of trackpoint by pressing 'd'
  	if (ch == 100)
  	{
  		clearTrackingFlag = true;
  	 	pointTrackingFlag = true;
  	}

    // Update 'previous' to 'current' point vector
    std::swap(trackingPoints[1], trackingPoints[0]);

    // Update previous image to current image
    cv::swap(prevGrayImage, curGrayImage);
    }

  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ic.spin();
  return 0;
}