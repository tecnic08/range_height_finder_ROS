// ROS Header
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// OpenCV Header
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
// Standard C++ Header
#include <iostream>
#include <math.h>

float PI = 3.141592653589793;

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

// Good Point border criteria
int borderLeft = 110, borderRight = 530, borderLower = 220, borderUpper = 75;

// Warning, Danger trigger border criteria
int triggerBorderLeft = 200, triggerBorderRight = 440, triggerBorderLower = 160, triggerBorderUpper = 70;

// Camera verical axis calibration equation
// Camera y pixel vs angle slope equation (Linear Equation) refer to excel file
// [angle = Ay + B]
/*float aConstant = -0.00305;
float bConstant = 0.678;*/
float aConstant = -0.00400673;
float bConstant = 0.97309;

// Camera horizontal axis calibration equation
float aHorizonConstant = 0.0051887;
float bHorizonConstant = -1.695288;

// deltaX is how far the camera has moved in X direction, deltaY is in Y direction, deltaPos is the displacement,
// currentPos is where the robot now, cameraHeight is the height of the camera from the ground
float deltaX;
float deltaY;
float deltaPos;
Point2f currentPos;
float currentLinearMotion;
float cameraHeight = 0.312;
float xDist, xDistC, height, horizonAngle;
vector<Point2f> locationOfInitiation;
vector<int> calculateWithBackwardMotion;
bool addToBackwardMotionVector = false;

// Joint information array: used to decide whether to stop the calculation or not 
// because the camera is not in horizontal position when robot is tilting.
float jointPosition[4];

// Distance Error Correction !!REMOVED!! (Parabolic Equation) refer to excel file
// [Error Percentage = cConstant y^2 + dConstant y +eConstant]
// Turn this function on or off with the errorCompensation bool variable
/*
float cConstant = 0.0025;
float dConstant = -0.6445;
float eConstant = 45.775;
bool errorCompensation = false;*/

// Odometry Error Correction
// A temporary workaround on the report of the odometry data from robot that is linearly inaccurate.
float odomErrorCorrection = 1;

// Set the desired point grid
// For 640x480
int desiredX[13] = { 140,170,200,230,260,290,320,350,380,410,440,470,500 };
int desiredY[5] = { 100,130,160,190,210 };

// Point accumulation check
vector<int> pointAccumulationCheckIndex[6][3];
int zoneBorderHorizontal[7] = {110,180,250,320,390,460,531};
int zoneBorderVertical[4] = {70,120,170,221};

// Camera calibration for undistortion
Mat cameraMatrix1 = (Mat_<double>(3,3) << 3.7562890829701979e+02, 0., 320.,
    0., 3.7562890829701979e+02, 240.,
    0., 0., 1.);
    Mat distCoef1 = (Mat_<double>(5,1) << -3.7568807123087655e-01, 1.4804966706210765e-01, 0., 0.,
    -2.6655460202595321e-02);

// Declaring some flags
bool pointTrackingFlag = true;
bool calculateTrackpointFlag = false;
bool clearTrackingFlag = false;
bool recenterOffGridPointFlag = false;
bool stopCalculationFlag = false;

Point2f currentPoint;

// Vector of grid location of tracking point that will be deployed.
vector<Point2f> desiredPoint;

// Index of point that is out of bound or invalid and needs to be recentered.
vector<int> pointNeedsRecenter;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber subOdom;
  ros::Subscriber subJoint;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    /*image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);*/
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    subOdom = nh_.subscribe("/ypspur_ros/odom", 1, &ImageConverter::cbOdom,this);
    subJoint = nh_.subscribe("/ypspur_ros/joint", 1, &ImageConverter::cbJoint,this);

    //cv::namedWindow(OPENCV_WINDOW);
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
  currentPos = Point2f(odomErrorCorrection*(float)msg->pose.pose.position.x, odomErrorCorrection*(float)msg->pose.pose.position.y);
  currentLinearMotion = (float)msg->twist.twist.linear.x;
}

void cbJoint(const sensor_msgs::JointState::ConstPtr &msg)
{
  // Joint angle received
  // Print out joint data
  /*ROS_INFO("joint_angle: %0.3f, %0.3f, %0.3f, %0.3f\n", 
      msg->position[0], msg->position[1], msg->position[2], msg->position[3]);
  ROS_INFO("joint_vel: %0.3f, %0.3f, %0.3f, %0.3f\n", 
      msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->velocity[3]);*/

  // Assign joint position from ROS nav msg to global variable
  jointPosition[0] = msg->position[0];
  jointPosition[1] = msg->position[1];
  jointPosition[2] = msg->position[2];
  jointPosition[3] = msg->position[3];

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
  for (int i = 0; i < 13; i++)
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
  const int maxNumPoints = 65;

  string windowName = "Height and Range finder";
  namedWindow(windowName, 1);

  Mat prevGrayImage, curGrayImage, image, frame, originalDistortedFrame;
  Mat map1, map2;
  Size imageSize;
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

    cap >> originalDistortedFrame;

    if (originalDistortedFrame.empty())
      break;

    // Get input frame size for undistortion
    imageSize = originalDistortedFrame.size();

    // Undistort the input image without cropping
    initUndistortRectifyMap(
                cameraMatrix1, distCoef1, Mat(),
                getOptimalNewCameraMatrix(cameraMatrix1, distCoef1, imageSize, 1, imageSize, 0), imageSize,
                CV_16SC2, map1, map2);

    remap(originalDistortedFrame, frame, map1, map2, INTER_LINEAR);
    
    // Resize the frame if needed
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

      // loop to highlight the point and check the quality of point
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

        // Track point icon
        int radius = 5;
        int thickness = 2;
        int lineType = 3;
        // Circle the point gray to show that this point is tracked, but not being calculated yet.
        circle(image, trackingPoints[1][i], radius, Scalar(128, 128, 128), thickness, lineType);

        // Show point number in frame
        bufferstring.str("");
        bufferstring << i;
        gg = bufferstring.str();
        putText(image, gg, Point(trackingPoints[1][i].x + 10,trackingPoints[1][i].y + 10), CV_FONT_NORMAL, 0.5, Scalar(0, 0, 255), 1, 1);

        // Add goodPoints count and save the point index in goodPointsVec for calculation
        goodPoints++;
        goodPointsVec.push_back(i);

      }


      // Transfer local vector variable to global vector variable
      goodPointsVecTransfer = goodPointsVec;
    }
      /* Joint check KENAF
          _________
         ||        ||
         || 3    2 ||
          |        |
         || 0    1 ||
         ||________||
      */
    // Check joint position if it is changing camera angle
    if (jointPosition[0] < -2.95 || jointPosition[0] > 0.20)
      calculateTrackpointFlag = false;
    else if (jointPosition[1] > 2.95 || jointPosition[1] < -0.20)
      calculateTrackpointFlag = false;
    else if (jointPosition[2] < -2.85 || jointPosition[2] > 0.15)
        calculateTrackpointFlag = false;
    else if (jointPosition[3] > 2.85 || jointPosition[3] < -0.15)
      calculateTrackpointFlag = false;
    // Check for keyboard input to stop the calculation
    else if (!stopCalculationFlag)
      calculateTrackpointFlag = true;

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

        //deltaPos = norm(currentPos - locationOfInitiation[goodPointsVecTransfer[i]]);

        // Push index of point that needs to be calculated as backward motion into a vector.
        if (deltaPos >= 0.29 && deltaPos <= 0.3)
        {
          if (currentLinearMotion < -0.005)
         {
            // If the vector is empty, push the index of the point in.
            if (calculateWithBackwardMotion.empty())
            {
              calculateWithBackwardMotion.push_back(goodPointsVecTransfer[i]);
              //cout << "First point added to backward motion" << endl;
            }
            // If not empty, check that this index is not duplicated.
            else
            {
              // Check if the point is already in the vector
              for(int k = 0; k < calculateWithBackwardMotion.size(); k++)
              {
                if (calculateWithBackwardMotion[k] == goodPointsVecTransfer[i])
                {
                  addToBackwardMotionVector = false;
                  break;
                }
                else
                  addToBackwardMotionVector = true;
              }
            }
          }
          // Remove from backward motion if that point is now in forward motion
          else if (currentLinearMotion > 0.005)
          {
            if(!calculateWithBackwardMotion.empty())
            {
              for(int k = 0; k < calculateWithBackwardMotion.size(); k++)
              {
                if (calculateWithBackwardMotion[k] == goodPointsVecTransfer[i])
                {
                  calculateWithBackwardMotion.erase(calculateWithBackwardMotion.begin() + k);
                  //cout << "Point changed to forward motion " << goodPointsVecTransfer[i] << endl;
                }
              }
            }
          }
        }
        // If it passed all criteria, time to push the index into the vector
        if (addToBackwardMotionVector)
        {
          calculateWithBackwardMotion.push_back(goodPointsVecTransfer[i]);
          addToBackwardMotionVector = false;
          //cout << "Added to backward motion [" << goodPointsVecTransfer[i] << "]" << endl;
        } 

        // Point Accumulation Check by categorize point into zone.
        for (int d = 0; d < 6; d++)
        {
          if (trackingPoints[1][goodPointsVecTransfer[i]].x >= zoneBorderHorizontal[d]
              && trackingPoints[1][goodPointsVecTransfer[i]].x < zoneBorderHorizontal[d+1])
          {
            for (int e = 0; e < 3; e++)
            {
              if (trackingPoints[1][goodPointsVecTransfer[i]].y >= zoneBorderVertical[e]
              && trackingPoints[1][goodPointsVecTransfer[i]].y < zoneBorderVertical[e+1])
              {
                pointAccumulationCheckIndex[d][e].push_back(goodPointsVecTransfer[i]);
                break;
              }
            } 
          }
        }

        // Calculation Part
        if (deltaPos >= 0.3)
        {
          int radius = 5;
          int thickness = 2;
          int lineType = 3;

          // Change circle color to green to show that this point is under control
          circle(image, trackingPoints[1][goodPointsVecTransfer[i]], radius, Scalar(0, 255, 0), thickness, lineType);

          // xDist calculation (How far is it from the object)
          xDist = (tan(aConstant*calculatePoints[0][goodPointsVecTransfer[i]].y + bConstant)*deltaPos)
            / (tan(aConstant*trackingPoints[1][goodPointsVecTransfer[i]].y + bConstant)
            - tan(aConstant*calculatePoints[0][goodPointsVecTransfer[i]].y + bConstant));

          // Check for vector with backward motion.
          for (int b = 0; b < calculateWithBackwardMotion.size(); b++)
          {
            // Calculate this point as backward motion if current index matches the index in calculateWithBackwardMotion
            if (goodPointsVecTransfer[i] == calculateWithBackwardMotion[b])            
              xDist = -xDist;
          }

          horizonAngle = (360*((aHorizonConstant*trackingPoints[0][goodPointsVecTransfer[i]].x)+bHorizonConstant))/(2*PI);

          // height calculation (How high is the object)
          height = xDist*tan(aConstant*trackingPoints[1][goodPointsVecTransfer[i]].y + bConstant) + cameraHeight;

          // Error compensation (removed because image is already undistorted)
          /*if (errorCompensation)
          {
            // xDist error compensation
            xDistC = xDist - abs((xDist*(((cConstant*calculatePoints[0][goodPointsVecTransfer[i]].y*calculatePoints[0][goodPointsVecTransfer[i]].y)
              + (dConstant*calculatePoints[0][goodPointsVecTransfer[i]].y) + eConstant) / 100)));
          }*/

          // Print out the distance and height This one print every point out.
          /*if (xDist < 0 || xDist >= 15)
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
          }*/

          // Miscalculation check.
          if (height <= 0 || xDist <= 0)
              continue;

          // Hightlight the point that is risky and print out the information
          radius = 8;
          thickness = 2;
          lineType = 3;
          if (trackingPoints[1][goodPointsVecTransfer[i]].x >= triggerBorderLeft && trackingPoints[1][goodPointsVecTransfer[i]].x <= triggerBorderRight)
          {
            if(trackingPoints[1][goodPointsVecTransfer[i]].y <= triggerBorderLower && trackingPoints[1][goodPointsVecTransfer[i]].y >= triggerBorderUpper)
            {
              if (height <= 1.00 && xDistC <= 1.0)
              {
                // Highlight the point RED if it is dangerously close
                circle(image, trackingPoints[1][goodPointsVecTransfer[i]], radius, Scalar(0, 0, 255), thickness, lineType);
                cout << "*DANGER: Point " << goodPointsVecTransfer[i] <<": H = " << height <<"m D = " << xDist <<"m. Angle is " << horizonAngle << endl;
              }
              else if (height <= 1.0)
              {
                // Highlight the point ORANGE if it is risky.
                circle(image, trackingPoints[1][goodPointsVecTransfer[i]], radius, Scalar(0, 144, 255), thickness, lineType);
                cout << "WARNING: Point " << goodPointsVecTransfer[i] <<": H = " << height <<"m D = " << xDist <<"m. Angle is " << horizonAngle << endl;
              }
            }
          }
        }
      }

      // Check if point accumulation policy is being violated
      for (int d = 0; d < 6; d++)
      {
        for (int e = 0; e < 3; e++)
        {
          if (pointAccumulationCheckIndex[d][e].size() >= 10)
          {
            for (int f = 0; f < pointAccumulationCheckIndex[d][e].size(); f++)
            {
              pointNeedsRecenter.push_back(pointAccumulationCheckIndex[d][e][f]);
            }
            recenterOffGridPointFlag = true;
          }
          pointAccumulationCheckIndex[d][e].clear();
        }
      }

      // Add a line to separate each iteration
      cout << "End of iteration" << endl;

      // Remove to make the calculation autonomous.
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
      locationOfInitiation.clear();
      calculateWithBackwardMotion.clear();
      addToBackwardMotionVector = false;

      for (int d = 0; d < 6; d++)
      {
        for (int e = 0; e < 3; e++)
        {
          pointAccumulationCheckIndex[d][e].clear();
        }
      }

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

        // Remove backward motion indicator
          for (int b = 0; b<calculateWithBackwardMotion.size(); b++)
          {
          // Calculate this point as backward motion if current index matches the index in calculateWithBackwardMotion
            if (pointNeedsRecenter[k] == calculateWithBackwardMotion[b])
            {
              calculateWithBackwardMotion.erase(calculateWithBackwardMotion.begin() + b);
              //cout << "Removed from backward motion" << pointNeedsRecenter[k] << endl;
            }
          }

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
  	// Start/stop Calaulation by pressing spacebar
  	if (ch == 32)
  	  stopCalculationFlag = !stopCalculationFlag;
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