// ROS Header
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace cv;
using namespace std;

// Good Point border criteria
int borderLeft = 75, borderRight = 900, borderLower = 265, borderUpper = 85;

// Warning, Danger trigger border criteria
int triggerBorderLeft = 360, triggerBorderRight = 600, triggerBorderLower = 170, triggerBorderUpper = 85;

// Camera verical axis calibration equation
// Camera y pixel vs angle slope equation (Linear Equation) refer to excel file
// [angle = Ay + B]
float aConstant = -0.002776;
float bConstant = 0.752686;

// Camera horizontal axis calibration equation
float aHorizonConstant = 0.0023787;
float bHorizonConstant = -1.144790;

// deltaX is how far the camera has moved in X direction, deltaY is in Y direction, deltaPos is the displacement,
// currentPos is where the robot now, cameraHeight is the height of the camera from the ground
float deltaX;
float deltaY;
float deltaPos;
Point2f currentPos;
float currentLinearMotion;
float cameraHeight = 0.38;
float xDist, xDistC, height, horizonAngle, horizonAngleRad;
vector<Point2f> locationOfInitiation;
vector<int> calculateWithBackwardMotion;
bool addToBackwardMotionVector = false;
int pclCount;

// Pitch and roll check with IMU data used to decide whether to stop the calculation or not 
// because the camera is not in horizontal position when robot is tilting.
float imuPitchLimit = 0.0095;
float imuRollLimit = 0.0095;
float imuPitch, imuRoll;
bool pitchRollWarning;

// Odometry Error Correction
// A temporary workaround on the report of the odometry data from robot that is linearly inaccurate.
float odomErrorCorrection = 1;

// Set the desired point grid
// For 960x540
int desiredX[16] = { 100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850 };
int desiredY[5] = { 100,140,180,220,260 };

// Point accumulation check
// For 960x540
int maxPointInAZone = 10;
vector<int> pointAccumulationCheckIndex[9][3];
int zoneBorderHorizontal[10] = {50,150,250,350,450,550,650,750,850,901};
int zoneBorderVertical[4] = {80,160,240,280};

// Camera calibration for undistortion
	Mat cameraMatrix1 = (Mat_<double>(3,3) << 1.0501385794410448e+03, 0., 960., 0., 1.0501385794410448e+03,
	540., 0., 0., 1.);
    Mat distCoef1 = (Mat_<double>(5,1) << -2.5516666807314803e-01, 1.6890821185525082e-02, 0., 0.,
    1.2221616752393763e-02);


// Declaring some flags
bool pointTrackingFlag = true;
bool calculateTrackpointFlag = false;
bool clearTrackingFlag = false;
bool recenterOffGridPointFlag = false;
bool stopCalculationFlag = false;

// Cosmetic Variable
int lineCosmeticCounter = -1;
bool thereIsAPreviousLine = false;

// Vector of grid location of tracking point that will be deployed.
vector<Point2f> desiredPoint;

// Index of point that is out of bound or invalid and needs to be recentered.
vector<int> pointNeedsRecenter;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  //image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;
  ros::Subscriber subOdom;
  ros::Subscriber imuData;
  ros::Publisher pointCloudPub;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    subOdom = nh_.subscribe("/ypspur_ros/odom", 1, &ImageConverter::cbOdom,this);
    imuData = nh_.subscribe("/imu/data", 1, &ImageConverter::cbImu,this);
    pointCloudPub = nh_.advertise<sensor_msgs::PointCloud2> ("frtcampcl",1);
  }

  void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Assign position from ROS nav msg to global variable
  currentPos = Point2f(odomErrorCorrection*(float)msg->pose.pose.position.x, odomErrorCorrection*(float)msg->pose.pose.position.y);
  currentLinearMotion = (float)msg->twist.twist.linear.x;
}

void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
  // Assign IMU data from sensor msg to global variable
  imuPitch = msg->orientation.y;
  imuRoll = msg->orientation.x;
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

  cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);

  // Push desired (x,y) in vector of desiredPoint
  for (int i = 0; i < 16; i++)
  {
    for (int j = 0; j < 5; j++)
    {
      desiredPoint.push_back(Point2f(desiredX[i], desiredY[j]));
    }
  }

  // PointCloud initialization
  sensor_msgs::PointCloud2 pclMsg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ds(new pcl::PointCloud<pcl::PointXYZ>);
  pc_ds->header.frame_id = "odom";

  TermCriteria terminationCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.02);

  // Matching box size
  Size windowSize(25, 25);

  // Max number of points
  const int maxNumPoints = 80;

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
  float scalingFactor = 0.5;

    while(ros::ok())
    {
      // To slow down the processing causing CPU overload
      //wait.sleep();

      ros::spinOnce();
      // my code here

    for (int i = 0; i < 6; i++)
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

    // Draw outline of ROI and trigger zone
    rectangle (image, Point(borderLeft,borderUpper), Point(borderRight,borderLower), Scalar(0,255,0), 1, 8);
    rectangle (image, Point(triggerBorderLeft,triggerBorderUpper), Point(triggerBorderRight,triggerBorderLower), Scalar(0,0,255), 1, 8);

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

    pitchRollWarning = false;
    // Check pitch and roll if it is changing camera angle
    if (imuPitch > imuPitchLimit)
      {
        calculateTrackpointFlag = false;
        pitchRollWarning = true;
      }
    else if (imuRoll > imuRollLimit)
      {
        calculateTrackpointFlag = false;
        pitchRollWarning = true;
      }
    // Check for keyboard input to stop the calculation
    else if (!stopCalculationFlag)
      calculateTrackpointFlag = true;

    // Show that the calculation is stopped.
    if (!calculateTrackpointFlag)
    {
      if (pitchRollWarning)
        cout << "\r[==CHECK PITCH/ROLL==]";
      else
        cout << "\r[==SCANNING  PAUSED==]";
    }

    // Calculate the distance
    if (calculateTrackpointFlag)
    {
      if (!goodPointsVecTransfer.empty())
      {
      // Add scanning text.
      if (lineCosmeticCounter == 1)
        cout << "\rScanning [X=========]";
      else if (lineCosmeticCounter == 2)
        cout << "\rScanning [=X========]";
      else if (lineCosmeticCounter == 3)
        cout << "\rScanning [==X=======]";
      else if (lineCosmeticCounter == 4)
        cout << "\rScanning [===X======]";
      else if (lineCosmeticCounter == 5)
        cout << "\rScanning [====X=====]";
      else if (lineCosmeticCounter == 6)
        cout << "\rScanning [=====X====]";
      else if (lineCosmeticCounter == 7)
        cout << "\rScanning [======X===]";
      else if (lineCosmeticCounter == 8)
        cout << "\rScanning [=======X==]";
      else if (lineCosmeticCounter == 9)
        cout << "\rScanning [========X=]";
      else if (lineCosmeticCounter == 10)
        {
          cout << "\rScanning [=========X]";
          lineCosmeticCounter = 0;
        }
      lineCosmeticCounter++;
      thereIsAPreviousLine = false;
      }
      else
        cout << "\rScanning [NO TRK PTS]";

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
        for (int d = 0; d < 9; d++)
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

          horizonAngleRad = ((aHorizonConstant*trackingPoints[0][goodPointsVecTransfer[i]].x)+bHorizonConstant);
          horizonAngle = (360*horizonAngleRad)/(2*PI);
          // height calculation (How high is the object)
          height = xDist*tan(aConstant*trackingPoints[1][goodPointsVecTransfer[i]].y + bConstant) + cameraHeight;

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
              if (height <= 1.2 && xDist <= 0.5)
              {
                if (!thereIsAPreviousLine)
                  cout << endl;
                // Highlight the point RED if it is dangerously close
                circle(image, trackingPoints[1][goodPointsVecTransfer[i]], radius, Scalar(0, 0, 255), thickness, lineType);
                cout << "*DANGER: Point " << goodPointsVecTransfer[i] <<": H = " << height <<"m D = " << xDist <<"m. Angle is " << horizonAngle << endl;
                thereIsAPreviousLine = true;
              }
              else if (height <= 1.2 && xDist <= 1.0)
              {
                if (!thereIsAPreviousLine)
                  cout << endl;
                // Highlight the point ORANGE if it is risky.
                circle(image, trackingPoints[1][goodPointsVecTransfer[i]], radius, Scalar(0, 144, 255), thickness, lineType);
                cout << "WARNING: Point " << goodPointsVecTransfer[i] <<": H = " << height <<"m D = " << xDist <<"m. Angle is " << horizonAngle << endl;
                thereIsAPreviousLine = true;
              }
            }
          }
          // Save point to pcl points to be published
          pc_ds->points.push_back (pcl::PointXYZ(tan(horizonAngleRad)*xDist, xDist, height));
        }
      }

      // Check if point accumulation policy is being violated
      for (int d = 0; d < 10; d++)
      {
        for (int e = 0; e < 4; e++)
        {
          if (pointAccumulationCheckIndex[d][e].size() >= maxPointInAZone)
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

      for (int d = 0; d < 10; d++)
      {
        for (int e = 0; e < 4; e++)
        {
          pointAccumulationCheckIndex[d][e].clear();
        }
      }

      clearTrackingFlag = false;
      cout << "\nTracking Point Cleared!" << endl;
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
      cout << "\nTracking Point Deployed!" << endl;
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
      { 
        cout << "\rESC Key pressed, Exiting." << endl;;
        break;
      }
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

    // Set pcl array width and height, covert to sensor_msg and publish the pcl
    pc_ds->height = 1;
    pc_ds->width = pc_ds->points.size();
    pcl::toROSMsg(*pc_ds, pclMsg);
    pclMsg.header.stamp = ros::Time::now();
    pointCloudPub.publish (pclMsg);
    pc_ds->points.clear();

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