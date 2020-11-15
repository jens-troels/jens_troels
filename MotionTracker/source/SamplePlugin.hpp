#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "../build/ui_SamplePlugin.h"

#include <opencv2/opencv.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/rw.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

#include <QPushButton>
#include <QTimer>
#include <QtPlugin>
#include <boost/bind.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <chrono>

using namespace rw::math;
using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;
using namespace cv;
using namespace std;

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void btnPressed();
    void timer();
  
    void stateChangedListener(const rw::kinematics::State& state);

private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    QTimer* _timer;

    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;

    //Robotic functions
    void moveMarker();
    Jacobian computeS();
    Jacobian imageJacobian();
    void maxVelocityCheck(Q &dq);
    double calculatePixelError();

    //Path to SamplePlugin for PA10
    string worker = "/home/jeppe/SDU/7_semester/finalROVI/";
//    string worker = "/home/jeh/7_semester/finalROVI/";
//    string worker = "/your_path_to_sampleplugin/";

    //Choice of using vision and what marker
    bool useVision = true;
    int cvChoice = 1; //1 = color, 2 = Corny

    //Choose marker(1 is color, 3 is corny)
    string markerType = "1";
    int numberOfPoints = 5; //4 corny, 5 color

    //Choose marker type (Slow, Medium, Fast (case sensitive))
    string markerMovement = "Fast";

    //Size of time steps.
    double dT = 0.4;
    double dT = 1.0;
    double tau = 0;

    //Points for Robotics part
    vector<double> trackingPoint = {0,0,0};
    vector<double> trackingPoint0 = {0.15,0.15,0};
    vector<double> trackingPoint1 = {-0.15,0.15,0};
    vector<double> trackingPoint2 = {0.15,-0.15,0};

    //Points (pixel values) to track for color marker
    vector<double> colorPoint0 = {0, 0, 0};
    vector<double> colorPoint1 = {-90, -90, 0};
    vector<double> colorPoint2 = {90, 90, 0};
    vector<double> colorPoint3 = {90, -90, 0};
    vector<double> colorPoint4 = {-90, 90, 0};

    //Points (pixel values) to track for corny marker
    vector<double> cornyPoint0 = {-200, -200, 0};
    vector<double> cornyPoint1 = {200, -200, 0};
    vector<double> cornyPoint2 = {200, 200, 0};
    vector<double> cornyPoint3 = {-200, 200, 0};


    vector<double> desiredPoint{0,0,0,0,0,0};
    vector<double> uv{0,0,0,0,0,0};

    double kappa = 1.2; //Originally used for dampening of singular values

    // Loading frames and device
    Device::Ptr _device;
    Frame* _cameraFrame;
    rw::kinematics::MovableFrame* _markerFrame;
    rw::math::Q _maxVel;

    //Controlling marker
    int currentMPos = 0;
    vector<rw::math::VelocityScrew6D<double>> markerPoints;

    double z = 0.5;
    double f = 823;

    //Used for resetting simulation
    int endOfFile = 0;
    ofstream myData;

    //Vision private variables
    vector<cv::KeyPoint> markerKeypoints;
    cv::Mat markerDescriptors;
    vector< cv::DMatch> markerMatches;
    cv::Mat markerImg;
    cv::Ptr<cv::xfeatures2d::SURF> objectDetector = cv::xfeatures2d::SURF::create( 300 ); // MinHessian = 400;

    //Vision functions
    vector<cv::Point2f> findCircles(Mat& _bin);
    vector<cv::Point2f> detectColorMarker(Mat& img);
    vector<cv::Point2f> detectCornyMarker(Mat& img, vector<cv::KeyPoint>& markerKeypoints, Mat& markerDescriptors, vector< cv::DMatch>& markerMatches, cv::Mat& markerImg);
    vector<double> extractPtsCV();
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
