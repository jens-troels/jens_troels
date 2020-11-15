#include "SamplePlugin.hpp"

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() {

	log().info() << "INITALIZE" << "\n";

        getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(worker+"PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

	// Load Lena image
	Mat im, image;
    im = imread(worker+"SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
        cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
	// Add the texture render to this workcell if there is a frame for texture
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
	}
	// Add the background render to this workcell if there is a frame for texture
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
	}

	// Create a GLFrameGrabber if there is a camera frame with a Camera property set
	Frame* cameraFrame = _wc->findFrame("CameraSim");
	if (cameraFrame != NULL) {
		if (cameraFrame->getPropertyMap().has("Camera")) {
			// Read the dimensions and field of view
			double fovy;
			int width,height;
			std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
			_framegrabber = new GLFrameGrabber(width,height,fovy);
			SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			_framegrabber->init(gldrawer);
		}
	}
        //Check this to see if corny marker is placed correctly
        markerImg = cv::imread(worker+"SamplePluginPA10/markerCorny.png", cv::IMREAD_GRAYSCALE );
        objectDetector->detectAndCompute( markerImg, cv::Mat(), markerKeypoints, markerDescriptors );

         //Load frames, devices and marker motion
        _device = _wc->findDevice("PA10");
        _cameraFrame = _wc->findFrame("Camera");
        _markerFrame = _wc->findFrame<MovableFrame>("Marker");
        _maxVel = _device->getVelocityLimits();
        std::ifstream finput(worker+"SamplePluginPA10/motions/MarkerMotion"+markerMovement+".txt");
        rw::math::VelocityScrew6D<double> points;
        while(!finput.eof())
        {
            finput >> points[0] >> points[1] >> points[2] >> points[3] >> points[4] >> points[5];
            markerPoints.push_back(points);
            endOfFile++;
        }
    }
}


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
	res.data = (uchar*)img.getImageData();
	return res;
}


void SamplePlugin::btnPressed() {
    QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
                //Check to see if correctly loading marker(1 is color, 3 is corny)
        image = ImageLoader::Factory::load(worker+"SamplePluginPA10/markers/Marker"+markerType+".ppm");
		_textureRender->setImage(*image);
        image = ImageLoader::Factory::load(worker+"SamplePluginPA10/backgrounds/color1.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
                //Jens & Jeppe
                //Resets the simulation and open test file

                ostringstream strs1;
                ostringstream strs2;
                strs1 << dT;
                string dTname = strs1.str();
                int vision = useVision;
                strs2 << vision;
                string cv = strs2.str();
                myData.open (worker+"DATA/DATA_dT_"+dTname+"_MARKER"+markerType+"_MOVE_"+markerMovement+"_CV_"+cv+".dat");
                myData << "x\ty\tz\troll\tpitch\tyaw\ttau\tq1\tq2\tq3\tq4\tq5\tq6\tq7\terror" << endl;

                currentMPos = 0;
                Q qstart(7,0, -0.65, 0, 1.8, 0, 0.42, 0);
                _device->setQ(qstart,_state);
                moveMarker();
                getRobWorkStudio()->setState(_state);

	} else if(obj==_btn1){
		log().info() << "Button 1\n";
		// Toggle the timer on and off
		if (!_timer->isActive())
		    _timer->start(100); // run 10 Hz
		else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

void SamplePlugin::timer() {
        //Jeppe & Jens code
        if(endOfFile!=currentMPos)
        {
            //Moves the marker with respect to the extracted marker motions from the given txt files.
            moveMarker();
            getRobWorkStudio()->setState(_state);

            //Extract points from given marker (set by cvChoice and useVision flag in hpp)
            //Computes execution time of routine.

            auto start = std::chrono::system_clock::now();
            vector<double> uvCV = extractPtsCV();
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end-start;
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            tau = elapsed_seconds.count();

            if( useVision ){

                auto start = std::chrono::system_clock::now();
                vector<double> uvCV = extractPtsCV();
                auto end = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = end-start;
                tau = elapsed_seconds.count();


                if(dT-tau <= 0)
                {
                    RW_THROW("Time-step not legal");
                }

                uv.resize(uvCV.size());
                  for (int i = 0; i < numberOfPoints; i++) {
                    uv[i*2]   = uvCV[i*2];
                    uv[i*2 + 1] = uvCV[i*2 +1];
                  }
                desiredPoint.resize(uv.size());

                if(numberOfPoints == 4)
                {
                    desiredPoint[0]=cornyPoint0[0];
                    desiredPoint[1]=cornyPoint0[1];
                    desiredPoint[2]=cornyPoint1[0];
                    desiredPoint[3]=cornyPoint1[1];
                    desiredPoint[4]=cornyPoint2[0];
                    desiredPoint[5]=cornyPoint2[1];
                    desiredPoint[6]=cornyPoint3[0];
                    desiredPoint[7]=cornyPoint3[1];
                }
                if(numberOfPoints == 5)
                {
                    desiredPoint[0]=colorPoint0[0];
                    desiredPoint[1]=colorPoint0[1];
                    desiredPoint[2]=colorPoint1[0];
                    desiredPoint[3]=colorPoint1[1];
                    desiredPoint[4]=colorPoint2[0];
                    desiredPoint[5]=colorPoint2[1];
                    desiredPoint[6]=colorPoint3[0];
                    desiredPoint[7]=colorPoint3[1];
                    desiredPoint[8]=colorPoint4[0];
                    desiredPoint[9]=colorPoint4[1];
                }
          }
        else{
            //From marker to camera transformation
            Transform3D<> cameraTMarker = inverse(_markerFrame->fTf(_cameraFrame,_state));

            //Vector to hold our tracked points in the frame of marker to camera
            vector<Vector3D<>> trackingPoints;
            if(numberOfPoints != 1)
            {
                trackingPoints.push_back(cameraTMarker.R()*Vector3D<>(trackingPoint0[0],trackingPoint0[1],trackingPoint0[2])+cameraTMarker.P());
                trackingPoints.push_back(cameraTMarker.R()*Vector3D<>(trackingPoint1[0],trackingPoint1[1],trackingPoint1[2])+cameraTMarker.P());
                trackingPoints.push_back(cameraTMarker.R()*Vector3D<>(trackingPoint2[0],trackingPoint2[1],trackingPoint2[2])+cameraTMarker.P());
            }

            if(numberOfPoints == 1)
            {
                trackingPoints.push_back(cameraTMarker.R()*Vector3D<>(trackingPoint[0],trackingPoint[1],trackingPoint[2])+cameraTMarker.P());
            }

            trackingPoints.push_back(cameraTMarker.P());

            // Calculate u and v from x and y.
            if (numberOfPoints == 1) {
              uv[0] = ( trackingPoints[0][0] * f ) / z;
              uv[1] = ( trackingPoints[0][1] * f ) / z;
            }
            else {
              for (int i = 0; i < trackingPoints.size()-1; i++) {
                uv[i*2]   = ( trackingPoints[i][0] * f ) / z;
                uv[i*2+1] = ( trackingPoints[i][1] * f ) / z;
              }
            }

            // Places current position according to desired points
            if (currentMPos==0) {
              desiredPoint=uv;
              for (int i = 0; i < numberOfPoints; i++) {
                desiredPoint[i*2]   -= (trackingPoints[trackingPoints.size()-1][0] *f) / z;
                desiredPoint[i*2+1] -= (trackingPoints[trackingPoints.size()-1][1] *f) / z;
                }
              }
            }
          // Calculate delta u and delta v
          Jacobian duv(numberOfPoints*2,1);
          for (int i = 0; i < numberOfPoints; i++) {
            duv(i*2,0) = desiredPoint[i*2]-uv[i*2];
            duv(i*2+1,0) = desiredPoint[i*2+1]-uv[i*2+1];
          }

           // Calculate S(q)
           Jacobian JSq = computeS();

           // Calculate the image Jacobian
           Jacobian JImage = imageJacobian();

           // Calculate J for robot
           Jacobian JRobot = _device->baseJframe(_cameraFrame,_state);

           // Calculate Z_image
           Jacobian zImage = JImage * JSq * JRobot;

           //Use pseudoinverse (uses Moore-Penrose & SVD) to solve the system of linear equations
           Jacobian pseduinverse(LinearAlgebra::pseudoInverse(zImage.e()));
           Jacobian Jdq(pseduinverse.e() * duv.e());
           Q dQ(Jdq.e());

           // Check for velocity limits
           Q currentQ(_device->getQ(_state));
           maxVelocityCheck(dQ);

           // Update the new Q
           _device->setQ(currentQ+dQ, _state);
           currentMPos++;
        }

        if(endOfFile == currentMPos)
            myData.close();


        double error = calculatePixelError();
        Transform3D<> baseTcam = _device->baseTframe(_cameraFrame, _state);
        RPY<> R0(baseTcam.R());
        Vector3D<> V0(baseTcam.P());
        Q printQ(_device->getQ(_state));
        myData << V0[0] << "\t" << V0[1] << "\t" << V0[2] << "\t" << R0[0] << "\t" << R0[1] << "\t" << R0[2] << "\t"
               << tau << "\t" << printQ[0] << "\t" << printQ[1] << "\t" << printQ[2] << "\t" << printQ[3] << "\t"
               << printQ[4] << "\t" << printQ[5] << "\t" << printQ[6] << "\t" << error << endl;


}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}

//Jeppe og Jens funktioner

void SamplePlugin::moveMarker()
{
    Vector3D<> V0(markerPoints[currentMPos][0], markerPoints[currentMPos][1], markerPoints[currentMPos][2]);
    RPY<> R0(markerPoints[currentMPos][3], markerPoints[currentMPos][4], markerPoints[currentMPos][5]);
    Transform3D<> T0(V0, R0.toRotation3D());
    _markerFrame->setTransform(T0,_state);
}

double SamplePlugin::calculatePixelError()
{
    if( useVision ){
        //Extract points from given marker (set by cvChoice and useVision flag in hpp)
        //Computes execution time of routine.

        vector<double> uvCV = extractPtsCV();

        uv.resize(uvCV.size());
          for (int i = 0; i < numberOfPoints; i++) {
            uv[i*2]   = uvCV[i*2];
            uv[i*2 + 1] = uvCV[i*2 +1];
          }
        desiredPoint.resize(uv.size());

        if(numberOfPoints == 4)
        {
            desiredPoint[0]=cornyPoint0[0];
            desiredPoint[1]=cornyPoint0[1];
            desiredPoint[2]=cornyPoint1[0];
            desiredPoint[3]=cornyPoint1[1];
            desiredPoint[4]=cornyPoint2[0];
            desiredPoint[5]=cornyPoint2[1];
            desiredPoint[6]=cornyPoint3[0];
            desiredPoint[7]=cornyPoint3[1];
        }
        if(numberOfPoints == 5)
        {
            desiredPoint[0]=colorPoint0[0];
            desiredPoint[1]=colorPoint0[1];
            desiredPoint[2]=colorPoint1[0];
            desiredPoint[3]=colorPoint1[1];
            desiredPoint[4]=colorPoint2[0];
            desiredPoint[5]=colorPoint2[1];
            desiredPoint[6]=colorPoint3[0];
            desiredPoint[7]=colorPoint3[1];
            desiredPoint[8]=colorPoint4[0];
            desiredPoint[9]=colorPoint4[1];
        }
  }
else{
    //From marker to camera transformation
    Transform3D<> cameraTMarker = inverse(_markerFrame->fTf(_cameraFrame,_state));

    //Vector to hold our tracked points in the frame of marker to camera
    vector<Vector3D<>> trackingPoints;
    if(numberOfPoints != 1)
    {
        trackingPoints.push_back(cameraTMarker.R()*Vector3D<>(trackingPoint0[0],trackingPoint0[1],trackingPoint0[2])+cameraTMarker.P());
        trackingPoints.push_back(cameraTMarker.R()*Vector3D<>(trackingPoint1[0],trackingPoint1[1],trackingPoint1[2])+cameraTMarker.P());
        trackingPoints.push_back(cameraTMarker.R()*Vector3D<>(trackingPoint2[0],trackingPoint2[1],trackingPoint2[2])+cameraTMarker.P());
    }

    trackingPoints.push_back(cameraTMarker.P());

    // Calculate u and v from x and y.
    if (numberOfPoints == 1) {
      uv[0] = ( trackingPoints[0][0] * f ) / z;
      uv[1] = ( trackingPoints[0][1] * f ) / z;
    }
    else {
      for (int i = 0; i < trackingPoints.size()-1; i++) {
        uv[i*2]   = ( trackingPoints[i][0] * f ) / z;
        uv[i*2+1] = ( trackingPoints[i][1] * f ) / z;
      }
    }

    // Places current position according to desired points
    if (currentMPos==0) {
      desiredPoint=uv;
      for (int i = 0; i < numberOfPoints; i++) {
        desiredPoint[i*2]   -= (trackingPoints[trackingPoints.size()-1][0] *f) / z;
        desiredPoint[i*2+1] -= (trackingPoints[trackingPoints.size()-1][1] *f) / z;
        }
      }
    }
  // Calculate delta u and delta v
  Jacobian duv(numberOfPoints*2,1);
  for (int i = 0; i < numberOfPoints; i++) {
    duv(i*2,0) = desiredPoint[i*2]-uv[i*2];
    duv(i*2+1,0) = desiredPoint[i*2+1]-uv[i*2+1];
  }

  double sum = 0;

  for(int i = 0; i < numberOfPoints; i++)
  {
      sum += sqrt(pow(duv(i*2,0),2)+pow(duv(i*2+1,0),2));
  }
  return sum/numberOfPoints;
}


void SamplePlugin::maxVelocityCheck(Q &dQ)
{
    double scaling = numeric_limits<double>::infinity();
    for(int i = 0; i < dQ.size(); i++)
    {
        if(abs(dQ[i]) > _maxVel[i]*(dT-tau))
        {
            if((_maxVel[i]*(dT-tau))/abs(dQ[i])<scaling)
                scaling = (_maxVel[i]*(dT-tau))/abs(dQ[i]);
        }
    }
    if(scaling != numeric_limits<double>::infinity())
    {
        for(int i = 0; i < dQ.size(); i++)
        {
            dQ[i] *= scaling;
        }
    }
}

Jacobian SamplePlugin::imageJacobian()
{
    Jacobian JImage(numberOfPoints*2,6);
    for (int i = 0; i < numberOfPoints; i++) {
      JImage(i*2, 0)   = -(f / z);
      JImage(i*2, 1)   = 0;
      JImage(i*2, 2)   = uv[i*2]/z;
      JImage(i*2, 3)   = uv[i*2]*uv[i*2+1]/f;
      JImage(i*2, 4)   = -(((f*f)+(uv[i*2]*uv[i*2]))/(f));
      JImage(i*2, 5)   = uv[i*2+1];
      JImage(i*2+1, 0) = 0;
      JImage(i*2+1, 1) = -(f / z);
      JImage(i*2+1, 2) = (uv[i*2+1]/z);
      JImage(i*2+1, 3) = (((f*f)+(uv[i*2+1]*uv[i*2+1]))/(f));
      JImage(i*2+1, 4) = -((uv[i*2]*uv[i*2+1])/(f));
      JImage(i*2+1, 5) = -uv[i*2];
    }
    return JImage;
}

Jacobian SamplePlugin::computeS(){
    Transform3D<> baseTcam = inverse(_device->baseTframe(_cameraFrame, _state));
    Jacobian JSq = Jacobian(baseTcam.R());
    return JSq;
}


vector<cv::Point2f> SamplePlugin::findCircles(cv::Mat& _bin){

    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > circleContours;
    std::vector<cv::Vec4i> hierarchy;

    /// Find contours
    findContours(_bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Find circles
    for( int j = 0; j < contours.size(); j++ )
    {
        // Calculate area, perimeter and findCircle
        double area             = abs(contourArea(contours[j], true));
        double perimeter        = arcLength(contours[j], 1);
        double findCircle  = (4 * M_PI * area) / (perimeter*perimeter);

        // Check if the contour belongs to a circle
        if(perimeter > 100 && findCircle > 0.7 && area>2000)
        {
//            std::cout<<"area: "<<area<<" perimeter: "<<perimeter<<" isCircle: "<<findCircle<<std::endl;
            circleContours.push_back(contours.at(j));
        }
    }

    std::vector<cv::Point2f> circleCenters;

    for(int i = 0; i < circleContours.size(); i++) // For every contour
    {
        // Calculate moments and determine center
        cv::Moments circleMoments = cv::moments(circleContours[i], false);
        int x = floor(circleMoments.m10/circleMoments.m00);
        int y = floor(circleMoments.m01/circleMoments.m00);

        circleCenters.push_back(cv::Point2f(x, y));
    }

    return circleCenters;
}

vector<cv::Point2f> SamplePlugin::detectColorMarker(cv::Mat& img){
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat dst_blue;
    cv::inRange(hsv,
                cv::Scalar(110, 30, 30),
                cv::Scalar(125, 255, 255),
                dst_blue);   //HSV
    //cv::imshow("dstblue",dst_blue);
    cv::Mat dst_red;
    cv::inRange(hsv,
                cv::Scalar(0, 150, 30),
                cv::Scalar(15, 255, 255),
                dst_red);   //HSV
    cv::Mat kernel = cv::Mat::ones(9, 9, CV_8U);
    cv::erode(dst_red, dst_red, kernel);    //Opening
    cv::dilate(dst_red, dst_red, kernel);
    cv::erode(dst_blue, dst_blue, kernel);
    cv::dilate(dst_blue, dst_blue, kernel);
    cv::Mat label_test;
    std::vector<cv::Point2f> redPoints =findCircles(dst_red);
    std::vector<cv::Point2f> bluePoints =findCircles(dst_blue);

    //Find center of marker
    std::vector<cv::Point2f> markerPoints;
    double xCenter=0;
    double yCenter=0;
    for(int i=0; i<redPoints.size(); i++){
        xCenter+=redPoints.at(i).x;
        yCenter+=redPoints.at(i).y;
        markerPoints.push_back(redPoints.at(i));
    }

    for(int i=0; i<bluePoints.size(); i++){
        xCenter+=bluePoints.at(i).x;
        yCenter+=bluePoints.at(i).y;
        markerPoints.push_back(bluePoints.at(i));
    }
    xCenter = xCenter/(redPoints.size()+bluePoints.size());
    yCenter = yCenter/(redPoints.size()+bluePoints.size());
    markerPoints.push_back(cv::Point(xCenter,yCenter));


    std::vector<cv::Point2f> markerPointsOut;
    //Check if the marker is detected correctly
    if(markerPoints.size()==5){
        //find the opposite of the red point by distance
        int index;
        double maxDistance=0;
        for (int i=1; i<4; i++){
            if(sqrt(pow(markerPoints.at(i).x-markerPoints.at(0).x,2)+pow(markerPoints.at(i).y-markerPoints.at(0).y,2)) > maxDistance){
                maxDistance = sqrt(pow(markerPoints.at(i).x-markerPoints.at(0).x,2)+pow(markerPoints.at(i).y-markerPoints.at(0).y,2));
                index = i;
            }
        }
        markerPointsOut.push_back(markerPoints.at(4));  //Push back the middle point as marker0
        markerPointsOut.push_back(markerPoints.at(0));  //Push back red point as marker1
        markerPointsOut.push_back(markerPoints.at(index));  //Push back opposite marker as marker2



        markerPoints.erase(markerPoints.begin()+4);
        markerPoints.erase(markerPoints.begin()+index);
        markerPoints.erase(markerPoints.begin());
        //Find the direction of the marker and sort the rest of the points
        double detOfTri = (markerPointsOut.at(2).x * markerPoints.at(0).y - markerPointsOut.at(2).y * markerPoints.at(0).x) - markerPointsOut.at(1).x * (markerPoints.at(0).y - markerPointsOut.at(2).y) + markerPointsOut.at(1).y * (markerPoints.at(0).x - markerPointsOut.at(2).x);
        if(detOfTri >=0){
            markerPointsOut.push_back(markerPoints.at(1));
            markerPointsOut.push_back(markerPoints.at(0));
        }
        else{
            markerPointsOut.push_back(markerPoints.at(0));
            markerPointsOut.push_back(markerPoints.at(1));
        }

    }
    else{
        return markerPoints;
    }
    return markerPointsOut;


}

vector<cv::Point2f> SamplePlugin::detectCornyMarker(cv::Mat& img, vector<cv::KeyPoint>& markerKeypoints, cv::Mat& markerDescriptors, vector< cv::DMatch>& markerMatches, cv::Mat& markerImg){
    // Generate SURF class object and parameters needed for the scene
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( 500 ); // 500 - We want more points on the scene than on object image.
    std::vector<cv::KeyPoint> sceneKeypoints;
    cv::Mat sceneDescriptors;
    std::vector< cv::DMatch> sceneMatches;
    cv::Mat sceneImg;

    // Clone input image to scene, and detect and compute keypoints and descriptors.
    sceneImg = img.clone();
    detector->detectAndCompute( sceneImg, cv::Mat(), sceneKeypoints, sceneDescriptors );

    // Generate Flann Based Matcher
    cv::FlannBasedMatcher matcher;

    // Convert types if not correct
    if(markerDescriptors.type()!=CV_32F) {
        markerDescriptors.convertTo(markerDescriptors, CV_32F);
    }
    if(sceneDescriptors.type()!=CV_32F) {
        sceneDescriptors.convertTo(sceneDescriptors, CV_32F);
    }

    // Match the scene with the corny marker.
    matcher.match( markerDescriptors, sceneDescriptors, sceneMatches );
    double min_dist = 100;

    // Calculate the smallest distance match
    for( int i = 0; i < markerDescriptors.rows; i++ )
    {
        double dist = sceneMatches[i].distance;
        if( dist < min_dist ){
            min_dist = dist;
        }
    }

    // Collect the good matches (Those who are below 3 times the smallest distance)
    std::vector< cv::DMatch > goodMatches;
    for( int i = 0; i < markerDescriptors.rows; i++ ){
        if( sceneMatches[i].distance < 3*min_dist ){
            goodMatches.push_back( sceneMatches[i]);
        }
    }
    sceneMatches = goodMatches;

    // Find the keypoints of both the object and the scene, that belongs to the good matches.
    std::vector<cv::Point2f> markerPoints;
    std::vector<cv::Point2f> scenePoints;
    for( size_t i = 0; i < sceneMatches.size(); i++ )
    {
        markerPoints.push_back( markerKeypoints[ sceneMatches[i].queryIdx ].pt );
        scenePoints.push_back( sceneKeypoints[ sceneMatches[i].trainIdx ].pt );
    }

    // Find Homography based on the object points and scene points using RANSAC.
    cv::Mat H = cv::findHomography( markerPoints, scenePoints, cv::RANSAC );

    // Specify the corners of the object image.
    std::vector<cv::Point2f> markerCorners(4);
    markerCorners[0] = cvPoint(0,0);
    markerCorners[1] = cvPoint( markerImg.cols, 0 );
    markerCorners[2] = cvPoint( markerImg.cols, markerImg.rows );
    markerCorners[3] = cvPoint( 0, markerImg.rows );

    // Find the corners of the marker on the scene, based on the object image and the homography.
    std::vector<cv::Point2f> sceneCorners(4);
    perspectiveTransform( markerCorners, sceneCorners, H);

    // Generate center point and push back reference points (corners + center)
    cv::Point2f markerCenterPoint = (sceneCorners[0] + sceneCorners[1] + sceneCorners[2] +sceneCorners[3]) / 4;
    markerPoints.push_back(markerCenterPoint);

    for(int i = 0; i < sceneCorners.size(); i++) {
        markerPoints.push_back(sceneCorners[i]);
    }

    cv::Mat imgMatches;
//    drawMatches( markerImg, markerKeypoints, sceneImg, sceneKeypoints,
//                 sceneMatches, imgMatches, cv::Scalar::all(-1), cv::Scalar::all(1),
//                 std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
//        cv::imshow("plsWork", imgMatches);
//        while (cv::waitKey() != 27);
//        cv::destroyAllWindows();
    return sceneCorners;
}

vector<double> SamplePlugin::extractPtsCV()
{
    vector<double> uv_points;
    vector<cv::Point2f> temp;
    if (_framegrabber != NULL) {
            // Get the image as a RW image
            Frame* cameraFrame = _wc->findFrame("CameraSim");
            _framegrabber->grab(cameraFrame, _state);
            const Image& image = _framegrabber->getImage();

            // Convert to OpenCV image
            Mat im = toOpenCVImage(image);
            Mat imflip;
            cv::flip(im, imflip, 0);

            // Show in QLabel
            QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p = QPixmap::fromImage(img);

            cvtColor(imflip, imflip, CV_RGB2BGR);
            //cv::imshow("Billede", imflip);

            unsigned int maxW = 400;
            unsigned int maxH = 800;
            _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

            if(cvChoice == 1)
            {
               temp = detectColorMarker(imflip);
               for(int i = 0; i < temp.size(); i++){
                 uv_points.push_back(temp[i].x - (imflip.cols/2));
                 uv_points.push_back(temp[i].y - (imflip.rows/2));
               }
            }
            if(cvChoice == 2)
            {
                temp = detectCornyMarker(imflip, markerKeypoints, markerDescriptors, markerMatches, markerImg);
                for(int i = 0; i < temp.size(); i++){
                  uv_points.push_back(temp[i].x - (imflip.cols/2));
                  uv_points.push_back(temp[i].y - (imflip.rows/2));
                }
            }
            return uv_points;
    }

}
