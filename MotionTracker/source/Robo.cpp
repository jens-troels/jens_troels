#include <iostream>
#include <vector>
#include <string>
#include <rw/rw.hpp>
#include "rw/math/LinearAlgebra.hpp"
#include <fstream>


Eigen::Matrix<double,2,6> imageJacobian(double u, double v, double z, double f)
{
    Eigen::Matrix<double,2,6> output;
    output(0,0)=-f/z;
    output(0,1)=0;
    output(0,2)=u/z;
    output(0,3)=(u*v)/f;
    output(0,4)=(pow(f,2)+pow(u,2))/f;
    output(0,5)=v;
    output(1,0)=0;
    output(1,1)=-f/z;
    output(1,2)=v/z;
    output(1,3)=(pow(f,2)+pow(v,2))/f;
    output(1,4)=-(u*v)/f;
    output(1,5)=-u;

    return output;
}

Eigen::Matrix<double, 6, 6> computeS(rw::kinematics::State _state, rw::kinematics::Frame* _camFrame, rw::models::Device::Ptr _device){
    rw::math::Transform3D<> baseTtool = _device->baseTframe(_camFrame, _state);
    baseTtool.R().inverse();
    Eigen::MatrixXd result(6,6);
    result<<Eigen::MatrixXd::Zero(6,6);

    for(size_t i=0; i < 3; i++){
        for(size_t j=0; j < 3; j++){
            result(i,j) = baseTtool.R()(i,j);
            result(i+3,j+3) = baseTtool.R()(i,j);
        }
    }

    return result;
}

Eigen::Matrix<double, 2,7> computeZImg(rw::kinematics::State _state, rw::kinematics::Frame* _camFrame, rw::models::Device::Ptr _device, double _u, double _v, double _z, double _f){
    Eigen::Matrix<double,2,6> jImg = imageJacobian(_u, _v, _z, _f);
    Eigen::Matrix<double, 6, 6> Sq = computeS(_state, _camFrame, _device);
    rw::math::Jacobian J = _device->baseJframe(_camFrame, _state);
    Eigen::Matrix<double, 6,7> jEig;
    for(int i= 0; i<6; i++){
        for(int j=0; j<7; j++){
            jEig(i,j)=J(i,j);
        }
    }
    return jImg*Sq*jEig;
}

Eigen::Matrix<double, 7,1> computeDQ(Eigen::MatrixXd _zImg){
    //    std::cout<<zImg*zImg.transpose()<<std::endl;
    Eigen::MatrixXd zImgzImgT = _zImg*_zImg.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(zImgzImgT, Eigen::ComputeThinU | Eigen::ComputeThinV);
//    std::cout << svd.singularValues()<<std::endl;
    Eigen::Vector2d y;
    Eigen::Vector2d dudv(1,1);
    svd.solve(dudv);
    y= svd.solve(dudv);
    //std::cout<<svd.solve(dudv)<<std::endl;
    return _zImg.transpose()*y;
}



int main() {
//    std::string worker = "/home/jeppe/SDU/";
    std::string worker = "/home/jeh/";
            const std::string device_name = "PA10";
            rw::models::Device::Ptr device = _wc->findDevice(device_name);
            if(device == nullptr) {
                RW_THROW("Device " << device_name << " was not found!");
            }

            rw::kinematics::Frame* tool_frame = _wc->findFrame("CameraSim");
            if(tool_frame == nullptr) {
                RW_THROW("Tool frame not found!");
            }

           rw::kinematics::MovableFrame* marker_frame = _wc->findFrame<rw::kinematics::MovableFrame>("Marker");
            if(marker_frame == nullptr) {
                RW_THROW("Tool frame not found!");
            }

            rw::math::Q start(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
            double dt = 1;
            rw::math::Q maxdQ = device->getVelocityLimits();
            State localState = _state;
            rw::kinematics::Frame* base = _wc->findFrame("Robot");
            rw::math::Transform3D<> TW = device->baseTframe(tool_frame,localState);
            rw::math::Transform3D<> CTW = TW * base->getTransform(localState);
            CTW.R().inverse();
            CTW.P().operator *=(-1);
            std::ifstream finput(worker+"/7_semester/finalROVI/SamplePluginPA10/motions/MarkerMotionSlow.txt");
            double x,y,z,roll,pitch,yaw;
            int counter = 0;
            while(!finput.eof())
            {
                if(counter == lineCounter)
                    finput >> x >> y >> z >> roll >> pitch >> yaw;
                counter++;
            }

            Eigen::Matrix<double,4,4> ctwEig;
            for(int i= 0; i<4; i++)
            {
                for(int j=0; j<4; j++)
                {
                    ctwEig(i,j)=CTW(i,j);
                }
            }
            Eigen::Matrix<double,4,1> pointWorld;
            pointWorld << x,y,z,1;
            Eigen::Matrix<double,4,1> pointCamMarker = ctwEig * pointWorld;
            double f = 823;
            double xcm = pointCamMarker(0,0);
            double ycm = pointCamMarker(1,0);
            double zcm = pointCamMarker(2,0);
            double u = (f*xcm)/zcm;
            double v = (f*ycm)/zcm;

            Eigen::MatrixXd zImg = computeZImg(localState, tool_frame, device, u, v, z, f);
            Eigen::Matrix<double, 7,1> dq = computeDQ(zImg,u,v);
    //        std::cout << dq << std::endl;
            std::vector<double> tempQ;
            for(int i = 0; i < dq.size();i++)
            {
                tempQ.push_back(dq(i,0));
            }
            rw::math::Q q_desired(tempQ);
            device->setQ(device->getQ(localState)+q_desired,localState);

            rw::math::Vector3D<> V0(x, y, z);
            rw::math::RPY<> R0(roll, pitch, yaw);
            rw::math::Transform3D<> T0(V0, R0.toRotation3D());
            marker_frame->setTransform(T0,localState);

            lineCounter++;
            getRobWorkStudio()->setState(localState);

            Eigen::Vector3d v(1,2,3);

            std::cout << v.at(0) << std::endl;

    return 0;
}
