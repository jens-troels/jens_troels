//
// Created by jeppe on 4/21/19.
// fy jeppe, MM/DD/YY er føj

#include "Rrt.h"

Rrt::Rrt() {

}

Rrt::Rrt(Device::Ptr device, CollisionDetector::Ptr col_detector, State state, WorkCell::Ptr wc, Eigen::Matrix<double, 6,6> C):
    _device(device), _col_detector(col_detector), _state(state), _wc(wc), _C(C)
{
    Math::seed();
    _disp << 0,0,0,0,0,0;
}

Tree* Rrt::task_constrained_rrt(Tree* init_tree, double step_size, int nodes) {
    _disp = task_coordinates(init_tree->get_root()->_q, init_tree->get_root()->_q); //Outcomment this line if you wish constraints to exactly resemble taskframe coords.
    cout << _disp.transpose() << endl;
    for(int i=0; i<nodes; i++){
        Q qrand     = random_config();
        Node qRand(qrand);
        Node* qNear = init_tree->find_nearest_neighbor(qRand);
        Q qdir      = (qrand-qNear->_q)/(qrand-qNear->_q).norm2();
        Q qs        = qNear->_q+qdir*step_size;
        Node* qS    = new Node(qs);
        if(RGD_new_config(qS, qNear)){
            init_tree->add_node(qS);
            init_tree->add_edge(qNear, qS);
        }
    }
    return init_tree;
}

bool Rrt::in_collision(Q q)
{
    _device->setQ(q, _state);
    return _col_detector->inCollision(_state);
}

Q Rrt::random_config()
{
    Q qMin = _device->getBounds().first;
    Q qMax = _device->getBounds().second;
    Q qrand = Math::ranQ(qMin,qMax);
    return qrand;
}



Eigen::Matrix<double,6,1> Rrt::computeTaskError(Q qs, Q qnear)
{
    _device->setQ(qs,_state);

    Eigen::Matrix<double,6,1> dx = task_coordinates(qs, qnear);

    dx = _C * (dx - _disp);

    return dx;

}

Eigen::Matrix<double,6,1> Rrt::task_coordinates(Q qs, Q qnear)
{
    Frame* taskFrame = _wc->findFrame("TaskFrame");
    Frame* endFrame = _wc->findFrame("UR5.TCP");
    Transform3D<> TTaskEnd = taskFrame->fTf(endFrame, _state); //End-effector as seen in task frame

    //Retrieve position and orientation:
    Rotation3D<> R_TaskEnd = TTaskEnd.R();
    RPY<> RPY_diff(R_TaskEnd);
    Vector3D<> disp_diff = TTaskEnd.P();

    Eigen::Matrix<double,6,1> dx;
    dx << disp_diff[0], disp_diff[1], disp_diff[2], RPY_diff[2], RPY_diff[1], RPY_diff[0]; //[XYZ YPR]

    return dx;
}

bool Rrt::RGD_new_config(Node* qS, Node* qNear) {
    Eigen::Matrix<double,6,1> dx_err = computeTaskError(qS->_q, qNear->_q);

    int i = 0, j = 0;
    int I = 1000, J = 100;
    double dmax = 0.05 / 10, eps = EPS;

    if(dx_err.norm() < eps && !in_collision(qS->_q)) //If we are very lucky...
    {
        return true;
    }

    while(i < I && j < J && dx_err.norm() > eps)
    {
        i++; j++;
        Q qrand = random_config();
        Q qs_temp = qS->_q + ((qS->_q - qrand) / (qS->_q - qrand).norm2()) * dmax;

        Eigen::Matrix<double,6,1> dx_err_temp = computeTaskError(qs_temp, qNear->_q);
        if(dx_err_temp.norm() <= dx_err.norm())
        {
            j = 0;
            qS->_q = qs_temp;
            dx_err = dx_err_temp;
        }
        if(dx_err.norm() <= eps)
        {
            if(in_collision(qS->_q))
                return false;

            else
                return true;

        }
    }
    return false;
}

bool Rrt::TS_new_config(Node *_qS, Node *_qNear) {
    Frame* toolFrame = _wc->findFrame("UR5.TCP");
    _device->setQ(_qNear->_q, _state);
    Jacobian J = _device->baseJframe(toolFrame, _state);
    Q dQ = _qS->_q - _qNear->_q;
    Jacobian JInv(LinearAlgebra::pseudoInverse(J.e()));
    Eigen::Matrix<double,6,1> dQ_e = dQ.e() - JInv.e() * _C * J.e() * dQ.e();
    Q dQ_(dQ_e);
    _qS->_q = _qNear->_q + dQ_;
    return RGD_new_config(_qS, _qNear);
}

bool Rrt::FR_new_config(Node *_qS, Node *_qNear) {
    Q qr = _qS->_q;
    Eigen::Matrix<double, 6, 1> dx = computeTaskError(_qS->_q, _qNear->_q);
    while (dx.norm() > EPS)
    {
        retract_config(_qS, dx);
        if ((_qS->_q.e()-qr.e()).norm() > (qr.e()-_qNear->_q.e()).norm())
            return false;
        dx = computeTaskError(_qS->_q, _qNear->_q);
    }
    if(in_collision(_qS->_q))
        return false;
    return true;

}

void Rrt::retract_config(Node *_qS, Eigen::Matrix<double, 6, 1> _dx) {
    Frame* toolFrame = _wc->findFrame("UR5.TCP");
    _device->setQ(_qS->_q, _state);
    Jacobian J = _device->baseJframe(toolFrame, _state);
    Jacobian JInv(LinearAlgebra::pseudoInverse(J.e()));
    Eigen::Matrix<double, 6, 1> dq = JInv.e() * _dx;
    Q dQ(dq);
    _qS->_q = _qS->_q - dQ;
}

bool Rrt::local_planner(Q q1, Q q2)
{
    double eps = 0.01;
    Q dq = q2 - q1;
    Q dq_uv = dq / dq.norm2(); //unit vector
    int n = ceil((dq.norm2() / eps)) -1;

    for(int i = 1; i <= n; i++)
    {
        Q qi = q1 + dq_uv * i * eps;
        _device->setQ(qi,_state);
        bool collision = _col_detector->inCollision(_state);

        if(collision)
            return false;
    }
    return true;
}


vector<Q> Rrt::CBiRRT(Q qstart, Q qgoal, Eigen::Matrix<double,6,6> C, int constrained_method)
{
    _C              = C;
    Node* nstart    = new Node(qstart);
    Node* ngoal     = new Node(qgoal);
    Tree* Ta        = new Tree(nstart);
    Tree* Tb        = new Tree(ngoal);
    Tree* Tdummy    = new Tree;
    _device->setQ(qstart, _state);
    _disp = task_coordinates(qstart, qstart);


    cout << "initial q: " << qstart << endl;
    cout << "initial displacement: " << _disp.transpose() << endl;

    vector<Q> path;

    int numberOfIterations = 5000;

    double stepsize = 0.05;

    for(int i = 0; i < numberOfIterations; i++)
    {
        cout << "CBiRRT iteration: " << i << endl;

        Node* qrand = new Node( random_config() );
        Node* qa_near = Ta->find_nearest_neighbor(*qrand);

        Node* qa_reached = constrained_extend(Ta, qa_near, qrand, stepsize, constrained_method);
        Node* qb_near = Tb->find_nearest_neighbor(*qa_reached);
        Node* qb_reached = constrained_extend(Tb, qb_near, qa_reached, stepsize, constrained_method);

        double thresh = 1e-4;
        if( (qa_reached->_q - qb_reached->_q).norm2() < thresh )
        {
            cout << "Hurra - der er forbindelse med CBiRRT" << endl;
            if(i%2 == 0)
                path = extract_path(Ta,qa_reached,Tb,qb_reached);
            if(i%2 != 0)
                path = extract_path(Tb,qb_reached,Ta,qa_reached);
            return path;
        }
        else //Swap trees:
        {
            Tdummy = Ta;
            Ta = Tb;
            Tb = Tdummy;
        }
    }

    cout << "Could not find a path with CBiRRT" << endl;
    cout << "Tree A size: " << Ta->get_nodes().size();
    cout << "Tree B size: " << Tb->get_nodes().size();

    return path;

}


Node* Rrt::constrained_extend(Tree* T, Node* qnear, Node* qtarget, double qstep, int constrained_method)
{
    double error = 10e-5;
    Node* qs = qnear;
    Node* qsold = qnear;
    Node* qdummy = NULL;
    while(true)
    {
        if((qtarget->_q - qs->_q).norm2() < error) //Check if qtarget and qs are equal
            return qsold;
        else
            if(qdummy != NULL)
            {
                if((qtarget->_q - qs->_q).norm2() >= (qdummy->_q - qtarget->_q).norm2())
                    return qdummy;
            }
        Q dir = (qtarget->_q - qs->_q) / (qtarget->_q - qs->_q).norm2();
        qs = new Node ( qs->_q + min(qstep, (qtarget->_q - qs->_q).norm2()) * dir );
        switch(constrained_method)
        {
        case RGD:
            if(RGD_new_config(qs, qsold)) //Checks if collision free and updates qs
            {
                T->add_node(qs);
                T->add_edge(qsold, qs);
                qdummy = qsold;
                qsold = qs;
            }
            else
            {
                return qsold;
            }
            break;
         case TS:
            if(TS_new_config(qs, qsold)) //Checks if collision free and updates qs
            {
                T->add_node(qs);
                T->add_edge(qsold, qs);
                qdummy = qsold;
                qsold = qs;
            }
            else
            {
                return qsold;
            }
            break;
        case FR:
            if(FR_new_config(qs, qsold)) //Checks if collision free and updates qs
            {
                T->add_node(qs);
                T->add_edge(qsold, qs);
                qdummy = qsold;
                qsold = qs;
            }
            else
            {
                return qsold;
            }
            break;
        }


    }
}


vector<Q> Rrt::extract_path(Tree* Ta, Node* qa_reached, Tree* Tb, Node* qb_reached)
{
    cout << "Extracting path from tree" << endl;
    vector<Q> Ta_vec;
    Ta_vec.push_back(qa_reached->_q);


    Node* nptr = qa_reached;
    while(nptr->parent != NULL)
    {
        nptr = nptr->parent;
        Ta_vec.push_back(nptr->_q);
    }

    reverse(Ta_vec.begin(),Ta_vec.end());

    nptr = qb_reached;  //Not pushed back since qa and qb are the same location (or very close)
    while(nptr->parent != NULL)
    {
        nptr = nptr->parent;
        Ta_vec.push_back(nptr->_q);
    }
    return Ta_vec; //From init to Goal
}


State Rrt::get_state()
{
    return _state; //Careful will alter state of Rrt object if.
}


void Rrt::set_state(State state)
{
    _state = state;
}

void Rrt::grip_frame(Frame* gripper, Frame* object)
{
    Frame *bottle_frame = _wc->findFrame("Bottle");
    Frame *tool_frame = _wc->findFrame("UR5.TCP");
    Kinematics::gripFrame(bottle_frame, tool_frame, _state);

}

void Rrt::release_bottle()
{
    Frame *bottle_frame = _wc->findFrame("Bottle");
    Frame *ground_frame = _wc->findFrame("Ground");
    Kinematics::gripFrame(bottle_frame, ground_frame, _state);
}

bool Rrt::path_col_expanded_binary(Q q1, Q q2)
{
    double error = 10e-4;
    double eps = 0.01;
    Q qi;
    Q dq = q2 -q1;
    int N = ceil(log2(dq.norm2()/eps));
    int n = pow(2,N);
    Q dq_new = (pow(2,N)*eps)/dq.norm2() * dq; //opskalering af dq
    int levels = N;
    for(int i = 1; i <= levels; i++)
    {
        int steps = pow(2,i-1);
        Q step = dq_new/steps;
        for(int j = 1; j <= steps; j++)
        {
            qi = q1 + (j - 0.5) * step;

            if((qi-q1).norm2() < dq.norm2())
            {
                if(in_collision(qi) || computeTaskError(qi,qi).norm() >= error)
                {
                    return false;
                }
            }
        }

    }
    return true;
}

vector<Q> Rrt::removeRedundantNodes(vector<Q> currentPath)
{
    cout << "path pruning begun" << endl;
    int i = 0;
    int nodesRemoved = 0;
    while(i < currentPath.size()-2)
    {
        if(path_col_expanded_binary(currentPath.at(i),currentPath.at(i+2)))
        {
            currentPath.erase(currentPath.begin()+i+1);
            nodesRemoved++;
            if(i > 0)
                i--;
        }
        else
            i++;

    }
    cout << nodesRemoved << endl;
    return currentPath;
}

pair<bool, vector<Q>> Rrt::parabolicBlend(Q q1, Q q2, Q q3, int tau)
{
    pair<bool, vector<Q>> returnPair;
    returnPair.first = true;
    double error = 10e-4;
    vector<Q> interpPath;
    Q step;
    double stepsize = 0.01;
    int tauTemp = ceil(tau*stepsize);
    double dq1 = (q2-q1).norm2();
    double dq3 = (q3-q2).norm2();
    int steps1 = ceil(dq1/stepsize);
    for(int i = 0; i < steps1+1-tauTemp; i++)
    {
        step = q1+((double)i/steps1)*(q2-q1);
        if(in_collision(step) || computeTaskError(step,step).norm() >= error)
        {
            returnPair.first = false;
        }
        interpPath.push_back(step);
    }
    for(int i = 0; i < 2*tau; i++)
    {
        Q v2 = (tau*(q3-q2))/dq3;
        Q v1 = (tau*(q2-q1))/dq1;
        step = (((v2-v1)/(4*tau))*pow(i,2))+(v1*(i-tau))+q2;
        if(in_collision(step) || computeTaskError(step,step).norm() >= error)
        {
            returnPair.first = false;
        }
        interpPath.push_back(step);
    }
    returnPair.second = interpPath;
    return returnPair;
}

pair<bool, vector<Q>> Rrt::linearInterp(Q q1, Q q2)
{
    pair<bool, vector<Q>> returnPair;
    returnPair.first = true;
    double error = 10e-4;
    vector<Q> interpPath;
    double stepsize = 0.05;
    double dq = (q2-q1).norm2();
    int steps = ceil(dq/stepsize);
    for(int i = 0; i < steps+1; i++)
    {
        Q step = q1+((double)i/steps)*(q2-q1);
        if(in_collision(step) || computeTaskError(step,step).norm() >= error)
        {
            returnPair.first = false;
        }
        interpPath.push_back(step);
    }
    returnPair.second = interpPath;
    return returnPair;
}

vector<Q> Rrt::interpolateRobotPath(vector<Q> robotPath)
{
    //TODO Get functionality into removeRedundantNodes. Discard path_col and use interps instead.
    vector<Q> interpedPath;
    for(int i = 0; i < robotPath.size()-1; i++)
    {
       if(i < robotPath.size()-2)
       {
           pair<bool,vector<Q>> parabolicPair = parabolicBlend(robotPath.at(i),robotPath.at(i+1),robotPath.at(i+2),3);
           if(parabolicPair.first == true)
           {
               vector<Q> tempPath = parabolicPair.second;
               cout << "para" << endl;
               for(int k = 0; k < tempPath.size(); k++)
               {
                   interpedPath.push_back(tempPath.at(k));
               }
           }
           else
           {
               pair<bool,vector<Q>> linearInterpPair = linearInterp(robotPath.at(i),robotPath.at(i+1));
               if(linearInterpPair.first == true)
               {
                   vector<Q> tempPath = linearInterpPair.second;
                   for(int j = 0; j < tempPath.size(); j++)
                   {
                       if(i == 0)
                       {
                         interpedPath.push_back(tempPath.at(j));
                       }
                       else
                       {
                           if(j != 0)
                               interpedPath.push_back(tempPath.at(j));
                       }
                   }
                   cout << "linear1" << endl;
               }
               else{
                   interpedPath.push_back(robotPath.at(i));
               }
           }
       }
       else
       {
           pair<bool,vector<Q>> linearInterpPair = linearInterp(robotPath.at(i),robotPath.at(i+1));
           if(linearInterpPair.first == true)
           {
               vector<Q> tempPath = linearInterpPair.second;
               for(int j = 0; j < tempPath.size(); j++)
               {
                   if(i == 0)
                   {
                     interpedPath.push_back(tempPath.at(j));
                   }
                   else
                   {
                       if(j != 0)
                           interpedPath.push_back(tempPath.at(j));
                   }
               }
               cout << "linear2" << endl;
           }
           else{
               interpedPath.push_back(robotPath.at(i));
               if(i == robotPath.size()-2)
                   interpedPath.push_back(robotPath.at(i+1));
               cout << "butwhy" << endl;
           }
       }
    }
    return interpedPath;

}


/*
 * p_desired and rot_desired MUST be TCP in base coordinates
 */
vector<Q> Rrt::JIKS(Q q_initial, Vector3D<> p_desired, Rotation3D<> rot_desired)
{
        _device->setQ(q_initial, _state);
        rw::invkin::JacobianIKSolver JIKS_obj(_device, _state);
        Transform3D<> TbaseTCP(p_desired, rot_desired);
        vector<Q> qdesired = JIKS_obj.solve(TbaseTCP,_state);
        return qdesired;
}

vector<Q> Rrt::JIKS(Q q_initial, Transform3D<> TbaseTCP_desired)
{
        _device->setQ(q_initial, _state);
        rw::invkin::JacobianIKSolver JIKS_obj(_device, _state);
        JIKS_obj.setCheckJointLimits(true);
        JIKS_obj.setMaxError(1e-5);
        JIKS_obj.setMaxIterations(10000);
        cout << "Max Error: " << JIKS_obj.getMaxError() << endl;
        cout << "Max Iterations: " << JIKS_obj.getMaxIterations() << endl;
        vector<Q> qdesired = JIKS_obj.solve(TbaseTCP_desired,_state);
        return qdesired;
}


/*
 * camera_q is the configuration of the robot as it grabbed the image
 */


Transform3D<> Rrt::get_TBaseObj(Transform3D<> TCamObj, Q camera_q)
{
    _device->setQ(camera_q, _state);
    Frame* cam_frame = _wc->findFrame("CameraFrame");
    Frame* base_frame = _wc->findFrame("UR5.Base");
    Transform3D<> TBaseCam = base_frame->fTf(cam_frame, _state);
    Transform3D<> TBaseObj = TBaseCam * TCamObj;
    return TBaseObj;
}

Transform3D<> Rrt::get_TBaseObj_fake(Transform3D<> TCamObj, Q camera_q)
{
    _device->setQ(camera_q, _state);
    Frame* cam_frame = _wc->findFrame("CameraFrame");
    Frame* base_frame = _wc->findFrame("UR5.Base");
    Transform3D<> TBaseCam = base_frame->fTf(cam_frame, _state);
    Transform3D<> TBaseObj = TBaseCam * TCamObj;

    cout << "TBaseObj: " << endl;
    cout << TBaseObj << endl;

    Rotation3D<> rot_dummy = TBaseObj.R();
    RPY<> rpy_fake(rot_dummy);
    rpy_fake(1) = 0; rpy_fake(2) = 1.57; //Only keep the roll estimate
    cout << "rpy fake: \n" << rpy_fake << endl;
    Rotation3D<> rot_fake = rpy_fake.toRotation3D();


    Transform3D<> TBaseObj_fake(TBaseObj.P(), rot_fake);

    cout << "TBaseObj_fake: " << endl;
    cout << TBaseObj_fake << endl;

    return TBaseObj_fake;
}

Transform3D<> Rrt::get_desired_TCP_pos_fake(Transform3D<> TCamObj, Q camera_q)
{
    Transform3D<> TBaseObj_fake = get_TBaseObj_fake(TCamObj, camera_q);

//    Vector3D<> p_des(-0.016, 0.050, -0.141); //Grip from the side:
//    RPY<> rpy_des(0,0,0);

    //Grip from the top:
    Vector3D<> p_des(0,0.202,0);
    RPY<> rpy_des(-3.14, 1.57, -1.57);

    Rotation3D<> rot_des = rpy_des.toRotation3D();
    Transform3D<> TObjTCP_des(p_des, rot_des);

    Transform3D<> TBaseTCP_des = TBaseObj_fake * TObjTCP_des;

    cout << "TBaseTCP desired fake: " << endl;
    cout << TBaseTCP_des << endl;
    return TBaseTCP_des;
}


Transform3D<> Rrt::get_desired_TCP_pos(Transform3D<> TCamObj, Q camera_q)
{
    _device->setQ(camera_q, _state);
    Frame* cam_frame = _wc->findFrame("CameraFrame");
    Frame* tcp_frame = _wc->findFrame("UR5.TCP");
    Frame* base_frame = _wc->findFrame("UR5.Base");

    Transform3D<> TBaseCam = base_frame->fTf(cam_frame, _state);
    Transform3D<> TBaseTCP = _device->baseTframe(tcp_frame, _state);

    /***Extract object position***/
    cout << "TCamObj: " << endl;
    cout << TCamObj << endl;
    Transform3D<> TBaseObj = TBaseCam * TCamObj;
    cout << "Transform Base -> Object: " << endl;
    cout << TBaseObj << endl;

    /***Find the desired transform TBaseTCP_desired***/
    //TBaseTCP_des = TBaseObj * [TObjTCP]des
    RPY<> rpy_ObjTCP(0,0,1.57);
    Vector3D<> p_ObjTCP(0,0.3,0);
    Transform3D<> TObjTCP_des(p_ObjTCP, rpy_ObjTCP);
    Transform3D<> TBaseTCP_desired = TBaseObj * TObjTCP_des;
/*
//    Vector3D<> p_cam = TCamObj.P();
//    RPY<> rpy_cam(0,0,0);
//    Transform3D<> TCamObj_no_rotation(p_cam, rpy_cam);
    //Transform3D<> TBaseObj = TBaseCam * TCamObj_no_rotation;
    //cout << "Transform Base -> Object: " << endl;

//    Vector3D<> p_des = TBaseObj.P();
//    p_des(2) = p_des(2) + 0.3; //Displace TCP 30 centimeters above object.
    //RPY<> rpy_des(0,0,1.57); //(0,0,1.57) means the gripper is gripping the obj from the side.
//    RPY<> rpy_des(0,0,3.14); //(0,0,3.14) means the gripper is gripping the obj from the top.
//    Transform3D<> TBaseTCP_desired(p_des, rpy_des);
*/

    cout << "TbaseTCP_desired: " << endl;
    cout << TBaseTCP_desired << endl;

    return TBaseTCP_desired;
}


void Rrt::move_snemand(Transform3D<> TBaseSnemand)
{
    Frame* base_frame = _wc->findFrame("UR5.Base");
    MovableFrame* snemand_frame = _wc->findFrame<MovableFrame>("Snemand");

    Transform3D<> snemand_pose = base_frame->fTf(snemand_frame,_state);
    cout << "Snemand was previously placed in: " << endl;
    cout << snemand_pose << endl;

    snemand_frame->setTransform(TBaseSnemand, _state);
    Transform3D<> snemand_pose_new = base_frame->fTf(snemand_frame,_state);

    cout << "Snemand is now placed in: " << endl;
    cout << snemand_pose_new << endl;

}

void Rrt::grip_snemand(Q q_grab)
{
    Frame *tcp_frame = _wc->findFrame("UR5.TCP");
    Frame *snemand_frame = _wc->findFrame("Snemand");
    _device->setQ(q_grab,_state);
    Kinematics::gripFrame(snemand_frame, tcp_frame, _state);   //Grip snemand and fix him to tcp.
}

void Rrt::release_snemand(Q q_release)
{
    Frame *base_frame = _wc->findFrame("UR5.Base");
    Frame *snemand_frame = _wc->findFrame("Snemand");
    _device->setQ(q_release,_state);
    Kinematics::gripFrame(snemand_frame, base_frame, _state);   //Release snemand and fix him to base
}



Transform3D<> Rrt::vectorToTrans(vector<float> input) //column major input
{
    Rotation3D<> first(input.at(0),input.at(4),input.at(8),input.at(1),input.at(5),input.at(9),input.at(2),input.at(6),input.at(10));
    Vector3D<> second(input.at(12),input.at(13),input.at(14));
    Transform3D<> T(second,first);
    return T;
}


void Rrt::log_data_path(vector<vector<Q>> path, string filepath)
{
    ofstream file(filepath);

    for(int i = 0; i < path.size(); i++)
    {
        for(int j = 0; j < path[i].size(); j++)
        {
            _device->setQ(path[i][j], _state);
            Frame* tcp_frame = _wc->findFrame("UR5.TCP");
            Transform3D<> TBaseTCP = _device->baseTframe(tcp_frame, _state);

            Vector3D<> p = TBaseTCP.P();
            RPY<> rot(TBaseTCP.R());
            file << p(0) << " " << p(1) << " " << p(2) << " " << rot(0) << " " << rot(1) << " " << rot(2) << endl;

        }
    }

    file.close();


}


