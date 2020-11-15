#include "helpfuncs.h"

namespace grp7
{
    void lua_parser(std::vector<rw::math::Q> path, std::string filepath)
    {
        std::ofstream file(filepath);

        std::string beginning = "wc = rws.getRobWorkStudio():getWorkCell()\n"
        "state = wc:getDefaultState()\n"
        "device = wc:findDevice(\"UR5\")\n"


        "function setQ(q)\n"
        "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
        "device:setQ(qq,state)\n"
        "rws.getRobWorkStudio():setState(state)\n"
        "rw.sleep(0.01)\n"
        "end\n\n";
        file << beginning;

        for(int i = 0; i < path.size(); i++)
        {
            std::string setQ = "setQ({" + std::to_string(path[i][0]) + "," +  std::to_string(path[i][1]) + "," +  std::to_string(path[i][2]) + "," +
                     std::to_string(path[i][3]) + "," +  std::to_string(path[i][4]) + "," +  std::to_string(path[i][5]) + "})";

            file << setQ << std::endl;
        }

        file.close();
    }


    void lua_parser_splicer(std::vector<rw::math::Q> path1, std::vector<rw::math::Q> path2, std::string filepath)
    {
        std::ofstream file(filepath);

        std::string beginning = "wc = rws.getRobWorkStudio():getWorkCell()\n"
        "state = wc:getDefaultState()\n"
        "device = wc:findDevice(\"UR5\")\n"
        "gripper = wc:findFrame(\"UR5.TCP\")\n"
        "bottle = wc:findFrame(\"Bottle\")\n"

        "function setQ(q)\n"
        "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
        "device:setQ(qq,state)\n"
        "rws.getRobWorkStudio():setState(state)\n"
        "rw.sleep(0.1)\n"
        "end\n\n"

        "function attach(obj, tool)\n"
        "rw.gripFrame(obj, tool, state)\n"
        "rws.getRobWorkStudio():setState(state)\n"
        "rw.sleep(0.01)\n"
        "end\n\n\n";

        file << beginning;

        for(int i = 0; i < path1.size(); i++)
        {
            std::string setQ = "setQ({" + std::to_string(path1[i][0]) + "," +  std::to_string(path1[i][1]) + "," +  std::to_string(path1[i][2]) + "," +
                     std::to_string(path1[i][3]) + "," +  std::to_string(path1[i][4]) + "," +  std::to_string(path1[i][5]) + "})";

            file << setQ << std::endl;
        }


        file << "attach(bottle,gripper)\n";

        for(int i = 0; i < path2.size(); i++)
        {
            std::string setQ = "setQ({" + std::to_string(path2[i][0]) + "," +  std::to_string(path2[i][1]) + "," +  std::to_string(path2[i][2]) + "," +
                     std::to_string(path2[i][3]) + "," +  std::to_string(path2[i][4]) + "," +  std::to_string(path2[i][5]) + "})";

            file << setQ << std::endl;
        }

        file.close();


    }

    void full_lua_parser_splicer(std::vector<std::vector<rw::math::Q>> fullpath, std::string filepath)
    {
        std::ofstream file(filepath);

        std::string beginning = "wc = rws.getRobWorkStudio():getWorkCell()\n"
        "state = wc:getDefaultState()\n"
        "device = wc:findDevice(\"UR5\")\n"
        "gripper = wc:findFrame(\"UR5.TCP\")\n"
        "bottle = wc:findFrame(\"Bottle\")\n"
        "ground = wc:findFrame(\"Ground\")\n"

        "function setQ(q)\n"
        "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
        "device:setQ(qq,state)\n"
        "rws.getRobWorkStudio():setState(state)\n"
        "rw.sleep(0.01)\n"
        "end\n\n"

        "function attach(obj, tool)\n"
        "rw.gripFrame(obj, tool, state)\n"
        "rws.getRobWorkStudio():setState(state)\n"
        "rw.sleep(0.01)\n"
        "end\n\n\n";

        file << beginning;

        for(int j = 0; j < fullpath.size(); j++)
        {

            for(int i = 0; i < fullpath.at(j).size(); i++)
            {
                std::string setQ = "setQ({" + std::to_string(fullpath.at(j)[i][0]) + "," +  std::to_string(fullpath.at(j)[i][1]) + "," +  std::to_string(fullpath.at(j)[i][2]) + "," +
                         std::to_string(fullpath.at(j)[i][3]) + "," +  std::to_string(fullpath.at(j)[i][4]) + "," +  std::to_string(fullpath.at(j)[i][5]) + "})";

                file << setQ << std::endl;
            }

            if(j == 0)
                file << "attach(bottle,gripper)\n";
            if(j == fullpath.size()-2)
                file << "attach(bottle,ground)\n";
        }

    }


    void visualize_tree(Tree T, std::string filename, int dof1, int dof2)
    {
        vector<Node*> nodes = T.get_nodes();
        //rw::sensor::Image::Ptr canvas = rw::loaders::PGMLoader::load("canvas.pgm"); //Blank 720x720 canvas
        rw::sensor::Image::Ptr canvas = new rw::sensor::Image;
        canvas->resize(720,720);

        //Paint entire canvas white:
        for(int i = 0; i < canvas->getWidth(); i++)
            for(int j = 0; j < canvas->getHeight(); j++)
                canvas->setPixel8U(i,j,255);

        //Draw nodes:x
        for(int i = 0; i < nodes.size(); i++)
        {
            double q1 = nodes[i]->_q[dof1];
            double q2 = nodes[i]->_q[dof2];

            //Coordinate system is (-360,-360) at bottom left -> (360,360) top right. This assumes constraints of [-360;360] for every joint
            int x = q1 * 360 / (2 * M_PI) + 360;
            int y = -q2 * 360 / (2 * M_PI) + 360;
            canvas->setPixel8U(x,y,0);
        }

        if( canvas->saveAsPGM(filename) )
            cout << "saved tree visualization to file: " << filename << endl;
        else
            cout << "failed to save file" << endl;

    }

    vector<Q> construct_path(Q from, Q to, Eigen::Matrix<double,6,6> constraint, Rrt& rrt_obj, int method)
    {
        vector<Q> path = rrt_obj.CBiRRT(from, to, constraint, method);
        //path = rrt_obj.removeRedundantNodes(path);
        //path = rrt_obj.interpolateRobotPath(path);
        return path;
    }

    Transform3D<> vectorToTrans(vector<float> input) //column major input
    {
        Rotation3D<> first(input.at(0),input.at(4),input.at(8),input.at(1),input.at(5),input.at(9),input.at(2),input.at(6),input.at(10));
        Vector3D<> second(input.at(12),input.at(13),input.at(14));
        Transform3D<> T(second,first);
        return T;
    }

    void log_data(Transform3D<> TBaseObj, ofstream &file)
    {
        Vector3D<> p = TBaseObj.P();
        RPY<> rot(TBaseObj.R());
        file << p(0) << " " << p(1) << " " << p(2) << " " << rot(0) << " " << rot(1) << " " << rot(2) << endl;

        cout << "Logged TBaseObj: " << endl;
        cout << "[XYZ]: " << "(" << p(0) << ", " << p(1) << ", " << p(2) << ")" << endl;
        cout << "[RPY]: " << "(" << rot(0) << ", " << rot(1) << ", " << rot(2) << ")" << endl;
    }
}




