#ifndef HELPFUNCS_H
#define HELPFUNCS_H

//#include "tree.h"
#include <fstream>
#include <vector>
#include <rw/math.hpp>
#include <rw/loaders/image/PGMLoader.hpp>
#include <rw/sensor/Image.hpp>
#include "Rrt.h"
#include "global_def.h"


namespace grp7
{
    void lua_parser(std::vector<rw::math::Q> path, std::string filepath);
    void lua_parser_splicer(std::vector<rw::math::Q> path1, std::vector<rw::math::Q> path2, std::string filepath);
    void full_lua_parser_splicer(std::vector<std::vector<rw::math::Q>> fullpath, std::string filepath);
    void visualize_tree(Tree T, std::string filename, int dof1, int dof2);
    void save_tree(Tree T, std::string filepath); //Saving the environment
    void load_tree(Tree &T, std::string filepath);
    vector<Q> construct_path(Q from, Q to, Eigen::Matrix<double,6,6> constraint, Rrt& rrt_obj, int method = RGD);
    Transform3D<> vectorToTrans(vector<float> input);

    void log_data(Transform3D<> TBaseObj, ofstream &file); //writes a line with [XYZ RPY] into file
}

#endif
