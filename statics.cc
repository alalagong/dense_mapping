/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <memory>
#include <functional>
#include<list>
#include<vector>
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include"boost/thread.hpp"
#include<opencv2/core/core.hpp>
#include<rosbag/bag.h>
#include<rosbag/chunked_file.h>
#include<rosbag/view.h>
#include<rosbag/query.h>
#include"src/System.h"
#include <stdio.h>
#include <X11/Xlib.h>
#include <boost/filesystem.hpp>

using namespace std;


int main(int argc, char **argv)
{
    XInitThreads();
    ros::init(argc, argv, "Stereo");
    ros::start();
    ros::NodeHandle nh("~") ;
    string packagePath = ros::package::getPath("orb_slam2");
    string filePath = packagePath + "//00.txt" ;
    ifstream read;



    //0.read gt poses
    FILE* fp = std::fopen(filePath.c_str(), "r");
    if(!fp) {
        std::perror("File opening failed");
        return EXIT_FAILURE;
    }
    int kind ;
    double time, b[6], a[6], c[6], d[6] ;
    pair<int,double> tmp;
    vector<pair<int,double>> list ;

    memset(a, 0, sizeof(a) ) ;
    memset(b, 0, sizeof(b) ) ;
    memset(c, 0, sizeof(c) ) ;
    memset(d, 0, sizeof(d) ) ;
    while( fscanf(fp, "%d %lf", &kind, &time ) ==2 ){
        b[kind] += time ;
        a[kind]++ ;
        tmp.first = kind ;
        tmp.second = time ;
        list.push_back(tmp);
        if ( kind == 3 ){
            printf("%lf ", time ) ;
        }
    }
    puts("") ;
    std::fclose(fp);
    for( int i = 0 ; i < 6 ; i++ )
    {
        if ( i == 2 ){
            continue ;
        }
        c[i] = b[i]/a[i] ;
        printf("%lf ", c[i] ) ;
    }
    puts("") ;
    for( int i = 0, sz = list.size() ; i < sz; i++ )
    {
        int kind = list[i].first ;
        double time = list[i].second ;
        d[kind] += (time-c[kind])*(time-c[kind]) ;
    }
    for( int i = 0 ; i < 6 ; i++ )
    {
        if ( i == 2 ){
            continue ;
        }
        d[i] = sqrt(d[i]/a[i]) ;
        printf("%lf ", d[i] ) ;
    }
    puts("") ;



    return 0;
}
