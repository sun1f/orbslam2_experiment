#include <iostream>
#include <fstream>
using namespace std;

int main()
{
    ofstream f("/home/sun/catkin_ws/src/ORB_SLAM2/Trajectory/rgbd/posegraph.g2o", ios::app);
    f << "i love china" << endl;
    f.close();
    return 0;
}