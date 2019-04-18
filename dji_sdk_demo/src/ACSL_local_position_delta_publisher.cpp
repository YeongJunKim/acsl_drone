#include "dji_sdk_demo/ACSL_local_position_delta_publisher.h"

ros::Publisher ctrlPosYawPub;
ros::Publisher pub_local_position;

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_local_position_delta_control_node");
    ros::NodeHandle nh;
    float gain = 0.5;
    pub_local_position = nh.advertise<dji_sdk::ACSL_local_position_delta>("ACSL/local_position_delta",10);

    dji_sdk::ACSL_local_position_delta local_position_delta;

    local_position_delta.seq = 0;
    local_position_delta.deltaX = 0;
    local_position_delta.deltaY = 0;
    local_position_delta.deltaZ = 0;
    local_position_delta.deltaYaw = 0;
    pub_local_position.publish(local_position_delta);

    while (1)
    {
        char a;
        float target[4] = {0,};
        
        cout << "now gain  :" << gain << endl;

        cout << "continue : c exit : q" << endl << "increase gain : g decrease gain : b" << endl << "setting gain : h " << endl << " : ";
        
        cin >> a;

        if( a == 'q')
            break;
        
        local_position_delta.seq++;
        local_position_delta.deltaX = 0;
        local_position_delta.deltaY = 0;
        local_position_delta.deltaZ = 0;
        local_position_delta.deltaYaw = 0;
        switch(a)
        {
          case 'a':
          local_position_delta.deltaX = gain;
          break;
          case 'z':
          local_position_delta.deltaX = -gain;
          break;
          case 's':
          local_position_delta.deltaY = gain;
          break;
          case 'x':
          local_position_delta.deltaY = -gain;
          break;
          case 'd':
          local_position_delta.deltaZ = gain;
          break;
          case 'c':
          local_position_delta.deltaZ = -gain;
          break;
          case 'f':
          local_position_delta.deltaYaw = gain;
          break;
          case 'v':
          local_position_delta.deltaYaw = -gain;
          break;
          case 'g':
          gain = gain + 0.01;
          break;
          case 'b':
          gain = gain - 0.01;
        }

        cout << "input" << a << endl;

        pub_local_position.publish(local_position_delta);
        ros::spinOnce();
    }

    return 0;
}