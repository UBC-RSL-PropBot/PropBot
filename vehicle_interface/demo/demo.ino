
#include "autonomy_interface.h"


extern ros::NodeHandle node;
void setup(){
   setup_rosserial();
}


void loop()
{
  node.spinOnce();
  delay(2);
}
