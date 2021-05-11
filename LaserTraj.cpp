#include <ros/ros.h>
#include <my_pcl_tutorial/jaco.h>
#include <iostream>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>
#include <kinova_msgs/SetPosition.h>
#include <dlfcn.h>
#include <stdio.h>
#include "KinovaTypes.h"
#include "Kinova.API.CommLayerUbuntu.h"
#include "std_msgs/String.h"

#include <sstream>
#include <fstream>
#include <time.h> //计算每步骤需要的事件的头文件

#include <detector_sample/msgmix.h>
#include <std_msgs/Float32.h>


using namespace std;
#define pi 3.1415926
ros::ServiceClient trajectoryClient;
kinova_msgs::SetPosition srv;

float x = 0.2140;
float y = -0.2581;
float z = 0.5068;
float tx = 1.6513;
float ty = 1.1145;
float tz = 0.1311;
float fin = 0;


void callback(const detector_sample::msgmix::ConstPtr &msg)
{
  std_msgs::Float32 Px;
  std_msgs::Float32 Py;
  std_msgs::Float32 Pz;
  std_msgs::String obj_one;
  std_msgs::String obj_two;
  std_msgs::Float32 isopen;
  std_msgs::Float32 order;


  isopen.data = msg->isopen;
  order.data = msg->order;
  obj_one.data =msg->obj_one;
  obj_two.data =msg->obj_two;

  Px.data = msg->point_x;
  Py.data = msg->point_y;
  Pz.data = msg->laser_depth;


  Eigen::Vector4d jacoxyz,xtionxyz;
  Eigen::Vector3d txyz,pixxyz;
  double jacoeuler[6];

  Eigen::Matrix4d tf;
  Eigen::Matrix3d tf1,tf11;

  pixxyz[0] = Px.data;
  pixxyz[1] = Py.data;
  pixxyz[2] = 1;

// 内参矩阵的逆
  tf1<<0.0019, 0,-0.5827,
       0, 0.0019,-0.4493,
       0,0,1;

   txyz = tf1 * pixxyz;

   xtionxyz[0]=txyz[0];
   xtionxyz[1]=txyz[1];
   xtionxyz[2]=Pz.data * 0.001;
   xtionxyz[3]=1;


// 外参转移矩阵
   tf<<-0.9914, 0.4313,-0.3956,0.625,
     0.5271, 0.8219,-0.6190,  -0.046  ,
    0.0696,-0.8961,-0.9554,0.741,
        0  ,   0  ,   0   ,  1  ;

//tf<<-1.0229, 0.3378,-0.4372,0.657,
//     0.5190, 0.7484,-0.6111, -0.046 ,
//   0.0322,-0.8282,-0.9266,0.747,
//        0  ,   0  ,   0   ,  1  ;

  jacoxyz=tf*xtionxyz;
//  cout<<isopen<<endl;
//  cout<<order<<endl;
//  cout<<obj_one<<endl;
//  cout<<obj_two<<endl;

int y;
cout<<isopen.data<<"  "<<order.data<<"   "<<obj_one.data<<"   "<<obj_two.data<<endl;
cout<<"y/n ?"<<endl;

cin>>y;

cout<<y<<endl;

if (y==1)
{




// 放到指定位置
 if ((isopen.data == 0)&&(order.data == 1)&&((obj_one.data == "NULL")||(obj_one.data == "diningtable"))&&((obj_two.data == "diningtable")||(obj_two.data == "NULL"))){


    cout<<"闭合，点选一次，放到指定位置"<<endl;
    cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1];
    srv.request.Z=jacoxyz[2]+0.17;

    // cup
    srv.request.ThetaX=-2.2692;//1.91
    srv.request.ThetaY=1.4793;//1.09;
    srv.request.ThetaZ=-2.27;//-0.25;

    tx = srv.request.ThetaX;
    ty = srv.request.ThetaY;
    tz = srv.request.ThetaZ;


    srv.request.finger1=5000;
    srv.request.finger2=5000;
    srv.request.finger3=5000;

    trajectoryClient.call(srv) ;//调用服务


    srv.request.finger1=0;
    srv.request.finger2=0;
    srv.request.finger3=0;
    fin = srv.request.finger3;
    trajectoryClient.call(srv) ;//调用服务


    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1];
    srv.request.Z=jacoxyz[2]+0.35;
    x = srv.request.X;
    y = srv.request.Y;
    z = srv.request.Z;

    trajectoryClient.call(srv) ;//调用服务

 }

// 打开机械爪
else if((isopen.data == 0)&&(order.data == 2)&&((obj_one.data == "NULL")||(obj_one.data == "diningtable"))&&((obj_two.data == "diningtable")||(obj_two.data == "NULL"))){
    cout<<"闭合，点选二次，松开夹爪"<<endl;
    cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

     srv.request.X=x;
     srv.request.Y=y;
     srv.request.Z=z;

     srv.request.ThetaX=tx;//1.91
     srv.request.ThetaY=ty;//1.09;
     srv.request.ThetaZ=tz;//-0.25;

     srv.request.finger1=0;
     srv.request.finger2=0;
     srv.request.finger3=0;
     fin = srv.request.finger1;

     trajectoryClient.call(srv) ;//调用服务

}

// 擦桌子

else if((isopen.data == 0)&&(order.data == 3)){
    cout<<"闭合，常亮，擦桌子"<<endl;
    cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1];
    srv.request.Z=jacoxyz[2]+0.11;

    srv.request.finger1=7000;
    srv.request.finger2=7000;
    srv.request.finger3=7000;


    trajectoryClient.call(srv) ;//调用服务

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1] + 0.1;
    srv.request.Z=jacoxyz[2]+0.11;
    trajectoryClient.call(srv) ;//调用服务

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1] - 0.05;
    srv.request.Z=jacoxyz[2]+0.11;
    trajectoryClient.call(srv) ;//调用服务

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1] + 0.1;
    srv.request.Z=jacoxyz[2]+0.11;
    trajectoryClient.call(srv) ;//调用服务

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1] - 0.05;
    srv.request.Z=jacoxyz[2]+0.11;
    trajectoryClient.call(srv) ;//调用服务

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1] + 0.1;
    srv.request.Z=jacoxyz[2]+0.11;
    trajectoryClient.call(srv) ;//调用服务

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1] - 0.05;
    srv.request.Z=jacoxyz[2]+0.11;
    trajectoryClient.call(srv) ;//调用服务
//复位
	srv.request.X=0.2140;
	srv.request.Y=-0.2581;
	srv.request.Z=0.5068;
	x = srv.request.X;
	y = srv.request.Y;
	z = srv.request.Z;

	srv.request.ThetaX=1.6513;//1.91;
	srv.request.ThetaY=1.1145;//1.09;
	srv.request.ThetaZ=0.1311;//-0.25;
	tx = srv.request.ThetaX;
    ty = srv.request.ThetaY;
    tz = srv.request.ThetaZ;

	trajectoryClient.call(srv) ;
}

// 倾倒
else if((isopen.data == 0)&&(order.data == 2)&&(obj_two.data != "diningtable")&&(obj_two.data != "NULL")){
    cout<<"闭合，点选二次，倾倒"<<endl;
    cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=jacoxyz[0]+0.05;
    srv.request.Y=jacoxyz[1];
    srv.request.Z=jacoxyz[2]+0.3;

    srv.request.finger1=5000;
    srv.request.finger2=5000;
    srv.request.finger3=5000;
    fin = srv.request.finger1;


    srv.request.ThetaX=1.5539;
    srv.request.ThetaY=0.0552;//1.09;
    srv.request.ThetaZ=-0.0602;//-0.25;
    trajectoryClient.call(srv) ;//调用服务


    srv.request.ThetaX=1.5539;
    srv.request.ThetaY=0.0552;//1.09;
    srv.request.ThetaZ=-2.1884;//-0.25;
    trajectoryClient.call(srv) ;//调用服务

    srv.request.ThetaX=1.5539;
    srv.request.ThetaY=0.0552;//1.09;
    srv.request.ThetaZ=-0.0602;//-0.25;
    trajectoryClient.call(srv) ;//调用服务


    srv.request.X=x;
    srv.request.Y=y;
    srv.request.Z=z;

    srv.request.finger1=fin;
    srv.request.finger2=fin;
    srv.request.finger3=fin;


    srv.request.ThetaX=tx;
    srv.request.ThetaY=ty;//1.09;
    srv.request.ThetaZ=tz;//-0.25;
    trajectoryClient.call(srv) ;//调用服务


}
// 送到嘴边
else if((isopen.data == 0)&&(order.data == 5)){

    cout<<"闭合，点选常亮，送到嘴边"<<endl;
    cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=0.3591;
    srv.request.Y=-0.1650;
    srv.request.Z=0.5957;
    x = srv.request.X;
    y = srv.request.Y;
    z = srv.request.Z;

    srv.request.finger1=5000;
    srv.request.finger2=5000;
    srv.request.finger3=5000;


    srv.request.ThetaX=2.9546;
    srv.request.ThetaY=1.5347;//1.09;
    srv.request.ThetaZ=-1.2643;//-0.25;
	tx = srv.request.ThetaX;
    ty = srv.request.ThetaY;
    tz = srv.request.ThetaZ;

    trajectoryClient.call(srv) ;//调用服务

}

// 按点选位置
else if((isopen.data == 1)&&(order.data == 5)){

    cout<<"张开，点选常亮，按点选位置"<<endl;
    cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1];
    srv.request.Z=jacoxyz[2]+0.30;


    srv.request.finger1=7000;
    srv.request.finger2=7000;
    srv.request.finger3=0;

    srv.request.ThetaX=-3.0659;
    srv.request.ThetaY=-0.0822;//1.09;
    srv.request.ThetaZ=-1.3473;//-0.25;
    trajectoryClient.call(srv) ;//调用服务

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1];
    srv.request.Z=jacoxyz[2]+0.2;

    trajectoryClient.call(srv) ;//调用服务

    srv.request.X=x;
    srv.request.Y=y;
    srv.request.Z=z;


    srv.request.finger1=fin;
    srv.request.finger2=fin;
    srv.request.finger3=fin;

    srv.request.ThetaX=tx;
    srv.request.ThetaY=ty;//1.09;
    srv.request.ThetaZ=tz;//-0.25;

    trajectoryClient.call(srv) ;//调用服务

}


// 机械臂复位
else if((isopen.data == 1)&&(order.data == 2)&&((obj_one.data == "NULL")||(obj_one.data == "diningtable"))&&((obj_two.data == "diningtable")||(obj_two.data == "NULL"))){
     cout<<"张开，点选二次，机械臂复位"<<endl;
     cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

//reset
	 srv.request.X=0.2140;
	 srv.request.Y=-0.2581;
	 srv.request.Z=0.5068;
	 x = srv.request.X;
	 y = srv.request.Y;
	 z = srv.request.Z;

	 srv.request.ThetaX=1.6513;//1.91;
	 srv.request.ThetaY=1.1145;//1.09;
	 srv.request.ThetaZ=0.1311;//-0.25;
	 tx = srv.request.ThetaX;
     ty = srv.request.ThetaY;
     tz = srv.request.ThetaZ;

     srv.request.finger1=0;
     srv.request.finger2=0;
     srv.request.finger3=0;
     fin = srv.request.finger1;
     trajectoryClient.call(srv) ;//调用服务

}



// 推
else if((isopen.data == 1)&&(order.data == 2)&&(obj_two.data != "diningtable")&&(obj_two.data != "NULL")){

    cout<<"张开，点选二次，推选中物体"<<endl;
    cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1] + 0.1;
    srv.request.Z=jacoxyz[2]+0.2;
	x = srv.request.X;
	y = srv.request.Y;
	z = srv.request.Z;

    srv.request.finger1=0;
    srv.request.finger2=0;
    srv.request.finger3=0;
    fin = srv.request.finger1;


    srv.request.ThetaX=-3.0659;
    srv.request.ThetaY=-0.0822;//1.09;
    srv.request.ThetaZ=-1.3473;//-0.25;
    tx = srv.request.ThetaX;
    ty = srv.request.ThetaY;
    tz = srv.request.ThetaZ;
    trajectoryClient.call(srv) ;//调用服务


    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1] - 0.08;
    srv.request.Z=jacoxyz[2]+0.2;
    trajectoryClient.call(srv) ;//调用服务

    srv.request.X=x;
    srv.request.Y=y;
    srv.request.Z=z;

    srv.request.finger1=fin;
    srv.request.finger2=fin;
    srv.request.finger3=fin;


    srv.request.ThetaX=tx;
    srv.request.ThetaY=ty;//1.09;
    srv.request.ThetaZ=tz;//-0.25;
    trajectoryClient.call(srv) ;//调用服务


}

// 抓起送到嘴边
else if((isopen.data == 1)&&(order.data == 3)&&(obj_one.data == "cup")){

     cout<<"张开，常亮，抓起送嘴边"<<endl;
     cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1]+0.05;
    srv.request.Z=jacoxyz[2]+0.2;



    srv.request.finger1=0;
    srv.request.finger2=0;
    srv.request.finger3=0;
    fin = srv.request.finger1;
    // cup
    srv.request.ThetaX=1.5938;//1.91
    srv.request.ThetaY=0.0344;//1.09;
    srv.request.ThetaZ=-0.0602;//-0.25;

    trajectoryClient.call(srv) ;//调用服务

    srv.request.finger1=3500;
    srv.request.finger2=3500;
    srv.request.finger3=3500;


     trajectoryClient.call(srv) ;//调用服务
	 srv.request.X=0.2140;
	 srv.request.Y=-0.2581;
	 srv.request.Z=0.5068;
	 x = srv.request.X;
	 y = srv.request.Y;
	 z = srv.request.Z;

	 srv.request.ThetaX=1.6513;//1.91;
	 srv.request.ThetaY=1.1145;//1.09;
	 srv.request.ThetaZ=0.1311;//-0.25;
	 tx = srv.request.ThetaX;
     ty = srv.request.ThetaY;
     tz = srv.request.ThetaZ;
     trajectoryClient.call(srv) ;//调用服务


}


//cup

//x =0.1342
//y -0.5772
// 0.3211

//1.6545
//0.0978
//-0.038




// x 0.2235
// y -0.4376
// z 0.4055

//1
//tz -0.038

//2
//tx 1.668
//ty 0.3409
//tz -1.5298





//抓东西（瓶子类）

else if((isopen.data == 1)&&(order.data == 1)&&(obj_one.data == "cup")){

    cout<<"张开，点选，抓起"<<endl;
    cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=jacoxyz[0]+0.1;
    srv.request.Y=jacoxyz[1]+0.15;
    srv.request.Z=jacoxyz[2];

    srv.request.finger1=0;
    srv.request.finger2=0;
    srv.request.finger3=0;
    fin = srv.request.finger1;

    // cup
    srv.request.ThetaX=1.5938;//1.91
    srv.request.ThetaY=0.0344;//1.09;
    srv.request.ThetaZ=-0.0602;//-0.25;

    trajectoryClient.call(srv) ;//调用服务

    srv.request.X=jacoxyz[0]+0.1;
    srv.request.Y=jacoxyz[1]+0.08;
    srv.request.Z=jacoxyz[2];
    trajectoryClient.call(srv) ;//调用服务

    srv.request.finger1=5000;
    srv.request.finger2=5000;
    srv.request.finger3=5000;
    trajectoryClient.call(srv) ;//调用服务


    srv.request.Z=jacoxyz[2]+0.1;

	 x = srv.request.X;
	 y = srv.request.Y;
	 z = srv.request.Z;
	 tx = srv.request.ThetaX;
     ty = srv.request.ThetaY;
     tz = srv.request.ThetaZ;
     trajectoryClient.call(srv) ;//调用服务


}

//抓东西（叉子类）
else if((isopen.data == 1)&&(order.data == 1)&&(obj_one.data == "towel")){

  cout<<"张开，点选，抓起"<<endl;
    cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=jacoxyz[0]+0.1;
    srv.request.Y=jacoxyz[1]-0.05;
    srv.request.Z=jacoxyz[2]+0.15;

    srv.request.finger1=0;
    srv.request.finger2=0;
    srv.request.finger3=0;
    fin = srv.request.finger1;

    // spoon
    srv.request.ThetaX=3.0775;//1.91
    srv.request.ThetaY=-0.1663;//1.09;
    srv.request.ThetaZ=-0.4465;//-0.25;

    trajectoryClient.call(srv) ;//调用服务

    srv.request.Z=jacoxyz[2]+0.035;
    trajectoryClient.call(srv) ;//调用服务


    srv.request.finger1=7000;
    srv.request.finger2=7000;
    srv.request.finger3=7000;
    trajectoryClient.call(srv) ;//调用服务

    srv.request.Z=jacoxyz[2]+0.12;

	 x = srv.request.X;
	 y = srv.request.Y;
	 z = srv.request.Z;
	 tx = srv.request.ThetaX;
     ty = srv.request.ThetaY;
     tz = srv.request.ThetaZ;
     trajectoryClient.call(srv) ;//调用服务


}




// 闭合机械爪
else if((isopen.data == 1)&&(order.data == 3)&&((obj_one.data == "NULL")||(obj_one.data == "diningtable"))&&((obj_two.data == "diningtable")||(obj_two.data == "NULL"))){
     cout<<"张开，常亮，闭合机械爪"<<endl;
     cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=x;
    srv.request.Y=y;
    srv.request.Z=z;

    srv.request.finger1=6000;
    srv.request.finger2=6000;
    srv.request.finger3=6000;

    fin = srv.request.finger3;

    srv.request.ThetaX=tx;//1.91;
    srv.request.ThetaY=ty;//1.09;
    srv.request.ThetaZ=tz;//-0.25;

    trajectoryClient.call(srv) ;//调用服务

}

// 移动至指定位置
else if((isopen.data == 1)&&(order.data == 1)&&((obj_one.data == "NULL")||(obj_one.data == "diningtable"))&&((obj_two.data == "diningtable")||(obj_two.data == "NULL"))){
     
    cout<<"张开，点选，移动至激光点"<<endl;
    cout<<"选择物体： "<<obj_one.data<<"   "<<obj_two.data<<endl;

    srv.request.X=jacoxyz[0];
    srv.request.Y=jacoxyz[1];
    srv.request.Z=jacoxyz[2]+0.2;
    x = srv.request.X;
	y = srv.request.Y;
	z = srv.request.Z;

    // cup
    srv.request.ThetaX=-2.2692;//1.91
    srv.request.ThetaY=1.4793;//1.09;
    srv.request.ThetaZ=-2.27;//-0.25;
	tx = srv.request.ThetaX;
    ty = srv.request.ThetaY;
    tz = srv.request.ThetaZ;

    trajectoryClient.call(srv) ;//调用服务

}

}
y = -1;
}




int main(int argc, char **argv){
    ros::init(argc,argv,"LaserTrajector");
    ros::NodeHandle nh;
    trajectoryClient= nh.serviceClient <kinova_msgs::SetPosition>("j2n6a300_driver/in/set_Position") ;
    ros::Subscriber sub = nh.subscribe("msg_mix_info", 1, callback);
    ros::spin();
    return 0;
}
