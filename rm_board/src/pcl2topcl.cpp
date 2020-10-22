#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

tf::StampedTransform transform;  // base_linkとbase_laserの座標変換を保存する用

void pc_callback(sensor_msgs::PointCloud2 pc2){
  sensor_msgs::PointCloud2 pc2_transformed;

  // 座標変換はこの一行だけ！
  pcl_ros::transformPointCloud("base_link", transform, pc2, pc2_transformed);

  // 確認用にxyz座標のsensor_msgs::PointCloudに変換
  sensor_msgs::PointCloud pc;
  sensor_msgs::convertPointCloud2ToPointCloud(pc2_transformed, pc);

  // xyzの値を表示
  for(int i=0; i<pc.points.size(); i++){
    ROS_INFO("x:%f y:%f z:%f", pc.points[i].x, pc.points[i].y, pc.points[i].z);
  }
}

int main(int argc, char** argv){
  ros::init(argc,argv,"transform_test");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::Subscriber sub = nh.subscribe("/multisense_local/hsi_filter/output", 1, pc_callback);
  ros::Publisher pub = nh.advertise("out", 1);

  // TFを受信するやつ
  tf::TransformListener listener;

  // base_linkとbase_laserの間のTFが取得できるまで繰り返す
  while(true){
    try{
      //取得できたらtransformに保存しておく
      // listener.lookupTransform("base_link", "base_laser", ros::Time(0), transform);
      ROS_INFO("I got a transform!");
      break;
    }
    catch(tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  //メインループでは特に何もしない
  while(ros::ok()){
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
