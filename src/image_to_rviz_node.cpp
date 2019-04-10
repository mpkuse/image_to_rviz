#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cv.h>
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <dynamic_reconfigure/server.h>
#include <image_to_rviz/paramsConfig.h>
#include <string>

double lu_x = 0, lu_y = 0;
double yaw = -3.14;
double sizeSq = 1;
double z = -10;

void callback(image_to_rviz::paramsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf",
            config.size_param, config.dx_param,
            config.dy_param,
            config.yaw,
            config.z);

  lu_x = config.dx_param;
  lu_y = config.dy_param;
  yaw = config.yaw;
  sizeSq = config.size_param;
  z = config.z;
};

int main( int argc, char** argv )
{

      ros::init(argc, argv, "image_to_rviz");
      ros::NodeHandle n("~");
      ros::Rate r(2);
      ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

      dynamic_reconfigure::Server<image_to_rviz::paramsConfig> server;
      dynamic_reconfigure::Server<image_to_rviz::paramsConfig>::CallbackType f;

      f = boost::bind(&callback, _1, _2);
      server.setCallback(f);

      cv::Mat src;
      std::string packagePath = ros::package::getPath("image_to_rviz");
      std::string fname = "/img/cyt.png";
      if( argc == 1 ) {
          std::cout << "No image name specified, so using default image.\n";
          std::cout << "Invalid Usage. Sample Usage:\n$ " << argv[0] << " <image name> \n";
          std::cout << "image name need to be relative to this package.\n";

      } else if( argc == 2 ) {
          std::cout << "Will try to use specified image name: " << argv[1] << " \n";
          fname = std::string( argv[1] );
      } else {
          std::cout << "Invalid Usage. Sample Usage:\n$ " << argv[0] << " <image name> \n";
          exit(1);
      }

      std::cout << "Open Image : " << packagePath + "/" + fname << std::endl;
      src = cv::imread(packagePath + "/" + fname, 1 );

      if( src.empty() ) {
          std::cout << "[ERROR] Cannot open image file. Most likely the image, the path you specified did not exist or there are permission issue.\n";
          exit(1);
      }
    //   src = cv::imread(packagePath + "/img/cyt.png", 1 );
      cv::resize(src, src, cv::Size(src.cols / 2, src.rows / 2));
    std::cout << "Image was successfully opened...\n";

      visualization_msgs::Marker image;
      image.header.frame_id = "/world";
      image.header.stamp = ros::Time::now();
      image.ns = "image";
      image.id = 0;
      image.action = visualization_msgs::Marker::ADD;
      image.type = visualization_msgs::Marker::TRIANGLE_LIST;
      image.scale.x = 1;
      image.scale.y = 1;
      image.scale.z = 1;

      double pix;
          geometry_msgs::Point p;
      std_msgs::ColorRGBA crgb;

      double center_x = src.rows / 2.0;
      double center_y = src.cols / 2.0;


      while (ros::ok())
      {
            pix = sizeSq / src.rows;
            image.points.clear();
            image.colors.clear();
                  for(int r = 0; r < src.rows; ++r) {
                      for(int c = 0; c < src.cols; ++c) {
                        cv::Vec3b intensity = src.at<cv::Vec3b>(r, c);
                        crgb.r = intensity.val[2] / 255.0;
                        crgb.g = intensity.val[1] / 255.0;
                        crgb.b = intensity.val[0] / 255.0;
                        crgb.a = 1.0;

                        p.z = z;
                        double x, y;
                        x = (r - center_x);
                        y = (c - center_y);
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);

                        x = (r - center_x) + 1;
                        y = (c - center_y);
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);

                        x = (r - center_x);
                        y = (c - center_y) + 1;
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);

                        x = (r - center_x) + 1;
                        y = (c - center_y);
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);

                        x = (r - center_x) + 1;
                        y = (c - center_y) + 1;
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);

                        x = (r - center_x);
                        y = ((c - center_y) + 1);
                        p.x = lu_x + (x * cos(yaw) - y * sin(yaw)) * pix;
                        p.y = lu_y + (x * sin(yaw) + y * cos(yaw)) * pix;
                        image.points.push_back(p);
                        image.colors.push_back(crgb);

                      }
                  }

            marker_pub.publish(image);
            ros::spinOnce();
            r.sleep();

      }
}
