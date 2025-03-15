/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rclcpp/rclcpp.hpp>
#include <find_object_2d/msg/objects_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QTransform>
#include <QColor>
#include <find_object_2d/msg/detection_info.hpp>

class PrintObjects : public rclcpp::Node {
public:
  PrintObjects() : Node("object_overlay_node") {
    image_transport::TransportHints hints(this);

    imagePub_ = image_transport::create_publisher(
        this, "image_with_objects",
        rclcpp::QoS(1)
            .reliability((rmw_qos_reliability_policy_t)2)
            .get_rmw_qos_profile());

    // Synchronized image + objects example
    imageSub_.subscribe(this, "image", hints.getTransport(),
                        rclcpp::QoS(1)
                            .reliability((rmw_qos_reliability_policy_t)2)
                            .get_rmw_qos_profile());
    objectsSub_.subscribe(this, "info",
                          rclcpp::QoS(1)
                              .reliability((rmw_qos_reliability_policy_t)2)
                              .get_rmw_qos_profile());

    exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(
        MyExactSyncPolicy(30), imageSub_, objectsSub_);
    exactSync_->registerCallback(
        std::bind(&PrintObjects::imageObjectsDetectedCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  virtual ~PrintObjects() { delete exactSync_; }


  std::string getFileNameWithoutExtension(const std::string& filePath) {
    // 1. Dateinamen extrahieren (nach letztem '/')
    size_t lastSlash = filePath.find_last_of('/');
    std::string fileName = (lastSlash == std::string::npos) ? filePath : filePath.substr(lastSlash + 1);

    // 2. Dateiendung entfernen (nach letztem '.')
    size_t lastDot = fileName.find_last_of('.');
    return (lastDot == std::string::npos) ? fileName : fileName.substr(0, lastDot);
}

  void imageObjectsDetectedCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
      const find_object_2d::msg::DetectionInfo::ConstSharedPtr objectsMsg) {
    printf("imageObjectsDetectedCallback.\n");
    if (imagePub_.getNumSubscribers() > 0) {
      printf("Subed imageObjectsDetectedCallback.\n");
      cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageMsg, "bgr8");

      cv_bridge::CvImage img;
      img = *imageDepthPtr;

      for (size_t i = 0; i < objectsMsg->ids.size(); ++i) {

        std::string id = getFileNameWithoutExtension(objectsMsg->file_paths[i].data);
        RCLCPP_INFO_STREAM(this->get_logger(), "Object: " << id);

        if (objectsMsg->homographies[i].data.size() < 9)
          continue;

        // Homographie-Matrix extrahieren
        const std::vector<float> &h_data = objectsMsg->homographies[i].data;

        cv::Mat H(3, 3, CV_32F);
        H.at<float>(0, 0) = h_data[0];
        H.at<float>(1, 0) = h_data[1];
        H.at<float>(2, 0) = h_data[2];
        H.at<float>(0, 1) = h_data[3];
        H.at<float>(1, 1) = h_data[4];
        H.at<float>(2, 1) = h_data[5];
        H.at<float>(0, 2) = h_data[6];
        H.at<float>(1, 2) = h_data[7];
        H.at<float>(2, 2) = h_data[8];

        // Eckpunkte des Objekts (normiertes Quadrat)
        std::vector<cv::Point2f> object_corners = {
            {0, 0},
            {static_cast<float>(objectsMsg->widths[i].data), 0},
            {static_cast<float>(objectsMsg->widths[i].data),
             static_cast<float>(objectsMsg->heights[i].data)},
            {0, static_cast<float>(objectsMsg->heights[i].data)},
            {static_cast<float>(objectsMsg->widths[i].data / 2),
             static_cast<float>(objectsMsg->heights[i].data / 2)}};

        // Eckpunkte transformieren
        std::vector<cv::Point2f> outPts;
        cv::perspectiveTransform(object_corners, outPts, H);

        RCLCPP_INFO_STREAM(this->get_logger(),
                           "transformed_corners: " << outPts);

        std::vector<cv::Point2i> outPtsInt;
        outPtsInt.push_back(outPts[0]);
        outPtsInt.push_back(outPts[1]);
        outPtsInt.push_back(outPts[2]);
        outPtsInt.push_back(outPts[3]);

        cv::Scalar cvColor(0, 0, 255);
        cv::polylines(img.image, outPtsInt, true, cvColor, 3);

        // Objekt-ID als Text einfügen
        cv::putText(img.image, id, cv::Point2i(0,30), cv::FONT_HERSHEY_SIMPLEX, 1,
                    cvColor, 2);

        RCLCPP_INFO_STREAM(this->get_logger(), "Draw Object: " << id);
      }

      imagePub_.publish(img.toImageMsg());
      RCLCPP_INFO_STREAM(this->get_logger(), "publish");
    }
  }

private:
  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image, find_object_2d::msg::DetectionInfo>
      MyExactSyncPolicy;
  message_filters::Synchronizer<MyExactSyncPolicy> *exactSync_;
  image_transport::Publisher imagePub_;
  image_transport::SubscriberFilter imageSub_;
  message_filters::Subscriber<find_object_2d::msg::DetectionInfo> objectsSub_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PrintObjects>());
	rclcpp::shutdown();

    return 0;
}
