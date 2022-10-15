/*
 * Copyright (C) 2019-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

/*
 * Developed by the UCLA Mobility Lab, 9/20/2022. 
 * Note: This is a node to publish sythetic emergency vehicle location for road test.
 *
 * Creator: Xu Han
 * Author: Xu Han, Xin Xia, Zonglin Meng, Jiaqi Ma
 */


#include <ros/ros.h>
#include <string>
#include "external_object_list_publisher.h"
#include <array>
#include <stdlib.h> 


namespace external_object_list_publisher
{
    // -------------------------------- Plug-in constructor --------------------------------
    ExternalObjectListPublisher::ExternalObjectListPublisher(
                                                             ExternalObjectListPublisherConfig config, 
                                                             ExternalObjectListCB external_object_list_publisher):
    
    config_(config), 
    external_object_list_publisher_(external_object_list_publisher)
    {
        ROS_DEBUG_STREAM("Top of ExternalObjectListPublisher.");
        long cur_t = ros::Time::now().toNSec()/1000000; // current time in millisecond
        double emergency_vehicle_distance = config_.emergency_vehicle_distance; //desired distance between host vehicle and sythetic emergency vehicle
        ROS_DEBUG_STREAM("ExternalObjectListPublisher complete.");
    }
    



    // enable lane change callback
    void ExternalObjectListPublisher::emergncy_detection_cb(const std_msgs::BoolConstPtr& msg)
    {
        is_emergency_vehicle_detected_ = msg->data;

        // publish external onject if emergency detection result is set to true
        if (is_emergency_vehicle_detected_)
        {
            publish_external_object_list();
        }

    }

    // -------------------------------- Generate maneuver plan --------------------------------
    void ExternalObjectListPublisher::publish_external_object_list()
    {       
        // 3. generate external object message 
        cav_msgs::ExternalObjectList msg;

        // creat one object
        cav_msgs::ExternalObject obj;

        // populate information 
        obj.header.stamp = ros::Time::now();
        
        // Presence vector message is used to describe objects coming from potentially
        // different sources. The presence vector is used to determine what items are set
        // by the producer
        obj.presence_vector = obj.presence_vector | obj.ID_PRESENCE_VECTOR;
        obj.presence_vector = obj.presence_vector | obj.POSE_PRESENCE_VECTOR;
        obj.presence_vector = obj.presence_vector | obj.VELOCITY_PRESENCE_VECTOR;
        obj.presence_vector = obj.presence_vector | obj.SIZE_PRESENCE_VECTOR;
        obj.presence_vector = obj.presence_vector | obj.OBJECT_TYPE_PRESENCE_VECTOR;
        obj.presence_vector = obj.presence_vector | obj.DYNAMIC_OBJ_PRESENCE;
        obj.presence_vector = obj.presence_vector | obj.CONFIDENCE_PRESENCE_VECTOR;

        // ID
        obj.id = 666;
        //pose (todo: using host position, need to change to previous lanelet location) 
        // obj.pose.pose.x = previous_llt_location.x;
        // obj.pose.pose.y = previous_llt_location.y;
        obj.pose.pose = pose_msg_.pose;

        // covariance (a sample covariance matrix)
        boost::array<double, 36> input_covariance = { 
          1,  0,  0,  0,  0, 0,
          0,  1,  0,  0,  0, 0,
          0,  0,  1,  0,  0, 0,
          0,  0,  0,  1,  0, 0, // Since no covariance for the orientation is provided we will assume an identity relationship (1s on the diagonal)
          0,  0,  0,  0,  1, 0, // TODO when autoware suplies this information we should update this to reflect the new covariance
          0,  0,  0,  0,  0, 1
        };
        obj.pose.covariance = input_covariance;

        // confidence 
        obj.confidence = 0.86; 

        //speed
        obj.velocity.twist.linear.x = current_speed_;

        // size
        obj.size.x = 2.0;
        obj.size.y = 2.0;
        obj.size.z = 2.0;

        // type
        obj.object_type = obj.EMERGENCY_VEHICLE;

        // dynamic 
        obj.dynamic_obj = true;

        // 4. populate list
        msg.objects.emplace_back(obj);
        // carma_perception_msgs::msg::ExternalObjectList output_list;
        // output_list.insert(obj);

        // 5. publish 
        external_object_list_publisher_(msg);

    }

    // -------------------------------- Spin Emergency Pullover plugin --------------------------------
    bool ExternalObjectListPublisher::onSpin() 
    {        
        ROS_DEBUG_STREAM("Sythetic external object list publisher is on Spin... ");
        return true;
    }
    
} //external_object_list_publisher