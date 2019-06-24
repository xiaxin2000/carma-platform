/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
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

#include "test_utils.h"

TEST_F(TrajectoryExecutorTestSuite, test_emit_multiple) {
    waitForSubscribers(traj_pub, 1, 500);
    cav_msgs::TrajectoryPlan plan = buildSampleTraj();

    traj_pub.publish(plan);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    ASSERT_EQ(10, msg_count) << "Failed to receive whole trajectory from TrajectoryExecutor node.";
    ASSERT_TRUE(shrinking) << "Output trajectory plans were not shrunk each time step as expected.";
}

TEST_F(TrajectoryExecutorTestSuite, test_control_plugin_not_found) {
    waitForSubscribers(traj_pub, 1, 500);
    cav_msgs::TrajectoryPlan plan = buildSampleTraj();

    plan.trajectory_points[0].controller_plugin_name = "NULL";
    traj_pub.publish(plan);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    boost::shared_ptr<const cav_msgs::SystemAlert> msg = ros::topic::waitForMessage<cav_msgs::SystemAlert>("system_alert", ros::Duration(1.5));
    ASSERT_FALSE(!msg) << "Failed to receive system shutdown alert message from TrajectoryExecutor node.";
}

int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_executor_test");

    std::thread spinner([] {while (ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
