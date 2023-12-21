/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// #include <publisher_factory.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory> 
#include <rclcpp/rclcpp.hpp>
#include <string> 
#include <thread>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "safedataaccess.h"
#include "std_msgs/msg/string.hpp"
#include "v4l2Image_msg.h"
#include "publisher_factory.h"
// #include "network.h"

#define FRAME_SIZE 1920 * 1080

using namespace std::chrono;
using namespace std::chrono_literals;

// Create the node class named MinimalPublisher which inherits the attributes
// and methods of the rclcpp::Node class.

std::string * parseArgs(int argc, char ** argv){
        return nullptr;
}

class GmslNode : public rclcpp::Node
{
private:
        // Initializing camera and establishing connection
        uint16_t **frame;
        PublisherFactory publishers;
        // camera parameters
        bool m_video_thread;
        std::vector<std::string> m_videoDevices;


public:
        SafeDataAccess<uint16_t *> m_safeDataAccess;

private:
        rcl_interfaces::msg::SetParametersResult parameterCallback(
            const std::vector<rclcpp::Parameter> &parameters)
        {
                publishers.setThreadMode("video", get_parameter("video").as_bool());

                m_video_thread = get_parameter("video").as_bool();

                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                result.reason = "success";

                return result;
        }

        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

public:
        GmslNode(std::string *arguments, std::vector<std::string> videoDevices)
            : Node("tof_camera_node")
        {
                m_video_thread = true;

                this->declare_parameter("video", true);
                this->get_parameter("video", m_video_thread);
                m_videoDevices = videoDevices;

                for (int i = 0; i <= 12; i++)
                {
                        m_safeDataAccess.populateData(new uint16_t[FRAME_SIZE]);
                }

                if (!streamOnFlag)
                {
                        streamOnFlag = true;
                }

                publishers.createNew(this, videoDevices, frame, &m_safeDataAccess);

                callback_handle_ = this->add_on_set_parameters_callback(
                    std::bind(&GmslNode::parameterCallback, this, std::placeholders::_1));

                if (
                    (std::strcmp(arguments[3].c_str(), "True") == 0) ||
                    (std::strcmp(arguments[3].c_str(), "true") == 0))
                        publishers.createMultiThreadPublisherWorkers(frame);
                else
                        publishers.createSingleThreadPublisherWorker(frame);
        }

        void service_callback()
        {
                if (streamOnFlag)
                {
                        globalTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();

                        uint16_t *tmp1 = m_safeDataAccess.getNextElement();

                        m_safeDataAccess.setReadytoStart();

                        //       getNewFrame(camera, tmp1);
                        //       TO DO: get frame from server
                        for (int i = 0; i < FRAME_SIZE; i++)
                        {
                                tmp1[i] = 1;
                        }

                        m_safeDataAccess.addElement(tmp1);
                }
        }

        void stopNode()
        {
                publishers.removePublisherWorkers();
                publishers.deletePublishers(m_videoDevices);
        }
};

int main(int argc, char *argv[])
{
        // Initialize ROS 2
        rclcpp::init(argc, argv);

        std::vector<std::string> videoDevices = {
                "/dev/video0",
                "/dev/video1",
                "/dev/video2",
                "/dev/video3",
        };

        std::string *arguments = parseArgs(argc, argv);

        // Create ToF Node
        std::shared_ptr<GmslNode> gmsl_node = std::make_shared<GmslNode>(arguments, videoDevices);

        while (rclcpp::ok())
        {
                gmsl_node->service_callback();
                rclcpp::spin_some(gmsl_node);
        }

        gmsl_node->stopNode();

        // Shutdown the node when finished
        rclcpp::shutdown();
        return 0;
}