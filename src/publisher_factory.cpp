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

#include "publisher_factory.h"

bool streamOnFlag;
rclcpp::Time globalTimeStamp;

PublisherFactory::PublisherFactory(){};

void PublisherFactory::createNew(
    rclcpp::Node *node, std::vector<std::string> videoDevices, uint16_t **frame,
    SafeDataAccess<uint16_t *> *safeDataAccess)
{
        m_safeDataAccess = safeDataAccess;
        for (auto iter : videoDevices)
        {
                imgPublishers.emplace_back(
                    node->create_publisher<sensor_msgs::msg::Image>("iter", 2));
                imgMsgs.emplace_back(new v4l2ImageMsg(iter, frame, sensor_msgs::image_encodings::YUV422));
                std::cout << "Added v4l2 " << iter << " video publisher";
        }
}

void PublisherFactory::setThreadMode(std::string topicIndex, bool mode)
{
        int size = imgPublishers.size();
        for (int i = 0; i < size; i++)
        {
                const std::string topicname = std::string(imgPublishers.at(i)->get_topic_name());
                if (strstr(topicname.c_str(), topicIndex.c_str()))
                {
                        imgMsgs.at(i)->publisherEnabled = mode;
                }
        }
}

void PublisherFactory::createMultiThreadPublisherWorkers(uint16_t **frame)
{
        deletePublisherWorkers = false;
        for (unsigned int i = 0; i < imgMsgs.size(); ++i)
        {
                tofThreads.emplace_back(new std::thread(
                    publisherImgMsgsWorker, imgMsgs.at(i), imgPublishers.at(i), frame, m_safeDataAccess));
        }
}

void PublisherFactory::createSingleThreadPublisherWorker(uint16_t **frame)
{
        deletePublisherWorkers = false;
        tofThreads.emplace_back(new std::thread(
            publisherSingleThreadWorker, imgPublishers, imgMsgs,
            frame, m_safeDataAccess));
}

void PublisherFactory::removePublisherWorkers()
{
        deletePublisherWorkers = true;
        for (int i = 0; i < tofThreads.size(); i++)
                tofThreads[i]->join();
}

void PublisherFactory::deletePublishers(const std::vector<std::string> videoDevices)
{
        //   stopvideoDevice(videoDevice);
        imgPublishers.clear();
        imgMsgs.clear();
}

void publisherImgMsgsWorker(
    std::shared_ptr<AdiSensorMsg> imgMsgs,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher, uint16_t **frame,
    SafeDataAccess<uint16_t *> *safeDataAccess)
{
        rclcpp::Time localTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();
        rclcpp::Rate loop_rate(1.0 / 10);

        while (!deletePublisherWorkers)
        {
                if (rclcpp::ok() && streamOnFlag && imgMsgs->publisherEnabled)
                {
                        localTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();
                        uint16_t *fr = safeDataAccess->getCurrentElement();

                        if (fr != nullptr)
                        {
                                imgMsgs->FrameDataToMsg(fr, localTimeStamp);
                                imgMsgs->publishMsg(*img_publisher);
                        }
                }
                else
                {
                        std::stringstream ss;
                        ss << std::this_thread::get_id();
                        uint64_t id = std::stoull(ss.str());

                        std::cout << "Thread sleep id=" << id;
                        loop_rate.sleep();
                }
        }
}

void publisherSingleThreadWorker(
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> imgPublishers,
    std::vector<std::shared_ptr<AdiSensorMsg>> imgMsgs, uint16_t **frame,
    SafeDataAccess<uint16_t *> *safeDataAccess)
{
        rclcpp::Time localTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();

        while (!deletePublisherWorkers)
        {
                if (streamOnFlag)
                {
                        localTimeStamp = rclcpp::Clock{RCL_ROS_TIME}.now();
                        for (unsigned int i = 0; i < imgMsgs.size(); ++i)
                        {
                                if (rclcpp::ok() && imgMsgs.at(i)->publisherEnabled)
                                {
                                        uint16_t *fr = safeDataAccess->getCurrentElement();

                                        if (fr != nullptr)
                                        {
                                                imgMsgs.at(i)->FrameDataToMsg(fr, localTimeStamp);
                                                imgMsgs.at(i)->publishMsg(*imgPublishers.at(i));
                                        }
                                }
                        }
                }
        }
}
