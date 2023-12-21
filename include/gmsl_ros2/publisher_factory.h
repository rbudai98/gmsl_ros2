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
#ifndef PUBLISHER_FACTORY_H
#define PUBLISHER_FACTORY_H

#include <v4l2Image_msg.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>

#include "safedataaccess.h"

using namespace std::chrono;

static bool deletePublisherWorkers = false;
extern bool streamOnFlag;
extern rclcpp::Time globalTimeStamp;

void publisherImgMsgsWorker(
    std::shared_ptr<AdiSensorMsg> imgMsgs,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher,
    uint16_t **frame,
    SafeDataAccess<uint16_t *> *safeDataAccess);
void publisherSingleThreadWorker(
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> imgPublisher,
    std::vector<std::shared_ptr<AdiSensorMsg>> imgMsgs,
    uint16_t **frame,
    SafeDataAccess<uint16_t *> *safeDataAccess);

class PublisherFactory
{
public:
        PublisherFactory();
        void createNew(
            rclcpp::Node *node, std::vector<std::string> videoDevices, uint16_t **frame,
            SafeDataAccess<uint16_t *> *safeDataAccess);
        void createSingleThreadPublisherWorker(uint16_t **frame);
        void createMultiThreadPublisherWorkers(uint16_t **frame);
        void removePublisherWorkers();
        void deletePublishers(const std::vector<std::string> videoDevices);

        void setThreadMode(std::string topicIndex, bool mode);

private:
        std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> imgPublishers;
        std::vector<std::shared_ptr<AdiSensorMsg>> imgMsgs;
        std::vector<std::shared_ptr<std::thread>> tofThreads;
        SafeDataAccess<uint16_t *> *m_safeDataAccess;
};

#endif // PUBLISHER_FACTORY_H
