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
 * FOR ANY Dv4l2ECT, INDv4l2ECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "v4l2Image_msg.h"

v4l2ImageMsg::v4l2ImageMsg() {}

v4l2ImageMsg::v4l2ImageMsg(
    const std::string device_name, uint16_t **frame, std::string encoding)
{
        imgEncoding = encoding;
}

void v4l2ImageMsg::FrameDataToMsg( uint16_t *frame,
            rclcpp::Time tStamp)
{

        uint16_t width = 1920;
        uint16_t height = 1080;

        setMetadataMembers(width, height, tStamp);

        // uint16_t *frameData = getFrameData(frame, "");
        uint16_t *frameData;

        if (!frameData)
        {
                std::cout<< "Frame data grabbing failed";
                return;
        }

        setDataMembers(frameData);
}

void v4l2ImageMsg::setMetadataMembers(int width, int height, rclcpp::Time tStamp)
{
        message.header.stamp = tStamp;
        message.header.frame_id = "gmsl_v4l2_img";

        message.width = width;
        message.height = height;

        message.encoding = imgEncoding;
        message.is_bigendian = false;

        int pixelByteCnt = 2;
        message.step = width * pixelByteCnt;

        message.data.resize(message.step * height);
}

void v4l2ImageMsg::setDataMembers(uint16_t *frameData)
{
        // if (message.encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
        // {
                // v4l2To16bitGrayscale(frameData, message.width, message.height);
                message.data.data() = frameData;
                // memcpy(msgDataPtr, frameData, message.step * message.height);
        // }
        // else
                // LOG(ERROR) << "Image encoding invalid or not available";
}

sensor_msgs::msg::Image v4l2ImageMsg::getMessage() { return message; }

void v4l2ImageMsg::publishMsg(rclcpp::Publisher<sensor_msgs::msg::Image> &pub)
{
        pub.publish(message);
}
