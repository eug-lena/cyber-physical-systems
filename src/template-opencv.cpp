/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Lower threshold for detecting blue cones 
cv::Scalar blueLow = cv::Scalar(105, 70, 46);
// Higher threshold for detecting blue cones 
cv::Scalar blueHigh = cv::Scalar(149, 255, 144);

int32_t main(int32_t argc, char **argv)
{
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("name")) ||
        (0 == commandlineArguments.count("width")) ||
        (0 == commandlineArguments.count("height")))
    {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else
    {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid())
        {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env)
            {
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            // Get the distance sensor data
            opendlv::proxy::DistanceReading distanceReading;
            std::mutex distanceMutex;
            auto onDistanceReading = [&distanceReading, &distanceMutex](cluon::data::Envelope &&env)
            {
                std::lock_guard<std::mutex> lck(distanceMutex);
                distanceReading = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
            };

            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning())
            {
                // OpenCV data structure to hold an image.
                cv::Mat img;

                // TimeStamp variable
                std::time_t currentTimeStamp;
                std::pair<bool, cluon::data::TimeStamp> timeStamp;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();

                    // Add TimeStamp
                    timeStamp = sharedMemory->getTimeStamp();
                }
                // TODO: Here, you can add some code to check the sampleTimePoint when the current frame was captured.
                sharedMemory->unlock();

                // Make a copy of the image
                cv::Mat imgCopy;
                img.copyTo(imgCopy);

                // Change the original image into HSV
                cv::cvtColor(img, img, cv::COLOR_BGR2HSV);

                // Create a mask image
                cv::Mat mask;
                // Get pixels that are in range for blue cones
                cv::inRange(img, blueLow, blueHigh, mask);

                // Initialize an array of contours
                std::vector <std::vector <cv::Point>> contours;
                // Find contours from the mask
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                // Iterate through the contours
                for(int i = 0; i < contours.size(); i++){
                    // Create a rectangle out of the vecotrs
                    cv::Rect rect = cv::boundingRect(contours[i]);
                    // Check if the rectangle is in the lower part of the frame and is not really small
                    if(rect.area() > 100 && rect.y > 230){
                        // Draw the rectangle on the image copy
                        cv::rectangle(imgCopy, rect.tl(), rect.br(), cv::Scalar(0, 0, 255), 2);
                    }
                    
                }

                if (timeStamp.first)
                {
                    currentTimeStamp = cluon::time::toMicroseconds(timeStamp.second);
                }

                float ground;
                float distance;

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    ground = gsr.groundSteering();
                    std::cout << "main: groundSteering = " << ground << std::endl;
                }

                // Distance data
                {
                    std::lock_guard<std::mutex> lck(distanceMutex);

                    distance = distanceReading.distance();
                }

                // Add overlay for current date and time in UTC format
                cluon::data::TimeStamp now = cluon::time::now();

                std::time_t currentTimeSec = cluon::time::toMicroseconds(now) / 1000000; // Convert microseconds to seconds
                std::tm *gmtime = std::gmtime(&currentTimeSec);                          // Convert time_t to tm as UTC time

                // OVERLAY METADATA
                cv::putText(imgCopy, "Insane Raccoons", cv::Point(200, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(36, 0, 201), 1);

                std::stringstream metadataStream;
                metadataStream << "Now:" << std::put_time(gmtime, "%Y-%m-%dT%H:%M:%SZ") << "; ts:" << std::to_string(currentTimeStamp) << "; ";
                std::string overlayMetadata = metadataStream.str();

                cv::putText(imgCopy, overlayMetadata, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(36, 0, 201), 1);

                // OVERLAY DISTANCE
                std::stringstream distanceStream;
                distanceStream << "Distance: " << distance << " [meters]";
                std::string overlayDistance = distanceStream.str();

                cv::putText(imgCopy, overlayDistance, cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(36, 0, 201), 1);

                // OVERLAY GROUND
                std::stringstream groundStream;
                groundStream << "Ground Steering: " << ground;
                std::string overlayGround = groundStream.str();

                cv::putText(imgCopy, overlayGround, cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(36, 0, 201), 1);

                // Display image on your screen.
                if (VERBOSE)
                {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}
