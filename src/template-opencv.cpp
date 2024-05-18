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

// Include fstream & iostream
#include <iostream>
#include <fstream>

#include "ImageDenoiser.hpp"

// Lower threshold for detecting blue cones
cv::Scalar blueLow = cv::Scalar(107, 111, 45);
// Higher threshold for detecting blue cones
cv::Scalar blueHigh = cv::Scalar(140, 155, 86);
// Lower threshold for detecting yellow cones
cv::Scalar yellowLow = cv::Scalar(11, 20, 128);
// Higher threshold for detecting yellow cones
cv::Scalar yellowHigh = cv::Scalar(54, 198, 232);

// Blue threshold
int blueThreshold = 30;
// Blue max value
int blueMaxValue = 255;

// Yellow threshold
int yellowThreshold = 30;
// Yellow max value
int yellowMaxValue = 255;

// PID constants
double kP = 0.005;
double kI = 0;
double kD = 0;

// Callback function to avoid the warnings
static void onBlueTrackbar(int value, void *userdata) {
    int trackbarIndex = reinterpret_cast<intptr_t>(userdata);

    switch (trackbarIndex) {
    case 0:
        // Hue Low
        blueLow[0] = value;
        break;
    case 1:
        // Hue High
        blueHigh[0] = value;
        break;
    case 2:
        // Saturation Low
        blueLow[1] = value;
        break;
    case 3:
        // Saturation High
        blueHigh[1] = value;
        break;
    case 4:
        // Value Low
        blueLow[2] = value;
        break;
    case 5:
        // Value High
        blueHigh[2] = value;
        break;
    case 6:
        // Threshold
        blueThreshold = value;
        break;
    case 7:
        // Max value
        blueMaxValue = value;
        break;
    default:
        break;
    }
}

static void onYellowTrackbar(int value, void *userdata)
{
    int trackbarIndex = reinterpret_cast<intptr_t>(userdata);

    switch (trackbarIndex)
    {
    case 0:
        // Hue Low
        yellowLow[0] = value;
        break;
    case 1:
        // Hue High
        yellowHigh[0] = value;
        break;
    case 2:
        // Saturation Low
        yellowLow[1] = value;
        break;
    case 3:
        // Saturation High
        yellowHigh[1] = value;
        break;
    case 4:
        // Value Low
        yellowLow[2] = value;
        break;
    case 5:
        // Value High
        yellowHigh[2] = value;
        break;
    case 6:
        // Threshold
        yellowThreshold = value;
        break;
    case 7:
        // Max value
        yellowMaxValue = value;
        break;
    default:
        break;
    }
}

static void onPIDTrackbar(int value, void *userdata) {
    int trackbarIndex = reinterpret_cast<intptr_t>(userdata);

    switch (trackbarIndex) {
    case 0:
        // Proportional
        kP = value / 10000.0;
        break;
    case 1:
        // Integral
        kI = value / 10000.0;
        break;
    case 2:
        // Derivative
        kD = value / 10000.0;
        break;
    default:
        break;
    }
}

int32_t main(int32_t argc, char **argv) {
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
    else {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const bool BLUE{commandlineArguments.count("blue") != 0};
        const bool YELLOW{commandlineArguments.count("yellow") != 0};
        const bool PID{commandlineArguments.count("pid") != 0};

        // If the blue command argument is passed, we debug the blue detection
        if (BLUE) {
            cv::namedWindow("Mask Blue", cv::WINDOW_NORMAL);

            cv::createTrackbar("Hue - low", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(0));
            cv::setTrackbarPos("Hue - low", "Mask Blue", static_cast<int>(blueLow[0]));

            cv::createTrackbar("Hue - high", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(1));
            cv::setTrackbarPos("Hue - high", "Mask Blue", static_cast<int>(blueHigh[0]));

            cv::createTrackbar("Sat - low", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(2));
            cv::setTrackbarPos("Sat - low", "Mask Blue", static_cast<int>(blueLow[1]));

            cv::createTrackbar("Sat - high", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(3));
            cv::setTrackbarPos("Sat - high", "Mask Blue", static_cast<int>(blueHigh[1]));

            cv::createTrackbar("Val - low", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(4));
            cv::setTrackbarPos("Val - low", "Mask Blue", static_cast<int>(blueLow[2]));

            cv::createTrackbar("Val - high", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(5));
            cv::setTrackbarPos("Val - high", "Mask Blue", static_cast<int>(blueHigh[2]));

            cv::namedWindow("Processed Blue", cv::WINDOW_NORMAL);

            cv::createTrackbar("Threshold", "Processed Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(6));
            cv::setTrackbarPos("Threshold", "Processed Blue", blueThreshold);

            cv::createTrackbar("Max Value", "Processed Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(7));
            cv::setTrackbarPos("Max Value", "Processed Blue", blueMaxValue);
        }

        // If the yellow command argument is passed, we debug the yellow detection
        if (YELLOW) {
            cv::namedWindow("Mask Yellow", cv::WINDOW_NORMAL);

            cv::createTrackbar("Hue - low", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(0));
            cv::setTrackbarPos("Hue - low", "Mask Yellow", static_cast<int>(yellowLow[0]));

            cv::createTrackbar("Hue - high", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(1));
            cv::setTrackbarPos("Hue - high", "Mask Yellow", static_cast<int>(yellowHigh[0]));

            cv::createTrackbar("Sat - low", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(2));
            cv::setTrackbarPos("Sat - low", "Mask Yellow", static_cast<int>(yellowLow[1]));

            cv::createTrackbar("Sat - high", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(3));
            cv::setTrackbarPos("Sat - high", "Mask Yellow", static_cast<int>(yellowHigh[1]));

            cv::createTrackbar("Val - low", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(4));
            cv::setTrackbarPos("Val - low", "Mask Yellow", static_cast<int>(yellowLow[2]));

            cv::createTrackbar("Val - high", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(5));
            cv::setTrackbarPos("Val - high", "Mask Yellow", static_cast<int>(yellowHigh[2]));

            cv::namedWindow("Processed Yellow", cv::WINDOW_NORMAL);

            cv::createTrackbar("Threshold", "Processed Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(6));
            cv::setTrackbarPos("Threshold", "Processed Yellow", yellowThreshold);

            cv::createTrackbar("Max Value", "Processed Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(7));
            cv::setTrackbarPos("Max Value", "Processed Yellow", yellowMaxValue);
        }

        if (PID) {
            cv::namedWindow("PID", cv::WINDOW_NORMAL);

            cv::createTrackbar("Proportional", "PID", NULL, 10000, onPIDTrackbar, reinterpret_cast<void *>(0));
            cv::setTrackbarPos("Proportional", "PID", 0);

            cv::createTrackbar("Integral", "PID", NULL, 10000, onPIDTrackbar, reinterpret_cast<void *>(1));
            cv::setTrackbarPos("Integral", "PID", 0);

            cv::createTrackbar("Derivative", "PID", NULL, 10000, onPIDTrackbar, reinterpret_cast<void *>(2));
            cv::setTrackbarPos("Derivative", "PID", 0);
        }

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
                // std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
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

            // Get the angular velocity data
            opendlv::proxy::AngularVelocityReading angularVelocity;
            std::mutex angularVelocityMutex;
            auto onAngularVelocityReading = [&angularVelocity, &angularVelocityMutex](cluon::data::Envelope &&env)
            {
                std::lock_guard<std::mutex> lck(angularVelocityMutex);
                angularVelocity = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(env));
            };

            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
            od4.dataTrigger(opendlv::proxy::AngularVelocityReading::ID(), onAngularVelocityReading);

            double error;
            double previousError = 0;
            double steadyStateError = 0;
            double rateOfChangeError = 0;

            double angular;

            bool isCalibrated = false;
            bool isLeftBlue = true;

            // Initialize fstream for storing frame by frame values
            std::ofstream fout;
            fout.open("/tmp/output.csv");
            fout << "sampleTimeStamp;groundSteering;output" << std::endl;

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning())
            {
                // OpenCV data structure to hold an image.
                cv::Mat outputImage;

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
                    outputImage = wrapped.clone();

                    // Add TimeStamp
                    timeStamp = sharedMemory->getTimeStamp();
                }

                sharedMemory->unlock();

                // Variable for the center bottom of the image
                cv::Point imageCenter = cv::Point(WIDTH/2, HEIGHT);
                
                // Create a region of interest (ROI) to focus on the bottom part of the image
                int roiHeight = outputImage.rows - 230;
                cv::Rect roi(0, 230, outputImage.cols, roiHeight); // x, y, width, height
                cv::Mat imageROI = outputImage(roi);
                
                // Make a copy of the image
                cv::Mat blueImage;
                cv::Mat yellowImage;

                // Change the original image into HSV
                cv::cvtColor(imageROI, blueImage, cv::COLOR_BGR2HSV);
                cv::cvtColor(imageROI, yellowImage, cv::COLOR_BGR2HSV);

                // Create masked images
                cv::Mat maskBlue;
                cv::Mat maskYellow;

                // Get pixels that are in range for blue cones
                cv::inRange(blueImage, blueLow, blueHigh, maskBlue);
                cv::inRange(yellowImage, yellowLow, yellowHigh, maskYellow);

                // Create processed images
                cv::Mat processedBlue;
                cv::Mat processedYellow;

                // Denoise processed images
                ImageDenoiser::denoiseImage(imageROI, maskBlue, processedBlue, blueThreshold, blueMaxValue);
                ImageDenoiser::denoiseImage(imageROI, maskYellow, processedYellow, yellowThreshold, yellowMaxValue);

                // Initialize an array of contours
                std::vector<std::vector<cv::Point>> contoursBlue;
                std::vector<std::vector<cv::Point>> contoursYellow;

                // Find contours from the mask
                cv::findContours(processedBlue, contoursBlue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::findContours(processedYellow, contoursYellow, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                // Angular velocity data
                {
                    std::lock_guard<std::mutex> lck(angularVelocityMutex);

                    angular = angularVelocity.angularVelocityZ();
                }

                // TODO: Implement calibration step here
                // if not calibrated, go straight and continue to the next loop iteration
                // if two types of cones exist on screen, calibrate it as it should be
                // if only one type of cone, use angular velocity
                // if no cone, go straight
                if (isCalibrated == false) {
                    //std::cout << "Calibrating..." << std::endl;
                    // If the size of the blue and yellow contours is greater than 0
                    // Check which cones are on the left and right side of the track
                    if (contoursBlue.size() != 0 && contoursYellow.size() != 0) {
                        // Check if the blue cones are on the left side of the track
                        //std::cout << "Both cones are detected" << std::endl;

                        if (contoursBlue[0][0].x > contoursYellow[0][0].x) {                            
                            // Yellow cones are on the left side
                            isLeftBlue = false;
                        }
                        // Set the calibration flag to true
                        isCalibrated = true;
                    }
                    // Else if there is only one type of cones
                    else {
                        std::cout << "Only one type of cones detected" << std::endl;
                        // If we only detect blue cones
                        if (contoursBlue.size() != 0) {
                            std::cout << "Only blue cones detected" << std::endl;
                            // Check angular velocity
                            if (angular > 0) {
                                // Blue cones are on the right side
                                isLeftBlue = false;
                            } else {
                                // Blue cones are on the left side
                                isLeftBlue = true;
                            }
                            isCalibrated = true;
                        }
                        // Else if we only detect yellow cones
                        else if (contoursYellow.size() != 0) {
                            std::cout << "Only yellow cones detected" << std::endl;
                            // Check angular velocity
                            if (angular > 0) {
                                // Yellow cones are on the right side
                                isLeftBlue = true;
                            } else {
                                // Yellow cones are on the left side
                                isLeftBlue = false;
                            }
                            isCalibrated = true;
                        }
                    }
                }

                //std::cout << "isLeftBlue: " << isLeftBlue << std::endl;

                // This runs once the calibration is done

                // Declare variables to keep track of the average distance to the left part and right part of the track
                double averageDistanceBlue = 0;
                double averageDistanceYellow = 0;

                // Iterate through the blue contours
                for(size_t i = 0; i < contoursBlue.size(); i++){
                    // Create a rectangle out of the vectors
                    cv::Rect rect = cv::boundingRect(contoursBlue[i]);

                    // Adjust the rectangle to the ROI
                    rect.y += 230;

                    // Check if the rectangle is not really small
                    if(rect.area() > 100) {
                        // Draw the rectangle on the output image 
                        // Start point
                        cv::Point center = (rect.tl() + rect.br()) / 2;
                        cv::line(outputImage, center, imageCenter, cv::Scalar(0, 255, 0), 3);
                        cv::rectangle(outputImage, rect.tl(), rect.br(), cv::Scalar(255, 0, 0), 2);

                        // Add the distance from the car to the center of a cone
                        averageDistanceBlue += cv::norm(imageCenter - center);
                    }
                }

                // Divide by the number of blue cones to get the average distance
                if (contoursBlue.size() != 0) {
                    averageDistanceBlue /= contoursBlue.size();
                } else {
                    averageDistanceBlue = 0;
                }

                // Iterate through the yellow contours
                for(size_t i = 0; i < contoursYellow.size(); i++){
                    // Create a rectangle out of the vectors
                    cv::Rect rect = cv::boundingRect(contoursYellow[i]);

                    // Adjust the rectangle to the ROI
                    rect.y += 230;
                    
                    // Check if the rectangle is not really small
                    if(rect.area() > 100 && rect.y < 450 && (rect.x > 390 || rect.x < 340)){
                        // Draw the rectangle on the output image
                        cv::Point center = (rect.tl() + rect.br()) / 2;
                        cv::line(outputImage, center, imageCenter, cv::Scalar(0, 255, 0), 3);
                        cv::rectangle(outputImage, rect.tl(), rect.br(), cv::Scalar(0, 255, 255), 2);

                        // Add the distance from the car to the center of a cone
                        averageDistanceYellow += cv::norm(imageCenter - center);
                    }
                }

                // Divide by the number of yellow cones to get the average distance
                if (contoursYellow.size() != 0) {
                    averageDistanceYellow /= contoursYellow.size();
                } else {
                    averageDistanceYellow = 0;
                }

                // TODO: If the blue cones are in the left,
                // B | --- x ------- | Y -> (R - L) > 0 -> we want to turn right, but it won't because turning right needs < 0
                // B | --- x ------- | Y -> (L - R) < 0 -> we want to turn right, and it is correct
                // negative -> turn right, positive -> turn left

                // B | ------- x --- | Y -> (R - L) < 0 -> turn left, but it's not because left is > 0
                // B | ------- x --- | Y -> (L - R) > 0 -> turn left, which is correct


                if (isLeftBlue) {
                    error = averageDistanceBlue - averageDistanceYellow;
                } else {
                    error = averageDistanceYellow - averageDistanceBlue;
                }
                
                // Calculate the error based on the average distances
                // error = averageDistanceBlue - averageDistanceYellow; // Error in pixels
                
                steadyStateError += error;
                rateOfChangeError = error - previousError;

                // PID Controller
                double Proportional = error * kP;
                double Integral = steadyStateError * kI;
                double Derivative = rateOfChangeError * kD;

                // The output goes into the ground steering request
                double output = Proportional + Integral + Derivative;
                
                // LENA'S CODE
                if (isLeftBlue) {
                    if (averageDistanceBlue == 0) output = 1;
                    else if (averageDistanceYellow == 0) output = -1;
                }
                else {
                    if (averageDistanceBlue == 0) output = -1;
                    else if (averageDistanceYellow == 0) output = 1;
                }
                // END OF LENA'S CODE

                if (output > 0.22107488) output = 0.22107488;
                else if (output < -0.22107488) output = -0.22107488;

                // Update the previous error
                previousError = error;

                if (timeStamp.first) {
                    currentTimeStamp = cluon::time::toMicroseconds(timeStamp.second);
                }

                float ground;
                float distance;

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    ground = gsr.groundSteering();
                    // std::cout << "main: groundSteering = " << ground << std::endl;
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
                cv::putText(outputImage, "Insane Raccoons", cv::Point(200, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(36, 0, 201), 1);

                std::stringstream metadataStream;
                metadataStream << "Now:" << std::put_time(gmtime, "%Y-%m-%dT%H:%M:%SZ") << "; ts:" << std::to_string(currentTimeStamp) << "; ";
                std::string overlayMetadata = metadataStream.str();

                cv::putText(outputImage, overlayMetadata, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(36, 0, 201), 1);

                // OVERLAY DISTANCE
                std::stringstream distanceStream;
                distanceStream << "Distance: " << distance << " [meters]";
                std::string overlayDistance = distanceStream.str();

                cv::putText(outputImage, overlayDistance, cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(36, 0, 201), 1);

                // OVERLAY GROUND
                std::stringstream groundStream;
                groundStream << "Ground Steering: " << ground;
                std::string overlayGround = groundStream.str();

                cv::putText(outputImage, overlayGround, cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(36, 0, 201), 1);

                // Display image on your screen.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), outputImage);
                    cv::imshow("ROI", imageROI);

                    if (BLUE)
                    {
                        cv::imshow("Mask Blue", maskBlue);
                        cv::imshow("Processed Blue", processedBlue);
                    }

                    if (YELLOW)
                    {
                        cv::imshow("Mask Yellow", maskYellow);
                        cv::imshow("Processed Yellow", processedYellow);
                    }

                    cv::waitKey(1);
                }

                // std::cout << "group_18;" << std::to_string(currentTimeStamp) << ";" << output << std::endl;
                std::cout << output << ", original: " << ground << std::endl;
                fout << std::to_string(currentTimeStamp) << ";" << ground << ";" << output << std::endl;
            }

            fout.close();
        }
        retCode = 0;
    }
    return retCode;
}
