/*
 * Copyright (C) 2024 Christian Berger, Ionel Pop, Adrian Hassa,
 *                        Teodora Portase, Vasilena Karaivanova
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
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

// Include queue library
#include <queue>

// Include ImageDenoiser header file
#include "ImageDenoiser.hpp"

// Define min and max steering angles (+/-24% of max/min original groundSteering angles)
#define MAX_STEERING 0.22107488
#define MIN_STEERING -0.22107488

// Lower bound for detecting blue cones
cv::Scalar blueLow = cv::Scalar(109, 68, 42);
// Upper bound for detecting blue cones
cv::Scalar blueHigh = cv::Scalar(135, 250, 120);
// Lower bound for detecting yellow cones
cv::Scalar yellowLow = cv::Scalar(11, 20, 128);
// Upper bound for detecting yellow cones
cv::Scalar yellowHigh = cv::Scalar(54, 198, 232);

// Blue threshold
int blueThreshold = 30;
// Blue max value
int blueMaxValue = 255;

// Yellow threshold
int yellowThreshold = 30;
// Yellow max value
int yellowMaxValue = 255;

// Size of the frame delay queue
int queueSize = 2;

// Queue to delay our output by 2 frames if the car is moving forward
std::queue<double> steeringQueue;       // Store original groundSteering angles
std::queue<std::time_t> timestampQueue; // Store frame timestamps

int queueCounter = 0; // To count the first elements

// Callback function used as a debug menu when detecting blue cones
static void onBlueTrackbar(int value, void *userdata)
{
    int trackbarIndex = reinterpret_cast<intptr_t>(userdata);

    switch (trackbarIndex)
    {
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

// Callback function used as a debug menu when detecting yellow cones
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
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose [--blue] [--yellow]] " << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "         --verbose: display the image on the screen" << std::endl;
        std::cerr << "         --blue: display a debugging window for detecting blue cones" << std::endl;
        std::cerr << "         --yellow: display a debugging window for detecting yellow cones" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose --blue --yellow" << std::endl;
    }
    else
    {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const bool BLUE{commandlineArguments.count("blue") != 0};
        const bool YELLOW{commandlineArguments.count("yellow") != 0};

        // If the blue command argument is passed, we debug the blue detection
        if (VERBOSE && BLUE)
        {
            cv::namedWindow("Mask Blue", cv::WINDOW_NORMAL);

            // Create a section for editing the lower boundary for hue
            cv::createTrackbar("Hue - low", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(0));
            cv::setTrackbarPos("Hue - low", "Mask Blue", static_cast<int>(blueLow[0]));

            // Create a section for editing the upper boundary for hue
            cv::createTrackbar("Hue - high", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(1));
            cv::setTrackbarPos("Hue - high", "Mask Blue", static_cast<int>(blueHigh[0]));

            // Create a section for editing the lower boundary for saturation
            cv::createTrackbar("Sat - low", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(2));
            cv::setTrackbarPos("Sat - low", "Mask Blue", static_cast<int>(blueLow[1]));

            // Create a section for editing the upper boundary for saturation
            cv::createTrackbar("Sat - high", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(3));
            cv::setTrackbarPos("Sat - high", "Mask Blue", static_cast<int>(blueHigh[1]));

            // Create a section for editing the lower boundary for value
            cv::createTrackbar("Val - low", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(4));
            cv::setTrackbarPos("Val - low", "Mask Blue", static_cast<int>(blueLow[2]));

            // Create a section for editing the upper boundary for value
            cv::createTrackbar("Val - high", "Mask Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(5));
            cv::setTrackbarPos("Val - high", "Mask Blue", static_cast<int>(blueHigh[2]));

            cv::namedWindow("Processed Blue", cv::WINDOW_NORMAL);

            cv::createTrackbar("Threshold", "Processed Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(6));
            cv::setTrackbarPos("Threshold", "Processed Blue", blueThreshold);

            cv::createTrackbar("Max Value", "Processed Blue", NULL, 255, onBlueTrackbar, reinterpret_cast<void *>(7));
            cv::setTrackbarPos("Max Value", "Processed Blue", blueMaxValue);
        }

        // If the yellow command argument is passed, we debug the yellow detection
        if (VERBOSE && YELLOW)
        {
            cv::namedWindow("Mask Yellow", cv::WINDOW_NORMAL);

            // Create a section for editing the lower boundary for hue
            cv::createTrackbar("Hue - low", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(0));
            cv::setTrackbarPos("Hue - low", "Mask Yellow", static_cast<int>(yellowLow[0]));

            // Create a section for editing the upper boundary for hue
            cv::createTrackbar("Hue - high", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(1));
            cv::setTrackbarPos("Hue - high", "Mask Yellow", static_cast<int>(yellowHigh[0]));

            // Create a section for editing the lower boundary for saturation
            cv::createTrackbar("Sat - low", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(2));
            cv::setTrackbarPos("Sat - low", "Mask Yellow", static_cast<int>(yellowLow[1]));

            // Create a section for editing the upper boundary for saturation
            cv::createTrackbar("Sat - high", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(3));
            cv::setTrackbarPos("Sat - high", "Mask Yellow", static_cast<int>(yellowHigh[1]));

            // Create a section for editing the lower boundary for value
            cv::createTrackbar("Val - low", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(4));
            cv::setTrackbarPos("Val - low", "Mask Yellow", static_cast<int>(yellowLow[2]));

            // Create a section for editing the upper boundary for value
            cv::createTrackbar("Val - high", "Mask Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(5));
            cv::setTrackbarPos("Val - high", "Mask Yellow", static_cast<int>(yellowHigh[2]));

            cv::namedWindow("Processed Yellow", cv::WINDOW_NORMAL);

            cv::createTrackbar("Threshold", "Processed Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(6));
            cv::setTrackbarPos("Threshold", "Processed Yellow", yellowThreshold);

            cv::createTrackbar("Max Value", "Processed Yellow", NULL, 255, onYellowTrackbar, reinterpret_cast<void *>(7));
            cv::setTrackbarPos("Max Value", "Processed Yellow", yellowMaxValue);
        }

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid())
        {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Ground stering request
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
            // End of ground stering request

            // Angular Velocity Reading
            opendlv::proxy::AngularVelocityReading angularVelocity;
            std::mutex angularVelocityMutex;
            auto onAngularVelocityReading = [&angularVelocity, &angularVelocityMutex](cluon::data::Envelope &&env)
            {
                std::lock_guard<std::mutex> lck(angularVelocityMutex);
                angularVelocity = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(env));
            };

            od4.dataTrigger(opendlv::proxy::AngularVelocityReading::ID(), onAngularVelocityReading);
            // End of angular velocity reading

            // Initialize fstream for storing frame by frame values for comparison
            std::ofstream fout;
            fout.open("/tmp/output.csv");
            fout << "sampleTimeStamp;groundSteering;output" << std::endl;

            // Previous timestamp
            std::time_t previousTimeStamp = 0;
            // Check if the car is going backwards or forwards
            bool isForward = true;
            // Counter for how many frames have passed
            int frameCounter = 0;

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

                    if (timeStamp.first)
                    {
                        // return 0;
                        // std::cout << "Timestamp detected" << std::endl;
                        if (cluon::time::toMicroseconds(timeStamp.second) == previousTimeStamp)
                        {
                            return 0;
                            break;
                            // std::cout << "Duplicate timestamp detected!" << std::endl;
                        }
                    }
                }
                // TODO: Here, you can add some code to check the sampleTimePoint when the current frame was captured.
                sharedMemory->unlock();

                // Variable for the center bottom of the image
                cv::Point imageCenter = cv::Point(WIDTH / 2, HEIGHT);

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

                // Declare variables to keep track of the average distance to the left part and right part of the track
                double averageDistanceLeft = 0;
                double averageDistanceRight = 0;

                // Iterate through the blue contours
                for (size_t i = 0; i < contoursBlue.size(); i++)
                {
                    // Create a rectangle out of the vectors
                    cv::Rect rect = cv::boundingRect(contoursBlue[i]);

                    // Adjust the rectangle to the ROI
                    rect.y += 230;

                    // Check if the rectangle is not really small
                    if (rect.area() > 100)
                    {
                        // Draw the rectangle on the output image
                        cv::Point center = (rect.tl() + rect.br()) / 2; // Start point
                        cv::line(outputImage, center, imageCenter, cv::Scalar(0, 255, 0), 3);
                        cv::rectangle(outputImage, rect.tl(), rect.br(), cv::Scalar(255, 0, 0), 2);

                        // Add the distance from the car to the center of a cone
                        averageDistanceLeft += cv::norm(imageCenter - center);
                    }
                }

                // Divide by the number of blue cones to get the average distance
                if (contoursBlue.size() != 0)
                {
                    averageDistanceLeft /= contoursBlue.size();
                }
                else
                {
                    averageDistanceLeft = 0;
                }

                // Iterate through the yellow contours
                for (size_t i = 0; i < contoursYellow.size(); i++)
                {
                    // Create a rectangle out of the vectors
                    cv::Rect rect = cv::boundingRect(contoursYellow[i]);

                    // Adjust the rectangle to the ROI
                    rect.y += 230;

                    // Check if the rectangle is not really small
                    if (rect.area() > 100 && rect.y < 450 && (rect.x > 390 || rect.x < 340))
                    {
                        // Draw the rectangle on the output image
                        cv::Point center = (rect.tl() + rect.br()) / 2;
                        cv::line(outputImage, center, imageCenter, cv::Scalar(0, 255, 0), 3);
                        cv::rectangle(outputImage, rect.tl(), rect.br(), cv::Scalar(0, 255, 255), 2);

                        // Add the distance from the car to the center of a cone
                        averageDistanceRight += cv::norm(imageCenter - center);
                    }
                }

                // Divide by the number of yellow cones to get the average distance
                if (contoursYellow.size() != 0)
                {
                    averageDistanceRight /= contoursYellow.size();
                }
                else
                {
                    averageDistanceRight = 0;
                }

                if (timeStamp.first)
                {
                    currentTimeStamp = cluon::time::toMicroseconds(timeStamp.second);
                }

                // Check if the video is played forwards or backwards
                // After frame 2, we determine the direction and we proceed with the rest of the steps
                // We assume that it's going forward at first
                if (frameCounter < 1)
                {
                    frameCounter++;
                    previousTimeStamp = currentTimeStamp;
                }
                else if (frameCounter == 1)
                {
                    if (previousTimeStamp < currentTimeStamp)
                    {
                        isForward = true;
                    }
                    else
                    {
                        isForward = false;
                    }
                }

                float ground;   // Original GroundSteeringRequest we want to match
                double angular; // AngularVelocity gotten from sensor data

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    ground = gsr.groundSteering();
                }

                // Angular velocity data
                {
                    std::lock_guard<std::mutex> lck(angularVelocityMutex);
                    angular = angularVelocity.angularVelocityZ();
                }

                // Add overlay for current date and time in UTC format
                cluon::data::TimeStamp now = cluon::time::now();

                std::time_t currentTimeSec = cluon::time::toMicroseconds(now) / 1000000; // Convert microseconds to seconds
                std::tm *gmtime = std::gmtime(&currentTimeSec);                          // Convert time_t to tm as UTC time

                // OVERLAY METADATA
                cv::putText(outputImage, "Group 18", cv::Point(200, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(36, 0, 201), 1);

                std::stringstream metadataStream;
                metadataStream << "Now:" << std::put_time(gmtime, "%Y-%m-%dT%H:%M:%SZ") << "; ts:" << std::to_string(currentTimeStamp) << "; ";
                std::string overlayMetadata = metadataStream.str();

                cv::putText(outputImage, overlayMetadata, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(36, 0, 201), 1);

                // OVERLAY GROUND
                std::stringstream groundStream;
                groundStream << "Ground Steering: " << ground;
                std::string overlayGround = groundStream.str();
                cv::putText(outputImage, overlayGround, cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(36, 0, 201), 1);

                // OVERLAY ANGULAR VELOCITY
                std::stringstream angularStream;
                angularStream << "Angular velocity: " << angular << " [Z - Axis]";
                std::string overlayAngular = angularStream.str();
                cv::putText(outputImage, overlayAngular, cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(36, 0, 201), 1);

                // Display image on your screen.
                // If the verbose flag is set, display the original image and the ROI image
                if (VERBOSE)
                {
                    cv::imshow(sharedMemory->name().c_str(), outputImage);
                    cv::imshow("ROI", imageROI);

                    // If the blue flag is set, display the blue mask and the processed blue image, as well as sliders to adjust HSV values
                    if (BLUE)
                    {
                        cv::imshow("Mask Blue", maskBlue);
                        cv::imshow("Processed Blue", processedBlue);
                    }

                    // If the yellow flag is set, display the yellow mask and the processed yellow image, as well as sliders to adjust HSV values
                    if (YELLOW)
                    {
                        cv::imshow("Mask Yellow", maskYellow);
                        cv::imshow("Processed Yellow", processedYellow);
                    }

                    cv::waitKey(1);
                }

                // Divide the angular velocity by approximately 100 and multiply by 0.3
                // The minimum and maximum values for angularVelocityZ are -101.2573 and 111.0229
                // The values are different than exactly 100, so we divide by 100 -(-1+11) = 90
                // However, after playing around with that value, we found that 86 has the best accuracy
                double output = (angular / 86) * 0.3;

                // Clip the output ground steering angle
                if (output > MAX_STEERING)
                    output = MAX_STEERING;
                else if (output < MIN_STEERING)
                    output = MIN_STEERING;

                // If the video is playing forward, we delay the output by 2 frames
                // If the video is playing backwards, we output the values immediately
                if (isForward == true)
                {
                    // Push the ground steering angle and the timestamp to the queue
                    steeringQueue.push(ground);
                    timestampQueue.push(currentTimeStamp);

                    // Increment the queue counter to delay the first 2 frames
                    if (queueCounter < queueSize)
                    {
                        queueCounter++;
                    }
                    else
                    {
                        if (steeringQueue.empty())
                        {
                            // Output to the console
                            std::cout << "group_18;" << std::to_string(currentTimeStamp) << ";" << output << std::endl;
                            // Output to the csv file
                            fout << std::to_string(currentTimeStamp) << ";" << ground << ";" << output << std::endl;
                        }
                        else
                        {
                            // Output to the console
                            std::cout << "group_18;" << std::to_string(timestampQueue.front()) << ";" << output << std::endl;
                            // Output to the csv file
                            fout << std::to_string(timestampQueue.front()) << ";" << steeringQueue.front() << ";" << output << std::endl;
                            // Pop the first element from the queue to make space for the next frame
                            timestampQueue.pop();
                            steeringQueue.pop();
                        }
                    }
                }
                else
                {
                    // Output to the console
                    std::cout << "group_18;" << std::to_string(currentTimeStamp) << ";" << output << std::endl;
                    // Output to the csv file
                    fout << std::to_string(currentTimeStamp) << ";" << ground << ";" << output << std::endl;
                }
                // Update the previous timestamp variable
                previousTimeStamp = currentTimeStamp;
            }
            fout.close();
        }
        retCode = 0;
    }
    return retCode;
}
