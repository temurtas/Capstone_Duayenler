/*Copyright (c) 2019 DUAYENLER Ltd. Sti
 */
/*
 *@copyright Copyright 2019 DUAYENLER Ltd. Sti
 *@file moduleKobra.cpp
 *@author Erdem Tuna
 *@author Ilker Saglik
 *@brief A complete implementation of the autonomous vehicle
 *@brief with handshake capability with other vehicles.
 */
/*** Handshake Libraries and Variables ***/
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include "./headers/vl6180_pi.h"
#include <wiringPi.h>
#include <thread>
#include <mutex>

#define CATCH "00"
#define ACK   "01"
#define STOP  "10"
#define REJ   "11"
#define SCATCH "0700"
#define SACK   "0701"
#define SSTOP  "0710"
#define SREJ   "0711"
#define PORT 5000
#define sensor1Pin 0
#define sensor2Pin 1
#define red     2
#define green   3
#define blue    4
#define yellow  5

char opp_id[] = "00";
char opp_ip[20];

bool RUN = true; //SET THIS TO FALSE TO WORK WITH HANDSHAKE

// sensor related variables
vl6180 fhandle,bhandle;
int freading = 100, breading = 100;
int catchThreshold = 50;
/*** Handshake Libraries ***/

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include <iostream>
//#include <fstream>
//#include <functional> // MIGHT BE UNNECESSARY
#include "./headers/LaneDetector.hpp"
#include "./sources/LaneDetector.cpp"
#include "./headers/ArduinoComm.hpp"
#include "./sources/ArduinoComm.cpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <numeric>

/** HSV PARAMETER SETTINGS **/
const int max_value_H = 255;
const int max_value = 255;
//const cv::String window_orig = "Lane Detection";
const cv::String window_lane_detected = "Lane Detection";
const cv::String winodw_hsv_filtered = "HSV Filtered";
const cv::String window_canny_applied = "Canny Applied";
const cv::String window_masked = "Masked";
const cv::String window_vision = "Lane and Vehicle Vision";

int low_H = 49, low_S = 48, low_V = 130; // low_S = 144,introduces bug
int high_H = max_value_H, high_S = 100, high_V = 188;
int maxGap = 50;
int minLength = 50;
/** HSV PARAMETER SETTINGS **/

/** **/
int threshold = 20;
int maxLineGap = 20;
int minLineLength = 30; //was 30
/** **/

static void on_low_H_thresh_trackbar(int, void *);
static void on_high_H_thresh_trackbar(int, void *);
static void on_low_S_thresh_trackbar(int, void *);
static void on_high_S_thresh_trackbar(int, void *);
static void on_low_V_thresh_trackbar(int, void *);
static void on_high_V_thresh_trackbar(int, void *);

static void on_threshold_thresh_trackbar(int, void *);
static void on_maxLineGap_trackbar(int, void *);
static void on_minLineLength_thresh_trackbar(int, void *);

void quickSort(std::vector<cv::Vec4i> & v, unsigned int low, unsigned int high);
void reverseVector(std::vector<cv::Vec4i> & v);

unsigned int pivot(std::vector<cv::Vec4i> & v, unsigned int start,
                   unsigned int stop, unsigned int position);


/*** Handshake Function prototypes ***/
void print_usage();
void print_arguments(char* type,char* opp_id,char* opp_ip);
void sensor_setup(vl6180* fhandle, vl6180* bhandle);

void sensor_read() {
    while(true) {
        freading = get_distance(fhandle);
        breading = get_distance(bhandle);
        usleep(100000); // slepp 100ms
    }
}
void server_funct();

void client_funct();


bool new_frame = false;
std::mutex edit_frame;
cv::Mat frame_orig;
cv::VideoCapture cap(0, cv::CAP_V4L);
void capture_frame() {
    int frame_counter = 0;
    while (true) {
        if (frame_counter != 1) {
            frame_counter++;
            edit_frame.lock();
            new_frame = false;
            edit_frame.unlock();
        } else {
            cap >> frame_orig;
            frame_counter=0;
            edit_frame.lock();
            new_frame = true;
            edit_frame.unlock();
        }

    }
}

/*** Handshake Function prototypes ***/

/**
 *@brief Function main executes the algorithm of the lane detection.
 *@brief It reads a video of a green lane and it will output the
 *@brief same video scene but with the detected lane.
 *@param argv[] is a string to the full path of the demo video
 *@return flag_plot tells if the demo has sucessfully finished
 */
int main(int argc, char* argv[]) {
    wiringPiSetup();
    sensor_setup(&fhandle, &bhandle);

    //cv::VideoCapture cap("./videos/green-640-18.mp4");
    //cv::VideoCapture cap(0, cv::CAP_V4L);
    LaneDetector lanedetector;  // Create the class object
    //LaneDetector lanedetector;  // Create the class object

    // variables for calculating average process time
    double avgRunTime;
    int avgCounter = 0;
    int const fps = 9;
    double timeCapture;

    // image processing variables
    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat frame_masked;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;
    std::string turn;
    int frame_counter = 0;

    // colors for hough lines
    int redImage = 250;
    int greenImage = 250;

    //Mat whiteFrame(426, 640, CV_8UC3, Scalar(255, 255, 255));

    cv::VideoWriter writer;
    int codec = cv::VideoWriter::fourcc('X', 'V', 'I', 'D'); // select desired codec (must be available at runtime)
    std::string filename = "./laneDetection.avi"; // name of the output video file

    // create output window
    cv::namedWindow(window_lane_detected);

    // create trackbar for editing the HSV filtering

    
    cv::createTrackbar("Low H", window_lane_detected, &low_H, max_value_H,
    		on_low_H_thresh_trackbar);
    cv::createTrackbar("High H", window_lane_detected, &high_H, max_value_H,
    		on_high_H_thresh_trackbar);
    cv::createTrackbar("Low S", window_lane_detected, &low_S, max_value,
    		on_low_S_thresh_trackbar);
    cv::createTrackbar("High S", window_lane_detected, &high_S, max_value,
    		on_high_S_thresh_trackbar);
    cv::createTrackbar("Low V", window_lane_detected, &low_V, max_value,
    		on_low_V_thresh_trackbar);
    cv::createTrackbar("High V", window_lane_detected, &high_V, max_value,
    		on_high_V_thresh_trackbar);

    cv::createTrackbar("threshold", window_lane_detected, &threshold, max_value,
    		on_threshold_thresh_trackbar);
    cv::createTrackbar("maxLineGap", window_lane_detected, &maxLineGap,
    		max_value, on_maxLineGap_trackbar);
    cv::createTrackbar("minLineLength", window_lane_detected, &minLineLength,
    		max_value, on_minLineLength_thresh_trackbar);

    cv::Mat frame_HSV;
    cv::Mat frame_captured, frame_fitered2D, frame_threshed, frame_cannied,
    frame_final;
    cv::Mat frame_houghP;
    std::vector<cv::Vec4i> lines_houghP; // will hold the results of the detection

    cap >> frame_orig;
    writer.open(filename, codec, fps, frame_orig.size(), true);
    ArduinoComm arduinoComm;

    /*** Handhake Thread Calls and Variables ***/
    std::thread serverThread;
    std::thread clientThread;
    std::thread sensorReadThread;
    sensorReadThread = std::thread{sensor_read};
    std::thread cameraThread;
    cameraThread = std::thread{capture_frame};
    char subnet_addr[] = "192.168.1.";
    //char opp_id[] = "00";
    char last_octet[4];
    //char opp_ip[20];
    char type[2];
    int option_index = 0;
    while (( option_index = getopt(argc, argv, "t:o:")) != -1) {
        switch (option_index) {
        case 't':
            strcpy (type, optarg);
            break;
        case 'o':
            strcpy (opp_id, optarg);
            break;
        default:
            print_usage();
        }
    }

    if (opp_id[0] == '0') {
        //printf("if\n");
        //last_octet = new char [2];
        strcpy(last_octet, &opp_id[1]);
    }

    else {
        //printf("else\n");
        //last_octet = new char [3];
        strcpy(last_octet, opp_id);
    }

    strcpy(opp_ip,subnet_addr);

    strcat(opp_ip, last_octet);
    //delete(last_octet);

    print_arguments(type,opp_id,opp_ip);

    if (strcmp(type,"s")==0) {
        printf("I am server.\n");
        //server_funct(opp_id);

        serverThread = std::thread{server_funct};
        //std::thread serverThread(dummy);
    } else if (strcmp(type,"c")==0) {
        printf("I am client.\n");
        //client_funct(opp_id, opp_ip);
        clientThread = std::thread{client_funct};
        //std::thread serverThread(dummy);
    } else {
        printf("I don't know who I am.\n");
        print_usage();
    }
    /*** Handhake Thread Calls and Variables ***/

    // loop through live frames
    while (true) {
        timeCapture = (double) cv::getTickCount(); // capture the starting time


        if(!RUN || (freading < 50)) {
            int stopCode = 1;
            arduinoComm.sendToController("S", std::to_string(stopCode));
            usleep(100000);
        } else if (new_frame) {
            /*
                edit_frame.lock();
            new_frame = false;
            edit_frame.unlock();*/
            frame_counter = 0;
            frame_captured = frame_orig;
            // check if the input video can be opened
            if (frame_captured.empty()) {
                std::cout << "!!! Input video could not be opened" << std::endl;
                return -1;
            }
            avgCounter++; // increment the process counter
            
            lanedetector.updateROVariables(freading);
            // denoise the frame using a Gaussian filter
            img_denoise = lanedetector.deNoise(frame_captured);

            // convert from BGR to HSV colorspace
            // convert from BGR to HSV colorspace
            cv::cvtColor(img_denoise, frame_HSV, cv::COLOR_BGR2Lab);

            // apply color thresholding HSV range for green color
            cv::inRange(frame_HSV, cv::Scalar(low_H, low_S, low_V),
                        cv::Scalar(high_H, high_S, high_V), frame_threshed);

            // frame_threshed is reduced to ROI
            frame_threshed = lanedetector.cropROI(frame_threshed);

            lanedetector.setLineBorders(frame_threshed);

            // canny edge detection to the color thresholded image
            // (50,200,3)
            Canny(frame_threshed, frame_cannied, 133, 400, 5, true);

            // copy cannied image
            cv::cvtColor(frame_cannied, frame_houghP, cv::COLOR_GRAY2BGR);

            frame_masked = lanedetector.cropROI(frame_cannied);
            // runs the line detection
            std::vector<cv::Vec4i> line;
            HoughLinesP(frame_masked, lines_houghP, 1, CV_PI / 180, threshold,
                        (double) maxLineGap, (double) minLineLength);

            if (!lines_houghP.empty()) {
                // sort the found lines from smallest y to largest y coordinate
                quickSort(lines_houghP, 0, lines_houghP.size());
                // reverse the order largest y to smallest y coordinate
                reverseVector(lines_houghP);

                // Separate lines into left and right lines
                left_right_lines = lanedetector.lineSeparation(lines_houghP,
                                   frame_masked);

                // Apply regression to obtain only one line for each side of the lane
                lane = lanedetector.regression(left_right_lines,
                                               frame_threshed);

                // Plot lane detection
                lanedetector.plotLane(frame_captured, lane);
            }
            for (size_t i = 0; i < lines_houghP.size(); i++) {
                cv::Vec4i l = lines_houghP[i];
                if (redImage < 0)
                    redImage = 155;
                if (greenImage < 0)
                    greenImage = 55;
                cv::line(frame_houghP, cv::Point(l[0], l[1]),
                         cv::Point(l[2], l[3]), cv::Scalar(255, greenImage, redImage), 3,
                         cv::LINE_AA);
                redImage = redImage - 20;
                greenImage = greenImage - 20;
            }

            // calculate the process time
            timeCapture = ((double) cv::getTickCount() - timeCapture)
                          / cv::getTickFrequency() * 1000;
            if (avgCounter == fps) {
                std::cout
                        << "The average process time for each 30 frames in millixseconds:     "
                        << (avgRunTime / fps) << std::endl;
                avgCounter = 0;
                avgRunTime = 0;
            } else
                avgRunTime += timeCapture;

            //imshow(window_capture_name, frame_orig);


            imshow(window_lane_detected, frame_houghP);
            imshow(winodw_hsv_filtered, frame_threshed);
            imshow(window_canny_applied, frame_cannied);
            imshow(window_masked, frame_masked);
            imshow(window_vision, frame_captured);

            char key = (char) cv::waitKey(30);
            if (key == 'q' || key == 27) {
                break;
            }

            /*if (!writer.isOpened()) {
            	std::cout << "Could not open the output video file for write\n";
            	return -1;
            }

            writer.write(frame_captured);*/
            redImage = 250;
            greenImage = 250;




            //std::cin.get();
        }

    }


    return 0;
}

static void on_low_H_thresh_trackbar(int, void *) {
    low_H = cv::min(high_H - 1, low_H);
    cv::setTrackbarPos("Low H", window_lane_detected, low_H);
}

static void on_high_H_thresh_trackbar(int, void *) {
    high_H = cv::max(high_H, low_H + 1);
    cv::setTrackbarPos("High H", window_lane_detected, high_H);
}

static void on_low_S_thresh_trackbar(int, void *) {
    low_S = cv::min(high_S - 1, low_S);
    cv::setTrackbarPos("Low S", window_lane_detected, low_S);
}

static void on_high_S_thresh_trackbar(int, void *) {
    high_S = cv::max(high_S, low_S + 1);
    cv::setTrackbarPos("High S", window_lane_detected, high_S);
}

static void on_low_V_thresh_trackbar(int, void *) {
    low_V = cv::min(high_V - 1, low_V);
    cv::setTrackbarPos("Low V", window_lane_detected, low_V);
}

static void on_high_V_thresh_trackbar(int, void *) {
    high_V = cv::max(high_V, low_V + 1);
    cv::setTrackbarPos("High V", window_lane_detected, high_V);
}
static void on_threshold_thresh_trackbar(int, void *) {
    threshold = cv::min(high_V - 1, threshold);
    cv::setTrackbarPos("threshold", window_lane_detected, threshold);
}
static void on_maxLineGap_trackbar(int, void *) {
    maxLineGap = cv::min(high_V - 1, maxLineGap);
    cv::setTrackbarPos("maxLineGap", window_lane_detected, maxLineGap);
}
static void on_minLineLength_thresh_trackbar(int, void *) {
    minLineLength = cv::min(high_V - 1, minLineLength);
    cv::setTrackbarPos("minLineLength", window_lane_detected, minLineLength);
}

unsigned int pivot(std::vector<cv::Vec4i> & v, unsigned int start,
                   unsigned int stop, unsigned int position)
// partition vector into two groups
// values smaller than or equal to pivot
// values larger than pivot
// return location of pivot element
{
    unsigned int target = 1;
    // swap pivot into starting position
    std::swap(v[start], v[position]);

    // partition values
    unsigned int low = start + 1;
    unsigned int high = stop;
    while (low < high)
        if (v[low][target] < v[start][target])
            low++;
        else if (v[--high][target] < v[start][target])
            std::swap(v[low], v[high]);

    // then swap pivot back into place
    std::swap(v[start], v[--low]);
    return low;
}

void reverseVector(std::vector<cv::Vec4i> & v) {
    /*
     for(auto i:v)
     std::cout << i << " ";
     std::cout << std::endl;*/

    std::reverse(v.begin(), v.end());
    /*
     for(auto i:v)
     std::cout << i << " ";
     std::cout << std::endl;*/

    return;
}

void quickSort(std::vector<cv::Vec4i> & v, unsigned int low,
               unsigned int high) {
    if (low >= high)
        return;

    // select the pivot value
    unsigned int pivotIndex = (low + high) / 2;

    // partition the vector
    pivotIndex = pivot(v, low, high, pivotIndex);

    // sort the two sub vectors
    if (low < pivotIndex)
        quickSort(v, low, pivotIndex);
    if (pivotIndex < high)
        quickSort(v, pivotIndex + 1, high);
}
/*** Handshake Functions ***/
void print_usage() {
    printf("Usage: -t <HostType(s/c)> -o <OpponentID(2 digits)>\n");
    exit (1);
}

void print_arguments(char* type,char* opp_id,char* opp_ip) {
    printf("HOST TYPE: %s\n",type);
    printf("OPPONENT ID: %s\n",opp_id);
    printf("OPPONENT IP: %s\n",opp_ip);
}

void sensor_setup(vl6180* fhandle, vl6180* bhandle) {
    pinMode(sensor1Pin, OUTPUT);
    pinMode(sensor2Pin, OUTPUT);
    digitalWrite(sensor1Pin,LOW);
    digitalWrite(sensor2Pin,LOW);

    digitalWrite(sensor1Pin,HIGH);
    delay(50);
    *fhandle = vl6180_initialise(1);
    if(*fhandle<=0) {
        printf("ERROR FOR FRONT HANDLE !\n");
        exit(-1);
    }
    vl6180_change_addr(*fhandle,0x54);

    digitalWrite(sensor2Pin,HIGH);
    delay(50);
    *bhandle = vl6180_initialise(1);
    if(*bhandle<=0) {
        printf("ERROR FOR BACK HANDLE !\n");
        exit(-1);
    }
    vl6180_change_addr(*bhandle,0x56);
}

/*** Handshake Functions ***/
void server_funct() {
    // Define message strings to be sent
    char ctch[5];
    char ack[5];
    char rej[5];
    char stop[5];

    strcpy(ctch, opp_id);
    strcpy(ack, opp_id);
    strcpy(rej, opp_id);
    strcpy(stop, opp_id);

    strcat(ctch, CATCH);
    strcat(ack, ACK);
    strcat(rej, REJ);
    strcat(stop, STOP);

    printf("CATCH = %s\n", ctch);
    printf("ACK = %s\n", ack);
    printf("REJ = %s\n", rej);
    printf("STOP = %s\n", stop);

    /*  if(wiringPiSetup() == -1)
      {
    	printf("setup wiringPi failed !\n");
    	exit(-1);
      }*/
    pinMode(red, OUTPUT);
    pinMode(green, OUTPUT);
    pinMode(blue, OUTPUT);
    pinMode(yellow, OUTPUT);

    digitalWrite (red,  LOW);
    digitalWrite (green,  LOW);
    digitalWrite (blue,  LOW);
    digitalWrite (yellow,  LOW);

    //vl6180 fhandle,bhandle;
    //sensor_setup(&fhandle, &bhandle);
    //int freading,breading;
    struct timeval timeout;
    fd_set rfd;

    //Initial Connection
    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[5];
    memset (buffer, 0x00, 5);

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 5000
    if (bind(server_fd, (struct sockaddr *)&address,
             sizeof(address))<0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                             (socklen_t*)&addrlen))<0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    RUN = true;
    while (1) {
        // Check front sensor data
        //freading = get_distance(fhandle);
        printf ("Front sensor value is: %d\n", freading);
        if (freading <= catchThreshold) {
            send(new_socket, SCATCH, strlen(SCATCH), 0 );
            digitalWrite (red,  LOW);
            digitalWrite (green,  LOW);
            digitalWrite (blue,  LOW);
            digitalWrite (yellow,  LOW);
            digitalWrite (red, HIGH) ;
            FD_ZERO(&rfd);
            FD_SET(new_socket, &rfd);
            timeout.tv_sec =  5;
            timeout.tv_usec = 0;
            printf("Waiting...\n");
            int ret = select(new_socket+1, &rfd, NULL, NULL, &timeout);
            if (ret != 0) {
                // No Time out, i.e. data available, read it
                recv(new_socket, buffer, sizeof(buffer), 0);
                printf("%s is received.\n", buffer );
                if (strcmp(buffer,ack) == 0) {
                    memset (buffer, 0x00, 5);
                    send ( new_socket, SSTOP, strlen(SSTOP), 0 );
                    digitalWrite (red,  LOW);
                    digitalWrite (green,  LOW);
                    digitalWrite (blue,  LOW);
                    digitalWrite (yellow,  LOW);
                    digitalWrite (blue, HIGH) ;
                    RUN = false;
                    break;
                } else if (strcmp(buffer,rej) == 0) {
                    printf ("I was rejected, checking again... \n");
                    memset (buffer, 0x00, 5);
                } else
                    memset (buffer, 0x00, 5);
            }
        } // winning case


        else {


            FD_ZERO(&rfd);
            FD_SET(new_socket, &rfd);
            timeout.tv_sec =  5;
            timeout.tv_usec = 0;

            printf("Waiting...\n");
            int ret = select(new_socket+1, &rfd, NULL, NULL, &timeout);

            if (ret != 0) {
                // No Time out, i.e. data available, read it
                recv(new_socket, buffer, sizeof(buffer), 0);
                printf("%s is received.\n", buffer );

                if (strcmp(buffer,ctch) == 0) {
                    memset (buffer, 0x00, 5);
                    //breading = get_distance(bhandle);
                    printf ("Back sensor value is: %d\n", breading);
                    if (breading <= catchThreshold) {
                        send(new_socket, SACK, strlen(SACK), 0 );
                        digitalWrite (red,  LOW);
                        digitalWrite (green,  LOW);
                        digitalWrite (blue,  LOW);
                        digitalWrite (yellow,  LOW);
                        digitalWrite (green, HIGH) ;
                        FD_ZERO(&rfd);
                        FD_SET(new_socket, &rfd);
                        timeout.tv_sec = 5;
                        timeout.tv_usec = 0;
                        printf("Waiting...\n");
                        ret = select(new_socket+1, &rfd, NULL, NULL, &timeout);
                        if (ret != 0) {
                            recv(new_socket, buffer, sizeof(buffer), 0);
                            printf("%s is received.\n", buffer );
                            if (strcmp(buffer, stop)==0) {
                                RUN = false;
                                memset (buffer, 0x00, 5);
                                break;
                            } else
                                memset (buffer, 0x00, 5);
                        }
                    } else {
                        send(new_socket, SREJ, strlen(SREJ), 0 );
                        digitalWrite (red,  LOW);
                        digitalWrite (green,  LOW);
                        digitalWrite (blue,  LOW);
                        digitalWrite (yellow,  LOW);
                        digitalWrite (yellow, HIGH) ;
                    }
                } else
                    memset (buffer, 0x00, 5);
            }
        } // defeat case
    } // while loop


    return;
}


void client_funct() {
    std::cout << "this is client thread." << std::endl;
    // Define message strings to be sent
    char ctch[5];
    char ack[5];
    char rej[5];
    char stop[5];

    strcpy(ctch, opp_id);
    strcpy(ack, opp_id);
    strcpy(rej, opp_id);
    strcpy(stop, opp_id);

    strcat(ctch, CATCH);
    strcat(ack, ACK);
    strcat(rej, REJ);
    strcat(stop, STOP);

    printf("CATCH = %s\n", ctch);
    printf("ACK = %s\n", ack);
    printf("REJ = %s\n", rej);
    printf("STOP = %s\n", stop);

    /*  if(wiringPiSetup() == -1)
      {
    	printf("setup wiringPi failed !\n");
    	exit(-1);
      }*/

    pinMode(red, OUTPUT);
    pinMode(green, OUTPUT);
    pinMode(blue, OUTPUT);
    pinMode(yellow, OUTPUT);

    digitalWrite (red,  LOW);
    digitalWrite (green,  LOW);
    digitalWrite (blue,  LOW);
    digitalWrite (yellow,  LOW);

    //vl6180 fhandle,bhandle;
    //sensor_setup(&fhandle, &bhandle);
    //int freading,breading;
    struct timeval timeout;
    fd_set rfd;


    //Initial Connection
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;

    char buffer[5];
    memset (buffer, 0x00, 5);
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return;
    }

    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, opp_ip, &serv_addr.sin_addr)<=0) {
        printf("\nInvalid address/ Address not supported \n");
        return;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("\n Connection Failed \n");
        return;
    }
    RUN = true;
    while (1) {
        // Check front sensor data

        //freading = get_distance(fhandle);
        printf ("Front sensor value is: %d\n", freading);
        if (freading<=catchThreshold) {
            send(sock, SCATCH, strlen(SCATCH), 0 );
            digitalWrite (red,  LOW);
            digitalWrite (green,  LOW);
            digitalWrite (blue,  LOW);
            digitalWrite (yellow,  LOW);
            digitalWrite (red, HIGH) ;
            FD_ZERO(&rfd);
            FD_SET(sock, &rfd);
            timeout.tv_sec =  5;
            timeout.tv_usec = 0;
            printf("Waiting...\n");
            int ret = select(sock+1, &rfd, NULL, NULL, &timeout);
            if (ret != 0) {
                // No Time out, i.e. data available, read it
                recv(sock, buffer, sizeof(buffer), 0);
                printf("%s is received.\n", buffer );
                if (strcmp(buffer,ack) == 0) {
                    memset (buffer, 0x00, 5);
                    send(sock, SSTOP, strlen(SSTOP), 0 );
                    digitalWrite (red,  LOW);
                    digitalWrite (green,  LOW);
                    digitalWrite (blue,  LOW);
                    digitalWrite (yellow,  LOW);
                    digitalWrite (blue, HIGH) ;
                    RUN = false;
                    break;
                } else if (strcmp(buffer,rej) == 0) {
                    printf ("I was rejected, checking again... \n");
                    memset (buffer, 0x00, 5);
                } else
                    memset (buffer, 0x00, 5);
            }
        } // winning case


        else {

            FD_ZERO(&rfd);
            FD_SET(sock, &rfd);
            timeout.tv_sec =  5;
            timeout.tv_usec = 0;
            printf("Waiting...\n");
            int ret = select(sock+1, &rfd, NULL, NULL, &timeout);

            if (ret != 0) {
                // No Time out, i.e. data available, read it
                recv(sock, buffer, sizeof(buffer), 0);
                printf("%s is received.\n", buffer );

                if (strcmp(buffer,ctch) == 0) {
                    memset (buffer, 0x00, 5);
                    //breading = get_distance(bhandle);
                    printf ("Back sensor value is: %d\n", breading);
                    if (breading<=catchThreshold) {
                        send(sock, SACK, strlen(SACK), 0 );
                        digitalWrite (red,  LOW);
                        digitalWrite (green,  LOW);
                        digitalWrite (blue,  LOW);
                        digitalWrite (yellow,  LOW);
                        digitalWrite (green, HIGH) ;
                        FD_ZERO(&rfd);
                        FD_SET(sock, &rfd);
                        timeout.tv_sec = 5;
                        timeout.tv_usec = 0;
                        printf("Waiting...\n");
                        ret = select(sock+1, &rfd, NULL, NULL, &timeout);
                        if (ret != 0) {
                            recv(sock, buffer, sizeof(buffer), 0);
                            printf("%s is received.\n", buffer );
                            if (strcmp(buffer,stop) == 0) {
                                memset (buffer, 0x00, 5);
                                RUN = false;
                                break;
                            } else
                                memset (buffer, 0x00, 5);
                        }
                    } else {
                        send(sock, SREJ, strlen(SREJ), 0 );
                        digitalWrite (red,  LOW);
                        digitalWrite (green,  LOW);
                        digitalWrite (blue,  LOW);
                        digitalWrite (yellow,  LOW);
                        digitalWrite (yellow, HIGH) ;
                    }
                } else
                    memset (buffer, 0x00, 5);
            }
        } // defeat case
    } // while loop
}
/*** Handshake Function prototypes ***/
