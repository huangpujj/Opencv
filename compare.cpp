#ifndef __SOCKETMATTRANSMISSIONSEVER_H__
#define __SOCKETMATTRANSMISSIONSEVER_H__

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <fstream>
#define PORT_ORIGINAL_VIDEO 8080
using namespace cv;
using namespace std;

#define PACKAGE_NUM 2
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define width 640
#define height 480

#define BLOCKSIZE IMG_WIDTH*IMG_HEIGHT*3/PACKAGE_NUM

struct recvBuf
{
    char buf[BLOCKSIZE];
    int flag;
};

class SocketMatTransmissionServer
{
public:
    SocketMatTransmissionServer(void);
    ~SocketMatTransmissionServer(void);
    int sockConn;
private:
    struct recvBuf data;

    int needRecv;
    int count;

public:
    int socketConnect(int PORT); // create a server and port;
    int receive(cv::Mat& image); // receive the image
    void socketDisconnect(void); // disconnect the client

};

#endif

int SocketMatTransmissionServer::socketConnect(int PORT)
{
    int server_sockfd = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in server_sockaddr;
    server_sockaddr.sin_family = AF_INET;
    server_sockaddr.sin_port = htons(PORT);
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(server_sockfd, (struct sockaddr*) & server_sockaddr, sizeof(server_sockaddr)) == -1)
    {
        perror("bind");
        return -1;
    }

    if (listen(server_sockfd, 5) == -1)
    {
        perror("listen");
        return -1;
    }

    struct sockaddr_in client_addr;
    socklen_t length = sizeof(client_addr);

    sockConn = accept(server_sockfd, (struct sockaddr*) & client_addr, &length);
    if (sockConn < 0)
    {
        perror("connect");
        return -1;
    }
    else
    {
        printf("connect successful!\n");
        return 1;
    }

    close(server_sockfd);
}

void SocketMatTransmissionServer::socketDisconnect(void)
{
    close(sockConn);
}

int SocketMatTransmissionServer::receive(cv::Mat& image)
{
    int returnflag = 0;
    cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, cv::Scalar(0));
    needRecv = sizeof(recvBuf);
    count = 0;
    memset(&data, 0, sizeof(data));

    for (int i = 0; i < PACKAGE_NUM; i++)
    {
        int pos = 0;
        int len0 = 0;

        while (pos < needRecv)
        {
            len0 = recv(sockConn, (char*)(&data) + pos, needRecv - pos, 0);
            if (len0 < 0)
            {
                printf("Server Recieve Data Failed!\n");
                break;
            }
            pos += len0;
        }

        count = count + data.flag;

        int num1 = IMG_HEIGHT / PACKAGE_NUM * i;
        for (int j = 0; j < IMG_HEIGHT / PACKAGE_NUM; j++)
        {
            int num2 = j * IMG_WIDTH * 3;
            uchar* ucdata = img.ptr<uchar>(j + num1);
            for (int k = 0; k < IMG_WIDTH * 3; k++)
            {
                ucdata[k] = data.buf[num2 + k];
            }
        }

        if (data.flag == 2)
        {
            if (count == PACKAGE_NUM + 1)
            {
                image = img;
                returnflag = 1;
                count = 0;
            }
            else
            {
                count = 0;
                i = 0;
            }
        }
    }
    if (returnflag == 1)
        return 1;
    else
        return -1;
}



int socket_setup()
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        cout << "error in socket()" << endl;
    }

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT_ORIGINAL_VIDEO);
    int addrlen = sizeof(serv_addr);

    if (inet_pton(AF_INET, "192.168.1.120", &serv_addr.sin_addr) <= 0) {
        cout << "error in inet_pton" << endl;
    }

    if (connect(sock, (struct sockaddr*) & serv_addr, addrlen) < 0) {
        cout << "error in connect" << endl;
    }
    return sock;
}

void receive_frame(int socket, Mat* img_buffer, int img_buffer_size, int* id_buffer)
{

    int buffer1[4] = { 0 };
    int buffer_size = 0;
    int id = 9999;
    vector	<uchar> buffer;
    for (int i = 0; i < img_buffer_size - 1; i++) {
        img_buffer[i] = img_buffer[i + 1];
        id_buffer[i] = id_buffer[i + 1];
    }
    recv(socket, &buffer1, 4, MSG_WAITALL);
    buffer_size = buffer1[0];
    buffer.resize(buffer_size);
    recv(socket, buffer.data(), buffer_size, MSG_WAITALL);

    img_buffer[img_buffer_size - 1] = imdecode(buffer, 1);
    recv(socket, &id, sizeof(id), MSG_WAITALL);
    id_buffer[img_buffer_size - 1] = id;


}


int compare(Mat& imgGray1, Mat& imgGray2, Mat& original_images, Mat& corruption_images)
{
    Mat temp;
    vector<KeyPoint> keypoints;
    vector<KeyPoint> keypoints1;

    Mat descriptors1, descriptors2;
    Ptr<ORB> orb = ORB::create();
    orb->detect(imgGray1, keypoints);
    orb->detect(imgGray2, keypoints1);

    orb->compute(imgGray1, keypoints, descriptors1);
    orb->compute(imgGray2, keypoints1, descriptors2);

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    vector< DMatch> matches;
    vector<vector<DMatch>> knnmatches;

    matcher->knnMatch(descriptors1, descriptors2, knnmatches, 2);

    float min_dist = FLT_MAX;
    for (int r = 0; r < knnmatches.size(); ++r)
    {
        //find the good matched
        if (knnmatches[r][0].distance > 0.75 * knnmatches[r][1].distance)
            continue;

        float dist = knnmatches[r][0].distance;
        if (dist < min_dist) min_dist = dist;
    }
    matches.clear();

    for (size_t r = 0; r < knnmatches.size(); ++r)
    {
        if (knnmatches[r][0].distance > 0.6 * knnmatches[r][1].distance || knnmatches[r][0].distance > 5 * max(min_dist, 10.0f))
            continue;
        matches.push_back(knnmatches[r][0]);
    }

    drawMatches(original_images, keypoints, corruption_images, keypoints1, matches, temp);

    if (matches.size() > 10 && !(matches.size() == (keypoints.size() <= keypoints1.size() ? keypoints.size() : keypoints1.size()))) {
        //Point2d phase_shift;
        //imgGray.convertTo(dst, CV_32FC1);
        //imgGray1.convertTo(dst1, CV_32FC1);
        //phase_shift = phaseCorrelate(dst, dst1); //get the shift degree
        //cout << endl << "warp :" << endl << "\tX shift : " << phase_shift.x << "\tY shift : " << phase_shift.y << endl;
        cout << "shifted" << endl;
        return 0;
    }
    else if (matches.size() == (keypoints.size() <= keypoints1.size() ? keypoints.size() : keypoints1.size())) {
        cout << "full matched" << endl;
        return -1;
    }
    else if (matches.size() < 10) {
        cout << "Not matched!" << endl;
        return -2;
    }

}

int freeze_comp(Mat prev, Mat corr, int freeze_cnt, int frame_num, ofstream &myfile){
    int frz_cnt = 0;
    int frz_not = 0;
    int frz_check = 0;
    //cout << "here";
    Vec3b x = corr.at<Vec3b>(0,0);
    Vec3b y = prev.at<Vec3b>(0,0);
    while(1){
        for(int i =0; i < 480; i++){
            for(int j = 0; j < 640; j++){ // 480
                //cout << i << " " << j << endl;
                x = corr.at<Vec3b>(i,j);
                y = prev.at<Vec3b>(i,j);
                if(x == y){
                    frz_cnt++;
                    //cout << "identical" << endl;
                }
                else{
                    frz_not++;
                    if(freeze_cnt > 0){
                        float counter = freeze_cnt/8.0;
                        int counter2 = round(counter);
                        cout << "COMP: frozen for: " << counter << " secs" << endl;
                        myfile << frame_num << ": COMP: frozen for: " << counter << " secs" << endl;
                        freeze_cnt = 0;
                        break;
                    }
                    else if(frz_not > 50){
                        //cout << frame_num << ": " << "no freeeze " << endl;
                        //myfile << frame_num << ": no freeze detected"  << endl;
                        break;
                    }
                }
            } // inner for loop
        } // outer for loop
        //cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!Freeze_cnt " << frz_cnt << endl;
        if(frz_cnt >= 307000){
            freeze_cnt++;
            myfile << frame_num << ": COMP: frozen "  << endl;
        }
        break;
    }// while
    return freeze_cnt;
}


void color_comp(Mat corr_img, int frame_num, ofstream &myfile)
{
    //int height = 480;
    //int width = 640;
    Mat red(height, width, CV_8UC3, Scalar(0,0,255));
    Mat blue(height, width, CV_8UC3, Scalar(255,0,0));
    Mat green(height, width, CV_8UC3, Scalar(0,255,0));
    Mat white(height, width, CV_8UC3, Scalar(255,255,255));

    int cntw = 0;
    int cntb = 0;
    int cntg = 0;
    int cntr = 0;
    int n = 0;
    Vec3b white_o = white.at<Vec3b>(0,0);
    Vec3b red_o = red.at<Vec3b>(0,0);
    Vec3b blue_o = blue.at<Vec3b>(0,0);
    Vec3b green_o = green.at<Vec3b>(0,0);
    Vec3b corr_o = corr_img.at<Vec3b>(0,0);

    for(int i =0; i < 630; i++){
        for(int j = 0; j < 470; j++){
            corr_o = corr_img.at<Vec3b>(j,i);
            if(white_o == corr_o){
                cntw++;
            }
            else if(red_o == corr_o){
                cntr++;
            }
            else if(blue_o == corr_o){
                cntb++;
            }
            else if(green_o == corr_o){
                cntg++;
            }
            else{
                n = 0;
            }

        } // inner for loop
    } // outer for loop
    if(cntw >= 296000){
        cout << "WHITE SCREEN COMPARED" << endl;
        myfile << frame_num << ": WHITE SCREEN COMPARED" << endl;
        //break;
    }
    else if(cntb >= 296000){
        cout << "BLUE SCREEN COMPARED" << endl;
         myfile << frame_num << ": BLUE SCREEN COMPARED" << endl;
        //break;
    }
    else if(cntg >= 296000){
        cout << "GREEN SCREEN COMPARED" << endl;
         myfile << frame_num << ": GREEN SCREEN COMPARED" << endl;
        //break;
    }
    else if(cntr >= 296000){
        cout << "RED SCREEN COMPARED" << endl;
         myfile << frame_num << ": RED SCREEN COMPARED" << endl;
        //break;
    }
    else{
        cout << "no color detection " << endl;
        myfile << frame_num << ": no color detection " << endl;
    }
} // color comp


int main(int argc, char const* argv[]) {

    // create the server and reveive the corruption videos stream
    SocketMatTransmissionServer socketMat;
    if (socketMat.socketConnect(6666) < 0)
    {
        return 0;
    }

    // create the server and receive the original video stream
    int original_receive_socket = socket_setup();
    int img_buffer_size = 10;
    int* id_buffer = new int[img_buffer_size];
    Mat* img_buffer = new Mat[img_buffer_size];
    namedWindow("Original", CV_WINDOW_AUTOSIZE);


    //define some variables
    Mat corruption_images;
    Mat original_images;
    Mat imgGray1; // original_images to gray
    Mat imgGray2; //  corruption_images to gray
    while (1) {
        // receive the corruption images;
        if (socketMat.reveive(corruption_images) > 0) {
            //imwrite("test",corruption_images);
            waitKey(1);
        }

        // reveive the original images
        receive_frame(original_receive_socket, img_buffer, img_buffer_size, id_buffer);
        original_images = img_buffer[img_buffer_size - 1];
        imshow("Original", original_images);
        cv::imshow("Corrupted", corruption_images);

        // compare functions
        cvtColor(original_images, imgGray1, 7);
        cvtColor(corruption_images, imgGray2, 7);


    }


    return 0;
}
