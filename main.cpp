
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>

#include <sstream>
#include <iostream>
#include <iomanip>
#include "src/networkclient.h"
/*#include "controllercore.h"*/

#include <regex>
#include <string>
#include <memory>
#include <iostream>
#include <istream>
#include <math.h>
#include <functional>


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <cv_bridge/cv_bridge.h>

static std::string ip = "11.0.0.230";
static std::string port = "2240";
/*/above might be 8840 */
bool _stream;

static NetworkClient client(ip, port);


void send(const std::string &message)
{
    std::string answer;
    int length;

    client.write(message);
    length = client.read_until(answer, '\0', std::chrono::seconds(10));

    std::cout << "Command:" << message << std::endl;
    std::cout << "Answer:";
    if (length > 0) {
        std::cout << std::endl << answer;
        std::cout << "--------" << std::endl;
    }
}



int main(int argc, char **argv) {
    
    ros::init(argc, argv, "thermal");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    ros::Rate loop_rate(80);
    std::cout << "-----FU---" << std::endl;

    if (client) {
        std::cout << "### Sending test message. ###" << std::endl;
        send("GSRN\n");
/*        std::cout << "### ZOOM test ###" << std::endl;
        send("GZTV");
        send("GZTL");
        send("SZTN 5");
        send("GZTV");

        send("GZVV");
        send("GZVL");
        send("SZVN 5");
        send("GZVV");*/
        std::istringstream iss;
        //Sending test message

        //Setting delimiter to null character
        send("SDLM NULL\n");
        send("SETH TRUE");
        send("GETH");
    /*Below changes color palette*/
        send("GPTE");
        send("GPTL");
        send("SPTE TEMPERATURE");

    /*SO below activvates the thing, it should be activated already. i got a TRUE */
   /*     send("IACT");
        send("ACTV hha-0zzr0qhpqhi");
        send("IACT");*/
        const std::string addr("rtspsrc location=rtsp://" + ip + ":8554/thermal latency=300 caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink");

        cv::VideoCapture cap(addr, cv::CAP_GSTREAMER);
        //cv::VideoCapture capv("rtspsrc location=rtsp://" + ip + ":8554/visible latency=3000 caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink", cv::CAP_GSTREAMER);
        /*above, at visual, latency was 3000*/
        //cv::waitKey(30);
        std::cout << "### sTOP ###" << std::endl;
        _stream = true;
        std::cout << "p1" << std::endl;
        cv::Mat frame;
        //cv::Mat framev;
        std::cout << "p2" << std::endl;
        while(ros::ok())
        {
//            std::cout << "### Shouldn't you get here at least. ###" << std::endl;
            cap >> frame;
            //capv >> framev;

/*            std::cout << "test test hello" << std::endl;
            std::cout << "OpenCV version : " << CV_VERSION << std::endl;
            std::cout << "Major version : " << CV_MAJOR_VERSION << std::endl;

            std::cout << "Minor version : " << CV_MINOR_VERSION << std::endl;

            std::cout << "Subminor version : " << CV_SUBMINOR_VERSION << std::endl;*/
//            std::cout << "### STREAMING. ###" << std::endl;
            if (frame.empty())
            {
                std::cout << "### Frameless. ###" << std::endl;
                break;
            }
/*            cv::Mat image;
            image = cv::imread("1607399379162.jpg");
            cv::imshow("Faggot", image);*/
            cv::cvtColor(frame, frame,cv::ColorConversionCodes::COLOR_BGR2RGB);
            cv::imshow("Display Image", frame);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);

      /*      cv::cvtColor(framev, framev,cv::ColorConversionCodes::COLOR_BGR2RGB);
            cv::imshow("Visual", framev);*/


     /*       cv::waitKey(0);
            return 0;*/
            ros::spinOnce();

            loop_rate.sleep();
            if (cv::waitKey(5) >= 0)
                break;



        }

        cap.release();
        //capv.release();

/*        *_stream = false;*/

    }

    return 0;
}