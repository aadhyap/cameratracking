#include <iostream>
#include <stdexcept>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <librealsense2/rs.hpp>
using namespace cv;
using namespace std;



//This is drawing the bounding box
static void detectAndDraw(const HOGDescriptor &hog, Mat &img)
{
    vector<Rect> found, found_filtered;
    double t = (double) getTickCount();
    // Run the detector with default parameters. to get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);
    t = (double) getTickCount() - t;
    cout << "detection time = " << (t*1000./cv::getTickFrequency()) << " ms" << endl;
    for(size_t i = 0; i < found.size(); i++ )
    {
        Rect r = found[i];
        size_t j;
        // Do not add small detections inside a bigger detection.
        for ( j = 0; j < found.size(); j++ )
            if ( j != i && (r & found[j]) == r )
                break;
        if ( j == found.size() )
            found_filtered.push_back(r);
    }
    for (size_t i = 0; i < found_filtered.size(); i++)
    {
        Rect r = found_filtered[i];
        // The HOG detector returns slightly larger rectangles than the real objects,
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width*0.1);
        r.width = cvRound(r.width*0.8);
        r.y += cvRound(r.height*0.07);
        r.height = cvRound(r.height*0.8);
        rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
    }
}
int main(int argc, char** argv)
{

    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    namedWindow("people detector", 1);
    int camera_id = 0;


    if (camera_id != -1 )
    {
      rs2::pipeline p;
      rs2::config cfg;
      cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

      p.start(cfg);





        VideoCapture vc;
        //Mat frame;
        if (camera_id != -1)
        {
            vc.open(camera_id);
            if (!vc.isOpened())
            {
                stringstream msg;
                msg << "can't open camera: " << camera_id;
                throw runtime_error(msg.str());
            }
        }

      //  vector<String>::const_iterator it_image = frames.begin();


        for (;;)
        {

          //wait for frames and get frameset
          rs2::frameset frames = p.wait_for_frames();




            auto colored_frame = frames.get_color_frame();

	          //rs2::frameset rs2Frame = rs2Pipe.wait_for_frames();

	          //Get each frame
	          //rs2::frame color_frame = rs2Frame.get_color_frame();

            // Get the depth frame's dimensions

            const int w = colored_frame.as<rs2::video_frame>().get_width();
            const int h = colored_frame.as<rs2::video_frame>().get_height();


            //cv::Mat frame = cv::Mat(cv::Size(1280, 720), CV_8UC1, (void*)colored_frame.get_data());

            cv::Mat frame = cv::Mat(cv::Size(w, h), CV_8UC1, (void*)colored_frame.get_data());
            Mat image(Size(w, h), CV_8UC3, (void*)colored_frame.get_data(), Mat::AUTO_STEP);


            // if (frame.empty())
                // break;
            detectAndDraw(hog, image);
            imshow("people detector", image);
            int c = waitKey( vc.isOpened() ? 30 : 0 ) & 255;
            if ( c == 'q' || c == 'Q' || c == 27)
                break;

            // if ( c == 'q' || c == 'Q' || c == 27)
            //     break;
        }


    }
    return 0;
}
