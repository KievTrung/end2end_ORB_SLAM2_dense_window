#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>

using namespace std;

bool loadImages(const string& pathToSequenceImg, int imgId, cv::Mat& rgb, cv::Mat &depth);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat imRGB, imD;
    int ni = 0;
    while(loadImages(argv[3], ni, imRGB, imD))
    {
        cout << "sequence id: " << ni << endl;
        // Read image and depthmap from file
        double tframe = ni;
        ++ni;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int i=0; i<ni; ni++)
    {
        totaltime+=vTimesTrack[i];
    }

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

bool loadImages(const string& pathToSequenceImg, int imgId, cv::Mat& rgb, cv::Mat &depth)
{
    rgb = cv::imread(pathToSequenceImg + "\\f_rgb\\" + to_string(imgId) + ".png", CV_LOAD_IMAGE_UNCHANGED);

    if (rgb.empty())
        return false;

    depth = cv::imread(pathToSequenceImg + "\\f_depth\\" + to_string(imgId) + ".png", CV_LOAD_IMAGE_UNCHANGED);
    
    if (depth.empty())
        return false;

    return true;
}
