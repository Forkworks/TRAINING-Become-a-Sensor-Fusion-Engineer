# SFND 2D Feature Tracking

![](images/KITTI/2011_09_26/image_00/data/0000000000.png)

## Performance Detectors


Mid-Term Report Specifications

MP.1 Data Buffer Opt 

```
// push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if (dataBuffer.size() == dataBufferSize)
            dataBuffer.erase(dataBuffer.begin());

        dataBuffer.push_back(frame);
```

MP.2 Keypoint Detection

```
        string detectorType = "SIFT";
        bool bVisKey = false;

        double t_detector; //time detector ms

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, t_detector, bVisKey);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints,imgGray, t_detector, bVisKey);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType,t_detector, bVisKey);
        }
```

```
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, double &t_detector, bool bVis)
{

    cv::Ptr<cv::FeatureDetector> detector;

    if (detectorType.compare("FAST") == 0)
    {
        detector = cv::FastFeatureDetector::create();
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        detector = cv::BRISK::create();
    }
```
...
```
   detector->detect(img,keypoints);
```

MP.3






