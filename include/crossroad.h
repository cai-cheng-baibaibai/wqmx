#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "tracking.h"
#include "mapping.h"

using namespace cv;
using namespace std;

class Crossroad
{
public:
    Crossroad();

    void reset(void);

    bool crossRecognition(Tracking& track, PerspectiveMapping& mapping);

    int first_rowStart = 0;

    Point pt_Left = Point(0, 0);
    Point pt_Right = Point(0, 0);

private:

    enum CrossroadSype
    {
        None = 0,
        Cross_Begin,
        Cross_IN,
        Cross_OUT
    };
    Counter none; 
    Counter cross_Begin;
    Counter cross_IN;
    Counter cross_OUT;

    Counter cross_IN_step;
    Counter outtype;

    
    bool cross_IN_flag = 0;
    int rowStart = 50;

    

    CrossroadSype crossroadSype = CrossroadSype::None;
    ;
};