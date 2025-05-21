#include "../../include/obstacle.h"

bool Obstacle::process(Tracking& track, vector<PredictResult> predict, PerspectiveMapping& mapping)
{
    vector<PredictResult> resultsObs; 
    for (int i = 0; i < predict.size(); i++)
    {
        if ((predict[i].type == LABEL_CONE || predict[i].type == LABEL_BLOCK || predict[i].type == LABEL_PEDESTRIAN) && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.4)
            resultsObs.push_back(predict[i]);
    }

    if (resultsObs.size() <= 0)
        return enable;

    if (resultsObs.size() <= 0)
        return enable;

    int areaMax = 0;
    int index = 0;  
    for (int i = 0; i < resultsObs.size(); i++)
    {
        int area = resultsObs[i].width * resultsObs[i].height;
        if (area >= areaMax)
        {
            index = i;
            areaMax = area;
        }
    }

    resultObs = resultsObs[index];
    enable = true; 
    int disLeft = COLSIMAGE;
    int disRight = COLSIMAGE;
    if (!track.PointsEdgeLeft_BA.empty())
    {
        for (int i = 0; i < track.PointsEdgeLeft_BA.size(); i++)
        {
            if (track.PointsEdgeLeft_BA[i].x == resultsObs[index].y)
            {
                disLeft = resultsObs[index].x + resultsObs[index].width - track.PointsEdgeLeft_BA[i].y;
            }
        }
    }
    if (!track.PointsEdgeRight_BA.empty())
    {
        for (int i = 0; i < track.PointsEdgeRight_BA.size(); i++)
        {
            if (track.PointsEdgeRight_BA[i].x == resultsObs[index].y)
            {
                disLeft = track.PointsEdgeRight_BA[i].y - resultsObs[index].x;
            }
        }
    }
    if (disLeft < disRight)
    {
        mapping.direction = RIGHT;
    }
    if (disRight > disLeft)
    {
        mapping.direction = LEFT;
    }

    return enable;

}

void Obstacle::drawImage(Mat& img)
{
    if (enable)
    {
        putText(img, "[2] Obstacle - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        cv::Rect rect(resultObs.x, resultObs.y, resultObs.width, resultObs.height);
        cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }
}