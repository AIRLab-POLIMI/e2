#include "Blob.h"
using namespace cv;

Blob::Blob(){}


Blob::Blob(CvPoint ptA, unsigned int H, unsigned int W)
{
	pt1.x = ptA.x;
	pt1.y = ptA.y;
	width = W;
	height = H;
	pt2.x = pt1.x + width;
	pt2.y = pt1.y + height;
	center.x = pt1.x + (int)(width/2);
	center.y = pt1.y + (int)(height/2);
	classification = 'U';
}

Blob::Blob(CvPoint ptA, CvPoint ptB)
{
	pt1.x = ptA.x;
	pt1.y = ptA.y;
	pt2.x = ptB.x;
	pt2.y = ptB.y;
	width = pt2.x - pt1.x;
	height = pt2.y - pt1.y;
	center.x = pt1.x + (int)(width/2);
	center.y = pt1.y + (int)(height/2);
	classification = 'U';
}


Rect Blob::getRect()
{
	return (rect = cvRect(pt1.x, pt1.y, width, height));
}
