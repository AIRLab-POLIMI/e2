/*=================================================
 Authors:	cristianmandelli@gmail.com 
			deborahzamponi@gmail.com
 Data: 20/12/2011
=================================================*/
#include "opencv2/opencv.hpp" 

class Blob
{
	private:
		CvPoint pt1;	 // top-left corner
		CvPoint pt2; 	// bottom-right corner
		CvPoint center;
		
		unsigned int height;
		unsigned int width;
		
		unsigned char classification;
		
		cv::Rect rect;
		
	public:
		//Constructors
		Blob();
		Blob(CvPoint pt1, unsigned int height, unsigned int width);
		Blob(CvPoint ptA, CvPoint ptB);
	
		//Getters
		CvPoint getPt1() const { return pt1; }
		CvPoint getPt2() const { return pt2; }
		CvPoint getCenter() const { return center; }
		unsigned int getHeight() const { return height; }
		unsigned int getWidth() const { return width; }
		unsigned char getClass() const { return classification; }

		//Setters
		void setClassification(unsigned char C) { classification = C; }
		
		//Other functions
		cv::Rect getRect();
	
};

