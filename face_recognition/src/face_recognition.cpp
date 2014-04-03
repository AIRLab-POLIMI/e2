/*************************************************************
 * Face Recognition node
 *************************************************************/
#include <ros/ros.h>
#include "ros/package.h"
#include <actionlib/server/simple_action_server.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cvaux.h>
#include <cxcore.hpp>
#include <sys/stat.h>
#include <termios.h>
#include <term.h>
#include <unistd.h>
#include "face_recognition_lib.cpp"
#include <boost/filesystem.hpp>
#include <dirent.h>

#define RATE 4 // Hz

using namespace std;

class FaceRecognition {

public:

	FaceRecognition(std::string name) : frl(), it_(nh_), as_(nh_, name,boost::bind(&FaceRecognition::executeCB, this, _1), false)
	{
		ros::NodeHandle nh("~");

		cvNamedWindow("Input", CV_WINDOW_AUTOSIZE);
		cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0, 4.0, 2, 2, CV_AA);

		goal_id_ = -99;

		textColor = CV_RGB(0,255,255); 								// light blue text
		confidence_value = 0.82;											//a face recognized with confidence value higher than the confidence_value threshold is accepted as valid.
		show_screen_flag = true;											//if output screen is shown
		add_face_number = 25;												//a parameter for the "add_face_images" goal which determines the number of training images for a new face (person) to be acquired from the video stream
		person_number = 0;														//the number of persons in the training file (train.txt)

		as_.start();																		//starting the actionlib server

		//if the number of persons in the training file is not equal with the number of persons in the trained database, the database is not updated and user should be notified to retrain the database if new tarining images are to be considered.
		if (calcNumTrainingPerson(frl.train_filename) != frl.nPersons)
		{
			frl.database_updated = false;
			ROS_INFO("[FaceRecognition]:: Alert: Database is not updated, You better (re)train from images!");
		}
		string rgb_camera_topic, depth_camera_topic;
		nh.param<string>("rgb_camera", rgb_camera_topic, "/camera/rgb/image_raw");
		nh.param<string>("depth_camera", depth_camera_topic, "/camera/depth/image_raw");

		ROS_INFO("[FaceRecognition]:: rgb camera: %s",rgb_camera_topic.c_str());
		rgb_image_sub_ = it_.subscribe(rgb_camera_topic, 1,&FaceRecognition::imageCB, this); 								// Real image stream

		//ROS_INFO("[FaceRecognition]:: depth camera: %s",depth_camera_topic.c_str());
		//depth_image_sub_ = it_.subscribe(depth_camera_topic, 1,&FaceRecognition::imageCB, this); 						// Depth image stream

	}

	~FaceRecognition(void)
	{
		cvDestroyWindow("Input");
	}

	//==============================================================
	//	Exectute Callback
	//==============================================================
	void executeCB(const face_recognition::FaceRecognitionGoalConstPtr &goal)
	{
		//check to be sure if the goal should be still persuaded
		if (as_.isPreemptRequested() || ros::isShuttingDown())
		{
			as_.setPreempted();
			return;
		}
		//check if the name of the person has been provided for the add-face-images goal
		if (goal->order_id == 2 && goal->order_argument.empty())
		{
			ROS_INFO("[FaceRecognition]:: No name has been provided for the add_person_images goal");
			as_.setPreempted();
			return;
		}

		ros::Rate r(RATE);

		//Storing the information about the current goal and reseting feedback and result variables
		goal_argument_ = goal->order_argument;
		result_.order_id = goal->order_id;
		feedback_.order_id = goal->order_id;

		result_.names.clear();
		result_.confidence.clear();
		feedback_.names.clear();
		feedback_.confidence.clear();

		goal_id_ = goal->order_id;

		switch (goal_id_)
		{
			// Delete photo and clean trainfile to original status
			case 5:
				ROS_INFO("[FaceRecognition]::  * Restore to initial config");

				std::remove(frl.train_filename);
				boost::filesystem::remove_all(frl.dataFolder);
				boost::filesystem::create_directory(frl.dataFolder);
				boost::filesystem::copy_file(frl.original_train_filename, frl.train_filename);

				DIR *dir;
				struct dirent *ent;

				if ((dir = opendir(frl.original_data_filename)) != NULL) {

					/* print all the files and directories within directory */
					while ((ent = readdir(dir)) != NULL)
					{
						string src_file = frl.original_data_folder + "/" + ent->d_name;
						string dst_file = frl.data_folder + "/" + ent->d_name;

						if (!boost::filesystem::is_directory(src_file))
							boost::filesystem::copy_file(src_file.c_str(),dst_file.c_str());
					}
					closedir(dir);
				} else {
					/* could not open directory */
					ROS_INFO("[FaceRecognition]:: Problem opening directory.");
				}

				ROS_INFO("[FaceRecognition]:: Restored");
				if (frl.retrainOnline())
					as_.setSucceeded(result_);
				else
					as_.setAborted(result_);

				break;

			//(exit) Goal is to exit
			case 4:
				ROS_INFO("[FaceRecognition]:: exit request");
				as_.setSucceeded(result_);
				r.sleep();
				ros::shutdown();
				break;
			case 3:
				//(train_database) Goal is to (re)train the database from training images
				if (frl.retrainOnline())
					as_.setSucceeded(result_);
				else
					as_.setAborted(result_);
				break;
				//(recognize_once) Goal is to recognize the person in the video stream, succeed when the first person is found
			case 0:
				//(recognize_continuous) Goal is to Continuously recognize persons in the video stream and provide feedback continuously. This goal is persuaded for infinite time
			case 1:
				//(add_face_images) Goal is to take a number of(add_face_number) images of a person's face from the video stream and save them as training images
			case 2:

			{

				if (goal_id_ == 2)
					add_face_count = 0;

				//to synchronize with processes performed in the subscribed function to the video stream (imageCB)
				//as far as the goal id is 0, 1 or 2, it's active and there is no preempting request, imageCB function can be called.
				while (as_.isActive() && !as_.isPreemptRequested()&& !ros::isShuttingDown())
					r.sleep();

				mutex_.lock();

				if (as_.isActive())
				{
					as_.setPreempted();
					ROS_INFO("[FaceRecognition]:: Goal %d is preempted", goal_id_);
				}
				goal_id_ = -99;
				mutex_.unlock();
				break;
			}
		}
		goal_id_ = -99;
	}

	//==============================================================
	//	Calculate number training person from file
	//==============================================================
	int calcNumTrainingPerson(char * filename)
	{
		FILE * imgListFile = 0;
		char imgFilename[512];
		int iFace, nFaces = 0;
		int person_num = 0;

		// open the input file
		if (!(imgListFile = fopen(filename, "r")))
		{
			ROS_INFO("[FaceRecognition]:: Can\'t open file %s\n", filename);
			return 0;
		}

		// count the number of faces
		while (fgets(imgFilename, 512, imgListFile))
			++nFaces;

		rewind(imgListFile);

		//count the number of persons
		for (iFace = 0; iFace < nFaces; iFace++)
		{
			char personName[256];
			int personNumber;
			// read person number (beginning with 1), their name and the image filename.
			fscanf(imgListFile, "%d %s %s", &personNumber, personName,imgFilename);

			if (personNumber > person_num)
				person_num = personNumber;
		}
		fclose(imgListFile);
		return (person_num);
	}

	//==============================================================
	//	Image callback
	//==============================================================
	void imageCB(const sensor_msgs::ImageConstPtr& msg) {
		//to synchronize with executeCB function.
		//as far as the goal id is 0, 1 or 2, it's active and there is no preempting request, imageCB function is proceed.
		if (!as_.isActive() || goal_id_ > 2)
			return;
		if (!mutex_.try_lock())
			return;
		if (as_.isPreemptRequested()) {
			ROS_INFO("[FaceRecognition]:: Goal %d is preempted", goal_id_);
			as_.setPreempted();
			mutex_.unlock();
			return;
		}
		cv_bridge::CvImagePtr cv_ptr;
		//convert from ros image format to opencv image format
		try {
			cv_ptr = cv_bridge::toCvCopy(msg);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			as_.setPreempted();
			ROS_INFO("[FaceRecognition]:: Goal %d is preempted", goal_id_);
			mutex_.unlock();
			return;
		}

		ros::Rate r(RATE);

		IplImage img_input = cv_ptr->image;
		IplImage *img = cvCloneImage(&img_input);
		IplImage *greyImg;
		IplImage *faceImg;
		IplImage *sizedImg;
		IplImage *equalizedImg;
		CvRect faceRect;

		// Make sure the image is greyscale, since the Eigenfaces is only done on greyscale image.
		greyImg = frl.convertImageToGreyscale(img);
		// Perform face detection on the input image, using the given Haar cascade classifier.
		faceRect = frl.detectFaceInImage(greyImg, frl.faceCascade);

		// Make sure a valid face was detected.
		if (faceRect.width < 1)
		{
			ROS_INFO("[FaceRecognition]:: No face was detected in the last frame");
			text_image.str("");
			text_image << "No face was detected. ";

			cvPutText(img, text_image.str().c_str(), cvPoint(10, faceRect.y + 50), &font, textColor);

			if (show_screen_flag)
			{
				cvShowImage("Input", img);
				cvWaitKey(1);
			}

			cvReleaseImage(&greyImg);
			cvReleaseImage(&img);

			r.sleep();

			mutex_.unlock();
			return;
		}

		cvRectangle(img, cvPoint(faceRect.x, faceRect.y),cvPoint(faceRect.x + faceRect.width - 1,faceRect.y + faceRect.height - 1), CV_RGB(0,255,0), 1,8, 0);
		faceImg = frl.cropImage(greyImg, faceRect); // Get the detected face image.

		// Make sure the image is the same dimensions as the training images.
		sizedImg = frl.resizeImage(faceImg, frl.faceWidth, frl.faceHeight);

		// Give the image a standard brightness and contrast, in case it was too dark or low contrast.
		equalizedImg = cvCreateImage(cvGetSize(sizedImg), 8, 1); // Create an empty greyscale image

		cvEqualizeHist(sizedImg, equalizedImg);
		cvReleaseImage(&greyImg);
		cvReleaseImage(&faceImg);
		cvReleaseImage(&sizedImg);

		//check again if preempting request is not there!
		if (as_.isPreemptRequested())
		{
			ROS_INFO("[FaceRecognition]:: Goal %d is preempted", goal_id_);
			cvReleaseImage(&equalizedImg);
			cvReleaseImage(&img);
			as_.setPreempted();
			ROS_INFO("[FaceRecognition]:: Goal %d is preempted", goal_id_);
			mutex_.unlock();
			return;
		}

		//get the value of show_screen_flag from the parameter server
		ros::param::getCached("show_screen_flag", show_screen_flag);

		//goal is add_face_images
		if (goal_id_ == 2) {
			if (add_face_count == 0) {
				//assign the correct number for the new person
				person_number = calcNumTrainingPerson(frl.train_filename) + 1;
			}
			char cstr[500];
			char cstr_absolute[500];
			sprintf(cstr, "data/%d_%s%d.pgm", person_number, &goal_argument_[0],add_face_count + 1);

			std::string temp_string = frl.path + "/" + cstr;
			strcpy(cstr_absolute, cstr);
			strcpy(cstr, temp_string.c_str());

			ROS_INFO("[FaceRecognition]:: Storing the current face of '%s' into image '%s'.", &goal_argument_[0], cstr);
			//save the new training image of the person
			cvSaveImage(cstr, equalizedImg, NULL);

			// Append the new person to the end of the training data.
			trainFile = fopen(frl.train_filename, "a");
			fprintf(trainFile, "%d %s %s\n", person_number, &goal_argument_[0],cstr_absolute);
			fclose(trainFile);

			if (add_face_count == 0)
			{
				//get from parameter server how many training imaged should be acquire.
				ros::param::get("add_face_number", add_face_number);
				if (add_face_number <= 0)
				{
					ROS_INFO("[FaceRecognition]:: add_face_number parameter is Zero, it is Invalid. One face was added anyway!");
					add_face_number = 1;
				}
				frl.database_updated = false;
			}

			text_image.str("");
			text_image << "A picture of " << &goal_argument_[0] << "was added" << endl;
			cvPutText(img, text_image.str().c_str(), cvPoint(10, 50), &font,textColor);

			//check if enough number of training images for the person has been acquired, then the goal is succeed.
			if (++add_face_count == add_face_number)
			{
				result_.names.push_back(goal_argument_);
				as_.setSucceeded(result_);

				if (show_screen_flag)
				{
					cvShowImage("Input", img);
					cvWaitKey(1);
				}

				cvReleaseImage(&equalizedImg);
				cvReleaseImage(&img);
				mutex_.unlock();
				return;
			}

			feedback_.names.clear();
			feedback_.confidence.clear();
			feedback_.names.push_back(goal_argument_);

			//      feedback_.confidence.push_back();
			as_.publishFeedback(feedback_);
		}

		//goal is to recognize person in the video stream
		if (goal_id_ < 2)
		{
			int iNearest, nearest;
			float confidence;
			float * projectedTestFace = 0;

			if (!frl.database_updated)
				ROS_INFO("[FaceRecognition]:: Alert: Database is not updated, You better (re)train from images!");

			if (frl.nEigens < 1)
			{
				ROS_INFO("[FaceRecognition]:: NO database available, goal is Aborted");
				cvReleaseImage(&equalizedImg);
				cvReleaseImage(&img);
				ROS_INFO("[FaceRecognition]:: Goal %d is Aborted", goal_id_);
				as_.setAborted();
				mutex_.unlock();
				return;
			}

			// Project the test images onto the PCA subspace
			projectedTestFace = (float *) cvAlloc(frl.nEigens * sizeof(float));

			// project the test image onto the PCA subspace
			cvEigenDecomposite(equalizedImg, frl.nEigens, frl.eigenVectArr, 0,	0, frl.pAvgTrainImg, projectedTestFace);

			// Check which person it is most likely to be.
			iNearest = frl.findNearestNeighbor(projectedTestFace, &confidence);
			nearest = frl.trainPersonNumMat->data.i[iNearest];

			//get the desired confidence value from the parameter server
			ros::param::getCached("confidence_value", confidence_value);
			cvFree(&projectedTestFace);
			text_image.str("");

			if (confidence < confidence_value)
			{
				ROS_INFO("[FaceRecognition]:: Confidence is less than %f, detected face is not considered.", (float)confidence_value);
				text_image << "Confidence is less than " << confidence_value;
				cvPutText(img, text_image.str().c_str(),cvPoint(faceRect.x, faceRect.y + faceRect.height + 25),&font, textColor);
			}
			else
			{
				text_image << frl.personNames[nearest - 1].c_str()	<< " is recognized";
				cvPutText(img, text_image.str().c_str(),cvPoint(faceRect.x, faceRect.y + faceRect.height + 25),&font, textColor);

				//goal is to recognize_once, therefore set as succeeded.
				if (goal_id_ == 0)
				{
					result_.names.push_back(frl.personNames[nearest - 1].c_str());
					result_.confidence.push_back(confidence);
					as_.setSucceeded(result_);
				}
				else
				{
					//goal is recognize continuous, provide feedback and continue.
					ROS_INFO("[FaceRecognition]:: detected %s  confidence %f ", frl.personNames[nearest-1].c_str(), confidence);
					feedback_.names.clear();
					feedback_.confidence.clear();
					feedback_.names.push_back(frl.personNames[nearest - 1].c_str());
					feedback_.confidence.push_back(confidence);
					as_.publishFeedback(feedback_);
				}

			}

		}
		if (show_screen_flag)
		{
			cvShowImage("Input", img);
			cvWaitKey(1);
		}
		cvReleaseImage(&equalizedImg);
		cvReleaseImage(&img);
		r.sleep();
		mutex_.unlock();
		return;
	}

protected:

	boost::mutex mutex_; 																				//for synchronization between executeCB and imageCB
	std::string goal_argument_;
	int goal_id_;
	face_recognition::FaceRecognitionFeedback feedback_;
	face_recognition::FaceRecognitionResult result_;
	int add_face_count; 																					//help variable to count the number of training images already taken in the add_face_images goal
	FILE *trainFile;
	double confidence_value; 																			//a face recognized with confidence value higher than confidence_value threshold is accepted as valid.
	bool show_screen_flag;		 																		//if output window is shown
	int add_face_number; 																				//the number of training images to be taken in add_face_images goal
	CvFont font;
	CvScalar textColor;
	ostringstream text_image;
	ros::NodeHandle nh_;
	FaceRecognitionLib frl;
	image_transport::ImageTransport it_;
	image_transport::Subscriber rgb_image_sub_, depth_image_sub_;
	actionlib::SimpleActionServer<face_recognition::FaceRecognitionAction> as_;
	int person_number; 																					//the number of persons in the train file (train.txt)
};


//--------------------------------------------------------------------------------------------
//		Main code
//--------------------------------------------------------------------------------------------
int main(int argc, char** argv) {
	ros::init(argc, argv, "face_recognition");
	FaceRecognition face_recognition(ros::this_node::getName());

	ros::spin();
	return 0;
}

