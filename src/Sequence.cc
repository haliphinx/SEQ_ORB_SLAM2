#include "Sequence.h"
#include <math.h>

namespace ORB_SLAM2{


Sequence::Sequence(KeyFrame* pKF):seqLength(0){
	
	dAngle = 0;

	cKF = pKF;

	//random color for each sequence
	c1 = rand()/double(RAND_MAX);
	c2 = rand()/double(RAND_MAX);
	c3 = rand()/double(RAND_MAX);
}//Sequence::Sequence

void Sequence::add(KeyFrame* pKF){
	seqLength++;

	cv::Mat Rcw1 = cKF->GetPose().rowRange(0,3).colRange(0,3);
	cv::Mat Rcw2 = pKF->GetPose().rowRange(0,3).colRange(0,3);
	cv::Mat Rcw = Rcw2*Rcw1.inv();
	float ang = acos((Rcw.at<float>(0,0)+Rcw.at<float>(1,1)+Rcw.at<float>(2,2)-1)/2);
	// cout<<ang<<endl;

	dAngle = ang;

	cKF = pKF;


}//Sequence::add

void Sequence::erase(KeyFrame* pKF){
	seqLength--;
}//Sequence::erase

void Sequence::clear(){
	seqLength = 0;
}//Sequence::clear

bool Sequence::NewSeqVarify(KeyFrame* pKF){
	int lenThresh = 20;
	float angThresh = 0.086;

	cv::Mat Rcw1 = cKF->GetPose().rowRange(0,3).colRange(0,3);
	cv::Mat Rcw2 = pKF->GetPose().rowRange(0,3).colRange(0,3);
	cv::Mat Rcw = Rcw2*Rcw1.inv();
	float ang = acos((Rcw.at<float>(0,0)+Rcw.at<float>(1,1)+Rcw.at<float>(2,2)-1)/2);
	// float ang = 5.0;

	// if(ang>10.0){
	// 	return true;
	// }

	if(seqLength<2){
		return false;
	}

	if(seqLength>=lenThresh){
		return true;
	}


	if((ang>angThresh)&&(dAngle<angThresh)){
		return true;
	}

	else if((ang<angThresh)&&(dAngle>angThresh)){
		return true;
	}
	// if(seqLength>=lenThresh){
	// 	return true;
	// }//if





	return false;
}//Sequence::NewSeqVarify

}//ORB_SLAM2
