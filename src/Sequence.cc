#include "Sequence.h"

namespace ORB_SLAM2{

Sequence::Sequence():seqLength(0){
	c1 = rand()/double(RAND_MAX);
	c2 = rand()/double(RAND_MAX);
	c3 = rand()/double(RAND_MAX);
}//Sequence::Sequence

void Sequence::add(){
	seqLength++;
}//Sequence::add

void Sequence::erase(KeyFrame* pKF){
	seqLength--;
}//Sequence::erase

void Sequence::clear(){
	seqLength = 0;
}//Sequence::clear

bool Sequence::NewSeqVarify(cv::Mat mTcw){
	int lenThresh = 10;
	
	if(seqLength>=lenThresh){
		return true;
	}//if





	return false;
}//Sequence::NewSeqVarify

}//ORB_SLAM2