#include "Sequence.h"

namespace ORB_SLAM2{

Sequence::Sequence(KeyFrame* pKF):seqLength(1), firstKF(pKF){
	pRotation = pKF->GetRotation();
}//Sequence::Sequence

void Sequence::add(KeyFrame* pKF){
	seqLength++;
}//Sequence::add

void Sequence::erase(KeyFrame* pKF){
	seqLength--;
}//Sequence::erase

void Sequence::clear(){
	seqLength = 0;
	firstKF = NULL;
}//Sequence::clear

bool Sequence::NewSeqVarify(KeyFrame* pKF){
	if(seqLength>50){
		return true;
	}//if



	return false;
}//Sequence::NewSeqVarify

}//ORB_SLAM2