#include "SequenceDatabase.h"

namespace ORB_SLAM2{
	SequenceDatabase::SequenceDatabase(ORBVocabulary* voc):mpVocabulary(voc){

	}//SequenceDatabase::SequenceDatabase

	void SequenceDatabase::AddNewKeyFrame(KeyFrame* pKF){
		if(mSeqList.size()==0){
			cuSeq = new Sequence(pKF);
			mSeqList.push_back(cuSeq);
		}
		else if(cuSeq->NewSeqVarify(pKF)){
			CreateNewSequence(pKF);
		}
		else{
			cuSeq->add(pKF);
		}
	}//SequenceDatabase::AddNewKeyFrame

	void SequenceDatabase::CreateNewSequence(KeyFrame* pKF){
		EndCurrenrSequence();
		cuSeq = new Sequence(pKF);
		mSeqList.push_back(cuSeq);
	}//SequenceDatabase::CreateNewSequence
	
	void SequenceDatabase::EndCurrenrSequence(){
		cuSeq->ComputeBoW(mpVocabulary);
		// std::cout<<cuSeq->seqBowVec.size()<<std::endl;
	}//SequenceDatabase::EndCurrenrSequence

}//namespace