#include "SequenceDatabase.h"

namespace ORB_SLAM2{
	SequenceDatabase::SequenceDatabase(ORBVocabulary* voc):mpVocabulary(voc){

	}//SequenceDatabase::SequenceDatabase

	void SequenceDatabase::AddNewKeyFrame(KeyFrame* pKF){
		if(mSeqList.size()==0){
			cuSeq = new Sequence(pKF, mSeqList.size());
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
		cuSeq = new Sequence(pKF, mSeqList.size());
		mSeqList.push_back(cuSeq);
	}//SequenceDatabase::CreateNewSequence
	
	void SequenceDatabase::EndCurrenrSequence(){
		cuSeq->ComputeBoW(mpVocabulary);
		std::cout<<cuSeq->seqId<<std::endl;
		std::vector<Sequence*> can = FindSeqLoopCandidate(cuSeq);
	}//SequenceDatabase::EndCurrenrSequence

	std::vector<Sequence*> SequenceDatabase::FindSeqLoopCandidate(Sequence* Seq){
		float score_threshold = 1;
		std::vector<Sequence*> seqCandidate;
		const DBoW2::BowVector &cuBoW = Seq->seqBowVec;
		for(int i = 0; i<static_cast<int>(mSeqList.size()); i++){
			const DBoW2::BowVector &preBoW = mSeqList[i]->seqBowVec;
			float score = mpVocabulary->score(cuBoW, preBoW);
			if(score>score_threshold){
				seqCandidate.push_back(mSeqList[i]);
			}
			std::cout<<Seq->seqId<<","<<i<<":"<<score<<std::endl;
		}
		return seqCandidate;
	}//std::vector<Sequence*> SequenceDatabase::FindSeqLoopCandidate
}//namespace