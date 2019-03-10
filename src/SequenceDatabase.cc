#include "SequenceDatabase.h"

namespace ORB_SLAM2{
	SequenceDatabase::SequenceDatabase(ORBVocabulary* voc):mpVocabulary(voc){

	}//SequenceDatabase::SequenceDatabase

	void SequenceDatabase::AddNewKeyFrame(KeyFrame* pKF){
		if(mSeqList.size()==0){
			cuSeq = new Sequence(pKF, mSeqList.size(), false);
			mSeqList.push_back(cuSeq);
			return;
		}
		int corner = cuSeq->NewSeqVarify(pKF);
		if(corner!=0){
			CreateNewSequence(pKF, corner);
		}
		else{
			cuSeq->add(pKF);
		}
	}//SequenceDatabase::AddNewKeyFrame

	void SequenceDatabase::CreateNewSequence(KeyFrame* pKF, int corner){
		EndCurrenrSequence();
		bool iscorner = (corner==1)?false:true;
		cuSeq = new Sequence(pKF, mSeqList.size(), iscorner);
		
		
	}//SequenceDatabase::CreateNewSequence
	
	void SequenceDatabase::EndCurrenrSequence(){
		cuSeq->ComputeBoW(mpVocabulary);
		if(cuSeq->seqLength>3){
			mSeqList.push_back(cuSeq);
			unProcessedSeqList.push_back(cuSeq);
		}
		// std::cout<<cuSeq->seqId<<std::endl;
		// std::vector<Sequence*> can = FindSeqLoopCandidate(cuSeq);
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

	bool SequenceDatabase::UnProcessSeqListisEmpty(){
		if(unProcessedSeqList.empty()){
			return true;
		}
		return false;
	}//SequenceDatabase::UnProcessSeqListisEmpty()

	int SequenceDatabase::GetLatestCorner(int& seqId){
		for(int i = seqId; i>=0; i--){
			if(mSeqList[i]->iscorner){
				return i;
			}
		}
		return seqId;
	}//SequenceDatabase::GetLatestCorner
}//namespace