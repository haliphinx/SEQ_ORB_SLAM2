#include "SequenceDatabase.h"
#include "LoopClosing.h"

namespace ORB_SLAM2{
	SequenceDatabase::SequenceDatabase(ORBVocabulary* voc):mbFinishRequested(false), mbFinished(true), mpVocabulary(voc){

	}//SequenceDatabase::SequenceDatabase

	void SequenceDatabase::AddNewKeyFrame(KeyFrame* pKF){
		// {
  //   		unique_lock<mutex> lock(mMutexInsertKeyFrameQueue);
	        // mInsertKeyFrameQueue.push_back(pKF);
	        // cout<<pKF->mnId<<endl;
  //   	}
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
	void SequenceDatabase::SetLoopCloser(LoopClosing* pLoopCloser){
		mpLoopCloser = pLoopCloser;
	}

	// Main function
    void SequenceDatabase::Run(){
    	mbFinished =false;
    	while(1){
    		// if(CheckNewKeyFrames()){
    		// 	ProcessNewKeyFrame();
    		
	    		if(!UnProcessSeqListisEmpty()){
	    			mpLoopCloser->InsertSequence(unProcessedSeqList.front());
	                unProcessedSeqList.pop_front();
	    		}//if(UnProcessSeqListisEmpty())
	    	// }
    		if(CheckFinish())
            	break;

        	usleep(5000);
    

        	SetFinish();
    	}//while(1)
    }

    void SequenceDatabase::ProcessNewKeyFrame(){
  //   	{
  //   		unique_lock<mutex> lock(mMutexInsertKeyFrameQueue);
	        // mpCurrentKF = mInsertKeyFrameQueue.front();
	        // mInsertKeyFrameQueue.pop_front();
	        // cout<<mpCurrentKF->mnId<<endl;
  //   	}
  //   	if(mSeqList.size()==0){
		// 	cuSeq = new Sequence(mpCurrentKF, mSeqList.size(), false);
		// 	mSeqList.push_back(cuSeq);
		// 	return;
		// }
		// int corner = cuSeq->NewSeqVarify(mpCurrentKF);
		// // cout<<corner<<endl;
		// if(corner!=0){
		// 	CreateNewSequence(mpCurrentKF, corner);
		// }
		// else{
		// 	cuSeq->add(mpCurrentKF);
		// }
    }

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

	void SequenceDatabase::SequenceMatch(){
	    // {
	    //     unique_lock<mutex> lock(mMutexLoopSeqQueue);
	    //     mpCurrentSeq = mlpLoopSeqQueue.front();
	    //     mlpLoopSeqQueue.pop_front();
	    // }

	    // if(mpCurrentSeq->seqId>justLoopedSeqId+2){
	    //     float score = 0;
	    //     float meta_score = 0;
	    //     bool findMatch = false;
	    //     std::vector<float> scoreList;
	    //     const DBoW2::BowVector &cuBoW = mpCurrentSeq->seqBowVec;
	    //     int testRange = (mpSeqDatabase->GetLatestCorner(mpCurrentSeq->seqId)<mpCurrentSeq->seqId-2)?mpSeqDatabase->GetLatestCorner(mpCurrentSeq->seqId):mpCurrentSeq->seqId-2;
	    //     for(int i = 0; i<testRange; i++){
	    //         const DBoW2::BowVector &preBoW = mpSeqDatabase->mSeqList[i]->seqBowVec;
	    //         meta_score = mpORBVocabulary->score(cuBoW, preBoW);
	    //         scoreList.push_back(meta_score);
	    //         if(meta_score>score){
	    //             score = meta_score;
	    //             mpMatchedSeq = mpSeqDatabase->mSeqList[i];
	    //             findMatch = true;
	    //         }
	    //     }
	    //     if(findMatch){
	    //         double sum = std::accumulate(std::begin(scoreList), std::end(scoreList), 0.0);  
	    //         double mean =  sum / scoreList.size();
	    //         std::cout<<"Most similar sequence pair:("<<mpCurrentSeq->seqId<<","<<mpMatchedSeq->seqId<<")"<<std::endl;
	    //         if(score>1.5*mean){
	    //             std::cout<<"Most similar sequence pair:("<<mpCurrentSeq->seqId<<","<<mpMatchedSeq->seqId<<") "<<"score:"<<score<<" mean:"<<mean<<std::endl;
	    //             for(int i = 0; i<mpCurrentSeq->NumOfKeyFrames(); i++){
	    //                 unique_lock<mutex> lock(mMutexLoopQueue);
	    //                 mlpLoopKeyFrameQueue.push_back(mpCurrentSeq->GetKeyFrame(i));
	    //                 mlpLoopCandidateSeq.push_back(mpMatchedSeq->seqId);
	    //             }
	    //             return ;
	    //         }
	    //     }
	    // }

	    // for(int i = 0; i<mpCurrentSeq->NumOfKeyFrames(); i++){
	    //     mpKeyFrameDB->add(mpCurrentSeq->GetKeyFrame(i));
	    // 	}

	    // return ;
	}//void SequenceDatabase::SequenceMatch()


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

	bool SequenceDatabase::CheckFinish()
	{
	    unique_lock<mutex> lock(mMutexFinish);
	    return mbFinishRequested;
	}

	void SequenceDatabase::SetFinish()
	{
	    unique_lock<mutex> lock(mMutexFinish);
	    mbFinished = true;
	}

	void SequenceDatabase::RequestFinish()
	{
	    unique_lock<mutex> lock(mMutexFinish);
	    mbFinishRequested = true;
	}

	bool SequenceDatabase::isFinished()
	{
	    unique_lock<mutex> lock(mMutexFinish);
	    return mbFinished;
	}

	bool SequenceDatabase::CheckNewKeyFrames(){
		// unique_lock<mutex> lock(mMutexInsertKeyFrameQueue);
    	return(!mInsertKeyFrameQueue.empty());
	}
}//namespace