#include "SequenceDatabase.h"
#include "LoopClosing.h"

namespace ORB_SLAM2{
	SequenceDatabase::SequenceDatabase(ORBVocabulary* voc, KeyFrameDatabase* pDB):mbFinishRequested(false), mbFinished(true), mpVocabulary(voc), mpKeyFrameDB(pDB),
	justLoopedId(0)
	{

	}//SequenceDatabase::SequenceDatabase

	void SequenceDatabase::AddNewKeyFrame(KeyFrame* pKF){
		// {
  //   		unique_lock<mutex> lock(mMutexInsertKeyFrameQueue);
	 //        mInsertKeyFrameQueue.push_back(pKF);
  //   	}
    	// if(CheckNewKeyFrames()){
	    //     ProcessNewKeyFrame();}
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
    	// ofstream outfile("/home/xhu/Desktop/res.txt");
    	while(1){
    		// if(CheckNewKeyFrames()){
    		// 	ProcessNewKeyFrame();
    		
    		if(!UnProcessSeqListisEmpty()){



//     			#ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
// #endif



    			if(SequenceMatch()){



//     				#ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
// #endif
//         double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//         outfile<<"%&"<<1000*ttrack<<"&%"<<endl;



    				ProcessNewSequence();
                }
                unProcessedSeqList.pop_front();

    		}//if(UnProcessSeqListisEmpty())
	    	// }




    		if(CheckFinish())
            	break;

        	usleep(5000);
    

        	SetFinish();
    	}//while(1)
    }

    void SequenceDatabase::ProcessNewSequence(){
    	{
	        unique_lock<mutex> lock(unique_lock<mutex> lock);
	        mpCurrentSeq = unProcessedSeqList.front();
	    }
	    justLoopedId = mpCurrentSeq->seqId;
	    for(int i = 0; i<mpCurrentSeq->NumOfKeyFrames();i++){
	    	mpLoopCloser->InsertKeyFrame(mpCurrentSeq->GetKeyFrame(i));
	    }
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

	bool SequenceDatabase::SequenceMatch(){
	    {
	        unique_lock<mutex> lock(unique_lock<mutex> lock);
	        mpCurrentSeq = unProcessedSeqList.front();
	    }
	    
	    cout<<"qq"<<endl;
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif


	    if(mpCurrentSeq->seqId>justLoopedId+2){
	        float score = 0;
	        float meta_score = 0;
	        bool findMatch = false;
	        std::vector<float> scoreList;
	        const DBoW2::BowVector &cuBoW = mpCurrentSeq->seqBowVec;
	        // int testRange = (mpSeqDatabase->GetLatestCorner(mpCurrentSeq->seqId)<mpCurrentSeq->seqId-2)?mpSeqDatabase->GetLatestCorner(mpCurrentSeq->seqId):mpCurrentSeq->seqId-2;
	        int testRange = mpCurrentSeq->seqId-2;
	        for(int i = 0; i<testRange; i++){
	            const DBoW2::BowVector &preBoW = mSeqList[i]->seqBowVec;
	            meta_score = mpVocabulary->score(cuBoW, preBoW);
	            scoreList.push_back(meta_score);
	            if(meta_score>score){
	                score = meta_score;
	                mpMatchedSeq = mSeqList[i];
	                findMatch = true;
	            }
	        }


	            				#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        cout<<"%&"<<1000*ttrack<<"&%"<<endl;


	        if(findMatch){
	            double sum = std::accumulate(std::begin(scoreList), std::end(scoreList), 0.0);  
	            double mean =  sum / scoreList.size();
	            std::cout<<"Most similar sequence pair:("<<mpCurrentSeq->seqId<<","<<mpMatchedSeq->seqId<<")"<<std::endl;
	            if(score>1.5*mean){
	                std::cout<<"Most similar sequence pair:("<<mpCurrentSeq->seqId<<","<<mpMatchedSeq->seqId<<") "<<"score:"<<score<<" mean:"<<mean<<std::endl;
	                return true ;
	            }
	        }
	    }

	    for(int i = 0; i<mpCurrentSeq->NumOfKeyFrames(); i++){
	        mpKeyFrameDB->add(mpCurrentSeq->GetKeyFrame(i));
	    	}

	    return false ;
	}//void SequenceDatabase::SequenceMatch()


	bool SequenceDatabase::UnProcessSeqListisEmpty(){
		{
			unique_lock<mutex> lock(mMutexUnProcessedSeqList);
			if(unProcessedSeqList.empty()){
				return true;
			}
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

	// bool SequenceDatabase::CheckNewKeyFrames(){
	// 	// unique_lock<mutex> lock(mMutexInsertKeyFrameQueue);
 //    	return(!mInsertKeyFrameQueue.empty());
	// }
}//namespace