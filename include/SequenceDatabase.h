#ifndef SEQUENCEDATABASE_H
#define SEQUENCEDATABASE_H

#include "Sequence.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include <mutex>

namespace ORB_SLAM2{
	class Sequence;

	class LoopClosing;

	class SequenceDatabase{
	public:
		SequenceDatabase(ORBVocabulary* voc);
		void AddNewKeyFrame(KeyFrame* pKF);
		void SetLoopCloser(LoopClosing* pLoopCloser); 
		// Main function
    	void Run();
		void CreateNewSequence(KeyFrame* pKF, int corner);
		void EndCurrenrSequence();
		void SequenceMatch();
		bool UnProcessSeqListisEmpty();
		int GetLatestCorner(int& seqId);
		bool CheckFinish();
    	void SetFinish();
    	void RequestFinish();
    	bool isFinished();

		bool mbFinishRequested;
   		bool mbFinished;
    	std::mutex mMutexFinish;

		//Sequence vector

		ORBVocabulary* mpVocabulary;
    	std::vector<Sequence*> mSeqList;
    	std::list<Sequence*> unProcessedSeqList;
    	Sequence* cuSeq;
    	LoopClosing* mpLoopCloser;

    protected:
    	void ProcessNewKeyFrame();
    	bool CheckNewKeyFrames();
    	Sequence* mpCurrentSeq;
    	std::list<KeyFrame*> mInsertKeyFrameQueue;

	    // std::mutex mMutexInsertKeyFrameQueue;
	    KeyFrame* mpCurrentKF;
	};//SequenceDatabase
}//namspace ORB_SLAM2
#endif //SEQUENCEDATABASE_H