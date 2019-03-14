#ifndef SEQUENCEDATABASE_H
#define SEQUENCEDATABASE_H

#include "Sequence.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"

#include <mutex>

namespace ORB_SLAM2{
	class Sequence;

	class LoopClosing;

	class KeyFrameDatabase;

	class SequenceDatabase{
	public:
		SequenceDatabase(ORBVocabulary* voc, KeyFrameDatabase* pDB);
		void AddNewKeyFrame(KeyFrame* pKF);
		void SetLoopCloser(LoopClosing* pLoopCloser); 
		// Main function
    	void Run();
    	void ProcessNewSequence();
		void CreateNewSequence(KeyFrame* pKF, int corner);
		void EndCurrenrSequence();
		bool SequenceMatch();
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
    	std::mutex mMutexUnProcessedSeqList;

    	Sequence* cuSeq;
    	LoopClosing* mpLoopCloser;

    protected:
    	KeyFrameDatabase* mpKeyFrameDB;
    	Sequence* mpCurrentSeq;
    	// std::list<KeyFrame*> mInsertKeyFrameQueue;
    	Sequence* mpMatchedSeq;

	    // std::mutex mMutexInsertKeyFrameQueue;
	    KeyFrame* mpCurrentKF;
	    int justLoopedId;
	};//SequenceDatabase
}//namspace ORB_SLAM2
#endif //SEQUENCEDATABASE_H