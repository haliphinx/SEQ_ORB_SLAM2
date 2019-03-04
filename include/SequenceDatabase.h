#ifndef SEQUENCEDATABASE_H
#define SEQUENCEDATABASE_H

#include "Sequence.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include <mutex>

namespace ORB_SLAM2{
	class Sequence;

	class SequenceDatabase{
	public:
		SequenceDatabase(ORBVocabulary* voc);
		void AddNewKeyFrame(KeyFrame* pKF);
		void CreateNewSequence(KeyFrame* pKF, int corner);
		void EndCurrenrSequence();
		std::vector<Sequence*> FindSeqLoopCandidate(Sequence* Seq);
		bool UnProcessSeqListisEmpty();
		int GetLatestCorner(int& seqId);

		//Sequence vector
		ORBVocabulary* mpVocabulary;
    	std::vector<Sequence*> mSeqList;
    	std::list<Sequence*> unProcessedSeqList;
    	Sequence* cuSeq;
	};//SequenceDatabase
}//namspace ORB_SLAM2
#endif //SEQUENCEDATABASE_H