#ifndef SEQUENCEDATABASE_H
#define SEQUENCEDATABASE_H

#include "Sequence.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"

namespace ORB_SLAM2{
	class Sequence;

	class SequenceDatabase{
	public:
		SequenceDatabase(ORBVocabulary* voc);
		void AddNewKeyFrame(KeyFrame* pKF);
		void CreateNewSequence(KeyFrame* pKF);
		void EndCurrenrSequence();
		std::vector<Sequence*> FindSeqLoopCandidate(Sequence* Seq);

		//Sequence vector
		ORBVocabulary* mpVocabulary;
    	std::vector<Sequence*> mSeqList;
    	Sequence* cuSeq;
	};//SequenceDatabase
}//namspace ORB_SLAM2
#endif //SEQUENCEDATABASE_H