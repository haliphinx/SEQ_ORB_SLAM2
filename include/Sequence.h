#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "KeyFrame.h"

namespace ORB_SLAM2{
	class KeyFrame;

	class Sequence{
	public:
		Sequence();
		void add();
		void erase(KeyFrame* pKF);
		void clear();
		bool NewSeqVarify(cv::Mat mTcw);

		int seqLength;


	};//Sequence
} //ORB_SLAM2

#endif //SEQUENCE_H