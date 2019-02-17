#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "KeyFrame.h"

namespace ORB_SLAM2{
	class KeyFrame;

	class Sequence{
	public:
		Sequence(KeyFrame* pKF);
		void add(KeyFrame* pKF);
		void erase(KeyFrame* pKF);
		void clear();
		bool NewSeqVarify(KeyFrame* pKF);

		int seqLength;
		KeyFrame* firstKF;
		cv::Mat pRotation;


	};//Sequence
} //ORB_SLAM2

#endif //SEQUENCE_H