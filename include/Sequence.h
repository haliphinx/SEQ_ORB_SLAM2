#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "KeyFrame.h"
#include "stdlib.h"

namespace ORB_SLAM2{
	class KeyFrame;

	class Sequence{
	public:
		Sequence(KeyFrame* pKF);
		void add(KeyFrame* pKF);
		void erase(KeyFrame* pKF);
		void clear();
		bool NewSeqVarify(KeyFrame* pKF);

		KeyFrame* cKF;

		int seqLength;

		float dAngle;


		//for the frame drawer color
		float c1;
		float c2;
		float c3;


	};//Sequence
} //ORB_SLAM2

#endif //SEQUENCE_H