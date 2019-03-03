#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "KeyFrame.h"
#include "stdlib.h"
#include "Converter.h"

namespace ORB_SLAM2{
	class KeyFrame;

	class Sequence{
	public:
		Sequence(KeyFrame* pKF, int id);
		void add(KeyFrame* pKF);
		void erase(KeyFrame* pKF);
		void clear();
		bool NewSeqVarify(KeyFrame* pKF);
		float CalAngle(KeyFrame* cKF, KeyFrame* pKF);
		int NumOfKeyFrames();
		int NumOfKeyPoints();
		void ComputeBoW(ORBVocabulary* voc);
		cv::Mat GetDescriptors();

		int seqId;

		int seqLength;

		float dAngle;


		//for the frame drawer color
		float c1;
		float c2;
		float c3;

		//BoW
    	DBoW2::BowVector seqBowVec;
    	DBoW2::FeatureVector seqFeatVec;

	protected:

		std::vector<KeyFrame*> KFList;


	};//Sequence
} //ORB_SLAM2

#endif //SEQUENCE_H