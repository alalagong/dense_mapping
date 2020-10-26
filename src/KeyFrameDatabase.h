#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include "util.h"
#include "Settings.h"
#include "KeyFrame.h"

namespace LT_SLAM
{

class KeyFrame;


class KeyFrameDatabase
{
public:

    KeyFrameDatabase();

    void add(KeyFrame* pKF);

    void erase(KeyFrame* pKF);

    void clear();

    // Loop Detection
    std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

    // Relocalization
    std::vector<KeyFrame*> DetectRelocalizationCandidates(KeyFrame *F);

    // Inverted file
    std::vector<list<KeyFrame*> > mvInvertedFile;

    // Mutex
    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
