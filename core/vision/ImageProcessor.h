#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <kinematics/ForwardKinematics.h>
#include <common/RobotDimensions.h>
#include <common/Profiling.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/RobotInfo.h>
#include <common/RobotCalibration.h>
#include <vision/structures/BallCandidate.h>
#include <math/Pose3D.h>
#include <vision/structures/VisionParams.h>

class BallDetector;
class Classifier;
class BeaconDetector;

struct RLE {
    int lcol;
    int rcol;
    int parent;
    int curr;
    int npixels;
    int color;
    int rank;

    RLE(int l, int r, int idx, int c) {
        lcol = l;
        rcol = r;
        parent = idx;
        curr = idx;
        npixels = r - l + 1;
        color = c;
        rank = 1;
    }
};

vector<RLE*> getRLERow(unsigned char* img_ptr, int width, int &start_idx) {
    // handle NULL case
    auto prev_color = img_ptr[0];
    auto prev_idx = 0;
    vector<RLE*> encoding;
    for(int x = 0; x < width; ++x) {
        if(img_ptr[x] == prev_color)
            continue;
        else {
            encoding.push_back(new RLE(prev_idx, x - 1, start_idx, prev_color));
            start_idx++;
            prev_color = img_ptr[x];
            prev_idx = x;
        }
    }
    encoding.push_back(new RLE(prev_idx, width - 1, start_idx, prev_color));
    start_idx++;
    return encoding;
}

int getParent(int idx, unordered_map<int, RLE*> &rle_ptr) {
    if(rle_ptr.find(idx) == rle_ptr.end())
        return -1;
    if(rle_ptr[idx]->parent == rle_ptr[idx]->curr)
        return rle_ptr[idx]->parent;
    // Path compression
    int p = getParent(rle_ptr[idx]->parent, rle_ptr);
    rle_ptr[idx]->parent = p;
    return p;
}

void mergeBlobs(int idx1, int idx2, unordered_map<int, RLE*> &rle_ptr) {
    int p1 = getParent(idx1, rle_ptr);
    int p2 = getParent(idx2, rle_ptr);
    if(p1 == -1 || p2 == -1) {
        std::cout << "Unknown RLE" << endl;
        return;
    }
    if(p1 == p2)
        return;
    // Union by rank
    int r1 = rle_ptr[p1]->rank;
    int r2 = rle_ptr[p2]->rank;
    if(r1 > r2) {
        rle_ptr[p2]->parent = p1;
        rle_ptr[p1]->npixels += rle_ptr[p2]->npixels;
    }
    else if(r2 > r1) {
        rle_ptr[p1]->parent = p2;
        rle_ptr[p2]->npixels += rle_ptr[p1]->npixels;
    }
    else {
        rle_ptr[p2]->parent = p1;
        rle_ptr[p1]->rank++;
        rle_ptr[p1]->npixels += rle_ptr[p2]->npixels;
    }
}

void mergeEncodings(vector<RLE*> &prev_encoding, vector<RLE*> &encoding, unordered_map<int, RLE*> &rle_ptr) {
    if(prev_encoding.size() == 0 || encoding.size() == 0)
        return;
    int i = 0, j = 0;
    while(i < prev_encoding.size() && j < encoding.size()) {
        if(prev_encoding[i]->rcol < encoding[j].lcol) {
            i++;
        }
        else if(prev_encoding[i]->lcol > encoding[j].rcol) {
            j++;
        }
        else {
            // overlap detected, if colors match then merge blobs
            if(prev_encoding[i]->color == encoding[j]->color) {
                mergeBlobs(prev_encoding[i]->curr, encoding[j]->curr, rle_ptr);
            }
            // progress pointers
            if(prev_encoding[i]->rcol >= encoding[j]->rcol) {
                j++;
            }
            else {
                i++;
            }
        }
    }
}

unordered_map<int, RLE*> calculateBlobs(unsigned char* img_ptr, int height, int width) {
    // handle NULL case
    int loc_idx = 0;
    unordered_map<int, RLE*> rle_ptr;
    vector<RLE*> prev_encoding;

    if(img_ptr == NULL)
        return rle_ptr;

    for(int y = 0; y < height; y++) {
        auto encoding = getRLERow(img_ptr + y * width, width, loc_idx);
        // initialising the hash table with RLE pointers
        for(int i = 0; i < encoding.size(); ++i) {
            assert(rle_ptr.find(encoding[i]->curr) == rle_ptr.end());
            rle_ptr[encoding[i]->curr] = encoding[i];
        }
        mergeEncodings(prev_encoding, encoding);
        prev_encoding = encoding;
    }
    return rle_ptr;
}

/// @ingroup vision
class ImageProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);
    ~ImageProcessor();
    void processFrame();
    void init(TextLogger*);
    void SetColorTable(unsigned char*);
    std::unique_ptr<BeaconDetector> beacon_detector_;
    std::unique_ptr<Classifier> color_segmenter_;
    unsigned char* getImg();
    unsigned char* getSegImg();
    unsigned char* getColorTable();
    bool isRawImageLoaded();
    int getImageHeight();
    int getImageWidth();
    const ImageParams& getImageParams() const { return iparams_; }
    const CameraMatrix& getCameraMatrix();
    void setCalibration(const RobotCalibration& calibration);
    void enableCalibration(bool value);
    void updateTransform();
    std::vector<BallCandidate*> getBallCandidates();
    BallCandidate* getBestBallCandidate();
    bool isImageLoaded();
    void detectBall();
    void findBall(int& imageX, int& imageY);
  private:
    int getTeamColor();
    double getCurrentTime();

    VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    Camera::Type camera_;
    CameraMatrix cmatrix_;
    
    VisionParams vparams_;
    unsigned char* color_table_;
    TextLogger* textlogger;

    float getHeadPan() const;
    float getHeadTilt() const;
    float getHeadChange() const;
    
    std::unique_ptr<RobotCalibration> calibration_;
    bool enableCalibration_;

    //void saveImg(std::string filepath);
    int topFrameCounter_ = 0;
    int bottomFrameCounter_ = 0;
};

#endif
