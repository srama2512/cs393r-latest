#include <vision/ImageProcessor.h>
#include <vision/Classifier.h>
#include <vision/BeaconDetector.h>
#include <vision/Logging.h>
#include <iostream>

vector<RLE*> ImageProcessor::getRLERow(int y, int width, int &start_idx) {
    // handle NULL case
    int xstep = 1 << iparams_.defaultHorizontalStepScale;
    auto prev_color = getSegImg()[y * width];
    auto prev_idx = 0;
    vector<RLE*> encoding;
    for(int x = 0; x < width; x += xstep) {
        auto c = getSegImg()[y * width + x];
        if(c == prev_color)
            continue;
        else {
            encoding.push_back(new RLE(y, prev_idx, x - 1, start_idx, prev_color));
            start_idx++;
            prev_color = c;
            prev_idx = x;
        }
    }
    encoding.push_back(new RLE(y, prev_idx, width - 1, start_idx, prev_color));
    start_idx++;

    return encoding;
}

int ImageProcessor::getParent(int idx) {
    if(rle_ptr.find(idx) == rle_ptr.end())
        return -1;
    if(rle_ptr[idx]->parent == rle_ptr[idx]->curr)
        return rle_ptr[idx]->parent;
    // Path compression
    int p = getParent(rle_ptr[idx]->parent);
    rle_ptr[idx]->parent = p;
    return p;
}

void ImageProcessor::mergeBlobs(int idx1, int idx2) {
    int p1 = getParent(idx1);
    int p2 = getParent(idx2);
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
        rle_ptr[p1]->xi = min(rle_ptr[p1]->xi, rle_ptr[p2]->xi);
        rle_ptr[p1]->xf = max(rle_ptr[p1]->xf, rle_ptr[p2]->xf);
        rle_ptr[p1]->yi = min(rle_ptr[p1]->yi, rle_ptr[p2]->yi);
        rle_ptr[p1]->yf = max(rle_ptr[p1]->yf, rle_ptr[p2]->yf);
        rle_ptr[p1]->xsum += rle_ptr[p2]->xsum;
        rle_ptr[p1]->ysum += rle_ptr[p2]->ysum;
    }
    else if(r2 > r1) {
        rle_ptr[p1]->parent = p2;
        rle_ptr[p2]->npixels += rle_ptr[p1]->npixels;
        rle_ptr[p2]->xi = min(rle_ptr[p1]->xi, rle_ptr[p2]->xi);
        rle_ptr[p2]->xf = max(rle_ptr[p1]->xf, rle_ptr[p2]->xf);
        rle_ptr[p2]->yi = min(rle_ptr[p1]->yi, rle_ptr[p2]->yi);
        rle_ptr[p2]->yf = max(rle_ptr[p1]->yf, rle_ptr[p2]->yf);
        rle_ptr[p2]->xsum += rle_ptr[p1]->xsum;
        rle_ptr[p2]->ysum += rle_ptr[p1]->ysum;
    }
    else {
        rle_ptr[p2]->parent = p1;
        rle_ptr[p1]->rank++;
        rle_ptr[p1]->npixels += rle_ptr[p2]->npixels;
        rle_ptr[p1]->xi = min(rle_ptr[p1]->xi, rle_ptr[p2]->xi);
        rle_ptr[p1]->xf = max(rle_ptr[p1]->xf, rle_ptr[p2]->xf);
        rle_ptr[p1]->yi = min(rle_ptr[p1]->yi, rle_ptr[p2]->yi);
        rle_ptr[p1]->yf = max(rle_ptr[p1]->yf, rle_ptr[p2]->yf);
        rle_ptr[p1]->xsum += rle_ptr[p2]->xsum;
        rle_ptr[p1]->ysum += rle_ptr[p2]->ysum;
    }
}

void ImageProcessor::mergeEncodings(vector<RLE*> &prev_encoding, vector<RLE*> &encoding) {
    if(prev_encoding.size() == 0 || encoding.size() == 0)
        return;
    int i = 0, j = 0;
    while(i < prev_encoding.size() && j < encoding.size()) {
        if(prev_encoding[i]->rcol < encoding[j]->lcol) {
            i++;
        }
        else if(prev_encoding[i]->lcol > encoding[j]->rcol) {
            j++;
        }
        else {
            // overlap detected, if colors match then merge blobs
            if(prev_encoding[i]->color == encoding[j]->color) {
                mergeBlobs(prev_encoding[i]->curr, encoding[j]->curr);
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

Blob makeBlob(RLE* r) {
    Blob b;
    b.xi = r->xi;
    b.xf = r->xf;
    b.yi = r->yi;
    b.yf = r->yf;
    b.dx = r->xf - r->xi + 1;
    b.dy = r->yf - r->yi + 1;
    b.color = r->color;
    b.lpCount = r->npixels;
    b.avgX = r->xsum / r->npixels;
    b.avgY = r->ysum / r->npixels;
    
    return b;
}

vector<Blob> ImageProcessor::filterBlobs(uint8_t color, int size=0) {
    vector<Blob> filtered;
    for(int i = 0; i < detected_blobs.size(); ++i) {
        if(detected_blobs[i].color != color)
            continue;
        if(detected_blobs[i].lpCount < size)
            continue;
        filtered.push_back(detected_blobs[i]);
    }
    return filtered;
}

void ImageProcessor::calculateBlobs() {
    // handle NULL case
    int height = iparams_.height;
    int width = iparams_.width;
    int ystep = 1 << iparams_.defaultVerticalStepScale;
    int loc_idx = 0;
    rle_ptr.clear();
    vector<RLE*> prev_encoding;

    for(int y = 0; y < height; y += ystep) {
        auto encoding = getRLERow(y, width, loc_idx);
        // initialising the hash table with RLE pointers
        for(int i = 0; i < encoding.size(); ++i) {
            assert(rle_ptr.find(encoding[i]->curr) == rle_ptr.end());
            rle_ptr[encoding[i]->curr] = encoding[i];
        }
        mergeEncodings(prev_encoding, encoding);
        prev_encoding = encoding;
    }
    // setting the detected blobs in the vector
    detected_blobs.clear();
    for(auto it = rle_ptr.begin(); it != rle_ptr.end(); ++it) {
        RLE* b = it->second;
        if(b->parent == b->curr) {
            detected_blobs.push_back(makeBlob(b));
        }
        delete(b);
    }
}

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera)
{
  enableCalibration_ = false;
  color_segmenter_ = std::make_unique<Classifier>(vblocks_, vparams_, iparams_, camera_);
  beacon_detector_ = std::make_unique<BeaconDetector>(DETECTOR_PASS_ARGS);
  calibration_ = std::make_unique<RobotCalibration>();
}

ImageProcessor::~ImageProcessor() {
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  color_segmenter_->init(tl);
  beacon_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

//void ImageProcessor::saveImg(std::string filepath) {
//  cv::Mat mat;
//  int xstep_ = 1 << iparams_.defaultHorizontalStepScale;
//  int ystep_ = 1 << iparams_.defaultVerticalStepScale;
//  cv::resize(color_segmenter_->img_grayscale(), mat, cv::Size(), 1.0 / xstep_, 1.0 / ystep_, cv::INTER_NEAREST); 
  
//  cv::imwrite(filepath, mat);
//}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;

  Pose3D pcamera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_.data(), NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_.data(), NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D *rel_parts = vblocks_.body_model->rel_parts_.data(), *abs_parts = vblocks_.body_model->abs_parts_.data();
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    pcamera = abs_parts[camera];
  }
  else pcamera = vblocks_.body_model->abs_parts_[camera];

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
    auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
    pcamera.translation.z += self.height;
  }

  cmatrix_.updateCameraPose(pcamera);
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->isLoaded();
  return vblocks_.image->isLoaded();
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

double ImageProcessor::getCurrentTime() {
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(const RobotCalibration& calibration){
  *calibration_ = calibration;
}

void ImageProcessor::processFrame(){
  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
  if(camera_ == Camera::BOTTOM) return;

  tlog(30, "Process Frame camera %i", camera_);

  // Horizon calculation
  tlog(30, "Calculating horizon line");
  updateTransform();
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 20000);
  vblocks_.robot_vision->horizon = horizon;
  tlog(30, "Classifying Image: %i", camera_);
  if(!color_segmenter_->classifyImage(color_table_)) return;
  calculateBlobs();
  detectBall();
  beacon_detector_->findBeacons();
}

void ImageProcessor::detectBall() {
  int imageX = -1, imageY = -1;
  findBall(imageX, imageY);

  WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];
  if(imageX == -1 && imageY == -1){
    ball->seen = false;
    return;
  }

  ball->imageCenterX = imageX;
  ball->imageCenterY = imageY;

  Position p = cmatrix_.getWorldPosition(imageX, imageY);
  ball->visionBearing = cmatrix_.bearing(p);
  ball->visionElevation = cmatrix_.elevation(p);
  ball->visionDistance = cmatrix_.groundDistance(p);

  cout << "Ball pan: " << ball->visionBearing << "   Ball tilt: " << ball->visionElevation << endl;
  cout << "Ball distance: " << ball->visionDistance << endl << endl;
  ball->seen = true;
}

void ImageProcessor::findBall(int& imageX, int& imageY) {
    if(getSegImg() == NULL){
        imageX = -1;
        imageY = -1;
        cout << "Ball not detected" << endl;
        return;
    }
    auto orangeBlobs = filterBlobs(c_ORANGE, 100);
    sort(orangeBlobs.begin(), orangeBlobs.end(), BlobCompare);
    if(orangeBlobs.size() > 0) {
        cout << "Ball detected at: " << orangeBlobs[0].avgX << "\t" << orangeBlobs[0].avgY << endl;
        imageX = orangeBlobs[0].avgX;
        imageY = orangeBlobs[0].avgY;
    }
    else {
        imageX = -1;
        imageY = -1;
        cout << "Ball not detected" << endl;
    }
}


int ImageProcessor::getTeamColor() {
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const {
  if (vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->getJointDelta(HeadPan);
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  return std::vector<BallCandidate*>();
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
  return NULL;
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->isLoaded();
}

