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
            encoding.push_back(new RLE(prev_idx, x - 1, start_idx, prev_color));
            //cout << x - prev_idx << "," << (int) prev_color << " ";
			start_idx++;
            prev_color = c;
            prev_idx = x;
        }
    }
    encoding.push_back(new RLE(prev_idx, width - 1, start_idx, prev_color));
    //cout << width - prev_idx << "," << (int) prev_color << " ";
	//cout << endl;
    start_idx++;
    return encoding;
}

int ImageProcessor::getParent(int idx, unordered_map<int, RLE*> &rle_ptr) {
    if(rle_ptr.find(idx) == rle_ptr.end())
        return -1;
    if(rle_ptr[idx]->parent == rle_ptr[idx]->curr)
        return rle_ptr[idx]->parent;
    // Path compression
    int p = getParent(rle_ptr[idx]->parent, rle_ptr);
    rle_ptr[idx]->parent = p;
    return p;
}

void ImageProcessor::mergeBlobs(int idx1, int idx2, unordered_map<int, RLE*> &rle_ptr) {
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
        // std::cout << "pixels: " << rle_ptr[p1]->npixels;
    }
    else if(r2 > r1) {
        rle_ptr[p1]->parent = p2;
        rle_ptr[p2]->npixels += rle_ptr[p1]->npixels;
        // std::cout << "pixels: " << rle_ptr[p2]->npixels;
    }
    else {
        rle_ptr[p2]->parent = p1;
        rle_ptr[p1]->rank++;
        rle_ptr[p1]->npixels += rle_ptr[p2]->npixels;
        // std::cout << "pixels: " << rle_ptr[p1]->npixels;
    }
}

void ImageProcessor::mergeEncodings(vector<RLE*> &prev_encoding, vector<RLE*> &encoding, unordered_map<int, RLE*> &rle_ptr) {
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

void displayImage(unsigned char* img_ptr, int h, int w) {
    if(img_ptr == NULL)
        return;
    for(int y = 0; y < h; ++y) {
        for(int x = 0; x < w; ++x) {
            std::cout << (int)img_ptr[y * w + x] << " ";
        }
        std::cout << std::endl;
    }
}

unordered_map<int, RLE*> ImageProcessor::calculateBlobs(int height, int width) {
    // handle NULL case
	int ystep = 1 << iparams_.defaultVerticalStepScale;
    int loc_idx = 0;
    unordered_map<int, RLE*> rle_ptr;
    vector<RLE*> prev_encoding;

    for(int y = 0; y < height; y += ystep) {
        auto encoding = getRLERow(y, width, loc_idx);
        // initialising the hash table with RLE pointers
        for(int i = 0; i < encoding.size(); ++i) {
            assert(rle_ptr.find(encoding[i]->curr) == rle_ptr.end());
            rle_ptr[encoding[i]->curr] = encoding[i];
        }
        mergeEncodings(prev_encoding, encoding, rle_ptr);
        prev_encoding = encoding;
    }
	vector<RLE*> blobs;
    for(auto it = rle_ptr.begin(); it != rle_ptr.end(); ++it) {
        RLE* b = it->second;
        if(b->parent != b->curr)
          continue;
        if(b->color != c_ORANGE)
          continue;
		blobs.push_back(b);
    }
	sort(blobs.begin(), blobs.end(), RLECompare());
	for(int i = 0; i < min((int)blobs.size(), 4); ++i) {
		cout << blobs[i]->npixels << " ";
	}
	cout << endl;
    return rle_ptr;
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
  tlog(30, "Process Frame camera %i", camera_);

  // Horizon calculation
  tlog(30, "Calculating horizon line");
  updateTransform();
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 20000);
  vblocks_.robot_vision->horizon = horizon;
  tlog(30, "Classifying Image: %i", camera_);
  if(!color_segmenter_->classifyImage(color_table_)) return;
  detectBall();
  beacon_detector_->findBeacons();
}

void ImageProcessor::detectBall() {
if(getSegImg() == NULL)
return;
  unordered_map<int, RLE*> blobs = calculateBlobs(iparams_.height, iparams_.width);
  for(auto it = blobs.begin(); it != blobs.end(); ++it) {
   RLE* b = it->second;
    if(b->parent != b->curr)
     continue;
    if(b->color != c_FIELD_GREEN)
     continue;
    // cout << "Orange: " << b->npixels << ", ";
 }
  cout << endl;
}

void ImageProcessor::findBall(int& imageX, int& imageY) {
  imageX = imageY = 0;
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

