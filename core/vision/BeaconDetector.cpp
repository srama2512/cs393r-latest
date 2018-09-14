#include <vision/BeaconDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

vector<Blob> filterByAspectRatio(vector<Blob> blobs) {
    vector<Blob> ret;
    for(int i = 0; i < blobs.size(); ++i) {
        double aspect_ratio = calculateBlobAspectRatio(blobs[i]);

        if (aspect_ratio < ASPECT_RATIO_LOW_BOUND || aspect_ratio > ASPECT_RATIO_HIGH_BOUND)
            continue;

        ret.push_back(blobs[i]);
    }
    return ret;
}

vector<Blob> filterByDensity(vector<Blob> blobs, int xstep, int ystep) {
    vector<Blob> ret;
    for(int i = 0; i < blobs.size(); ++i) {
        double area = calculateBlobArea(blobs[i]) / (ystep);
        double density = blobs[i].lpCount / area;
        if(density < DENSITY_LOW_BOUND)
            continue;
        ret.push_back(blobs[i]);
    }
    return ret;
}

vector<pair<Blob, Blob> > makeBeaconPairs(vector<Blob> &tblobs, vector<Blob> &bblobs) {
    vector<pair<Blob, Blob> > beacons;
    for(int i = 0; i < tblobs.size(); ++i) {
        for(int j = 0; j < bblobs.size(); ++j) {
            if(tblobs[i].avgY > bblobs[j].avgY)
                continue;
            if(tblobs[i].avgX > bblobs[j].xf || tblobs[i].avgX < bblobs[j].xi)
                continue;
            if(bblobs[j].avgX > tblobs[i].xf || bblobs[j].avgX < tblobs[i].xi)
                continue;
            double tarea = calculateBlobArea(tblobs[i]);
            double barea = calculateBlobArea(bblobs[j]);
            double areaSim = tarea / barea;
            if(areaSim > AREA_SIM_HIGH_BOUND || areaSim < AREA_SIM_LOW_BOUND)
                continue;
            tblobs[i].invalid = false;
            bblobs[j].invalid = false;
            // cout << "AS: " << areaSim << endl;
            beacons.push_back(make_pair(tblobs[i], bblobs[j]));
        }
    }
    return beacons;
}

pair<Blob, Blob> BeaconDetector::findBeaconsOfType(const vector<Blob> &tb, const vector<Blob> &bb) {
    int ystep = 1 << iparams_.defaultVerticalStepScale;
    int xstep = 1 << iparams_.defaultHorizontalStepScale;
    // check if the aspect ratio is within ASPECT_RATIO_LOW_BOUND & ASPECT_RATIO_HIGH_BOUND
    auto tblobs = filterByAspectRatio(tb);
    auto bblobs = filterByAspectRatio(bb);
    // check if the density is greater than DENSITY_LOW_BOUND
    tblobs = filterByDensity(tblobs, xstep, ystep);
    bblobs = filterByDensity(bblobs, xstep, ystep);

    auto beacons = makeBeaconPairs(tblobs, bblobs);

    if(beacons.size() == 0) {
        Blob b;
        b.invalid = true;
        return make_pair(b, b);
    }
    return beacons[0];
}

double density(Blob &b, int ystep) {
    double area = calculateBlobArea(b) / (ystep);
    double density = b.lpCount / area;

    return density;
}

void BeaconDetector::findBeacons(vector<Blob> &blobs) {
    if(camera_ == Camera::BOTTOM) return;
    static map<WorldObjectType,int> heights = {
        { WO_BEACON_BLUE_YELLOW,    300 },
        { WO_BEACON_YELLOW_BLUE,    300 },
        { WO_BEACON_BLUE_PINK,      200 },
        { WO_BEACON_PINK_BLUE,      200 },
        { WO_BEACON_PINK_YELLOW,    200 },
        { WO_BEACON_YELLOW_PINK,    200 }
    };
    static map<WorldObjectType, vector<Color> > beacons = {
        { WO_BEACON_BLUE_YELLOW,    { c_BLUE, c_YELLOW } },
        { WO_BEACON_YELLOW_BLUE,    { c_YELLOW, c_BLUE } },
        { WO_BEACON_BLUE_PINK,      { c_BLUE, c_PINK } },
        { WO_BEACON_PINK_BLUE,      { c_PINK, c_BLUE } },
        { WO_BEACON_PINK_YELLOW,    { c_PINK, c_YELLOW } },
        { WO_BEACON_YELLOW_PINK,    { c_YELLOW, c_PINK } }
    };

    map<Color, vector<Blob> > colorBlobs = {
        { c_BLUE,       filterBlobs(blobs, c_BLUE, 100)     },
        { c_YELLOW,     filterBlobs(blobs, c_YELLOW, 100)   },
        { c_PINK,       filterBlobs(blobs, c_PINK, 100)     },
        { c_WHITE,      filterBlobs(blobs, c_WHITE, 100)    }
    };

    for(auto beacon : beacons) {
        auto& object = vblocks_.world_object->objects_[beacon.first];
        auto ctop = beacon.second[0];
        auto cbottom = beacon.second[1];

        pair<Blob, Blob> bblob = findBeaconsOfType(colorBlobs[ctop], colorBlobs[cbottom]);
        if(bblob.first.invalid || bblob.second.invalid) {
            object.seen = false;
            continue;
        }

        object.imageCenterX = (bblob.first.avgX + bblob.second.avgX) / 2;
        object.imageCenterY = (bblob.first.avgY + bblob.second.avgY) / 2;
        auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, heights[beacon.first]);
        object.visionDistance = cmatrix_.groundDistance(position);
        // object.visionDistance = 1600;
        object.visionBearing = cmatrix_.bearing(position);
        object.seen = true;
        object.fromTopCamera = (camera_ == Camera::TOP);
        
        // cout << "AR: " << calculateBlobAspectRatio(bblob.first) << ", " << calculateBlobAspectRatio(bblob.second) << endl;
        // cout << "density: " << density(bblob.first, (1 << iparams_.defaultVerticalStepScale)) << ", " << density(bblob.second, (1 << iparams_.defaultVerticalStepScale)) << endl;
        cout << "saw " << getName(beacon.first) << " at (" << object.imageCenterX << "," << object.imageCenterY << ") with calculated distance " << object.visionDistance << endl;
    }
    cout << endl << endl;
}
