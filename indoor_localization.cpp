/*

https://chatgpt.com/c/68903cc5-3aec-8325-9529-efbbc9e7a5e5
 1. Map side (VVM: visual vector map) is 3D points
 2.  
*/

// pnp_vvm_toy.cpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>
#include <vector>
#include <cmath>

using namespace std;

// Add Gaussian noise to a point
static inline cv::Point2d addNoise(const cv::Point2d& p, double sigma, std::mt19937& rng) {
    std::normal_distribution<double> N(0.0, sigma);
    return cv::Point2d(p.x + N(rng), p.y + N(rng));
}

// Compute mean reprojection error
double meanReprojError(const vector<cv::Point3d>& objPts,
                       const vector<cv::Point2d>& imgPts,
                       const cv::Mat& K,
                       const cv::Mat& rvec, const cv::Mat& tvec) {
    vector<cv::Point2d> proj;
    cv::projectPoints(objPts, rvec, tvec, K, cv::noArray(), proj);
    double err = 0.0;
    for (size_t i = 0; i < imgPts.size(); ++i) {
        cv::Point2d d = imgPts[i] - proj[i];
        err += std::sqrt(d.x*d.x + d.y*d.y);
    }
    return err / imgPts.size();
}

int main() {
    // ---- 1) Camera intrinsics (pixels)
    double fx = 800, fy = 800, cx = 640/2.0, cy = 480/2.0;
    cv::Mat K = (cv::Mat_<double>(3,3) << fx, 0, cx,
                                          0,  fy, cy,
                                          0,  0,  1);

    // ---- 2) Visual Vector Map: 3D landmarks in MAP frame (meters)
    vector<cv::Point3d> landmarks_3d;
    // A small grid in front of the origin, z positive (forward)
    for (int y = -2; y <= 2; ++y) {
        for (int x = -3; x <= 3; ++x) {
            landmarks_3d.emplace_back(0.3*x, 0.3*y, 4.0 + 0.1*y); // mild depth variation
        }
    }

    // ---- 3) Ground-truth camera pose T_cm: map -> camera
    // We'll define a small yaw/pitch/roll and translation
    cv::Mat rvec_gt, tvec_gt;
    {
        // Create rotation from small Euler angles (roll, pitch, yaw)
        double roll = CV_PI/180.0 * 5.0;
        double pitch = CV_PI/180.0 * (-3.0);
        double yaw = CV_PI/180.0 * 10.0;

        // Build rotation matrix Rz(yaw)*Ry(pitch)*Rx(roll)
        auto Rx = [](double a){
            cv::Mat R = (cv::Mat_<double>(3,3) <<
                1,        0,         0,
                0,  cos(a),  -sin(a),
                0,  sin(a),   cos(a));
            return R;
        };
        auto Ry = [](double a){
            cv::Mat R = (cv::Mat_<double>(3,3) <<
                 cos(a), 0, sin(a),
                      0, 1,      0,
                -sin(a), 0, cos(a));
            return R;
        };
        auto Rz = [](double a){
            cv::Mat R = (cv::Mat_<double>(3,3) <<
                cos(a), -sin(a), 0,
                sin(a),  cos(a), 0,
                     0,       0, 1);
            return R;
        };

        cv::Mat Rgt = Rz(yaw) * Ry(pitch) * Rx(roll);
        cv::Rodrigues(Rgt, rvec_gt);
        tvec_gt = (cv::Mat_<double>(3,1) << 0.1, -0.05, 0.2); // small translation
    }

    // ---- 4) Project landmarks to image with noise; add some outliers
    std::mt19937 rng(42);
    vector<cv::Point2d> img_points;
    {
        vector<cv::Point2d> proj;
        cv::projectPoints(landmarks_3d, rvec_gt, tvec_gt, K, cv::noArray(), proj);

        img_points.reserve(proj.size());
        for (auto& p : proj) {
            img_points.push_back(addNoise(p, 0.7, rng)); // ~0.7 px noise
        }

        // Inject a few gross outliers (simulate bad matches)
        for (int k = 0; k < 5 && k < (int)img_points.size(); ++k) {
            img_points[k].x += 50.0; // shift some points
            img_points[k].y -= 40.0;
        }
    }

    // ---- 5) Robust PnP (RANSAC)
    cv::Mat rvec_est, tvec_est;
    vector<int> inliers;
    bool ok = cv::solvePnPRansac(landmarks_3d, img_points, K, cv::noArray(),
                                 rvec_est, tvec_est,
                                 false,           // no initial guess
                                 1000,            // iterations
                                 3.0,             // reprojection error (px)
                                 0.99,            // confidence
                                 inliers,
                                 cv::SOLVEPNP_AP3P); // or P3P/EPnP
    if (!ok) {
        std::cerr << "RANSAC PnP failed.\n";
        return 1;
    }

    std::cout << "RANSAC inliers: " << inliers.size() << " / " << img_points.size() << "\n";
    double err_all = meanReprojError(landmarks_3d, img_points, K, rvec_est, tvec_est);
    std::cout << "Initial reprojection error (all points): " << err_all << " px\n";

    // ---- 6) Optional nonlinear refinement on inliers (ITERATIVE)
    {
        // Gather inlier correspondences
        vector<cv::Point3d> obj_inl; obj_inl.reserve(inliers.size());
        vector<cv::Point2d> img_inl; img_inl.reserve(inliers.size());
        for (int idx : inliers) {
            obj_inl.push_back(landmarks_3d[idx]);
            img_inl.push_back(img_points[idx]);
        }

        bool ok2 = cv::solvePnP(obj_inl, img_inl, K, cv::noArray(),
                                rvec_est, tvec_est,
                                true,                 // use extrinsic guess
                                cv::SOLVEPNP_ITERATIVE); // Gauss-Newton
        if (ok2) {
            double err_inl = meanReprojError(obj_inl, img_inl, K, rvec_est, tvec_est);
            std::cout << "Refined reprojection error (inliers): " << err_inl << " px\n";
        }
    }

    // ---- 7) Report pose & compare to GT
    auto printVec = [](const cv::Mat& v, const string& name){
        cout << name << ": [" << v.at<double>(0) << ", "
                               << v.at<double>(1) << ", "
                               << v.at<double>(2) << "]\n";
    };

    // Convert rvec to R to compute a simple rotation difference
    cv::Mat Rgt, Rest;
    cv::Rodrigues(rvec_gt, Rgt);
    cv::Rodrigues(rvec_est, Rest);
    cv::Mat Rdiff = Rest * Rgt.t();
    double trace = (Rdiff.at<double>(0,0) + Rdiff.at<double>(1,1) + Rdiff.at<double>(2,2));
    trace = std::min(3.0, std::max(-1.0, trace));
    double angle_err_rad = std::acos((trace - 1.0)/2.0);
    double angle_err_deg = angle_err_rad * 180.0 / CV_PI;

    printVec(tvec_gt,  "t_gt");
    printVec(tvec_est, "t_est");
    cout << "Rotation error (deg): " << angle_err_deg << "\n";

    return 0;
}
