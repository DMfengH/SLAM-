#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>
#include <ceres/ceres.h>

using namespace std;
using namespace Eigen;
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// baseline
double baseline = 0.573;
// paths
string left_file = "../left.png";
string disparity_file = "../disparity.png";
boost::format fmt_others("../%06d.png");    // other files

// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;


// 优化参数的更新方式
class SE3Parameterization : public ceres::LocalParameterization
{
public:
    SE3Parameterization() {}
    virtual ~SE3Parameterization() {}

    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const
    {   // 复用数据的内存，将它们转变为Eigen类型
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(x);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_lie(delta);

        Sophus::SE3d T = Sophus::SE3d::exp(lie);
        Sophus::SE3d delta_T = Sophus::SE3d::exp(delta_lie);

        // 李代数左乘更新
        Eigen::Matrix<double, 6, 1> x_plus_delta_lie = (delta_T * T).log();

        // 这里是把 Matrix 变回数组
        for(int i = 0; i < 6; ++i)
            x_plus_delta[i] = x_plus_delta_lie(i, 0);

        return true;
    }
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const
    {   
        // 应该是类似上面的Eigen::Map
        // 这里是把等号右边的Matrix赋给jacobian，左右两边的大小必须一致
        // 【但是这里并不管jacobian数组本身的大小，就从参数给的指针位置开始赋值】
        ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
        return true;
    }
    virtual int GlobalSize() const { return 6; }
    virtual int LocalSize() const { return 6; }
};


// 1维的光度误差，6维的sophus::se3d类型的优化参数R、t
class directMethodCostFunction : public ceres::SizedCostFunction<1,6>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    ~directMethodCostFunction(){}
    // 输入参数：三维点坐标，相机内参K
    directMethodCostFunction(
                const cv::Mat &img1,
                const cv::Mat &img2,
                const Vector2d &px_ref,
                const double depth_ref): 
                _img1(img1),_img2(img2), _px_ref(px_ref), _depth_ref(depth_ref){};

    // parameters[0]是se3d
    // 关于返回值：可不可以返回false，用来表示这个代价函数无效了？？？
    virtual bool Evaluate(double const* const* parameters,
                          double *residuals,
                          double **jacobians) const{
        // 这里fx、fy、cx、cy直接使用全局变量可以么？
        Eigen::Vector3d point_ref =
            _depth_ref * Eigen::Vector3d((_px_ref[0] - cx) / fx, (_px_ref[1] - cy) / fy, 1);
        Eigen::Vector3d point_cur = Sophus::SE3d::exp(Vector6d(parameters[0])) * point_ref;
  
        double u = fx * point_cur[0] / point_cur[2] + cx;
        double v = fy * point_cur[1] / point_cur[2] + cy;
        
        double X = point_cur[0];
        double Y = point_cur[1];
        double Z = point_cur[2];
        double Z2 = Z * Z;
        double Z_inv = 1.0 / Z;
        double Z2_inv = Z_inv * Z_inv;

        residuals[0] = GetPixel(_img1, _px_ref[0], _px_ref[1]) - GetPixel(_img2, u, v);

        // 这里如果没有这两个jacobians的判断，就会产生segmentation fault
        if(jacobians){
            if(jacobians[0]){
                Matrix26d J_pixel_xi;
                Eigen::Vector2d J_img_pixel;

                J_pixel_xi(0, 0) = fx * Z_inv;
                J_pixel_xi(0, 1) = 0;
                J_pixel_xi(0, 2) = -fx * X * Z2_inv;
                J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
                J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
                J_pixel_xi(0, 5) = -fx * Y * Z_inv;

                J_pixel_xi(1, 0) = 0;
                J_pixel_xi(1, 1) = fy * Z_inv;
                J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
                J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
                J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
                J_pixel_xi(1, 5) = fy * X * Z_inv;

                J_img_pixel = Eigen::Vector2d(
                    0.5 * (GetPixel(_img2, u + 1, v ) - GetPixel(_img2, u - 1, v)),
                    0.5 * (GetPixel(_img2, u, v + 1) - GetPixel(_img2, u, v - 1))
                );

                // total jacobian
                Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();
                jacobians[0][0] = J[0];
                jacobians[0][1] = J[1];
                jacobians[0][2] = J[2];
                jacobians[0][3] = J[3];
                jacobians[0][4] = J[4];
                jacobians[0][5] = J[5];
                return true;
            }
        } 
    }

    double GetPixel(const cv::Mat &img, double x, double y) const {
    // boundary check
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x >= img.cols) x = img.cols - 1;
        if (y >= img.rows) y = img.rows - 1;
        uchar *data = &img.data[int(y) * img.step + int(x)];
        double xx = x - floor(x);
        double yy = y - floor(y);
        return double(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]);
    }

private:
    const cv::Mat &_img1;
    const cv::Mat &_img2;
    const Vector2d &_px_ref;
    const double _depth_ref;
    // const Matrix3d _K;

};


bool poseEstimationDirectCeres( const cv::Mat &img1,
                                const cv::Mat &img2,
                                const VecVector2d &px_ref,
                                const vector<double> depth_ref,
                                Sophus::SE3d &T21)
{
    ceres::Problem problem;
    double pose[6];

    Eigen::Matrix<double, 6, 1> poseVec = T21.log();
    pose[0] = poseVec(0);
    pose[1] = poseVec(1);
    pose[2] = poseVec(2);
    pose[3] = poseVec(3);
    pose[4] = poseVec(4);
    pose[5] = poseVec(5);
    cout << "添加cost\n";
    for (int i = 0; i < px_ref.size() ; i++){
        ceres::CostFunction *costFunction = new directMethodCostFunction(
                                                        img1,
                                                        img2,
                                                        px_ref[i],
                                                        depth_ref[i]);
        problem.AddResidualBlock(costFunction,NULL, pose);
    }
    cout << "设置parameter\n";
    problem.SetParameterization(pose, new SE3Parameterization());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    cout << "求解\n";
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << "求解完毕\n";
    cout << summary.BriefReport() << endl;
    T21 = Sophus::SE3d::exp(Vector6d(pose));
    cout << "T21 = \n" << T21.matrix() << endl;
}


/// class for accumulator jacobians in parallel
class JacobianAccumulator {
public:
    JacobianAccumulator(
        const cv::Mat &img1_,
        const cv::Mat &img2_,
        const VecVector2d &px_ref_,
        const vector<double> depth_ref_,
        Sophus::SE3d &T21_) :
        img1(img1_), img2(img2_), px_ref(px_ref_), depth_ref(depth_ref_), T21(T21_) {
        projection = VecVector2d(px_ref.size(), Eigen::Vector2d(0, 0));
    }

    /// accumulate jacobians in a range
    void accumulate_jacobian(const cv::Range &range);

    /// get hessian matrix
    Matrix6d hessian() const { return H; }

    /// get bias
    Vector6d bias() const { return b; }

    /// get total cost
    double cost_func() const { return cost; }

    /// get projected points
    VecVector2d projected_points() const { return projection; }

    /// reset h, b, cost to zero
    void reset() {
        H = Matrix6d::Zero();
        b = Vector6d::Zero();
        cost = 0;
    }

private:
    const cv::Mat &img1;
    const cv::Mat &img2;
    const VecVector2d &px_ref;
    const vector<double> depth_ref;
    Sophus::SE3d &T21;
    VecVector2d projection; // projected points

    std::mutex hessian_mutex;
    Matrix6d H = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    double cost = 0;
};

/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21,
    Sophus::SE3d &newT21
);

/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21,
    Sophus::SE3d &newT21
);

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    // boundary check
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;
    if (y >= img.rows) y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - xx) * yy * data[img.step] +
        xx * yy * data[img.step + 1]
    );
}


// 添加了ceres求解，与手写光流法求解的结果有差异，但是比较相近，但是cost差距比较大。
// 可能是ceres求解哪里写错了。
// 疑问：ceres如何像手写光流法一样，在优化过程中删除一些无效点？？
int main(int argc, char **argv) {
    cout << "开始!\n";
    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 2000;
    int boarder = 20;
    VecVector2d pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data   
    for (int i = 0; i < nPoints; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3d T_cur_ref;
    Sophus::SE3d newT_cur_ref;
    cv::Mat img = cv::imread((fmt_others % 1).str(), 0);
    cout << "进入单层位子估计\n";
    // DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref,newT_cur_ref);

    cout << "进入循环\n";
    // for (int i = 1; i < 6; i++) {  // 1~10
    //     cv::Mat img = cv::imread((fmt_others % i).str(), 0);
    //     // try single layer by uncomment this line
    //     // DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    //     DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    // }
    return 0;
}
//  T21 存储一种求解方法的结果，newT21存储另一种求解方法的结果。
void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21,
    Sophus::SE3d &newT21) {

    const int iterations = 20;
    double cost = 0, lastCost = 0;
    auto t1 = chrono::steady_clock::now();
    cout << "求解之前：\n";

    // Sophus::SE3d newT21 = T21;
    // 这句是使用ceres进行求解，求解结果在T21中
    poseEstimationDirectCeres(img1, img2, px_ref, depth_ref, T21);
    
    //  下面是手写光流法求解，求解结果在newT21中
    JacobianAccumulator jaco_accu(img1, img2, px_ref, depth_ref, newT21);

    // 这个for循环就是一次求解最优化，可以使用ceres或g2o代替。
    for (int iter = 0; iter < iterations; iter++) {
        
        jaco_accu.reset();
        cv::parallel_for_(cv::Range(0, px_ref.size()),
                          std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_accu, std::placeholders::_1));

        Matrix6d H = jaco_accu.hessian();
        Vector6d b = jaco_accu.bias();

        // solve update and put it into estimation
        // 求解出来的update是一个se3量
        Vector6d update = H.ldlt().solve(b);;
        newT21 = Sophus::SE3d::exp(update) * newT21;
        cost = jaco_accu.cost_func();

        if (std::isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        if (update.norm() < 1e-3) {
            // converge
            break;
        }

        lastCost = cost;
        cout << "iteration: " << iter << ", cost: " << cost << endl;
    }

    cout << "newT21 = \n" << newT21.matrix() << endl;
    auto t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "direct method for single layer: " << time_used.count() << endl;

    // plot the projected pixels here
    // cv::Mat img2_show;
    // cv::cvtColor(img2, img2_show, CV_GRAY2BGR);
    // VecVector2d projection = jaco_accu.projected_points();
    // for (size_t i = 0; i < px_ref.size(); ++i) {
    //     auto p_ref = px_ref[i];
    //     auto p_cur = projection[i];
    //     if (p_cur[0] > 0 && p_cur[1] > 0) {
    //         cv::circle(img2_show, cv::Point2f(p_cur[0], p_cur[1]), 2, cv::Scalar(0, 250, 0), 2);
    //         cv::line(img2_show, cv::Point2f(p_ref[0], p_ref[1]), cv::Point2f(p_cur[0], p_cur[1]),
    //                  cv::Scalar(0, 250, 0));
    //     }
    // }
    // cv::imshow("current", img2_show);
    // cv::waitKey();
}

void JacobianAccumulator::accumulate_jacobian(const cv::Range &range) {

    // parameters
    const int half_patch_size = 1;
    int cnt_good = 0;
    Matrix6d hessian = Matrix6d::Zero();
    Vector6d bias = Vector6d::Zero();
    double cost_tmp = 0;

    for (size_t i = range.start; i < range.end; i++) {

        // compute the projection in the second image
        Eigen::Vector3d point_ref =
            depth_ref[i] * Eigen::Vector3d((px_ref[i][0] - cx) / fx, (px_ref[i][1] - cy) / fy, 1);
        Eigen::Vector3d point_cur = T21 * point_ref;

        // 深度为负，无效
        if (point_cur[2] < 0)   // depth invalid
            continue;

        float u = fx * point_cur[0] / point_cur[2] + cx, v = fy * point_cur[1] / point_cur[2] + cy;

        // 太靠近边界，无效
        if (u < half_patch_size || u > img2.cols - half_patch_size || v < half_patch_size ||
            v > img2.rows - half_patch_size)
            continue;

        projection[i] = Eigen::Vector2d(u, v);
        double X = point_cur[0], Y = point_cur[1], Z = point_cur[2],
            Z2 = Z * Z, Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;
        cnt_good++;

        // and compute error and jacobian
        for (int x = -half_patch_size; x <= half_patch_size; x++)
            for (int y = -half_patch_size; y <= half_patch_size; y++) {

                double error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y) -
                               GetPixelValue(img2, u + x, v + y);
                Matrix26d J_pixel_xi;
                Eigen::Vector2d J_img_pixel;

                J_pixel_xi(0, 0) = fx * Z_inv;
                J_pixel_xi(0, 1) = 0;
                J_pixel_xi(0, 2) = -fx * X * Z2_inv;
                J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
                J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
                J_pixel_xi(0, 5) = -fx * Y * Z_inv;

                J_pixel_xi(1, 0) = 0;
                J_pixel_xi(1, 1) = fy * Z_inv;
                J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
                J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
                J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
                J_pixel_xi(1, 5) = fy * X * Z_inv;

                J_img_pixel = Eigen::Vector2d(
                    0.5 * (GetPixelValue(img2, u + 1 + x, v + y) - GetPixelValue(img2, u - 1 + x, v + y)),
                    0.5 * (GetPixelValue(img2, u + x, v + 1 + y) - GetPixelValue(img2, u + x, v - 1 + y))
                );

                // total jacobian
                Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();

                hessian += J * J.transpose();
                bias += -error * J;
                cost_tmp += error * error;
            }
    }

    if (cnt_good) {
        // set hessian, bias and cost
        unique_lock<mutex> lck(hessian_mutex);
        H += hessian;
        b += bias;
        cost += cost_tmp / cnt_good;
    }
}

void DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21,
    Sophus::SE3d &newT21) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    for (int i = 0; i < pyramids; i++) {
        if (i == 0) {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        } else {
            cv::Mat img1_pyr, img2_pyr;
            cv::resize(pyr1[i - 1], img1_pyr,
                       cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1], img2_pyr,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(img1_pyr);
            pyr2.push_back(img2_pyr);
        }
    }

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
    for (int level = pyramids - 1; level >= 0; level--) {
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px: px_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }

        // scale fx, fy, cx, cy in different pyramid levels
        fx = fxG * scales[level];
        fy = fyG * scales[level];
        cx = cxG * scales[level];
        cy = cyG * scales[level];
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21,newT21);
    }

}
