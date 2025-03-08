#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <Eigen/SVD>
//Eigen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>


using namespace cv;
using namespace aruco;
using namespace Eigen;

// global variables
double reproj_error;
MatrixXd cum_error = MatrixXd::Zero(6, 1);
int count_frame = 0;
int error_count = 0;

// Camera parameters
aruco::CameraParameters CamParam;
cv::Mat K, D;

// Aruco marker and detector
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;

// Topic subscribers and publishers
ros::Subscriber sub_img;
ros::Publisher pub_path_ref, pub_path;
nav_msgs::Path path_ref, path;

// This function publishes the pose results as a path.
void publishPath(
  const ros::Time& t,
  Vector3d& T,
  Quaterniond& Q,
  nav_msgs::Path& path,
  ros::Publisher& pub_path)
{
  geometry_msgs::PoseStampedPtr ps_ptr(new geometry_msgs::PoseStamped());
  ps_ptr->header.stamp = t;
  ps_ptr->header.frame_id = "world";
  ps_ptr->pose.position.x = T(0);
  ps_ptr->pose.position.y = T(1);
  ps_ptr->pose.position.z = T(2);
  ps_ptr->pose.orientation.x = Q.x();
  ps_ptr->pose.orientation.y = Q.y();
  ps_ptr->pose.orientation.z = Q.z();
  ps_ptr->pose.orientation.w = Q.w();

  path.header.stamp = t;
  path.header.frame_id = ps_ptr->header.frame_id;
  path.poses.push_back(*ps_ptr);
  pub_path.publish(path);
}

// Obtain the
cv::Point3f getPositionFromIndex(int idx, int nth)
{
  int idx_x = idx % 6, idx_y = idx / 6;
  double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
  double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
  return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

/* THIS IS WHAT YOU NEED TO WORK WITH */
void calculateReprojectionError(
  const vector<cv::Point3f> &pts_3, // the input 3D points
  const vector<cv::Point2f> &pts_2, // the input 2D features that are corresponding to the 3D points
  const cv::Mat R, // the under-estimated rotation matrix
  const cv::Mat t) // the under-estimated translation
{
   //TODO
  // reproj_error = ...;
  // 将旋转矩阵转换为旋转向量
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  // 存储投影点
  vector<cv::Point2f> projected_pts_2;
  // 使用cv::projectPoints进行投影
  cv::projectPoints(pts_3, rvec, t, K, D, projected_pts_2);
  // 重置重投影误差
  reproj_error = 0.0;
  // 计算重投影误差
  for (size_t i = 0; i < pts_3.size(); i++) {
      reproj_error += cv::norm(projected_pts_2[i] - pts_2[i]);
  }
  // 计算平均重投影误差
  reproj_error /= pts_3.size();
}

Eigen::Matrix<double, 2, 6> Jacobian(Vector3d pt3, VectorXd at){
  MatrixXd J(2, 6);
  double x, y, z, a, b, c, t1, t2, t3;
  x = pt3(0);    y = pt3(1);    z = pt3(2);
  a = at(0);    b = at(1);    c = at(2);
  t1 = at(3);    t2 = at(4);    t3 = at(5);
  J(0,0)= ((x*(cos(a)*sin(c) + cos(c)*sin(a)*sin(b)) + y*(cos(a)*cos(c) - sin(a)*sin(b)*sin(c)) - z*cos(b)*sin(a))*(t1 + z*sin(b) + x*cos(b)*cos(c) - y*cos(b)*sin(c)))/ pow((t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)),2);
  J(0,1)=- (z*cos(b) - x*cos(c)*sin(b) + y*sin(b)*sin(c))/(t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)) - ((z*cos(a)*sin(b) + x*cos(a)*cos(b)*cos(c) - y*cos(a)*cos(b)*sin(c))*(t1 + z*sin(b) + x*cos(b)*cos(c) - y*cos(b)*sin(c)))/pow((t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)),2);
  J(0,2)=(y*cos(b)*cos(c) + x*cos(b)*sin(c))/(t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)) + ((x*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) - y*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)))*(t1 + z*sin(b) + x*cos(b)*cos(c) - y*cos(b)*sin(c)))/pow((t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)),2);
  J(0,3)=-1/(t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b));
  J(0,4)=0;
  J(0,5)=(t1 + z*sin(b) + x*cos(b)*cos(c) - y*cos(b)*sin(c))/pow((t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)),2);
  J(1,0)=(x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b))/(t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)) + ((x*(cos(a)*sin(c) + cos(c)*sin(a)*sin(b)) + y*(cos(a)*cos(c) - sin(a)*sin(b)*sin(c)) - z*cos(b)*sin(a))*(t2 + x*(cos(a)*sin(c) + cos(c)*sin(a)*sin(b)) + y*(cos(a)*cos(c) - sin(a)*sin(b)*sin(c)) - z*cos(b)*sin(a)))/pow((t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)),2);
  J(1,1)=- (z*sin(a)*sin(b) + x*cos(b)*cos(c)*sin(a) - y*cos(b)*sin(a)*sin(c))/(t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)) - ((z*cos(a)*sin(b) + x*cos(a)*cos(b)*cos(c) - y*cos(a)*cos(b)*sin(c))*(t2 + x*(cos(a)*sin(c) + cos(c)*sin(a)*sin(b)) + y*(cos(a)*cos(c) - sin(a)*sin(b)*sin(c)) - z*cos(b)*sin(a)))/pow((t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)),2);
  J(1,2)=((x*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) - y*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)))*(t2 + x*(cos(a)*sin(c) + cos(c)*sin(a)*sin(b)) + y*(cos(a)*cos(c) - sin(a)*sin(b)*sin(c)) - z*cos(b)*sin(a)))/pow((t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)),2) - (x*(cos(a)*cos(c) - sin(a)*sin(b)*sin(c)) - y*(cos(a)*sin(c) + cos(c)*sin(a)*sin(b)))/(t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b));
  J(1,3)=0;
  J(1,4)=-1/(t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b));
  J(1,5)=(t2 + x*(cos(a)*sin(c) + cos(c)*sin(a)*sin(b)) + y*(cos(a)*cos(c) - sin(a)*sin(b)*sin(c)) - z*cos(b)*sin(a))/pow((t3 + x*(sin(a)*sin(c) - cos(a)*cos(c)*sin(b)) + y*(cos(c)*sin(a) + cos(a)*sin(b)*sin(c)) + z*cos(a)*cos(b)),2);
  return J;
}
Eigen::VectorXi selectRandom(int subset_size, int num_points)
{
    VectorXi subset;
    subset.resize(subset_size);
    for (int i = 0; i < subset_size; ++i)
    {
        bool was;
        do
        {
            subset[i] = rand() % num_points;
            was = false;
            for (int j = 0; j < i; ++j)
                if (subset[j] == subset[i])
                {
                    was = true;
                    break;
                }
        } while (was);
    }
    return subset;
}

void MyPnP(
  const vector<cv::Point3f> &pts_3,
  const vector<cv::Point2f> &pts_2,
  cv::Mat &K, 
  cv::Mat &D,
  Eigen::Matrix3d  &R_res,
  Eigen::Vector3d  &t_res)
{
    // Number of points
    const size_t num_points = pts_2.size();
    // 解畸变
    vector<cv::Point2f> unpts_2;
    cv::undistortPoints(pts_2, unpts_2, K, D);
    // Construct the matrix A for Ax = 0 formulation
    MatrixXd A(2 * num_points, 9);
    for (int i = 0; i < num_points; ++i) {
        float X = pts_3[i].x;
        float Y = pts_3[i].y;
        float Z = pts_3[i].z;
        float u = unpts_2[i].x;
        float v = unpts_2[i].y;
        A.row(2*i)   << X, Y, 1, 0, 0, 0, -u*X, -u*Y, -u;
        A.row(2*i+1) << 0, 0, 0, X, Y, 1, -v*X, -v*Y, -v;
    }
    // Apply SVD to A
    JacobiSVD<MatrixXd> svd1(A, ComputeThinV);
    VectorXd h = svd1.matrixV().col(8);
    Matrix3d H;
    H << h(0), h(1), h(2),
            h(3), h(4), h(5),
            h(6), h(7), h(8);
    if (h(8) < 0) H = -H;
    Matrix3d K_eigen;
    cv::cv2eigen(K, K_eigen);
    // H = K_eigen.inverse() * H;
    // 构造近似的正交矩阵 RR
    Eigen::Matrix3d RR; 
    RR.col(0) = H.col(0);
    RR.col(1) = H.col(1);
    RR.col(2) = H.col(0).cross(H.col(1));
    // 计算 U * V^T
    JacobiSVD<MatrixXd> svd2(RR, ComputeFullU | ComputeFullV); 
    RR = svd2.matrixU() * svd2.matrixV().transpose();
    // 算出估计的平移向量 tt
    Eigen::Vector3d tt;
    tt = H.col(2) / H.col(0).norm();
    // 线性部分完成，得到粗略的RR和tt
    // 从 RR 中分解出欧拉角
    Vector3d RR_euler = RR.eulerAngles(0, 1, 2);
    // 构造 [theta, t] 向量 angle_translation
    VectorXd at(6);
    at << RR_euler(0), RR_euler(1), RR_euler(2), tt(0), tt(1), tt(2);
    // 雅可比矩阵 J
    MatrixXd J(2, 6);  
    int iter_threshold = 100;
    for(int count=0; count<iter_threshold; count++)
    {
        MatrixXd An = MatrixXd::Zero(6,6);
        VectorXd bn = VectorXd::Zero(6);
        Matrix3d R;
        Vector3d t;
        R =  Eigen::AngleAxisd(at(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(at(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(at(2), Eigen::Vector3d::UnitZ());
        t << at(3), at(4), at(5);
        R_res = R;
        t_res = t;
        // 遍历一帧图像内的所有3D-2D点对
        for (int i=0; i<(int)num_points; i++)
        {
            Vector3d pt3(pts_3[i].x, pts_3[i].y, pts_3[i].z);
            Vector2d pt2(unpts_2[i].x, unpts_2[i].y);
            // 得到当前这对点的雅可比矩阵
            J = Jacobian(pt3, at);
            // 计算3D-2D投影点
            Vector2d pt_reproj;
            pt3 =  R * pt3 + t;
            pt3 = pt3 / pt3(2);
            pt_reproj << pt3(0), pt3(1);
            // 计算重投影误差
            Vector2d gamma;
            gamma = pt2 - pt_reproj;
            // 把所有点对应的An, Bn累加起来
            An += J.transpose() *J;
            bn -= J.transpose() * gamma;
        }
        // 计算迭代差值
        VectorXd at_delta;
        at_delta = An.inverse() * bn * 0.1;
        if (at_delta.cwiseAbs().sum() < 0.0001 || !at_delta.allFinite())  break;
        //进行迭代
        at += at_delta;
    }
   
}



void MyPnPRansac(
  const vector<cv::Point3f> &pts_3,
  const vector<cv::Point2f> &pts_2,
  cv::Mat &K, 
  cv::Mat &D,
  Eigen::Matrix3d  &R_res,
  Eigen::Vector3d  &t_res)
{
    int num_points = pts_2.size();  // 点的数量
    int subset_size = 6;  // 每次抽样的点的数量
    int range = 100;  // 方案的数量
    double dist_threshold = 0.01;  // 内点阈值
    std::vector<int> subsets_score(range);  // 所有方案的成绩表
    std::vector<std::vector<short>> subsets_index(range);  // 所有方案的内点索引
    vector<cv::Point2f> unpts_2;  // 去失真后的点
    cv::undistortPoints(pts_2, unpts_2, K, D);
    for (int i=0; i<range; i++){  // 开始创建方案，每次选取少量3D-2D点对来创建方案
      VectorXi random_vec =  selectRandom(subset_size, num_points);  // 生成一个随机索引向量
       vector<cv::Point3f> subset_pts3;  // 单次抽样的3D点集
       vector<cv::Point2f> subset_pts2;  // 单次抽样的2D点集
       Matrix3d R;  // 此方案的旋转矩阵
       Vector3d t;  // 此方案的平移向量
        for(int j=0; j<subset_size; j++){
            subset_pts3.push_back(pts_3[random_vec(j)]);
            subset_pts2.push_back(pts_2[random_vec(j)]);
        }
        MyPnP(subset_pts3, subset_pts2, K, D, R, t);  // 将该次抽样的3D-2D点对代入PnP方法来得到变换方案 R, t
        int score = 0;  // 此方案的得分，即内点数量
        std::vector<short> index;  // 此方案的包含的内点的索引值
        for(short j=0; j<num_points; j++){  // 计算重投影误差
            Vector3d pt3(pts_3[j].x, pts_3[j].y, pts_3[j].z);
            Vector2d pt2(unpts_2[j].x, unpts_2[j].y);
            pt3 =  R * pt3 + t;
            pt3 = pt3 / pt3(2);
            Vector2d pt3_2d(pt3(0), pt3(1));
            double error;  // 重投影误差
            error = (pt2 - pt3_2d).norm();
            error *= error;
            if (!std::isfinite(error)) break;
            if  (error < dist_threshold){  // 当某个点对误差小于内点阈值时，认为它是内点
                score++;  // 得分加一
                index.push_back(j);  // 记录这个点对在 pts_2&3 中的索引
            }
        }
        subsets_score.push_back(score);
        subsets_index.push_back(index);
    }
    // 找到最高得分对应的方案的索引
    auto max_it = std::max_element(subsets_score.begin(), subsets_score.end());
    int max_id = std::distance(subsets_score.begin(), max_it);
    std::vector<short> max_index = subsets_index[max_id];  // 最好方案的索引
    int max_score = subsets_score[max_id];  // 最高得分
    vector<cv::Point3f> max_pts3;  // 最佳方案的3D点支持集
    vector<cv::Point2f> max_pts2;  // 最佳方案的2D点支持集
    for(short i=0; i<max_index.size(); i++){
        max_pts3.push_back(pts_3[max_index[i]]);
        max_pts2.push_back(pts_2[max_index[i]]);
    }
    if(max_pts3.size()>10) MyPnP(max_pts3, max_pts2, K, D, R_res, t_res);  // 只有最佳方案的得分高于设定值时，我们才采用它
    else MyPnP(pts_3, pts_2, K, D, R_res, t_res);  // 否则还不如用传统PnP
}

/* THIS IS THE POSE ESTIMATION FUNCTION YOU NEED TO IMPLEMENT */
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(
  const vector<int> &pts_id,
  const vector<cv::Point3f> &pts_3,
  const vector<cv::Point2f> &pts_2,
  const ros::Time& frame_time)
{
    Eigen::Quaterniond Q_ref;
    Eigen::Matrix3d R_ref, R_res;
    Eigen::Vector3d t_ref, t_res;
    cv::Mat r_cv, t_cv;
    // 使用OpenCV的PnPRansac函数进行位姿估计
    cv::solvePnPRansac(pts_3, pts_2, K, D, r_cv, t_cv);
    cv::Mat Rotation;
    cv::Rodrigues(r_cv, Rotation);
    cv::cv2eigen(Rotation, R_ref);
    R_ref.transposeInPlace();
    Q_ref = R_ref;
    t_ref << t_cv.at<double>(0), t_cv.at<double>(1), t_cv.at<double>(2);
    t_ref = R_ref * t_ref;
    t_ref(1) =  -t_ref(1);
    // 发布参考路径
    publishPath(frame_time, t_ref, Q_ref, path_ref, pub_path_ref);
    
    // 使用自己的PnPRansac函数进行位姿估计
    MyPnPRansac(pts_3, pts_2, K, D, R_res, t_res);
     // // publish path
    Quaterniond Q_yourwork;
    R_res.transposeInPlace();
    Q_yourwork = R_res;
    t_res = R_res * t_res;
    t_res(1) =  -t_res(1);
    // 发布结果路径
    publishPath(frame_time, t_res, Q_yourwork, path, pub_path);

    /* For quantitative evaluation */
    double diff_psi = R_ref.eulerAngles(2, 0, 1)(0) - R_res.eulerAngles(2, 0, 1)(0);
    if (diff_psi > M_PI)
    {
      diff_psi -= 2 * M_PI;
    }
    else if (diff_psi < - M_PI)
    {
      diff_psi += 2 * M_PI;
    }

    double diff_phi = R_ref.eulerAngles(2, 0, 1)(1) - R_res.eulerAngles(2, 0, 1)(1);
    if (diff_phi > M_PI)
    {
      diff_phi -= 2 * M_PI;
    }
    else if (diff_phi < -M_PI)
    {
      diff_phi += 2 * M_PI;
    }

    double diff_theta = R_ref.eulerAngles(2, 0, 1)(2) - R_res.eulerAngles(2, 0, 1)(2);
    if (diff_theta > M_PI)
    {
      diff_theta -= 2 * M_PI;
    }
    else if (diff_theta < -M_PI)
    {
      diff_theta += 2 * M_PI;
    }

    count_frame = count_frame + 1;
    cum_error(0, 0) = cum_error(0, 0) + pow(diff_psi, 2);
    cum_error(1, 0) = cum_error(1, 0) + pow(diff_phi, 2);
    cum_error(2, 0) = cum_error(2, 0) + pow(diff_theta, 2);
    cum_error(3, 0) = cum_error(3, 0) + pow(t_cv.at<double>(0, 0) - t_ref(0),2);
    cum_error(4, 0) = cum_error(4, 0) + pow(t_cv.at<double>(1, 0) - t_ref(1),2);
    cum_error(5, 0) = cum_error(5, 0) + pow(t_cv.at<double>(2, 0) - t_ref(2),2);
    ROS_INFO("RMSE X, Y, Z, roll, pitch, yaw: \n %f, %f, %f, %f, %f, %f",
             sqrt(cum_error(3, 0) / count_frame), sqrt(cum_error(4, 0) / count_frame), sqrt(cum_error(5, 0) / count_frame),
             sqrt(cum_error(1, 0) / count_frame), sqrt(cum_error(2, 0) / count_frame), sqrt(cum_error(0, 0) / count_frame));

    cv::Mat R_ref_cv(3, 3, CV_64FC1);
    cv::eigen2cv(R_ref, R_ref_cv);

    cv::Mat t_ref_cv(3, 1, CV_64FC1);
    cv::eigen2cv(t_ref, t_ref_cv);

    calculateReprojectionError(pts_3, pts_2, R_ref_cv, t_ref_cv);
    std::cout << "ref error" << reproj_error << std::endl;
    // 创建并打开一个CSV文件
    std::ofstream file("/home/morjava/project2/data/data.csv", std::ios::app);
    if (file.is_open()) {
        // 向文件中写入数据
        file << reproj_error << ","; 
    }

    cv::Mat R_res_cv(3, 3, CV_64FC1);
    cv::eigen2cv(R_res, R_res_cv);

    cv::Mat t_res_cv(3, 1, CV_64FC1);
    cv::eigen2cv(t_res, t_res_cv);

    calculateReprojectionError(pts_3,pts_2,R_res_cv,t_res_cv);
    std::cout << "res error" << reproj_error << std::endl;
    if (file.is_open()) {
        // 向文件中写入数据
        file << reproj_error << "," <<  "\n"; 
        // 关闭文件
        file.close();
    }
    
}

// Callback function for processing the incoming images.
//
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
  /* Detect markers and obtain 3D-2D data association */
  double t = clock();
  cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  MDetector.detect(bridge_ptr->image, Markers);
  float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
  ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

  vector<int> pts_id;
  vector<cv::Point3f> pts_3;
  vector<cv::Point2f> pts_2;
  for (unsigned int i = 0; i < Markers.size(); i++)
  {
    // Obtain the ID of the marker
    int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

    char str[100];
    sprintf(str, "%d", idx);
    cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(-1));
    for (unsigned int j = 0; j < 4; j++)
    {
      sprintf(str, "%d", j);
      cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,0,1));
    }
    // j here is index of four corners
    for (unsigned int j = 0; j < 4; j++)
    {
      pts_id.push_back(Markers[i].id * 4 + j);
      pts_3.push_back(getPositionFromIndex(idx, j));
      pts_2.push_back(Markers[i][j]);
    }
  }

  /* Call your pose estimation function */
  if (pts_id.size() > 5)
    process(pts_id, pts_3, pts_2, img_msg->header.stamp);

  /* Render the detection result on the raw image */
  cv::imshow("in", bridge_ptr->image);
  cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_path_ref = n.advertise<nav_msgs::Path>("path_ref", 10);
    pub_path = n.advertise<nav_msgs::Path>("path_yourwork", 10);

    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    //init window for visualization
    cv::namedWindow("in", 1);
    
    ros::spin();
}


