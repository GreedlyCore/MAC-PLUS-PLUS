#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Eva.h"

namespace py = pybind11;

// Registration entry point from registration.cpp
bool registration(const std::string &name,
                  const std::string &src_pointcloud,
                  const std::string &des_pointcloud,
                  const std::string &corr_path,
                  const std::string &label_path,
                  const std::string &ov_label,
                  const std::string &gt_mat,
                  const std::string &folderPath,
                  const std::string &descriptor,
                  double &time_epoch,
                  double &mem_epoch,
                  std::vector<double> &time_number,
                  float &RE,
                  float &TE,
                  int &correct_est_num,
                  int &inlier_num,
                  int &total_num,
                  std::vector<double> &pred_inlier);

static py::array_t<double> identity_result() {
    py::array_t<double> result({4, 4});
    auto r = result.mutable_unchecked<2>();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            r(i, j) = (i == j) ? 1.0 : 0.0;
        }
    }
    return result;
}

static std::string make_temp_dir() {
    char tmpl[] = "/tmp/macpp_solver_XXXXXX";
    char *dir = mkdtemp(tmpl);
    if (dir == nullptr) {
        throw std::runtime_error("mkdtemp failed for macpp_solver");
    }
    return std::string(dir);
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr points_from_numpy(
    py::array_t<float, py::array::c_style | py::array::forcecast> xyz) {
    auto arr = xyz.unchecked<2>();
    const ssize_t n = arr.shape(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(static_cast<size_t>(n));
    for (ssize_t i = 0; i < n; ++i) {
        pcl::PointXYZ p;
        p.x = arr(i, 0);
        p.y = arr(i, 1);
        p.z = arr(i, 2);
        cloud->push_back(p);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

static void write_corr_file(
    const std::string &corr_path,
    py::array_t<float, py::array::c_style | py::array::forcecast> src_xyz,
    py::array_t<float, py::array::c_style | py::array::forcecast> tgt_xyz) {
    auto src = src_xyz.unchecked<2>();
    auto tgt = tgt_xyz.unchecked<2>();

    std::ofstream out(corr_path);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open correspondence file: " + corr_path);
    }

    for (ssize_t i = 0; i < src.shape(0); ++i) {
        out << src(i, 0) << " " << src(i, 1) << " " << src(i, 2) << " "
            << tgt(i, 0) << " " << tgt(i, 1) << " " << tgt(i, 2) << "\n";
    }
}

static void write_label_file(const std::string &label_path, ssize_t n_corr) {
    std::ofstream out(label_path);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open label file: " + label_path);
    }
    for (ssize_t i = 0; i < n_corr; ++i) {
        out << 1 << "\n";
    }
}

static void write_identity_gt(const std::string &gt_path) {
    std::ofstream out(gt_path);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open GT matrix file: " + gt_path);
    }
    out << "1 0 0 0\n";
    out << "0 1 0 0\n";
    out << "0 0 1 0\n";
    out << "0 0 0 1\n";
}

static py::array_t<double> read_estimate_or_identity(const std::string &est_path) {
    std::ifstream in(est_path);
    if (!in.is_open()) {
        return identity_result();
    }

    py::array_t<double> result({4, 4});
    auto r = result.mutable_unchecked<2>();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double v = 0.0;
            if (!(in >> v)) {
                return identity_result();
            }
            r(i, j) = v;
        }
    }
    return result;
}

static py::array_t<double> macpp_solve(
    py::array_t<float, py::array::c_style | py::array::forcecast> src_xyz,
    py::array_t<float, py::array::c_style | py::array::forcecast> tgt_xyz,
    float inlier_thresh,
    const std::string &dataset_name = "3dmatch",
    const std::string &descriptor = "fpfh") {

    (void)inlier_thresh;

    if (src_xyz.ndim() != 2 || tgt_xyz.ndim() != 2 ||
        src_xyz.shape(1) != 3 || tgt_xyz.shape(1) != 3 ||
        src_xyz.shape(0) != tgt_xyz.shape(0)) {
        throw std::invalid_argument("src_xyz and tgt_xyz must both be (K, 3) float32 arrays");
    }

    // MAC++ internals assume a reasonably sized correspondence set.
    // For tiny K, return identity instead of entering unstable code paths.
    // TODO: why not < 3 (???)
    if (src_xyz.shape(0) < 20) {
        return identity_result();
    }

    const std::string work_dir = make_temp_dir();
    const std::string src_ply = work_dir + "/src.ply";
    const std::string tgt_ply = work_dir + "/tgt.ply";
    const std::string src_pcd = work_dir + "/src.pcd";
    const std::string tgt_pcd = work_dir + "/tgt.pcd";
    const std::string src_kpts_pcd = work_dir + "/src_kpts.pcd";
    const std::string tgt_kpts_pcd = work_dir + "/tgt_kpts.pcd";
    const std::string corr_path = work_dir + "/corr.txt";
    const std::string label_path = work_dir + "/label.txt";
    const std::string gt_path = work_dir + "/GTmat.txt";
    const std::string out_dir = work_dir + "/result";

    std::string mkdir_cmd = "mkdir -p " + out_dir;
    std::system(mkdir_cmd.c_str());

    auto src_cloud = points_from_numpy(src_xyz);
    auto tgt_cloud = points_from_numpy(tgt_xyz);

    pcl::io::savePLYFileBinary(src_ply, *src_cloud);
    pcl::io::savePLYFileBinary(tgt_ply, *tgt_cloud);
    pcl::io::savePCDFileBinary(src_pcd, *src_cloud);
    pcl::io::savePCDFileBinary(tgt_pcd, *tgt_cloud);
    // MAC++ KITTI/U3M code paths expect these exact keypoint sidecar names.
    pcl::io::savePCDFileBinary(src_kpts_pcd, *src_cloud);
    pcl::io::savePCDFileBinary(tgt_kpts_pcd, *tgt_cloud);

    write_corr_file(corr_path, src_xyz, tgt_xyz);
    write_label_file(label_path, src_xyz.shape(0));
    write_identity_gt(gt_path);

    double time_epoch = 0.0;
    double mem_epoch = 0.0;
    std::vector<double> time_number(4, 0.0);
    float RE = 0.0f;
    float TE = 0.0f;
    int correct_est_num = 0;
    int inlier_num = 0;
    int total_num = 0;
    std::vector<double> pred_inlier;

    registration(dataset_name,
                 src_ply,
                 tgt_ply,
                 corr_path,
                 label_path,
                 "NULL",
                 gt_path,
                 out_dir,
                 descriptor,
                 time_epoch,
                 mem_epoch,
                 time_number,
                 RE,
                 TE,
                 correct_est_num,
                 inlier_num,
                 total_num,
                 pred_inlier);

    return read_estimate_or_identity(out_dir + "/est.txt");
}

PYBIND11_MODULE(macpp_solver, m) {
    m.doc() = "MAC++ pybind interface";

    m.def("macpp_solve",
          &macpp_solve,
          py::arg("src_xyz"),
          py::arg("tgt_xyz"),
          py::arg("inlier_thresh"),
          py::arg("dataset_name") = "3dmatch",
          py::arg("descriptor") = "fpfh",
          R"doc(
Run MAC++ on precomputed correspondences.

Args:
    src_xyz: (K, 3) float32 numpy array (source points)
    tgt_xyz: (K, 3) float32 numpy array (target points)
    inlier_thresh: compatibility argument for API parity with mac_solve
    dataset_name: one of 3dmatch, 3dlomatch, KITTI, U3M
    descriptor: descriptor tag expected by MAC++ (default: fpfh)

Returns:
    (4, 4) float64 transform matrix.
)doc");
}
