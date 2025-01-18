#include <ros/ros.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/robust_kernel_factory.h>
#include <deque>
#include <map>
#include <iostream>

class GraphOptimization {
public:
    GraphOptimization() {
        // 初始化优化器
        optimizer_ = std::make_shared<g2o::SparseOptimizer>();
        auto linear_solver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
        auto block_solver = g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver));
        optimizer_->setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver)));
        optimizer_->setVerbose(false);
    }

    // 添加顶点
    void addVertex(int id, const g2o::SE3Quat& pose, bool fixed = false) {
        auto* vertex = new g2o::VertexSE3();
        vertex->setId(id);
        vertex->setEstimate(pose);
        vertex->setFixed(fixed);
        optimizer_->addVertex(vertex);
    }

    // 添加边
    void addEdge(int from_id, int to_id, const g2o::SE3Quat& relative_pose, const Eigen::Matrix<double, 6, 6>& information) {
        auto* edge = new g2o::EdgeSE3();
        edge->vertices()[0] = optimizer_->vertex(from_id);
        edge->vertices()[1] = optimizer_->vertex(to_id);
        edge->setMeasurement(relative_pose);
        edge->setInformation(information);

        // 添加鲁棒核函数
        auto* kernel = g2o::RobustKernelFactory::instance()->construct("Huber");
        if (kernel) {
            kernel->setDelta(1.0);
            edge->setRobustKernel(kernel);
        }

        optimizer_->addEdge(edge);
    }

    // 执行优化
    void optimize(int iterations = 10) {
        optimizer_->initializeOptimization();
        optimizer_->optimize(iterations);
    }

    // 获取优化后的姿态
    g2o::SE3Quat getOptimizedPose(int id) {
        auto* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(id));
        if (vertex) {
            return vertex->estimate();
        }
        throw std::runtime_error("Vertex not found in graph.");
    }

private:
    std::shared_ptr<g2o::SparseOptimizer> optimizer_;
};

// 示例：使用图优化处理回环检测结果
void processGraphOptimization(const std::vector<std::tuple<int, int, rtabmap::Transform>>& loop_closures) {
    GraphOptimization graph_optimizer;

    // 假设初始顶点姿态
    std::map<int, g2o::SE3Quat> initial_poses = {
        {0, g2o::SE3Quat(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero())},
        {1, g2o::SE3Quat(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1, 0, 0))}
    };

    // 添加初始顶点
    for (const auto& [id, pose] : initial_poses) {
        graph_optimizer.addVertex(id, pose, id == 0);  // 固定第一个顶点
    }

    // 添加边（例如里程计约束）
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
    graph_optimizer.addEdge(0, 1, initial_poses[1], information);

    // 添加回环约束
    for (const auto& [from_id, to_id, transform] : loop_closures) {
        g2o::SE3Quat loop_transform = convertTransformToSE3Quat(transform);  // 转换为 g2o 格式
        graph_optimizer.addEdge(from_id, to_id, loop_transform, information);
    }

    // 执行优化
    graph_optimizer.optimize();

    // 输出优化结果
    for (const auto& [id, pose] : initial_poses) {
        g2o::SE3Quat optimized_pose = graph_optimizer.getOptimizedPose(id);
        std::cout << "Vertex " << id << " Optimized Pose: " << optimized_pose.translation().transpose() << std::endl;
    }
}

// Transform 转换为 SE3Quat 的函数
g2o::SE3Quat convertTransformToSE3Quat(const rtabmap::Transform& transform) {
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d translation(transform.x(), transform.y(), transform.z());

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation_matrix(i, j) = transform.rotation().at<double>(i, j);
        }
    }

    Eigen::Quaterniond rotation(rotation_matrix);
    return g2o::SE3Quat(rotation, translation);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "graph_optimization");

    // 模拟回环检测结果
    std::vector<std::tuple<int, int, rtabmap::Transform>> loop_closures = {
        {1, 0, rtabmap::Transform(0.9, 0.1, 0.0, 0.0, 0.0, 0.1)}  // 示例变换
    };

    processGraphOptimization(loop_closures);

    return 0;
}


