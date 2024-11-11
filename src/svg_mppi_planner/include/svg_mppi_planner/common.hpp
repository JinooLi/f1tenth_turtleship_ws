#pragma once

#include <string>
#include <Eigen/Dense>

namespace svg_mppi {

/**
 * @brief ko)상태공간내 원소의 순서(인덱스)와 상태공간의 차원 정의 
 * @brief en)Definition of the order (index) and dimension of the elements in the state space
 */
namespace STATE_SPACE {
    static constexpr int x = 0; // x-coordinate index
    static constexpr int y = 1; // y-coordinate index
    static constexpr int yaw = 2; // yaw angle index
    static constexpr int velocity = 3; // velocity index
    static constexpr int steering = 4; // steering angle index
    static constexpr int dim = 5; // dimension of state space
} // STATE_SPACE

/**
 * @brief ko)제어공간의 순서(인덱스)와 차원 정의 
 * @brief en)Definition of the order (index) and dimension of the control space
 */
namespace CONTROL_SPACE {
    static constexpr int steering = 0; // steering angle index
    static constexpr int dim = 1; // dimension of control space
} // CONTROL_SPACE

struct Parameters {
    struct Common {
        int PREDICTION_STEP_SIZE;
        double PREDICTION_INTERVAL;
        int lf;
        int lr;
        double MAX_STEERING_ANGLE;
        double MIN_STEERING_ANGLE;
    };
    Common common;

    struct SVG_MPPI {
        double LAMBDA;
        double ALPHA;
        double NON_BIASED_SAMPLING_RATE;
    };
    SVG_MPPI svg_mppi;
};

namespace planning {
    using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
    using Control = Eigen::Matrix<double, CONTROL_SPACE::dim, 1>;

    using StateSequence = Eigen::MatrixXd;
    using ControlSequence = Eigen::MatrixXd;

    // 각 원소가 Matrix인 Vector.
    // 이때, 하나의 Matrix는 한 trajectory 내 각각의 state vector가 column vector인 Matrix이다.
    using StateSequenceBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    // 각 원소가 Matrix인 Vector.
    // 이때, 하나의 Matrix는 한 trajectory 내 각각의 control vector가 column vector인 Matrix이다.
    using ControlSequenceBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;

    using ControlCovarianceSequence = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
} // namespace planning
} // namespace svg_mppi
