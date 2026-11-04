#pragma once
// Filtre de Kalman Etendu (modele unicycle) - header-only, matrices de taille fixe
// (aucune allocation dynamique). Etat : [x, y, theta].
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace ekf {

    inline double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

    class ExtendedKalmanFilter {
    public:
        using Vec2 = Eigen::Vector2d;
        using Vec3 = Eigen::Vector3d;
        using Mat3 = Eigen::Matrix3d;

        // sigmaV / sigmaW : ecart-type du bruit d'odometrie (vitesse lineaire / angulaire)
        ExtendedKalmanFilter(double sigmaV, double sigmaW) : sigmaV_(sigmaV), sigmaW_(sigmaW) {
            reset(Vec3::Zero(), Mat3::Identity());
        }

        // Reinitialise l'etat ET la covariance
        void reset(const Vec3& x0, const Mat3& P0) {
            x_ = x0;
            P_ = P0;
        }

        // ---------- Prediction non lineaire ----------
        void predict(double v, double w, double dt) {
            const double th = x_(2);
            const double c = std::cos(th), s = std::sin(th);

            x_(0) += v * c * dt;
            x_(1) += v * s * dt;
            x_(2) = wrapAngle(th + w * dt);

            // Jacobienne de l'etat
            Mat3 F;
            F << 1, 0, -v * s * dt,
                0, 1, v* c* dt,
                0, 0, 1;

            // Bruit de processus issu du bruit de commande : Q = G * Qu * G^T
            Eigen::Matrix<double, 3, 2> G;
            G << c * dt, 0,
                s* dt, 0,
                0, dt;
            Eigen::Matrix2d Qu = Eigen::Matrix2d::Zero();
            Qu(0, 0) = sigmaV_ * sigmaV_;
            Qu(1, 1) = sigmaW_ * sigmaW_;

            P_ = F * P_ * F.transpose() + G * Qu * G.transpose() + Mat3::Identity() * 1e-9;
        }

        // ---------- Mise a jour generique (forme de Joseph, sans inverse explicite) ----------
        // innovation = z - h(x) ; H = dh/dx. Retourne le NIS (innovation^T S^-1 innovation).
        template <int M>
        double correct(const Eigen::Matrix<double, M, 1>& innovation,
            const Eigen::Matrix<double, M, 3>& H,
            const Eigen::Matrix<double, M, M>& R) {
            using MatM = Eigen::Matrix<double, M, M>;
            const MatM S = H * P_ * H.transpose() + R;
            const auto ldlt = S.ldlt();

            // K = P H^T S^-1 = (S^-1 H P)^T  (S et P symetriques)
            const Eigen::Matrix<double, 3, M> K = ldlt.solve(H * P_).transpose();

            x_ += K * innovation;
            x_(2) = wrapAngle(x_(2));

            const Mat3 IKH = Mat3::Identity() - K * H;
            P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

            return innovation.dot(ldlt.solve(innovation));
        }

        // Mesure lineaire de la position (GPS)
        double updatePosition(const Vec2& z, double sigma) {
            Eigen::Matrix<double, 2, 3> H;
            H << 1, 0, 0,
                0, 1, 0;
            const Vec2 y = z - H * x_;
            return correct<2>(y, H, Eigen::Matrix2d::Identity() * sigma * sigma);
        }

        // Mesure NON lineaire : distance a une balise connue -> H_k recalculee a chaque appel
        double updateRange(const Vec2& beacon, double measuredRange, double sigma) {
            const double dx = x_(0) - beacon.x();
            const double dy = x_(1) - beacon.y();
            const double r = std::max(std::sqrt(dx * dx + dy * dy), 1e-6);

            Eigen::Matrix<double, 1, 3> H;
            H << dx / r, dy / r, 0.0;
            Eigen::Matrix<double, 1, 1> y;
            y(0) = measuredRange - r;
            return correct<1>(y, H, Eigen::Matrix<double, 1, 1>::Constant(sigma * sigma));
        }

        const Vec3& state() const { return x_; }
        const Mat3& covariance() const { return P_; }
        double x() const { return x_(0); }
        double y() const { return x_(1); }
        double theta() const { return x_(2); }

    private:
        Vec3 x_;
        Mat3 P_;
        double sigmaV_, sigmaW_;
    };

} // namespace ekf