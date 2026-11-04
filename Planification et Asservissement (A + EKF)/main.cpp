// Planification (A*) + Asservissement de trajectoire + Estimation (EKF)
// Version corrigee : coordonnees coherentes (centre des cellules), obstacles gonfles,
// A* avec file de priorite, lissage du chemin, bruit gaussien, pas de temps fixe.
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <raylib.h>

#include "ekf.hpp"

namespace cfg {
    constexpr int    COLS = 40;
    constexpr int    ROWS = 30;
    constexpr int    CELL = 20;            // 1 cellule = 1 m = 20 px
    constexpr int    INFLATE = 1;          // marge de securite (en cellules) autour des obstacles

    constexpr double DT = 1.0 / 60.0;      // pas de simulation fixe
    constexpr double GPS_PERIOD = 6;       // GPS a 10 Hz (tous les 6 pas)

    constexpr double V_MAX = 2.0;          // m/s
    constexpr double W_MAX = 2.5;          // rad/s
    constexpr double KP_HEADING = 3.0;
    constexpr double WP_TOL = 0.5;         // distance de validation d'un waypoint
    constexpr double GOAL_TOL = 0.25;      // distance d'arret a l'arrivee

    constexpr double SIGMA_GPS = 0.3;      // m   (bruit gaussien du GPS)
    constexpr double SIGMA_V = 0.15;       // m/s (bruit d'odometrie lineaire)
    constexpr double SIGMA_W = 0.05;       // rad/s (bruit d'odometrie angulaire)

    constexpr size_t TRAIL_MAX = 900;
} // namespace cfg

using Vec2 = Eigen::Vector2d;

// ==========================================
// 1. A* (planification)
// ==========================================
struct Node {
    int x = 0, y = 0;
    bool isObstacle = false;   // obstacle reel
    bool isBlocked = false;    // obstacle gonfle (zone interdite au centre du robot)
    bool closed = false;
    float gCost = std::numeric_limits<float>::infinity();
    Node* parent = nullptr;
};
using Grid = std::vector<std::vector<Node>>;

static Grid makeGrid() {
    Grid g(cfg::ROWS, std::vector<Node>(cfg::COLS));
    for (int y = 0; y < cfg::ROWS; ++y)
        for (int x = 0; x < cfg::COLS; ++x) { g[y][x].x = x; g[y][x].y = y; }
    return g;
}

static void inflateObstacles(Grid& g, int radius) {
    for (auto& row : g) for (auto& n : row) n.isBlocked = n.isObstacle;
    for (int y = 0; y < cfg::ROWS; ++y)
        for (int x = 0; x < cfg::COLS; ++x) {
            if (!g[y][x].isObstacle) continue;
            for (int dy = -radius; dy <= radius; ++dy)
                for (int dx = -radius; dx <= radius; ++dx) {
                    const int nx = x + dx, ny = y + dy;
                    if (nx >= 0 && nx < cfg::COLS && ny >= 0 && ny < cfg::ROWS)
                        g[ny][nx].isBlocked = true;
                }
        }
}

static void resetSearchState(Grid& g) {
    for (auto& row : g)
        for (auto& n : row) {
            n.closed = false;
            n.parent = nullptr;
            n.gCost = std::numeric_limits<float>::infinity();
        }
}

static float heuristic(const Node* a, const Node* b) {   // Manhattan (4-connexite : admissible)
    return float(std::abs(a->x - b->x) + std::abs(a->y - b->y));
}

struct QueueEntry {
    float f, h;
    Node* node;
};
struct QueueCmp {   // plus petit f d'abord ; a f egal, plus petit h (departage -> moins de zigzags)
    bool operator()(const QueueEntry& a, const QueueEntry& b) const {
        return a.f > b.f || (a.f == b.f && a.h > b.h);
    }
};

static std::vector<Node*> solveAStar(Grid& grid, Node* start, Node* target) {
    resetSearchState(grid);   // permet la replanification

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCmp> open;
    start->gCost = 0.0f;
    open.push({ heuristic(start, target), heuristic(start, target), start });

    static const int DX[4] = { 0, 1, 0, -1 };
    static const int DY[4] = { -1, 0, 1, 0 };

    while (!open.empty()) {
        Node* current = open.top().node;
        open.pop();
        if (current->closed) continue;   // entree obsolete (suppression paresseuse)
        current->closed = true;

        if (current == target) {
            std::vector<Node*> path;
            for (Node* n = current; n != nullptr; n = n->parent) path.push_back(n);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 4; ++i) {
            const int nx = current->x + DX[i];
            const int ny = current->y + DY[i];
            if (nx < 0 || nx >= cfg::COLS || ny < 0 || ny >= cfg::ROWS) continue;

            Node* nb = &grid[ny][nx];
            if (nb->isBlocked || nb->closed) continue;

            const float g = current->gCost + 1.0f;
            if (g < nb->gCost) {
                nb->gCost = g;
                nb->parent = current;
                const float h = heuristic(nb, target);
                open.push({ g + h, h, nb });
            }
        }
    }
    return {};   // cible inatteignable
}

// Centre d'une cellule en coordonnees monde (metres)
static Vec2 cellCenter(const Node* n) { return Vec2(n->x + 0.5, n->y + 0.5); }

static bool cellBlockedAt(const Grid& g, double wx, double wy) {
    const int cx = int(std::floor(wx)), cy = int(std::floor(wy));
    if (cx < 0 || cx >= cfg::COLS || cy < 0 || cy >= cfg::ROWS) return true;
    return g[cy][cx].isBlocked;
}

static bool lineFree(const Grid& g, const Vec2& a, const Vec2& b) {
    const double len = (b - a).norm();
    const int steps = std::max(1, int(len / 0.1));
    for (int i = 0; i <= steps; ++i) {
        const Vec2 p = a + (b - a) * (double(i) / steps);
        if (cellBlockedAt(g, p.x(), p.y())) return false;
    }
    return true;
}

// Lissage : on saute les waypoints intermediaires tant que la ligne droite reste libre
static std::vector<Vec2> smoothPath(const Grid& g, const std::vector<Vec2>& raw) {
    if (raw.size() <= 2) return raw;
    std::vector<Vec2> out{ raw.front() };
    size_t i = 0;
    while (i < raw.size() - 1) {
        size_t j = raw.size() - 1;
        while (j > i + 1 && !lineFree(g, raw[i], raw[j])) --j;
        out.push_back(raw[j]);
        i = j;
    }
    return out;
}

// ==========================================
// 2. Simulation (verite terrain + capteurs + EKF + commande)
// ==========================================
struct Truth { double x, y, theta; };

class Simulation {
public:
    Simulation()
        : grid_(makeGrid()), ekf_(cfg::SIGMA_V, cfg::SIGMA_W) {
        for (int y = 10; y < 25; ++y) grid_[y][15].isObstacle = true;
        for (int x = 15; x < 25; ++x) grid_[24][x].isObstacle = true;
        inflateObstacles(grid_, cfg::INFLATE);
        restart(Vec2(5.5, 5.5), 35, 25);
    }

    void restart(const Vec2& start, int goalX, int goalY) {
        truth_ = { start.x(), start.y(), 0.0 };
        ekf::ExtendedKalmanFilter::Vec3 x0(start.x(), start.y(), 0.0);
        ekf_.reset(x0, Eigen::Matrix3d::Identity() * 0.1);
        lastGps_ = start;
        trailTruth_.clear();
        trailEkf_.clear();
        collisions_ = 0;
        sumSqEkf_ = sumSqGps_ = 0.0;
        samples_ = gpsSamples_ = 0;
        tick_ = 0;
        planTo(start, goalX, goalY);
    }

    // (Re)planification depuis une position vers une cellule but
    bool planTo(const Vec2& from, int gx, int gy) {
        const int sx = std::clamp(int(std::floor(from.x())), 0, cfg::COLS - 1);
        const int sy = std::clamp(int(std::floor(from.y())), 0, cfg::ROWS - 1);
        if (grid_[gy][gx].isBlocked) { hasPath_ = false; path_.clear(); return false; }

        goalX_ = gx; goalY_ = gy;
        std::vector<Node*> raw = solveAStar(grid_, &grid_[sy][sx], &grid_[gy][gx]);
        if (raw.empty()) { hasPath_ = false; path_.clear(); return false; }

        std::vector<Vec2> pts;
        pts.reserve(raw.size());
        for (const Node* n : raw) pts.push_back(cellCenter(n));
        path_ = smoothPath(grid_, pts);
        idx_ = path_.size() > 1 ? 1 : 0;
        hasPath_ = true;
        arrived_ = false;
        return true;
    }

    void step() {
        // ---- Loi de commande (basee sur l'ESTIMATION EKF, pas sur la verite) ----
        double v = 0.0, w = 0.0;
        if (hasPath_ && !arrived_) {
            const Vec2 p(ekf_.x(), ekf_.y());
            const size_t last = path_.size() - 1;

            if ((path_[idx_] - p).norm() < cfg::WP_TOL && idx_ < last) ++idx_;
            const Vec2 target = path_[idx_];
            const double dist = (target - p).norm();

            if (idx_ == last && dist < cfg::GOAL_TOL) {
                arrived_ = true;
            }
            else {
                const double desired = std::atan2(target.y() - p.y(), target.x() - p.x());
                const double err = ekf::wrapAngle(desired - ekf_.theta());
                w = std::clamp(cfg::KP_HEADING * err, -cfg::W_MAX, cfg::W_MAX);
                v = cfg::V_MAX * std::clamp(1.0 - std::abs(err) / 1.2, 0.1, 1.0);
                if (idx_ == last) v = std::min(v, 2.0 * dist);   // freinage progressif
            }
        }
        cmdV_ = v; cmdW_ = w;

        // ---- Verite terrain : odometrie reelle bruitee ----
        const double vTrue = v + cfg::SIGMA_V * n01_(rng_) * (v != 0.0);
        const double wTrue = w + cfg::SIGMA_W * n01_(rng_) * (v != 0.0);
        truth_.x += vTrue * std::cos(truth_.theta) * cfg::DT;
        truth_.y += vTrue * std::sin(truth_.theta) * cfg::DT;
        truth_.theta = ekf::wrapAngle(truth_.theta + wTrue * cfg::DT);

        // Detection de collision sur la vraie position
        if (cellIsRealObstacle(truth_.x, truth_.y)) ++collisions_;

        // ---- EKF : prediction avec les commandes (sans connaitre le bruit) ----
        ekf_.predict(v, w, cfg::DT);

        // ---- GPS a 10 Hz, bruit gaussien ----
        if (tick_ % int(cfg::GPS_PERIOD) == 0) {
            lastGps_ = Vec2(truth_.x + cfg::SIGMA_GPS * n01_(rng_),
                truth_.y + cfg::SIGMA_GPS * n01_(rng_));
            ekf_.updatePosition(lastGps_, cfg::SIGMA_GPS);
            sumSqGps_ += (lastGps_ - Vec2(truth_.x, truth_.y)).squaredNorm();
            ++gpsSamples_;
        }
        sumSqEkf_ += (Vec2(ekf_.x(), ekf_.y()) - Vec2(truth_.x, truth_.y)).squaredNorm();
        ++samples_;
        ++tick_;

        if (tick_ % 3 == 0) {
            pushTrail(trailTruth_, Vec2(truth_.x, truth_.y));
            pushTrail(trailEkf_, Vec2(ekf_.x(), ekf_.y()));
        }
    }

    // ---- Accesseurs (lecture seule pour le rendu) ----
    const Grid& grid() const { return grid_; }
    const std::vector<Vec2>& path() const { return path_; }
    const Truth& truth() const { return truth_; }
    const ekf::ExtendedKalmanFilter& ekf() const { return ekf_; }
    const Vec2& lastGps() const { return lastGps_; }
    const std::deque<Vec2>& trailTruth() const { return trailTruth_; }
    const std::deque<Vec2>& trailEkf() const { return trailEkf_; }
    bool hasPath() const { return hasPath_; }
    bool arrived() const { return arrived_; }
    int goalX() const { return goalX_; }
    int goalY() const { return goalY_; }
    int collisions() const { return collisions_; }
    double cmdV() const { return cmdV_; }
    double cmdW() const { return cmdW_; }
    double rmseEkf() const { return samples_ ? std::sqrt(sumSqEkf_ / samples_) : 0.0; }
    double rmseGps() const { return gpsSamples_ ? std::sqrt(sumSqGps_ / gpsSamples_) : 0.0; }

private:
    bool cellIsRealObstacle(double wx, double wy) const {
        const int cx = int(std::floor(wx)), cy = int(std::floor(wy));
        if (cx < 0 || cx >= cfg::COLS || cy < 0 || cy >= cfg::ROWS) return true;
        return grid_[cy][cx].isObstacle;
    }
    static void pushTrail(std::deque<Vec2>& t, const Vec2& p) {
        t.push_back(p);
        if (t.size() > cfg::TRAIL_MAX) t.pop_front();
    }

    Grid grid_;
    std::vector<Vec2> path_;
    size_t idx_ = 0;
    bool hasPath_ = false, arrived_ = false;
    int goalX_ = 0, goalY_ = 0;

    ekf::ExtendedKalmanFilter ekf_;
    Truth truth_{ 0, 0, 0 };
    Vec2 lastGps_ = Vec2::Zero();

    std::mt19937 rng_{ 42 };   // graine fixe -> execution reproductible
    std::normal_distribution<double> n01_{ 0.0, 1.0 };

    std::deque<Vec2> trailTruth_, trailEkf_;
    int collisions_ = 0, tick_ = 0;
    double cmdV_ = 0.0, cmdW_ = 0.0;
    double sumSqEkf_ = 0.0, sumSqGps_ = 0.0;
    long samples_ = 0, gpsSamples_ = 0;
};

// ==========================================
// 3. Rendu
// ==========================================
static Vector2 toPx(const Vec2& p) {
    return { float(p.x() * cfg::CELL), float(p.y() * cfg::CELL) };
}

static void drawTrail(const std::deque<Vec2>& t, Color c) {
    for (size_t i = 1; i < t.size(); ++i) DrawLineV(toPx(t[i - 1]), toPx(t[i]), c);
}

static void render(const Simulation& sim) {
    ClearBackground(RAYWHITE);

    // Grille, zones de securite et obstacles
    for (int y = 0; y < cfg::ROWS; ++y)
        for (int x = 0; x < cfg::COLS; ++x) {
            const Node& n = sim.grid()[y][x];
            Rectangle r = { float(x * cfg::CELL), float(y * cfg::CELL), float(cfg::CELL), float(cfg::CELL) };
            if (n.isObstacle) DrawRectangleRec(r, DARKGRAY);
            else if (n.isBlocked) DrawRectangleRec(r, Fade(RED, 0.12f));
            else DrawRectangleLinesEx(r, 1, Fade(LIGHTGRAY, 0.5f));
        }

    // But
    DrawRectangle(sim.goalX() * cfg::CELL + 3, sim.goalY() * cfg::CELL + 3,
        cfg::CELL - 6, cfg::CELL - 6, Fade(GREEN, 0.7f));

    // Chemin A* lisse (au centre des cellules, comme le robot)
    const auto& path = sim.path();
    for (size_t i = 1; i < path.size(); ++i) DrawLineEx(toPx(path[i - 1]), toPx(path[i]), 2, ORANGE);
    for (const Vec2& p : path) DrawCircleV(toPx(p), 4, ORANGE);

    // Traces
    drawTrail(sim.trailTruth(), Fade(DARKGREEN, 0.6f));
    drawTrail(sim.trailEkf(), Fade(BLUE, 0.6f));

    // Mesure GPS (rouge), verite terrain (vert), estimation EKF (bleu)
    DrawCircleV(toPx(sim.lastGps()), 3, RED);
    DrawCircleV(toPx(Vec2(sim.truth().x, sim.truth().y)), 6, DARKGREEN);

    const Vec2 e(sim.ekf().x(), sim.ekf().y());
    DrawCircleV(toPx(e), 10, Fade(BLUE, 0.8f));
    const Vec2 head = e + 1.0 * Vec2(std::cos(sim.ekf().theta()), std::sin(sim.ekf().theta()));
    DrawLineEx(toPx(e), toPx(head), 3, DARKBLUE);

    // HUD (ASCII uniquement : la police par defaut de raylib n'a pas les accents)
    DrawText("Rouge: GPS | Bleu: estimation EKF | Vert: verite terrain | Clic: nouveau but", 10, 10, 16, DARKGRAY);
    DrawText(TextFormat("Commande: v=%.1f m/s  w=%.1f rad/s", sim.cmdV(), sim.cmdW()), 10, 30, 16, DARKGREEN);
    DrawText(TextFormat("RMSE position  EKF: %.3f m  |  GPS brut: %.3f m", sim.rmseEkf(), sim.rmseGps()), 10, 50, 16, DARKBLUE);
    DrawText(TextFormat("Collisions (pas): %d", sim.collisions()), 10, 70, 16, sim.collisions() ? RED : DARKGRAY);

    if (!sim.hasPath()) DrawText("AUCUN CHEMIN : cible inatteignable ou bloquee", 10, 90, 20, RED);
    else if (sim.arrived()) DrawText("ARRIVE", 10, 90, 20, DARKGREEN);
}

// ==========================================
// 4. Programme principal
// ==========================================
int main() {
    InitWindow(cfg::COLS * cfg::CELL, cfg::ROWS * cfg::CELL, "Portfolio R&D - Asservissement sur trajectoire (A* + EKF)");
    SetTargetFPS(60);

    Simulation sim;
    double accumulator = 0.0;

    while (!WindowShouldClose()) {
        // Clic gauche : nouveau but (replanification depuis la position estimee)
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            const Vector2 m = GetMousePosition();
            const int gx = int(m.x) / cfg::CELL, gy = int(m.y) / cfg::CELL;
            if (gx >= 0 && gx < cfg::COLS && gy >= 0 && gy < cfg::ROWS)
                sim.planTo(Vec2(sim.ekf().x(), sim.ekf().y()), gx, gy);
        }
        if (IsKeyPressed(KEY_R)) sim.restart(Vec2(5.5, 5.5), 35, 25);

        // Pas de temps fixe, independant de la cadence d'affichage
        accumulator += std::min(double(GetFrameTime()), 0.1);
        while (accumulator >= cfg::DT) {
            sim.step();
            accumulator -= cfg::DT;
        }

        BeginDrawing();
        render(sim);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}