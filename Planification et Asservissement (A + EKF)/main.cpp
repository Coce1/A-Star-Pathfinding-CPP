#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <raylib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==========================================
// 1. PARAMÈTRES SPATIAUX
// ==========================================
const int COLS = 40;
const int ROWS = 30;
const int CELL_SIZE = 20; // 1 Cellule = 1 Mètre = 20 Pixels

// ==========================================
// 2. ARCHITECTURE A* (Planification)
// ==========================================
struct Node {
    int x, y;
    bool isObstacle = false;
    float gCost = INFINITY, hCost = 0, fCost = INFINITY;
    Node* parent = nullptr;
};

float getHeuristic(Node* a, Node* b) {
    return std::abs(a->x - b->x) + std::abs(a->y - b->y);
}

std::vector<Node*> solveAStar(std::vector<std::vector<Node>>& grid, Node* start, Node* target) {
    std::vector<Node*> openSet;
    std::vector<Node*> closedSet;

    start->gCost = 0;
    start->hCost = getHeuristic(start, target);
    start->fCost = start->hCost;
    openSet.push_back(start);

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        for (auto it = openSet.begin(); it != openSet.end(); ++it) {
            if ((*it)->fCost < (*current_it)->fCost) current_it = it;
        }
        Node* current = *current_it;

        if (current == target) {
            std::vector<Node*> path;
            while (current != nullptr) {
                path.push_back(current);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end()); // Inversion pour aller de Start vers Target
            return path;
        }

        openSet.erase(current_it);
        closedSet.push_back(current);

        int dx[] = { 0, 1, 0, -1 };
        int dy[] = { -1, 0, 1, 0 };

        for (int i = 0; i < 4; i++) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS) {
                Node* neighbor = &grid[ny][nx];
                if (neighbor->isObstacle || std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()) continue;

                float tentative_gCost = current->gCost + 1.0f;
                if (tentative_gCost < neighbor->gCost) {
                    neighbor->parent = current;
                    neighbor->gCost = tentative_gCost;
                    neighbor->hCost = getHeuristic(neighbor, target);
                    neighbor->fCost = neighbor->gCost + neighbor->hCost;
                    if (std::find(openSet.begin(), openSet.end(), neighbor) == openSet.end()) openSet.push_back(neighbor);
                }
            }
        }
    }
    return {};
}

// ==========================================
// 3. FILTRE DE KALMAN ÉTENDU (Estimation)
// ==========================================
class ExtendedKalmanFilter {
private:
    Eigen::VectorXd x_hat;
    Eigen::MatrixXd P, Q, R, I, H;
public:
    ExtendedKalmanFilter() {
        x_hat = Eigen::VectorXd::Zero(3);
        P = Eigen::MatrixXd::Identity(3, 3);
        Q = Eigen::MatrixXd::Identity(3, 3) * 0.05; // Modèle physique
        R = Eigen::MatrixXd::Identity(2, 2) * 1.5;  // Capteur GPS bruité
        H = Eigen::MatrixXd::Zero(2, 3);
        H << 1, 0, 0,
            0, 1, 0;
        I = Eigen::MatrixXd::Identity(3, 3);
    }
    void setInitialState(double x, double y, double theta) { x_hat << x, y, theta; }

    void predict(double v, double w, double dt) {
        double theta = x_hat(2);
        x_hat(0) += v * cos(theta) * dt;
        x_hat(1) += v * sin(theta) * dt;
        x_hat(2) += w * dt;

        Eigen::MatrixXd J_F(3, 3);
        J_F << 1, 0, -v * sin(theta) * dt,
            0, 1, v* cos(theta)* dt,
            0, 0, 1;
        P = J_F * P * J_F.transpose() + Q;
    }

    void update(const Eigen::VectorXd& z) {
        Eigen::VectorXd y = z - H * x_hat;
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();
        x_hat = x_hat + K * y;
        P = (I - K * H) * P;
    }

    double getX() { return x_hat(0); }
    double getY() { return x_hat(1); }
    double getTheta() { return x_hat(2); }
};

// ==========================================
// 4. PROGRAMME PRINCIPAL
// ==========================================
int main() {
    InitWindow(COLS * CELL_SIZE, ROWS * CELL_SIZE, "Portfolio R&D - Asservissement sur Trajectoire (A* + EKF)");
    SetTargetFPS(60);

    // Initialisation Environnement
    std::vector<std::vector<Node>> grid(ROWS, std::vector<Node>(COLS));
    for (int y = 0; y < ROWS; y++)
        for (int x = 0; x < COLS; x++)
            grid[y][x] = { x, y, false };

    for (int y = 10; y < 25; y++) grid[y][15].isObstacle = true;
    for (int x = 15; x < 25; x++) grid[24][x].isObstacle = true;

    Node* startNode = &grid[5][5];
    Node* targetNode = &grid[25][35];
    std::vector<Node*> path = solveAStar(grid, startNode, targetNode);

    // Initialisation Robot (Système Dynamique)
    ExtendedKalmanFilter ekf;
    ekf.setInitialState(5.0, 5.0, 0.0);
    double vrai_x = 5.0, vrai_y = 5.0, vrai_theta = 0.0;
    Eigen::VectorXd z(2);

    int targetIndex = 0;
    double dt = 1.0 / 60.0;
    const double LOOKAHEAD_DIST = 2.0; // Le robot vise toujours 2 mètres (cases) devant lui

    while (!WindowShouldClose()) {
        // --- LOI DE COMMANDE (ASSERVISSEMENT CAP) ---
        double v = 0.0;
        double w = 0.0;

        if (targetIndex < path.size()) {
            // Anticipation du prochain Waypoint
            Node* targetWP = path[targetIndex];
            double dist_to_wp = sqrt(pow(targetWP->x - ekf.getX(), 2) + pow(targetWP->y - ekf.getY(), 2));

            if (dist_to_wp < LOOKAHEAD_DIST && targetIndex < path.size() - 1) {
                targetIndex++;
            }

            // Correcteur Proportionnel pour l'angle
            double desired_theta = atan2(targetWP->y - ekf.getY(), targetWP->x - ekf.getX());
            double error_theta = desired_theta - ekf.getTheta();

            // Normalisation de l'erreur entre -PI et PI pour éviter que le robot tourne sur lui-même
            while (error_theta > M_PI) error_theta -= 2.0 * M_PI;
            while (error_theta < -M_PI) error_theta += 2.0 * M_PI;

            double Kp = 3.5; // Gain du correcteur
            w = Kp * error_theta;

            // Saturation de la vitesse de rotation
            w = std::clamp(w, -2.0, 2.0);
            v = 3.0; // Vitesse d'avancée constante (3 m/s)

            // Freinage à l'arrivée
            if (targetIndex == path.size() - 1 && dist_to_wp < 0.5) {
                v = 0.0;
                w = 0.0;
            }
        }

        // --- VÉRITÉ TERRAIN & CAPTEUR BRUITÉ ---
        vrai_x += v * cos(vrai_theta) * dt;
        vrai_y += v * sin(vrai_theta) * dt;
        vrai_theta += w * dt;

        z(0) = vrai_x + (GetRandomValue(-40, 40) / 100.0);
        z(1) = vrai_y + (GetRandomValue(-40, 40) / 100.0);

        // --- ESTIMATION EKF ---
        ekf.predict(v, w, dt);
        ekf.update(z);

        // --- RENDU GRAPHIQUE ---
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Grille et Obstacles
        for (int y = 0; y < ROWS; y++) {
            for (int x = 0; x < COLS; x++) {
                Rectangle rect = { (float)(x * CELL_SIZE), (float)(y * CELL_SIZE), (float)CELL_SIZE, (float)CELL_SIZE };
                if (grid[y][x].isObstacle) DrawRectangleRec(rect, DARKGRAY);
                else DrawRectangleLinesEx(rect, 1, Fade(LIGHTGRAY, 0.5f));
            }
        }

        // Chemin A* (Ligne directrice théorique)
        for (Node* n : path) {
            DrawRectangle(n->x * CELL_SIZE + CELL_SIZE / 4, n->y * CELL_SIZE + CELL_SIZE / 4, CELL_SIZE / 2, CELL_SIZE / 2, ORANGE);
        }

        // Capteur GPS (Rouge)
        DrawCircle((int)(z(0) * CELL_SIZE), (int)(z(1) * CELL_SIZE), 3, RED);

        // Robot Estimé EKF (Bleu)
        int robot_px = (int)(ekf.getX() * CELL_SIZE);
        int robot_py = (int)(ekf.getY() * CELL_SIZE);
        DrawCircle(robot_px, robot_py, 10, BLUE);

        // Vecteur Direction
        DrawLineEx(
            { (float)robot_px, (float)robot_py },
            { (float)(robot_px + 20 * cos(ekf.getTheta())), (float)(robot_py + 20 * sin(ekf.getTheta())) },
            3, DARKBLUE
        );

        DrawText("Point Rouge : GPS | Point Bleu : Modèle Asservi par EKF", 10, 10, 20, DARKGRAY);
        DrawText(TextFormat("Consigne Moteur : v=%.1fm/s, w=%.1frad/s", v, w), 10, 40, 20, DARKGREEN);

        EndDrawing();
    }
    CloseWindow();
    return 0;
}