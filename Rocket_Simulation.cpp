#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>

// Constants
const double PI = 3.14159265358979323846;
const double AIR_DENSITY = 1.225; // kg/m^3, density of air at sea level

// Struct to store simulation data
struct State {
    double x;      // Horizontal position
    double y;      // Vertical position
    double vx;     // Horizontal velocity
    double vy;     // Vertical velocity
};

// Struct to store parameters
struct Parameters {
    double mass;
    double dragCoefficient;
    double crossSectionalArea;
    double gravity;
    double windSpeed;
};

// Struct to store trajectory data points
struct TrajectoryPoint {
    double time;
    double x;
    double y;
    double velocity;
};

// Function to convert degrees to radians
double toRadians(double degrees) {
    return (degrees * PI) / 180.0;
}

// Function to calculate the drag force
double calculateDragForce(double velocity, double dragCoefficient, double crossSectionalArea) {
    return 0.5 * AIR_DENSITY * dragCoefficient * crossSectionalArea * velocity * velocity;
}

// Function to calculate acceleration using forces
State calculateAcceleration(const State& state, const Parameters& params) {
    double velocity = std::sqrt(state.vx * state.vx + state.vy * state.vy);
    double dragForce = calculateDragForce(velocity, params.dragCoefficient, params.crossSectionalArea);
    double dragAcceleration = dragForce / params.mass;

    State acceleration;
    acceleration.vx = -dragAcceleration * (state.vx / velocity) + params.windSpeed / params.mass;
    acceleration.vy = -params.gravity - dragAcceleration * (state.vy / velocity);

    return acceleration;
}

// Runge-Kutta 4th Order Method to solve differential equations
State rungeKuttaStep(const State& currentState, const Parameters& params, double timeStep) {
    State k1 = calculateAcceleration(currentState, params);
    State k2 = calculateAcceleration({currentState.x + 0.5 * timeStep * currentState.vx,
                                      currentState.y + 0.5 * timeStep * currentState.vy,
                                      currentState.vx + 0.5 * timeStep * k1.vx,
                                      currentState.vy + 0.5 * timeStep * k1.vy}, params);
    State k3 = calculateAcceleration({currentState.x + 0.5 * timeStep * currentState.vx,
                                      currentState.y + 0.5 * timeStep * currentState.vy,
                                      currentState.vx + 0.5 * timeStep * k2.vx,
                                      currentState.vy + 0.5 * timeStep * k2.vy}, params);
    State k4 = calculateAcceleration({currentState.x + timeStep * currentState.vx,
                                      currentState.y + timeStep * currentState.vy,
                                      currentState.vx + timeStep * k3.vx,
                                      currentState.vy + timeStep * k3.vy}, params);

    State nextState;
    nextState.x = currentState.x + timeStep * currentState.vx;
    nextState.y = currentState.y + timeStep * currentState.vy;
    nextState.vx = currentState.vx + (timeStep / 6.0) * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx);
    nextState.vy = currentState.vy + (timeStep / 6.0) * (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy);

    return nextState;
}

// Function to simulate the rocket trajectory using Runge-Kutta
std::vector<TrajectoryPoint> simulateTrajectory(
    double initialVelocity, double launchAngle, const Parameters& params, double timeStep
) {
    double angleInRadians = toRadians(launchAngle);
    State state = {0.0, 0.0, initialVelocity * cos(angleInRadians), initialVelocity * sin(angleInRadians)};
    std::vector<TrajectoryPoint> trajectory;
    double time = 0.0;

    // Simulation loop
    while (state.y >= 0) {
        trajectory.push_back({time, state.x, state.y, std::sqrt(state.vx * state.vx + state.vy * state.vy)});
        state = rungeKuttaStep(state, params, timeStep);
        time += timeStep;
    }

    return trajectory;
}

// Function to print the trajectory data
void printTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time (s) | X Position (m) | Y Position (m) | Velocity (m/s)\n";
    std::cout << "------------------------------------------------------------\n";
    for (const auto& point : trajectory) {
        std::cout << std::setw(8) << point.time << " | "
                  << std::setw(14) << point.x << " | "
                  << std::setw(13) << point.y << " | "
                  << std::setw(13) << point.velocity << "\n";
    }
}

// Function to visualize the trajectory using SFML
void visualizeTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Rocket Trajectory Simulation");
    window.setFramerateLimit(60);

    sf::VertexArray path(sf::LinesStrip, trajectory.size());

    // Scale factors for visualization
    double xScale = 5.0;
    double yScale = 5.0;

    // Convert trajectory points to SFML vertices
    for (size_t i = 0; i < trajectory.size(); ++i) {
        path[i].position = sf::Vector2f(trajectory[i].x * xScale, 600 - trajectory[i].y * yScale);
        path[i].color = sf::Color::Red;
    }

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(path);
        window.display();
    }
}

int main() {
    // User input
    double initialVelocity, launchAngle, gravity, dragCoefficient, crossSectionalArea, windSpeed, mass;

    std::cout << "Enter the initial velocity (m/s): ";
    std::cin >> initialVelocity;
    std::cout << "Enter the launch angle (degrees): ";
    std::cin >> launchAngle;
    std::cout << "Enter the gravitational acceleration (m/s^2, e.g., 9.81 for Earth): ";
    std::cin >> gravity;
    std::cout << "Enter the drag coefficient (dimensionless, e.g., 0.5 for a sphere): ";
    std::cin >> dragCoefficient;
    std::cout << "Enter the cross-sectional area (m^2): ";
    std::cin >> crossSectionalArea;
    std::cout << "Enter the wind speed (m/s): ";
    std::cin >> windSpeed;
    std::cout << "Enter the mass of the rocket (kg): ";
    std::cin >> mass;

    // Set parameters
    Parameters params = {mass, dragCoefficient, crossSectionalArea, gravity, windSpeed};

    // Simulate the trajectory
    double timeStep = 0.01; // Smaller time step for higher accuracy
    auto trajectory = simulateTrajectory(initialVelocity, launchAngle, params, timeStep);

    // Print the trajectory
    printTrajectory(trajectory);

    // Visualize the trajectory
    visualizeTrajectory(trajectory);

    return 0;
}

