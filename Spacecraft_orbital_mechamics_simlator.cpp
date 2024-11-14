#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>

// Constants
const double G = 6.67430e-11;  // Universal gravitational constant (m^3/kg/s^2)
const double EARTH_MASS = 5.972e24;  // Mass of the Earth (kg)
const double EARTH_RADIUS = 6.371e6; // Radius of the Earth (m)

// Struct to store state of the spacecraft
struct State {
    double x;  // X position
    double y;  // Y position
    double vx; // X velocity
    double vy; // Y velocity
};

// Struct to store trajectory points
struct TrajectoryPoint {
    double x;
    double y;
};

// Function to calculate gravitational acceleration
double calculateGravitationalAcceleration(double x, double y) {
    double distance = std::sqrt(x * x + y * y);
    return -G * EARTH_MASS / (distance * distance);
}

// Function to calculate the acceleration components
State calculateAcceleration(const State& state) {
    double distance = std::sqrt(state.x * state.x + state.y * state.y);
    double accelerationMagnitude = calculateGravitationalAcceleration(state.x, state.y);
    double ax = accelerationMagnitude * (state.x / distance);
    double ay = accelerationMagnitude * (state.y / distance);

    return {0, 0, ax, ay};
}

// Runge-Kutta 4th Order Method to solve differential equations
State rungeKuttaStep(const State& currentState, double timeStep) {
    State k1 = calculateAcceleration(currentState);
    State k2 = calculateAcceleration({currentState.x + 0.5 * timeStep * currentState.vx,
                                      currentState.y + 0.5 * timeStep * currentState.vy,
                                      currentState.vx + 0.5 * timeStep * k1.vx,
                                      currentState.vy + 0.5 * timeStep * k1.vy});
    State k3 = calculateAcceleration({currentState.x + 0.5 * timeStep * currentState.vx,
                                      currentState.y + 0.5 * timeStep * currentState.vy,
                                      currentState.vx + 0.5 * timeStep * k2.vx,
                                      currentState.vy + 0.5 * timeStep * k2.vy});
    State k4 = calculateAcceleration({currentState.x + timeStep * currentState.vx,
                                      currentState.y + timeStep * currentState.vy,
                                      currentState.vx + timeStep * k3.vx,
                                      currentState.vy + timeStep * k3.vy});

    State nextState;
    nextState.x = currentState.x + timeStep * currentState.vx;
    nextState.y = currentState.y + timeStep * currentState.vy;
    nextState.vx = currentState.vx + (timeStep / 6.0) * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx);
    nextState.vy = currentState.vy + (timeStep / 6.0) * (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy);

    return nextState;
}

// Function to simulate the orbit
std::vector<TrajectoryPoint> simulateOrbit(double initialAltitude, double initialVelocity, double timeStep, double totalTime) {
    // Initial conditions
    State state = {EARTH_RADIUS + initialAltitude, 0.0, 0.0, initialVelocity};
    std::vector<TrajectoryPoint> trajectory;

    for (double time = 0; time <= totalTime; time += timeStep) {
        trajectory.push_back({state.x, state.y});
        state = rungeKuttaStep(state, timeStep);
    }

    return trajectory;
}

// Function to visualize the orbit using SFML
void visualizeOrbit(const std::vector<TrajectoryPoint>& trajectory) {
    sf::RenderWindow window(sf::VideoMode(800, 800), "Spacecraft Orbital Mechanics Simulator");
    window.setFramerateLimit(60);

    // Draw Earth
    sf::CircleShape earth(50); // Scale factor for Earth
    earth.setFillColor(sf::Color::Blue);
    earth.setOrigin(50, 50); // Center the shape
    earth.setPosition(400, 400); // Center of the window

    // Scale factors for visualization
    double scale = 1e-5; // Adjust this to fit the orbit on the screen

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(earth);

        // Draw the orbit
        sf::VertexArray orbit(sf::LinesStrip, trajectory.size());
        for (size_t i = 0; i < trajectory.size(); ++i) {
            orbit[i].position = sf::Vector2f(400 + trajectory[i].x * scale, 400 - trajectory[i].y * scale);
            orbit[i].color = sf::Color::White;
        }
        window.draw(orbit);

        window.display();
    }
}

int main() {
    double initialAltitude, initialVelocity, timeStep, totalTime;

    // User input
    std::cout << "Enter the initial altitude (m) above Earth's surface: ";
    std::cin >> initialAltitude;
    std::cout << "Enter the initial velocity (m/s): ";
    std::cin >> initialVelocity;
    std::cout << "Enter the time step for the simulation (s): ";
    std::cin >> timeStep;
    std::cout << "Enter the total simulation time (s): ";
    std::cin >> totalTime;

    // Simulate the orbit
    auto trajectory = simulateOrbit(initialAltitude, initialVelocity, timeStep, totalTime);

    // Visualize the orbit
    visualizeOrbit(trajectory);

    return 0;
}

