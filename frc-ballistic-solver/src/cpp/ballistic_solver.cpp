#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include <string>
#include <cmath>
#include <algorithm>
#include <vector>

namespace py = pybind11;

#include "frcmath.h"
void initMath(py::module &m);

#include "crt.h"
void initCRT(py::module &m);


// ============================================================================
// Configuration
// ============================================================================

struct BallisticConfig {
    // Simulation parameters
    float simTimestep = 0.0005f;        // 0.5ms timestep for accuracy
    float maxSimTime = 5.0f;            // Maximum simulation time (seconds)
    
    // Solver parameters
    int sampleCount = 15;               // Initial samples
    int refinementPasses = 3;           // Binary search refinement iterations
    float convergenceThreshold = 0.02f; // 2cm error tolerance
    
    // Projectile defaults (override with Projectile struct)
    float dragCoefficient = 0.47f;      // Sphere default
    float crossSectionArea = 0.01f;     // m² 
    float mass = 0.3f;                  // kg
    float airDensity = 1.225f;          // kg/m³ at sea level
    
    // Magnus effect (optional)
    bool useMagnusEffect = false;
    float magnusCoefficient = 0.1f;     // Tune experimentally
};


// ============================================================================
// Range
// ============================================================================

struct Range {
    float min;
    float max;

    Range() : min(0.0f), max(0.0f) {}
    Range(float min, float max) : min(min), max(max) {}

    float lerp(float t) const {
        return this->min + t * (this->max - this->min);
    }

    bool between(float v) const {
        return v >= this->min && v <= this->max;
    }
    
    float clamp(float v) const {
        return std::max(this->min, std::min(this->max, v));
    }
    
    float center() const {
        return (this->min + this->max) * 0.5f;
    }
    
    float width() const {
        return this->max - this->min;
    }

    std::string format(const std::string& spec) const {
        py::object py_format = py::module::import("builtins").attr("format");
        return py::str("(min={0}, max={1})").format(
            py_format(this->min, spec),
            py_format(this->max, spec)
        );
    }
    std::string to_string() { return this->format(""); }
    std::string to_repr() { return "<Range" + this->format(".3f") + ">"; }
};

/*
// ============================================================================
// Vector2
// ============================================================================

struct Vector2 {
    float x;
    float y;

    Vector2() : x(0.0f), y(0.0f) {}
    Vector2(float x, float y) : x(x), y(y) {}

    float sqrMagnitude() const {
        return (this->x * this->x) + (this->y * this->y);
    }
    float magnitude() const {
        return std::sqrt(sqrMagnitude());
    }

    float dot(const Vector2& other) const {
        return (this->x * other.x) + (this->y * other.y);
    }

    friend Vector2 operator+(Vector2 lhs, const Vector2& rhs) {
        return Vector2(lhs.x + rhs.x, lhs.y + rhs.y);
    }
    friend Vector2 operator+(Vector2 lhs, float rhs) {
        return Vector2(lhs.x + rhs, lhs.y + rhs);
    }
    friend Vector2 operator+(Vector2 lhs, int rhs) {
        return Vector2(lhs.x + rhs, lhs.y + rhs);
    }
    Vector2& operator+=(const Vector2& rhs) {
        this->x += rhs.x; this->y += rhs.y;
        return *this;
    }
    Vector2& operator+=(const float rhs) {
        this->x += rhs; this->y += rhs;
        return *this;
    }
    Vector2& operator+=(const int rhs) {
        this->x += rhs; this->y += rhs;
        return *this;
    }

    friend Vector2 operator-(Vector2 lhs, const Vector2& rhs) {
        return Vector2(lhs.x - rhs.x, lhs.y - rhs.y);
    }
    friend Vector2 operator-(Vector2 lhs, float rhs) {
        return Vector2(lhs.x - rhs, lhs.y - rhs);
    }
    friend Vector2 operator-(Vector2 lhs, int rhs) {
        return Vector2(lhs.x - rhs, lhs.y - rhs);
    }
    Vector2& operator-=(const Vector2& rhs) {
        this->x -= rhs.x; this->y -= rhs.y;
        return *this;
    }
    Vector2& operator-=(const float rhs) {
        this->x -= rhs; this->y -= rhs;
        return *this;
    }
    Vector2& operator-=(const int rhs) {
        this->x -= rhs; this->y -= rhs;
        return *this;
    }

    friend Vector2 operator*(Vector2 lhs, const Vector2& rhs) {
        return Vector2(lhs.x * rhs.x, lhs.y * rhs.y);
    }
    friend Vector2 operator*(Vector2 lhs, float rhs) {
        return Vector2(lhs.x * rhs, lhs.y * rhs);
    }
    friend Vector2 operator*(Vector2 lhs, int rhs) {
        return Vector2(lhs.x * rhs, lhs.y * rhs);
    }
    Vector2& operator*=(const Vector2& rhs) {
        this->x *= rhs.x; this->y *= rhs.y;
        return *this;
    }
    Vector2& operator*=(const float rhs) {
        this->x *= rhs; this->y *= rhs;
        return *this;
    }
    Vector2& operator*=(const int rhs) {
        this->x *= rhs; this->y *= rhs;
        return *this;
    }

    friend Vector2 operator/(Vector2 lhs, const Vector2& rhs) {
        return Vector2(lhs.x / rhs.x, lhs.y / rhs.y);
    }
    friend Vector2 operator/(Vector2 lhs, float rhs) {
        return Vector2(lhs.x / rhs, lhs.y / rhs);
    }
    friend Vector2 operator/(Vector2 lhs, int rhs) {
        return Vector2(lhs.x / rhs, lhs.y / rhs);
    }
    Vector2& operator/=(const Vector2& rhs) {
        this->x /= rhs.x; this->y /= rhs.y;
        return *this;
    }
    Vector2& operator/=(const float rhs) {
        this->x /= rhs; this->y /= rhs;
        return *this;
    }
    Vector2& operator/=(const int rhs) {
        this->x /= rhs; this->y /= rhs;
        return *this;
    }
};
*/

// ============================================================================
// Vector3
// ============================================================================

struct Vector3 {
    float x;
    float y;
    float z;

    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    float sqrMagnitude() const {
        return (this->x * this->x) + (this->y * this->y) + (this->z * this->z);
    }
    float magnitude() const {
        return std::sqrt(this->sqrMagnitude());
    }

    float dot(const Vector3& other) const {
        return (this->x * other.x) + (this->y * other.y) + (this->z * other.z);
    }
    
    Vector3 cross(const Vector3& other) const {
        return Vector3(
            this->y * other.z - this->z * other.y,
            this->z * other.x - this->x * other.z,
            this->x * other.y - this->y * other.x
        );
    }

    Vector3 normalized() const {
        float mag = this->magnitude();
        if (mag < 1e-8f) return Vector3(0, 0, 0);
        return *this / mag;
    }

    std::string format(const std::string& spec) const {
        py::object py_format = py::module::import("builtins").attr("format");
        return py::str("(x={0}, y={1}, z={2})").format(
            py_format(this->x, spec),
            py_format(this->y, spec),
            py_format(this->z, spec)
        );
    }
    std::string to_string() { return this->format(""); }
    std::string to_repr() { return "<Vector3" + this->format(".3f") + ">"; }

    // Operators
    friend Vector3 operator+(Vector3 lhs, const Vector3& rhs) {
        return Vector3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }
    friend Vector3 operator+(Vector3 lhs, float rhs) {
        return Vector3(lhs.x + rhs, lhs.y + rhs, lhs.z + rhs);
    }
    Vector3& operator+=(const Vector3& rhs) {
        this->x += rhs.x; this->y += rhs.y; this->z += rhs.z;
        return *this;
    }

    friend Vector3 operator-(Vector3 lhs, const Vector3& rhs) {
        return Vector3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }
    friend Vector3 operator-(Vector3 lhs, float rhs) {
        return Vector3(lhs.x - rhs, lhs.y - rhs, lhs.z - rhs);
    }
    Vector3 operator-() const {
        return Vector3(-this->x, -this->y, -this->z);
    }

    friend Vector3 operator*(Vector3 lhs, const Vector3& rhs) {
        return Vector3(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
    }
    friend Vector3 operator*(Vector3 lhs, float rhs) {
        return Vector3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
    }
    friend Vector3 operator*(float lhs, Vector3 rhs) {
        return Vector3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
    }
    Vector3& operator*=(float rhs) {
        this->x *= rhs; this->y *= rhs; this->z *= rhs;
        return *this;
    }

    friend Vector3 operator/(Vector3 lhs, const Vector3& rhs) {
        return Vector3(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z);
    }
    friend Vector3 operator/(Vector3 lhs, float rhs) {
        return Vector3(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
    }
    Vector3& operator/=(float rhs) {
        this->x /= rhs; this->y /= rhs; this->z /= rhs;
        return *this;
    }
};


// ============================================================================
// Quaternion (for future rotation handling)
// ============================================================================

struct Quaternion {
    float x, y, z, w;

    Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
    Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

    static Quaternion identity() {
        return Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    }
};


// ============================================================================
// Projectile
// ============================================================================

struct Projectile {
    float kObjArea;           // Cross-sectional area (m²)
    float kObjMass;           // Mass (kg)
    float kObjMomentInertia;  // Moment of inertia (kg·m²)
    float kAirDensity;        // Air density (kg/m³)
    float kDragCoef;          // Drag coefficient
    float kCDARho;            // Precomputed: Area * DragCoef * AirDensity
    float kMagnusCoef;        // Magnus effect coefficient

    Projectile() : kObjArea(0.01f), kObjMass(0.3f), kObjMomentInertia(0.001f),
                   kAirDensity(1.225f), kDragCoef(0.47f), kMagnusCoef(0.0f) {
        this->kCDARho = kObjArea * kDragCoef * kAirDensity;
    }
    
    Projectile(float objArea, float objMass, float objMomentInertia,
               float airDensity = 1.225f, float dragCoef = 0.47f, float magnusCoef = 0.0f)
        : kObjArea(objArea), kObjMass(objMass), kObjMomentInertia(objMomentInertia),
          kAirDensity(airDensity), kDragCoef(dragCoef), kMagnusCoef(magnusCoef) {
        this->kCDARho = objArea * dragCoef * airDensity;
    }

    Vector3 computeDragForce(Vector3 velocity) const {
        float speed = velocity.magnitude();
        if (speed < 1e-6f) return Vector3();
        
        // F_drag = -0.5 * rho * Cd * A * v² * v_hat
        // Precomputed: kCDARho = Cd * A * rho
        // So: F_drag = -0.5 * kCDARho * speed * velocity

        Vector3 aeroDragForce = velocity / -speed;
        aeroDragForce *= 0.5f * (speed*speed) * this->kCDARho;

        return aeroDragForce;
    }
    
    Vector3 computeMagnusForce(Vector3 velocity, Vector3 angularVelocity) const {
        if (this->kMagnusCoef < 1e-6f) return Vector3();
        
        // F_magnus = kMagnus * (omega x v)
        return angularVelocity.cross(velocity) * this->kMagnusCoef;
    }
    
    Vector3 computeAccel(Vector3 velocity, Vector3 angularVelocity, Vector3 gravity) const {
        Vector3 drag = computeDragForce(velocity);
        Vector3 magnus = computeMagnusForce(velocity, angularVelocity);
        return gravity + (drag + magnus) / this->kObjMass;
    }
};


// ============================================================================
// BallisticSimState
// ============================================================================

struct BallisticSimState {
    float time;
    Vector3 position;
    Vector3 rotation;
    Vector3 linearVelocity;
    Vector3 angularVelocity;

    BallisticSimState() : time(0.0f) {}
};


// ============================================================================
// BallisticSimulator - Forward simulation with RK4
// ============================================================================

class BallisticSimulator {
public:
    BallisticSimulator(const Projectile& projectile,
                       const BallisticSimState& initialState,
                       float dt,
                       const Vector3& gravity = Vector3(0.0f, 0.0f, -9.81f))
        : m_projectile(projectile),
          m_currentState(initialState),
          m_dt(dt),
          m_gravity(gravity) {}

    BallisticSimState doStep() {
        // Use RK4 for better accuracy
        m_currentState = rk4Step(m_currentState);
        return m_currentState;
    }
    
    BallisticSimState doStepEuler() {
        // Simple Euler (less accurate but faster)
        Vector3 forceSum = m_projectile.computeDragForce(m_currentState.linearVelocity);
        Vector3 magnusForce = m_projectile.computeMagnusForce(
            m_currentState.linearVelocity, m_currentState.angularVelocity);
        forceSum = forceSum + magnusForce;

        BallisticSimState newState;
        newState.time = m_currentState.time + m_dt;
        
        // Linear motion
        Vector3 linAccel = m_gravity + (forceSum / m_projectile.kObjMass);
        newState.linearVelocity = m_currentState.linearVelocity + (linAccel * m_dt);
        newState.position = m_currentState.position + (newState.linearVelocity * m_dt);
        
        // Angular motion (simplified - no angular drag)
        if (m_projectile.kObjMomentInertia > 1e-8f) {
            // Could add angular drag here if needed
        }

        m_currentState = newState;
        return newState;
    }

    const BallisticSimState& getState() const { return m_currentState; }
    
    void setState(const BallisticSimState& state) { m_currentState = state; }

private:
    Projectile m_projectile;
    BallisticSimState m_currentState;
    float m_dt;
    Vector3 m_gravity;
    
    // RK4 integration for better accuracy with drag
    BallisticSimState rk4Step(const BallisticSimState& state) {
        auto derive = [this](const BallisticSimState& s) -> BallisticSimState {
            BallisticSimState deriv;
            deriv.time = 1.0f;
            deriv.position = s.linearVelocity;
            
            Vector3 force = m_projectile.computeDragForce(s.linearVelocity);
            force = force + m_projectile.computeMagnusForce(s.linearVelocity, s.angularVelocity);
            
            deriv.linearVelocity = m_gravity + (force / m_projectile.kObjMass);
            deriv.angularVelocity = Vector3(); // No angular acceleration for now
            deriv.rotation = s.angularVelocity;
            
            return deriv;
        };
        
        float dt = m_dt;
        
        BallisticSimState k1 = derive(state);
        
        BallisticSimState s2;
        s2.time = state.time + dt * 0.5f;
        s2.position = state.position + k1.position * (dt * 0.5f);
        s2.linearVelocity = state.linearVelocity + k1.linearVelocity * (dt * 0.5f);
        s2.angularVelocity = state.angularVelocity + k1.angularVelocity * (dt * 0.5f);
        s2.rotation = state.rotation + k1.rotation * (dt * 0.5f);
        BallisticSimState k2 = derive(s2);
        
        BallisticSimState s3;
        s3.time = state.time + dt * 0.5f;
        s3.position = state.position + k2.position * (dt * 0.5f);
        s3.linearVelocity = state.linearVelocity + k2.linearVelocity * (dt * 0.5f);
        s3.angularVelocity = state.angularVelocity + k2.angularVelocity * (dt * 0.5f);
        s3.rotation = state.rotation + k2.rotation * (dt * 0.5f);
        BallisticSimState k3 = derive(s3);
        
        BallisticSimState s4;
        s4.time = state.time + dt;
        s4.position = state.position + k3.position * dt;
        s4.linearVelocity = state.linearVelocity + k3.linearVelocity * dt;
        s4.angularVelocity = state.angularVelocity + k3.angularVelocity * dt;
        s4.rotation = state.rotation + k3.rotation * dt;
        BallisticSimState k4 = derive(s4);
        
        BallisticSimState result;
        result.time = state.time + dt;
        result.position = state.position + (k1.position + k2.position * 2.0f + k3.position * 2.0f + k4.position) * (dt / 6.0f);
        result.linearVelocity = state.linearVelocity + (k1.linearVelocity + k2.linearVelocity * 2.0f + k3.linearVelocity * 2.0f + k4.linearVelocity) * (dt / 6.0f);
        result.angularVelocity = state.angularVelocity + (k1.angularVelocity + k2.angularVelocity * 2.0f + k3.angularVelocity * 2.0f + k4.angularVelocity) * (dt / 6.0f);
        result.rotation = state.rotation + (k1.rotation + k2.rotation * 2.0f + k3.rotation * 2.0f + k4.rotation) * (dt / 6.0f);
        
        return result;
    }
};


// ============================================================================
// BallisticSolution
// ============================================================================

struct BallisticSolution {
    bool valid = false;
    float time = 0.0f;
    float speed = 0.0f;
    float angSpeed = 0.0f;
    Vector3 aimDir;
    Vector3 impactVelocity;
    Vector3 impactDir;
    float heading = 0.0f;
    float elevation = 0.0f;
    float score = INFINITY;
    float error = INFINITY;  // New: error in meters

    float bottomWheelVelocity = 0.0f;
    float topWheelVelocity = 0.0f;

    Vector3 getAngularVelocity() const {
        Vector3 projectedAimDir = Vector3(aimDir.x, aimDir.y, 0.0f);
        Vector3 angularVelocity = projectedAimDir.cross(aimDir);
        angularVelocity /= angularVelocity.magnitude();
        angularVelocity *= angSpeed;

        return angularVelocity;
    }

    std::string format(const std::string& spec) const {
        py::object py_format = py::module::import("builtins").attr("format");
        return py::str("(valid={0}, time={1}, speed={2}, heading={3}, elevation={4}, error={5})").format(
            this->valid,
            py_format(this->time, spec),
            py_format(this->speed, spec),
            py_format(this->heading, spec),
            py_format(this->elevation, spec),
            py_format(this->error, spec)
        );
    }
    std::string to_string() { return this->format(""); }
    std::string to_repr() { return "<BallisticSolution" + this->format(".3f") + ">"; }
};


// ============================================================================
// HybridBallisticSolver
// ============================================================================

class HybridBallisticSolver {
public:
    HybridBallisticSolver(const Vector3& targetPos,
                          const Range& speedRange,
                          const Range& airtimeRange,
                          const Projectile& projectile = Projectile(),
                          const Vector3& impactConeAxis = Vector3(0.0f, 0.0f, -1.0f),
                          float impactConeTolerance = 0.785f,
                          const Vector3& gravity = Vector3(0.0f, 0.0f, -9.81f),
                          int sampleCount = 15,
                          bool preferHighArc = true,
                          int refinementPasses = 3,
                          float convergenceThreshold = 0.02f,
                          float dt = 0.0005f,
                          float magnusRatio = 1.0f,
                          float ballRadius = 0.072f,
                          float bottomWheelRadius = 0.0048f,
                          float topWheelRadius = 1.0f)
        : m_targetPos(targetPos),
          m_speedRange(speedRange),
          m_airtimeRange(airtimeRange),
          m_projectile(projectile),
          m_impactConeAxis(impactConeAxis.normalized()),
          m_impactConeTolerance(impactConeTolerance),
          m_impactConeCosMax(std::cos(impactConeTolerance * 0.5f)),
          m_gravity(gravity),
          m_halfGravity(gravity * 0.5f),
          m_sampleCount(sampleCount),
          m_preferHighArc(preferHighArc),
          m_refinementPasses(refinementPasses),
          m_convergenceThreshold(convergenceThreshold),
          m_dt(dt),
          m_magnusRatio(magnusRatio / ballRadius),
          m_ballRadius(ballRadius),
          m_bottomWheelRadius(bottomWheelRadius),
          m_topWheelRadius(topWheelRadius) {}

    BallisticSolution solve(const Vector3& shooterPos, const Vector3& shooterWorldVel) {
        // Phase 1: Kinematic estimate (fast, no drag)
        BallisticSolution estimate = kinematicSolve(shooterPos, shooterWorldVel);
        
        if (!estimate.valid) {
            return estimate;
        }
        
        // Phase 2: Validate with drag simulation
        float error = validateWithSimulation(estimate, shooterPos, shooterWorldVel);
        estimate.error = error;
        
        if (error < m_convergenceThreshold) {
            return estimate;
        }
        
        // Phase 3: Refine with binary search if error is too high
        BallisticSolution refined = refineSolution(estimate, shooterPos, shooterWorldVel);
        
        return refined;
    }
    
    // Setters for runtime configuration
    void setTargetPos(const Vector3& pos) { m_targetPos = pos; }
    void setSpeedRange(const Range& range) { m_speedRange = range; }
    void setAirtimeRange(const Range& range) { m_airtimeRange = range; }
    void setProjectile(const Projectile& proj) { m_projectile = proj; }

    // Expose kinematic solve for compatibility
    BallisticSolution kinematicSolve(const Vector3& shooterPos, const Vector3& shooterWorldVel) {
        BallisticSolution best;

        Vector3 relativeTargetPos = m_targetPos - shooterPos;
        Vector3 relativeTargetVel = shooterWorldVel * -1.0f;

        for (int i = 0; i < m_sampleCount; i++) {
            float u = (i + 0.5f) / static_cast<float>(m_sampleCount);

            // Arc bias
            if (m_preferHighArc) {
                u = 1.0f - (1.0f - u) * (1.0f - u);
            } else {
                u = u * u;
            }

            float t = m_airtimeRange.lerp(u);
            if (t < 1e-6f) continue;

            // Kinematic displacement (no drag)
            Vector3 totalDisplacement = relativeTargetPos + (relativeTargetVel * t) - (m_halfGravity * (t * t));
            float displacementDistance = totalDisplacement.magnitude();
            if (displacementDistance < 1e-6f) continue;

            float requiredSpeed = displacementDistance / t;
            if (!m_speedRange.between(requiredSpeed)) continue;

            Vector3 aimDir = totalDisplacement * (1.0f / displacementDistance);

            // Impact velocity estimate
            Vector3 v0 = shooterWorldVel + aimDir * requiredSpeed;
            Vector3 vImpact = v0 + m_gravity * t;

            if (vImpact.sqrMagnitude() < 1e-8f) continue;
            if (!inCone(vImpact.normalized())) continue;

            // Score: prefer lower speed and shorter time
            float score = requiredSpeed + 0.1f * t;
            if (!best.valid || score < best.score) {
                best.valid = true;
                best.time = t;
                best.speed = requiredSpeed;
                best.aimDir = aimDir;
                best.impactVelocity = vImpact;
                best.impactDir = vImpact.normalized();
                aimDirToHeadingElevation(aimDir, best.heading, best.elevation);
                best.score = score;
            }
        }

        return best;
    }

private:
    Vector3 m_targetPos;
    Range m_speedRange;
    Range m_airtimeRange;
    Projectile m_projectile;
    Vector3 m_impactConeAxis;
    float m_impactConeTolerance;
    float m_impactConeCosMax;
    Vector3 m_gravity;
    Vector3 m_halfGravity;
    int m_sampleCount;
    bool m_preferHighArc;
    int m_refinementPasses;
    float m_convergenceThreshold;
    float m_dt;
    float m_magnusRatio;
    float m_ballRadius;
    float m_bottomWheelRadius;
    float m_topWheelRadius;

    // Phase 2: Validate with full drag simulation
    float validateWithSimulation(BallisticSolution& sol, 
                                 const Vector3& shooterPos,
                                 const Vector3& shooterWorldVel) {
        BallisticSimState state;
        state.position = shooterPos;
        state.linearVelocity = shooterWorldVel + sol.aimDir * sol.speed;
        state.angularVelocity = Vector3();  // No spin for now
        state.time = 0.0f;

        BallisticSimulator sim(m_projectile, state, m_dt, m_gravity);

        // Simulate until we reach the target height or time
        while (state.time < sol.time && state.position.z > m_targetPos.z - 0.5f) {
            state = sim.doStep();
        }

        sol.impactVelocity = state.linearVelocity;
        sol.impactDir = state.linearVelocity.normalized();

        return (m_targetPos - state.position).magnitude();
    }

    // Phase 3: Binary search refinement
    BallisticSolution refineSolution(const BallisticSolution& initial,
                                     const Vector3& shooterPos,
                                     const Vector3& shooterWorldVel) {
        Range speedSearch(m_speedRange.clamp(initial.speed * 0.85f),
                          m_speedRange.clamp(initial.speed * 1.15f));
        Range timeSearch(m_airtimeRange.clamp(initial.time * 0.85f),
                         m_airtimeRange.clamp(initial.time * 1.15f));

        BallisticSolution best = initial;
        best.error = simulateAndMeasureError(best, shooterPos, shooterWorldVel);

        if (best.error < m_convergenceThreshold)
            return best;

        for (int pass = 0; pass < m_refinementPasses; pass++) {
            BallisticSolution passBest = best;
            float searchDensity = 5.0f;

            for (int si = 0; si < static_cast<int>(searchDensity); si++) {
                float speedT = (si + 0.5f) / searchDensity;
                float speed = speedSearch.lerp(speedT);

                for (int ti = 0; ti < static_cast<int>(searchDensity); ti++) {
                    float timeT = (ti + 0.5f) / searchDensity;
                    float time = timeSearch.lerp(timeT);

                    BallisticSolution candidate = evaluateSolution(
                        speed, time, shooterPos, shooterWorldVel);
                    
                    if (candidate.valid && candidate.error < passBest.error) {
                        passBest = candidate;

                        if (passBest.error < m_convergenceThreshold)
                            return passBest;
                    }
                }
            }

            if (passBest.error < best.error) {
                best = passBest;

                if (best.error < m_convergenceThreshold)
                    return best;
            }

            // Narrow search around best
            float speedHalfWidth = speedSearch.width() * 0.25f;
            float timeHalfWidth = timeSearch.width() * 0.25f;

            speedSearch = Range(
                m_speedRange.clamp(best.speed - speedHalfWidth),
                m_speedRange.clamp(best.speed + speedHalfWidth)
            );
            timeSearch = Range(
                m_airtimeRange.clamp(best.time - timeHalfWidth),
                m_airtimeRange.clamp(best.time + timeHalfWidth)
            );
        }

        return best;
    }

    BallisticSolution evaluateSolution(float speed, float time,
                                       const Vector3& shooterPos,
                                       const Vector3& shooterWorldVel) {
        BallisticSolution sol;
        sol.time = time;
        sol.speed = speed;
        sol.angSpeed = m_magnusRatio * speed;

        sol.bottomWheelVelocity = (speed * (1.0f + m_magnusRatio * m_ballRadius)) / m_bottomWheelRadius;
        sol.topWheelVelocity = (speed * (1.0f - m_magnusRatio * m_ballRadius)) / m_bottomWheelRadius;

        // Compute aim direction from kinematic equation
        Vector3 relativeTargetPos = m_targetPos - shooterPos;
        Vector3 relativeTargetVel = shooterWorldVel * -1.0f;
        Vector3 totalDisplacement = relativeTargetPos + (relativeTargetVel * time) - (m_halfGravity * (time * time));
        
        float displacementDistance = totalDisplacement.magnitude();
        if (displacementDistance < 1e-6f) {
            return sol;
        }

        sol.aimDir = totalDisplacement * (1.0f / displacementDistance);

        // Check impact cone
        Vector3 vImpact = shooterWorldVel + sol.aimDir * speed + m_gravity * time;
        if (vImpact.sqrMagnitude() < 1e-8f || !inCone(vImpact.normalized())) {
            return sol;
        }

        sol.valid = true;
        sol.impactVelocity = vImpact;
        sol.impactDir = vImpact.normalized();
        aimDirToHeadingElevation(sol.aimDir, sol.heading, sol.elevation);
        sol.error = simulateAndMeasureError(sol, shooterPos, shooterWorldVel);
        sol.score = speed + 0.1f * time;

        return sol;
    }

    float simulateAndMeasureError(const BallisticSolution& sol,
                                  const Vector3& shooterPos,
                                  const Vector3& shooterWorldVel) {
        BallisticSimState state;
        state.position = shooterPos;
        state.linearVelocity = shooterWorldVel + sol.aimDir * sol.speed;
        state.angularVelocity = sol.getAngularVelocity();
        state.time = 0.0f;

        BallisticSimulator sim(m_projectile, state, m_dt, m_gravity);

        float best = INFINITY;
        // Simulate until we pass the target height or exceed time
        while (state.time < sol.time * 1.1f) {
            state = sim.doStepEuler();

            float distanceToTarget = (m_targetPos - state.position).magnitude();
            if (distanceToTarget < best)
                best = distanceToTarget;
        }

        return best;
    }

    bool inCone(const Vector3& impactDir) const {
        return impactDir.dot(m_impactConeAxis) >= m_impactConeCosMax;
    }

    void aimDirToHeadingElevation(const Vector3& aimDir, float& heading, float& elevation) const {
        float horizontalMag = std::hypot(aimDir.x, aimDir.y);
        heading = std::atan2(aimDir.x, -aimDir.y);
        elevation = std::atan2(aimDir.z, horizontalMag);
    }
};


// ============================================================================
// Legacy BallisticSolver (backward compatible)
// ============================================================================

class BallisticSolver {
public:
    BallisticSolver(const Vector3& targetPos,
                    const Range& speedRange,
                    const Range& airtimeRange,
                    const Vector3& impactConeAxis = Vector3(0.0f, 0.0f, -1.0f),
                    float impactConeTolerance = 0.785f,
                    const Vector3& gravity = Vector3(0.0f, 0.0f, -9.81f),
                    int sampleCount = 15,
                    bool preferHighArc = true)
        : m_hybridSolver(targetPos, speedRange, airtimeRange,
                         Projectile(), impactConeAxis, impactConeTolerance,
                         gravity, sampleCount, preferHighArc,
                         0, 1.0f) {}  // No refinement for backward compat

    BallisticSolution solve(const Vector3& shooterPos, const Vector3& shooterWorldVel) {
        return m_hybridSolver.kinematicSolve(shooterPos, shooterWorldVel);
    }

private:
    HybridBallisticSolver m_hybridSolver;
};


// ============================================================================
// FieldZone
// ============================================================================

struct FieldZone
{
    int kId;

    FieldZone() : kId(-1) {}
    FieldZone(int id, std::vector<FRC::Vector2> vertices)
    {
        kId = id;
        m_vertices = vertices;
    }

    bool pointInZone(const FRC::Vector2& p)
    {
        int crossings = 0;
        const size_t n = m_vertices.size();

        for (size_t i = 0; i < n; ++i)
        {
            const FRC::Vector2& a = m_vertices[i];
            const FRC::Vector2& b = m_vertices[(i + 1) % n];

            // Check if point is exactly on a vertex or lies on an edge.
            if ((p.x == a.x && p.y == a.y) || (p.x == b.x && p.y == b.y))
                return true;

            if (a.y == b.y && p.y == a.y && ((p.x >= std::min(a.x, b.x)) && (p.x <= std::max(a.x, b.x))))
                return true; // point lies on a horizontal edge

            if ((p.y > std::min(a.y, b.y)) && (p.y <= std::max(a.y, b.y)) && (a.y != b.y))
            {
                double x_intersect = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if (x_intersect == p.x)   // point lies on boundary
                    return true;

                if (x_intersect > p.x)
                    ++crossings;
            }
        }

        return (crossings % 2) == 1;
    }

    private:
    std::vector<FRC::Vector2> m_vertices;
};


struct Field {
    Field() {}
    Field(std::vector<FieldZone> zones)
    {
        m_zones = zones;
    }

    int pointZone(const FRC::Vector2& p)
    {
        const size_t n = m_zones.size();

        for (size_t i = 0; i < n; ++i)
        {
            FieldZone& zone = m_zones[i];
            if (zone.pointInZone(p))
                return zone.kId;
        }

        return -1;
    }

    private:
    std::vector<FieldZone> m_zones;
};


// ============================================================================
// Python Module Bindings
// ============================================================================


//PYBIND11_MAKE_OPAQUE(std::vector<Vector2>);


PYBIND11_MODULE(_core, m) {
    m.doc() = "FRC Ballistic Solver - Hybrid drag-aware projectile solver";

    initCRT(m);
    initMath(m);

    // BallisticConfig
    py::class_<BallisticConfig>(m, "BallisticConfig")
        .def(py::init<>())
        .def_readwrite("sim_timestep", &BallisticConfig::simTimestep)
        .def_readwrite("max_sim_time", &BallisticConfig::maxSimTime)
        .def_readwrite("sample_count", &BallisticConfig::sampleCount)
        .def_readwrite("refinement_passes", &BallisticConfig::refinementPasses)
        .def_readwrite("convergence_threshold", &BallisticConfig::convergenceThreshold)
        .def_readwrite("drag_coefficient", &BallisticConfig::dragCoefficient)
        .def_readwrite("cross_section_area", &BallisticConfig::crossSectionArea)
        .def_readwrite("mass", &BallisticConfig::mass)
        .def_readwrite("air_density", &BallisticConfig::airDensity)
        .def_readwrite("use_magnus_effect", &BallisticConfig::useMagnusEffect)
        .def_readwrite("magnus_coefficient", &BallisticConfig::magnusCoefficient);

    // Range
    py::class_<Range>(m, "Range")
        .def(py::init<>())
        .def(py::init<float, float>(), py::arg("min"), py::arg("max"))
        .def("__format__", &Range::format)
        .def("__str__", &Range::to_string)
        .def("__repr__", &Range::to_repr)
        .def("lerp", &Range::lerp)
        .def("between", &Range::between)
        .def("clamp", &Range::clamp)
        .def("center", &Range::center)
        .def("width", &Range::width)
        .def_readwrite("min", &Range::min)
        .def_readwrite("max", &Range::max);

    // Vector2
    /*py::class_<Vector2>(m, "Vector2")
        .def(py::init<>())
        .def(py::init<float, float>(), py::arg("x"), py::arg("y"))
        .def_readwrite("x", &Vector2::x)
        .def_readwrite("y", &Vector2::y);*/

    // Vector3
    py::class_<Vector3>(m, "Vector3")
        .def(py::init<>())
        .def(py::init<float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def("__format__", &Vector3::format)
        .def("__str__", &Vector3::to_string)
        .def("__repr__", &Vector3::to_repr)
        .def("sqr_magnitude", &Vector3::sqrMagnitude)
        .def("magnitude", &Vector3::magnitude)
        .def("dot", &Vector3::dot)
        .def("cross", &Vector3::cross)
        .def("normalized", &Vector3::normalized)
        .def_readwrite("x", &Vector3::x)
        .def_readwrite("y", &Vector3::y)
        .def_readwrite("z", &Vector3::z)
        .def(py::self + py::self)
        .def(py::self + float())
        .def(py::self - py::self)
        .def(py::self - float())
        .def(-py::self)
        .def(py::self * py::self)
        .def(py::self * float())
        .def(float() * py::self)
        .def(py::self / py::self)
        .def(py::self / float());

    // Quaternion
    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<>())
        .def(py::init<float, float, float, float>())
        .def_static("identity", &Quaternion::identity)
        .def_readwrite("x", &Quaternion::x)
        .def_readwrite("y", &Quaternion::y)
        .def_readwrite("z", &Quaternion::z)
        .def_readwrite("w", &Quaternion::w);

    // Projectile
    py::class_<Projectile>(m, "Projectile")
        .def(py::init<float, float, float, float, float, float>(),
             py::arg("obj_area"),
             py::arg("obj_mass"),
             py::arg("obj_moment_inertia"),
             py::arg("air_density") = 1.225f,
             py::arg("drag_coef") = 0.47f,
             py::arg("magnus_coef") = 0.0f)
        .def_readwrite("OBJ_AREA", &Projectile::kObjArea)
        .def_readwrite("OBJ_MASS", &Projectile::kObjMass)
        .def_readwrite("OBJ_MOMENT_INERTIA", &Projectile::kObjMomentInertia)
        .def_readwrite("AIR_DENSITY", &Projectile::kAirDensity)
        .def_readwrite("DRAG_COEF", &Projectile::kDragCoef)
        .def_readwrite("MAGNUS_COEF", &Projectile::kMagnusCoef)
        .def("compute_drag_force", &Projectile::computeDragForce)
        .def("compute_magnus_force", &Projectile::computeMagnusForce);

    // BallisticSimState
    py::class_<BallisticSimState>(m, "BallisticSimState")
        .def(py::init<>())
        .def_readwrite("time", &BallisticSimState::time)
        .def_readwrite("position", &BallisticSimState::position)
        .def_readwrite("rotation", &BallisticSimState::rotation)
        .def_readwrite("linear_velocity", &BallisticSimState::linearVelocity)
        .def_readwrite("angular_velocity", &BallisticSimState::angularVelocity);

    // BallisticSimulator
    py::class_<BallisticSimulator>(m, "BallisticSimulator")
        .def(py::init<Projectile, BallisticSimState, float, Vector3>(),
             py::arg("projectile"),
             py::arg("initial_state"),
             py::arg("dt"),
             py::arg("gravity") = Vector3(0.0f, 0.0f, -9.81f))
        .def("do_step", &BallisticSimulator::doStep)
        .def("do_step_euler", &BallisticSimulator::doStepEuler)
        .def("get_state", &BallisticSimulator::getState, py::return_value_policy::reference)
        .def("set_state", &BallisticSimulator::setState);

    // BallisticSolution
    py::class_<BallisticSolution>(m, "BallisticSolution")
        .def(py::init<>())
        .def("__format__", &BallisticSolution::format)
        .def("__str__", &BallisticSolution::to_string)
        .def("__repr__", &BallisticSolution::to_repr)
        .def_readwrite("valid", &BallisticSolution::valid)
        .def_readwrite("time", &BallisticSolution::time)
        .def_readwrite("speed", &BallisticSolution::speed)
        .def_readwrite("aim_dir", &BallisticSolution::aimDir)
        .def_readwrite("impact_vel", &BallisticSolution::impactVelocity)
        .def_readwrite("impact_dir", &BallisticSolution::impactDir)
        .def_readwrite("heading", &BallisticSolution::heading)
        .def_readwrite("elevation", &BallisticSolution::elevation)
        .def_readwrite("score", &BallisticSolution::score)
        .def_readwrite("error", &BallisticSolution::error)
        .def_readwrite("bottom_wheel_vel", &BallisticSolution::bottomWheelVelocity)
        .def_readwrite("top_wheel_vel", &BallisticSolution::topWheelVelocity)
        .def("get_angular_velocity", &BallisticSolution::getAngularVelocity);

    // BallisticSolver (legacy)
    py::class_<BallisticSolver>(m, "BallisticSolver")
        .def(py::init<Vector3, Range, Range, Vector3, float, Vector3, int, bool>(),
             py::arg("target_pos"),
             py::arg("speed_range"),
             py::arg("airtime_range"),
             py::arg("impact_cone_axis") = Vector3(0.0f, 0.0f, -1.0f),
             py::arg("impact_cone_tolerance") = 0.785f,
             py::arg("gravity") = Vector3(0.0f, 0.0f, -9.81f),
             py::arg("sample_count") = 15,
             py::arg("prefer_high_arc") = true)
        .def("solve", &BallisticSolver::solve);

    // HybridBallisticSolver (new)
    py::class_<HybridBallisticSolver>(m, "HybridBallisticSolver")
        .def(py::init<Vector3, Range, Range, Projectile, Vector3, float, Vector3, int, bool, int, float, float, float, float, float, float>(),
             py::arg("target_pos"),
             py::arg("speed_range"),
             py::arg("airtime_range"),
             py::arg("projectile") = Projectile(),
             py::arg("impact_cone_axis") = Vector3(0.0f, 0.0f, -1.0f),
             py::arg("impact_cone_tolerance") = 0.785f,
             py::arg("gravity") = Vector3(0.0f, 0.0f, -9.81f),
             py::arg("sample_count") = 15,
             py::arg("prefer_high_arc") = true,
             py::arg("refinement_passes") = 3,
             py::arg("convergence_threshold") = 0.02f,
             py::arg("dt") = 0.01f,
             py::arg("magnus_ratio") = 1.0f,
             py::arg("ball_radius") = 0.072f,
             py::arg("bottom_wheel_radius") = 0.0048f,
             py::arg("top_wheel_radius") = 1.0f)
        .def("solve", &HybridBallisticSolver::solve)
        .def("set_target_pos", &HybridBallisticSolver::setTargetPos)
        .def("set_speed_range", &HybridBallisticSolver::setSpeedRange)
        .def("set_airtime_range", &HybridBallisticSolver::setAirtimeRange)
        .def("set_projectile", &HybridBallisticSolver::setProjectile);

    // FieldZone
    py::class_<FieldZone>(m, "FieldZone")
        .def(py::init<int, std::vector<FRC::Vector2>>(), py::arg("id"), py::arg("vertices"))
        .def_readwrite("ID", &FieldZone::kId)
        .def("is_point_in_zone", &FieldZone::pointInZone);

    // Field
    py::class_<Field>(m, "Field")
        .def(py::init<std::vector<FieldZone>>(), py::arg("zones"))
        .def("point_zone", &Field::pointZone);
}