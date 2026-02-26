#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <string>
#include <cmath>

namespace py = pybind11;


struct Range {
    public:
    Range() {
        this->min = 0.0f;
        this->max = 0.0f;
    }
    Range(float min, float max) {
        this->min = min;
        this->max = max;
    }

    float min;
    float max;

    float lerp(float t) {
        return this->min + t * (this->max - this->min);
    }

    bool between(float v) {
        return v >= this->min && v <= this->max;
    }

    std::string format(const std::string& spec) const {
        py::object py_format = py::module::import("builtins").attr("format");
        return py::str("(min={0}, max={1})").format(
            py_format(this->min, spec),
            py_format(this->max, spec)
        );
    }
    std::string to_string() {
        return this->format("");
    }
    std::string to_repr() {
        return "<Range" + this->format(".3f") + ">";
    }
};

struct Vector3 {
    public:
    Vector3() {
        this->x = 0.0f;
        this->y = 0.0f;
        this->z = 0.0f;
    }
    Vector3(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    float x;
    float y;
    float z;

    float sqrMagnitude() {
        return (this->x * this->x) + (this->y * this->y) + (this->z * this->z);
    }
    float magnitude() {
        return std::sqrt(this->sqrMagnitude());
    }

    float dot(Vector3 other) {
        return (this->x * other.x) + (this->y * other.y) + (this->z * other.z);
    }

    Vector3 normalized() {
        return *this / this->magnitude();
    }

    std::string format(const std::string& spec) const {
        py::object py_format = py::module::import("builtins").attr("format");
        return py::str("(x={0}, y={1}, z={2})").format(
            py_format(this->x, spec),
            py_format(this->y, spec),
            py_format(this->z, spec)
        );
    }
    std::string to_string() {
        return this->format("");
    }
    std::string to_repr() {
        return "<Vector3" + this->format(".3f") + ">";
    }

    friend Vector3 operator+(Vector3 lhs, const Vector3& rhs) {
        return Vector3(
            lhs.x + rhs.x,
            lhs.y + rhs.y,
            lhs.z + rhs.z
        );
    }
    friend Vector3 operator+(Vector3 lhs, const float& rhs) {
        return Vector3(
            lhs.x + rhs,
            lhs.y + rhs,
            lhs.z + rhs
        );
    }
    friend Vector3 operator+(Vector3 lhs, const int& rhs) {
        return Vector3(
            lhs.x + rhs,
            lhs.y + rhs,
            lhs.z + rhs
        );
    }
    Vector3& operator+=(const Vector3& rhs) {
        this->x += rhs.x;
        this->y += rhs.y;
        this->z += rhs.z;

        return *this;
    }
    Vector3& operator+=(const float& rhs) {
        this->x += rhs;
        this->y += rhs;
        this->z += rhs;

        return *this;
    }
    Vector3& operator+=(const int& rhs) {
        this->x += rhs;
        this->y += rhs;
        this->z += rhs;

        return *this;
    }

    friend Vector3 operator-(Vector3 lhs, const Vector3& rhs) {
        return Vector3(
            lhs.x - rhs.x,
            lhs.y - rhs.y,
            lhs.z - rhs.z
        );
    }
    friend Vector3 operator-(Vector3 lhs, const float& rhs) {
        return Vector3(
            lhs.x - rhs,
            lhs.y - rhs,
            lhs.z - rhs
        );
    }
    friend Vector3 operator-(Vector3 lhs, const int& rhs) {
        return Vector3(
            lhs.x - rhs,
            lhs.y - rhs,
            lhs.z - rhs
        );
    }
    Vector3& operator-=(const Vector3& rhs) {
        this->x -= rhs.x;
        this->y -= rhs.y;
        this->z -= rhs.z;

        return *this;
    }
    Vector3& operator-=(const float& rhs) {
        this->x -= rhs;
        this->y -= rhs;
        this->z -= rhs;

        return *this;
    }
    Vector3& operator-=(const int& rhs) {
        this->x -= rhs;
        this->y -= rhs;
        this->z -= rhs;

        return *this;
    }

    friend Vector3 operator*(Vector3 lhs, const Vector3& rhs) {
        return Vector3(
            lhs.x * rhs.x,
            lhs.y * rhs.y,
            lhs.z * rhs.z
        );
    }
    friend Vector3 operator*(Vector3 lhs, const float& rhs) {
        return Vector3(
            lhs.x * rhs,
            lhs.y * rhs,
            lhs.z * rhs
        );
    }
    friend Vector3 operator*(Vector3 lhs, const int& rhs) {
        return Vector3(
            lhs.x * rhs,
            lhs.y * rhs,
            lhs.z * rhs
        );
    }
    Vector3& operator*=(const Vector3& rhs) {
        this->x *= rhs.x;
        this->y *= rhs.y;
        this->z *= rhs.z;

        return *this;
    }
    Vector3& operator*=(const float& rhs) {
        this->x *= rhs;
        this->y *= rhs;
        this->z *= rhs;

        return *this;
    }
    Vector3& operator*=(const int& rhs) {
        this->x *= rhs;
        this->y *= rhs;
        this->z *= rhs;

        return *this;
    }

    friend Vector3 operator/(Vector3 lhs, const Vector3& rhs) {
        return Vector3(
            lhs.x / rhs.x,
            lhs.y / rhs.y,
            lhs.z / rhs.z
        );
    }
    friend Vector3 operator/(Vector3 lhs, const float& rhs) {
        return Vector3(
            lhs.x / rhs,
            lhs.y / rhs,
            lhs.z / rhs
        );
    }
    friend Vector3 operator/(Vector3 lhs, const int& rhs) {
        return Vector3(
            lhs.x / rhs,
            lhs.y / rhs,
            lhs.z / rhs
        );
    }
    Vector3& operator/=(const Vector3& rhs) {
        this->x /= rhs.x;
        this->y /= rhs.y;
        this->z /= rhs.z;

        return *this;
    }
    Vector3& operator/=(const float& rhs) {
        this->x /= rhs;
        this->y /= rhs;
        this->z /= rhs;

        return *this;
    }
    Vector3& operator/=(const int& rhs) {
        this->x /= rhs;
        this->y /= rhs;
        this->z /= rhs;

        return *this;
    }
};
struct Quaternion {
    public:
    Quaternion()
    {
        this->x = 0.0f;
        this->y = 0.0f;
        this->z = 0.0f;
        this->w = 0.0f;
    }
    Quaternion(float x, float y, float z, float w)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }

    float x;
    float y;
    float z;
    float w;

    static Quaternion identity()
    {
        return Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    }
};

struct Projectile {
    public:
    Projectile() {};
    Projectile(float objArea,
               float objMass,
               float objMomentInertia,
               float airDensity = 1.293f,
               float dragCoef = 0.47f) :
                    kObjArea(objArea),
                    kObjMass(objMass),
                    kObjMomentInertia(objMomentInertia),
                    kAirDensity(airDensity),
                    kDragCoef(dragCoef) {
        this->kCDARho = objArea * airDensity * dragCoef;
    }

    float kObjArea;
    float kObjMass;
    float kObjMomentInertia;
    float kAirDensity;
    float kDragCoef;
    float kCDARho;

    Vector3 computeAccel(Vector3 projVel, Vector3 gravity) {
        float speed = projVel.magnitude();
        return gravity - projVel * this->kCDARho * speed;
    }
};

struct BallisticSimState {
    public:
    float time;
    Vector3 position;
    Vector3 rotation;

    Vector3 linearVelocity;
    Vector3 angularVelocity;
};
class BallisticSimulator {
    public:
    BallisticSimulator(Projectile projectile,
                       BallisticSimState initialState,
                       float dt,
                       Vector3 gravity = Vector3(0.0f, 0.0f, -9.81f)) {
        this->projectile = projectile;
        this->currentState = initialState;
        this->dt = dt;
        this->gravity = gravity;
    }

    BallisticSimState doStep() {
        Vector3 forceSum = Vector3(0.0f, 0.0f, 0.0f);
        Vector3 torqueSum = Vector3(0.0f, 0.0f, 0.0f);

        float v = this->currentState.linearVelocity.magnitude();

        // Air Drag
        Vector3 aeroDragForce = this->currentState.linearVelocity / -v;
        aeroDragForce *= 0.5f * (v*v) * this->projectile.kCDARho;

        forceSum += aeroDragForce;

        // Magnus Effect

        this->currentState = eulerIntegrator(forceSum, torqueSum);
        return this->currentState;
    }

    private:
    Projectile projectile;
    BallisticSimState currentState;
    float dt;
    Vector3 gravity;

    BallisticSimState eulerIntegrator(Vector3 force, Vector3 torque) {
        BallisticSimState nextState;
        nextState.time = this->currentState.time + this->dt;

        // Linear
        Vector3 linAccel = this->gravity + (force / this->projectile.kObjMass);
        nextState.linearVelocity = this->currentState.linearVelocity + (linAccel * this->dt);
        nextState.position = this->currentState.position + (nextState.linearVelocity * this->dt);

        // Angular
        Vector3 angAccel = torque / this->projectile.kObjMomentInertia;
        nextState.angularVelocity = this->currentState.angularVelocity + (angAccel * this->dt);
        nextState.rotation = this-> currentState.rotation + (nextState.angularVelocity * this->dt);

        return nextState;
    }
    /*BallisticSimState rk4Integrator() {

    }*/
};

struct BallisticSolution {
    public:
    bool valid = false;
    float time = 0.0f;
    float speed = 0.0f;
    Vector3 aimDir;
    Vector3 impactVelocity;
    Vector3 impactDir;
    float heading = 0.0f;
    float elevation = 0.0f;
    float score = INFINITY;

    std::string format(const std::string& spec) const {
        py::object py_format = py::module::import("builtins").attr("format");
        return py::str("(valid={0}, time={1}, speed={2}, heading={3}, elevation={4})").format(
            this->valid,
            py_format(this->time, spec),
            py_format(this->speed, spec),
            py_format(this->heading, spec),
            py_format(this->elevation, spec)
        );
    }
    std::string to_string() {
        return this->format("");
    }
    std::string to_repr() {
        return "<BallisticSolution" + this->format(".3f") + ">";
    }
};
class BallisticSolver {
    public:
    BallisticSolver(Vector3 targetPos,
                    Range speedRange,
                    Range airtimeRange,
                    Vector3 impactConeAxis = Vector3(0.0f, 0.0f, -1.0f),
                    float impactConeTolerance = 0.785f,
                    Vector3 gravity = Vector3(0.0f, 0.0f, -9.81f),
                    int sampleCount = 15,
                    bool preferHighArc = true) {
        this->targetPos = targetPos;
        this->speedRange = speedRange;
        this->airtimeRange = airtimeRange;
        this->impactConeAxis = impactConeAxis;
        this->impactConeTolerance = impactConeTolerance;
        this->impactConeCosMax = std::cos(impactConeTolerance * 0.5f);
        this->gravity = gravity;
        this->halfGravity = gravity / 2.0f;
        this->sampleCount = sampleCount;
        this->preferHighArc = preferHighArc;
    }

    BallisticSolution solve(Vector3 shooterPos, Vector3 shooterWorldVel) {
        BallisticSolution best;

        Vector3 relativeTargetPos = this->targetPos - shooterPos;
        Vector3 relativeTargetVel = shooterWorldVel * -1;

        for (int i = 0; i < this->sampleCount; i++)
        {
            // Normalized centered sampling between [0, 1]
            float u = (i + 0.5f) / this->sampleCount;

            // Apply arc bias
            if (this->preferHighArc)
                u = 1.0f - (1.0f - u) * (1.0f - u);
            else
                u = u * u;

            // Convert to time
            float t = this->airtimeRange.lerp(u);
            if (t < 1e-6f)
                continue;

            // Estimate total displacement including shooter world velocity and gravity
            Vector3 totalDisplacement = relativeTargetPos + (relativeTargetVel * t) - (this->halfGravity * (t * t));
            float displacementDistance = totalDisplacement.magnitude();
            if (displacementDistance < 1e-6f)
                continue;

            // Estimate speed required to make the shot
            float requiredSpeed = displacementDistance / t;
            if (!this->speedRange.between(requiredSpeed))
                continue;

            Vector3 aimDir = totalDisplacement / displacementDistance;

            // Estimate the impact velocity and direction
            Vector3 v0 = shooterWorldVel + aimDir * requiredSpeed;
            Vector3 vImpact = v0 + this->gravity * t;

            if (vImpact.sqrMagnitude() < 1e-8f)
                continue;

            Vector3 impactDir = vImpact / vImpact.magnitude();
            if (!this->inCone(impactDir))
                continue;

            // Estimate the score of this solution and keep it if it's currently the best
            float score = requiredSpeed - 0.05f * t;
            if (!best.valid || score < best.score)
            {
                best.valid = true;
                best.time = t;
                best.speed = requiredSpeed;
                best.aimDir = aimDir;
                best.impactVelocity = vImpact;
                best.impactDir = impactDir;
                this->aimDirToHeadingElevation(aimDir, best.heading, best.elevation);
                best.score = score;
            }
        }

        return best;
    }

    private:
    Vector3 targetPos;
    Range speedRange;
    Range airtimeRange;
    Vector3 impactConeAxis;
    float impactConeTolerance;
    float impactConeCosMax;
    Vector3 gravity;
    Vector3 halfGravity;
    int sampleCount;
    bool preferHighArc;

    float dragCoef;

    bool inCone(Vector3 impactDir) {
        return impactDir.dot(this->impactConeAxis) >= this->impactConeCosMax;
    }

    void aimDirToHeadingElevation(Vector3 aimDir, float &heading, float &elevation) {
        float horizontalMag = std::hypot(aimDir.x, aimDir.y);

        heading = std::atan2(aimDir.x, -aimDir.y);
        elevation = std::atan2(aimDir.z, horizontalMag);
    }

    /*Vector3 quadraticDrag(Vector3 objVel) {
    }

    void stepRK4(Vector3 &objPos, Vector3 &objVel, float dt) {

    }*/
};

PYBIND11_MODULE(_core, m) {
    py::class_<Range>(m, "Range")
        .def(py::init<>())
        .def(py::init<float, float>())
        .def("__format__", &Range::format)
        .def("__str__", &Range::to_string)
        .def("__repr__", &Range::to_repr)
        .def_readwrite("min", &Range::min)
        .def_readwrite("max", &Range::max);

    py::class_<Vector3>(m, "Vector3")
        .def(py::init<>())
        .def(py::init<float, float, float>())
        .def("__format__", &Vector3::format)
        .def("__str__", &Vector3::to_string)
        .def("__repr__", &Vector3::to_repr)
        .def_readwrite("x", &Vector3::x)
        .def_readwrite("y", &Vector3::y)
        .def_readwrite("z", &Vector3::z)
        .def("sqr_magnitude", &Vector3::sqrMagnitude)
        .def("magnitude", &Vector3::magnitude)
        .def("dot", &Vector3::dot)
        .def(py::self + py::self)
        .def(py::self + float())
        .def(py::self + int())
        .def(py::self - py::self)
        .def(py::self - float())
        .def(py::self - int())
        .def(py::self * py::self)
        .def(py::self * float())
        .def(py::self * int())
        .def(py::self / py::self)
        .def(py::self / float())
        .def(py::self / int());

    py::class_<Projectile>(m, "Projectile")
        .def(py::init<float, float, float, float, float>(),
                py::arg("obj_area"),
                py::arg("obj_mass"),
                py::arg("obj_moment_inertia"),
                py::arg("air_density") = 1.293f,
                py::arg("drag_coef") = 0.47f)
        .def_readwrite("OBJ_AREA", &Projectile::kObjArea)
        .def_readwrite("OBJ_MASS", &Projectile::kObjMass)
        .def_readwrite("OBJ_MOMENT_INERTIA", &Projectile::kObjMomentInertia)
        .def_readwrite("AIR_DENSITY", &Projectile::kAirDensity)
        .def_readwrite("DRAG_COEF", &Projectile::kDragCoef);

    py::class_<BallisticSimState>(m, "BallisticSimState")
        .def(py::init<>())
        .def_readwrite("time", &BallisticSimState::time)
        .def_readwrite("position", &BallisticSimState::position)
        .def_readwrite("rotation", &BallisticSimState::rotation)
        .def_readwrite("linear_velocity", &BallisticSimState::linearVelocity)
        .def_readwrite("angular_velocity", &BallisticSimState::angularVelocity);

    py::class_<BallisticSimulator>(m, "BallisticSimulator")
        .def(py::init<Projectile, BallisticSimState, float, Vector3>(),
                py::arg("projectile"),
                py::arg("initial_state"),
                py::arg("dt"),
                py::arg("gravity") = Vector3(0.0f, 0.0f, -9.81f))
        .def("do_step", &BallisticSimulator::doStep);

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
        .def_readwrite("score", &BallisticSolution::score);

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
}