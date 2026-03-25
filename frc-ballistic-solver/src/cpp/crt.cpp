#include "crt.h"

#include <memory>
#include <numeric>
#include <cmath>
#include <algorithm>

#include <pybind11/pybind11.h>
namespace py = pybind11;

int gcd(int a, int b) {
    while (b) {
        a %= b;
        std::swap(a, b);
    }
    return a;
}

/*-----------------------\
|       CRTEncoder       |
\-----------------------*/

FRC::CRTEncoder::CRTEncoder() {}
FRC::CRTEncoder::CRTEncoder(const int& gearTeeth, const int& turretTeeth, const float& unit) {
    m_gearTeeth = gearTeeth;
    m_turretTeeth = turretTeeth;
    m_unit = unit;

    m_gearRatio = gearTeeth / (float)turretTeeth;
    m_inverseGearRatio = turretTeeth / (float)gearTeeth;
    m_encoderAngle = 0.0f;
}

int FRC::CRTEncoder::getGearTeeth() {
    return m_gearTeeth;
}
int FRC::CRTEncoder::getTurretTeeth() {
    return m_turretTeeth;
}

float FRC::CRTEncoder::getAngle01() {
    return m_encoderAngle;
}
void FRC::CRTEncoder::setAngle01(const float& angle) {
    m_encoderAngle = std::fmod(angle, 1.0f);
}

float FRC::CRTEncoder::getAngle() {
    return m_encoderAngle * m_unit;
}
void FRC::CRTEncoder::setAngle(const float& angle) {
    setAngle01(angle / m_unit);
}

float FRC::CRTEncoder::estimateEncoderTurn(const float& turretTurn) {
    return m_inverseGearRatio * turretTurn;
}

float FRC::CRTEncoder::estimate(const int& n) {
    return (n + m_encoderAngle) * m_gearRatio;
}

/*----------------------\
|          CRT          |
\----------------------*/

FRC::CRT::CRT(const std::shared_ptr<FRC::CRTEncoder>& encoderA, const std::shared_ptr<FRC::CRTEncoder>& encoderB, const int& turretTeeth) :
        m_encoderA(encoderA), m_encoderB(encoderB) {
    m_turretTeeth = turretTeeth;
}

float FRC::CRT::maxRotation() {
    return (m_encoderA->getGearTeeth() * m_encoderB->getGearTeeth()) / (float)m_turretTeeth;
}

bool FRC::CRT::isCoprime() {
    return gcd(m_encoderA->getGearTeeth(), m_encoderB->getGearTeeth()) == 1;
}
bool FRC::CRT::canDoRotation(const float& rotationCount) {
    return maxRotation() >= rotationCount;
}
bool FRC::CRT::isValid(const float& rotationCount) {
    return isCoprime() && canDoRotation(rotationCount);
}

float FRC::CRT::getTurn(const float& weightA,
                        const float& weightB,
                        const float& maxTurn,
                        const float& previousTurn,
                        const float& proximityWeight) {
    int nMaxA = m_encoderB->getGearTeeth(); //std::min((int)std::ceil(m_encoderA->estimateEncoderTurn(maxTurn)), m_encoderB->getGearTeeth());
    int nMaxB = m_encoderA->getGearTeeth(); //std::min((int)std::ceil(m_encoderB->estimateEncoderTurn(maxTurn)), m_encoderA->getGearTeeth());

    float totWeight = weightA + weightB;
    if (totWeight == 0.0f)
        totWeight = 1.0f;

    const bool usePrevious = std::isfinite(previousTurn) && proximityWeight > 0.0f;

    const float wrapPeriod = std::isfinite(maxTurn) && maxTurn > 0.0f ? maxTurn : maxRotation();
    auto wrappedDistance = [wrapPeriod](float a, float b) {
        float d = std::abs(a - b);
        if (std::isfinite(wrapPeriod) && wrapPeriod > 0.0f) {
            d = std::fmod(d, wrapPeriod);
            d = std::min(d, wrapPeriod - d);
        }
        return d;
    };

    float best = 0.0f;
    float bestScore = INFINITY;

    for (int i = 0; i < nMaxB; i++)
    {
        for (int j = 0; j < nMaxA ; j++)
        {
            float aEstimate = m_encoderA->estimate(j);
            float bEstimate = m_encoderB->estimate(i);

            float err = std::abs(aEstimate - bEstimate);
            float curr = ((aEstimate * weightA) + (bEstimate * weightB)) / totWeight;
            if (curr > maxTurn)
                continue;

            float score = err;
            if (usePrevious)
                score += proximityWeight * wrappedDistance(curr, previousTurn);

            if (score < bestScore)
            {
                best = curr;
                bestScore = score;
            }
        }
    }

    return best;
}
void FRC::CRT::setTurn(const float& turn) {
    m_encoderA->setAngle01(turn * (m_turretTeeth / (float)m_encoderA->getGearTeeth()));
    m_encoderB->setAngle01(turn * (m_turretTeeth / (float)m_encoderB->getGearTeeth()));
}

void initCRT(py::module &m) {
    // CRT Encoder
    py::class_<FRC::CRTEncoder, std::shared_ptr<FRC::CRTEncoder>>(m, "CRTEncoder")
        .def(py::init<int, int, float>(),
            py::arg("gear_teeth"),
            py::arg("turret_teeth"),
            py::arg("unit"))
        .def("get_gear_teeth", &FRC::CRTEncoder::getGearTeeth)
        .def("get_turret_teeth", &FRC::CRTEncoder::getTurretTeeth)
        .def("get_angle01", &FRC::CRTEncoder::getAngle01)
        .def("set_angle01", &FRC::CRTEncoder::setAngle01)
        .def("get_angle", &FRC::CRTEncoder::getAngle)
        .def("set_angle", &FRC::CRTEncoder::setAngle)
        .def("estimate_encoder_turn", &FRC::CRTEncoder::estimateEncoderTurn)
        .def("estimate", &FRC::CRTEncoder::estimate);

    // CRT
    py::class_<FRC::CRT>(m, "CRT")
        .def(py::init<std::shared_ptr<FRC::CRTEncoder>, std::shared_ptr<FRC::CRTEncoder>, int>(),
            py::arg("encoder_a"),
            py::arg("encoder_b"),
            py::arg("turret_teeth"))
        .def("max_rotation", &FRC::CRT::maxRotation)
        .def("is_coprime", &FRC::CRT::isCoprime)
        .def("can_do_rotation", &FRC::CRT::canDoRotation)
        .def("is_valid", &FRC::CRT::isValid)
        .def("get_turn", &FRC::CRT::getTurn,
            py::arg("weight_a") = 1.0f,
            py::arg("weight_b") = 1.0f,
            py::arg("max_turn") = INFINITY,
            py::arg("previous_turn") = NAN,
            py::arg("proximity_weight") = 0.0f)
        .def("set_turn", &FRC::CRT::setTurn);
}