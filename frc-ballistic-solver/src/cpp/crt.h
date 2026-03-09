#ifndef CRT_H
#define CRT_H

#include <pybind11/pybind11.h>

#include <cmath>
#include <memory>

namespace py = pybind11;

void initCRT(py::module &m);
namespace FRC
{
    class CRTEncoder : public std::enable_shared_from_this<CRTEncoder> {
        public:
        CRTEncoder();
        CRTEncoder(const int& gearTeeth, const int& turretTeeth, const float& unit);

        int getGearTeeth();
        int getTurretTeeth();

        float getAngle01();
        void setAngle01(const float& angle);

        float getAngle();
        void setAngle(const float& angle);

        float estimateEncoderTurn(const float& turretTurn);

        float estimate(const int& n);

        private:
        int m_gearTeeth;
        int m_turretTeeth;
        float m_unit;

        float m_gearRatio;
        float m_inverseGearRatio;
        float m_encoderAngle;
    };

    class CRT {
        public:
        CRT(const std::shared_ptr<FRC::CRTEncoder>& encoderA, const std::shared_ptr<FRC::CRTEncoder>& encoderB, const int& turretTeeth);

        float maxRotation();

        bool isCoprime();
        bool canDoRotation(const float& rotationCount);
        bool isValid(const float& rotationCount);

        float getTurn(const float& weightA = 1.0f, const float& weightB = 1.0f, const float& maxTurn = INFINITY);
        void setTurn(const float& turn);

        private:
        std::shared_ptr<CRTEncoder> m_encoderA;
        std::shared_ptr<CRTEncoder> m_encoderB;
        int m_turretTeeth;
    };
}
#endif