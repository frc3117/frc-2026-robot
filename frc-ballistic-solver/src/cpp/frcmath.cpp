#include "frcmath.h"

#include <pybind11/pybind11.h>
namespace py = pybind11;

#include "frcmath/vector2.h"
void initVector2(py::module &m);

void initMath(py::module &m) {
    initVector2(m);
}