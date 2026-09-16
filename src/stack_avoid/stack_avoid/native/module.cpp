// Python buffer binding; no NumPy headers or NumPy ABI dependency.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <cstdint>
#include <cstring>

extern "C" bool footprint(const double *, const double *, const double *, std::int64_t,
                          const double *, std::int64_t, double, double, double, double);

static PyObject * check(PyObject *, PyObject * args)
{
  PyObject * objects[4];
  double width, front, rear, margin;
  if (!PyArg_ParseTuple(args, "OOOOdddd", &objects[0], &objects[1], &objects[2],
      &objects[3], &width, &front, &rear, &margin)) {return nullptr;}
  Py_buffer buffers[4]{};
  int acquired = 0;
  for (; acquired < 4; ++acquired) {
    if (PyObject_GetBuffer(objects[acquired], &buffers[acquired],
        PyBUF_FORMAT | PyBUF_C_CONTIGUOUS) != 0) {break;}
  }
  PyObject * result = nullptr;
  if (acquired == 4) {
    bool valid = true;
    for (const auto & b : buffers) {
      valid &= b.format && std::strcmp(b.format, "d") == 0 && b.itemsize == sizeof(double);
    }
    const auto & p = buffers[0];
    const auto & c = buffers[1];
    const auto & s = buffers[2];
    const auto & e = buffers[3];
    valid = valid && p.ndim == 2 && p.shape[1] == 4 &&
      c.ndim == 1 && s.ndim == 1 && c.shape[0] == p.shape[0] && s.shape[0] == p.shape[0] &&
      e.ndim == 3 && e.shape[1] == 2 && e.shape[2] == 2;
    if (!valid) {
      PyErr_SetString(PyExc_ValueError, "Expected native float64 buffers: poses(N,4), cos/sin(N), edges(M,2,2)");
    } else {
      // Keep the GIL: callers cannot mutate these buffers during the check.
      result = PyBool_FromLong(footprint(
        static_cast<const double *>(p.buf), static_cast<const double *>(c.buf),
        static_cast<const double *>(s.buf), p.shape[0], static_cast<const double *>(e.buf),
        e.shape[0], width, front, rear, margin));
    }
  }
  for (int i = 0; i < acquired; ++i) {PyBuffer_Release(&buffers[i]);}
  return result;
}

static PyMethodDef methods[] = {
  {"footprint", check, METH_VARARGS, "Check sampled vehicle boxes against obstacle edges."},
  {nullptr, nullptr, 0, nullptr}
};
static PyModuleDef module = {
  PyModuleDef_HEAD_INIT, "_footprint_native", nullptr, -1, methods
};
PyMODINIT_FUNC PyInit__footprint_native() {return PyModule_Create(&module);}
