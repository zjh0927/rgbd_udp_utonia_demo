#include "rgbd_udp_receiver/zdepth_decoder.hpp"

#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace rgbd_udp_receiver
{

class PythonZdepthDecoder
{
public:
  PythonZdepthDecoder(
    std::uint32_t max_width,
    std::uint32_t max_height,
    std::size_t max_pixels,
    std::size_t max_compressed_bytes)
  : decoder_(ZdepthDecodeLimits{max_width, max_height, max_pixels, max_compressed_bytes})
  {
  }

  py::array_t<std::uint16_t> decode(const py::bytes & encoded)
  {
    const std::string owned = encoded;
    ZdepthDecodeResult result;
    {
      py::gil_scoped_release release;
      result = decoder_.decode(
        reinterpret_cast<const std::uint8_t *>(owned.data()), owned.size());
    }
    if (!result.ok) {
      throw std::runtime_error(result.error);
    }
    const std::vector<py::ssize_t> shape{
      static_cast<py::ssize_t>(result.height), static_cast<py::ssize_t>(result.width)};
    py::array_t<std::uint16_t> output(shape);
    std::memcpy(
      output.mutable_data(), result.millimeters.data(),
      result.millimeters.size() * sizeof(std::uint16_t));
    return output;
  }

private:
  ZdepthDecoderNative decoder_;
};

}  // namespace rgbd_udp_receiver

PYBIND11_MODULE(_zdepth, module)
{
  module.doc() = "Bounded Zdepth decoder for rgbd_udp_receiver";
  py::class_<rgbd_udp_receiver::PythonZdepthDecoder>(module, "Decoder")
    .def(
      py::init<std::uint32_t, std::uint32_t, std::size_t, std::size_t>(),
      py::arg("max_width"), py::arg("max_height"), py::arg("max_pixels"),
      py::arg("max_compressed_bytes"))
    .def("decode", &rgbd_udp_receiver::PythonZdepthDecoder::decode);
}
