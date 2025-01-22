from torch.utils.cpp_extension import load
import tempfile
import torch

source = """
#include <torch/extension.h>

torch::Tensor test_kernel(torch::Tensor input) {
    return input * 2;
}

PYBIND11_MODULE(TORCH_EXTENSION_NAME, m) {
    m.def("test_kernel", &test_kernel, "Test kernel");
}
"""

# Write the source code to a temporary file
with tempfile.NamedTemporaryFile(delete=False, suffix=".cpp") as temp_file:
    temp_file.write(source.encode("utf-8"))
    temp_file_path = temp_file.name

# Load the extension from the temporary file
test_extension = load(name="test_extension", sources=[temp_file_path])
print("CUDA extension compiled successfully.")


# Check PyTorch version
print(f"PyTorch version: {torch.__version__}")

# Check if CUDA is available
if torch.cuda.is_available():
    print(f"CUDA is available. CUDA version: {torch.version.cuda}")
    print(f"Number of GPUs: {torch.cuda.device_count()}")
    print(f"GPU Name: {torch.cuda.get_device_name(0)}")
else:
    print("CUDA is not available.")
