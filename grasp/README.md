# ROS2 Grasp Module
ROS2 uses python3.10.
```
$ python --version
Python 3.10.12
```

To be compatible with ROS2, I did not use conda environment.

## Install AnyGrasp

### MinkowskiEngine
According to [anygrasp_sdk installation instruction](https://github.com/graspnet/anygrasp_sdk?tab=readme-ov-file#installation), MinkowskiEngine is required. 

#### 1. Pytorch and CUDA
Before installing it, make sure the torch and cuda version are compatible with MinkowskiEngine, which are not clarified clearly in the repo but I think cuda 11.x will work. 

Check [version correspondence between Python and torch](https://github.com/pytorch/vision#installation) and bear in mind that **torch >= 1.11**

Checking [previous versions of pytorch](https://pytorch.org/get-started/previous-versions/), I chose torch 2.0.1 and CUDA 11.7. Remember to export $CUDA_HOME to 11.7 directory. I use the following to check my environment and install the MinkowskiEngine successfully:
```
$ python
>>> import torch
>>> torch.__version__
'2.0.1+cu117'
>>> torch.version.cuda
'11.7'
```
```
$ echo $CUDA_HOME
/usr/local/cuda-11.7
```
```
$ nvcc --version
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2022 NVIDIA Corporation
Built on Wed_Jun__8_16:49:14_PDT_2022
Cuda compilation tools, release 11.7, V11.7.99
Build cuda_11.7.r11.7/compiler.31442593_0
```
#### 2. Install MinkowskiEngine from source using pip
Refer to [MinkowskiEngine pip installation](https://github.com/NVIDIA/MinkowskiEngine?tab=readme-ov-file#pip)

If your pip is new, **DO NOT** use this command ```pip install -U MinkowskiEngine --install-option="--blas=openblas" -v --no-depspip install -U MinkowskiEngine --install-option="--blas=openblas" -v --no-deps``` because *--install-option* is deprecated. You can install from source in either way:

1. Clone the GitHub repo, change the directory to it, and use ```pip install -U ./```

2. Use ```pip install -U git+https://github.com/NVIDIA/MinkowskiEngine --no-deps```

#### 3. Check MinkowskiEngine installation
You can find the example Python script [here](./test_scripts/MinkowskiEngineTest.py) to test MinkowskiEngine.

```python
import torch
import MinkowskiEngine as ME
import torch.nn as nn
import numpy as np

class ExampleNetwork(nn.Module):
    def __init__(self, in_feat, out_feat, D):
        super(ExampleNetwork, self).__init__()
        self.conv1 = ME.MinkowskiConvolution(in_channels=in_feat, out_channels=64, kernel_size=3, stride=2, dimension=D)
        self.bn1 = ME.MinkowskiBatchNorm(64)
        self.relu1 = ME.MinkowskiReLU()
        self.conv2 = ME.MinkowskiConvolution(in_channels=64, out_channels=128, kernel_size=3, stride=2, dimension=D)
        self.bn2 = ME.MinkowskiBatchNorm(128)
        self.relu2 = ME.MinkowskiReLU()
        self.pooling = ME.MinkowskiGlobalPooling()
        self.linear = ME.MinkowskiLinear(128, out_feat)

    def forward(self, x):
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu1(out)
        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu2(out)
        out = self.pooling(out)
        return self.linear(out)

# Arbitrary data loader function
def data_loader(batch_size=4, num_points=100, num_features=3, num_classes=5):
    coords = []
    feat = []
    labels = []
    for i in range(batch_size):
        batch_coords = np.random.randint(0, 100, (num_points, 2))  # Random 2D coordinates
        batch_coords = np.hstack((np.full((num_points, 1), i), batch_coords))  # Add batch index
        coords.append(batch_coords)
        feat.append(np.random.rand(num_points, num_features))  # Random features
        labels.append(np.random.randint(0, num_classes, num_points))  # Random labels for each point

    coords = np.vstack(coords)
    feat = np.vstack(feat)
    labels = np.hstack(labels)

    return coords, feat, labels

# Initialize network and loss function
net = ExampleNetwork(in_feat=3, out_feat=5, D=2)
criterion = nn.CrossEntropyLoss()

# Load data
coords, feat, labels = data_loader()
coords = torch.tensor(coords, dtype=torch.int32)
feat = torch.tensor(feat, dtype=torch.float32)
labels = torch.tensor(labels, dtype=torch.long)

# Create SparseTensor
input = ME.SparseTensor(feat, coordinates=coords)

# Forward pass
output = net(input)

# Compute loss
loss = criterion(output.F, labels)
print(f'Loss: {loss.item()}')
```

### Install anygrasp_sdk

1. Clone the repo and follow [intallation instruction](https://github.com/graspnet/anygrasp_sdk?tab=readme-ov-file#installation)

    If you meet this error, which is saying 'sklearn' is deprecated, you could refer to [this answer for the issue](https://github.com/graspnet/graspnetAPI/issues/43#issuecomment-1594338597). Simply run `export SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL=True` to bypass the error.
    ```
    Collecting sklearn (from graspnetAPI->-r requirements.txt (line 5))
    Downloading sklearn-0.0.post12.tar.gz (2.6 kB)
    Preparing metadata (setup.py) ... error
    error: subprocess-exited-with-error
    
    × python setup.py egg_info did not run successfully.
    │ exit code: 1
    ╰─> [15 lines of output]
        The 'sklearn' PyPI package is deprecated, use 'scikit-learn'
        rather than 'sklearn' for pip commands.
        
        Here is how to fix this error in the main use cases:
        - use 'pip install scikit-learn' rather than 'pip install sklearn'
        - replace 'sklearn' by 'scikit-learn' in your pip requirements files
            (requirements.txt, setup.py, setup.cfg, Pipfile, etc ...)
        - if the 'sklearn' package is used by one of your dependencies,
            it would be great if you take some time to track which package uses
            'sklearn' instead of 'scikit-learn' and report it to their issue tracker
        - as a last resort, set the environment variable
            SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL=True to avoid this error
    ```

2. I used `pip install .` to install `pointnet2` module in its directory.

3. Refer to [license registration](https://github.com/graspnet/anygrasp_sdk?tab=readme-ov-file#license-registration) to get SDK library file.

4. Run the demo

    I think in the email reply there is an instruction of downloading model weights. Follow this [README](https://github.com/graspnet/anygrasp_sdk/tree/main/grasp_detection#anygrasp-detection-demo) to run the detection demo. You need to put the library file, your license folder, and model weights into correct place.

    Author had tested the demo code using numpy 1.23.4. But me using 1.24.4 there is an error, as described in [Issue #17](https://github.com/graspnet/anygrasp_sdk/issues/17). So I downgrade the numpy to 1.23.5 and succeeded.

    Most of warnings and errors running the demo can be found in Issues.

## Install GroundingDINO

### Install as submodule
GroundingDINO should be cloned as a submodule in gdino_sam ROS2 package using following commands in the root directory of repo:

```
git submodule init
git submodule update
```

Then install the package according to its [README](./gdino_sam/GroundingDINO/README.md).

If you have set the CUDA_HOME, just go to GroundingDINO directory and use
```
pip install -e .
```

### Download pre-trained weights
After that download the pre-trained model weights 
```
mkdir weights
cd weights
wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
cd ..
```

(It's ok to put the .pth file in /pth folder for management as long as you refer to correct location in your code.)

### Test
Use [this script](./gdino_sam/scripts/GDinoTest.py) to test your installation. Remember to figure out the paths before running.

The results will be save as an annotated image and a json file containing the information of bounding boxes. You can check the output directory indicated in the code.

## Install Segment Anything Model

Refer to [this link](https://github.com/facebookresearch/segment-anything?tab=readme-ov-file#installation) to install Segment Anything and downloas the pre-trained weights in pth folder.

Run [this test script](./gdino_sam/scripts/SAMTest.py) (after you have tested with GroundingDINO) to segment the object prompted by the bounding boxes derived from GoundingDINO. The result is a masked image stored in output path and will be shown using cv on the screen.

### Test two models together

Run [this test script](./gdino_sam/scripts/GDinoSAMTest.py). The result image will contain both bounding boxes and masks.