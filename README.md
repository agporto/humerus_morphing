
## Humerus Morphing and Completion

This repo provides a pipeline for morphing and completing partial humerus models using a template. The script uses 3D point cloud registration techniques with the Open3D library to align and merge the partial humerus model with a complete template. The script reads in two 3D models, converts them into point clouds, and then aligns them using a combination of global and local (refined) registration methods. The script also includes a deformable registration method using the bcpd algorithm.


### Prerequisites

The project requires Python 3.7 or later and Conda. If you don't have Python installed, you can download it from the official site: https://www.python.org/downloads/

Conda can be installed by downloading Anaconda or Miniconda. You can download Anaconda from here: https://www.anaconda.com/products/distribution or Miniconda from here: https://docs.conda.io/en/latest/miniconda.html

### Setting up the Conda Environment

After installing Conda, you can create a new environment and install the necessary packages with the following commands:

```bash
# Create a new Conda environment with Python 3.7
conda create -n morphing python=3.7

# Activate the environment
conda activate morphing

# Install necessary packages
pip install open3d
pip install numpy
```

### Compiling bcpd

The bcpd algorithm is used for deformable registration in this project. You can compile it by following these steps:

1. Clone the bcpd repository:

```bash
git clone https://github.com/ohirose/bcpd.git
```

2. Navigate to the cloned repository:

```bash
cd bcpd
```

3. Compile the code:

#### Windows

Ready to go. The compilation is not required. Use the binary file `bcpd.exe` in the `win` directory.
The binary file was created by GCC included in the 32-bit version of the MinGW system.
Therefore, it might be quite slower than the one compiled in a Mac/Linux system.

**Note that in the Jupyter notebook, remove ".\" from .\bcpd .\target.txt and .\source.txt
Furthermore, as described in the bcpd README file, ensure that the current director is set to the bcpd folder when running the Jupyter script, and that all meshes are included in that folder.**

#### MacOS and Linux

1. Install OpenMP and the LAPACK library if not installed. If your machine is a Mac, install Xcode, Xcode command-line tools,
   and MacPorts (or Homebrew).
2. Download and decompress the zip file that includes source codes.
3. Move into the top directory of the uncompressed folder using the terminal window.
4. Type `make OPT=-DUSE_OPENMP ENV=<your-environment>`; replace `<your-environment>` with `LINUX`,
   `HOMEBREW`, `HOMEBREW_INTEL`, or `MACPORTS`. Type `make OPT=-DNUSE_OPENMP` when disabling OpenMP.

Homebrew's default installation path changes according to Mac's CPU type.
If you use an Intel Mac, specify `HOMEBREW_INTEL` instead of `HOMEBREW`.


### Running the Script

After setting up the environment and compiling bcpd, you can run open the `morphing.ipynb` notebook and run the script. 



## Authors

* **Arthur Porto**
* **Eva Herbst**
