# RA-ISAM2 Project Structure

## Overview
This is a **Resource-Aware ISAM2** fork of the GTSAM (Georgia Tech Smoothing and Mapping) library that implements resource-aware incremental optimization used in SuperNoVA.

GTSAM is a C++ library that performs incremental smoothing and mapping (ISAM) and other robotics/vision optimization tasks. The resource-aware version adds capabilities for managing computational resources efficiently.

---

## Top-Level Directory Structure

```
ra-isam2/
├── CMakeLists.txt              # Root CMake configuration
├── README.md                   # Project overview
├── DEVELOP.md                  # Developer guidelines & conventions
├── INSTALL.md                  # Installation instructions
├── USAGE.md                    # Usage guide and examples
├── TESTING.md                  # Testing documentation
├── GTSAM-Concepts.md           # GTSAM concepts explanation
├── Using-GTSAM-EXPORT.md       # Windows export macro guide
├── LICENSE & LICENSE.BSD       # License files
│
├── cmake/                      # CMake utility modules
│   ├── FindBoost.cmake, FindEigen3.cmake, etc.  # Dependency finders
│   ├── HandleBoost.cmake, HandleEigen.cmake, etc.  # Dependency handlers
│   └── GtsamTesting.cmake      # Testing setup
│
├── CppUnitLite/                # Custom unit testing framework
│   ├── Test.h/cpp              # Test base class
│   ├── TestRegistry.h/cpp      # Test registry
│   └── TestResult.h/cpp        # Test result tracking
│
├── gtsam/                      # CORE LIBRARY (main source code)
├── gtsam_unstable/             # Experimental/development features
├── python/                     # Python bindings
├── matlab/                     # MATLAB bindings
├── wrap/                       # Wrapper generation tools
│
├── examples/                   # Example programs
│   ├── CameraResectioning.cpp
│   ├── ImuFactorsExample.cpp
│   ├── ISAM2Example_SmartFactor.cpp
│   └── ... (many more examples)
│
├── tests/                      # Test suite
├── timing/                     # Performance timing tools
├── doc/                        # Documentation (Doxygen, LaTeX)
├── docker/                     # Docker configurations
└── scripts/                    # Utility scripts
```

---

## CORE LIBRARY: `gtsam/` Directory

This is the heart of the project, organized into functional modules:

### 1. **base/** - Foundation & Utilities
```
base/
├── Group.h, Lie.h              # Abstract base classes for manifolds
├── Matrix.h, Vector.h          # Linear algebra wrappers
├── types.h                     # Type system and definitions
├── numericalDerivative.h       # Numerical differentiation utilities
├── Testable.h                  # Base class for testable objects
├── TestableAssertions.h        # Assertion utilities for testing
├── ConcurrentMap.h             # Thread-safe map implementation
├── FastVector.h, FastMap.h, FastSet.h  # Optimized containers
├── serialization.h             # Serialization support
└── timing.h/cpp                # Performance timing
```
**Purpose**: Provides basic data structures, mathematical utilities, and testing infrastructure.

---

### 2. **geometry/** - Geometric Primitives
```
geometry/
├── Point3.h                    # 3D point class
├── Rot2.h, Rot3.h              # 2D/3D rotation representations
├── Pose2.h, Pose3.h            # 2D/3D pose (position + rotation)
├── Camera.h                    # Camera model
├── Calibration*.h              # Camera calibration models
├── OrientedPlane3.h            # Plane representation
└── ... (other geometric types)
```
**Purpose**: Geometric types needed for SLAM and vision applications.

---

### 3. **inference/** - Graphical Model Inference
```
inference/
├── Factor.h/cpp                # Base factor class
├── FactorGraph.h/cpp           # Factor graph representation
├── BayesNet.h                  # Bayesian network
├── BayesTree.h                 # Bayes tree (junction tree variant)
├── Conditional.h               # Conditional probability
├── EliminationTree.h           # Elimination tree for variable ordering
├── Ordering.h/cpp              # Variable ordering strategies
├── VariableIndex.h             # Index of variables in graph
├── ISAM.h                      # Basic ISAM algorithm
├── Symbol.h/cpp                # Variable naming system
├── LabeledSymbol.h             # Labeled variable names
└── Key.h/cpp                   # Key type for variables
```
**Purpose**: Core graphical model representations and algorithms.

---

### 4. **linear/** - Linear Algebra & Linear Inference
```
linear/
├── GaussianFactor.h            # Linear factor for Gaussian
├── GaussianFactorGraph.h       # Linear factor graph
├── GaussianConditional.h       # Gaussian conditional
├── GaussianBayesNet.h          # Gaussian Bayes net
├── GaussianBayesTree.h         # Gaussian Bayes tree
├── GaussianEliminationTree.h   # Linear elimination
├── Sampler.h                   # Gaussian sampler
├── Preconditioner.h            # Preconditioning
├── iterative-solver/           # Iterative linear solvers
│   ├── ConjugateGradientSolver.h
│   ├── MINRES.h
│   └── ... (various solvers)
└── ... (linear algebra utilities)
```
**Purpose**: Linear algebra operations and linear system solving.

---

### 5. **nonlinear/** - Nonlinear Optimization (RESOURCE-AWARE MODIFICATIONS)
```
nonlinear/
├── NonlinearFactor.h/cpp       # Base nonlinear factor
├── NonlinearFactorGraph.h/cpp  # Nonlinear factor graph
├── NonlinearOptimizer.h        # Base optimizer interface
├── GaussNewtonOptimizer.h      # Gauss-Newton method
├── LevenbergMarquardtOptimizer.h/cpp  # Levenberg-Marquardt
├── DoglegOptimizer.h           # Dogleg optimizer
├── NonlinearConjugateGradientOptimizer.h  # Nonlinear CG
│
├── ISAM2.h/cpp                 # **MAIN INCREMENTAL OPTIMIZER**
├── ISAM2Params.h               # ISAM2 configuration
├── ISAM2Result.h               # ISAM2 optimization results
├── ISAM2Clique.h               # ISAM2 clique data structure
│
├── Values.h/cpp                # Container for variable values
├── Marginals.h                 # Covariance extraction
├── ExtendedKalmanFilter.h      # EKF implementation
├── ExpressionFactorGraph.h     # Factor graphs with expressions
└── Expression.h                # Expression templates for factors
```
**Purpose**: Nonlinear optimization core - **ISAM2 is the main algorithm here**.

---

### 6. **slam/** - SLAM Applications
```
slam/
├── BetweenFactor.h             # Relative measurement factors
├── PriorFactor.h               # Prior factors
├── PoseRTV.h                   # Pose with velocity
├── ProjectionFactor.h          # Camera projection factors
├── RangeFactor.h               # Range measurement factors
├── IMUFactor.h                 # Inertial measurement factors
└── ... (SLAM-specific factors and models)
```
**Purpose**: Application-level factors for SLAM problems.

---

### 7. **navigation/** - Navigation & IMU
```
navigation/
├── AHRSFactor.h                # Attitude and heading reference system
├── CombinedImuFactor.h         # Combined IMU factor
├── NavState.h                  # Navigation state (pose, velocity, bias)
├── ImuFactor.h                 # IMU preintegration factor
├── TangentPreintegration.h     # Preintegration on tangent space
└── ... (navigation-related factors)
```
**Purpose**: IMU integration and navigation algorithms.

---

### 8. **sfm/** - Structure from Motion
```
sfm/
├── BundleAdjustmentData.h      # Bundle adjustment data structures
├── TranslationFactor.h         # Translation measurement factors
├── DnCmarks.h                  # Divide-and-conquer SfM
├── SfmData.h                   # SfM data container
└── ... (SfM-specific implementations)
```
**Purpose**: Vision-based structure from motion algorithms.

---

### 9. **discrete/** - Discrete Inference
```
discrete/
├── DiscreteConditional.h       # Discrete conditional
├── DiscreteFactor.h            # Discrete factor
├── DiscreteFactorGraph.h       # Discrete factor graph
├── DiscreteValues.h            # Discrete variable values
└── ... (discrete graphical model support)
```
**Purpose**: Discrete and hybrid graphical models.

---

### 10. **3rdparty/** - External Dependencies
```
3rdparty/
├── Eigen/                      # Eigen linear algebra library
├── metis/                      # METIS graph partitioning
├── ccolamd/                    # Column approximate minimum degree
└── ... (vendored dependencies)
```

---

### 11. **basis/** - Basis Utilities
```
basis/
├── Basis.h                     # Basis representation
└── ... (basis-related utilities)
```

---

## Supporting Components

### **python/** - Python Bindings
- SWIG-generated Python wrappers
- Allows using GTSAM from Python
- Located in `python/gtsam/`

### **matlab/** - MATLAB Bindings
- SWIG-generated MATLAB wrappers
- Allows using GTSAM from MATLAB

### **wrap/** - Wrapper Generation
- Tools for generating Python/MATLAB bindings from C++ code
- SWIG configuration files

### **examples/** - Example Code
- Demonstrates various GTSAM features:
  - `ImuFactorsExample.cpp` - IMU integration
  - `CameraResectioning.cpp` - Camera pose estimation
  - `ISAM2Example_SmartFactor.cpp` - **Key ISAM2 example**
  - Visual SLAM examples
  - Kalman filter examples

### **tests/** - Test Suite
- Comprehensive unit tests for all modules
- Test runners and fixtures

### **doc/** - Documentation
- Doxygen configuration
- LaTeX documentation files
- Algorithm descriptions
- Mathematical notation

### **docker/** - Container Configs
- Ubuntu-based Docker images
- Configurations for:
  - `ubuntu-gtsam/` - Basic GTSAM
  - `ubuntu-gtsam-python/` - GTSAM with Python
  - `ubuntu-gtsam-python-vnc/` - GTSAM with VNC server

### **cmake/** - Build Configuration
- Custom CMake modules for:
  - Finding dependencies (Boost, Eigen, TBB, MKL)
  - Handling compiler flags
  - Test configuration
  - Package generation

---

## Key Classes & Concepts

### Factor Graphs
- **FactorGraph**: Base undirected graphical model
- **BayesNet**: Bayesian network (directed acyclic graph)
- **BayesTree**: Efficient representation of Bayes network

### Inference
- **ISAM2**: **Incremental Smoothing and Mapping v2** - main optimization algorithm
  - Maintains a Bayes tree of marginal distributions
  - Efficient incremental updates
  - Relinearization on demand
  - **This is what "RA-ISAM2" (Resource-Aware ISAM2) extends**

### Values & Optimization
- **Values**: Container holding all variable values
- **NonlinearOptimizer**: Base class for optimization methods
- **Marginals**: Extract covariances from solution

### Geometry
- **Pose3**: 6DOF pose (position + rotation)
- **Point3**: 3D point
- **Rot3**: 3D rotation matrix
- **Camera**: Pinhole camera model

---

## Compilation & Build

```
CMakeLists.txt (root)
├── gtsam/CMakeLists.txt        # Core library
├── gtsam_unstable/CMakeLists.txt
├── examples/CMakeLists.txt
├── tests/CMakeLists.txt
├── python/CMakeLists.txt
├── matlab/CMakeLists.txt
└── doc/CMakeLists.txt
```

Build process:
1. CMake processes dependencies (Eigen, Boost, TBB, etc.)
2. Compiles C++ library
3. Optionally generates Python/MATLAB bindings
4. Runs test suite
5. Generates documentation

---

## Typical Workflow

### Using ISAM2 for SLAM:

1. **Create a Factor Graph**: Add measurement factors and priors
2. **Add Variables**: Use Keys and Symbols to identify poses/landmarks
3. **Initialize with ISAM2**: Create ISAM2 optimizer with parameters
4. **Incremental Updates**: Add new factors and update with ISAM2
5. **Extract Results**: Get optimized Values and Marginals

### Example Pseudo-code:
```cpp
// Create factor graph
NonlinearFactorGraph graph;
Values initialEstimate;

// Add factors (measurements)
graph.addPrior(0, pose, measurementNoise);
graph.add(BetweenFactor(0, 1, relativeMeasurement, noise));

// Initialize values
initialEstimate.insert(0, Pose3());
initialEstimate.insert(1, Pose3());

// Create and update ISAM2
ISAM2 isam2(ISAM2Params());
isam2.update(graph, initialEstimate);

// Get results
Values result = isam2.calculateEstimate();
```

---

## Resource-Aware Extensions

The "RA" (Resource-Aware) modifications to ISAM2 likely include:
- Computational budget management
- Variable importance tracking
- Adaptive relinearization
- Resource-constrained optimization
- Used by SuperNoVA project for efficient optimization under constraints

---

## Summary

The ra-isam2 project is a sophisticated C++ optimization library built on:
- **Strong mathematical foundation** (manifolds, Lie groups, factor graphs)
- **Efficient algorithms** (ISAM2, variable elimination, Bayes trees)
- **Production-ready code** (tested, documented, optimized)
- **Flexible interfaces** (C++, Python, MATLAB)
- **Resource-aware enhancements** for constrained environments
