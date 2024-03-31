<p style="display: inline">
  <!-- Programming Language -->
  <img src="https://img.shields.io/badge/-C++-00599C.svg?logo=c%2B%2B&style=for-the-badge">
  <!-- ROS 2 -->
  <img src="https://img.shields.io/badge/-ROS%202-22314E.svg?logo=ros&style=for-the-badge&logoColor=white">
</p>

## Functional Overview

## Requirements
### System Requirements
- OS : Ubuntu 22.04  
- ROS2 : Humble

## How To Use
### Execution Steps
```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash
ros2 launch path_smoother path_smoother.py
```

### Folder structure
```
path_smoother/
├── include/                     - Directory for header files
│   └── path_smoother/           - Header files for the path_smoother package
│       ├── cubic_spline_1d.hpp  - Implementation of 1D cubic spline
│       └── cubic_spline_2d.hpp  - Implementation of 2D cubic spline
├── launch/                      - Directory for ROS 2 launch files
│   └── path_smoother.py         - Launch script for the path_smoother node
├── path/                        - Directory for simulation path files
│   └── simulation_path.csv      - Path data for simulation (CSV format)
├── src/                         - Directory for source files
│   ├── cubic_spline_1d_node.cpp - Node implementation for 1D cubic spline
│   ├── cubic_spline_1d_component.cpp - Component implementation for 1D cubic spline
│   ├── cubic_spline_2d_node.cpp - Node implementation for 2D cubic spline
│   ├── cubic_spline_2d_component.cpp - Component implementation for 2D cubic spline
│   └── cubic_spline_node.cpp    - Node implementation for cubic spline calculation
├── CMakeLists.txt               - CMake build configuration file
└── package.xml                  - ROS 2 package metadata file
```

## Interface Table

### Input

### Output

### Internal Values

## Software architecture

### Class Diagram

```mermaid
classDiagram
    class CubicSpline1D {
        +std::vector<double> a
        +std::vector<double> b
        +std::vector<double> c
        +std::vector<double> d
        +std::vector<double> x
        +std::vector<double> y
        +int nx
        +CubicSpline1D(vector<double>&, vector<double>&)
        +double calc_position(double)
        +double calc_first_derivative(double)
        +double calc_second_derivative(double)
        -Eigen::MatrixXd calc_A(vector<double>&)
        -Eigen::VectorXd calc_B(vector<double>&)
        -int search_index(double)
    }

    class CubicSpline2D {
        +CubicSpline2D(vector<double>&, vector<double>&)
        +std::pair<double, double> calc_position(double)
        +double calc_curvature(double)
        +double calc_yaw(double)
        +double get_s_max()
        +std::vector<double> s
        +std::vector<double> ds
        +CubicSpline1D sx
        +CubicSpline1D sy
        -void calc_s(vector<double>&, vector<double>&)
    }
    CubicSpline2D --|> CubicSpline1D: Contains
```

### Flowchart
```mermaid
flowchart TD
    A[Start] --> B[Create CubicSpline1D for x and y]
    B --> C[Calculate Cumulative Distance s]
    C --> D[Create CubicSpline1D for sx and sy using s]
    D --> E[Calculate Position at s_query]
    E --> F[Calculate Curvature at s_query]
    E --> G[Calculate Yaw at s_query]
    F --> H[End]
    G --> H
```

## Functional Requirements

## Detailed Design
