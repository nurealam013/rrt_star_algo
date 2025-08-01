# Path Planning for Autonomous Robots using RRT\* Algorithm

This project presents an interactive MATLAB-based simulation tool for mobile robot path planning using the **Rapidly-exploring Random Tree Star (RRT\*) algorithm**. It includes a graphical user interface (GUI) that allows users to visualize and analyze robot navigation in 2D environments with static obstacles.

---

## 📁 Files

- **Project Report.docx**: Full academic report detailing the background, design, implementation, analysis, and results of the RRT\*-based path planner.
- **RRT_Star_Path_Planning.m / .mlapp** (not included here): MATLAB script or GUI app file for the implementation of the algorithm (referenced in the report).
- **README.md**: You're reading it now!

---

## 📌 Features

- **User-defined start and goal positions**
- **Support for up to 15 customizable rectangular obstacles**
- **Step size and node limit configurability**
- **Real-time GUI visualization:**
  - Tree growth and final path
  - Cumulative path cost graph
  - Path comparison (RRT\* vs Direct)
- **Handles special cases** like goal inside obstacle
- **Educational tool** with detailed code structure and plotting

---

## 🧠 Algorithms Used

- **RRT\***: Optimized version of RRT that includes a rewiring step to minimize cost.
- **Nearest-neighbor search** with radius constraint
- **Collision detection** via axis-aligned rectangle checks
- **Optional ideas discussed**: Informed RRT\*, APF integration, dynamic replanning.

---

## 📊 Results Highlights

- Demonstrates effective path generation in environments with narrow corridors and cluttered spaces.
- Trade-off between step size and planning success is thoroughly analyzed.
- GUI enables experimentation with different setups for educational and practical learning.

---

## 🚀 Future Work

- Extend to **3D environments** (e.g., drone path planning)
- Add **dynamic obstacle tracking** and **real-time replanning**
- Integrate with **ROS/Gazebo and real robots**
- Include **mouse-based editing** of start/goal/obstacles
- Performance boost via **parallel computing**

---

## 📚 References

1. J. Du and Y. Zhang, "Research on Path Planning Algorithm of Mobile Robot Based on RRT", IEEE CITSC, 2025.
2. S.M. LaValle, "Rapidly-Exploring Random Trees: A New Tool for Path Planning", 1998. [PDF](http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf)
3. [RRT\* in MATLAB - YouTube Demo](https://www.youtube.com/watch?v=4Rd_gTYIft8)

---

## 👨‍💻 Authors

- **Md. Nure Alam Shagur** (2206013)
- **Raiyanul Islam Kabbo** (2206014)
- **Hasan Hazary** (2206015)
- **Nasiba Nahian Ahona** (2206016)

---

## 📺 Project Links

- 🔗 [GitHub](https://github.com/nurealam013/rrt_star_algo)
- 📽️ [YouTube Demo](https://www.youtube.com/watch?v=JOZFty-iTGc)

---

> 🔒 Academic Honesty: All code and analysis presented in this report were developed independently in accordance with the academic integrity policies of BUET.

