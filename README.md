# ROS 2 + GitHub + WSL2 Setup Guide
This is a guide on how to setup your development environnment.

Linux distribution: Ubuntu 22.04 LTS
ROS 2 distribution: Humble Hawksbill (Humble)

## 1. Introduction

### What Is WSL2?
**WSL2** (Windows Subsystem for Linux 2) lets you run a Linux environment directly on Windows. It uses a lightweight virtual machine under the hood, but is more integrated with Windows than a normal VM. This means you can:
- Use Linux tools (like apt, Bash, and Python) in a Windows environment
- Seamlessly share files between Windows and Linux
- Avoid the overhead of a full dual-boot or separate physical machine

### Why Ubuntu on WSL2?
- **Ubuntu** is the most popular Linux distro for ROS 2 development
- ROS 2 Humble specifically targets Ubuntu 22.04 for official support
- Using it in WSL2 provides a near-native Linux experience on Windows

### What Is ROS 2?
**ROS 2** (Robot Operating System 2) is an open-source framework that provides tools, libraries, and conventions for building robot applications. It handles:
- Pub/Sub communication between different parts (nodes)
- Tools like `ros2 run`, `ros2 topic`, `rqt_graph`, and more
- Easy ways to integrate hardware drivers, sensor processing, and advanced algorithms
- Standard platform in robotics for code reusability
- Large ecosystem of packages for sensors, motors, planning

---

## 2. Install WSL2 & Ubuntu on Windows

### Step 1: Enable WSL2
In a **PowerShell (Admin)** session:
```powershell
wsl --install
```
This installs the Windows Subsystem for Linux with default settings, allowing your system to run Linux kernels under Windows.

**Potential Reboot**: If it prompts you to reboot, do so.

### Step 2: Check WSL Version
```powershell
wsl --set-default-version 2
```
WSL2 provides much better performance and more complete Linux kernel features compared to the older WSL1.

### Step 3: Launch Ubuntu (WSL)
Go to the **Start Menu** → type **Ubuntu** → launch it.

Finalising the setup of your Linux environment. Here you will setup your username and password.

### Step 4: Update Ubuntu
```bash
sudo apt update && sudo apt upgrade -y
```
Ensures system packages are current, removing security issues or old dependencies that might conflict with ROS 2.

---

## 3. Install ROS 2 Humble

ROS 2 targets Ubuntu 22.04 for the **Humble** release. We’ll install the full **desktop** variant for a complete environment.

### Step 1: Add ROS 2 Repositories
```bash
sudo apt install software-properties-common 
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Step 2: Add the ROS 2 apt Repository
```bash
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
This line adds the actual ROS 2 distribution (`ros2.list`) to your system so we can install `ros-humble-*` packages.

### Step 3: Install ROS 2 Desktop
```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```
`ros-humble-desktop` includes core ROS 2 plus visualisation tools like RViz. It’s the recommended starting point.

### Step 4: Source ROS Setup
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Add this line to `.bashrc` so every new shell automatically knows about ROS 2 commands (like `ros2 run` and `colcon`).

### Step 5: Install Colcon + Build Tools
```bash
sudo apt install python3-colcon-common-extensions python3-pip -y
```
**colcon** is the build tool for ROS 2, and Python’s `pip` can install various Python packages. `colcon-common-extensions` provides extra sub-commands and features.

---

## 4. Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```
**Why?**
- `mkdir -p ~/ros2_ws/src` sets up the standard ROS 2 workspace layout
- `colcon build` (even with an empty `src/`) preps the build environment
- `source install/setup.bash` makes sure the workspace is recognized in your shell

You can also add:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```
to **automatically** source your local workspace every new session.

---

## 5. Set Up VS Code + WSL

### Why VS Code?
- Lightweight yet powerful
- Extensions for Python, ROS, and WSL
- Built-in Git integration
- Big community and great debugging

### Step 1: Install VS Code on Windows
Download from [https://code.visualstudio.com/](https://code.visualstudio.com/). Install normally.

### Step 2: Install Key Extensions
- **Remote - WSL** (by Microsoft)
- **Python** (for linting, debugging, etc.)
- **ROS** (by ms-iot) for helpful ROS 2 functionality

### Step 3: Open Your Workspace from WSL
```bash
cd ~/ros2_ws
code .
```
**Why?** This commands VS Code (on Windows) to open the folder in a WSL remote session, so your code runs in Linux but you edit in Windows.

---

## 6. Set Up SSH Keys for Git + GitHub

### Why Use SSH?
GitHub no longer supports password-based authentication. SSH keys:
- Are more secure and convenient
- Don’t require re-entering credentials every push

### Step 1: Generate an SSH Key
```bash
ssh-keygen -t ed25519 -C "your@email.com"
```
**Why Ed25519?** Smaller, faster, and secure. Use the same email linked to your GitHub account.

### Step 2: Add Key to GitHub
```bash
cat ~/.ssh/id_ed25519.pub
```
Copy the entire line (starts with `ssh-ed25519 ...`) and go to [https://github.com/settings/keys](https://github.com/settings/keys). Click **New SSH Key** to paste.

### Step 3: Start the SSH Agent
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```
**Why?** This ensures your key is "active" for the current session.

### Step 4: Test Connection
```bash
ssh -T git@github.com
```
Expected:
```
Hi yourusername! You've successfully authenticated...
```
If you see *"permission denied"*, recheck the steps above.

---

## 7. Cloning/Creating Your Shared ROS 2 Repo

### Why Have a Shared GitHub Repo?
- Central place for all team members to store, share, and update code
- Each team member can clone, branch, and contribute without stepping on each other

### Step 1: Create a Repo on GitHub
1. Go to [https://github.com/new](https://github.com/new)
2. Name it, e.g. `battery_disassembly_framework`
3. Public or private (if private, add teammates as collaborators)

### Step 2: Clone the Repo into `~/ros2_ws/src`
```bash
cd ~/ros2_ws/src
git clone git@github.com:your-username/battery_disassembly_framework.git
```

### Step 3: Build and Source
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
**Why?** So any new ROS 2 packages in `src/` are compiled and recognized.

---

## 8. Team Collaboration Workflow

### Step 1: Pull Latest
Before coding each day:
```bash
git checkout main
git pull origin main
```
**Why?** Ensures you have the newest code before starting your work.

### Step 2: Create a Feature Branch
```bash
git checkout -b yourname/feature-description
```
**Why?** Each developer works in a **branch** to avoid conflicts on `main`.

### Step 3: Commit & Push
```bash
git add .
git commit -m "Implement feature X"
git push -u origin yourname/feature-description
```
**Why?** This pushes your branch to GitHub so others can see or review your progress.

### Step 4: Open Pull Request (PR)
- On GitHub, select **Compare & pull request**
- Describe your changes
- Request review or merge

### Step 5: Merge to Main
Once approved, merge your branch into `main`. Everyone else can then `git pull origin main` to see your changes.

---

## 9. Common .gitignore

In the root of your repo, create `.gitignore`:
```
build/
install/
log/
__pycache__/
*.pyc
.vscode/
```
**Why?** Keeps auto-generated files and personal editor settings out of version control.

---

## 10. Running a Python ROS 2 Node

### Step 1: Create a Python Package

Inside `~/ros2_ws/src/battery_disassembly_framework/`, you might have:
```
battery_disassembly_framework/
├── package.xml
├── setup.py
├── battery_disassembly_framework/
│   ├── __init__.py
│   └── disassembler_node.py
```

### Step 2: Declare the Node in `setup.py`
```python
entry_points={
    'console_scripts': [
        'disassemble = battery_disassembly_framework.disassembler_node:main',
    ],
},
```

### Step 3: Build & Run
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run battery_disassembly_framework disassemble
```
**Why?** This triggers your node’s `main()` function. `colcon build` ensures everything is up-to-date.

---

## ✅ Summary Table

| **Task**                            | **Command**                                                         | **Why?**                                                                                |
|-------------------------------------|---------------------------------------------------------------------|-----------------------------------------------------------------------------------------|
| **Install WSL2**                    | `wsl --install` (in PowerShell Admin)                              | Runs Linux natively on Windows                                                          |
| **Update Ubuntu**                   | `sudo apt update && sudo apt upgrade -y`                           | Ensures system is current                                                               |
| **Install ROS 2**                   | `sudo apt install ros-humble-desktop -y`                           | Full robotics framework                                                                 |
| **Create workspace**                | `mkdir -p ~/ros2_ws/src && colcon build`                           | Standard ROS 2 dev layout                                                               |
| **Set up VS Code**                  | `code .` from WSL                                                   | Use a powerful IDE in a remote Linux environment                                       |
| **Generate SSH key**                | `ssh-keygen -t ed25519 -C "email"`                                 | Secure GitHub access                                                                    |
| **Add key to GitHub**               | Copy from `~/.ssh/id_ed25519.pub`                                   | So GitHub recognizes your device                                                        |
| **Switch remote to SSH**            | `git remote set-url origin git@github.com:...`                     | No password needed, uses SSH keys                                                       |
| **Push changes**                    | `git push -u origin branch-name`                                   | Publishes your branch to GitHub                                                         |
| **Open Pull Request**               | Via GitHub UI (Compare & PR)                                        | Formal review, merge process                                                            |
| **Run Python node**                 | `ros2 run <package_name> <entry_point>`                             | Starts your ROS 2 Python code                                                          |


---

## Final Notes
- **Test** each step on a spare machine or a fresh WSL instance.
- Keep your system updated and watch for new ROS 2 releases.
- This structure (WSL2 + Ubuntu 22.04 + ROS 2 Humble) is a top recommendation by the ROS community for Windows-based dev.

Now your entire team can code in **ROS 2** using **Python**, run it in **WSL2** with **VS Code**, and manage everything with **Git + GitHub**!
