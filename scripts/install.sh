#!/bin/bash

# Dino Core Robot Control System - Installation Script
# Usage: sudo ./install.sh [--production] [--docker] [--development]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
ROS_DISTRO="noetic"
UBUNTU_VERSION="focal"
PYTHON_VERSION="3.8"
NODE_VERSION="16"
ROBOT_USER="robot"
ROBOT_HOME="/opt/robot"
WEB_PORT="8080"
WS_PORT="9090"
ROS_PORT="11311"

# Parse command line arguments
PRODUCTION=false
DOCKER_INSTALL=false
DEVELOPMENT=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --production)
            PRODUCTION=true
            shift
            ;;
        --docker)
            DOCKER_INSTALL=true
            shift
            ;;
        --development)
            DEVELOPMENT=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--production] [--docker] [--development]"
            echo "  --production    Install for production use"
            echo "  --docker        Install Docker and Docker Compose"
            echo "  --development   Install development tools"
            exit 0
            ;;
        *)
            echo "Unknown option $1"
            exit 1
            ;;
    esac
done

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo -e "${RED}This script must be run as root (use sudo)${NC}"
   exit 1
fi

# Get the actual user (in case of sudo)
ACTUAL_USER=${SUDO_USER:-$USER}
ACTUAL_HOME=$(eval echo ~$ACTUAL_USER)

# Logo and welcome message
echo -e "${BLUE}"
cat << "EOF"
╔═══════════════════════════════════════════════════╗
║                                                   ║
║        🤖 Dino Core Robot Control System          ║
║              Installation Script                  ║
║                                                   ║
╚═══════════════════════════════════════════════════╝
EOF
echo -e "${NC}"

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Check Ubuntu version
check_ubuntu_version() {
    log_step "Checking Ubuntu version..."
    
    if [[ ! -f /etc/os-release ]]; then
        log_error "Cannot determine OS version"
        exit 1
    fi
    
    source /etc/os-release
    
    if [[ "$ID" != "ubuntu" ]]; then
        log_error "This script is designed for Ubuntu. Detected: $ID"
        exit 1
    fi
    
    if [[ "$VERSION_CODENAME" != "focal" ]] && [[ "$VERSION_CODENAME" != "jammy" ]]; then
        log_warn "Untested Ubuntu version: $VERSION_CODENAME"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
    
    log_info "Ubuntu $VERSION_ID ($VERSION_CODENAME) detected"
}

# Update system
update_system() {
    log_step "Updating system packages..."
    apt update && apt upgrade -y
    apt install -y curl wget gnupg2 lsb-release software-properties-common apt-transport-https ca-certificates
    log_info "System updated successfully"
}

# Install ROS
install_ros() {
    log_step "Installing ROS $ROS_DISTRO..."
    
    # Check if ROS is already installed
    if command -v roscore &> /dev/null; then
        log_info "ROS is already installed"
        return
    fi
    
    # Add ROS repository
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    # Add ROS key
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    
    # Update and install ROS
    apt update
    apt install -y ros-$ROS_DISTRO-desktop-full
    
    # Initialize rosdep
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        rosdep init
    fi
    
    # Update rosdep as normal user
    sudo -u $ACTUAL_USER rosdep update
    
    # Install additional ROS packages
    apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    apt install -y ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-move-base
    apt install -y ros-$ROS_DISTRO-cv-camera ros-$ROS_DISTRO-rplidar-ros ros-$ROS_DISTRO-joy
    apt install -y ros-$ROS_DISTRO-teleop-twist-joy ros-$ROS_DISTRO-robot-state-publisher
    apt install -y ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-xacro
    
    log_info "ROS $ROS_DISTRO installed successfully"
}

# Install Python dependencies
install_python_deps() {
    log_step "Installing Python dependencies..."
    
    # Install pip if not present
    apt install -y python3-pip python3-dev python3-venv
    
    # Install system Python packages
    apt install -y python3-opencv python3-numpy python3-yaml python3-serial
    
    # Install additional Python packages via pip
    pip3 install --upgrade pip
    pip3 install websockets psutil asyncio aiohttp
    pip3 install pillow requests flask flask-cors
    pip3 install prometheus-client psycopg2-binary redis
    
    log_info "Python dependencies installed successfully"
}

# Install Node.js
install_nodejs() {
    log_step "Installing Node.js $NODE_VERSION..."
    
    # Check if Node.js is already installed
    if command -v node &> /dev/null; then
        NODE_VER=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
        if [[ $NODE_VER -ge $NODE_VERSION ]]; then
            log_info "Node.js $NODE_VER is already installed"
            return
        fi
    fi
    
    # Install Node.js
    curl -fsSL https://deb.nodesource.com/setup_${NODE_VERSION}.x | bash -
    apt install -y nodejs
    
    # Install global packages
    npm install -g npm@latest
    npm install -g yarn pm2
    
    log_info "Node.js $(node --version) installed successfully"
}

# Install Docker (optional)
install_docker() {
    if [[ "$DOCKER_INSTALL" != true ]]; then
        return
    fi
    
    log_step "Installing Docker and Docker Compose..."
    
    # Check if Docker is already installed
    if command -v docker &> /dev/null; then
        log_info "Docker is already installed"
    else
        # Install Docker
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
        add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
        apt update
        apt install -y docker-ce docker-ce-cli containerd.io
        
        # Add user to docker group
        usermod -aG docker $ACTUAL_USER
    fi
    
    # Install Docker Compose
    if ! command -v docker-compose &> /dev/null; then
        curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
        chmod +x /usr/local/bin/docker-compose
    fi
    
    # Start Docker service
    systemctl enable docker
    systemctl start docker
    
    log_info "Docker and Docker Compose installed successfully"
}

# Create robot user
create_robot_user() {
    log_step "Creating robot user and directories..."
    
    # Create robot user if not exists
    if ! id "$ROBOT_USER" &>/dev/null; then
        useradd -m -s /bin/bash $ROBOT_USER
        usermod -aG sudo,dialout,video,audio,plugdev $ROBOT_USER
        log_info "Created user: $ROBOT_USER"
    else
        log_info "User $ROBOT_USER already exists"
    fi
    
    # Create robot directories
    mkdir -p $ROBOT_HOME/{config,logs,maps,scripts,backups}
    mkdir -p /etc/robot
    
    # Set permissions
    chown -R $ROBOT_USER:$ROBOT_USER $ROBOT_HOME
    chmod -R 755 $ROBOT_HOME
    
    log_info "Robot directories created"
}

# Setup catkin workspace
setup_catkin_workspace() {
    log_step "Setting up catkin workspace..."
    
    CATKIN_WS="$ACTUAL_HOME/catkin_ws"
    
    # Create workspace as actual user
    sudo -u $ACTUAL_USER bash << EOF
    source /opt/ros/$ROS_DISTRO/setup.bash
    mkdir -p $CATKIN_WS/src
    cd $CATKIN_WS
    catkin_make
    
    # Add to bashrc
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $ACTUAL_HOME/.bashrc
    echo "source $CATKIN_WS/devel/setup.bash" >> $ACTUAL_HOME/.bashrc
    echo "export ROS_MASTER_URI=http://localhost:$ROS_PORT" >> $ACTUAL_HOME/.bashrc
    echo "export ROS_HOSTNAME=localhost" >> $ACTUAL_HOME/.bashrc
EOF
    
    log_info "Catkin workspace created at $CATKIN_WS"
}

# Clone robot repository
clone_repository() {
    log_step "Cloning robot repository..."
    
    REPO_URL="https://github.com/your-org/dino-core-robot.git"
    CATKIN_WS="$ACTUAL_HOME/catkin_ws"
    
    # Clone as actual user
    sudo -u $ACTUAL_USER bash << EOF
    cd $CATKIN_WS/src
    if [[ ! -d "dino-core-robot" ]]; then
        git clone $REPO_URL
        log_info "Repository cloned successfully"
    else
        log_info "Repository already exists"
    fi
EOF
}

# Build robot packages
build_packages() {
    log_step "Building robot packages..."
    
    CATKIN_WS="$ACTUAL_HOME/catkin_ws"
    
    # Build as actual user
    sudo -u $ACTUAL_USER bash << EOF
    source /opt/ros/$ROS_DISTRO/setup.bash
    cd $CATKIN_WS
    catkin_make
    source devel/setup.bash
EOF
    
    log_info "Robot packages built successfully"
}

# Install web interface
install_web_interface() {
    log_step "Installing web interface..."
    
    WEB_DIR="$ACTUAL_HOME/catkin_ws/src/dino-core-robot/web-interface"
    
    if [[ ! -d "$WEB_DIR" ]]; then
        log_warn "Web interface directory not found, skipping..."
        return
    fi
    
    # Install dependencies and build as actual user
    sudo -u $ACTUAL_USER bash << EOF
    cd $WEB_DIR
    npm install
    
    if [[ "$PRODUCTION" == true ]]; then
        npm run build
    fi
EOF
    
    log_info "Web interface installed successfully"
}

# Configure system services
configure_services() {
    if [[ "$PRODUCTION" != true ]]; then
        log_info "Skipping service configuration (not production mode)"
        return
    fi
    
    log_step "Configuring system services..."
    
    # Create systemd service file
    cat > /etc/systemd/system/robot-control.service << EOF
[Unit]
Description=Dino Core Robot Control System
After=network.target sound.target
Wants=network.target

[Service]
Type=forking
User=$ROBOT_USER
Group=$ROBOT_USER
WorkingDirectory=$ROBOT_HOME

Environment=ROS_MASTER_URI=http://localhost:$ROS_PORT
Environment=ROS_HOSTNAME=localhost
Environment=ROS_IP=127.0.0.1
Environment=ROS_DISTRO=$ROS_DISTRO
Environment=PYTHONPATH=/opt/ros/$ROS_DISTRO/lib/python3/dist-packages:$ACTUAL_HOME/catkin_ws/devel/lib/python3/dist-packages

ExecStartPre=/bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && source $ACTUAL_HOME/catkin_ws/devel/setup.bash'
ExecStart=/bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && source $ACTUAL_HOME/catkin_ws/devel/setup.bash && roslaunch robot_control complete_robot.launch'
ExecStopPost=/bin/bash -c 'pkill -f ros'

Restart=always
RestartSec=10
StartLimitInterval=0

LimitNOFILE=65536
LimitNPROC=32768

NoNewPrivileges=yes
PrivateTmp=yes
ProtectSystem=strict
ProtectHome=yes
ReadWritePaths=$ROBOT_HOME $ACTUAL_HOME /tmp /var/log
DeviceAllow=/dev/ttyUSB* rw
DeviceAllow=/dev/ttyACM* rw
DeviceAllow=/dev/video* rw
DeviceAllow=/dev/input* rw

StandardOutput=journal
StandardError=journal
SyslogIdentifier=robot-control

[Install]
WantedBy=multi-user.target
EOF
    
    # Reload systemd and enable service
    systemctl daemon-reload
    systemctl enable robot-control.service
    
    log_info "System service configured"
}

# Configure firewall
configure_firewall() {
    log_step "Configuring firewall..."
    
    # Install and configure UFW
    apt install -y ufw
    
    # Set default policies
    ufw --force reset
    ufw default deny incoming
    ufw default allow outgoing
    
    # Allow essential services
    ufw allow 22      # SSH
    ufw allow $WEB_PORT    # Web Interface
    ufw allow $WS_PORT     # WebSocket
    ufw allow $ROS_PORT    # ROS Master
    
    # Enable firewall
    ufw --force enable
    
    log_info "Firewall configured"
}

# Configure network (production only)
configure_network() {
    if [[ "$PRODUCTION" != true ]]; then
        return
    fi
    
    log_step "Configuring network for production..."
    
    # Create network configuration template
    cat > /etc/robot/network-template.yaml << EOF
# Network Configuration Template
# Copy to /etc/netplan/99-robot-network.yaml and modify as needed

network:
  version: 2
  ethernets:
    eth0:
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
  wifis:
    wlan0:
      access-points:
        "ROBOT_WIFI":
          password: "your_wifi_password"
      addresses: [192.168.1.101/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
EOF
    
    log_info "Network configuration template created at /etc/robot/network-template.yaml"
    log_warn "Please configure your network settings manually using the template"
}

# Install development tools
install_dev_tools() {
    if [[ "$DEVELOPMENT" != true ]]; then
        return
    fi
    
    log_step "Installing development tools..."
    
    # Install development packages
    apt install -y git vim nano htop iotop tree curl wget
    apt install -y build-essential cmake pkg-config
    apt install -y python3-dev python3-pip python3-venv
    apt install -y qt5-default libqt5svg5-dev
    
    # Install VS Code
    if ! command -v code &> /dev/null; then
        wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
        install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
        echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list
        apt update
        apt install -y code
    fi
    
    # Install useful VS Code extensions
    sudo -u $ACTUAL_USER code --install-extension ms-python.python
    sudo -u $ACTUAL_USER code --install-extension ms-vscode.cpptools
    sudo -u $ACTUAL_USER code --install-extension ms-iot.vscode-ros
    
    log_info "Development tools installed"
}

# Create useful scripts
create_scripts() {
    log_step "Creating utility scripts..."
    
    # Robot start script
    cat > $ROBOT_HOME/scripts/start-robot.sh << 'EOF'
#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch robot_control complete_robot.launch
EOF
    
    # Robot stop script
    cat > $ROBOT_HOME/scripts/stop-robot.sh << 'EOF'
#!/bin/bash
pkill -f ros
echo "Robot stopped"
EOF
    
    # System status script
    cat > $ROBOT_HOME/scripts/robot-status.sh << 'EOF'
#!/bin/bash
echo "=== Robot System Status ==="
echo "Date: $(date)"
echo "Uptime: $(uptime -p)"
echo ""
echo "=== System Resources ==="
echo "CPU: $(top -bn1 | grep load | awk '{printf "%.2f%%", $(NF-2)}')"
echo "Memory: $(free | grep Mem | awk '{printf("%.2f%%", $3/$2 * 100.0)}')"
echo "Disk: $(df -h / | awk 'NR==2{printf "%s", $5}')"
echo ""
echo "=== ROS Status ==="
if pgrep -f rosmaster > /dev/null; then
    echo "ROS Master: Running"
    echo "Active nodes: $(rosnode list 2>/dev/null | wc -l)"
    echo "Active topics: $(rostopic list 2>/dev/null | wc -l)"
else
    echo "ROS Master: Not running"
fi
echo ""
echo "=== Service Status ==="
systemctl is-active --quiet robot-control.service && echo "Robot Service: Active" || echo "Robot Service: Inactive"
echo ""
echo "=== Network ==="
echo "IP Address: $(hostname -I | awk '{print $1}')"
echo "Web Interface: http://$(hostname -I | awk '{print $1}'):8080"
EOF
    
    # Make scripts executable
    chmod +x $ROBOT_HOME/scripts/*.sh
    
    # Create symlinks for easy access
    ln -sf $ROBOT_HOME/scripts/robot-status.sh /usr/local/bin/robot-status
    ln -sf $ROBOT_HOME/scripts/start-robot.sh /usr/local/bin/start-robot
    ln -sf $ROBOT_HOME/scripts/stop-robot.sh /usr/local/bin/stop-robot
    
    log_info "Utility scripts created"
}

# Post-installation setup
post_install_setup() {
    log_step "Performing post-installation setup..."
    
    # Fix permissions for devices
    cat > /etc/udev/rules.d/99-robot-devices.rules << EOF
# Robot device permissions
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"
SUBSYSTEM=="usb", ATTRS{idVendor}=="046d", MODE="0666", GROUP="video"
EOF
    
    # Reload udev rules
    udevadm control --reload-rules
    udevadm trigger
    
    # Create log rotation configuration
    cat > /etc/logrotate.d/robot-logs << EOF
$ROBOT_HOME/logs/*.log {
    weekly
    rotate 4
    compress
    delaycompress
    missingok
    notifempty
    create 644 $ROBOT_USER $ROBOT_USER
}
EOF
    
    # Configure sudo for robot user
    echo "$ROBOT_USER ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart robot-control.service" > /etc/sudoers.d/robot
    echo "$ROBOT_USER ALL=(ALL) NOPASSWD: /usr/bin/systemctl start robot-control.service" >> /etc/sudoers.d/robot
    echo "$ROBOT_USER ALL=(ALL) NOPASSWD: /usr/bin/systemctl stop robot-control.service" >> /etc/sudoers.d/robot
    echo "$ROBOT_USER ALL=(ALL) NOPASSWD: /usr/bin/systemctl status robot-control.service" >> /etc/sudoers.d/robot
    
    log_info "Post-installation setup completed"
}

# Installation summary
show_summary() {
    log_step "Installation Summary"
    
    echo -e "${GREEN}"
    cat << EOF

╔═══════════════════════════════════════════════════╗
║             🎉 Installation Complete! 🎉          ║
╠═══════════════════════════════════════════════════╣
║                                                   ║
║  Dino Core Robot Control System has been         ║
║  installed successfully!                          ║
║                                                   ║
╚═══════════════════════════════════════════════════╝

EOF
    echo -e "${NC}"
    
    echo "📋 Installation Details:"
    echo "  - ROS $ROS_DISTRO installed"
    echo "  - Python dependencies installed"
    echo "  - Node.js $(node --version) installed"
    echo "  - Catkin workspace: $ACTUAL_HOME/catkin_ws"
    echo "  - Robot directory: $ROBOT_HOME"
    
    if [[ "$DOCKER_INSTALL" == true ]]; then
        echo "  - Docker and Docker Compose installed"
    fi
    
    if [[ "$DEVELOPMENT" == true ]]; then
        echo "  - Development tools installed"
    fi
    
    echo ""
    echo "🔧 Quick Commands:"
    echo "  robot-status    - Check system status"
    echo "  start-robot     - Start robot system"
    echo "  stop-robot      - Stop robot system"
    
    if [[ "$PRODUCTION" == true ]]; then
        echo ""
        echo "🚀 Production Setup:"
        echo "  sudo systemctl start robot-control.service"
        echo "  sudo systemctl enable robot-control.service"
    fi
    
    echo ""
    echo "🌐 Access Points:"
    echo "  Web Interface: http://localhost:$WEB_PORT"
    echo "  API Docs:      http://localhost:$WEB_PORT/api-management"
    echo "  System Monitor: http://localhost:$WEB_PORT/system-monitoring"
    
    echo ""
    echo "📚 Documentation:"
    echo "  README.md      - Main documentation"
    echo "  INSTALL.md     - Installation guide"
    echo "  docs/          - Detailed documentation"
    
    echo ""
    log_info "Installation completed successfully!"
    
    if [[ "$PRODUCTION" == true ]]; then
        log_warn "Please reboot the system to ensure all services start correctly"
        log_warn "Configure network settings using template at /etc/robot/network-template.yaml"
    fi
    
    echo ""
    log_info "Happy robotics! 🤖"
}

# Main installation flow
main() {
    log_info "Starting Dino Core Robot Control System installation..."
    log_info "Mode: $(if [[ "$PRODUCTION" == true ]]; then echo "Production"; elif [[ "$DEVELOPMENT" == true ]]; then echo "Development"; else echo "Standard"; fi)"
    
    check_ubuntu_version
    update_system
    install_ros
    install_python_deps
    install_nodejs
    install_docker
    create_robot_user
    setup_catkin_workspace
    clone_repository
    build_packages
    install_web_interface
    configure_services
    configure_firewall
    configure_network
    install_dev_tools
    create_scripts
    post_install_setup
    show_summary
}

# Trap errors
trap 'log_error "Installation failed at line $LINENO. Check the logs above."' ERR

# Run main installation
main "$@"

exit 0
