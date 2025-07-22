#!/bin/bash

# Dino Core Robot - Quick Installation Script
# One-liner: curl -sSL https://raw.githubusercontent.com/your-org/dino-core-robot/main/scripts/quick-install.sh | bash

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Logo
echo -e "${BLUE}"
cat << "EOF"
  ____  _              ____               
 |  _ \(_)_ __   ___   / ___|___  _ __ ___ 
 | | | | | '_ \ / _ \ | |   / _ \| '__/ _ \
 | |_| | | | | | (_) || |__| (_) | | |  __/
 |____/|_|_| |_|\___/  \____\___/|_|  \___|
                                          
    🤖 Robot Control System - Quick Install
EOF
echo -e "${NC}"

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   log_error "This script should NOT be run as root"
   log_info "Run as normal user: bash quick-install.sh"
   exit 1
fi

# Check internet connection
if ! ping -c 1 google.com &> /dev/null; then
    log_error "No internet connection. Please check your network."
    exit 1
fi

# Check Ubuntu version
if ! command -v lsb_release &> /dev/null; then
    log_error "This script requires Ubuntu Linux"
    exit 1
fi

UBUNTU_VERSION=$(lsb_release -rs)
if [[ $(echo "$UBUNTU_VERSION >= 20.04" | bc -l) -ne 1 ]]; then
    log_error "Ubuntu 20.04 or later required. Found: $UBUNTU_VERSION"
    exit 1
fi

log_info "Ubuntu $UBUNTU_VERSION detected - OK"

# Ask user for installation type
echo ""
echo "Please select installation type:"
echo "1) Docker (Recommended - Easy setup)"
echo "2) Native (Direct installation)"
echo "3) Development (Full development environment)"
echo ""
read -p "Enter choice [1-3]: " -n 1 -r choice
echo ""

case $choice in
    1)
        INSTALL_TYPE="docker"
        log_info "Selected: Docker installation"
        ;;
    2)
        INSTALL_TYPE="native"
        log_info "Selected: Native installation"
        ;;
    3)
        INSTALL_TYPE="development"
        log_info "Selected: Development installation"
        ;;
    *)
        log_error "Invalid choice. Exiting."
        exit 1
        ;;
esac

# Function to install Docker
install_docker() {
    log_info "Installing Docker..."
    
    # Update packages
    sudo apt update
    
    # Install Docker
    if ! command -v docker &> /dev/null; then
        curl -fsSL https://get.docker.com -o get-docker.sh
        sudo sh get-docker.sh
        sudo usermod -aG docker $USER
        rm get-docker.sh
    fi
    
    # Install Docker Compose
    if ! command -v docker-compose &> /dev/null; then
        sudo curl -L "https://github.com/docker/compose/releases/download/v2.20.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
        sudo chmod +x /usr/local/bin/docker-compose
    fi
    
    log_info "Docker installed successfully"
}

# Function to clone repository
clone_repo() {
    log_info "Cloning robot repository..."
    
    cd $HOME
    if [[ -d "dino-core-robot" ]]; then
        log_warn "Directory already exists. Updating..."
        cd dino-core-robot
        git pull
    else
        git clone https://github.com/your-org/dino-core-robot.git
        cd dino-core-robot
    fi
    
    log_info "Repository ready"
}

# Docker installation
docker_install() {
    install_docker
    clone_repo
    
    log_info "Starting Docker containers..."
    
    # Copy environment file
    if [[ ! -f .env ]]; then
        cp .env.example .env
        log_info "Created .env file - please customize if needed"
    fi
    
    # Start services
    docker-compose pull
    docker-compose up -d
    
    # Wait for services to start
    log_info "Waiting for services to start..."
    sleep 30
    
    # Check service health
    if docker-compose ps | grep -q "Up"; then
        log_info "✅ Services started successfully!"
    else
        log_error "❌ Some services failed to start"
        docker-compose logs
        exit 1
    fi
}

# Native installation
native_install() {
    clone_repo
    
    log_info "Running native installation script..."
    
    # Download and run installation script
    chmod +x scripts/install.sh
    sudo ./scripts/install.sh --production
    
    log_info "Building robot packages..."
    source /opt/ros/noetic/setup.bash
    cd ~/catkin_ws
    catkin_make
    
    log_info "Installing web interface..."
    cd ~/dino-core-robot/web-interface
    npm install
    npm run build
}

# Development installation
development_install() {
    clone_repo
    
    log_info "Running development installation script..."
    
    # Download and run installation script with development tools
    chmod +x scripts/install.sh
    sudo ./scripts/install.sh --development --docker
    
    log_info "Setting up development environment..."
    
    # Install VS Code extensions
    code --install-extension ms-python.python
    code --install-extension ms-vscode.cpptools
    code --install-extension ms-iot.vscode-ros
    
    # Setup git (optional)
    read -p "Configure Git? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        read -p "Git username: " git_username
        read -p "Git email: " git_email
        git config --global user.name "$git_username"
        git config --global user.email "$git_email"
        log_info "Git configured"
    fi
}

# Get local IP
get_local_ip() {
    hostname -I | awk '{print $1}'
}

# Show installation results
show_results() {
    echo ""
    echo -e "${GREEN}"
    cat << "EOF"
╔═══════════════════════════════════════════════════╗
║              🎉 Installation Complete! 🎉         ║
╚═══════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
    
    LOCAL_IP=$(get_local_ip)
    
    echo "🌐 Access your robot system:"
    echo "   Local:  http://localhost:8080"
    echo "   Remote: http://$LOCAL_IP:8080"
    echo ""
    echo "📱 Quick Links:"
    echo "   • Dashboard:     http://$LOCAL_IP:8080/"
    echo "   • API Docs:      http://$LOCAL_IP:8080/api-management"
    echo "   • Monitoring:    http://$LOCAL_IP:8080/system-monitoring"
    echo "   • Terminal:      http://$LOCAL_IP:8080/terminal"
    echo ""
    
    if [[ "$INSTALL_TYPE" == "docker" ]]; then
        echo "🐳 Docker Commands:"
        echo "   docker-compose ps              # Check status"
        echo "   docker-compose logs -f         # View logs"
        echo "   docker-compose restart         # Restart services"
        echo "   docker-compose down            # Stop services"
        echo ""
    elif [[ "$INSTALL_TYPE" == "native" ]]; then
        echo "🔧 System Commands:"
        echo "   robot-status                   # Check system status"
        echo "   sudo systemctl status robot-control  # Service status"
        echo "   start-robot                    # Start robot system"
        echo "   stop-robot                     # Stop robot system"
        echo ""
    fi
    
    echo "📚 Documentation:"
    echo "   README.md                      # Main documentation"
    echo "   docs/                          # Detailed guides"
    echo "   INSTALL.md                     # Installation options"
    echo ""
    
    echo "🆘 Need Help?"
    echo "   • GitHub Issues: https://github.com/your-org/dino-core-robot/issues"
    echo "   • Documentation: https://docs.robotcompany.com"
    echo "   • Community: https://discord.gg/robot-community"
    echo ""
    
    if [[ "$INSTALL_TYPE" == "docker" ]]; then
        log_info "Note: You may need to log out and back in for Docker permissions to take effect"
    fi
    
    log_info "Installation completed successfully! 🤖"
}

# Main installation flow
main() {
    case $INSTALL_TYPE in
        "docker")
            docker_install
            ;;
        "native")
            native_install
            ;;
        "development")
            development_install
            ;;
    esac
    
    show_results
}

# Check prerequisites
check_prerequisites() {
    log_info "Checking prerequisites..."
    
    # Check for required commands
    for cmd in curl wget git; do
        if ! command -v $cmd &> /dev/null; then
            log_error "$cmd is required but not installed"
            log_info "Install with: sudo apt install $cmd"
            exit 1
        fi
    done
    
    # Check for sudo access
    if ! sudo -n true 2>/dev/null; then
        log_info "This script requires sudo access. You may be prompted for your password."
        sudo true || {
            log_error "Sudo access required"
            exit 1
        }
    fi
    
    log_info "Prerequisites check passed"
}

# Trap errors
trap 'log_error "Installation failed. Check the output above for details."' ERR

# Run installation
log_info "Starting Dino Core Robot Quick Installation..."
check_prerequisites
main

exit 0
