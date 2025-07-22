# Production Deployment Guide

> คู่มือการ deploy ระบบหุ่นยนต์สำหรับการใช้งานจริง

## 🎯 ภาพรวมการ Deployment

เอกสารนี้จะแนะนำการ deploy ระบบ Dino Core Robot Control System ในสภาพแวดล้อม production ที่แตกต่างกัน

## 🏗️ Architecture Overview

```
Internet/LAN
     │
┌────▼─────┐    ┌─────────────┐    ┌─────────────┐
│   Nginx  │────│  Web App    │────│  ROS Core   │
│  (Port   │    │  (Port      │    │  (Port      │
│   80/443)│    │   8080)     │    │   11311)    │
└──────────┘    └─────────────┘    └─────────────┘
     │               │                   │
┌────▼─────┐    ┌────▼─────┐    ┌────▼─────┐
│   SSL    │    │WebSocket │    │ Hardware │
│   Cert   │    │ (Port    │    │Interface │
│          │    │  9090)   │    │          │
└──────────┘    └──────────┘    └──────────┘
```

## 🚀 Deployment Options

### Option 1: Single Robot Deployment (แนะนำสำหรับ 1 หุ่นยนต์)

```bash
# 1. ติดตั้งบน hardware ของหุ่นยนต์โดยตรง
# 2. ใช้ systemd สำหรับ auto-start
# 3. เข้าถึงผ่าน robot IP
```

### Option 2: Docker Deployment (แนะนำสำหรับ development/testing)

```bash
# 1. ใช้ Docker Compose
# 2. แยก services เป็น containers
# 3. Easy scaling และ management
```

### Option 3: Kubernetes Deployment (สำหรับ fleet management)

```bash
# 1. ใช้ K8s สำหรับ multiple robots
# 2. Central management
# 3. Load balancing และ monitoring
```

## 🔧 Single Robot Deployment

### Prerequisites

```bash
# Hardware Requirements
- Jetson Xavier NX หรือ Raspberry Pi 4 (8GB RAM)
- MicroSD Card 64GB+ (Class 10)
- Ethernet connection
- USB ports สำหรับ sensors

# Software Requirements
- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+
- Node.js 16+
```

### Step 1: OS Installation

```bash
# 1. Flash Ubuntu 20.04 ไปยัง SD Card
# ใช้ Raspberry Pi Imager หรือ Etcher

# 2. Enable SSH และ configure network
sudo systemctl enable ssh
sudo systemctl start ssh

# 3. Update system
sudo apt update && sudo apt upgrade -y
```

### Step 2: Install Dependencies

```bash
# ใช้ installation script
wget https://raw.githubusercontent.com/your-org/dino-core-robot/main/scripts/install.sh
chmod +x install.sh
sudo ./install.sh
```

หรือติดตั้งแบบ manual:

```bash
# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# Install robot packages
sudo apt install ros-noetic-navigation ros-noetic-slam-gmapping -y
sudo apt install ros-noetic-cv-camera ros-noetic-rplidar-ros -y

# Install Python dependencies
pip3 install opencv-python numpy websockets psutil pyserial pyyaml

# Install Node.js
curl -fsSL https://deb.nodesource.com/setup_16.x | sudo -E bash -
sudo apt-get install -y nodejs
```

### Step 3: Deploy Application

```bash
# 1. Clone repository
cd /opt
sudo git clone https://github.com/your-org/dino-core-robot.git robot
sudo chown -R $USER:$USER /opt/robot

# 2. Build ROS workspace
cd /opt/robot/ros_workspace
catkin_make
source devel/setup.bash

# 3. Build web interface
cd /opt/robot/web-interface
npm install
npm run build

# 4. Copy configuration files
sudo cp /opt/robot/config/robot_config.yaml /etc/robot/
sudo cp /opt/robot/scripts/robot-control.service /etc/systemd/system/
```

### Step 4: Configure Services

```bash
# 1. Configure systemd service
sudo nano /etc/systemd/system/robot-control.service
```

```ini
[Unit]
Description=Dino Core Robot Control System
After=network.target
Wants=network.target

[Service]
Type=forking
User=robot
Group=robot
WorkingDirectory=/opt/robot

Environment=ROS_MASTER_URI=http://localhost:11311
Environment=ROS_HOSTNAME=localhost
Environment=PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages

ExecStartPre=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /opt/robot/ros_workspace/devel/setup.bash'
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /opt/robot/ros_workspace/devel/setup.bash && roslaunch robot_control complete_robot.launch'
ExecStopPost=/bin/bash -c 'pkill -f ros'

Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

```bash
# 2. Enable และ start service
sudo systemctl daemon-reload
sudo systemctl enable robot-control.service
sudo systemctl start robot-control.service

# 3. Check status
sudo systemctl status robot-control.service
```

### Step 5: Configure Network

```bash
# 1. Set static IP
sudo nano /etc/netplan/01-network-manager-all.yaml
```

```yaml
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
          password: "your_password"
      addresses: [192.168.1.101/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

```bash
# 2. Apply network configuration
sudo netplan apply

# 3. Configure firewall
sudo ufw enable
sudo ufw allow 22      # SSH
sudo ufw allow 8080    # Web Interface
sudo ufw allow 9090    # WebSocket
sudo ufw allow 11311   # ROS Master
```

### Step 6: SSL Configuration (Optional)

```bash
# 1. Install certbot
sudo apt install certbot -y

# 2. Generate SSL certificate
sudo certbot certonly --standalone -d robot.yourdomain.com

# 3. Configure nginx
sudo apt install nginx -y
sudo nano /etc/nginx/sites-available/robot
```

```nginx
server {
    listen 80;
    server_name robot.yourdomain.com;
    return 301 https://$server_name$request_uri;
}

server {
    listen 443 ssl;
    server_name robot.yourdomain.com;

    ssl_certificate /etc/letsencrypt/live/robot.yourdomain.com/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/robot.yourdomain.com/privkey.pem;

    location / {
        proxy_pass http://localhost:8080;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection 'upgrade';
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        proxy_cache_bypass $http_upgrade;
    }

    location /ws {
        proxy_pass http://localhost:9090;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

```bash
# 4. Enable site และ restart nginx
sudo ln -s /etc/nginx/sites-available/robot /etc/nginx/sites-enabled/
sudo systemctl restart nginx
```

## 🐳 Docker Deployment

### Docker Compose Setup

```bash
# 1. Clone repository
git clone https://github.com/your-org/dino-core-robot.git
cd dino-core-robot

# 2. Copy และ edit configuration
cp docker-compose.prod.yml docker-compose.yml
nano docker-compose.yml
```

```yaml
version: '3.8'

services:
  # ROS Master
  roscore:
    image: ros:noetic-robot
    container_name: robot-roscore
    command: roscore
    networks:
      - robot-network
    ports:
      - "11311:11311"
    environment:
      - ROS_MASTER_URI=http://roscore:11311
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "rostopic", "list"]
      interval: 30s
      timeout: 10s
      retries: 3

  # Robot Control System
  robot-control:
    build:
      context: .
      dockerfile: Dockerfile.robot
    container_name: robot-control-system
    depends_on:
      - roscore
      - database
      - redis
    networks:
      - robot-network
    ports:
      - "9090:9090"
    environment:
      - ROS_MASTER_URI=http://roscore:11311
      - ROS_HOSTNAME=robot-control
      - DATABASE_URL=postgresql://robot:robot123@database:5432/robotdb
      - REDIS_URL=redis://redis:6379
    volumes:
      - ./config:/opt/robot/config:ro
      - ./logs:/opt/robot/logs:rw
      - ./maps:/opt/robot/maps:rw
      - /dev:/dev
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
      - /dev/ttyACM0:/dev/ttyACM0
      - /dev/video0:/dev/video0
    privileged: true
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "python3", "-c", "import requests; requests.get('http://localhost:9090/health')"]
      interval: 30s
      timeout: 10s
      retries: 3

  # Web Interface
  web-interface:
    build:
      context: ./web-interface
      dockerfile: Dockerfile
    container_name: robot-web-interface
    networks:
      - robot-network
    ports:
      - "8080:80"
    depends_on:
      - robot-control
    environment:
      - API_BASE_URL=http://robot-control:9090
      - WS_BASE_URL=ws://robot-control:9090
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:80"]
      interval: 30s
      timeout: 10s
      retries: 3

  # Database
  database:
    image: postgres:14
    container_name: robot-database
    networks:
      - robot-network
    environment:
      - POSTGRES_DB=robotdb
      - POSTGRES_USER=robot
      - POSTGRES_PASSWORD=robot123
    volumes:
      - postgres_data:/var/lib/postgresql/data
      - ./sql/init.sql:/docker-entrypoint-initdb.d/init.sql:ro
    ports:
      - "5432:5432"
    restart: unless-stopped
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U robot"]
      interval: 30s
      timeout: 5s
      retries: 5

  # Redis Cache
  redis:
    image: redis:7-alpine
    container_name: robot-redis
    networks:
      - robot-network
    ports:
      - "6379:6379"
    volumes:
      - redis_data:/data
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "redis-cli", "ping"]
      interval: 30s
      timeout: 5s
      retries: 3

  # Monitoring
  prometheus:
    image: prom/prometheus:latest
    container_name: robot-prometheus
    networks:
      - robot-network
    ports:
      - "9091:9090"
    volumes:
      - ./monitoring/prometheus.yml:/etc/prometheus/prometheus.yml:ro
      - prometheus_data:/prometheus
    command:
      - '--config.file=/etc/prometheus/prometheus.yml'
      - '--storage.tsdb.path=/prometheus'
      - '--web.console.libraries=/etc/prometheus/console_libraries'
      - '--web.console.templates=/etc/prometheus/consoles'
      - '--web.enable-lifecycle'
    restart: unless-stopped

  grafana:
    image: grafana/grafana:latest
    container_name: robot-grafana
    networks:
      - robot-network
    ports:
      - "3000:3000"
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=admin123
    volumes:
      - grafana_data:/var/lib/grafana
      - ./monitoring/grafana-datasources.yml:/etc/grafana/provisioning/datasources/datasources.yml:ro
      - ./monitoring/dashboards:/var/lib/grafana/dashboards:ro
    restart: unless-stopped

  # Log Aggregation
  fluentd:
    build:
      context: ./logging
      dockerfile: Dockerfile.fluentd
    container_name: robot-fluentd
    networks:
      - robot-network
    ports:
      - "24224:24224"
    volumes:
      - ./logging/fluent.conf:/fluentd/etc/fluent.conf:ro
      - ./logs:/var/log/robot:rw
    restart: unless-stopped

networks:
  robot-network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16

volumes:
  postgres_data:
  redis_data:
  prometheus_data:
  grafana_data:
```

### Deployment Commands

```bash
# 1. Build และ deploy
docker-compose build
docker-compose up -d

# 2. ตรวจสอบ status
docker-compose ps
docker-compose logs -f robot-control

# 3. Scale services (ถ้าต้องการ)
docker-compose scale robot-control=2

# 4. Update services
docker-compose pull
docker-compose up -d

# 5. Backup data
docker-compose exec database pg_dump -U robot robotdb > backup.sql

# 6. Restore data
cat backup.sql | docker-compose exec -T database psql -U robot robotdb
```

## ☸️ Kubernetes Deployment

### Prerequisites

```bash
# ติดตั้ง kubectl และ helm
curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl"
sudo install -o root -g root -m 0755 kubectl /usr/local/bin/kubectl

# ติดตั้ง Helm
curl https://raw.githubusercontent.com/helm/helm/main/scripts/get-helm-3 | bash
```

### Kubernetes Manifests

```yaml
# k8s/namespace.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: robot-system
---
# k8s/configmap.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: robot-config
  namespace: robot-system
data:
  robot_config.yaml: |
    robot:
      max_linear_speed: 1.0
      max_angular_speed: 2.0
    network:
      ros_master_uri: "http://roscore:11311"
    safety:
      min_obstacle_distance: 0.3
---
# k8s/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: robot-control
  namespace: robot-system
spec:
  replicas: 1
  selector:
    matchLabels:
      app: robot-control
  template:
    metadata:
      labels:
        app: robot-control
    spec:
      containers:
      - name: robot-control
        image: your-registry/robot-control:latest
        ports:
        - containerPort: 9090
        env:
        - name: ROS_MASTER_URI
          value: "http://roscore:11311"
        volumeMounts:
        - name: config
          mountPath: /opt/robot/config
        - name: dev
          mountPath: /dev
        securityContext:
          privileged: true
        resources:
          requests:
            memory: "512Mi"
            cpu: "500m"
          limits:
            memory: "2Gi"
            cpu: "2000m"
        livenessProbe:
          httpGet:
            path: /health
            port: 9090
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 9090
          initialDelaySeconds: 5
          periodSeconds: 5
      volumes:
      - name: config
        configMap:
          name: robot-config
      - name: dev
        hostPath:
          path: /dev
---
# k8s/service.yaml
apiVersion: v1
kind: Service
metadata:
  name: robot-control-service
  namespace: robot-system
spec:
  selector:
    app: robot-control
  ports:
  - port: 9090
    targetPort: 9090
    protocol: TCP
  type: LoadBalancer
```

### Helm Chart Deployment

```bash
# 1. สร้าง Helm chart
helm create robot-system

# 2. Deploy
helm install robot-system ./robot-system \
  --namespace robot-system \
  --create-namespace \
  --set image.tag=latest \
  --set database.password=robot123

# 3. Upgrade
helm upgrade robot-system ./robot-system \
  --namespace robot-system \
  --set image.tag=v2.0.1

# 4. Rollback
helm rollback robot-system 1 --namespace robot-system

# 5. Uninstall
helm uninstall robot-system --namespace robot-system
```

## 📊 Monitoring & Logging

### 1. System Monitoring

```bash
# Prometheus metrics
# Available at http://robot-ip:9091

# Grafana dashboards
# Available at http://robot-ip:3000
# Login: admin/admin123

# Key metrics to monitor:
- CPU และ Memory usage
- Network bandwidth
- Disk I/O
- ROS node status
- API response times
- WebSocket connections
```

### 2. Application Logs

```bash
# ROS logs
tail -f ~/.ros/log/latest/rosout.log

# Application logs
tail -f /opt/robot/logs/robot-control.log

# System logs
journalctl -f -u robot-control.service

# Docker logs
docker-compose logs -f robot-control
```

### 3. Health Checks

```bash
# API health check
curl http://robot-ip:8080/api/health

# ROS health check
rostopic echo /robot_status

# Database health check
curl http://robot-ip:8080/api/system/health

# WebSocket connection test
wscat -c ws://robot-ip:9090
```

## 🔒 Security Considerations

### 1. Network Security

```bash
# Configure firewall
sudo ufw enable
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow 22
sudo ufw allow 8080
sudo ufw allow 9090
sudo ufw allow from 192.168.1.0/24 to any port 11311

# Use VPN for remote access
sudo apt install openvpn
# Configure OpenVPN client
```

### 2. Application Security

```bash
# Enable HTTPS
# Use SSL certificates (Let's Encrypt)

# API authentication
# Implement JWT tokens

# Rate limiting
# Configure nginx rate limiting

# Input validation
# Validate all API inputs

# Regular security updates
sudo apt update && sudo apt upgrade
```

### 3. Container Security

```bash
# Use minimal base images
FROM ubuntu:20.04-slim

# Run as non-root user
USER robot

# Scan images for vulnerabilities
docker scan your-registry/robot-control:latest

# Use secrets management
kubectl create secret generic robot-secrets \
  --from-literal=db-password=robot123 \
  --namespace robot-system
```

## 📈 Performance Optimization

### 1. System Optimization

```bash
# CPU governor
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Swappiness
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf

# Increase file limits
echo 'robot soft nofile 65536' | sudo tee -a /etc/security/limits.conf
echo 'robot hard nofile 65536' | sudo tee -a /etc/security/limits.conf

# Network optimization
echo 'net.core.rmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' | sudo tee -a /etc/sysctl.conf
```

### 2. Application Optimization

```bash
# ROS node optimization
# Use nodelet for intensive processing
# Optimize topic publishing rates
# Use appropriate queue sizes

# Database optimization
# Use connection pooling
# Optimize queries
# Regular VACUUM operations

# Cache optimization
# Use Redis for frequently accessed data
# Implement proper cache invalidation
```

### 3. Monitoring Performance

```bash
# System resources
htop
iotop
nethogs

# ROS performance
rostopic hz /camera/image_raw
rostopic bw /scan

# Application metrics
# Monitor API response times
# Track WebSocket message rates
# Monitor database query performance
```

## 🔄 Backup & Recovery

### 1. Automated Backup

```bash
# Database backup script
#!/bin/bash
BACKUP_DIR="/opt/robot/backups"
DATE=$(date +%Y%m%d_%H%M%S)

# Create backup directory
mkdir -p $BACKUP_DIR

# Database backup
pg_dump -U robot robotdb > $BACKUP_DIR/db_backup_$DATE.sql

# Configuration backup
tar -czf $BACKUP_DIR/config_backup_$DATE.tar.gz /opt/robot/config

# Logs backup
tar -czf $BACKUP_DIR/logs_backup_$DATE.tar.gz /opt/robot/logs

# Clean old backups (keep 7 days)
find $BACKUP_DIR -name "*.sql" -mtime +7 -delete
find $BACKUP_DIR -name "*.tar.gz" -mtime +7 -delete

echo "Backup completed: $DATE"
```

```bash
# Add to crontab
echo "0 2 * * * /opt/robot/scripts/backup.sh" | crontab -
```

### 2. Disaster Recovery

```bash
# System recovery steps:

# 1. Restore from backup image
sudo dd if=robot-backup.img of=/dev/mmcblk0 bs=4M status=progress

# 2. Restore database
createdb -U robot robotdb
psql -U robot robotdb < db_backup_latest.sql

# 3. Restore configuration
tar -xzf config_backup_latest.tar.gz -C /

# 4. Restart services
sudo systemctl restart robot-control.service

# 5. Verify system health
curl http://localhost:8080/api/health
```

## 📋 Deployment Checklist

### Pre-deployment

- [ ] Hardware setup complete
- [ ] Network configuration tested
- [ ] Dependencies installed
- [ ] Configuration files prepared
- [ ] SSL certificates obtained (if needed)
- [ ] Firewall rules configured
- [ ] Backup strategy planned

### Deployment

- [ ] Application deployed
- [ ] Services started and enabled
- [ ] Health checks passing
- [ ] API endpoints responding
- [ ] WebSocket connections working
- [ ] Camera feed streaming
- [ ] Sensor data flowing
- [ ] Navigation system operational

### Post-deployment

- [ ] Monitoring setup
- [ ] Log aggregation configured
- [ ] Alerts configured
- [ ] Documentation updated
- [ ] Team training completed
- [ ] Maintenance schedule created
- [ ] Emergency procedures documented

## 🆘 Emergency Procedures

### System Recovery

```bash
# Emergency stop all robot operations
rostopic pub /emergency_stop std_msgs/Bool "data: true"

# Restart robot control system
sudo systemctl restart robot-control.service

# Restart all Docker services
docker-compose restart

# Factory reset (last resort)
sudo systemctl stop robot-control.service
sudo rm -rf /opt/robot/logs/*
sudo cp /opt/robot/config/default/* /opt/robot/config/
sudo systemctl start robot-control.service
```

### Contact Information

- **Technical Support**: support@robotcompany.com
- **Emergency Hotline**: +66-XX-XXX-XXXX
- **Documentation**: https://docs.robotcompany.com
- **Community Forum**: https://forum.robotcompany.com

---

> **Ready for Production** - Follow this guide for reliable deployment
