---
sidebar_position: 4
id: setup-cloud
title: Cloud-Native Development Setup
description: Set up Docker, Kubernetes, and cloud deployment
---

# Cloud-Native Development Setup

This guide covers containerizing your ROS 2 application and deploying to cloud platforms for distributed development and deployment.

## Architecture Overview

```
┌─────────────────────────────────────────┐
│        Cloud Platform (AWS/GCP)         │
├─────────────────────────────────────────┤
│  ┌─────────────────────────────────────┐│
│  │  Kubernetes Cluster                  ││
│  ├─────────────────────────────────────┤│
│  │ ┌──────────────┐  ┌──────────────┐ ││
│  │ │  ROS 2 Pods  │  │  Model Pods  │ ││
│  │ └──────────────┘  └──────────────┘ ││
│  └─────────────────────────────────────┘│
└─────────────────────────────────────────┘
```

## Prerequisites

- Docker installed
- Kubernetes CLI (kubectl)
- Cloud account (AWS/GCP/Azure)
- Basic Docker/Kubernetes knowledge

## Step 1: Containerize Your Application

### 1.1 Create Dockerfile

Create `Dockerfile` in your ROS 2 workspace:

```dockerfile
# Multi-stage build for optimal size
FROM osrf/ros:humble-desktop as builder

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY . src/

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Runtime stage - smaller image
FROM osrf/ros:humble-runtime

COPY --from=builder /ros2_ws/install /opt/ros2_ws/install

RUN echo "source /opt/ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
CMD ["-c", "source /opt/ros2_ws/install/setup.bash && ros2 launch your_package your_launch_file.py"]
```

### 1.2 Build Docker Image

```bash
docker build -t your-registry/ros2-app:latest .

# Tag for container registry
docker tag your-registry/ros2-app:latest your-registry/ros2-app:v1.0.0
```

### 1.3 Test Locally

```bash
# Run container
docker run --rm \
    --net=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    your-registry/ros2-app:latest
```

## Step 2: Push to Container Registry

### AWS ECR

```bash
# Create repository
aws ecr create-repository --repository-name ros2-app

# Login
aws ecr get-login-password --region us-east-1 | docker login --username AWS --password-stdin <account-id>.dkr.ecr.us-east-1.amazonaws.com

# Push
docker push <account-id>.dkr.ecr.us-east-1.amazonaws.com/ros2-app:latest
```

### Google Container Registry

```bash
# Configure gcloud
gcloud config set project PROJECT_ID

# Push
docker tag your-registry/ros2-app:latest gcr.io/PROJECT_ID/ros2-app:latest
docker push gcr.io/PROJECT_ID/ros2-app:latest
```

## Step 3: Set Up Kubernetes Cluster

### AWS EKS

```bash
# Create cluster
eksctl create cluster --name ros2-cluster --region us-east-1 --nodes 3

# Update kubeconfig
aws eks update-kubeconfig --region us-east-1 --name ros2-cluster

# Verify
kubectl get nodes
```

### Google GKE

```bash
# Create cluster
gcloud container clusters create ros2-cluster \
    --num-nodes=3 \
    --enable-ip-alias \
    --zone us-central1-a

# Get credentials
gcloud container clusters get-credentials ros2-cluster --zone us-central1-a

# Verify
kubectl get nodes
```

## Step 4: Deploy to Kubernetes

### 4.1 Create Deployment YAML

Create `ros2-deployment.yaml`:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: ros2-app
  labels:
    app: ros2-app
spec:
  replicas: 3
  selector:
    matchLabels:
      app: ros2-app
  template:
    metadata:
      labels:
        app: ros2-app
    spec:
      containers:
      - name: ros2-app
        image: gcr.io/PROJECT_ID/ros2-app:latest
        imagePullPolicy: Always
        resources:
          requests:
            memory: "2Gi"
            cpu: "1000m"
          limits:
            memory: "4Gi"
            cpu: "2000m"
        env:
        - name: ROS_DOMAIN_ID
          value: "1"
        - name: ROS_LOCALHOST_ONLY
          value: "0"
---
apiVersion: v1
kind: Service
metadata:
  name: ros2-app-service
spec:
  selector:
    app: ros2-app
  ports:
  - port: 11311
    targetPort: 11311
    protocol: UDP
  type: LoadBalancer
```

### 4.2 Deploy

```bash
# Apply deployment
kubectl apply -f ros2-deployment.yaml

# Check status
kubectl get pods
kubectl get svc

# View logs
kubectl logs -f deployment/ros2-app
```

## Step 5: Set Up ROS 2 DDS (DDS-aware networking)

For multi-pod communication:

```yaml
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: ros2-config
data:
  RMW_IMPLEMENTATION: rmw_fastrtps_cpp
  FASTRTPS_DEFAULT_PROFILES_FILE: /etc/ros2/fastrtps.xml
```

## Step 6: Scale and Monitor

### Auto-scaling

```yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: ros2-app-hpa
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: ros2-app
  minReplicas: 2
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
```

### Monitoring with Prometheus

```bash
# Install Prometheus
helm repo add prometheus-community https://prometheus-community.github.io/helm-charts
helm install prometheus prometheus-community/prometheus

# Create ServiceMonitor
kubectl apply -f - <<EOF
apiVersion: monitoring.coreos.com/v1
kind: ServiceMonitor
metadata:
  name: ros2-monitor
spec:
  selector:
    matchLabels:
      app: ros2-app
  endpoints:
  - port: metrics
    interval: 30s
EOF
```

## Step 7: CI/CD Pipeline

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to K8s

on:
  push:
    branches: [ main ]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v2

    - name: Build Docker image
      run: docker build -t gcr.io/${{ secrets.GCP_PROJECT }}/ros2-app:${{ github.sha }} .

    - name: Push to GCR
      run: |
        echo ${{ secrets.GCP_KEY }} | docker login -u _json_key --password-stdin https://gcr.io
        docker push gcr.io/${{ secrets.GCP_PROJECT }}/ros2-app:${{ github.sha }}

    - name: Deploy to K8s
      run: |
        kubectl set image deployment/ros2-app ros2-app=gcr.io/${{ secrets.GCP_PROJECT }}/ros2-app:${{ github.sha }}
```

## Troubleshooting

**Issue**: Pods stuck in Pending
- **Solution**: Check resource availability `kubectl describe node`

**Issue**: ROS 2 pods can't communicate
- **Solution**: Verify ROS_DOMAIN_ID and DDS configuration

**Issue**: High latency between pods
- **Solution**: Use node affinity to place pods on same node

## Cost Optimization

- Use spot instances (30-70% cheaper)
- Set resource requests/limits appropriately
- Use horizontal pod autoscaling
- Schedule non-critical workloads during off-hours

## Production Checklist

- [ ] Docker image optimized (multi-stage build)
- [ ] Container image security scanned
- [ ] Kubernetes manifests created
- [ ] Health checks configured (liveness/readiness probes)
- [ ] Resource limits set
- [ ] Monitoring and alerting enabled
- [ ] Backup and disaster recovery plan
- [ ] CI/CD pipeline automated

---

**Next Steps**: Monitor your deployed application and scale based on load
