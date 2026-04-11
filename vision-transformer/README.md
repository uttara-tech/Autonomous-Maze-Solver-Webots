
# 🚁 UZH-FPV Drone Vision Transformer + IMU Fusion

**Processing RPG Zurich's drone racing dataset for real-world navigation**

## 🚀 Current Progress (Week 1/4)
✅ Frame extraction + timestamp CSV parsing  
✅ 640X480 → 1200 patch tokens for single channel   
✅ IMU+vision multimodal fusion  

## 📈 Next Steps Roadmap
```mermaid
graph LR
  A[256×256 Preprocessing] --> B[Train/Val/Test Split]
  B --> C[Transformer Only Training]
  C --> D[Add RL Head]
  D --> E[Trajectory Prediction]

```


## Technical Notes

```markdown
- Why 256×256?
    Standard ViT size → 16×16 patches = 256 tokens (vs 675)
- Dataset: 
    UZH-FPV (RPG Zurich) → trajectory regression (x,y,z,quat)
- Loss: 
    MSE position + quaternion distance
```

