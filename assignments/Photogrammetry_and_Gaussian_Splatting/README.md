# Photogrammetry and Gaussian Splatting on Lunar Apollo 17 imagery
## Part A
### Generate textured mesh model using Agisoft

1. Generate Tie Points  
![Image](https://github.com/user-attachments/assets/95339ad5-b0f2-4064-825c-20c8bcb6f789)

2. Genarate Point Cloud Model  
![Image](https://github.com/user-attachments/assets/b5076da4-42d0-48bd-bae6-c4872e043a68)  

3. Generate Textured Mesh Model  
![Image](https://github.com/user-attachments/assets/cea5806d-641d-408a-9a73-f31e6e29c740)


### Gaussian Splatting using nerfstudio

Although there is some noise in the generated 3D Gaussian Splatting, the result appears visually very well-reconstructed. 
https://github.com/user-attachments/assets/edc8f7ab-bb56-45cb-95c2-a7cc4c1bac6d


#### Metrics  

The Gaussian Splatting model showed poor results. To improve performance, increasing image quality or quantity, and tuning training parameters may be needed.

PSNR : 14.21  
SSIM : 0.178


```
  "results": {
    "psnr": 14.207271575927734,
    "psnr_std": NaN,
    "ssim": 0.17881707847118378,
    "ssim_std": NaN,
    "lpips": 0.7272768020629883,
    "lpips_std": NaN,
    "num_rays_per_sec": 434151.90625,
    "num_rays_per_sec_std": NaN,
    "fps": 0.3144685924053192,
    "fps_std": NaN
  }
```

### Future Plan
To improve quantitative metrics, the tuning training parameters will be necessary.


