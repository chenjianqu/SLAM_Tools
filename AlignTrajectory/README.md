 

## 轨迹补偿

```shell
EXE_PATH=/home/cjq/CLionProjects/SLAM_Tools/AlignTrajectory/bin

${EXE_PATH}/complete_trajectory \
-vins_pose_path /mnt/done_data/22_GND_vslam/20230728_HS5001_2M/20230605_143344162_hq/pose_full/new_pose_imu.txt \
-op_pose_path /mnt/done_data/22_GND_vslam/20230728_HS5001_2M/20230605_143344162_hq/pose_opt/new_pose_2.txt


#-out_pose_path /mnt/done_data/22_GND_vslam/20230728_HS5001_2M/20230605_143344162_hq/pose_gq/complete_pose.txt


EXE_PATH=/home/cjq/CLionProjects/SLAM_Tools/AlignTrajectory/bin

${EXE_PATH}/complete_trajectory \
-vins_pose_path /home/cjq/CLionProjects/SLAM_Tools/AlignTrajectory/data/pose_vins.txt \
-op_pose_path /home/cjq/CLionProjects/SLAM_Tools/AlignTrajectory/data/new_pose_opt.txt

```



## Ceres优化

### 求导

误差：
$$
e = \ln (\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee}
$$





$$
\frac{d \ln (\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee} }{d \bold{R}_e} 
= 
\lim_{\Delta \boldsymbol{ \theta} \to 0} { \frac{ 
\ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{\text{e}}  \exp( \Delta \boldsymbol{ \theta}^{ \land } )
\bold{R}_{\text{vins}}
)^{\vee}} 
- \ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee}} }{\Delta \boldsymbol{ \theta}} }
\\根据BCH的性质，有：
\\
=\lim_{\Delta \boldsymbol{ \theta} \to 0} { \frac{ 
\ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{\text{e}} \bold{R}_{\text{vins}} \exp(( \bold{R}^{T}_{\text{vins}} \Delta \boldsymbol{ \theta})^{ \land } )
)^{\vee}} 
- 
\ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee}} }{\Delta \boldsymbol{ \theta}} }
\\
=\lim_{\Delta \boldsymbol{\theta} \to 0} { \frac{ 
\ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{\text{e}} \bold{R}_{\text{vins}})^{\vee}} 
+
\bold{J}_r^{-1} \Delta \boldsymbol{\theta} 
- 
\ln{(\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee}} }{\Delta \boldsymbol{ \theta}} }
\\
=\bold{J}_r^{-1}
$$


其中 $\bold{J}_r^{-1}$ 的计算： $\bold{J}_r^{-1} = \bold{J}_r^{-1}( \ln (\bold{R}_{\text{op}}^{T} \bold{R}_{e} \bold{R}_{\text{vins}})^{\vee}  )$ 

和 $\bold{J}^{-1}_{r}$  为 SO(3) 上的右雅可比：
$$
\boldsymbol{J}_r^{-1} (\theta \boldsymbol{\omega}) = \frac{\theta}{2} \cot \frac{\theta}{2} \bold{I} + ( 1 - \frac{\theta}{2} \cot \frac{\theta}{2}) \boldsymbol{\omega} \boldsymbol{\omega} ^{T} + \frac{\theta}{2} \boldsymbol{\omega}^{\land}
$$





