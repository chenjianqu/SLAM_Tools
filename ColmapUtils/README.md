# SLAM_Tools

## ColmapUtils

###  select_keyframe_by_gps

功能：根据GPS位置，选择图像中的某些帧作为关键帧。

* 用法

```shell
./select_keyframe_by_gps ${img_dir} ${gps_record_path} ${save_dir}
```



* 示例：

```shell
img_dir=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_400/images
gps_record_path=/home/cjq/CLionProjects/ROS_Tools/gnss_raw.txt
save_dir=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_400_kf/images

./select_keyframe_by_gps ${img_dir} ${gps_record_path} ${save_dir}
```



```shell
img_dir=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_2000/images
gps_record_path=/home/cjq/CLionProjects/ROS_Tools/gnss_raw.txt
save_dir=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_2000/images_kf

./select_keyframe_by_gps ${img_dir} ${gps_record_path} ${save_dir}
```



### select_mask_by_rgb

根据rgb关键帧选择mask关键帧

* 用法

```shell
./select_mask_by_rgb ${rgb_dir} ${mask_dir} ${save_dir}
```



* 示例：

```shell
rgb_dir=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_2000/images_kf
mask_dir=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_2000/mask
save_dir=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_2000/mask_kf

./select_mask_by_rgb ${rgb_dir} ${mask_dir} ${save_dir}
```



```shell
rgb_dir=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_400_kf/images
mask_dir=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_400/mask
save_dir=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_400_kf/mask

./select_mask_by_rgb ${rgb_dir} ${mask_dir} ${save_dir}
```


## colmap_yaml_setter

```shell

colmap_yaml_setter -yaml_path /home/cjq/CLionProjects/Mapping/vslam-mapping/config/params.cfg \
-key intensity_thr -int_value 180

```


