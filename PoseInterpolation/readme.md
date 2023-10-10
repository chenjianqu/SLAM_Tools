# Demo

## Usage

### PoseInterpolation

```shell
image_dir=/home/cjq/dataset/raw_image/

odom_txt_path=/home/cjq/CLionProjects/PoseInterpolation/data/odo.txt

./bin/PoseInterpolation ${odom_txt_path} ${image_dir}
```

或

```shell
./bin/PoseInterpolation /home/cjq/CLionProjects/PoseInterpolation/data/odo.txt /home/cjq/dataset/raw_image/
```







### interpolation_by_txt

* 用法

```shell
odom_txt_path

./interpolation_by_txt ${odom_txt_path} ${img_time_txt}

```



* 示例

先得到时间戳

```shell
bag_name=20230505_165515189_hq.bag
rosrun rosbag_test get_image_time_stamp ${bag_name} /camera/maxieye/610  ${bag_name}.txt

```





```shell
interpolation_dir=/home/cjq/CLionProjects/SLAM_Tools/PoseInterpolation/bin

bag_name=20230520_150407499

#keyframe_txt_path=${bag_name}_hq.txt
keyframe_txt_path=odo.txt
img_time_txt=${bag_name}_hq.bag.txt

${interpolation_dir}/interpolation_by_txt ${keyframe_txt_path} ${img_time_txt}



```





