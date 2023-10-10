 

# EvaluationTools

## ReadColmapToTum

功能：用于读取COLMAP输出的位姿文件`images.txt`，生成TUM格式的位姿。

* 用法

```shell
./ReadColmapToTum ${pose_file}
```

* 示例

```shell
pose_file=/home/cjq/dataset/YT_city/sfm/2022-10-17-12-04-27_mini_2000/spare/images.txt

exe_dir=/home/cjq/CLionProjects/SLAM_Tools/EvaluationTools/bin

${exe_dir}/ReadColmapToTum ${pose_file}
```



