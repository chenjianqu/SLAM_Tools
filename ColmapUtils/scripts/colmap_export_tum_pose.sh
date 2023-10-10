

colmap model_converter --input_path ./sparse/0 --output_path ./sparse/0 --output_type TXT



EvaluationTools_DIR=/home/cjq/CLionProjects/SLAM_Tools/EvaluationTools/bin

pose_file=./sparse/0/images.txt

${EvaluationTools_DIR}/ReadColmapToTum ${pose_file}

cp ./sparse/0/images_tum.txt ./images_tum.txt 

