## 运行新数据集yaml文件修改注意事项：
* 修改2个相机、惯导的话题名
* 根据图像消息的消息类型，选择Image或CompressedImage
* 修改相机-惯导之间的外参
* 在总的yaml文件中修改相机内参、相机分辨率（不修改分辨率会报错）
* 在总的yaml文件中正确填写各相机的内参文件
* 在各相机内参文件的投影参数、畸变参数、相机模型填写正确

## 如果要对某节点进行调试，在launch文件中进行如下修改：
``` <node name="MCVIO_estimator" pkg="mcvio" type="mcvio_estimator" output="screen" launch-prefix="xterm -e gdb --args"> ```