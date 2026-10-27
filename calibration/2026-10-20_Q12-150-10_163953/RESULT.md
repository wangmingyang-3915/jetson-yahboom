# 调焦后重新标定结果

棋盘：Q12-150-10；内角点 11×8；方格边长 10 mm；分辨率 1280×1024。
保存 68 张采样图像，离线复核成功 66 张，重投影 RMS 0.27693 px，最差单张 0.74095 px。
该误差是标定样本上的复核，不是独立测量精度。

- calibrationdata.tar.gz：原始采样与标定程序输出。
- calibrated.yaml：标定程序计算的新内参。
- quality_report.json：各图像的误差与复核方法。
- deployment_report.json：部署时间、文件及实时内参验证结果。
- pre_calibration.yaml / pre_deployment_*.yaml：部署前标定备份，属于标定数据。
- cameracalibrator.log：采样和计算日志。

新内参已通过 /set_camera_info 加载，并同步到工作空间配置；无需重启视觉服务。
