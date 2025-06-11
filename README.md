# jetson-lidar-wsl
1. WSL2에서 라이다 측정데이터를 이용하여 스캔영상을 그려주는 패키지 lidarplot를 완성
+ jetson 파일은 압축
+ 
+ /scan토픽 구독 -> (각도,거리)로 환산 -> 스캔 영상 그리기 순서로 처리할 것
+ 스캔 영상 그리기는 opencv의 회전변환 또는 삼각함수를 이용
+ 스캔 영상을 모니터에 출력하고 동시에 동영상(mp4)으로 저장할 것
+ Jetson nano 보드에서 sllidar_node를 실행하고 lidarplot 패키지는 WSL2에서 작성하고 실행할것
  
https://github.com/user-attachments/assets/14dad466-48f0-4163-954d-c26a8f1eba58

2. 

