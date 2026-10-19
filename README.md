# Bài thực hành 05 — TurtleBot 4, Nav2 và YOLO

## 1. Cài đặt và build

```bash
mkdir -p ~/workspaces
git clone https://github.com/Gnoud0612/tb4-lab05-23020732.git ~/workspaces/tb4_lab05
cd ~/workspaces/tb4_lab05
sudo apt update
sudo apt install -y ros-humble-turtlebot4-simulator \
  ros-humble-irobot-create-nodes ros-humble-turtlebot4-navigation \
  ros-humble-turtlebot4-viz ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-cv-bridge \
  ros-humble-teleop-twist-keyboard python3-venv python3-pil python3-yaml \
  python3-colcon-common-extensions
bash scripts/setup_yolo.sh
bash scripts/build.sh
```

Trong **mỗi terminal mới**:

```bash
cd ~/workspaces/tb4_lab05
source scripts/env.bash
```


## 2. Simulation và localization — Terminal A

```bash
bash scripts/run.sh navigation world:=maze model:=lite
```


```bash
bash scripts/run.sh navigation world:=maze model:=lite headless:=true
```

## 3. Chuẩn bị robot, vật thể và camera YOLO — Terminal B

Đợi robot xuất hiện rồi chạy:

```bash
bash scripts/prepare.sh
bash scripts/perception.sh
```

`prepare.sh` giảm tải sensor, áp dụng profile riêng cho maze phẳng, undock, thêm mô hình người/trụ cứu hỏa, khởi tạo AMCL một lần và kiểm tra topic/TF. Chạy một lần mỗi phiên simulator. Trước khi chạy mission, kiểm tra scan khớp tường và robot đúng vị trí trên RViz.

Node YOLO subscribe `/oakd/rgb/preview/image_raw`, chuyển ảnh bằng `cv_bridge`, hiển thị box/class/confidence và publish `/yolo/annotated_image`, `/yolo/detections`. Hai lớp mặc định là COCO `person` và `fire hydrant`. Cửa sổ camera hiển thị trạng thái mission lấy từ ROS; kết quả nhận diện thực được in ở Terminal B.

## 4. Tự chạy 3 goal bằng code — Terminal C

```bash
bash scripts/mission.sh
```

Node `waypoint_mission` gọi Action `/navigate_to_pose`, chờ `SUCCEEDED` rồi mới gửi goal tiếp theo. Không dùng teleop, không dùng Nav2 Goal hoặc đặt lại pose sau khi mission bắt đầu. Nếu goal lỗi, có BUMP hoặc hết thời gian, node dừng nhiệm vụ và hủy goal hiện tại.

| Thứ tự | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| Goal 1 — tây bắc | -8.0 | 8.8 | 0.0 |
| Goal 2 — tây nam | -8.5 | -9.0 | 2.2 |
| Goal 3 — đông nam | 8.5 | -9.0 | 2.08 |

Tọa độ thuộc frame `map`, cấu hình tại `src/tb4_yolo_nav/config/goals.yaml`. Thông tin thực thi lưu ở `evidence/mission.json`; kết quả YOLO ở `evidence/yolo/`.

Muốn kiểm tra độc lập các goal, mở Terminal D đã source môi trường:

```bash
ros2 run tb4_lab goal_monitor --required-goals 3 \
  --minimum-separation 2 --output evidence/navigation_goals.json
```

## 5. Tự lấy tọa độ khác trước khi chạy mission

Trong RViz đặt Fixed Frame là `map`, dùng **Publish Point**:

```bash
ros2 topic echo /clicked_point
ros2 run tb4_yolo_nav record_goals --ros-args -p yaw:=0.0 \
  -p output:=recorded_goals.yaml
```

`record_goals` lưu các điểm nhấp và yaw hiện tại. Đổi yaw cho điểm sắp ghi ở terminal khác bằng `ros2 param set /goal_recorder yaw 1.57`. Ghi ít nhất 3 điểm trong vùng trống; `Ctrl+C` rồi chạy tuyến mới:

```bash
bash scripts/mission.sh route_file:="$PWD/recorded_goals.yaml"
```

## 6. Kết thúc

Bấm `Ctrl+C` ở các terminal mission/YOLO/monitor, rồi:

```bash
bash scripts/stop_sim.sh
```

Profile tắt cliff reflex chỉ dùng cho **maze phẳng trong mô phỏng**; BUMP/STUCK/WHEEL_DROP vẫn bật. Các thư mục `.venv`, `.runtime`, `build`, `install`, `log`, video và báo cáo không cần đưa lên GitHub.
