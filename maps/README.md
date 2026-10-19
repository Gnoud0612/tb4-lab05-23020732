# Bản đồ SLAM

`map.yaml` và `map.pgm` là bản đồ mới của world `maze`, TurtleBot 4 Lite, độ phân giải 0,05 m/ô. Robot đã khảo sát các nhánh trái, hành lang trung tâm, khu đáy, nhánh phải và hai phía dãy tường chéo bằng teleoperation. Dữ liệu LiDAR và odometry được xử lý bằng SLAM Toolbox; lần xử lý cuối dùng điều kiện khép vòng chặt hơn trong `src/tb4_lab/config/slam.yaml` để tránh ghép nhầm các đoạn tường lặp lại. Không vẽ tường hoặc điền ô trống bằng dữ liệu hình học của simulator.

`map_raw.yaml` và `map_raw.pgm` giữ kết quả map_saver_cli. Bản dùng cho Nav2 chỉ crop vùng x/y từ -11 đến +11 m, giữ nguyên từng pixel; `map_crop.json` lưu chỉ số crop và SHA256. Các ô xám ngoài tường, bên trong vật cản và khu kín không có lối vào vẫn là UNKNOWN. `free_thresh: 0.196` giữ pixel 205 ở trạng thái UNKNOWN; planner dùng `allow_unknown: false`.

`keepout_mask.yaml` / `keepout_mask.pgm` là cấu hình vùng cấm quanh đế sạc thấp, độc lập với bản đồ đo SLAM. Mask đánh dấu x từ -0,05 đến 0,45 m, y từ -0,30 đến 0,30 m trong frame map; KeepoutFilter và inflation của Nav2 bảo vệ đường đi qua khu vực này. Không dùng mask như bản đồ SLAM.
