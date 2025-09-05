import traci
import yaml
import threading
import time
import os
import sys
import logging
from multiprocessing import Manager
from typing import Dict

# Import các thành phần từ các module khác
from sumosim import SumoSim
from data.collector.SqlCollector import SqlCollector
from data.intersection_config_manager import IntersectionConfigManager
from data.detector_config_manager import DetectorConfigManager

# Import thuật toán và hàm chạy thử nghiệm từ module algo
from algorithm.algo import (
    PerimeterController, 
    KP_H, 
    KI_H, 
    N_HAT, 
    CONTROL_INTERVAL_S,
    run_perimeter_control_mock_test
)

# --- CẤU HÌNH LOGGING ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(threadName)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# =================================================================
# REAL SUMO SIMULATION
# =================================================================

def traffic_light_controller(shared_dict: Dict, config_file: str, stop_event: threading.Event):
    """
    Luồng riêng để điều khiển đèn giao thông dựa trên dữ liệu từ shared_dict.
    """
    logging.info("Bắt đầu luồng điều khiển đèn.")
    config_manager = IntersectionConfigManager(config_file)
    intersection_ids = config_manager.get_intersection_ids()

    while not stop_event.is_set():
        try:
            green_times = shared_dict.get('green_times', None)
            if green_times:
                current_green_times = green_times.copy()
                for int_id in intersection_ids:
                    if int_id in current_green_times:
                        tl_id = config_manager.get_traffic_light_id(int_id)
                        if not tl_id:
                            logging.warning(f"Không tìm thấy traffic_light_id cho intersection {int_id}.")
                            continue

                        phase_info = config_manager.get_phase_info(int_id)
                        if not phase_info:
                            continue

                        main_phases = phase_info.get('p', {}).get('phase_indices', [])
                        secondary_phases = [s_phase['phase_indices'][0] for s_phase in phase_info.get('s', []) if s_phase.get('phase_indices')]

                        new_times = current_green_times[int_id]
                        
                        try:
                            logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)[0]
                            
                            for phase_index in main_phases:
                                if 0 <= phase_index < len(logic.phases):
                                    logic.phases[phase_index].duration = new_times['p']

                            for i, phase_index in enumerate(secondary_phases):
                                if 0 <= phase_index < len(logic.phases) and i < len(new_times['s']):
                                    logic.phases[phase_index].duration = new_times['s'][i]
                            
                            traci.trafficlight.setCompleteRedYellowGreenDefinition(tl_id, logic)
                        except traci.TraCIException as e:
                            logging.error(f"Lỗi khi cập nhật TLS {tl_id}: {e}")
            
            time.sleep(1)

        except Exception as e:
            logging.error(f"Lỗi nghiêm trọng trong luồng điều khiển: {e}", exc_info=True)
            break
    
    logging.info("Dừng luồng điều khiển đèn.")

def load_simulation_config(config_path: str) -> dict:
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
        if config.get('type') == 'sumo': 
            return config.get('config', {})
        else:
            raise ValueError("Loại mô phỏng không được hỗ trợ. Chỉ hỗ trợ 'sumo'.")

def load_application_config(config_path: str) -> dict:
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
        if config is not None:
            return config
        else:
            raise ValueError("Không tìm thấy cấu hình ứng dụng.")

def run_sumo_simulation():
    """Hàm chính để khởi tạo và chạy mô phỏng SUMO thực tế."""
    # Xác định đường dẫn tuyệt đối cho các file cấu hình
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    sim_config_path = os.path.join(project_root, 'src', 'config', 'simulation.yml')
    app_config_path = os.path.join(project_root, 'src', 'config', 'application.yml')
    detector_config_path = os.path.join(project_root, 'src', 'config', 'detector_config.json')
    intersection_config_path = os.path.join(project_root, 'src', 'intersection_config.json')

    # Load cấu hình
    sim_config = load_simulation_config(sim_config_path)
    app_config = load_application_config(app_config_path)
    detector_config_mgr = DetectorConfigManager(detector_config_path)

    # --- Lấy các tham số từ file cấu hình ---
    total_simulation_time = sim_config.get('total_simulation_time', 3600)
    # CONTROL_INTERVAL_S được import từ algo.py và có giá trị là 90 giây

    # Khởi tạo kết nối DB
    sql_conn = SqlCollector(
        host=app_config['mysql']["host"],
        port=app_config['mysql']["port"],
        user=app_config['mysql']["user"],
        password=app_config['mysql']["password"], 
        database=app_config['mysql']["database"]
    )

    # Lấy ID của các detector
    algorithm_detector_ids = detector_config_mgr.get_algorithm_input_detectors()
    solver_detectors = detector_config_mgr.get_solver_input_detectors()
    logging.info(f"Tìm thấy {len(algorithm_detector_ids)} detectors cho đầu vào thuật toán.")
    logging.info(f"Tìm thấy {len(solver_detectors)} intersections cho đầu vào bộ giải.")

    with Manager() as manager:
        shared_dict = manager.dict()
        stop_event = threading.Event()

        sumo_sim = SumoSim(sim_config)

        # Định nghĩa đường dẫn output tuyệt đối
        output_dir = os.path.join(project_root, "output")
        os.makedirs(output_dir, exist_ok=True)
        output_filenames = {
            "tripinfo": os.path.join(output_dir, "tripinfo.xml"),
            "vehroute": os.path.join(output_dir, "vehroutes.xml")
        }
        
        sumo_sim.start(output_files=output_filenames)

        # Khởi tạo bộ điều khiển, sử dụng chu kỳ 90s mặc định
        controller = PerimeterController(
            kp=KP_H, ki=KI_H, n_hat=N_HAT, 
            config_file=intersection_config_path,
            shared_dict=shared_dict
        )

        # Bắt đầu luồng điều khiển đèn
        controller_thread = threading.Thread(
            target=traffic_light_controller, 
            args=(shared_dict, intersection_config_path, stop_event),
            name="LightController"
        )
        controller_thread.start()

        n_previous = 0
        qg_previous = 3600
        
        # Lấy giá trị ban đầu
        sumo_sim.step()
        try:
            n_previous = sumo_sim.get_total_vehicle_count(algorithm_detector_ids)
        except traci.TraCIException as e:
            logging.warning(f"Không thể lấy dữ liệu ban đầu: {e}")
            n_previous = 0

        try:
            # --- Khởi tạo các biến hẹn giờ ---
            next_control_time = CONTROL_INTERVAL_S
            next_log_time = 10  # Log mỗi 10 giây

            # Thêm một thông báo trước khi bắt đầu vòng lặp
            logging.info(f"Bắt đầu vòng lặp mô phỏng chính. Dừng khi hết xe hoặc sau {total_simulation_time}s.")
            while traci.simulation.getTime() < total_simulation_time and traci.simulation.getMinExpectedNumber() > 0:
                sumo_sim.step()
                current_time = traci.simulation.getTime()

                # --- BƯỚC ĐIỀU KHIỂN: CHẠY THUẬT TOÁN VỚI DỮ LIỆU TRỰC TIẾP ---
                if current_time >= next_control_time:
                    logging.info(f"--- Chạy bước điều khiển tại thời điểm: {current_time:.1f}s ---")
                    
                    # Lấy dữ liệu trực tiếp (live) từ SUMO
                    n_current = sumo_sim.get_total_vehicle_count(algorithm_detector_ids)
                    live_queues = sumo_sim.get_live_queue_lengths(solver_detectors)
                    
                    logging.info(f"Sử dụng dữ liệu trực tiếp: n(k) = {n_current}")
                    
                    _, qg_previous, _ = controller.run_simulation_step(
                        n_current, n_previous, qg_previous, live_queues
                    )
                    
                    n_previous = n_current
                    next_control_time += CONTROL_INTERVAL_S
                
                # --- GHI LOG TIẾN ĐỘ ---
                if current_time >= next_log_time:
                    remaining_vehicles = traci.simulation.getMinExpectedNumber()
                    logging.info(f"Thời gian: {current_time:.0f}s, Xe còn lại: {remaining_vehicles}")
                    next_log_time += 10

        finally:
            logging.info("Hoàn tất mô phỏng. Dừng các luồng...")
            stop_event.set()
            controller_thread.join()
            try:
                sumo_sim.close()
                sql_conn.close()
            except Exception as e:
                logging.error(f"Lỗi khi đóng kết nối: {e}", exc_info=True)
            logging.info(f"Mô phỏng kết thúc. Tổng số bước: {sumo_sim.get_step_counts()}")


if __name__ == "__main__":
    # Mặc định chạy mô phỏng SUMO thực tế.
    # Để chạy mô phỏng thử nghiệm với dữ liệu giả lập, dùng lệnh:
    # python src/main.py mock
    
    if len(sys.argv) > 1 and sys.argv[1] == 'mock':
        # Gọi hàm chạy thử nghiệm đã được chuyển vào module algo
        run_perimeter_control_mock_test()
    else:
        run_sumo_simulation()

