# -*- coding: utf-8 -*-

"""
Chương trình chính để khởi chạy và điều khiển mô phỏng giao thông bằng SUMO.

Kịch bản hoạt động:
1. Khởi tạo mô phỏng SUMO và các trình quản lý cấu hình.
2. Chạy một luồng (thread) riêng để cập nhật trạng thái đèn giao thông.
3. Vòng lặp mô phỏng chính thực hiện 3 bước theo chu kỳ:
    - BƯỚC 1: Thu thập dữ liệu thô (số xe, độ dài hàng đợi) từ các detector trong SUMO.
    - BƯỚC 2: Tổng hợp dữ liệu thô thành các giá trị trung bình.
    - BƯỚC 3: Chạy thuật toán điều khiển vành đai (Perimeter Control) và bộ giải (Solver)
              để tính toán thời gian xanh mới cho các đèn tín hiệu.
4. Luồng điều khiển đèn sẽ nhận thời gian xanh mới và cập nhật vào mô phỏng.
5. Mô phỏng kết thúc khi hết thời gian hoặc không còn xe.
"""

import traci
import yaml
import threading
import time
import os
import sys
import logging
from multiprocessing import Manager
from typing import Dict, Any, List

# Import các thành phần cần thiết từ các module khác trong dự án
from sumosim import SumoSim
from data.intersection_config_manager import IntersectionConfigManager
from data.detector_config_manager import DetectorConfigManager
from algorithm.algo import (
    PerimeterController, 
    KP_H, 
    KI_H, 
    N_HAT, 
    CONTROL_INTERVAL_S
)

# --- CẤU HÌNH LOGGING ---
# Thiết lập hệ thống ghi log để theo dõi hoạt động của chương trình.
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(threadName)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# =============================================================================
# CÁC HÀM TẢI CẤU HÌNH
# =============================================================================

def load_yaml_config(config_path: str) -> Dict[str, Any]:
    """
    Tải và phân tích một file cấu hình YAML.

    Args:
        config_path: Đường dẫn đến file YAML.

    Returns:
        Một dictionary chứa nội dung của file cấu hình.
    
    Raises:
        ValueError: Nếu file không tồn tại hoặc rỗng.
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"File cấu hình không tồn tại: {config_path}")
    with open(config_path, "r", encoding='utf-8') as f:
        config = yaml.safe_load(f)
        if config is not None:
            return config
        else:
            raise ValueError(f"File cấu hình rỗng hoặc không hợp lệ: {config_path}")

# =============================================================================
# LUỒNG ĐIỀU KHIỂN ĐÈN GIAO THÔNG
# =============================================================================

def update_traffic_light_logic(tl_id: str, new_times: Dict[str, Any], phase_info: Dict[str, Any]):
    """
    Cập nhật logic (thời gian xanh) cho một đèn giao thông cụ thể.

    Args:
        tl_id: ID của đèn giao thông trong SUMO.
        new_times: Dictionary chứa thời gian xanh mới cho pha chính ('p') và các pha phụ ('s').
        phase_info: Thông tin về các chỉ số pha chính và phụ.
    """
    try:
        # Lấy định nghĩa đầy đủ của đèn (bao gồm các pha)
        logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)[0]

        # Cập nhật thời gian xanh cho các pha chính
        main_phases = phase_info.get('p', {}).get('phase_indices', [])
        for phase_index in main_phases:
            if 0 <= phase_index < len(logic.phases):
                logic.phases[phase_index].duration = new_times['p']

        # Cập nhật thời gian xanh cho các pha phụ
        secondary_phases = [s_phase['phase_indices'][0] for s_phase in phase_info.get('s', []) if s_phase.get('phase_indices')]
        for i, phase_index in enumerate(secondary_phases):
            if 0 <= phase_index < len(logic.phases) and i < len(new_times['s']):
                logic.phases[phase_index].duration = new_times['s'][i]
        
        # Áp dụng logic mới vào mô phỏng
        traci.trafficlight.setCompleteRedYellowGreenDefinition(tl_id, logic)

    except traci.TraCIException as e:
        logging.error(f"Lỗi Traci khi cập nhật TLS {tl_id}: {e}")

def traffic_light_controller(shared_dict: Dict, config_manager: IntersectionConfigManager, stop_event: threading.Event):
    """
    Luồng chạy nền để liên tục kiểm tra và cập nhật đèn giao thông.
    Hàm này hoạt động như một "người tiêu dùng" (consumer), lấy dữ liệu thời gian xanh
    từ `shared_dict` (do thuật toán chính tạo ra) và áp dụng vào mô phỏng.

    Args:
        shared_dict: Dictionary được chia sẻ giữa các luồng để truyền dữ liệu.
        config_manager: Đối tượng quản lý cấu hình giao lộ.
        stop_event: Sự kiện để báo hiệu dừng luồng.
    """
    logging.info("Bắt đầu luồng điều khiển đèn.")
    intersection_ids = config_manager.get_intersection_ids()

    while not stop_event.is_set():
        try:
            # Chỉ hoạt động khi thuật toán đã kích hoạt và có dữ liệu mới
            if shared_dict.get('is_active', False) and 'green_times' in shared_dict:
                green_times = shared_dict.get('green_times')
                if green_times:
                    # Cập nhật từng giao lộ
                    for int_id in intersection_ids:
                        if int_id in green_times:
                            tl_id = config_manager.get_traffic_light_id(int_id)
                            phase_info = config_manager.get_phase_info(int_id)
                            if tl_id and phase_info:
                                update_traffic_light_logic(tl_id, green_times[int_id], phase_info)
                            else:
                                logging.warning(f"Bỏ qua giao lộ {int_id} do thiếu tl_id hoặc phase_info.")
            
            # Tạm dừng một chút để tránh tiêu tốn CPU
            time.sleep(1)

        except Exception as e:
            logging.error(f"Lỗi nghiêm trọng trong luồng điều khiển đèn: {e}", exc_info=True)
            break
    
    logging.info("Dừng luồng điều khiển đèn.")

# =============================================================================
# CÁC HÀM HỖ TRỢ VÒNG LẶP MÔ PHỎNG
# =============================================================================

# def get_sum_from_traci_detectors(detector_ids: List[str], flow_ids: List[str]) -> int:
#     """
#     Lấy số lượng phương tiện từ một danh sách các area detector của Traci.
#     An toàn trước các lỗi Traci (trả về 0 nếu có lỗi).
#     """
#     try:
#         flow = sum(traci.inductionloop.getLastIntervalVehicleNumber(det_id) for det_id in flow_ids)
#         sum_of_avg_speed = sum(traci.lanearea.getLastIntervalMeanSpeed(det_id) * traci.lanearea.getLastIntervalVehicleNumber(det_id) for det_id in detector_ids)
#         if (sum(traci.lanearea.getLastIntervalVehicleNumber(det_id) for det_id in detector_ids)):
#             avg_speed = sum_of_avg_speed / sum(traci.lanearea.getLastIntervalVehicleNumber(det_id) for det_id in detector_ids)
#             return (flow / avg_speed) * 80
#         else: 
#             avg_speed = 0
#             return 0
        
#     except traci.TraCIException:
#         return 0

def get_sum_from_traci_detectors(detector_ids: List[str]) -> int:
    """
    Lấy số lượng phương tiện (tích lũy) dựa trên độ chiếm dụng theo không gian.
    An toàn trước các lỗi Traci và lỗi chia cho 0.
    """
    try:
        total_accumulation = 0
        for det_id in detector_ids:
            space_occupancy = traci.lanearea.getLastIntervalOccupancy(det_id)
            road_length = 80.00
            average_length_of_vehicles = 3
            num_lane = 1
            # Tính toán tích lũy tại một detector:
            accumulation = road_length * (num_lane / (100 * average_length_of_vehicles)) * space_occupancy
            total_accumulation += accumulation
        return total_accumulation # Trả về 0 nếu không có xe, tốc độ trung bình hoặc interval bằng 0

    except traci.TraCIException as e:
        logging.warning(f"Lỗi TraCI trong get_sum_from_traci_detectors: {e}")
        return 0


def initialize_queue_samples(solver_detectors: Dict) -> Dict:
    """Khởi tạo cấu trúc dữ liệu để lưu trữ các mẫu hàng đợi."""
    queue_samples = {}
    for int_id, int_details in solver_detectors.items():
        num_secondary_phases = len(int_details.get('phases', {}).get('s', []))
        queue_samples[int_id] = {
            'p': [],
            's': [[] for _ in range(num_secondary_phases)]
        }
    return queue_samples

def clear_samples(n_samples: List, queue_samples: Dict):
    """Xóa tất cả các mẫu đã thu thập để chuẩn bị cho chu kỳ tổng hợp tiếp theo."""
    n_samples.clear()
    for int_id in queue_samples:
        queue_samples[int_id]['p'].clear()
        for s_phase_samples in queue_samples[int_id]['s']:
            s_phase_samples.clear()

# =============================================================================
# HÀM CHẠY MÔ PHỎNG CHÍNH
# =============================================================================

def run_sumo_simulation():
    """Hàm chính để khởi tạo và chạy toàn bộ kịch bản mô phỏng SUMO."""
    try:
        # --- 1. KHỞI TẠO --- 
        logging.info("Bắt đầu quá trình khởi tạo mô phỏng...")

        # Xác định các đường dẫn file cấu hình
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..')) 
        sim_config_path = os.path.join(project_root, 'src', 'config', 'simulation.yml')
        detector_config_path = os.path.join(project_root, 'src', 'config', 'detector_config.json')
        intersection_config_path = os.path.join(project_root, 'src', 'config', 'intersection_config.json')

        # Tải các file cấu hình
        sim_config = load_yaml_config(sim_config_path).get('config', {})
        detector_config_mgr = DetectorConfigManager(detector_config_path)
        intersection_config_mgr = IntersectionConfigManager(intersection_config_path)

        # Lấy các tham số mô phỏng
        sampling_interval_s = sim_config.get('sampling_interval_s', 10)
        aggregation_interval_s = sim_config.get('aggregation_interval_s', 50)
        total_simulation_time = sim_config.get('total_simulation_time', 3600)

        # Lấy ID của các detector cần thiết
        algorithm_detector_ids = detector_config_mgr.get_algorithm_input_detectors()
        solver_detectors = detector_config_mgr.get_solver_input_detectors()
        flow_algorithm_detector = detector_config_mgr.get_mfd_input_flow_detectors()
        logging.info(f"Tìm thấy {len(algorithm_detector_ids)} detectors cho thuật toán vành đai.")
        logging.info(f"Tìm thấy {len(solver_detectors)} giao lộ cho bộ giải cục bộ.")
        logging.info(f"Tìm thấy {len(flow_algorithm_detector)} detectors cho flow đầu vào thuật toán.")

        # --- 2. THIẾT LẬP MÔI TRƯỜNG ĐA LUỒNG VÀ SUMO ---
        with Manager() as manager:
            shared_dict = manager.dict() # Dict để giao tiếp giữa luồng chính và luồng đèn
            stop_event = threading.Event() # Event để dừng luồng đèn

            # Khởi động SUMO
            sumo_sim = SumoSim(sim_config)
            output_dir = os.path.join(project_root, "output")
            os.makedirs(output_dir, exist_ok=True)
            output_files = {"tripinfo": os.path.join(output_dir, "tripinfo.xml")}
            sumo_sim.start(output_files=output_files)

            # Khởi tạo bộ điều khiển chính
            controller = PerimeterController(
                kp=KP_H, ki=KI_H, n_hat=N_HAT, 
                config_file=intersection_config_path,
                shared_dict=shared_dict
            )

            # Bắt đầu luồng điều khiển đèn
            controller_thread = threading.Thread(
                target=traffic_light_controller, 
                args=(shared_dict, intersection_config_mgr, stop_event),
                name="LightController"
            )
            controller_thread.start()

            # --- 3. CHUẨN BỊ CHO VÒNG LẶP CHÍNH ---
            n_previous = 0
            qg_previous = 0
            
            # Biến lưu trữ dữ liệu thu thập được
            n_samples = []
            queue_samples = initialize_queue_samples(solver_detectors)
            
            # Biến lưu trữ dữ liệu đã được tổng hợp
            latest_aggregated_n = 0
            latest_aggregated_queue_lengths = {}

            # Lấy giá trị ban đầu
            sumo_sim.step()
            n_previous = get_sum_from_traci_detectors(algorithm_detector_ids)
            latest_aggregated_n = n_previous

            # Thiết lập các mốc thời gian cho các hành động
            next_sampling_time = 0
            next_aggregation_time = aggregation_interval_s
            next_control_time = CONTROL_INTERVAL_S
            next_log_time = 10

            logging.info("Khởi tạo hoàn tất. Bắt đầu vòng lặp mô phỏng chính.")

            # --- 4. VÒNG LẶP MÔ PHỎNG CHÍNH ---
            while traci.simulation.getMinExpectedNumber() > 0:
                sumo_sim.step()
                current_time = traci.simulation.getTime()

                # --- BƯỚC 1: THU THẬP DỮ LIỆU MẪU ---
                if current_time >= next_sampling_time:
                    n_samples.append(get_sum_from_traci_detectors(algorithm_detector_ids))

                    for int_id, details in solver_detectors.items():
                        phases = details.get('phases', {})
                        # Hàng đợi pha chính
                        p_detectors = phases.get('p', {}).get('queue_detectors', [])
                        p2_detectors = phases.get('p', {}).get('queue2_detector', [])
                        queue_samples[int_id]['p'].append(get_sum_from_traci_detectors(p_detectors))
                        # Hàng đợi các pha phụ
                        for i, s_phase in enumerate(phases.get('s', [])):
                            s_detectors = s_phase.get('queue_detectors', [])
                            s2_detectors = s_phase.get('queue2_detector', [])
                            if i < len(queue_samples[int_id]['s']):
                                queue_samples[int_id]['s'][i].append(get_sum_from_traci_detectors(s_detectors))
                    
                    next_sampling_time += sampling_interval_s

                # --- BƯỚC 2: TỔNG HỢP DỮ LIỆU ---
                if current_time >= next_aggregation_time:
                    logging.info(f"--- Tổng hợp dữ liệu tại t={current_time:.1f}s ---")
                    
                    if n_samples:
                        latest_aggregated_n = sum(n_samples) / len(n_samples)

                    for int_id, data in queue_samples.items():
                        avg_p = sum(data['p']) / len(data['p']) if data['p'] else 0
                        avg_s = [sum(s) / len(s) if s else 0 for s in data['s']]
                        latest_aggregated_queue_lengths[int_id] = {'p': avg_p, 's': avg_s}

                    logging.info(f"n(k) mới={latest_aggregated_n:.2f}. Xóa {len(n_samples)} mẫu.")
                    # logging.info(get_sum_from_traci_detectors(algorithm_detector_ids, flow_algorithm_detector))
                    clear_samples(n_samples, queue_samples)
                    next_aggregation_time += aggregation_interval_s

                # --- BƯỚC 3: CHẠY THUẬT TOÁN ĐIỀU KHIỂN ---
                if current_time >= next_control_time:
                    logging.info(f"--- Chạy điều khiển tại t={current_time:.1f}s ---")
                    
                    result = controller.run_simulation_step(
                        latest_aggregated_n, n_previous, qg_previous, latest_aggregated_queue_lengths
                    )
                    qg_previous = result.qg_new
                    n_previous = latest_aggregated_n
                    next_control_time += CONTROL_INTERVAL_S
                
                # Ghi log tiến độ và kiểm tra điều kiện dừng
                if current_time >= next_log_time:
                    logging.info(f"Thời gian: {current_time:.0f}s / {total_simulation_time}s")
                    next_log_time += 10

                if current_time >= total_simulation_time:
                    logging.info(f"Đạt thời gian mô phỏng tối đa. Dừng lại.")
                    break

    except (traci.TraCIException, traci.FatalTraCIError) as e:
        logging.warning(f"Kết nối Traci bị đóng hoặc mô phỏng kết thúc sớm: {e}")
    except Exception as e:
        logging.error(f"Lỗi không mong muốn trong quá trình chạy mô phỏng: {e}", exc_info=True)
    finally:
        # --- 5. DỌN DẸP VÀ KẾT THÚC ---
        logging.info("Dừng luồng điều khiển và đóng mô phỏng.")
        if 'stop_event' in locals() and stop_event:
            stop_event.set()
        if 'controller_thread' in locals() and controller_thread.is_alive():
            controller_thread.join()
        if 'sumo_sim' in locals() and sumo_sim.is_running():
            sumo_sim.close()
            logging.info(f"Mô phỏng kết thúc. Tổng số bước: {sumo_sim.get_step_counts()}")

if __name__ == "__main__":
    # Mặc định, chương trình sẽ chạy mô phỏng với SUMO.
    # Tùy chọn chạy 'mock' (thử nghiệm giả lập) hiện không được sử dụng.
    if len(sys.argv) > 1 and sys.argv[1] == 'mock':
        logging.warning("Chức năng 'mock' test hiện không có sẵn.")
    else:
        run_sumo_simulation()
