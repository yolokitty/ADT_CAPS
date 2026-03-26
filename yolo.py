import cv2
import serial
import time
import numpy as np
import os
from collections import deque
from datetime import datetime
from ultralytics import YOLO

# -----------------------
# 1. Serial 설정
# -----------------------
try:
    ser = serial.Serial(
        port="/dev/ttyACM1",
        baudrate=115200,
        timeout=0.1,
        exclusive=True
    )
    time.sleep(2.0)
    print(f"Serial Connected! port={ser.port}, baud={ser.baudrate}")
except Exception as e:
    print(f"Serial Error: {e}")
    raise SystemExit

# -----------------------상세 
# 2. Model 설정
# -----------------------
# 박스 전용 커스텀 모델
model_parcel = YOLO("/home/ming/project/contest_project/YOLO_Project/runs/train/weights/best.pt")

# 사람 검출용 일반 모델
model_person = YOLO("yolo11n.pt")

# -----------------------
# 3. Camera 설정
# -----------------------
# Cam1: 박스용
cap_box = cv2.VideoCapture(2)

# Cam2: 사람용
cap_person = cv2.VideoCapture(4)

for cap in [cap_box, cap_person]:
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap_box.isOpened():
    print("Box camera (/dev/video2) open failed.")
    ser.close()
    raise SystemExit

if not cap_person.isOpened():
    print("Person camera (/dev/video4) open failed.")
    cap_box.release()
    ser.close()
    raise SystemExit

print("Box cam actual size:",
      int(cap_box.get(cv2.CAP_PROP_FRAME_WIDTH)),
      int(cap_box.get(cv2.CAP_PROP_FRAME_HEIGHT)))
print("Person cam actual size:",
      int(cap_person.get(cv2.CAP_PROP_FRAME_WIDTH)),
      int(cap_person.get(cv2.CAP_PROP_FRAME_HEIGHT)))

# -----------------------
# 3-1. Recording 설정 (박스 1->0 후 4 수신 이벤트용)
# -----------------------
VIDEO_SAVE_DIR = "/home/ming/project/contest_project/videos"
os.makedirs(VIDEO_SAVE_DIR, exist_ok=True)

record_fps = 30.0
record_size_box = None
record_size_person = None

pre_record_sec = 5.0
post_record_sec = 5.0

pre_event_buffer_box = deque(maxlen=int(record_fps * pre_record_sec))
pre_event_buffer_person = deque(maxlen=int(record_fps * pre_record_sec))

recording_active = False
recording_end_time = 0.0
video_writer_box = None
video_writer_person = None
recording_tag = None

# -----------------------
# 3-2. Person Recording 설정 (사람 이벤트 전용)
# -----------------------
person_recording_active = False
person_video_writer_box = None
person_video_writer_person = None
person_recording_tag = None

person_record_box_path = None
person_record_person_path = None

person_detect_start_time = None
person_event3_sent = False
person_min_valid_time = 30.0   # 30초 이상 있어야 3 전송 / 영상 유지
person_last_seen = None
person_lost_timeout = 1.0      # 사람이 1초 정도 안 보여도 바로 종료하지 않도록 완충

# -----------------------
# 4. 상태 변수
# -----------------------
last_send_time = 0.0
min_send_interval = 0.3

# ---- 박스 상태 ----
parcel_seen_start = None
parcel_last_seen = None
parcel_confirmed = False

prev_parcel_state = None

parcel_hold_time = 5.0
parcel_lost_timeout = 1.0
parcel_release_time = 2.5

# ---- 사람 상태 ----
# 사람은 기존 confirmed 방식 대신 세션 기반으로 처리
# prev_person_state도 더 이상 사용하지 않음

# ---- 녹화 트리거 대기 상태 ----
# parcel_state가 1 -> 0이 된 뒤,
# STM으로부터 문자 '4'를 수신하면 그때 박스 이벤트 녹화 시작
record_waiting_for_trigger = False

# -----------------------
# 5. 갈색 비율 계산
# -----------------------
def brown_ratio_hsv(roi):
    if roi is None or roi.size == 0:
        return 0.0

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 일반 갈색
    lower1 = np.array([5, 40, 40], dtype=np.uint8)
    upper1 = np.array([25, 255, 255], dtype=np.uint8)
    mask1 = cv2.inRange(hsv, lower1, upper1)

    # 어두운 갈색 보완
    lower2 = np.array([0, 30, 20], dtype=np.uint8)
    upper2 = np.array([30, 255, 200], dtype=np.uint8)
    mask2 = cv2.inRange(hsv, lower2, upper2)

    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    ratio = float(np.count_nonzero(mask)) / float(mask.size)
    return ratio

# -----------------------
# 6. 갈색 박스 판별
# -----------------------
def is_brown_box(frame, xyxy, min_ratio=0.12):
    x1, y1, x2, y2 = map(int, xyxy)
    h, w = frame.shape[:2]

    x1 = max(0, min(x1, w - 1))
    x2 = max(0, min(x2, w - 1))
    y1 = max(0, min(y1, h - 1))
    y2 = max(0, min(y2, h - 1))

    if x2 <= x1 or y2 <= y1:
        return False, 0.0

    roi = frame[y1:y2, x1:x2]
    ratio = brown_ratio_hsv(roi)

    return (ratio >= min_ratio), ratio

# -----------------------
# 7. 최적 박스 선택
# -----------------------
def get_best_parcel_detection(results, frame, conf_th=0.35, min_brown_ratio=0.12):
    if results.boxes is None or len(results.boxes) == 0:
        return None

    fh, fw = frame.shape[:2]
    candidates = []

    for b in results.boxes:
        conf = float(b.conf[0])
        if conf < conf_th:
            continue

        xyxy = b.xyxy[0].tolist()

        ok_brown, brown_ratio = is_brown_box(frame, xyxy, min_ratio=min_brown_ratio)
        if not ok_brown:
            continue

        box_w = max(0.0, xyxy[2] - xyxy[0])
        box_h = max(0.0, xyxy[3] - xyxy[1])
        area_ratio = (box_w * box_h) / (fh * fw + 1e-6)

        if area_ratio < 0.02:
            continue

        score = conf * 0.60 + brown_ratio * 0.25 + area_ratio * 0.15
        candidates.append((score, xyxy, conf, brown_ratio, area_ratio))

    if not candidates:
        return None

    return max(candidates, key=lambda x: x[0])

# -----------------------
# 8. 사람 검출
# -----------------------
def get_best_person_detection(results, conf_th=0.40):
    if results.boxes is None or len(results.boxes) == 0:
        return None

    candidates = []

    for b in results.boxes:
        conf = float(b.conf[0])
        cls = int(b.cls[0])

        if cls != 0:
            continue
        if conf < conf_th:
            continue

        xyxy = b.xyxy[0].tolist()
        candidates.append((conf, xyxy))

    if not candidates:
        return None

    return max(candidates, key=lambda x: x[0])

# -----------------------
# 9. Serial 전송 함수
# -----------------------
def send_flag(flag):
    """
    0: 상자 없음
    1: 상자 있음
    2: 사람 없음(사람 세션 종료)
    3: 사람 1분 이상 유지
    """
    ser.write(str(flag).encode("ascii"))

# -----------------------
# 9-1. Serial 수신 처리 함수
# -----------------------
def process_incoming_serial():
    """
    STM을 통해 들어오는 값을 읽어서 터미널에 출력하고,
    문자 '4'를 수신하면 armed 상태일 때만 박스 이벤트 녹화 시작
    """
    global record_waiting_for_trigger

    try:
        waiting = ser.in_waiting
        if waiting <= 0:
            return

        data = ser.read(waiting)
        if not data:
            return

        try:
            text = data.decode("ascii", errors="ignore")
        except Exception:
            text = ""

        if text:
            print(f"[STM RX] raw='{text}' bytes={list(data)}")
        else:
            print(f"[STM RX] bytes={list(data)}")

        for b in data:
            if b == ord('4'):
                print("[STM RX] Trigger '4' received")

                if record_waiting_for_trigger:
                    print("[REC] armed state + trigger 4 -> start recording")
                    start_zero_event_recording()
                    record_waiting_for_trigger = False
                else:
                    print("[REC] trigger 4 received, but recording is not armed")

    except Exception as e:
        print(f"[STM RX ERROR] {e}")

# -----------------------
# 9-2. Recording 보조 함수 (박스 이벤트용)
# -----------------------
def ensure_frame_size(frame, target_size):
    if frame is None or target_size is None:
        return None

    target_w, target_h = target_size
    h, w = frame.shape[:2]

    out = frame

    if (w, h) != (target_w, target_h):
        out = cv2.resize(out, (target_w, target_h))

    if out.dtype != np.uint8:
        out = out.astype(np.uint8)

    out = np.ascontiguousarray(out)
    return out

def stop_recording():
    global recording_active
    global video_writer_box, video_writer_person

    if video_writer_box is not None:
        video_writer_box.release()
        video_writer_box = None

    if video_writer_person is not None:
        video_writer_person.release()
        video_writer_person = None

    if recording_active:
        print("Recording finished.")

    recording_active = False

def start_zero_event_recording():
    global recording_active, recording_end_time
    global video_writer_box, video_writer_person, recording_tag
    global record_size_box, record_size_person

    now = time.time()

    if recording_active:
        recording_end_time = now + post_record_sec
        print("Recording already active -> extend post-record time")
        return

    if len(pre_event_buffer_box) == 0 or len(pre_event_buffer_person) == 0:
        print("Pre-event buffer is empty. Skip recording start.")
        return

    sample_box = pre_event_buffer_box[-1]
    sample_person = pre_event_buffer_person[-1]

    h_box, w_box = sample_box.shape[:2]
    h_person, w_person = sample_person.shape[:2]

    record_size_box = (w_box, h_box)
    record_size_person = (w_person, h_person)

    recording_tag = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    box_path = os.path.join(VIDEO_SAVE_DIR, f"{recording_tag}_cam_box.mp4")
    person_path = os.path.join(VIDEO_SAVE_DIR, f"{recording_tag}_cam_person.mp4")

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    video_writer_box = cv2.VideoWriter(box_path, fourcc, record_fps, record_size_box)
    video_writer_person = cv2.VideoWriter(person_path, fourcc, record_fps, record_size_person)

    if not video_writer_box.isOpened() or not video_writer_person.isOpened():
        print("VideoWriter open failed.")
        print(f"box writer size: {record_size_box}, person writer size: {record_size_person}")

        if video_writer_box is not None:
            video_writer_box.release()
        if video_writer_person is not None:
            video_writer_person.release()

        video_writer_box = None
        video_writer_person = None
        recording_active = False
        return

    for frame in pre_event_buffer_box:
        frame_to_write = ensure_frame_size(frame, record_size_box)
        if frame_to_write is not None:
            video_writer_box.write(frame_to_write)

    for frame in pre_event_buffer_person:
        frame_to_write = ensure_frame_size(frame, record_size_person)
        if frame_to_write is not None:
            video_writer_person.write(frame_to_write)

    recording_active = True
    recording_end_time = now + post_record_sec

    print("Recording started around flag 0 event")
    print(f"Saved to: {box_path}")
    print(f"Saved to: {person_path}")
    print(f"Box record size: {record_size_box}, Person record size: {record_size_person}")

def update_recording(frame_box, frame_person):
    global recording_active, recording_end_time
    global video_writer_box, video_writer_person
    global record_size_box, record_size_person

    if not recording_active:
        return

    if video_writer_box is not None:
        frame_box_w = ensure_frame_size(frame_box, record_size_box)
        if frame_box_w is not None:
            video_writer_box.write(frame_box_w)

    if video_writer_person is not None:
        frame_person_w = ensure_frame_size(frame_person, record_size_person)
        if frame_person_w is not None:
            video_writer_person.write(frame_person_w)

    if time.time() >= recording_end_time:
        stop_recording()

# -----------------------
# 9-3. Person Recording 보조 함수
# -----------------------
def start_person_recording(frame_box, frame_person):
    global person_recording_active
    global person_video_writer_box, person_video_writer_person
    global person_recording_tag
    global person_record_box_path, person_record_person_path
    global person_detect_start_time, person_event3_sent

    if person_recording_active:
        return

    h_box, w_box = frame_box.shape[:2]
    h_person, w_person = frame_person.shape[:2]

    person_recording_tag = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    person_record_box_path = os.path.join(
        VIDEO_SAVE_DIR, f"{person_recording_tag}_person_event_cam_box.mp4"
    )
    person_record_person_path = os.path.join(
        VIDEO_SAVE_DIR, f"{person_recording_tag}_person_event_cam_person.mp4"
    )

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    person_video_writer_box = cv2.VideoWriter(
        person_record_box_path, fourcc, record_fps, (w_box, h_box)
    )
    person_video_writer_person = cv2.VideoWriter(
        person_record_person_path, fourcc, record_fps, (w_person, h_person)
    )

    if not person_video_writer_box.isOpened() or not person_video_writer_person.isOpened():
        print("[PERSON REC] VideoWriter open failed.")

        if person_video_writer_box is not None:
            person_video_writer_box.release()
        if person_video_writer_person is not None:
            person_video_writer_person.release()

        person_video_writer_box = None
        person_video_writer_person = None
        person_recording_active = False
        return

    person_recording_active = True
    person_detect_start_time = time.time()
    person_event3_sent = False

    print("[PERSON REC] recording started")
    print(f"[PERSON REC] box path: {person_record_box_path}")
    print(f"[PERSON REC] person path: {person_record_person_path}")

def update_person_recording(frame_box, frame_person):
    global person_recording_active
    global person_video_writer_box, person_video_writer_person

    if not person_recording_active:
        return

    if person_video_writer_box is not None:
        person_video_writer_box.write(frame_box)

    if person_video_writer_person is not None:
        person_video_writer_person.write(frame_person)

def stop_person_recording(delete_if_short=False):
    global person_recording_active
    global person_video_writer_box, person_video_writer_person
    global person_record_box_path, person_record_person_path

    if person_video_writer_box is not None:
        person_video_writer_box.release()
        person_video_writer_box = None

    if person_video_writer_person is not None:
        person_video_writer_person.release()
        person_video_writer_person = None

    if person_recording_active:
        print("[PERSON REC] recording stopped")

    person_recording_active = False

    if delete_if_short:
        for path in [person_record_box_path, person_record_person_path]:
            try:
                if path is not None and os.path.exists(path):
                    os.remove(path)
                    print(f"[PERSON REC] deleted short recording: {path}")
            except Exception as e:
                print(f"[PERSON REC] delete failed: {path}, error={e}")

    person_record_box_path = None
    person_record_person_path = None

# -----------------------
# 10. Main Loop
# -----------------------
print("Dual camera stable detection started. Press 'q' to quit.")

while True:
    ret_box, frame_box = cap_box.read()
    ret_person, frame_person = cap_person.read()

    if not ret_box:
        print("Box camera frame read failed.")
        break

    if not ret_person:
        print("Person camera frame read failed.")
        break

    # STM에서 들어오는 값 확인
    process_incoming_serial()

    now = time.time()

    # 이벤트 전 프레임 버퍼 저장 (박스 이벤트용)
    pre_event_buffer_box.append(frame_box.copy())
    pre_event_buffer_person.append(frame_person.copy())

    # -----------------------
    # Cam1: 박스 검출
    # -----------------------
    results_box = model_parcel(frame_box, conf=0.30, imgsz=320, verbose=False)[0]
    best_parcel = get_best_parcel_detection(
        results_box,
        frame_box,
        conf_th=0.35,
        min_brown_ratio=0.12
    )
    parcel_detected = (best_parcel is not None)

    # -----------------------
    # Cam2: 사람 검출
    # -----------------------
    results_person = model_person(frame_person, conf=0.30, imgsz=320, verbose=False)[0]
    best_person = get_best_person_detection(results_person, conf_th=0.40)
    person_detected_raw = (best_person is not None)

    if person_detected_raw:
        person_last_seen = now

    # 사람 사라짐 판정을 바로 하지 않고 lost timeout 적용
    person_detected = False
    if person_detected_raw:
        person_detected = True
    elif person_last_seen is not None and (now - person_last_seen) <= person_lost_timeout:
        person_detected = True

    # -----------------------
    # 상자 안정화 로직
    # -----------------------
    if parcel_detected:
        parcel_last_seen = now

        if not parcel_confirmed:
            if parcel_seen_start is None:
                parcel_seen_start = now

            elapsed = now - parcel_seen_start
            if elapsed >= parcel_hold_time:
                parcel_confirmed = True
                print("Box confirmed -> state ON")

    else:
        if parcel_last_seen is not None:
            lost_duration = now - parcel_last_seen

            if not parcel_confirmed:
                if lost_duration > parcel_lost_timeout:
                    parcel_seen_start = None
                    parcel_last_seen = None
            else:
                if lost_duration > parcel_release_time:
                    parcel_seen_start = None
                    parcel_last_seen = None
                    parcel_confirmed = False
                    print("Box released -> state OFF")

    # -----------------------
    # 사람 즉시 녹화 / 1분 유지 시 3 전송 / 사라지면 종료+2
    # -----------------------
    if person_detected:
        if not person_recording_active:
            start_person_recording(frame_box, frame_person)

        if person_detect_start_time is not None:
            elapsed_person = now - person_detect_start_time

            if (elapsed_person >= person_min_valid_time) and (not person_event3_sent):
                now_send = time.time()
                if (now_send - last_send_time) < min_send_interval:
                    time.sleep(min_send_interval - (now_send - last_send_time))

                send_flag(3)
                print("Sent person event: 3")
                person_event3_sent = True
                last_send_time = time.time()

    else:
        if person_recording_active:
            elapsed_person = 0.0
            if person_detect_start_time is not None:
                elapsed_person = now - person_detect_start_time

            delete_short = elapsed_person < person_min_valid_time
            stop_person_recording(delete_if_short=delete_short)

            now_send = time.time()
            if (now_send - last_send_time) < min_send_interval:
                time.sleep(min_send_interval - (now_send - last_send_time))

            send_flag(2)
            print("Sent person event: 2")
            last_send_time = time.time()

            person_detect_start_time = None
            person_event3_sent = False
            person_last_seen = None

    # -----------------------
    # 상태 변화 이벤트 전송 (박스만 유지)
    # -----------------------
    parcel_state = parcel_confirmed

    # 초기 상태 1회 세팅 (박스만)
    if prev_parcel_state is None:
        prev_parcel_state = parcel_state
        send_flag(1 if parcel_state else 0)
        print(f"Sent init parcel state: {1 if parcel_state else 0}")
        last_send_time = now

    # 상자 상태 변화 전송
    if parcel_state != prev_parcel_state:
        now_send = time.time()
        if (now_send - last_send_time) < min_send_interval:
            time.sleep(min_send_interval - (now_send - last_send_time))

        # 1 -> 0 이 되면 즉시 녹화하지 않고, 4 수신 대기 상태로 전환
        if prev_parcel_state is True and parcel_state is False:
            record_waiting_for_trigger = True
            print("[REC] parcel 1 -> 0 detected, waiting for trigger '4'")

        # 다시 0 -> 1 로 복귀하면 기존 대기 상태 취소
        elif prev_parcel_state is False and parcel_state is True:
            if record_waiting_for_trigger:
                print("[REC] parcel returned to 1, cancel waiting for trigger")
            record_waiting_for_trigger = False

        send_flag(1 if parcel_state else 0)
        print(f"Sent parcel event: {1 if parcel_state else 0}")
        prev_parcel_state = parcel_state
        last_send_time = time.time()

    # -----------------------
    # 녹화 프레임 저장
    # -----------------------
    # 박스 이벤트 녹화 진행 중이면 현재 프레임 저장
    update_recording(frame_box, frame_person) 

    # 사람 이벤트 녹화 진행 중이면 현재 프레임 저장
    update_person_recording(frame_box, frame_person)

    # -----------------------
    # Visualization - Box Cam
    # -----------------------
    display_box = results_box.plot()

    if best_parcel is not None:
        xyxy = best_parcel[1]
        conf = best_parcel[2]
        brown_ratio = best_parcel[3]
        area_ratio = best_parcel[4]

        x1, y1, x2, y2 = map(int, xyxy)

        cv2.rectangle(display_box, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.putText(
            display_box,
            f"BOX CONF:{conf:.2f} BROWN:{brown_ratio:.2f} AREA:{area_ratio:.2f}",
            (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2
        )

    if parcel_seen_start is not None and not parcel_confirmed:
        elapsed = now - parcel_seen_start
        remain = max(0.0, parcel_hold_time - elapsed)

        cv2.putText(
            display_box,
            f"BOX HOLDING {elapsed:.1f}s / {parcel_hold_time:.1f}s",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )
        cv2.putText(
            display_box,
            f"BOX REMAIN {remain:.1f}s",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )

    if not parcel_detected and parcel_last_seen is not None:
        lost_duration = now - parcel_last_seen

        if not parcel_confirmed and lost_duration <= parcel_lost_timeout:
            cv2.putText(
                display_box,
                f"BOX TEMP LOST {lost_duration:.2f}s / {parcel_lost_timeout:.2f}s",
                (20, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 165, 255),
                2
            )

        if parcel_confirmed and lost_duration <= parcel_release_time:
            cv2.putText(
                display_box,
                f"BOX KEEP ON {lost_duration:.2f}s / {parcel_release_time:.2f}s",
                (20, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 0),
                2
            )

    if parcel_confirmed:
        cv2.putText(
            display_box,
            "BOX CONFIRMED -> EVENT 1",
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
    else:
        cv2.putText(
            display_box,
            "BOX OFF -> EVENT 0",
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

    cv2.putText(
        display_box,
        f"BOX STATE: {'ON' if parcel_confirmed else 'OFF'}",
        (20, 220),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )

    if recording_active:
        cv2.putText(
            display_box,
            "BOX EVENT RECORDING...",
            (140, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 0, 255),
            2
        )

    if record_waiting_for_trigger:
        cv2.putText(
            display_box,
            "WAITING FOR TRIGGER '4'",
            (20, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 0),
            2
        )

    if person_recording_active:
        cv2.putText(
            display_box,
            "PERSON RECORDING...",
            (140, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 0, 255),
            2
        )

    # -----------------------
    # Visualization - Person Cam
    # -----------------------
    display_person = results_person.plot()

    if best_person is not None:
        conf = best_person[0]
        xyxy = best_person[1]
        x1, y1, x2, y2 = map(int, xyxy)

        cv2.rectangle(display_person, (x1, y1), (x2, y2), (255, 0, 0), 3)
        cv2.putText(
            display_person,
            f"PERSON CONF:{conf:.2f}",
            (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2
        )

    if person_recording_active:
        elapsed_person = 0.0
        if person_detect_start_time is not None:
            elapsed_person = now - person_detect_start_time

        cv2.putText(
            display_person,
            f"PERSON RECORDING {elapsed_person:.1f}s / {person_min_valid_time:.1f}s",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 255),
            2
        )

        if person_event3_sent:
            status_text = "PERSON VALID -> EVENT 3 SENT"
            status_color = (255, 0, 0)
        else:
            remain_person = max(0.0, person_min_valid_time - elapsed_person)
            status_text = f"WAIT EVENT 3 / REMAIN {remain_person:.1f}s"
            status_color = (0, 255, 255)

        cv2.putText(
            display_person,
            status_text,
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            status_color,
            2
        )
    else:
        cv2.putText(
            display_person,
            "PERSON OFF -> EVENT 2",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (0, 0, 255),
            2
        )

    if recording_active:
        cv2.putText(
            display_person,
            "BOX EVENT RECORDING...",
            (120, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 0, 255),
            2
        )

    if person_recording_active:
        cv2.putText(
            display_person,
            "PERSON RECORDING...",
            (120, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 0, 255),
            2
        )

    cv2.putText(
        display_person,
        f"PERSON STATE: {'RECORDING' if person_recording_active else 'OFF'}",
        (20, 220),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )

    # -----------------------
    # 화면 출력
    # -----------------------
    cv2.imshow("Cam1 - Box Detection", display_box)
    cv2.imshow("Cam2 - Person Detection", display_person)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# -----------------------
# 종료 처리
# -----------------------
stop_recording()
stop_person_recording(delete_if_short=False)
cap_box.release()
cap_person.release()
cv2.destroyAllWindows()
ser.close()
print("Program terminated.")

