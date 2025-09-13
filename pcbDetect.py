import cv2
import serial, time
from flask import Flask, render_template, Response
from ultralytics import YOLO
import os
import threading

# Open camera (try video0, fallback video1)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    cap = cv2.VideoCapture(1, cv2.CAP_V4L2)

MODEL_PATH = "best.pt"
model = YOLO(MODEL_PATH, task="detect")

app = Flask(__name__)
ser = serial.Serial("/dev/serial0", baudrate=115200, timeout=1)

class_names = ["normal", "burnt", "oversolder"]
conf_threshold = 0.5

last_frame = None   # global frame for both Flask and imshow
running = True

def send_message(msg):
    ser.write((msg + "\r\n").encode("utf-8"))
    ser.flush()
    print(f"Sent: {msg}")

def camera_loop():
    global last_frame, running
    while running:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame")
            break

        results = model(frame)
        result = results[0]
        annotated_frame = result.plot()

        for box in result.boxes:
            conf = float(box.conf[0])
            if conf < conf_threshold:
                continue
            cls_id = int(box.cls[0])
            if cls_id < len(class_names):
                label = class_names[cls_id]
                send_message(label)

        # Non-blocking serial read
        if ser.in_waiting > 0:
            reply = ser.readline().decode("utf-8", errors="ignore").strip()
            if reply:
                print(f"Received from ESP32: {reply}")

        # Show in OpenCV window
        cv2.imshow("Webcam feed", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False
            break

        # Keep latest frame for Flask
        ret, buffer = cv2.imencode(".jpg", annotated_frame)
        if ret:
            last_frame = buffer.tobytes()

        time.sleep(0.05)

    cap.release()
    cv2.destroyAllWindows()


def gen_frames():
    global last_frame, running
    while running:
        if last_frame:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + last_frame + b"\r\n")
        time.sleep(0.1)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video")
def video():
    return Response(gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    # Start camera thread
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()

    try:
        app.run(port=int(os.environ.get("PORT", 8080)),
                host="0.0.0.0", debug=True, use_reloader=False)
    finally:
        running = False
        t.join()

