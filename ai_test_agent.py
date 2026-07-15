from __future__ import annotations

import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import serial
from sklearn.ensemble import IsolationForest


SERIAL_PORT = "COM5"
BAUD_RATE = 115200
COLLECTION_SECONDS = 60

OUTPUT_DIR = Path("test_results")
CSV_PATH = OUTPUT_DIR / "pid_test_data.csv"
REPORT_PATH = OUTPUT_DIR / "engineering_report.md"
CHART_PATH = OUTPUT_DIR / "pid_analysis.png"

EXPECTED_COLUMNS = [
    "time_ms",
    "s0",
    "s1",
    "s2",
    "s3",
    "s4",
    "error",
    "kp",
    "ki",
    "kd",
    "pid_value",
    "dynamic_speed",
    "left_pwm",
    "right_pwm",
    "line_lost",
    "valid_pattern",
]


def collect_serial_data() -> pd.DataFrame:
    """Thu thập dữ liệu CSV do ESP32 gửi qua Serial."""

    rows: list[list[str]] = []

    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")

    with serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=1,
    ) as connection:
        time.sleep(2)
        connection.reset_input_buffer()

        print(f"Collecting data for {COLLECTION_SECONDS} seconds...")

        start_time = time.time()

        while time.time() - start_time < COLLECTION_SECONDS:
            raw_line = connection.readline()

            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="ignore").strip()

            if not line or line.startswith("time_ms"):
                continue

            values = line.split(",")

            if len(values) != len(EXPECTED_COLUMNS):
                print(f"Skipped invalid row: {line}")
                continue

            rows.append(values)

    if not rows:
        raise RuntimeError("No valid telemetry was received from ESP32.")

    dataframe = pd.DataFrame(rows, columns=EXPECTED_COLUMNS)

    for column in EXPECTED_COLUMNS:
        dataframe[column] = pd.to_numeric(
            dataframe[column],
            errors="coerce",
        )

    dataframe = dataframe.dropna().reset_index(drop=True)

    if dataframe.empty:
        raise RuntimeError("Telemetry was received but could not be parsed.")

    return dataframe


def add_engineering_features(dataframe: pd.DataFrame) -> pd.DataFrame:
    """Tạo thêm đặc trưng phục vụ phân tích bất thường."""

    result = dataframe.copy()

    result["absolute_error"] = result["error"].abs()
    result["error_change"] = result["error"].diff().fillna(0)
    result["absolute_error_change"] = result["error_change"].abs()

    result["motor_difference"] = (
        result["left_pwm"] - result["right_pwm"]
    ).abs()

    result["pid_saturated"] = (
        result["pid_value"].abs() >= 79
    ).astype(int)

    return result


def detect_anomalies(dataframe: pd.DataFrame) -> pd.DataFrame:
    """Dùng Isolation Forest để phát hiện mẫu vận hành bất thường."""

    result = dataframe.copy()

    feature_columns = [
        "error",
        "absolute_error_change",
        "pid_value",
        "left_pwm",
        "right_pwm",
        "motor_difference",
    ]

    feature_data = result[feature_columns]

    if len(feature_data) < 20:
        result["anomaly"] = 0
        return result

    model = IsolationForest(
        n_estimators=200,
        contamination=0.05,
        random_state=42,
    )

    prediction = model.fit_predict(feature_data)

    # Isolation Forest trả -1 cho bất thường, 1 cho bình thường
    result["anomaly"] = (prediction == -1).astype(int)

    return result


def calculate_metrics(dataframe: pd.DataFrame) -> dict[str, float]:
    """Tính các chỉ số kỹ thuật của lần chạy."""

    error_values = dataframe["error"].to_numpy()

    mean_absolute_error = float(np.mean(np.abs(error_values)))
    rmse = float(np.sqrt(np.mean(np.square(error_values))))
    max_absolute_error = float(np.max(np.abs(error_values)))

    signs = np.sign(error_values)
    sign_changes = int(np.sum(signs[1:] * signs[:-1] < 0))

    oscillation_rate = (
        sign_changes / max(len(dataframe) - 1, 1)
    )

    line_lost_events = int(
        (
            dataframe["line_lost"].diff().fillna(0) == 1
        ).sum()
    )

    invalid_pattern_rate = float(
        1 - dataframe["valid_pattern"].mean()
    )

    pid_saturation_rate = float(
        dataframe["pid_saturated"].mean()
    )

    anomaly_rate = float(
        dataframe["anomaly"].mean()
    )

    return {
        "samples": float(len(dataframe)),
        "mean_absolute_error": mean_absolute_error,
        "rmse": rmse,
        "max_absolute_error": max_absolute_error,
        "oscillation_rate": oscillation_rate,
        "line_lost_events": float(line_lost_events),
        "invalid_pattern_rate": invalid_pattern_rate,
        "pid_saturation_rate": pid_saturation_rate,
        "anomaly_rate": anomaly_rate,
    }


def create_recommendations(metrics: dict[str, float]) -> list[str]:
    """Tạo đề xuất sơ bộ dựa trên chỉ số kiểm thử."""

    recommendations: list[str] = []

    if metrics["line_lost_events"] > 0:
        recommendations.append(
            "Kiểm tra độ cao và khoảng cách giữa dàn cảm biến với mặt đường; "
            "xem lại ngưỡng phát hiện line và logic xử lý mẫu 00000/11111."
        )

    if metrics["oscillation_rate"] > 0.25:
        recommendations.append(
            "Xe có dấu hiệu đổi hướng liên tục. Thử giảm Kp khoảng 5–10% "
            "hoặc tăng Kd nhẹ, mỗi lần chỉ thay đổi một thông số."
        )

    if metrics["pid_saturation_rate"] > 0.20:
        recommendations.append(
            "PID thường xuyên đạt giới hạn ±80. Kiểm tra lại Kp, Kd hoặc "
            "tăng giới hạn điều khiển nếu phần cứng động cơ cho phép."
        )

    if metrics["mean_absolute_error"] > 1.5:
        recommendations.append(
            "Sai số bám line trung bình còn cao. Kiểm tra lại cách quy đổi "
            "mẫu cảm biến thành vị trí và thực hiện thêm các lần chạy tinh chỉnh."
        )

    if metrics["invalid_pattern_rate"] > 0.10:
        recommendations.append(
            "Tỷ lệ mẫu cảm biến chưa được nhận dạng cao. Bổ sung xử lý cho "
            "các tổ hợp cảm biến chưa có trong bảng ánh xạ."
        )

    if not recommendations:
        recommendations.append(
            "Không phát hiện vấn đề nổi bật. Tiếp tục kiểm thử trên đường cua "
            "gắt, tốc độ cao và điều kiện ánh sáng khác nhau."
        )

    return recommendations


def create_chart(dataframe: pd.DataFrame) -> None:
    """Tạo biểu đồ sai số, PID và tốc độ hai động cơ."""

    time_seconds = (
        dataframe["time_ms"] - dataframe["time_ms"].iloc[0]
    ) / 1000.0

    plt.figure(figsize=(11, 7))

    plt.plot(
        time_seconds,
        dataframe["error"],
        label="Position error",
    )

    plt.plot(
        time_seconds,
        dataframe["pid_value"],
        label="PID output",
        alpha=0.7,
    )

    anomaly_rows = dataframe[dataframe["anomaly"] == 1]

    if not anomaly_rows.empty:
        anomaly_time = (
            anomaly_rows["time_ms"] - dataframe["time_ms"].iloc[0]
        ) / 1000.0

        plt.scatter(
            anomaly_time,
            anomaly_rows["error"],
            label="AI anomaly",
            marker="x",
        )

    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.title("PID Line-Following Test Analysis")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(CHART_PATH, dpi=160)
    plt.close()


def generate_report(
    dataframe: pd.DataFrame,
    metrics: dict[str, float],
    recommendations: list[str],
) -> None:
    """Tự động tạo báo cáo kỹ thuật dạng Markdown."""

    kp = dataframe["kp"].iloc[0]
    ki = dataframe["ki"].iloc[0]
    kd = dataframe["kd"].iloc[0]

    recommendation_text = "\n".join(
        f"- {item}" for item in recommendations
    )

    report = f"""# PID Line-Following Engineering Test Report

## 1. Test configuration

- Number of samples: {int(metrics["samples"])}
- PID parameters: Kp = {kp:.2f}, Ki = {ki:.2f}, Kd = {kd:.2f}
- Base motor speed: {int(dataframe["dynamic_speed"].max())}
- Sensor array: 5 digital infrared sensors

## 2. Performance metrics

- Mean absolute error: {metrics["mean_absolute_error"]:.3f}
- Root mean square error: {metrics["rmse"]:.3f}
- Maximum absolute error: {metrics["max_absolute_error"]:.3f}
- Oscillation rate: {metrics["oscillation_rate"] * 100:.2f}%
- PID saturation rate: {metrics["pid_saturation_rate"] * 100:.2f}%
- Invalid sensor pattern rate: {metrics["invalid_pattern_rate"] * 100:.2f}%
- AI anomaly rate: {metrics["anomaly_rate"] * 100:.2f}%
- Line-loss events: {int(metrics["line_lost_events"])}

## 3. AI-assisted findings

The anomaly detection model evaluated position error, error variation,
PID output and motor PWM commands to identify unusual operating samples.

Detected anomalous samples: {int(dataframe["anomaly"].sum())}

## 4. Engineering recommendations

{recommendation_text}

## 5. Generated artifacts

- Raw test data: `{CSV_PATH.name}`
- Analysis chart: `{CHART_PATH.name}`
- Engineering report: `{REPORT_PATH.name}`

> Recommendations are preliminary and must be verified through controlled
> test runs before changing the vehicle configuration.
"""

    REPORT_PATH.write_text(report, encoding="utf-8")


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    dataframe = collect_serial_data()
    dataframe = add_engineering_features(dataframe)
    dataframe = detect_anomalies(dataframe)

    dataframe.to_csv(CSV_PATH, index=False)

    metrics = calculate_metrics(dataframe)
    recommendations = create_recommendations(metrics)

    create_chart(dataframe)
    generate_report(dataframe, metrics, recommendations)

    print("\nAnalysis completed.")
    print(f"CSV: {CSV_PATH}")
    print(f"Chart: {CHART_PATH}")
    print(f"Report: {REPORT_PATH}")

    print("\nSummary:")
    print(f"- Mean absolute error: {metrics['mean_absolute_error']:.3f}")
    print(f"- RMSE: {metrics['rmse']:.3f}")
    print(f"- Line-loss events: {int(metrics['line_lost_events'])}")
    print(f"- AI anomaly rate: {metrics['anomaly_rate'] * 100:.2f}%")


if __name__ == "__main__":
    main()
