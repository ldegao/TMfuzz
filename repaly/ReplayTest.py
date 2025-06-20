import glob
import json
import os
import shutil
import socket
import subprocess
import sys
import time
import traceback
from datetime import datetime


def test_all_logs_in_folder(root_dir, out_dir, test_dir="/home/linshenghao/carla_data/"):
    os.makedirs(out_dir, exist_ok=True)
    log_file_path = os.path.join(out_dir, f"replay_test_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
    log_file = open(log_file_path, "a", buffering=1)
    summary = None

    def log(msg):
        print(msg)
        log_file.write(msg + "\n")

    total_logs = 0
    with_collision = 0
    successful_replay = 0
    no_collision = 0
    errors_by_type = {}
    for dirpath, _, filenames in os.walk(root_dir):
        for fname in filenames:
            if fname.endswith(".log"):
                total_logs += 1
                src_path = os.path.join(dirpath, fname)
                dst_path = os.path.join(test_dir, fname)
                shutil.copyfile(src_path, dst_path)

                try:
                    if not wait_for_carla_ready(timeout=15, interval=2):
                        restart_carla(log)

                    run_script = os.path.join(os.path.dirname(__file__), "replay.py")
                    result_path = os.path.join(out_dir, "replay_result.json")

                    if os.path.exists(result_path):
                        os.remove(result_path)

                    proc = subprocess.run(
                        ["python3", run_script, fname, out_dir],
                        stdout=None,
                        stderr=None,
                        timeout=300
                    )

                    if proc.returncode != 0:
                        log(f"[ERROR] Subprocess failed for {fname}, code={proc.returncode}")
                        result = {
                            "replay_result": False,
                            "error_type": f"Subprocess error {proc.returncode}"
                        }
                    elif not os.path.exists(result_path):
                        log(f"[ERROR] No result.json produced by subprocess for {fname}")
                        result = {
                            "replay_result": False,
                            "error_type": "No result file"
                        }
                    else:
                        try:
                            with open(result_path, "r") as f:
                                result = json.load(f)
                        except Exception as e:
                            result = {
                                "replay_result": False,
                                "error_type": f"Invalid JSON file: {str(e)}"
                            }

                    if result["replay_result"]:
                        with_collision += 1
                    else:
                        no_collision += 1

                    if result.get("error_type"):
                        err = result["error_type"].strip().split('\n')[0]
                        errors_by_type[err] = errors_by_type.get(err, 0) + 1
                    else:
                        successful_replay += 1

                except Exception as e:
                    msg = f"Unhandled error: {str(e)}"
                    errors_by_type[msg] = errors_by_type.get(msg, 0) + 1
                    traceback.print_exc(file=sys.stdout)
                    log(f"[EXCEPTION] {fname} - {msg}")
                finally:
                    if os.path.exists(dst_path):
                        os.remove(dst_path)

                rate_before = f"{successful_replay / total_logs * 100:.2f}%" if total_logs else "N/A"
                rate_still = f"{with_collision / successful_replay * 100:.2f}%" if successful_replay else "N/A"
                summary = (
                    f"\n=== Progress Update ===\n"
                    f"Processed logs: {total_logs}\n"
                    f"With collision: {with_collision}\n"
                    f"Without collision: {no_collision}\n"
                    f"Collision before: {successful_replay}\n"
                    f"Collision before rate: {rate_before}\n"
                    f"Collision still rate: {rate_still}\n"
                    f"=== Error Breakdown ===\n"
                )
                for err_type, count in errors_by_type.items():
                    summary += f"- {err_type}: {count}\n"

                log(summary)

    log("\n=== Final Summary ===\n" + summary)
    log_file.close()


def restart_carla(log_fn):
    """Stop and restart Carla server."""
    stop_script = os.path.expanduser("~/drivefuzz/TM-fuzzer/script/stop_carla.sh")
    start_script = os.path.expanduser("~/drivefuzz/TM-fuzzer/script/screen_run_carla.sh")

    subprocess.call(["bash", stop_script])
    log_fn("[INFO] Called stop_carla.sh")
    time.sleep(2)

    subprocess.Popen(["bash", start_script])
    time.sleep(30)
    log_fn("[INFO] Called screen_run_carla.sh")


def wait_for_carla_ready(timeout=15, interval=2):
    """Wait until Carla server is reachable via port 4000."""
    waited = 0
    while waited < timeout:
        if is_port_open("localhost", 4000):
            print(f"[INFO] Carla port 4000 is now open after waiting {waited} seconds.")
            return True
        time.sleep(interval)
        waited += interval
        print(f"[INFO] Waiting for Carla to open port 4000... ({waited}/{timeout}s)")
    print(f"[ERROR] Carla port 4000 not open after {timeout} seconds.")
    return False


def is_port_open(host: str, port: int) -> bool:
    """Check if a port is open (used to verify Carla server readiness)."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(2.0)
        try:
            sock.connect((host, port))
            return True
        except:
            return False


if __name__ == '__main__':
    # root_dir = "/home/linshenghao/drivefuzz/TM-fuzzer/data/save/20250605203343/logs/"
    root_dir = "/home/linshenghao/drivefuzz/save_autoware_6_20/"
    out_dir = "/home/linshenghao/drivefuzz/save_autoware_6_20/result"
    test_all_logs_in_folder(root_dir, out_dir)
