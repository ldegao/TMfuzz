import os
import csv
import json
import httpx
from tqdm import tqdm
from openai import OpenAI, OpenAIError
import sys

try:
    import tiktoken
except ImportError:
    import subprocess
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'tiktoken'])
    import tiktoken

# === ???? ===
proxy_url = "http://127.0.0.1:8080"
http_client = httpx.Client(proxy=proxy_url, timeout=30)

# === OpenAI ?????? ===
client = OpenAI(
    api_key="",
    http_client=http_client
)


# === ??????? ===
def init_usage_log(path="log/usage.csv"):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not os.path.exists(path):
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["File", "PromptTokens", "CompletionTokens", "TotalTokens", "PriceUSD"])


def generate_response(prompt, task_name, file_basename, log_path="log/usage.csv"):
    try:
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You are a helpful assistant for analyzing traffic accidents."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.5,
            max_tokens=1024
        )

        content = response.choices[0].message.content
        usage = response.usage

        price_prompt = usage.prompt_tokens / 1000000 * 0.15
        price_completion = usage.completion_tokens / 1000000 * 0.60
        price_total = round(price_prompt + price_completion, 6)

        print(f"[{file_basename} - {task_name}] prompt={usage.prompt_tokens}, "
              f"completion={usage.completion_tokens}, total={usage.total_tokens}, "
              f"price=${price_total:.6f}")

        with open(log_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{file_basename} [{task_name}]",
                usage.prompt_tokens,
                usage.completion_tokens,
                usage.total_tokens,
                price_total
            ])

        return content
    except OpenAIError as e:
        return f"[ERROR] OpenAIError: {str(e)}"
    except Exception as e:
        return f"[ERROR] {str(e)}"


# === ???? report.json ===
def find_report_jsons(folder):
    report_files = []
    for root, _, files in os.walk(folder):
        for f in files:
            if f == "report.json":
                report_files.append(os.path.join(root, f))
    return report_files


# === ?????? ===
def generate_type(input_folder, output_folder="type"):
    os.makedirs(output_folder, exist_ok=True)
    instruction = (
        "You are a helpful assistant to classify the traffic accident based on the structured text. "
        "Types include:\n"
        "1. Single-Vehicle Accident\n"
        "2. Backover Collision\n"
        "3. Rear-End Collision\n"
        "4. Frontal Collision\n"
        "5. Front-to-Side Collision\n"
        "6. Non-Motorized Vehicle or Pedestrian Crash\n"
        "7. Other\n"
        "Return the Type Number and a brief reason."
    )
    report_files = find_report_jsons(input_folder)
    for filepath in tqdm(report_files, desc="Generating Type"):
        with open(filepath, 'r') as f:
            content = f.read()
        parent_name = os.path.basename(os.path.dirname(filepath))
        prompt = f"{instruction}\n\n{content}"
        output = generate_response(prompt, "Type", parent_name)
        with open(os.path.join(output_folder, parent_name + ".txt"), 'w') as out_f:
            out_f.write(output if output else "")


# === ???????? JSON? ===
def generate_responsibility(input_folder, output_folder="responsibility", summary_log="log/responsibility_summary.csv"):
    os.makedirs(output_folder, exist_ok=True)
    os.makedirs(os.path.dirname(summary_log), exist_ok=True)
    if not os.path.exists(summary_log):
        with open(summary_log, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["File", "ResponsibleParty", "Reason"])
    instruction = (
        "You are a helpful assistant for determining responsibility in a traffic accident. "
        "Based on the structured input, determine which vehicle (e.g., V1, V2, pedestrian) is mainly responsible.\n\n"
        "Respond ONLY in JSON format with two fields:\n"
        "- \"ResponsibleParty\": the ID of the responsible agent (e.g., \"V1\", \"V2\", \"pedestrian\", or \"unmentioned\")\n"
        "- \"Reason\": a short explanation\n"
        "Example:\n"
        "{\n  \"ResponsibleParty\": \"V2\",\n  \"Reason\": \"V2 changed lanes abruptly without signaling.\"\n}"
    )
    report_files = find_report_jsons(input_folder)
    for filepath in tqdm(report_files, desc="Generating Responsibility"):
        with open(filepath, 'r') as f:
            content = f.read()
        parent_name = os.path.basename(os.path.dirname(filepath))
        prompt = f"{instruction}\n\n{content}"
        output = generate_response(prompt, "Responsibility", parent_name)
        out_path = os.path.join(output_folder, parent_name + ".txt")
        with open(out_path, 'w') as out_f:
            out_f.write(output if output else "")
        # 解析 ResponsibleParty 并写入 summary_log
        import re
        responsible_party = "parse_error"
        reason = "parse_error"
        if output is not None:
            # 尝试多种方式提取 ResponsibleParty
            # 1. 尝试 JSON 解析
            try:
                result = json.loads(output)
                responsible_party = result.get("ResponsibleParty", "unknown")
                reason = result.get("Reason", "")
            except Exception:
                # 2. 使用正则表达式从文本中提取
                try:
                    # 匹配 "ResponsibleParty": "xxx" 格式
                    party_match = re.search(r'"ResponsibleParty"\s*:\s*"([^"]+)"', output, re.IGNORECASE)
                    if party_match:
                        responsible_party = party_match.group(1)
                    
                    # 匹配 "Reason": "xxx" 格式
                    reason_match = re.search(r'"Reason"\s*:\s*"([^"]+)"', output, re.IGNORECASE)
                    if reason_match:
                        reason = reason_match.group(1)
                    
                    # 如果正则也没找到，尝试更宽松的匹配
                    if responsible_party == "parse_error":
                        # 匹配任何包含 ResponsibleParty 的行
                        party_lines = [line for line in output.split('\n') if 'ResponsibleParty' in line]
                        if party_lines:
                            # 提取引号内的内容
                            party_match = re.search(r'"([^"]+)"', party_lines[0])
                            if party_match:
                                responsible_party = party_match.group(1)
                except Exception:
                    pass
        with open(summary_log, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([parent_name, responsible_party, reason])


# === ???? ===
def summarize_usage(path="log/usage.csv"):
    total_price = 0.0
    if not os.path.exists(path):
        return
    with open(path, 'r') as f:
        next(f)  # skip header
        for row in csv.reader(f):
            try:
                total_price += float(row[4])
            except:
                pass
    print(f"\n? Total estimated cost: ${total_price:.6f}\n")


# === 统计所有文件的token和价格 ===
def estimate_total_cost(input_folder, model="gpt-4o-mini", max_completion_tokens=1024):
    # 确保 tiktoken 使用代理设置
    import os
    import requests
    
    # 设置代理环境变量，确保 tiktoken 使用代理
    proxy_url = "http://127.0.0.1:8080"
    os.environ['HTTP_PROXY'] = proxy_url
    os.environ['HTTPS_PROXY'] = proxy_url
    
    # 为 requests 设置代理（tiktoken 内部使用 requests）
    session = requests.Session()
    session.proxies = {
        'http': proxy_url,
        'https': proxy_url
    }
    
    # 临时替换 requests.get 以使用代理
    original_get = requests.get
    def proxied_get(url, **kwargs):
        return session.get(url, **kwargs)
    requests.get = proxied_get
    
    try:
        instruction_resp = (
            "You are a helpful assistant for determining responsibility in a traffic accident. "
            "Based on the structured input, determine which vehicle (e.g., V1, V2, pedestrian) is mainly responsible.\n\n"
            "Respond ONLY in JSON format with two fields:\n"
            "- \"ResponsibleParty\": the ID of the responsible agent (e.g., \"V1\", \"V2\", \"pedestrian\", or \"unmentioned\")\n"
            "- \"Reason\": a short explanation\n"
            "Example:\n"
            "{\n  \"ResponsibleParty\": \"V2\",\n  \"Reason\": \"V2 changed lanes abruptly without signaling.\"\n}"
        )
        report_files = find_report_jsons(input_folder)
        enc = tiktoken.encoding_for_model(model)
        total_prompt_tokens = 0
        total_completion_tokens = 0
        for filepath in report_files:
            with open(filepath, 'r') as f:
                content = f.read()
            # responsibility prompt only (type analysis commented out)
            prompt_resp = f"{instruction_resp}\n\n{content}"
            tokens_resp = len(enc.encode(prompt_resp))
            total_prompt_tokens += tokens_resp
            total_completion_tokens += max_completion_tokens  # 只计算一次调用
        price_prompt = total_prompt_tokens / 1000000 * 0.15
        price_completion = total_completion_tokens / 1000000 * 0.60
        price_total = round(price_prompt + price_completion, 6)
        print(f"\n预计将处理 {len(report_files)} 个文件")
        print(f"总prompt tokens: {total_prompt_tokens}")
        print(f"总completion tokens(最大值): {total_completion_tokens}")
        print(f"预计总价: ${price_total:.6f}\n")
        user_input = input("按回车键继续，或输入 q 后回车退出：")
        if user_input.strip().lower() == 'q':
            print("已退出。")
            exit(0)
    finally:
        # 恢复原始的 requests.get
        requests.get = original_get


if __name__ == "__main__":
    input_folder = "/home/linshenghao/drivefuzz/TM-fuzzer/reports"
    estimate_total_cost(input_folder)
    init_usage_log("log/usage.csv")
    # generate_type(input_folder)  # 暂时注释掉 type 分析
    generate_responsibility(input_folder)
    summarize_usage("log/usage.csv")
