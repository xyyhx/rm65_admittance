import re
import argparse
import sys
import os

def extract_regs_from_log(filepath, pattern_str):
    """
    使用正则表达式从 Log 文件中提取寄存器信息。
    返回字典 { reg_name: value }
    """
    try:
        # 编译正则表达式
        pattern = re.compile(pattern_str)
    except re.error as e:
        print(f"❌ 正则表达式编译失败: {e}")
        sys.exit(1)

    regs = {}
    matched_count = 0
    
    try:
        # 逐行读取，支持大文件
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                match = pattern.search(line)
                if match:
                    name = match.group(1).strip()
                    value = match.group(2).strip()
                    # 注意：如果同一寄存器在 Log 中出现多次，这里默认取最后一次的值
                    # 这通常符合验证逻辑（关注最终状态）
                    regs[name] = value
                    matched_count += 1
    except Exception as e:
        print(f"❌ 读取文件失败: {e}")
        sys.exit(1)
        
    print(f"ℹ️ [{os.path.basename(filepath)}] 提取到 {len(regs)} 个唯一寄存器 (共匹配 {matched_count} 行)")
    return regs

def normalize_value(val):
    """将字符串值转为数值以便比较，支持 0x 十六进制"""
    if not isinstance(val, str):
        return val
    try:
        v_lower = val.lower()
        if v_lower.startswith('0x'):
            return int(val, 16)
        elif v_lower.startswith('0b'):
            return int(val, 2)
        elif '.' in val:
            return float(val)
        else:
            return int(val)
    except ValueError:
        return val  # 无法转换则保持原字符串

def compare_logs(file_a, file_b, pattern, tolerance=0):
    print(f"🔍 开始解析文件...")
    regs_a = extract_regs_from_log(file_a, pattern)
    regs_b = extract_regs_from_log(file_b, pattern)

    keys_a = set(regs_a.keys())
    keys_b = set(regs_b.keys())

    # 集合运算
    common_keys = keys_a & keys_b
    only_in_a = keys_a - keys_b
    only_in_b = keys_b - keys_a

    print("\n" + "="*60)
    print("📊 Log 寄存器一致性检查报告")
    print("="*60)

    has_error = False

    # 1. 检查 A 独有的寄存器（根据描述，A 包含 B，所以 A 有额外信息是正常的）
    if only_in_a:
        print(f"\nℹ️ 仅在 A 中存在的寄存器 (共 {len(only_in_a)} 个，属于 A 的额外信息):")
        # 仅打印前 5 个作为示例
        for r in sorted(list(only_in_a)[:5]):
            print(f"   - {r}")
        if len(only_in_a) > 5: print("   ...")

    # 2. 检查 B 独有的寄存器（如果 A 包含 B，理论上 B 不应该有 A 没有的寄存器）
    if only_in_b:
        has_error = True
        print(f"\n⚠️ 仅在 B 中存在的寄存器 (共 {len(only_in_b)} 个，异常):")
        for r in sorted(list(only_in_b)):
            print(f"   - {r}")

    # 3. 比较共有的寄存器
    diffs = []
    for reg in sorted(list(common_keys)):
        val_a = regs_a[reg]
        val_b = regs_b[reg]

        # 尝试数值转换比较
        num_a = normalize_value(val_a)
        num_b = normalize_value(val_b)

        is_diff = False
        diff_detail = ""

        if isinstance(num_a, (int, float)) and isinstance(num_b, (int, float)):
            if abs(num_a - num_b) > tolerance:
                is_diff = True
                diff_detail = f"{num_a} vs {num_b} (差值: {num_a - num_b})"
        else:
            # 字符串比较
            if str(num_a) != str(num_b):
                is_diff = True
                diff_detail = f"'{val_a}' vs '{val_b}'"

        if is_diff:
            diffs.append((reg, val_a, val_b, diff_detail))

    if diffs:
        has_error = True
        print(f"\n❌ 发现 {len(diffs)} 个共有寄存器数值不一致:")
        print("-" * 80)
        print(f"{'寄存器名称':<30} | {'A 文件值':<15} | {'B 文件值':<15} | {'差异详情'}")
        print("-" * 80)
        for reg, va, vb, detail in diffs:
            print(f"{reg:<30} | {str(va):<15} | {str(vb):<15} | {detail}")
        print("-" * 80)
    else:
        print(f"\n✅ 共有的 {len(common_keys)} 个寄存器数值完全一致。")

    print("="*60)
    if has_error:
        print("🔴 结论: 存在差异或异常")
    else:
        print("🟢 结论: 匹配成功，数据一致")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='比较两个 Log 文件中的寄存器信息')
    parser.add_argument('file_a', help='文件 A (包含更多信息的参考文件)')
    parser.add_argument('file_b', help='文件 B (被包含的目标文件)')
    parser.add_argument('--pattern', default=r'([a-zA-Z0-9_]+)\s*[=:]\s*(0x[0-9a-fA-F]+|\d+)', 
                        help='正则表达式用于提取寄存器，默认匹配 "NAME = VALUE" 或 "NAME: VALUE"')
    parser.add_argument('--tolerance', type=float, default=0, help='数值比较容差')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.file_a) or not os.path.exists(args.file_b):
        print("❌ 找不到指定的文件")
        sys.exit(1)
        
    compare_logs(args.file_a, args.file_b, args.pattern, args.tolerance)
