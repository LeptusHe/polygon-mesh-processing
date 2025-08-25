import os
from PIL import Image
import numpy as np
import subprocess

def generate_constant_color_texture(length, width, rgb_values, file_path):
    """
    生成一张RGB PNG纹理图片
    
    参数:
        length: 纹理长度（像素）
        width: 纹理宽度（像素）
        rgb_values: RGB三元组，每个值在0-1范围内，例如(0.5, 0.2, 0.8)
        file_path: 文件存储路径
    
    返回:
        生成的PNG文件路径
    """
    # 验证参数
    if not isinstance(rgb_values, (tuple, list)) or len(rgb_values) != 3:
        raise ValueError("rgb_values必须是包含3个元素的元组或列表")
    
    for val in rgb_values:
        if not (0.0 <= val <= 1.0):
            raise ValueError("RGB值必须在0-1范围内")
    
    if length <= 0 or width <= 0:
        raise ValueError("长度和宽度必须为正整数")
    
    # 将浮点RGB值转换为0-255范围的整数
    r, g, b = [int(val * 255) for val in rgb_values]
    
    # 创建RGB图像数组
    image_array = np.full((width, length, 3), [r, g, b], dtype=np.uint8)
    
    # 创建PIL图像
    image = Image.fromarray(image_array, mode='RGB')
    
    # 确保目录存在
    file_path = os.path.abspath(file_path)
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    
    # 保存为PNG文件
    image.save(file_path, 'PNG')
    
    return file_path


exe_path = "../../../../build/release/bin/12-least-squares-vertex-baking.exe"
argument_json_path = "./argument.json"
work_dir = "../../../../"

# 使用示例
if __name__ == "__main__":
    # 测试函数
    image_path = "constant_color_texture.png"
    generate_constant_color_texture(256, 256, (1.0, 0.5, 0.0), image_path)
    print(f"succeed to generate constant color texture: {image_path}")

    exe_path = os.path.abspath(exe_path)
    argument_json_path = os.path.abspath(argument_json_path)
    
    # 设置工作目录为当前脚本所在目录
    work_dir = os.path.abspath(work_dir)
    
    # 使用subprocess.run替代os.system，可以设置工作目录
    command = [exe_path, argument_json_path]
    print(f"execute cmd: {' '.join(command)}")
    print(f"working directory: {work_dir}")
    
    try:
        result = subprocess.run(command, cwd=work_dir, check=True, capture_output=True, text=True)
        print("Command executed successfully")
        if result.stdout:
            print("Output:", result.stdout)
    except subprocess.CalledProcessError as e:
        print(f"Command failed with return code {e.returncode}")
        if e.stderr:
            print("Error:", e.stderr)
    except Exception as e:
        print(f"Error executing command: {e}")