'''
Author: 23Elapse userszy@163.com
Date: 2025-04-21 19:10:23
LastEditors: 23Elapse userszy@163.com
LastEditTime: 2025-04-21 21:24:31
FilePath: \Demod:\code\copy.py
Description: 

Copyright (c) 2025 by 23Elapse userszy@163.com, All Rights Reserved. 
'''
import os
from tqdm import tqdm
import sys
import io

# 修复标准流编码
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

# 设置 tqdm 进度条为 Unicode 模式
from tqdm import tqdm
os.environ["TQDM_ASCII"] = "False"  # 强制使用 Unicode 符号
def copy_folder_by_reading_with_progress(source_folder):
    # 自动生成目标文件夹名，加 "_backup"
    parent_dir = os.path.dirname(source_folder)
    folder_name = os.path.basename(source_folder)
    destination_folder = os.path.join(parent_dir, folder_name + "_backup")

    if not os.path.exists(destination_folder):
        os.makedirs(destination_folder)

    # 先收集所有要处理的文件列表
    all_files = []
    for root, dirs, files in os.walk(source_folder):
        for file in files:
            all_files.append(os.path.join(root, file))

    # tqdm包装一下，显示进度条
    for source_file_path in tqdm(all_files, desc="复制文件", unit="个"):
        relative_path = os.path.relpath(os.path.dirname(source_file_path), source_folder)
        target_dir = os.path.join(destination_folder, relative_path)
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)
        target_file_path = os.path.join(target_dir, os.path.basename(source_file_path))

        # 读取-写入
        with open(source_file_path, 'rb') as f_src:
            content = f_src.read()
        with open(target_file_path, 'wb') as f_dst:
            f_dst.write(content)

if __name__ == "__main__":
    src_folder = r"D:\code\Demo"  # 你的源文件夹
    copy_folder_by_reading_with_progress(src_folder)
