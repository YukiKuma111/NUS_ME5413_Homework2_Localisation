from PIL import Image

# 打开 PGM 文件
pgm_file = "/map/dir.pgm"
img = Image.open(pgm_file)

# 将其保存为 PNG 文件
png_file = "/map/dir.png"
img.save(png_file)

print(f"已成功将 {pgm_file} 转换为 {png_file}")

