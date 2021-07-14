import sys
from PIL import Image

if len(sys.argv) >= 3:
    input_filename = sys.argv[1]
    output_filename = sys.argv[2]
    image = Image.open(input_filename).convert("L")
    width = image.width
    height = image.height
    with open(output_filename, "w") as file:
        file.write('%d %d\n' % (height, width))
        pixels = image.load()
        for i in range(height):
            for j in range(width):
                file.write('%d ' % pixels[j, i])
            file.write('\n')
