from PIL import Image

# Open the image file
image = Image.open("visualization_result.png")

# Flip the image upside down
flipped_image = image.transpose(Image.FLIP_TOP_BOTTOM)

# Save the flipped image
flipped_image.save("flipped_visualization_result.png")