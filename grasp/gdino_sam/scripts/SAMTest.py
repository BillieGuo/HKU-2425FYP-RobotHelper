from segment_anything import SamPredictor, sam_model_registry
import json  # Added import
import cv2
import os
import numpy as np  # Added import for NumPy
import torch  # Added import for PyTorch

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))

# Define paths
assets_dir = os.path.join(script_dir, "..", "assets")
input_image_path = os.path.join(assets_dir, "realsense_capture_720p", "rgb_image_20241204_181629.png")
boxes_json_path = os.path.join(assets_dir, "GDinoTest_output", "boxes.json")  # Path to boxes.json
output_mask_path = os.path.join(assets_dir, "SAMTest_output", "masks.png")
os.makedirs(os.path.dirname(output_mask_path), exist_ok=True)

# Load the image
image = cv2.imread(input_image_path)
height, width, _ = image.shape  # Get image dimensions

print(f"Image shape: {image.shape}")

# Load bounding boxes
with open(boxes_json_path, "r") as f:
    data = json.load(f)
boxes = data["boxes"]

# Convert normalized boxes to absolute [x1, y1, x2, y2] format
absolute_boxes = []
for box in boxes:
    x_center, y_center, box_width, box_height = box
    x_center_abs = x_center * width
    y_center_abs = y_center * height
    box_width_abs = box_width * width
    box_height_abs = box_height * height
    x1 = int(x_center_abs - box_width_abs / 2)
    y1 = int(y_center_abs - box_height_abs / 2)
    x2 = int(x_center_abs + box_width_abs / 2)
    y2 = int(y_center_abs + box_height_abs / 2)
    absolute_boxes.append([x1, y1, x2, y2])

# Convert to NumPy array of type float
absolute_boxes = np.array(absolute_boxes, dtype=float)

# Initialize SAM
pth_path = os.path.join(script_dir, "..", "pth/sam_vit_h_4b8939.pth")
sam = sam_model_registry["vit_h"](checkpoint=pth_path)
predictor = SamPredictor(sam)
predictor.set_image(image)

# Prepare prompt boxes
prompt_boxes = absolute_boxes  # Now in [x1, y1, x2, y2] format and NumPy array

# Convert prompt_boxes to torch tensor
input_boxes = torch.tensor(prompt_boxes, device=predictor.device)

print(f"From json {boxes}")
print(f"Converted to absolute {absolute_boxes}")
print(f"Converted to tensor {input_boxes}")

# Transform the boxes to the input frame
print(f"Image shape: {image.shape}")
transformed_boxes = predictor.transform.apply_boxes_torch(input_boxes, image.shape[:2])

# Predict masks using predict_torch
masks, scores, logits = predictor.predict_torch(
    point_coords=None,
    point_labels=None,
    boxes=transformed_boxes,
    multimask_output=False,
)

print(masks.shape)
print(masks)

# Apply masks to the image
# Initialize a color mask layer
color_mask = np.zeros_like(image, dtype=np.uint8)

# Define colors for each mask
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]  # Blue  # Green  # Red  # Cyan  # Magenta  # Yellow

for idx, mask in enumerate(masks):
    mask = mask[0].cpu().numpy().astype(bool)  # Convert tensor to boolean numpy array
    color = colors[idx % len(colors)]  # Assign color based on mask index
    color_mask[mask] = color  # Apply color to the mask area

# Blend the color mask with the original image
alpha = 0.5  # Transparency factor
blended_image = cv2.addWeighted(image, 1.0, color_mask, alpha, 0)

# Display the image
cv2.imshow("Masks", blended_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save the masked image
cv2.imwrite(output_mask_path, blended_image)
print(f"Masks applied and saved to {output_mask_path}")
