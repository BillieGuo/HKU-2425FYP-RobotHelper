from groundingdino.util.inference import load_model, load_image, predict
import cv2
import os
import numpy as np
from segment_anything import SamPredictor, sam_model_registry
import torch

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))

# Define paths
assets_dir = os.path.join(script_dir, "..", "assets")
input_image_path = os.path.join(assets_dir, "realsense_capture_720p", "rgb_image_20241204_181629.png")
output_dir = os.path.join(assets_dir, "GDinoSAMTest_output")
os.makedirs(output_dir, exist_ok=True)
output_image_path = os.path.join(output_dir, "annotated_masked_image.jpg")
config_path = os.path.join(script_dir, "..", "GroundingDINO", "groundingdino", "config", "GroundingDINO_SwinT_OGC.py")
weights_path = os.path.join(script_dir, "..", "pth", "groundingdino_swint_ogc.pth")
# Load the GroundingDINO model
model = load_model(config_path, weights_path)

# Define constants for detection
TEXT_PROMPT = "tape . mouse ."
# TEXT_PROMPT = "tape ."
# TEXT_PROMPT = "keyboard ."
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25

# Load the image for GDino
image_source, GDino_image = load_image(input_image_path)

# Perform object detection
boxes, logits, phrases = predict(model=model, image=GDino_image, caption=TEXT_PROMPT, box_threshold=BOX_THRESHOLD, text_threshold=TEXT_THRESHOLD)


# Convert bounding boxes from normalized to absolute pixel coordinates
def convert_boxes(boxes, image_width, image_height):
    absolute_boxes = []
    for box in boxes:
        x_center, y_center, box_width, box_height = box
        x_center_abs = x_center * image_width
        y_center_abs = y_center * image_height
        box_width_abs = box_width * image_width
        box_height_abs = box_height * image_height
        x1 = int(x_center_abs - box_width_abs / 2)
        y1 = int(y_center_abs - box_height_abs / 2)
        x2 = int(x_center_abs + box_width_abs / 2)
        y2 = int(y_center_abs + box_height_abs / 2)
        absolute_boxes.append([x1, y1, x2, y2])
    # Conver to NumPy array of type float
    absolute_boxes = np.array(absolute_boxes, dtype=float)
    return absolute_boxes


# Load the image for SAM
image = cv2.imread(input_image_path)
image_height, image_width, _ = image.shape  # Get image dimensions
absolute_boxes = convert_boxes(boxes, image_width, image_height)
# print(f"Image shape: {image.shape}")


# Initialize SAM
sam_checkpoint = os.path.join(script_dir, "..", "pth/sam_vit_h_4b8939.pth")
sam = sam_model_registry["vit_h"](checkpoint=sam_checkpoint)
predictor = SamPredictor(sam)
predictor.set_image(image)

# Convert boxes to torch tensor
input_boxes = torch.tensor(absolute_boxes, device=predictor.device)

# print(f"From json {boxes}")
# print(f"Converted to absolute {absolute_boxes}")
# print(f"Converted to tensor {input_boxes}")

# Transform the boxes to the input frame
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
print(f"Number of masks: {masks.shape[0]}")

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

# print(f"phrases: {phrases}, scores: {scores}")
# Draw bounding boxes and labels on the blended image
for idx, box in enumerate(absolute_boxes):
    x1, y1, x2, y2 = map(int, box)  # Convert coordinates to integers
    label = phrases[idx] if idx < len(phrases) else "Object"
    score = scores[idx].item() if idx < len(scores) else 0.0  # Convert score to float
    # Draw the bounding box
    cv2.rectangle(blended_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
    # Draw the label and score

    cv2.putText(blended_image, f"{label}: {score:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

# Display the image
# cv2.imshow("Annotated Masks", blended_image)
# print("Press any key to close the visualization.")
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# Save the annotated masked image
cv2.imwrite(output_image_path, blended_image)
print(f"Annotated masked image saved to {output_image_path}")
