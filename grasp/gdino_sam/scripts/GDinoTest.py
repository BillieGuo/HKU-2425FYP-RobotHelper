from groundingdino.util.inference import load_model, load_image, predict, annotate
import cv2
import os
import json  # Added import

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))

# Define paths
# Ensure the paths are correct
assets_dir = os.path.join(script_dir, "..", "assets")
input_image_path = os.path.join(assets_dir, "realsense_capture_720p", "rgb_image_20241204_181629.png")
output_dir = os.path.join(assets_dir, "GDinoTest_output")
os.makedirs(output_dir, exist_ok=True)
output_image_path = os.path.join(output_dir, "annotated_image.jpg")
config_path = os.path.join(script_dir, "..", "GroundingDINO", "groundingdino", "config", "GroundingDINO_SwinT_OGC.py")
weights_path = os.path.join(script_dir, "..", "pth", "groundingdino_swint_ogc.pth")

# Load the model
model = load_model(config_path, weights_path)

# Define constants
TEXT_PROMPT = "tape . mouse ."
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25

# Load the image
image_source, image = load_image(input_image_path)

# Predict
boxes, logits, phrases = predict(model=model, image=image, caption=TEXT_PROMPT, box_threshold=BOX_THRESHOLD, text_threshold=TEXT_THRESHOLD)

# Annotate the image
annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)

# Save the annotated image
cv2.imwrite(output_image_path, annotated_frame)

# Save the detected bounding boxes to a JSON file
output_data = {"boxes": boxes.tolist(), "labels": phrases}

output_json_path = os.path.join(output_dir, "boxes.json")
with open(output_json_path, "w") as f:
    json.dump(output_data, f)

print(f"Bounding boxes saved to {output_json_path}")
print(f"Annotated image saved to {output_image_path}")
