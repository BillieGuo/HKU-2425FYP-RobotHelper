import os
import cv2
import numpy as np
import torch
from groundingdino.util.inference import load_model, load_image, predict
from segment_anything import SamPredictor, sam_model_registry
from PIL import Image


class GDinoSAM:
    def __init__(self, groundingdino_weights_path, groundingdino_config_path, sam_weights_path, mask_storage_path, box_threshold, text_threshold):
        self.groundingdino_config_path = groundingdino_config_path
        self.groundingdino_weights_path = groundingdino_weights_path
        self.sam_weights_path = sam_weights_path
        self.mask_storage_path = mask_storage_path
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold

        self.model = load_model(self.groundingdino_config_path, self.groundingdino_weights_path)
        self.sam = sam_model_registry["vit_h"](checkpoint=self.sam_weights_path)
        self.predictor = SamPredictor(self.sam)

    def infer(self, rgb_image, depth_image, prompt):
        image_rgb = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        image_pil = Image.fromarray(image_rgb)
        image_height, image_width, _ = rgb_image.shape

        # Perform object detection
        boxes, logits, phrases = predict(self.model, image_pil, caption=prompt, box_threshold=self.box_threshold, text_threshold=self.text_threshold)
        absolute_boxes = self.convert_boxes(boxes, image_width, image_height)

        # Predict masks using SAM
        self.predictor.set_image(rgb_image)
        input_boxes = torch.tensor(absolute_boxes, device=self.predictor.device)
        transformed_boxes = self.predictor.transform.apply_boxes_torch(input_boxes, rgb_image.shape[:2])
        masks, _, _ = self.predictor.predict_torch(point_coords=None, point_labels=None, boxes=transformed_boxes, multimask_output=False)

        # Save the mask locally
        mask = masks[0][0].cpu().numpy().astype(bool)
        cv2.imwrite(self.mask_storage_path, mask.astype(np.uint8) * 255)

        return mask

    def convert_boxes(self, boxes, image_width, image_height):
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
        return np.array(absolute_boxes, dtype=float)
