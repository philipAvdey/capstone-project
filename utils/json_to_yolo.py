import json
import os
import glob

LABELS_MAP = {
    "ball": 0,
    "soccer-ball": 0,
    "goal": 1,
}

json_dir = "./dataset/images/val"
output_dir = "./dataset/labels/val"

os.makedirs(output_dir, exist_ok=True)

json_files = glob.glob(os.path.join(json_dir, "*.json"))
print(f"Found {len(json_files)} JSON files")

converted = 0
skipped = 0

for json_path in json_files:
    with open(json_path, "r") as f:
        data = json.load(f)

    img_width = data["imageWidth"]
    img_height = data["imageHeight"]
    shapes = data["shapes"]

    lines = []
    for shape in shapes:
        label = shape["label"].lower()
        if label not in LABELS_MAP:
            print(f"  WARNING: unknown label '{label}' in {json_path}, skipping shape")
            continue

        class_id = LABELS_MAP[label]
        points = shape["points"]

        # Get bounding box from the two corner points
        x1 = points[0][0]
        y1 = points[0][1]
        x2 = points[1][0]
        y2 = points[1][1]

        # Normalize to 0-1
        x_center = ((x1 + x2) / 2) / img_width
        y_center = ((y1 + y2) / 2) / img_height
        width = abs(x2 - x1) / img_width
        height = abs(y2 - y1) / img_height

        lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}")

    # Save txt file to labels/train
    base_name = os.path.splitext(os.path.basename(json_path))[0]
    txt_path = os.path.join(output_dir, base_name + ".txt")

    with open(txt_path, "w") as f:
        f.write("\n".join(lines))

    converted += 1

print(f"\nDone! Converted {converted} files, skipped {skipped}")
print(f"Labels saved to: {output_dir}")