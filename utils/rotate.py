#!/usr/bin/env python3
"""
Rotates all images in training-photos/goal 180 degrees in-place.
Supports: .jpg, .jpeg, .png, .webp, .bmp, .tiff, .tif
"""
 
from pathlib import Path
from PIL import Image
 
FOLDER = Path("photos-for-processing")
SUPPORTED = {".jpg", ".jpeg", ".png", ".webp", ".bmp", ".tiff", ".tif"}
 
 
def rotate_images(folder: Path) -> None:
    if not folder.exists():
        print(f"Error: folder '{folder}' not found.")
        return
 
    images = [p for p in folder.iterdir() if p.suffix.lower() in SUPPORTED]
 
    if not images:
        print(f"No supported images found in '{folder}'.")
        return
 
    print(f"Found {len(images)} image(s) in '{folder}'. Rotating 180°...\n")
 
    success, failed = 0, 0
 
    for img_path in sorted(images):
        try:
            with Image.open(img_path) as img:
                # Preserve EXIF data if present
                exif = img.info.get("exif", b"")
                rotated = img.rotate(180)
 
                save_kwargs = {}
                if exif:
                    save_kwargs["exif"] = exif
 
                rotated.save(img_path, **save_kwargs)
 
            print(f"  ✓ {img_path.name}")
            success += 1
 
        except Exception as e:
            print(f"  ✗ {img_path.name} — {e}")
            failed += 1
 
    print(f"\nDone. {success} rotated, {failed} failed.")
 
 
if __name__ == "__main__":
    rotate_images(FOLDER)