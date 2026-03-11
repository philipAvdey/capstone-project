import io
import os
import numpy as np
from pathlib import Path
from PIL import Image, ImageFilter, ImageEnhance, ImageOps, ImageChops

# --- Config ---
INPUT_DIR = Path(__file__).parent / "photos-for-processing/pre-processed-photos"
OUTPUT_DIR = Path(__file__).parent / "photos-for-processing/post-processed-photos"

TARGET_WIDTH = 640
TARGET_HEIGHT = 480

def process_image(img: Image.Image) -> Image.Image:
# 1. Convert to RGB
    img = img.convert("RGB")

    # 2. Resize to 640x480
    img = ImageOps.fit(img, (TARGET_WIDTH, TARGET_HEIGHT), method=Image.Resampling.LANCZOS)

    # # 3. Purple/blue shadow cast (characteristic of this sensor)
    # r, g, b = img.split()
    # r = r.point([int(i * 1.12) for i in range(256)])
    # g = g.point([int(i * 0.85) for i in range(256)])
    # b = b.point([min(255, int(i * 1.0)) for i in range(256)])
    # img = Image.merge("RGB", (r, g, b))

    # 4. Crush shadows / blow highlights (poor dynamic range)
    arr = np.array(img, dtype=np.float32)
    arr = np.power(arr / 255.0, 1.4) * 255.0
    arr = np.where(arr > 200, arr * 1.4, arr)
    arr = np.clip(arr, 0, 255).astype(np.uint8)
    img = Image.fromarray(arr)

    # 5. Slight contrast boost
    img = ImageEnhance.Contrast(img).enhance(1.3)

    # 6. Muddy the colours slightly + exposure
    img = ImageEnhance.Color(img).enhance(0.80)

    img = ImageEnhance.Brightness(img).enhance(1.5)

    # 7. Soft lens blur — just enough to lose sharpness + chromatic aberation
    img = img.filter(ImageFilter.GaussianBlur(radius=1.0))
    r, g, b = img.split()
    r = ImageChops.offset(r, 3, 1)   # red shifts right/down
    b = ImageChops.offset(b, -3, -1) # blue shifts left/up
    img = Image.merge("RGB", (r, g, b))

    # 8. JPEG compression round-trip — the main degradation driver
    #    Two passes at different qualities creates layered blocking artifacts
    for quality in (45, 55):
        buffer = io.BytesIO()
        img.save(buffer, format="JPEG", quality=quality)
        buffer.seek(0)
        img = Image.open(buffer).copy()

    # 9. Bare minimum noise — just to break up flat areas slightly
    arr = np.array(img, dtype=np.int16)
    noise = np.random.randint(-6, 6, arr.shape, dtype=np.int16)
    arr = np.clip(arr + noise, 0, 255).astype(np.uint8)
    img = Image.fromarray(arr)

    return img



def main():
    # Validate input dir
    if not INPUT_DIR.exists():
        raise FileNotFoundError(f"Input folder not found: {INPUT_DIR}")

    # Create output dir if needed
    OUTPUT_DIR.mkdir(exist_ok=True)

    # Gather supported image files
    extensions = {".jpg", ".jpeg", ".png", ".webp", ".bmp", ".tiff"}
    image_files = [f for f in INPUT_DIR.iterdir() if f.suffix.lower() in extensions]

    if not image_files:
        print("No images found in input folder.")
        return

    print(f"Processing {len(image_files)} image(s)...")

    for image_path in image_files:
        print(f"  → {image_path.name}")

        with Image.open(image_path) as img:
            processed = process_image(img)

            output_path = OUTPUT_DIR / image_path.name
            processed.save(output_path)

    print(f"Done. Output saved to: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()