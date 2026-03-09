import os
from pathlib import Path
from PIL import Image  # pip install Pillow

# --- Config ---
INPUT_DIR = Path(__file__).parent / "pre-processed-photos"
OUTPUT_DIR = Path(__file__).parent / "post-processed-photos"

def process_image(img: Image.Image) -> Image.Image:
    """
    Apply downgrade transformations to a single image.
    TODO: decide and implement the actual degradation logic here.
    
    Ideas:
      - Reduce resolution (resize down then back up)
      - Lower JPEG quality on save (quality=10–40)
      - Reduce color depth / posterize
      - Add noise or grain
      - Convert to a limited palette (e.g. 256 colors)
      - Blur or sharpen artificially
      - Simulate old photo formats (sepia, VHS, 8-bit, etc.)
    """
    return img  # placeholder — return img unchanged for now


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
            # TODO: tweak save params based on chosen degradation method
            # e.g. processed.save(output_path, quality=20) for JPEG compression
            processed.save(output_path)

    print(f"Done. Output saved to: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()