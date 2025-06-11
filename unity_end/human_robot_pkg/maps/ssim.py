from skimage.metrics import structural_similarity as ssim
from skimage.io import imread
from skimage.transform import resize
import sys

def compute_ssim(image1_path, image2_path):
    # Load both images as grayscale
    img1 = imread(image1_path, as_gray=True)
    img2 = imread(image2_path, as_gray=True)

    # Normalize img1 if it's not float
    if img1.dtype != 'float64':
        img1 = img1 / 255.0

    # Resize img2 to match img1 if necessary
    if img1.shape != img2.shape:
        print(f"Resizing {image2_path} from {img2.shape} to match {img1.shape}")
        img2 = resize(img2, img1.shape, anti_aliasing=True)

    # Compute SSIM (with data_range=1.0 for float images)
    score, diff = ssim(img1, img2, data_range=1.0, full=True)
    print(f"SSIM between images: {score:.4f}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python ssim_pgm.py <image1.pgm> <image2.pgm>")
    else:
        compute_ssim(sys.argv[1], sys.argv[2])
