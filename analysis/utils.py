import cv2

def get_gradient_values(gradient_img, multiplier=1):
    """ Given the image gradient returns gradient_alpha_rgb_mul and one_minus_gradient_alpha.
    These pre-calculated numbers are used to apply the gradient on the camera image

    Arguments:
        gradient_img (Image): Gradient image that has to applied on the camera image
        multiplier (float): This decides what percentage of gradient images alpha has to be applied.
                            This is useful in fading feature.

    Returns:
        (tuple): gradient_alpha_rgb_mul (Numpy.Array) gradient_img * gradient_alpha value
                 one_minus_gradient_alpha (Numpy.Array) (1 - gradient_alpha)
    """
    (height, width, _) = gradient_img.shape
    gradient_alpha = (gradient_img[:, :, 3] / 255.0 * multiplier).reshape(height, width, 1)

    gradient_alpha_rgb_mul = gradient_img * gradient_alpha
    one_minus_gradient_alpha = (1 - gradient_alpha).reshape(height, width)
    return gradient_alpha_rgb_mul, one_minus_gradient_alpha

    
def apply_gradient(main_image, gradient_alpha_rgb_mul, one_minus_gradient_alpha):
    """ The gradient on the image is overlayed so that text looks visible and clear.
    This leaves a good effect on the image.
    The older code took 6.348s for 1000 runs

    Numpy broadcasting is slower than normal python
    major_cv_image_1[:, :, :4] = (gradient_alpha_rgb_mul + (major_cv_image_1 * one_minus_gradient_alpha))[:, :, :4]
    Timeit 1000 runs - 6.523s

    The current code takes - 5.131s for 1000 runs

    Args:
        main_image (Image): The main image where gradient has to be applied
        gradient_alpha_rgb_mul (Numpy.Array): gradient_img * gradient_alpha value
        one_minus_gradient_alpha (Numpy.Array): (1 - gradient_alpha)
    Returns:
        Image: Gradient applied image
    """
    for channel in range(0, 4):
        main_image[:, :, channel] = gradient_alpha_rgb_mul[:, :, channel] + \
            (main_image[:, :, channel] * one_minus_gradient_alpha)
    return main_image

def load_background_image(file: str, WIDTH, HEIGHT):
    """
    Load and process a background image.
    This function performs the following steps:
    1. Loads the background image from the specified file path.
    2. Resizes the image to have the specified height (HEIGHT) while maintaining the aspect ratio.
    3. Crops the width of the resized image to fit within the specified width (WIDTH).
    4. Converts the image to RGBA format.
    Args:
        file (str): The file path to the background image.
        WIDTH (int): The desired width of the processed image.
        HEIGHT (int): The desired height of the processed image.
    Returns:
        numpy.ndarray: The processed background image in RGBA format.
    """
    
    # Load the background image
    background = cv2.imread(file)
    if background is None:
        raise FileNotFoundError(f"The file {file} does not exist or is not a valid image.")

    # Resize the background image to have 720 rows while maintaining the aspect ratio
    aspect_ratio = background.shape[1] / background.shape[0]
    new_width = int(HEIGHT * aspect_ratio)
    resized_background = cv2.resize(background, (new_width, HEIGHT))

    # Crop the width to fit within the desired WIDTH
    if new_width > WIDTH:
        resized_background = resized_background[:, :WIDTH]

    return cv2.cvtColor(resized_background, cv2.COLOR_BGR2RGBA)