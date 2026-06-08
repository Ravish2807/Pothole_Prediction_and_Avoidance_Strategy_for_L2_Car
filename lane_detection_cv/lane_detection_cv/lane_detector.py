import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math

class LaneDetector:
    def __init__(self,
                 kernel_size=5,
                 low_threshold=180,
                 high_threshold=240,
                 rho=1,
                 theta=np.pi/180,
                 hough_threshold=20,
                 min_line_len=20,
                 max_line_gap=180,
                 weighted_alpha=0.8,
                 weighted_beta=1.,
                 weighted_gamma=0.):

        self.kernel_size = kernel_size
        self.low_threshold = low_threshold
        self.high_threshold = high_threshold
        self.rho = rho
        self.theta = theta
        self.hough_threshold = hough_threshold
        self.min_line_len = min_line_len
        self.max_line_gap = max_line_gap
        self.weighted_alpha = weighted_alpha
        self.weighted_beta = weighted_beta
        self.weighted_gamma = weighted_gamma

    def grayscale(self, img):
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    def canny(self, img):
        return cv2.Canny(img, self.low_threshold, self.high_threshold)

    def gaussian_blur(self, img):
        return cv2.GaussianBlur(img, (self.kernel_size, self.kernel_size), 0)

    def region_of_interest(self, img, vertices):
        mask = np.zeros_like(img)
        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def draw_lines(self, img, lines, color=[255, 0, 0], thickness=10):
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    def slope_lines(self, image, lines):
        img = image.copy()
        poly_vertices = []
        order = [0,1,3,2]

        left_lines = []
        right_lines = []
        for line in lines:
            for x1,y1,x2,y2 in line:
                if x1 == x2:
                    pass
                else:
                    m = (y2 - y1) / (x2 - x1)
                    c = y1 - m * x1

                    if m < 0:
                        left_lines.append((m,c))
                    elif m >= 0:
                        right_lines.append((m,c))

        # Handle cases where no lines are detected or only one type of line
        if not left_lines and not right_lines:
            return img # Return original image if no lines detected

        if left_lines:
            left_line = np.mean(left_lines, axis=0)
        else:
            left_line = (None, None) # Indicate no left line

        if right_lines:
            right_line = np.mean(right_lines, axis=0)
        else:
            right_line = (None, None) # Indicate no right line

        for slope, intercept in [left_line, right_line]:
            if slope is None: # Skip if no line was detected for this side
                continue

            rows, cols = image.shape[:2]
            y1= int(rows)
            y2= int(rows*0.6)

            # Avoid division by zero for vertical lines, though they should be handled by the 'x1 == x2' check
            if slope == 0: # This case should ideally not happen for lane lines after filtering
                continue

            x1=int((y1-intercept)/slope)
            x2=int((y2-intercept)/slope)
            poly_vertices.append((x1, y1))
            poly_vertices.append((x2, y2))
            self.draw_lines(img, np.array([[[x1,y1,x2,y2]]]))

        if len(poly_vertices) == 4: # Ensure we have all 4 vertices before drawing polygon
            poly_vertices = [poly_vertices[i] for i in order]
            cv2.fillPoly(img, pts = np.array([poly_vertices],'int32'), color = (0,255,0))

        return cv2.addWeighted(image,0.7,img,0.4,0.)

    def hough_lines(self, img):
        lines = cv2.HoughLinesP(img, self.rho, self.theta, self.hough_threshold, np.array([]),
                                minLineLength=self.min_line_len, maxLineGap=self.max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

        if lines is not None:
            line_img = self.slope_lines(line_img, lines)
        return line_img

    def weighted_img(self, img, initial_img):
        lines_edges = cv2.addWeighted(initial_img, self.weighted_alpha, img, self.weighted_beta, self.weighted_gamma)
        return lines_edges

    def get_vertices(self, image):
        rows, cols = image.shape[:2]
        bottom_left  = [cols*0.15, rows]
        top_left     = [cols*0.45, rows*0.6]
        bottom_right = [cols*0.95, rows]
        top_right    = [cols*0.55, rows*0.6]
        ver = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        return ver

    def detect_lanes(self, image):
        gray_img = self.grayscale(image)
        smoothed_img = self.gaussian_blur(img = gray_img)
        canny_img = self.canny(img = smoothed_img)
        masked_img = self.region_of_interest(img = canny_img, vertices = self.get_vertices(image))
        houghed_lines = self.hough_lines(img = masked_img)
        output = self.weighted_img(img = houghed_lines, initial_img = image)
        return output
