import cv2
import numpy as np

# Load the image
image = cv2.imread('images\WIN_20240309_15_07_36_Pro.jpg')

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur to smooth the image
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Apply Canny edge detection
edges = cv2.Canny(blurred, 50, 150)

# Detect lines using Hough transform
lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=100, maxLineGap=10)

# Find intersections of the detected lines
print("hrere")
intersections = []
if lines is not None:
    for i in range(len(lines)):
        for j in range(i+1, len(lines)):
            line1 = lines[i][0]
            line2 = lines[j][0]

            # Compute intersection point
            rho1, theta1 = line1[0], line1[1]
            rho2, theta2 = line2[0], line2[1]
            A = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
            b = np.array([rho1, rho2])
            intersection = np.linalg.solve(A, b)
            intersection = tuple(np.round(intersection).astype(int))

            intersections.append(intersection)

# Draw the detected intersections on the original image (for visualization)
for intersection in intersections:
    cv2.circle(image, intersection, 5, (0, 0, 255), -1)

# Display the result
cv2.imshow('Detected Corners', image)
cv2.waitKey(0)
cv2.destroyAllWindows()