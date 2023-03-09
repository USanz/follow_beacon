import cv2, numpy as np

def calc_centroid(points):
    centroid_x = sum(points[:, 0]) / len(points[:, 0])
    centroid_y = sum(points[:, 1]) / len(points[:, 1])
    return [centroid_x, centroid_y]

def calc_diag_size(points):
    if len(points) != 4: # if it's not squared
        return None
    #euclidean distance:
    diag02 = np.linalg.norm(points[0] - points[2])
    diag13 = np.linalg.norm(points[1] - points[3])
    return (diag02 + diag13) / 2.0
    
def calc_centroid_rel(centroid_abs, img_size):
    rel_centroid = [((centroid_abs[0] / img_size[0]) - 0.5) * 2.0,
                    ((centroid_abs[1] / img_size[1]) - 0.5) * 2.0]
    return rel_centroid

class QRCode:
    def __init__(self, points, data, img_size):
        self.points = points
        self.data = data
        self.centroid = calc_centroid(points)
        self.diag_avg_size = calc_diag_size(points)
        self.centroid_rel = calc_centroid_rel(self.centroid, img_size)

    def __gt__(self, other):
        return self.diag_avg_size > other.diag_avg_size

    def __lt__(self, other):
        return self.diag_avg_size < other.diag_avg_size

    def get_centroid(self): return self.centroid
    def get_centroid_rel(self): return self.centroid_rel
    def get_diag_avg_size(self): return self.diag_avg_size

    def draw_bbox(self, img, bbox_color, text_color):
        new_img = cv2.polylines(img, [self.points.astype(int)], True, bbox_color, 3)
        return cv2.putText(new_img, self.data, self.points[0].astype(int),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1, cv2.LINE_AA)
