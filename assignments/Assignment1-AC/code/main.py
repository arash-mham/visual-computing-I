import cv2
import numpy as np

from external_energy import external_energy
from internal_energy_matrix import get_matrix

def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        #save point
        xs.append(x)
        ys.append(y)

        #display point
        cv2.circle(img, (x, y), 3, 128, -1)
        cv2.imshow('image', img)


if __name__ == '__main__':
    #point initialization
    img_path = 'images/star.png'
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    xs = []
    ys = []
    cv2.imshow('image', img)

    cv2.setMouseCallback('image', click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #selected points are in xs and ys

    #interpolate
    #implement part 1: interpolate between the  selected points
    raise NotImplementedError

    alpha = 0.5
    beta = 0.5
    gamma = 0.5
    kappa = 1.
    num_points = len(xs)

    #get matrix
    M = get_matrix(alpha, beta, gamma, num_points)

    #get external energy
    w_line = 0.5
    w_edge = 0.5
    w_term = 0.5
    E = external_energy(img, w_line, w_edge, w_term)

    while True:
        #optimization loop
        raise NotImplementedError
