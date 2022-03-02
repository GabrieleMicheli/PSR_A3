#!/usr/bin/python3

import cv2


def callbackPodium(image):
    try:
        img = cv2.imread(image, 1)
        cv2.namedWindow('Winning Team', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Winning Team', 1200, 675)

        first_place = cv2.imread('green.jpg', 1)
        first_place = cv2.resize(first_place, (160, 90), interpolation=cv2.INTER_AREA)

        second_place = cv2.imread('blue.jpg', 1)
        second_place = cv2.resize(second_place, (100, 70), interpolation=cv2.INTER_AREA)

        third_place = cv2.imread('red.jpg', 1)
        third_place = cv2.resize(third_place, (100, 70), interpolation=cv2.INTER_AREA)

        img[100:190, 180:340] = first_place
        img[130:200, 50:150] = second_place
        img[140:210, 375:475] = third_place
        cv2.putText(img, 'Team Hunting Olympics - Aveiro 2022', (80, 310),
                    fontFace=cv2.FONT_ITALIC, fontScale=0.5, color=(100, 50, 200), thickness=2)
        return img

    except Exception as e:
        print(str(e))
